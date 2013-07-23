/*
Software License Agreement (BSD License)
 
Copyright (c) 2013, Southwest Research Institute
All rights reserved.
 
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
 
   * Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.
   * Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in the
     documentation and/or other materials provided with the distribution.
   * Neither the name of the Southwest Research Institute, nor the names
     of its contributors may be used to endorse or promote products derived
     from this software without specific prior written permission.
 
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
*/
#include<object_tracking/tracker.h>

ParticleTrack::ParticleTrack()
{
  Q_32 = 1.0 / (0.075);
  Q_03 = -319.5;
  Q_13 = -239.5;
  Q_23 = 525.0;
  Q_33 = 0;
}

void ParticleTrack::initialize(cv::Mat& input, cv::Mat& disp, CvRect inROI,
    std::string filterDir, cv::Mat& Q, double angle)
{
  if(inROI.x<0 || inROI.y<0 ||inROI.x+inROI.width >=input.cols || inROI.y+inROI.height>=input.rows){
    ROS_ERROR("inROI out of bounds %d %d %d %d",inROI.x,inROI.y,inROI.height,inROI.width);
  }
  if(inROI.x<0 || inROI.y<0 ||inROI.x+inROI.width >=disp.cols || inROI.y+inROI.height>=disp.rows){
    ROS_ERROR("inROI out of bounds %d %d %d %d",inROI.x,inROI.y,inROI.height,inROI.width);
  }
  if(inROI.height<=0 || inROI.width<=0){
    ROS_ERROR("inROI size too small %d %d",inROI.height,inROI.width);
  }
  age = 0.0;
  position_history.clear();

  // Reprojection matrix
  Q_32 = Q.at<double>(3,2);
  Q_03 = Q.at<double>(0,3);
  Q_13 = Q.at<double>(1,3);
  Q_23 = Q.at<double>(2,3);
  Q_33 = Q.at<double>(3,3);

  // Rotation matrix
  angle_ = angle;
  R = cv::Mat::zeros(3, 3, CV_64F);
  R.at<double>(0, 0) = 1;
  R.at<double>(1, 1) = std::cos(angle_ * CV_PI / 180.0);
  R.at<double>(1, 2) = -std::sin(angle_ * CV_PI / 180.0);
  R.at<double>(2, 1) = std::sin(angle_ * CV_PI / 180.0);
  R.at<double>(2, 2) = std::cos(angle_ * CV_PI / 180.0);

  // Find x-z location by filtering map
  cv::Point2f xz = find_location(disp, inROI);
  cv::Point xz_loc;
  cv::minMaxLoc(xz_map_, NULL, NULL, NULL, &xz_loc);
  make_object_hist(input, disp, xz_loc);

  // Object properties
  size_ = inROI.height * xz.y; // height * distance should be constant
  ar_ = (double) inROI.width / (double) inROI.height;

  // Initialize particle filter
  load_model(filterDir.c_str());
  Particle initialState;
  initialState.weight = 1.0;
  initialState.prevWeight = 1.0;
  initialState.x = cv::Mat::zeros(2, 1, CV_32F);
  initialState.x.at<float>(0, 0) = xz.x;
  initialState.x.at<float>(1, 0) = xz.y;
  ParticleFilter::initialize(initialState);

  // Initialize forest and features
  forest1.randomFeatures(27 + 48);
  features1.computePool1(input, inROI);
  forest1.addToTrees(features1.featurePool1);
}

void ParticleTrack::revive(cv::Mat& input, cv::Mat& disp, CvRect inROI)
{
  if(inROI.x<0 || inROI.y<0 ||inROI.x+inROI.width >=input.cols || inROI.y+inROI.height>=input.rows){
    ROS_ERROR("inROI out of bounds %d %d %d %d",inROI.x,inROI.y,inROI.height,inROI.width);
  }
  if(inROI.x<0 || inROI.y<0 ||inROI.x+inROI.width >=disp.cols || inROI.y+inROI.height>=disp.rows){
    ROS_ERROR("inROI out of bounds %d %d %d %d",inROI.x,inROI.y,inROI.height,inROI.width);
  }
  if(inROI.height<=0 || inROI.width<=0){
    ROS_ERROR("inROI size too small %d %d",inROI.height,inROI.width);
  }

  if (debug_mode_)
    ROS_INFO("Reviving tracker %d", (int) uid);
  age = 0.0;
  position_history.clear();
  // Find x-z location by filtering map
  cv::Point2f xz = find_location(disp, inROI);
  cv::Point xz_loc;
  cv::minMaxLoc(xz_map_, NULL, NULL, NULL, &xz_loc);
  make_object_hist(input, disp, xz_loc);

  // Initialize particle filter
  Particle initialState;
  initialState.weight = 1.0;
  initialState.prevWeight = 1.0;
  initialState.x = cv::Mat::zeros(2, 1, CV_32F);
  initialState.x.at<float>(0, 0) = xz.x;
  initialState.x.at<float>(1, 0) = xz.y;
  ParticleFilter::initialize(initialState);
}

int ParticleTrack::update_state(cv::Mat& input, cv::Mat& disp, double dt)
{
  age += dt;
  if (is_dead_)
    return is_dead_;
  // ROS_INFO("XZ map...");
  // Make overhead map
  make_xz_map_weighted(input, disp, NULL);

  // Normalize and display
  double maxVal;
  cv::minMaxLoc(xz_map_, NULL, &maxVal);
  //  ROS_ERROR("%f,maxVal", maxVal);
  maxVal = 2500.0;
  xz_map_ = xz_map_ * (1.0 / maxVal);

  if(debug_mode_)
  {
    cv::Mat xzDisplay;
    cv::resize(xz_map_, xzDisplay, cv::Size(400, 400), 0, 0, cv::INTER_AREA);
    cv::Point loc;
    loc.x = (current_map_.x.at<float>(0, 0) - xz_map_offset_) / xz_map_res_ * 400 / xz_map_.rows;
    loc.y = current_map_.x.at<float>(1, 0) / xz_map_res_ * 400 / xz_map_.rows;
    cv::circle(xzDisplay, loc, 10, cv::Scalar(200), 1);
    cv::flip(xzDisplay, xzDisplay, 0);
    cv::imshow("Test", xzDisplay);
    cv::waitKey(10);
  }

  // update filter state (will use passed through Mat objects in likelihood fcn)
  if (debug_mode_)
    ROS_INFO("Updating object %d, dt = %f", uid, dt);
  updateState(input, dt);
  if (is_dead_)
  {
    if (debug_mode_)
      ROS_INFO("Tracker %d died.", uid);
    return is_dead_;
  }

  // use frame to improve online learning
  grow(input);

  // push back position history
  pos_time temp;
  temp.dt = dt;
  temp.position.x = current_map_.x.at<float>(0, 0);
  temp.position.y = current_map_.x.at<float>(1, 0);
  position_history.push_back(temp);

  return is_dead_;
}

cv::Point2f ParticleTrack::compute_velocity()
{
  double dt_sum = 0.0;
  cv::Point2f v;
  v.x = 0;
  v.y = 0;
  if (position_history.size() < 2)
    return v;
  int idx = position_history.size() - 1;
  while (dt_sum < .5) //half a second history
  {
    dt_sum += position_history[idx].dt;
    idx--;
    if (idx == 0)
      break;
  }
  v.x = position_history.back().position.x - position_history[idx].position.x;
  v.y = position_history.back().position.y - position_history[idx].position.y;
  v.x *= 1.0 / dt_sum;
  v.y *= 1.0 / dt_sum;
  return v;
}

double ParticleTrack::compute_speed()
{
  cv::Point2f vel = compute_velocity();
  return std::sqrt(std::pow(vel.x, 2) + std::pow(vel.y, 2));
}

void ParticleTrack::grow(cv::Mat& input)
{
  cv::Point2f loc;
  loc.x = current_map_.x.at<float>(0, 0);
  loc.y = current_map_.x.at<float>(1, 0);
  objectROI = xz_to_box(loc);
  if ((objectROI.x > 0) && (objectROI.y > 0)
      && (objectROI.x + objectROI.width < input.cols)
      && (objectROI.y + objectROI.height < input.rows)
      && objectROI.width>0 && objectROI.height>0 )
  {
    features1.computePool1(input, objectROI);
    forest1.addToTrees(features1.featurePool1);
  }
}

void ParticleTrack::draw_box(cv::Mat& image, CvScalar clr)
{
  if (!is_dead_)
  {
    cv::Point2f loc;
    loc.x = current_map_.x.at<float>(0, 0);
    loc.y = current_map_.x.at<float>(1, 0);
    objectROI = xz_to_box(loc);
    cv::rectangle(image, objectROI, clr, 2);
    if ((objectROI.x > 0) && (objectROI.y > 0)
        && (objectROI.x + objectROI.width < image.cols)
        && (objectROI.y + objectROI.height < image.rows)
	&& objectROI.width>0 && objectROI.height>0 )
    {
      char buffer[50];
      sprintf(buffer, "%d", (int)uid);
      cv::putText(image, std::string(buffer), cv::Point(objectROI.x, objectROI.y), FONT_HERSHEY_PLAIN, 2.0, cv::Scalar(255));
      sprintf(buffer, "%0.2fm/s", compute_speed());
      cv::putText(image, std::string(buffer), cv::Point(objectROI.x, objectROI.y + objectROI.height), FONT_HERSHEY_PLAIN, 1., cv::Scalar(0, 255, 0));
    }

  }
}

cv::Rect ParticleTrack::xz_to_box(cv::Point2f xz)
{
  cv::Rect box;
  box.height = size_ / xz.y;
  box.width = box.height * ar_;
  box.x = (xz.x+xz_map_res_/2.0) / xz.y * Q_23 - Q_03 - box.width / 2;

  // y of box, rotate for camera orientation
  double y = h; // 0 draws box at height of camera
  y = y * std::cos(angle_ * CV_PI / 180.0) + xz.y * std::sin(angle_ * CV_PI / 180.0);

  // get from real-world y to image y
  double W = Q_23 / xz.y; // W = Q_23/z
  box.y = y * W - Q_13; // (i + Q_13)/W = y --> i = y*W - Q_13
  if(box.height <1 || box.width<1) ROS_WARN("Box = %d %d %d %d",box.x,box.y,box.height,box.width);

  return box;
}

double ParticleTrack::likelihood(cv::Mat& input, Particle states)
{
  double prob = 0.0;
  // Compute the particle's likelihood from xz
  // gaussian filter on image
  // score of the particle based on xz state
  float x = states.x.at<float>(0, 0);
  float z = states.x.at<float>(1, 0);
  int xz_row = z / xz_map_res_;
  int xz_col = (x - xz_map_offset_) / xz_map_res_;

  if((xz_row >= 0)&&(xz_col >= 0)&&(xz_row < xz_map_.rows)&&(xz_col < xz_map_.cols))
    prob = xz_map_.at<float>(z / xz_map_res_, (x - xz_map_offset_) / xz_map_res_); //Need to normalize

  return prob;
}

cv::Point2f ParticleTrack::find_location(cv::Mat& disparity, cv::Rect roi)
{
  // Create x-z image
  make_xz_map(disparity, &roi);

  if(0)
  {
    cv::Mat xzDisplay;
    cv::resize(xz_map_, xzDisplay, cv::Size(400, 400), 0, 0, cv::INTER_AREA);
    double maxVal = 0.0;
    cv::minMaxLoc(xz_map_, NULL, &maxVal, NULL, NULL);
    xzDisplay /= maxVal;
    cv::Point loc;
    loc.x = (current_map_.x.at<float>(0, 0) - xz_map_offset_) / xz_map_res_ * 400 / xz_map_.rows;
    loc.y = current_map_.x.at<float>(1, 0) / xz_map_res_ * 400 / xz_map_.rows;
    cv::circle(xzDisplay, loc, 10, cv::Scalar(200), 1);
    cv::flip(xzDisplay, xzDisplay, 0);
    cv::imshow("Trackers", xzDisplay);
    cv::waitKey(0);
  }

  // Find maxVal location
  cv::Point maxLoc;
  cv::minMaxLoc(xz_map_, NULL, NULL, NULL, &maxLoc);
  cv::Point2f loc;
  loc.x = maxLoc.x * xz_map_res_ + xz_map_offset_;
  loc.y = maxLoc.y * xz_map_res_;
  if(loc.y==0) loc.y = 1.0;
  return loc;
}

double ParticleTrack::score(cv::Mat& image, cv::Mat& disp, cv::Rect roi)
{
  double score = 0.0;
  // Compare location of roi with location of tracker
  cv::Point2f detLoc = find_location(disp, roi);
  cv::Point2f trkLoc;
  trkLoc.x = current_map_.x.at<float>(0, 0);
  trkLoc.y = current_map_.x.at<float>(1, 0);
  double dist = pow((double) detLoc.x - trkLoc.x, 2) + pow((double) detLoc.y - trkLoc.y, 2);
  dist = std::sqrt(dist);
  score += dist * -5.0 + 10.0; // score 10 up close, 0 for 5 meters away

  // Compare appearance of object with appearance of tracker
  roi = xz_to_box(detLoc);
  if ((roi.x > 0) && (roi.y > 0) && (roi.x + roi.width < image.cols) && (roi.y + roi.height < image.rows))
  {
    features1.computePool1(image, roi);
    double votes = forest1.classify(features1.featurePool1);
    score += votes / N_TREES;
    // ROS_INFO("Votes = %f", votes);
  }

  // return the score
  return score;
}

void ParticleTrack::make_xz_map(cv::Mat& disp, cv::Rect* roi)
{
  // Create x-z image
  xz_map_res_ = 0.0625; // res in meters
  xz_map_offset_ = -2.5; // start at x = -10m
  xz_map_ = cv::Mat::zeros(5 / xz_map_res_, 5 / xz_map_res_, CV_32F); //5m x 5m

  int minI, maxI, minJ, maxJ;
  if (roi == NULL)
  {
    minI = 0;
    minJ = 0;
    maxI = disp.rows;
    maxJ = disp.cols;
  }
  else
  {
    minI = roi->y;
    minJ = roi->x;
    maxI = roi->y + roi->height;
    maxJ = roi->x + roi->width;
  }

  // Project points into xz
  for (int i = minI; i < maxI; i++)
    for (int j = minJ; j < maxJ; j++)
      if (disp.at<float>(i, j) > 0.5)
      {
        Point3d xyz(j + Q_03, i + Q_13, Q_23);
        double W = Q_32 * disp.at<float>(i, j);
        xyz *= (1.0 / W);

        //Rotate
        Point3d rxyz;
        rxyz.x = xyz.x;
        rxyz.y = R.at<double>(1, 1) * xyz.y + R.at<double>(1, 2) * xyz.z;
        rxyz.z = R.at<double>(2, 1) * xyz.y + R.at<double>(2, 2) * xyz.z;
        xyz = rxyz;

        int x_idx = xyz.x / xz_map_res_ - xz_map_offset_ / xz_map_res_;
        int z_idx = xyz.z / xz_map_res_;
        //      model_.projectDisparityTo3d(Point2d(j, i), disparity.at<float>(i,j), xyz);
        if ((z_idx > 0) && (z_idx < xz_map_.rows) && (x_idx > 0) && (x_idx < xz_map_.cols))
        {
          if ((xyz.y > -.05) && (xyz.y < 1.0))
            xz_map_.at<float>(z_idx, x_idx) += xyz.z;
        } // if
      } // for i, j
  cv::GaussianBlur(xz_map_, xz_map_, cv::Size(3, 3), 1.0);
}

void ParticleTrack::make_xz_map_weighted(cv::Mat& image, cv::Mat& disp, cv::Rect* roi)
{
  // Create x-z image
  xz_map_res_ = 0.0625; // res in meters
  xz_map_offset_ = -2.5; // start at x = -10m
  xz_map_ = cv::Mat::zeros(5 / xz_map_res_, 5 / xz_map_res_, CV_32F); //5m x 5m

  int minI, maxI, minJ, maxJ;
  if (roi == NULL)
  {
    minI = 0;
    minJ = 0;
    maxI = disp.rows;
    maxJ = disp.cols;
  }
  else
  {
    minI = roi->y;
    minJ = roi->x;
    maxI = roi->y + roi->height;
    maxJ = roi->x + roi->width;
  }

  // Backproject object histogram
  // convert image to HSI and split
  cv::Mat hsv;
  cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);
  std::vector<cv::Mat> channels;
  cv::split(hsv, channels);
  cv::Mat planes[2];
  planes[0] = channels[0];
  planes[1] = channels[2];

  // hist properties
  int bins[] = { 50, 50 };
  float h_ranges[] = { 0, 255 };
  float s_ranges[] = { 0, 255 };
  const float* ranges[] = { h_ranges, s_ranges };

  cv::Mat backproj;
  cv::calcBackProject(planes, 2, 0, hist_, backproj, &(ranges[0]), 1, true);
  backproj.convertTo(backproj, CV_32F);

  /// Draw the backproj
  if(0)
  {
    imshow("BackProj", backproj / 10.0);
    cv::waitKey(10);
  }

  // Project points into xz
  for (int i = minI; i < maxI; i++)
    for (int j = minJ; j < maxJ; j++) /*if(disp.at<float>(i,j) > 0.5)*/
    {
      Point3d xyz(j + Q_03, i + Q_13, Q_23);
      double W = Q_32 * disp.at<float>(i, j);
      xyz *= (1.0 / W);

      //Rotate
      Point3d rxyz;
      rxyz.x = xyz.x;
      rxyz.y = R.at<double>(1, 1) * xyz.y + R.at<double>(1, 2) * xyz.z;
      rxyz.z = R.at<double>(2, 1) * xyz.y + R.at<double>(2, 2) * xyz.z;
      xyz = rxyz;

      int x_idx = xyz.x / xz_map_res_ - xz_map_offset_ / xz_map_res_;
      int z_idx = xyz.z / xz_map_res_;
      //      model_.projectDisparityTo3d(Point2d(j, i), disparity.at<float>(i,j), xyz);
      if ((z_idx > 0) && (z_idx < xz_map_.rows) && (x_idx > 0) && (x_idx < xz_map_.cols))
      {
        if ((xyz.y > -.05) && (xyz.y < 1.0))
          xz_map_.at<float>(z_idx, x_idx) += xyz.z * backproj.at<float>(i, j);
//        	else
//        		image.at<cv::Vec3b>(i,j) = cv::Vec3b(0,0,0);
      } // if
//        else
//            image.at<cv::Vec3b>(i,j) = cv::Vec3b(0,0,0);
    } // for i, j
  cv::GaussianBlur(xz_map_, xz_map_, cv::Size(5, 5), 2.0);
}

void ParticleTrack::make_object_hist(cv::Mat& image, cv::Mat& disp, cv::Point maxLoc)
{ // also finds the height of the object

  h = 0.356; // shortest height (95% adult female distribution, assume camera at h = 6')
  cv::Mat mask = cv::Mat::zeros(image.rows, image.cols, CV_8U);
  // Project points into xz
  for (int i = 0; i < disp.rows; i++)
    for (int j = 0; j < disp.cols; j++)
      if (disp.at<float>(i, j) > 0.5)
      {
        Point3d xyz(j + Q_03, i + Q_13, Q_23);
        double W = Q_32 * disp.at<float>(i, j);
        xyz *= (1.0 / W);

        //Rotate
        Point3d rxyz;
        rxyz.x = xyz.x;
        rxyz.y = R.at<double>(1, 1) * xyz.y + R.at<double>(1, 2) * xyz.z;
        rxyz.z = R.at<double>(2, 1) * xyz.y + R.at<double>(2, 2) * xyz.z;
        xyz = rxyz;

        int x_idx = xyz.x / xz_map_res_ - xz_map_offset_ / xz_map_res_;
        int z_idx = xyz.z / xz_map_res_;

        // Making the mask
        if ((z_idx == maxLoc.y) && (x_idx == maxLoc.x))
        {
          if ((xyz.y > -.05) && (xyz.y < 1.0))
            mask.at<uint8_t>(i, j) = 1;
        } // if

        // Finding the height
        if ((std::abs(z_idx - maxLoc.y) < 3) && (std::abs(x_idx - maxLoc.x) < 3))
        {
          if ((xyz.y > -.20) && (xyz.y < 1.0))
            if(xyz.y < h)
              h = xyz.y;
        } // if

      } // for i, j

    // convert image to HSI and split
  cv::Mat hsv;
  cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);
  std::vector<cv::Mat> channels;
  cv::split(hsv, channels);
  cv::Mat planes[2];
  planes[0] = channels[0];
  planes[1] = channels[2];

  // hist properties
  int bins[] = { 50, 50 };
  float h_ranges[] = { 0, 255 };
  float s_ranges[] = { 0, 255 };
  const float* ranges[] = { h_ranges, s_ranges };

  // calc hist
  cv::calcHist(planes, 2, 0, mask, hist_, 2, &(bins[0]), &(ranges[0]), true, false);

  // debug -- show backprojection image
  double sumT = cv::sum(hist_).val[0];
  hist_ = hist_ * 500.0 / sumT;
}
