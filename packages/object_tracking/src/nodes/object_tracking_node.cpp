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

#include "ros/ros.h"
#include <sstream>

//Publish Messages
#include "roi_msgs/RoiRect.h"
#include "roi_msgs/Rois.h"
#include "std_msgs/String.h"
#include "roi_msgs/HumanEntry.h"
#include "roi_msgs/HumanEntries.h"

//Time Synchronizer
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

//Subscribe Messages
#include <sensor_msgs/Image.h>
#include <stereo_msgs/DisparityImage.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

// Image Transport
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

// Used to display OPENCV images
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

// Tracker libraries
#include <object_tracking/tracker.h>
#include <boost/shared_ptr.hpp>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/make_shared.hpp>
#include <boost/thread.hpp>


using namespace stereo_msgs;
using namespace message_filters::sync_policies;
using namespace roi_msgs;
using namespace sensor_msgs;
using namespace sensor_msgs::image_encodings;
using sensor_msgs::Image;
using cv_bridge::CvImagePtr;
using std::vector;
using std::string;
using cv::Rect;
using cv::Mat;

class ObjectTrackingNode
{
private:
  // Define Node
  ros::NodeHandle node_;

  // Subscribe to Messages
  message_filters::Subscriber<DisparityImage> sub_disparity_;
  message_filters::Subscriber<Image> sub_image_;
  message_filters::Subscriber<Rois> sub_rois_;

  message_filters::Subscriber<sensor_msgs::CameraInfo> l_camera_info_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> r_camera_info_;

  // Define the Synchronizer
  typedef ApproximateTime<Image, DisparityImage, Rois> ApproximatePolicy;
  typedef message_filters::Synchronizer<ApproximatePolicy> ApproximateSync;
  boost::shared_ptr<ApproximateSync> approximate_sync_;

  typedef ApproximateTime<sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> ApproximatePolicy2;
  typedef message_filters::Synchronizer<ApproximatePolicy2> ApproximateSync2;
  boost::shared_ptr<ApproximateSync2> camera_info_sync_;

  // Messages to Publish
  ros::Publisher pub_Color_Image_;
  ros::Publisher human_tracker_data_;

  // Trackers
  std::vector<boost::shared_ptr<ParticleTrack> > trackers_;
  std::vector<cv::Rect> detections_;

  // Parallelization
  int thread_count_;
  boost::asio::io_service service_;
  boost::shared_ptr<boost::asio::io_service::work> work_;
  boost::thread_group task_threads_;
  boost::condition_variable task_condition_;
  boost::mutex task_mutex_;

  cv::Mat image, disp;
  cv::Mat Q_;
  uint64_t uid_counter_;
  uint64_t write_count_;
  std::string filter_dir_;
  std::string csv_filename_;
  std::ofstream log_file_;
  double object_timeout_;
  double angle_;
  double dt_;
  ros::Time current_;
  ros::Time previous_;
  bool geometry_initialized_;
  bool calculate_covariance_;
  bool show_images_;
  int max_objects_;
  bool write_image_results_;
  bool debug_mode_;

public:

  explicit ObjectTrackingNode(const ros::NodeHandle& nh):
    node_(nh)
  {

    string nn = ros::this_node::getName();
    int qs;
    if(!node_.getParam(nn + "/Q_Size",qs)){
      qs=3;
    }
    uid_counter_ = 0;
    write_count_ = 0;

    if(!node_.getParam(nn + "/Filter_Dir",filter_dir_))
    {
      filter_dir_ = std::string("/home/clewis/ros/10.17866/packages/object_tracking/perception_data");
    }

    if(!node_.getParam(nn + "/Thread_Count",thread_count_)){
      thread_count_ = 5; // number of threads for updating tracker states
    }

    if(!node_.getParam(nn + "/Max_Objects",max_objects_)){
      max_objects_ = 5; // number of threads for updating tracker states
    }

    if(!node_.getParam(nn + "/CSV_Filename",csv_filename_)){
      csv_filename_ = std::string("none"); // csv file
    }

    if(!node_.getParam(nn + "/Object_Timeout",object_timeout_)){
      object_timeout_ = 5.0; // 5.0s default timeout
    }

    if(!node_.getParam(nn + "/Calculate_Covariance", calculate_covariance_)){
      calculate_covariance_ = false; // don't calculate by default
    }

    if(!node_.getParam(nn + "/Show_Images", show_images_)){
      show_images_ = true; // show tracker image by default
    }

    if(!node_.getParam(nn + "/Write_image_results", write_image_results_)){
    	write_image_results_ = false; // don't publish by default
    }

    if(!node_.getParam(nn + "/Debug_mode", debug_mode_)){
      debug_mode_ = true; // debug_mode active by default
    }

    if(csv_filename_ != "none")
    {
      log_file_.open(csv_filename_.c_str(), std::ios_base::out);
      log_file_ << "# Time(ns)" << "," <<
          "personID" << "," <<
          "headCentroidX" << "," <<
          "headCentroidY" << "," <<
          "headCentroidZ" << "," <<
          "BoundingBoxTopCenterX" << "," <<
          "BoundingBoxTopCenterY" << "," <<
          "BoundingBoxTopCenterZ" << "," <<
          "Xvelocity" << "," <<
          "Yvelocity" << "," <<
          "Zvelocity" << "," <<
          "Xsigma" << "," <<
          "Ysigma" << "," <<
          "Zsigma" << "," <<
          "ROIwidth" << "," <<
          "ROIheight";

          if (write_image_results_)
          {
            log_file_ << "," <<
                "bbox_2D_x" << "," <<
                "bbox_2D_y" << "," <<
                "bbox_2D_width" << "," <<
                "bbox_2D_height";
          }

       log_file_ << std::endl;

    }

    // Published Messages
    human_tracker_data_ = node_.advertise<roi_msgs::HumanEntries> ("/human_tracker_data", 10);

    // Subscribe to Messages
    sub_image_.subscribe(node_,"Color_Image",qs);
    sub_disparity_.subscribe(node_, "Disparity_Image",qs);
    sub_rois_.subscribe(node_,"input_rois",qs);

    l_camera_info_.subscribe(node_,"l_camera_info",qs);
    r_camera_info_.subscribe(node_,"r_camera_info",qs);

    // Sync the Synchronizer
    approximate_sync_.reset(new ApproximateSync(ApproximatePolicy(qs),
            sub_image_, sub_disparity_, sub_rois_));

    approximate_sync_->registerCallback(boost::bind(&ObjectTrackingNode::imageCb,
                this,_1,_2,_3));

    camera_info_sync_.reset(new ApproximateSync2(ApproximatePolicy2(qs),
            l_camera_info_, r_camera_info_));

    camera_info_sync_->registerCallback(boost::bind(&ObjectTrackingNode::infoCallback,
                this,_1,_2));

    // Parallelization
    work_.reset(new boost::asio::io_service::work(service_));
    for (int i = 0; i < thread_count_; i++)
    {
      task_threads_.create_thread(
          boost::bind(&boost::asio::io_service::run, &service_));
    }

    // Visualization
    if(show_images_)
    {
      cv::namedWindow("Trackers", 0 ); // non-autosized
    }
    if(debug_mode_)
    {
      cv::namedWindow("Test",0);
    }
    if (show_images_ || debug_mode_)
    {
      cv::startWindowThread();
    }

    // Geometry
    angle_ = -180.0;
    geometry_initialized_ = false;
    Q_ = cv::Mat::zeros(4,4,CV_64F); // initialize with default kinect parameters
    Q_.at<double>(3,2) = 1.0 / (0.075); // 1/baseline
    Q_.at<double>(0,3) = -319.5; // -cx
    Q_.at<double>(1,3) = -239.5; // -cy

    double fixed_focal_length;
    if(!node_.getParam(nn + "/FixedFocalLength", fixed_focal_length)){
      fixed_focal_length = 525; // don't calculate by default
    }
    Q_.at<double>(2,3) = fixed_focal_length; // fx
    ROS_ERROR("using focal length of %lf",fixed_focal_length);
    previous_ = ros::Time::now();
  }

  void infoCallback(const sensor_msgs::CameraInfoConstPtr& l_cam_info,
      const sensor_msgs::CameraInfoConstPtr& r_cam_info)
  {
    // Get camera info
    if(!geometry_initialized_)
    {
      sensor_msgs::CameraInfo l_info = *l_cam_info;
      sensor_msgs::CameraInfo r_info = *r_cam_info;

      //extract baseline from camera info
      double baseline;
      if(l_info.P[3] < 0.0)
      {
        ROS_ERROR("Left and right camera info are switched."
            "Please check launch file");
        baseline = -l_info.P[3] / l_info.P[0];
      }
      else if(r_info.P[3] < 0.0)
      {
        baseline = -r_info.P[3]/r_info.P[0];
      }
      else
      {
        ROS_ERROR("No baseline detected.  Using default baseline"
            "of 0.075m");
        baseline = 0.075;
      }
      ROS_ERROR("using baseline=%lf",baseline);

      double divisor= l_info.binning_x;
      if (divisor<1) divisor = 1.0;// avoid div by zero (kinect)
      Q_.at<double>(3,2) = 1.0 / baseline; // 1/baseline
      Q_.at<double>(0,3) = -l_info.K[2]/divisor; // -cx
      Q_.at<double>(1,3) = -l_info.K[5]/divisor; // -cy

      string nn = ros::this_node::getName();
      double fixed_focal_length;
      if(!node_.getParam(nn + "/FixedFocalLength", fixed_focal_length)){
  fixed_focal_length = l_info.K[0]/divisor; // fx
      }
      Q_.at<double>(2,3) = fixed_focal_length; // fx
      ROS_ERROR("using focal length of %lf",fixed_focal_length);

      std::cout << "Reprojection Matrix:" << std::endl << Q_ << std::endl;
      geometry_initialized_ = true;
    }
  }


  void imageCb(const ImageConstPtr& image_msg,
         const DisparityImageConstPtr& disparity_msg,
         const RoisConstPtr& rois_msg){

    bool label_all;
    vector<Rect> R_out;
    vector<int> L_out;
    string param_name;
    string nn = ros::this_node::getName();
    string cfnm;
    int numSamples;

    // Make sure geometry has been initialized
    if(!geometry_initialized_)
      return;

    // Get timestamp
    current_ = image_msg->header.stamp;
    dt_ = current_.toSec() - previous_.toSec();

    //Use CV Bridge to convert image
    CvImagePtr cv_gray  = cv_bridge::toCvCopy(image_msg, image_encodings::MONO8);
    CvImagePtr cv_color  = cv_bridge::toCvCopy(image_msg, image_encodings::BGR8);
    image  = cv_color->image;

    // check encoding and create an intensity image from disparity image
    assert(disparity_msg->image.encoding == image_encodings::TYPE_32FC1);
    cv::Mat disp1(disparity_msg->image.height,
          disparity_msg->image.width,
          CV_32F,
          (float*) &disparity_msg->image.data[0],
          disparity_msg->image.step);
    disp = disp1;

    // Estimate angle initially to make sure it is within tolerance
    // Angle is used to correct for rotation when making 2D xz ground map
    if(angle_ < -179.0){
      angle_ = calculate_angle(disp);
      ROS_ERROR("Ground Plane Angle = %lf",angle_);
    }
    if(std::abs(angle_) > 15.0)
    {
      angle_ = -180.0;
      return;
    }

    // take the region of interest message and create vectors of ROIs and labels
    detections_.clear();
    for(unsigned int i = 0;i < rois_msg->rois.size(); i++){
      int x = rois_msg->rois[i].x;
      int y = rois_msg->rois[i].y;
      int w = rois_msg->rois[i].width;
      int h = rois_msg->rois[i].height;
      int l = rois_msg->rois[i].label;
      Rect R(x,y,w,h);
      detections_.push_back(R);
    }

    // Update all objects
    int threads = 0;
    int finished = 0;
    for(int i = 0; i < trackers_.size(); i++)
    {
      service_.post(boost::bind(&ObjectTrackingNode::update_task, this, i, 0.05, &finished)); // post task
      threads++;
    }

    // wait for all jobs to finish
    boost::unique_lock<boost::mutex> lock(task_mutex_);
    while (finished < threads && ros::ok())
    {
      task_condition_.wait(lock);
    }

    // Register detections to existing objects
    match_detections();

    // Add unresolved detections to world model
    add_new();

    // Draw all of the boxes
    for(int i = 0; i < trackers_.size(); i++)
    {
      trackers_[i]->draw_box(image,cv::Scalar(255));
    }

    if(show_images_)
      cv::imshow("Trackers", image);

    // Kill old objects (objects not matched to a detection recently)
    kill_old();

    // Publish object states
    publish_object_states();

    previous_ = current_;
  }

  void match_detections()
  {
    if(trackers_.empty() || detections_.empty()) return;

    cv::Mat score;
    score = cv::Mat::zeros(detections_.size(), trackers_.size(), CV_32F);
    for(int i = 0; i < detections_.size(); i++)
      for(int j = 0; j < trackers_.size(); j++)
        score.at<float>(i,j) = trackers_[j]->score(image, disp, detections_[i]);

    // enter a loop of finding best match, removing detections that match well enough
    double maxVal;
    cv::Point maxLoc;
    cv::minMaxLoc(score, 0, &maxVal, 0, &maxLoc);

    while(maxVal > 5.0)
    {
      // set row and column of best match to zero and invalidate the roi
      score.row(maxLoc.y) = 0.0; //this detection can't be matched to two trackers
      //score.col(maxLoc.x) = 0.0; // commented out means the tracker may match multiple detections
      if(trackers_[maxLoc.x]->is_dead_ && detections_[maxLoc.y].width > 0)
        trackers_[maxLoc.x]->revive(image,disp,detections_[maxLoc.y]);
      detections_[maxLoc.y].width = 0;
      trackers_[maxLoc.x]->age = 0;

      cv::minMaxLoc(score, 0, &maxVal, 0, &maxLoc);
    }
  }

  void add_new()
  {
    int current_trackers = trackers_.size();
    for(int i = 0; i < detections_.size(); i++)
      if(detections_[i].width > 0) if (trackers_.size() < max_objects_)
      {
        // Make sure object couldn't be resolved to newly added object
        bool should_continue = false;
        for(int j = current_trackers; j < trackers_.size(); j++)
          if(trackers_[j]->score(image, disp, detections_[i]) > 9.0)
            should_continue = true;
        if(should_continue)
          continue;

        // Create a new tracker object
        boost::shared_ptr<ParticleTrack> temp(new ParticleTrack());
        temp->debug_mode_ = debug_mode_;
        temp->initialize(image, disp, detections_[i], filter_dir_, Q_, angle_);
        if(temp->size_>1)
        {
          temp->uid = uid_counter_++;
          trackers_.push_back(temp);
        }
      }
  }

  void kill_old()
  {
    std::vector<boost::shared_ptr<ParticleTrack> >::iterator iter = trackers_.begin();
    while (iter != trackers_.end())
    {
      if(((*iter)->age > object_timeout_))
        iter = trackers_.erase(iter);
      else
        ++iter;
    }
  }

  void update_task(int idx, double dt, int* finished)
  {
    trackers_[idx]->update_state(image, disp, dt_);

    boost::lock_guard<boost::mutex> lock(task_mutex_);
    (*finished)++;
    task_condition_.notify_one();
  }

  void publish_object_states()
  {
    // accumulate data
    roi_msgs::HumanEntries humans;
    humans.entries.clear();
    humans.header.stamp = current_;
    std::vector<boost::shared_ptr<ParticleTrack> >::iterator iter = trackers_.begin();
    while (iter != trackers_.end())
    {
      if (!((*iter)->is_dead_))
      {
        // Output coordinate system rotated from tracker coordinate system
        // R = [1 0 0; 0 0 1; 0 -1 0]
        roi_msgs::HumanEntry entry;
        entry.stamp = humans.header.stamp;
        entry.personID = (*iter)->uid;
        entry.personCentroidX = (*iter)->current_map_.x.at<float>(0, 0);
        entry.personCentroidY = (*iter)->current_map_.x.at<float>(1, 0);
        entry.personCentroidZ = -(*iter)->get_height() - 5.0*.0254; // -half avg human head height
        entry.personBoundingBoxTopCenterX = (*iter)->current_map_.x.at<float>(0, 0);
        entry.personBoundingBoxTopCenterY = (*iter)->current_map_.x.at<float>(1, 0);
        entry.personBoundingBoxTopCenterZ = -(*iter)->get_height();
        entry.ROIwidth = (*iter)->get_box_size() / Q_.at<double>(2,3)*(*iter)->get_ar();
        entry.ROIheight = (*iter)->get_box_size() / Q_.at<double>(2,3);
        entry.Xvelocity = (*iter)->compute_velocity().x;
        entry.Yvelocity = (*iter)->compute_velocity().y;
        entry.Zvelocity = 0.0;
        if(!calculate_covariance_)
        {
          entry.Xsigma = 0.0;
          entry.Ysigma = 0.0;
          entry.Zsigma = 0.0;
        }
        else
        {
          cv::Mat covar = (*iter)->calculate_covariance();
          entry.Xsigma = std::sqrt(covar.at<double>(0,0));
          entry.Ysigma = std::sqrt(covar.at<double>(1,1));
          entry.Zsigma = 0.0;
        }

        humans.entries.push_back(entry);
      }
      ++iter;
    }
    // publish message
    human_tracker_data_.publish(humans);

    // Collect image bounding boxes for every track (if write_image_results_ is true):
    roi_msgs::Rois image_results;
    if (write_image_results_)
    {
    	// Compute image bounding box from 3D coordinates
    	image_results.rois.clear();
    	image_results.header.stamp = current_;
    	for(int i = 0; i < trackers_.size(); i++)
    	{
    	  if (!(trackers_[i]->is_dead_))
    	  {
    	    cv::Point2f loc;
    	    loc.x = trackers_[i]->current_map_.x.at<float>(0, 0);
    	    loc.y = trackers_[i]->current_map_.x.at<float>(1, 0);
    	    cv::Rect bbox_2D = trackers_[i]->xz_to_box(loc);

    	    roi_msgs::RoiRect entry;
          entry.x = bbox_2D.x;
          entry.y = bbox_2D.y;
          entry.width = bbox_2D.width;
          entry.height = bbox_2D.height;
          image_results.rois.push_back(entry);
    	  }
    	}
    }

    // log data
    if(csv_filename_ != "none")
    {
      for(int i = 0; i < humans.entries.size(); i++)
      {
  double vx = humans.entries[i].personCentroidX;
  double vy = humans.entries[i].personCentroidY;
  double vz = humans.entries[i].personCentroidZ;
  double norm = sqrt(vx*vx+vy*vy+vz*vz);
  vx = vx/norm*.12;
  vy = vy/norm*.12;
  vz = vz/norm*.12;
        log_file_ << humans.entries[i].stamp.toNSec() << "," <<
            humans.entries[i].personID << "," <<
            humans.entries[i].personCentroidX+vx << "," <<
            humans.entries[i].personCentroidY+vy << "," <<
            humans.entries[i].personCentroidZ+vz << "," <<
            humans.entries[i].personBoundingBoxTopCenterX + vx << "," <<
            humans.entries[i].personBoundingBoxTopCenterY + vy << "," <<
            humans.entries[i].personBoundingBoxTopCenterZ + vz << "," <<
            humans.entries[i].Xvelocity << "," <<
            humans.entries[i].Yvelocity << "," <<
            humans.entries[i].Zvelocity << "," <<
            humans.entries[i].Xsigma << "," <<
            humans.entries[i].Ysigma << "," <<
            humans.entries[i].Zsigma << "," <<
            humans.entries[i].ROIwidth << "," <<
            humans.entries[i].ROIheight;
        if (write_image_results_)
        {
        	// Write image bounding boxes to file
        	log_file_ << "," <<
        			image_results.rois[i].x << "," <<
        			image_results.rois[i].y << "," <<
        			image_results.rois[i].width << "," <<
        			image_results.rois[i].height;
        }
        log_file_ << std::endl;
      }
    }
  }

  double calculate_angle(cv::Mat& disparity_image)
  {
    // Kinect parameters
    double Q_32 = Q_.at<double>(3,2);
    double Q_03 = Q_.at<double>(0,3);
    double Q_13 = Q_.at<double>(1,3);
    double Q_23 = Q_.at<double>(2,3);
    double Q_33 = Q_.at<double>(3,3);

    // Make yz map
    double yz_map_res = 0.05; // 5 cm resolution
    cv::Mat yz_map_ = cv::Mat::zeros(2.5/yz_map_res, 5.0/yz_map_res, CV_32F);
    // Project disparity points into yz
    for (int i = 0; i < disparity_image.rows; i++)
      for (int j = 0; j < disparity_image.cols; j++)
        if (disparity_image.at<float>(i, j) > 0.1)
        {
          Point3d xyz(j + Q_03, i + Q_13, Q_23);
          double W = Q_32 * disp.at<float>(i, j);
          xyz *= (1.0 / W);
          int yz_row = xyz.y/yz_map_res;
          int yz_col = xyz.z/yz_map_res;
          if((yz_row >= 0)&&(yz_col >= 0)&&(yz_row < yz_map_.rows)&&(yz_col < yz_map_.cols))
          {
            yz_map_.at<float>(yz_row,yz_col) += yz_col;
          }
        }

    // Clean up map
    for(int j = 0; j < yz_map_.cols; j++)
    {
      bool found = false;
      for(int i = yz_map_.rows-1; i >= 0; i--)
      {
        if(found == true)
        {
          yz_map_.at<float>(i,j) = 0;
        }
        else if(yz_map_.at<float>(i,j))
        {
          found = true;
          i--;
        }
      }
    }

    // Do a hough transform to find height, angle
    double angle = 0.0;
    double height = 2.0; // needs to be 6 ft default height
    double best_score = 0.0;
    for(double b = 2.1; b > 1.5; b -= .05) // intercept
    {
      for(double m = -0.5; m < 0.5; m += 0.025) // slope
      {
        double score = 0.0;
        for(int j = 0; j < yz_map_.cols; j++)
        {
          int yz_row = ((j*yz_map_res)*m + b)/yz_map_res;
          int yz_col = j;
          if((yz_row >= 0)&&(yz_col >= 0)&&(yz_row < yz_map_.rows)&&(yz_col < yz_map_.cols))
          {
            score+= yz_map_.at<float>(yz_row,yz_col);
          }
        }
        if(score > best_score)
        {
          angle = std::atan(m) * 180.0/CV_PI;
          height = b;
          best_score = score;
        }
      }
      // ROS_ERROR("Height: %f, score= %f", b, best_score);
    }

    ROS_INFO("Height = %f m, Angle = %f degrees", height, angle);
    if(std::abs(height - 1.84) > 0.3)
    {
      ROS_ERROR("Detected camera height of %f m is sub-optimal.  Please adjust camera height to"
          "1.84 m", height);
    }
    if(std::abs(angle) > 15.0)
    {
      ROS_ERROR("Detected angle is %f degrees.  Please adjust camera to within 15 degrees of "
          "horizontal.", angle);
    }

    return angle;
  }

  ~ObjectTrackingNode()
  {
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ObjectTracking");
  ros::NodeHandle n;
  ObjectTrackingNode HN(n);
  ros::spin();
  return 0;
}
