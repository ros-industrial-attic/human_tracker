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
#include <string>
#include <vector>
#include <sstream>
#include <ros/ros.h>
#include "nodelet/nodelet.h"
#include <std_msgs/String.h>

// Ros package includes
#include <consistency/consistency.hpp>
#include <roi_msgs/overlap.hpp>
#include <roi_msgs/RoiRect.h>
#include <roi_msgs/Rois.h>

//Time Synchronizer
// NOTE: Time Synchronizer conflicts with QT includes may need to investigate
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

using namespace sensor_msgs;
using namespace stereo_msgs;
using namespace message_filters::sync_policies;
using namespace sensor_msgs::image_encodings;
using namespace roi_msgs;
using std::string;


namespace consistency
{

  class consistencyNodelet: public nodelet::Nodelet
  {
  private:
    // Define Node
    ros::NodeHandle node_;
    ros::NodeHandle private_node_;

    // Subscribe to Messages
    message_filters::Subscriber<DisparityImage> sub_disparity_;
    message_filters::Subscriber<Image> sub_image_;
    message_filters::Subscriber<Rois> sub_rois_;

    // Define the Synchronizer
    typedef ApproximateTime<Image, DisparityImage, Rois> ApproximatePolicy;
    typedef ApproximateTime<Image, DisparityImage> ApproximatePolicy2;
    typedef message_filters::Synchronizer<ApproximatePolicy> ApproximateSync;
    typedef message_filters::Synchronizer<ApproximatePolicy2> ApproximateSync2;
    boost::shared_ptr<ApproximateSync> approximate_sync_;
    boost::shared_ptr<ApproximateSync2> approximate_sync2_;

    // Messages to Publish
    ros::Publisher pub_rois_;
    ros::Publisher pub_Color_Image_;
    ros::Publisher pub_Disparity_Image_;

    Rois output_rois_;
    Rois non_overlapping_rois_;
    bool remove_overlapping_rois_;

    //Define the Classifier Object
    Cten::Consistency con_;

    //Mode of operation
    bool useDefaultRoi;
  public:
    virtual void onInit()
    {
      node_ = getNodeHandle();
      private_node_ = getPrivateNodeHandle();

      int qs;
      if(!private_node_.getParam("Q_Size",qs)){
	ROS_ERROR("Q_Size not set");
	qs=3;
      }

      // Published Messages
      pub_rois_           = node_.advertise<Rois>("ConsistencyOutputRois",qs);
      pub_Color_Image_    = node_.advertise<Image>("ConsistencyColorImage",qs);
      pub_Disparity_Image_= node_.advertise<DisparityImage>("ConsistencyDisparityImage",qs);
    
      // Subscribe to Messages
      sub_image_.subscribe(node_,"Color_Image",qs);
      sub_disparity_.subscribe(node_, "Disparity_Image",qs);

      if(!private_node_.getParam("UseDefaultRois",useDefaultRoi)){
	ROS_ERROR("UseDefaultRois param not set");
	useDefaultRoi=false;
      }
      if(useDefaultRoi){ // no roi available, use default
	ROS_ERROR("Using default roi");
	// sync the syncronizer
	approximate_sync2_.reset(new ApproximateSync2(ApproximatePolicy2(qs), 
						      sub_image_, 
						      sub_disparity_));

	// register the 2 element callback
	approximate_sync2_->registerCallback(boost::bind(&consistencyNodelet::imageCb2,
							 this,
							 _1,
							 _2));
      }
      else{
	// subscribe to the rois
	sub_rois_.subscribe(node_,"input_rois",1);

	// Sync the Synchronizer
	approximate_sync_.reset(new ApproximateSync(ApproximatePolicy(4), 
						    sub_image_, 
						    sub_disparity_,
						    sub_rois_));

	// register the 3 element callback
	approximate_sync_->registerCallback(boost::bind(&consistencyNodelet::imageCb,
							this,
							_1,
							_2,
							_3));
      }

      // load the classifier configuration
      if(!private_node_.getParam("yaml_filename",con_.ymlFilename)){
        ROS_ERROR("yaml_filename param not set");
      }
      con_.load();

      // The label of interest is:
      if(!private_node_.getParam("label",con_.label)){
        ROS_ERROR("label param not set");
        con_.label = 1;
      }

      // mode to start with
      if(!private_node_.getParam("mode", con_.mode)){
        ROS_ERROR("mode param not set");
        con_.mode = "load";
      }
      ROS_INFO("Selected mode: %s",con_.mode.c_str());

      // number of data to train with

      if(!private_node_.getParam("max_training_samples", con_.max_training_data)){
        ROS_ERROR("max_training_samples param not set");
        con_.max_training_data = 1000;
      }

      // flag for removing overlapping rois
      if(!private_node_.getParam("RemoveOverlappingRois",remove_overlapping_rois_)){
        ROS_ERROR("couldn't find RemoveOverlappingRois parameter");
        remove_overlapping_rois_ = false;
      }

      if(con_.mode.compare("detect") == 0){
	ROS_INFO("Starting in detection mode");
      }
      else if(con_.mode.compare("train")==0){
	ROS_ERROR("Starting in training mode is non-sensical");
      }
      else if(con_.mode.compare("load")==0){ //mode is accumulate
	ROS_INFO("Starting by loading consistency data");
      }
      else if(con_.mode.compare("accumulate")==0){
	ROS_INFO("Starting accumulation of training data");
      }
      else{
	ROS_ERROR("INVALID STARTING MODE: switching to accumulate");
	con_.mode = "accumulate";
	private_node_.setParam("mode", con_.mode);
      }

    }
    void imageCb2(const ImageConstPtr& image_msg,
		  const DisparityImageConstPtr& disparity_msg)
    {
      //      static int Q=0;
      //      Q++;
      //      if(Q%2 !=0) return;
      
      RoisPtr P(new Rois);

      P->rois.clear();
      P->header.stamp    = image_msg->header.stamp;
      P->header.frame_id = image_msg->header.frame_id;

      RoiRect dR;
      dR.x = 0;
      dR.y = 0;
      dR.height = image_msg->height;
      dR.width  = image_msg->width;
      dR.label  = 0;
      P->rois.push_back(dR);  
      imageCb(image_msg,disparity_msg,P);
    }

    void imageCb(const ImageConstPtr& image_msg,
		 const DisparityImageConstPtr& disparity_msg,
		 const RoisConstPtr& rois_msg)
    {
      string mode;
      if(!private_node_.getParam("mode", mode)){
	ROS_ERROR("mode parameter not set");
	mode = "";
      }
      con_.mode = mode;
      bool kinect_disparity_fix;
      if(!private_node_.getParam("Kinect_Disparity_Fix",kinect_disparity_fix)){
	ROS_ERROR("Kinect_Disparity_Fix not set");
	kinect_disparity_fix = false;
      }
      
      float percent_done=0.0;
      //Use CV Bridge to convert images	
      cv_bridge::CvImagePtr cv_color = cv_bridge::toCvCopy(image_msg,
							   sensor_msgs::image_encodings::BGR8);
    
      cv_bridge::CvImagePtr cv_gray = cv_bridge::cvtColor(cv_color,
							  sensor_msgs::image_encodings::MONO8);
    
    
      assert(disparity_msg->image.encoding == sensor_msgs::image_encodings::TYPE_32FC1);
      cv::Mat_<float> dmat(disparity_msg->image.height,
			   disparity_msg->image.width,
			   (float*) &disparity_msg->image.data[0],
			   disparity_msg->image.step);

      // **********************************************************************//
      // between these comments lies a hack that accounts for the difference   //
      // between the focal lengths of the kinect's color camera and ir cameras //
      // TODO, account for this using proper calibration and projection        //
      if(kinect_disparity_fix){
	int nrows = 434;
	int ncols = 579;
	int row_offset = (dmat.rows - nrows)/2;
	int col_offset = (dmat.cols - ncols)/2;
	cv::Mat Scaled_Disparity(nrows,ncols,CV_32FC1);
	resize(dmat,Scaled_Disparity,cv::Size(ncols,nrows),0,0, CV_INTER_NN );
	for(int i=0;i<dmat.rows;i++){
	  float *rp = &dmat.at<float>(i,0);
	  for(int j=0;j<dmat.cols;j++){
	    *rp = 0.0;
	    rp++;
	  }
	}
	for(int i=0;i<nrows;i++){
	  float *rp = &dmat.at<float>(i+row_offset,col_offset);
	  float *rp2 = &Scaled_Disparity.at<float>(i,0);
	  for(int j=0;j<ncols;j++){
	    *rp = *rp2;
	    rp++;
	    rp2++;
	  }
	}
      }
      // **********************************************************************//

      //load ROS rois.msg data into vector of CvRect and labels for Classifier to use
      RoiRect roi;
      CvRect cvRoi;
      con_.cvRoisInput.clear();
      con_.labelsInput.clear();
      for(unsigned int i = 0;i<rois_msg->rois.size();i++){
	roi = rois_msg->rois[i];
	cvRoi.x = roi.x;
	cvRoi.y = roi.y;
	cvRoi.width = roi.width;
	cvRoi.height = roi.height;			
	con_.cvRoisInput.push_back(cvRoi);
	con_.labelsInput.push_back(roi.label);	
      }

      // pass Color and Disparity Image to consistency object
      con_.color_image=cv_color->image;
      con_.disparity_image=dmat;

      // do the work of the callback
      if(mode.compare("detect") == 0){
	con_.detect();
      }
      else if(mode.compare("train")==0){
	con_.train();
	ROS_ERROR("done training switch to detect");
	con_.mode = "detect";
	private_node_.setParam("mode", con_.mode);
      }
      else if(mode.compare("accumulate")==0){
	con_.accumulate();
	percent_done = (float) con_.TD_.size()/(float)con_.max_training_data  * 100.0;
	if((int)con_.TD_.size()>= con_.max_training_data){
	  con_.mode = "train";
	  private_node_.setParam("mode", con_.mode);
	}
      }
      else if(mode.compare("load")==0){
	ROS_ERROR("consistency LOADING %s",con_.ymlFilename.c_str());
	con_.load();
	con_.mode = "detect";
	private_node_.setParam("mode", con_.mode);
      }
      else{
	ROS_ERROR("INVALID MODE: switching to accumulate");
	con_.mode = "accumulate";
	private_node_.setParam("mode", con_.mode);
      }

      //transfer results to output rois.msg
      output_rois_.rois.clear();
      output_rois_.header.stamp = image_msg->header.stamp;
      output_rois_.header.frame_id = image_msg->header.frame_id;
		
      for(unsigned int j=0;j<con_.cvRoisOutput.size();j++){
	roi.x = con_.cvRoisOutput[j].x;
	roi.y = con_.cvRoisOutput[j].y;
	roi.width = con_.cvRoisOutput[j].width;
	roi.height = con_.cvRoisOutput[j].height;
	roi.label = con_.labelsOutput[j];
	output_rois_.rois.push_back(roi);
      }
		
    if (remove_overlapping_rois_)
    {
      non_overlapping_rois_.rois.clear();
      non_overlapping_rois_.header.stamp = image_msg->header.stamp;
      non_overlapping_rois_.header.frame_id = image_msg->header.frame_id;
	  remove_overlap_Rois(output_rois_, non_overlapping_rois_);
    }
	/*
	non_overlapping_rois_.header.stamp    = image_msg->header.stamp;
	non_overlapping_rois_.header.frame_id = image_msg->header.frame_id;
	int s1 = output_rois_.rois.size();
	int s2 = non_overlapping_rois_.rois.size();
	ROS_ERROR("output_rois_.size = %d non_overlapping size = %d",s1,s2);
      */
      // publish results
      if(mode.compare("accumulate")==0){
	ROS_INFO("ACCUMULATING TRAINING DATA: %5.1f%c done",percent_done,'%');
      }
      else{
	DisparityImage d_msg;
	d_msg = *disparity_msg;
	d_msg.header.stamp = image_msg->header.stamp;
	if (remove_overlapping_rois_)
	{
      ROS_INFO("PUBLISHED: %d Consistency ROIs", int(non_overlapping_rois_.rois.size()));
      pub_rois_.publish(non_overlapping_rois_);
	}
	else
	{
	  ROS_INFO("PUBLISHED: %d Consistency ROIs", int(output_rois_.rois.size()));
	  pub_rois_.publish(output_rois_);
	}
	pub_Color_Image_.publish(image_msg);
	pub_Disparity_Image_.publish(d_msg);
      }

    }
    ~consistencyNodelet()
    {
    }
  };
}// end of namespace consistency

#include <pluginlib/class_list_macros.h>
// PLUGINLIB_DECLARE_CLASS(pkg,class_name,class_type,base_class_type)
PLUGINLIB_DECLARE_CLASS(consistency,consistencyNodelet, consistency::consistencyNodelet, nodelet::Nodelet)

