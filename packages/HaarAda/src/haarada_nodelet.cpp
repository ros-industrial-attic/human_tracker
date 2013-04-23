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
#include "HaarAda/haarada.hpp"

#include "ros/ros.h"
#include "nodelet/nodelet.h"
#include <sstream>

//Publish Messages
#include <roi_msgs/overlap.hpp>
#include "roi_msgs/RoiRect.h"
#include "roi_msgs/Rois.h"
#include "std_msgs/String.h"

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

namespace HaarAda
{
  class HaarAdaNodelet: public nodelet::Nodelet
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
    //    typedef ApproximateTime<Image, DisparityImage, Rois> ApproximatePolicy;
    typedef ExactTime<Image, DisparityImage, Rois> ApproximatePolicy;
    typedef message_filters::Synchronizer<ApproximatePolicy> ApproximateSync;
    boost::shared_ptr<ApproximateSync> approximate_sync_;

    // Messages to Publish
    ros::Publisher pub_rois_;
    ros::Publisher pub_Color_Image_;
    ros::Publisher pub_Disparity_Image_;


    Rois output_rois_;
    Rois non_overlapping_rois_;
    bool remove_overlapping_rois_;
      
    enum {ACCUMULATE=0, TRAIN, DETECT, EVALUATE, LOAD};

    //Define the Classifier Object
    HaarAda::HaarAdaClassifier HAC_;
    int num_class1;
    int num_class0;
    int num_TP_class1;
    int num_FP_class1;
    int num_TP_class0;
    int num_FP_class0;

  public:
    virtual void onInit()
    {
      node_ = getNodeHandle();
      private_node_ = getPrivateNodeHandle();

      num_class1 = 0;
      num_class0 = 0;
      num_TP_class1 = 0;
      num_FP_class1 = 0;
      num_TP_class0 = 0;
      num_FP_class0 = 0;

      int qs;
      if(!private_node_.getParam("Q_Size",qs)){
	ROS_ERROR("couldn' find Q_Size parameter");
	qs=3;
      }

      int NS;
      if(!private_node_.getParam("num_Training_Samples",NS)){
	NS = 350; // default sets asside very little for training
	private_node_.setParam("num_Training_Samples",NS);
      }
      HAC_.setMaxSamples(NS);

      if(!private_node_.getParam("RemoveOverlappingRois",remove_overlapping_rois_)){
	ROS_ERROR("couldn't find RemoveOverlappingRois parameter");
	remove_overlapping_rois_ = false;
      }

      // Published Messages
      pub_rois_           = node_.advertise<Rois>("HaarAdaOutputRois",qs);
      pub_Color_Image_    = node_.advertise<Image>("HaarAdaColorImage",qs);
      pub_Disparity_Image_= node_.advertise<DisparityImage>("HaarAdaDisparityImage",qs);
	
      // Subscribe to Messages
      sub_image_.subscribe(node_,"Color_Image",qs);
      sub_disparity_.subscribe(node_, "Disparity_Image",qs);
      sub_rois_.subscribe(node_,"input_rois",qs);

      // Sync the Synchronizer
      approximate_sync_.reset(new ApproximateSync(ApproximatePolicy(qs), 
						  sub_image_, 
						  sub_disparity_,
						  sub_rois_));

      approximate_sync_->registerCallback(boost::bind(&HaarAdaNodelet::imageCb,
						      this,
						      _1,
						      _2,
						      _3));
    }

    int get_mode()
    {

      int callback_mode;
      std::string mode;
      if(!private_node_.getParam("mode",mode)){
	ROS_ERROR("couldn't find mode parameter using detect as default");
	mode = "detect";
      }

      if(mode.compare("detect") == 0)
	{
	  callback_mode = DETECT;
	}
      else if(mode.compare("train")==0)
	{
	  callback_mode = TRAIN;
	}
      else if(mode.compare("load")==0)
	{
	  callback_mode = LOAD;
	}
      else if(mode.compare("evaluate")==0)
	{
	  callback_mode = EVALUATE;
	}

      else if(mode.compare("accumulate")==0)
	{
	  callback_mode = ACCUMULATE;
	}
      else // default mode is accumulate
	{
	  ROS_ERROR("in nonthing, going to accumulate");
	  callback_mode = ACCUMULATE;
	}
      return(callback_mode);
    }
    void imageCb(const ImageConstPtr& image_msg,
		 const DisparityImageConstPtr& disparity_msg,
		 const RoisConstPtr& rois_msg){
    
      bool label_all;
      vector<Rect> R_out;
      vector<int> L_out;
      string cfnm;
      int numSamples;
    
    
      //Use CV Bridge to convert images
      CvImagePtr cv_gray  = cv_bridge::toCvCopy(image_msg,image_encodings::MONO8);
      Mat im_gray  = cv_gray->image;		
    
      // take the region of interest message and create vectors of ROIs and labels
      vector<int> L_in;
      vector<Rect> R_in;
      R_in.clear();
      L_in.clear();
      for(unsigned int i=0;i<rois_msg->rois.size();i++){
	int x = rois_msg->rois[i].x;
	int y = rois_msg->rois[i].y;
	int w = rois_msg->rois[i].width;
	int h = rois_msg->rois[i].height;
	int l = rois_msg->rois[i].label;
	Rect R(x,y,w,h);
	R_in.push_back(R);
	L_in.push_back(l);
      }
		
      // get training prior
      double temp;
      if(!private_node_.getParam("HaarAdaPrior",temp)){
	temp = 0.0;
      }
      HAC_.HaarAdaPrior_ = temp;

      // do the work of the node
      switch(get_mode()){
      case DETECT:
	label_all = false;
	HAC_.detect(R_in,L_in,im_gray,R_out,L_out,label_all);
	output_rois_.rois.clear();
	output_rois_.header.stamp = image_msg->header.stamp;
	output_rois_.header.frame_id = image_msg->header.frame_id;
	ROS_INFO("HaarAda found %d objects",(int)L_out.size());
	for(unsigned int i=0; i<R_out.size();i++){
	  RoiRect R;
	  R.x      = R_out[i].x;
	  R.y      = R_out[i].y;
	  R.width  = R_out[i].width;
	  R.height = R_out[i].height;
	  R.label  = L_out[i];
	  output_rois_.rois.push_back(R);
	}

	if(!private_node_.getParam("RemoveOverlappingRois",remove_overlapping_rois_)){
	  remove_overlapping_rois_ = false;
	}
	if(remove_overlapping_rois_){
	  //	ROS_ERROR("removing overlapping rois");
	  remove_overlap_Rois(output_rois_, non_overlapping_rois_);
	  non_overlapping_rois_.header.stamp    = image_msg->header.stamp;
	  non_overlapping_rois_.header.frame_id = image_msg->header.frame_id;
	  pub_rois_.publish(non_overlapping_rois_);
	}
	else{
	  pub_rois_.publish(output_rois_);
	}

	pub_Color_Image_.publish(image_msg);
	pub_Disparity_Image_.publish(disparity_msg);
	break;
      case ACCUMULATE:
	numSamples = HAC_.addToTraining(R_in,L_in,im_gray);
	int NS;
	if(private_node_.getParam("num_Training_Samples",NS)){
	  float percent = (float)HAC_.numSamples_ * 100.0/NS;
	  ROS_INFO("ACCUMULATING: %6.1f%c",percent,'%');
	  if(numSamples >= NS){
	    private_node_.setParam("mode", std::string("train"));
	    ROS_ERROR("DONE Accumulating, switching to train mode");
	  }
	}
	else{
	  ROS_ERROR("num_Training_Samples param not set");
	}
	break;
      case TRAIN:
	if(!private_node_.getParam("classifier_file",cfnm)){
	  ROS_ERROR("classifier_file param not set");
	}
	ROS_ERROR("Submitting %d Samples to Train ouput= %s",HAC_.numSamples_,cfnm.c_str());
	HAC_.train(cfnm);
	private_node_.setParam("mode", std::string("evaluate"));
	ROS_ERROR("DONE TRAINING, switching to evaluate mode");
	break;
      case EVALUATE:
	{
	  if(!HAC_.loaded){
	    if(!private_node_.getParam("classifier_file",cfnm)){
	      ROS_ERROR("classifier_file param not set");
	    }
	    ROS_ERROR("HaarAda LOADING  %s",cfnm.c_str());
	    HAC_.load(cfnm);
	  }
	  label_all = false;
	  HAC_.detect(R_in,L_in,im_gray,R_out,L_out,label_all);
	  int total0_in_list=0;
	  int total1_in_list=0;
	  int fp_in_list=0;
	  int tp_in_list=0;
	
	  // count the input labels
	  for(unsigned int i=0; i<R_in.size();i++){
	    if(L_in[i] == 0 || L_in[i] == -1) total0_in_list++;
	    if(L_in[i] == 1) total1_in_list++;
	  }
	  num_class0 += total0_in_list;
	  num_class1 += total1_in_list;
	
	  // count the output labels which have the correct and incorrect label
	  for(unsigned int i=0; i<R_out.size();i++){
	    if(L_out[i] == 1){
	      tp_in_list++;
	    }
	    else fp_in_list++;
	  }
	  num_TP_class1 += tp_in_list;
	  num_FP_class1 += fp_in_list;
	
	  num_TP_class0 += total0_in_list - fp_in_list;
	  num_FP_class0 += total1_in_list - tp_in_list;
	  float tp1 = (float)num_TP_class1 / (float) num_class1 * 100.0;
	  float tp0 = (float)num_TP_class0 / (float) num_class0 * 100.0;
	  float fp1 = (float)num_FP_class1 / (float) num_class1 * 100.0;
	  float fp0 = (float)num_FP_class0 / (float) num_class0 * 100.0;
	  ROS_ERROR("TP0 = %6.2f FP0 =  %6.2f TP1 = %6.2f FP1 =  %6.2f",tp0,fp0,tp1,fp1);
	}
	break;
      case LOAD:
	if(!private_node_.getParam("classifier_file",cfnm)){
	  ROS_ERROR("classifier_file param not set");
	}
	ROS_ERROR("HaarAda LOADING  %s",cfnm.c_str());
	HAC_.load(cfnm);
	private_node_.setParam("mode", std::string("detect"));
	break;
      }// end switch

    }

    ~HaarAdaNodelet()
    {
    }

  };
}// end of haarada


#include <pluginlib/class_list_macros.h>
// PLUGINLIB_DECLARE_CLASS(pkg,class_name,class_type,base_class_type)
PLUGINLIB_DECLARE_CLASS(HaarAda,haarada_nodelet, HaarAda::HaarAdaNodelet, nodelet::Nodelet)
