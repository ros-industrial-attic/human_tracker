/**
 * @file /src/HaarSvm.cpp
 *
 * @brief Qt based gui.
 *
 * @date November 2012
 **/
/*****************************************************************************
 ** Includes
 *****************************************************************************/
//SEE TIme Synchronizer Note below
//#include <QtGui>
//#include <QApplication>

#include "HaarSvm/haarsvm.hpp"

#include "ros/ros.h"
#include <sstream>

//Publish Messages
#include "roi_msgs/RoiRect.h"
#include "roi_msgs/Rois.h"
#include "std_msgs/String.h"

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

/*
  using std::vector;
  using std::string;
  using cv::Rect;
  using cv::Mat;
*/
class HaarSvmNode
{
private:
  // Define Node
  ros::NodeHandle node_;

  // Subscribe to Messages
  message_filters::Subscriber<DisparityImage> sub_disparity_;
  message_filters::Subscriber<Image> sub_image_;
  message_filters::Subscriber<Rois> sub_rois_;

  // Define the Synchronizer
  typedef ApproximateTime<Image, DisparityImage, Rois> ApproximatePolicy;
  typedef message_filters::Synchronizer<ApproximatePolicy> ApproximateSync;
  boost::shared_ptr<ApproximateSync> approximate_sync_;

  // Messages to Publish
  ros::Publisher pub_rois_;
  ros::Publisher pub_Color_Image_;
  ros::Publisher pub_Disparity_Image_;

  Rois output_rois_;
      
  enum {ACCUMULATE=0, TRAIN, DETECT, EVALUATE, LOAD};

  //Define the Classifier Object
  HaarSvm::HaarSvmClassifier HSC_;
  int num_class1;
  int num_class0;
  int num_TP_class1;
  int num_FP_class1;
  int num_TP_class0;
  int num_FP_class0;

public:
	
  explicit HaarSvmNode(const ros::NodeHandle& nh):
    node_(nh)
  {
    num_class1 = 0;
    num_class0 = 0;
    num_TP_class1 = 0;
    num_FP_class1 = 0;
    num_TP_class0 = 0;
    num_FP_class0 = 0;

    string nn = ros::this_node::getName();
    int qs;
    if(!node_.getParam(nn + "/Q_Size",qs)){
      qs=3;
    }
    
    int NS;
    if(!node_.getParam(nn + "/num_Training_Samples",NS)){
      NS = 350;
      node_.setParam(nn + "/num_Training_Samples",NS);
    }
    HSC_.setMaxSamples(NS);

    // Published Messages
    pub_rois_= node_.advertise<Rois>("HaarSvmOutputRois",qs);
    pub_Color_Image_    = node_.advertise<Image>("HaarSvmColorImage",qs);
    pub_Disparity_Image_= node_.advertise<DisparityImage>("HaarSvmDisparityImage",qs);

	
    // Subscribe to Messages
    sub_image_.subscribe(node_,"Color_Image",qs);
    sub_disparity_.subscribe(node_, "Disparity_Image",qs);
    sub_rois_.subscribe(node_,"input_rois",qs);

    // Sync the Synchronizer
    approximate_sync_.reset(new ApproximateSync(ApproximatePolicy(qs), 
						sub_image_, 
						sub_disparity_,
						sub_rois_));

    approximate_sync_->registerCallback(boost::bind(&HaarSvmNode::imageCb,
						    this,
						    _1,
						    _2,
						    _3));
  }
  int get_mode()
  {

    int callback_mode;
    std::string mode="";
    node_.param(ros::this_node::getName() + "/mode", mode, std::string("none"));
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
    string param_name;
    string nn = ros::this_node::getName();
    string cfnm;
    int numSamples;

    //Use CV Bridge to convert image
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

    // get parameter for controlling point along ROC 
    param_name = nn + "/HaarSvmThreshold";
    double temp=0.0;
    node_.getParam(param_name,temp);
    HSC_.HaarSvmThreshold_ = (float)temp;
    //    ROS_ERROR("HaarSvm Threshold = %f",HSC_.HaarSvmThreshold_);

    // do the work of the node
    switch(get_mode()){
    case DETECT:
      label_all = false;
      HSC_.detect(R_in,L_in,im_gray,R_out,L_out,label_all);
      output_rois_.rois.clear();
      output_rois_.header.stamp = image_msg->header.stamp;
      output_rois_.header.frame_id = image_msg->header.frame_id;
      ROS_INFO("HaarSvm found %d objects",(int)L_out.size());
      for(unsigned int i=0; i<R_out.size();i++){
	RoiRect R;
	R.x      = R_out[i].x;
	R.y      = R_out[i].y;
	R.width  = R_out[i].width;
	R.height = R_out[i].height;
	R.label  = L_out[i];
	output_rois_.rois.push_back(R);
      }
      pub_rois_.publish(output_rois_);
      pub_Color_Image_.publish(image_msg);
      pub_Disparity_Image_.publish(disparity_msg);
      break;
    case ACCUMULATE:
      numSamples = HSC_.addToTraining(R_in,L_in,im_gray);
      param_name = nn + "/num_Training_Samples";
      int NS;
      if(node_.getParam(param_name,NS)){
	float percent = (float)HSC_.numSamples_ * 100.0/NS;
	ROS_INFO("ACCUMULATING: %6.1f%c",percent,'%');
	if(numSamples >= NS){
	  param_name = nn + "/mode";
	  node_.setParam(param_name, std::string("train"));
	  ROS_ERROR("DONE Accumulating, switching to train mode");
	}
      }
      break;
    case TRAIN:
      param_name = nn + "/classifier_file";
      node_.param(param_name,cfnm,std::string("/home/clewis/classifiers/test.xml"));
      ROS_ERROR("Submitting %d Samples to Train ouput= %s",HSC_.numSamples_,cfnm.c_str());
      HSC_.train(cfnm);
      param_name = nn + "/mode";
      node_.setParam(param_name, std::string("evaluate"));
      ROS_ERROR("DONE TRAINING, switching to evaluate mode");
      break;
    case EVALUATE:
      {
	if(!HSC_.loaded){
	  param_name = nn + "/classifier_file";
	  node_.param(param_name,cfnm,std::string("test.xml"));
	  ROS_ERROR("HaarSvm LOADING  %s",cfnm.c_str());
	  HSC_.load(cfnm);
	}
	label_all = false;
	HSC_.detect(R_in,L_in,im_gray,R_out,L_out,label_all);
	
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
      param_name = nn + "/classifier_file";
      node_.param(param_name,cfnm,std::string("test.xml"));
      ROS_ERROR("HaarSvm LOADING  %s",cfnm.c_str());
      HSC_.load(cfnm);
      param_name = nn + "/mode";
      node_.setParam(param_name, std::string("detect"));
      break;
    }// end switch

  }
  ~HaarSvmNode()
  {
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "HaarSvm");
  ros::NodeHandle n;
  HaarSvmNode HN(n);
  ros::spin();
  return 0;
}

