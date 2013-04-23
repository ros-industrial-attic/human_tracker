/**
 * @file /src/neg_example_node.cpp
 *
 * @date Feb. 6, 2012
 **/
/*****************************************************************************
 ** Includes
 *****************************************************************************/
//SEE TIme Synchronizer Note below
#include "ros/ros.h"
#include <sstream>

//Publish Messages
#include <roi_msgs/overlap.hpp>
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

class NegExampleNode
{
private:
  double disparity_scale_factor_; // hard coded to match roiPlayer
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

public:
  string imgFolderPath;
  explicit NegExampleNode(const ros::NodeHandle& nh): node_(nh)
  {
    disparity_scale_factor_ = 2.0; // hard coded to match roiPlayer
    string nn = ros::this_node::getName();
    node_.param(nn+"/imageFolderPath",imgFolderPath, std::string("."));
    int qs;
    if(!node_.getParam(nn + "/Q_Size",qs)){
      qs=3;
    }

    // Subscribe to Messages
    sub_image_.subscribe(node_,"Color_Image",qs);
    sub_disparity_.subscribe(node_, "Disparity_Image",qs);
    sub_rois_.subscribe(node_,"input_rois",qs);

    // Sync the Synchronizer
    approximate_sync_.reset(new ApproximateSync(ApproximatePolicy(qs), 
						sub_image_, 
						sub_disparity_,
						sub_rois_));

    approximate_sync_->registerCallback(boost::bind(&NegExampleNode::imageCb,
						    this,
						    _1,
						    _2,
						    _3));
  }
  void imageCb(const ImageConstPtr& image_msg,
	       const DisparityImageConstPtr& disparity_msg,
	       const RoisConstPtr& rois_msg){

    std::string timestamp;
    std::string imgName;
    std::string disName;
    std::string trueName;
    std::string filename;
    std::stringstream ss;
		
    if(rois_msg->rois.size()>0){

      ss.str("");
      ss<<image_msg->header.stamp.toNSec();
      timestamp  = ss.str();
      imgName    = imgFolderPath +"/" +timestamp+"_image.jpg";
      trueName   = imgFolderPath +"/" +timestamp+"_image.tru";
      disName    = imgFolderPath +"/" +timestamp+"_disparity.jpg";
      
      //Use CV Bridge to convert images then just write them out      
      CvImagePtr cv_color = cv_bridge::toCvCopy(image_msg,image_encodings::BGR8);
      cv::imwrite(imgName.c_str(),cv_color->image);
      
      assert(disparity_msg->image.encoding == image_encodings::TYPE_32FC1);
      cv::Mat_<float> dmat(disparity_msg->image.height,
				 disparity_msg->image.width,
				 (float*) &disparity_msg->image.data[0],
				 disparity_msg->image.step);
      
      cv::Mat dmatrix = cv::Mat::zeros(disparity_msg->image.height,
				       disparity_msg->image.width,
				       CV_8U);
      dmat = disparity_scale_factor_*dmat; // scale to match image_dump/roi_player
      dmat.convertTo(dmatrix, CV_8U);
      cv::imwrite(disName.c_str(),dmatrix);
      
      // write the true file 
      FILE *fp = fopen(trueName.c_str(),"w");
      if(fp != NULL){
	for(unsigned int i=0;i<rois_msg->rois.size();i++){
	  int x = rois_msg->rois[i].x;
	  int y = rois_msg->rois[i].y;
	  int w = rois_msg->rois[i].width;
	  int h = rois_msg->rois[i].height;
	  Rect R(x,y,w,h);
	  fprintf(fp,"Upper Left X: %d\n", x);
	  fprintf(fp,"Upper Left Y: %d\n", y);
	  fprintf(fp,"Lower Right X: %d\n",x+w);
	  fprintf(fp,"Lower Right Y: %d\n",y+h);
	  fprintf(fp,"ROI Label: -1\n");
	  fprintf(fp,"Randomly Generated: false\n");
	}
	fclose(fp);
      }
    }
  }
  ~NegExampleNode(){}
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "NegExample");
  ros::NodeHandle n;
  NegExampleNode NEN(n);
  ros::spin();
  return 0;
}

