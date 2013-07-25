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
#include "nodelet/nodelet.h"
#include <sstream>

//Publish Messages
#include "roi_msgs/RoiRect.h"
#include "roi_msgs/Rois.h"
#include "std_msgs/String.h"

#include <roi_msgs/overlap.hpp>

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

// Used for classifying ROIs
#include <HogSvmPCL/hogsvmpcl.h>

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

namespace HogSvmPCL
{
  class HogSvmPCLNodelet: public nodelet::Nodelet
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
    double max_overlap_;
    int mode_;

    //Define the Classifier Object
    HogSvmPCL::HogSvmPCLClassifier HSC_;

    public:
    virtual void onInit()
    {
      node_ = getNodeHandle();
      private_node_ = getPrivateNodeHandle();

      int qs;
      if(!private_node_.getParam("Q_Size",qs))
      {
        qs=3;
      }

      // flag for removing overlapping rois
      if(!private_node_.getParam("RemoveOverlappingRois",remove_overlapping_rois_)){
    	ROS_ERROR("couldn't find RemoveOverlappingRois parameter");
    	remove_overlapping_rois_ = false;
      }

      // max overlap allowed for rois
      if(!private_node_.getParam("MaxOverlap",max_overlap_)){
    	  ROS_ERROR("couldn't find MaxOverlap parameter");
    	  max_overlap_ = 0.8;
      }

      // Select between whole body (mode = 0) or half-body (mode = 1) classification
      if(!private_node_.getParam("Mode", mode_))
      {
    	  mode_ = 0;
      }

      std::string classifier_filename; // classifier file name
      if(!private_node_.getParam("classifier_file", classifier_filename))
      {
        ROS_ERROR("classifier_file parameter not set");
      }
      HSC_.loadSVMFromFile(classifier_filename);

      // Published Messages
      pub_rois_= node_.advertise<Rois>("HogSvmOutputRois",qs);
      pub_Color_Image_    = node_.advertise<Image>("HogSvmColorImage",qs);
      pub_Disparity_Image_= node_.advertise<DisparityImage>("HogSvmDisparityImage",qs);

      // Subscribe to Messages
      sub_image_.subscribe(node_,"Color_Image",qs);
      sub_disparity_.subscribe(node_, "Disparity_Image",qs);
      sub_rois_.subscribe(node_,"input_rois",qs);

      // Sync the Synchronizer
      approximate_sync_.reset(new ApproximateSync(ApproximatePolicy(qs),
          sub_image_,
          sub_disparity_,
          sub_rois_));

      approximate_sync_->registerCallback(boost::bind(&HogSvmPCLNodelet::imageCb,
          this,
          _1,
          _2,
          _3));
    }

    void classifyROIs (vector<Rect>& R_in, vector<double>& C_in, Mat image, double min_confidence, int mode, vector<Rect>& R_out, vector<double>& C_out)
    {
      R_out.clear();
      C_out.clear();

      for(unsigned int i=0;i<R_in.size();i++)
      {
        // Adapt ROIs to trained SVM for whole body: double height and takes width equal to half of the height
        int xmin = int (R_in[i].x + static_cast<float>(R_in[i].width/2) - (static_cast<float>(R_in[i].height)/2));
        int ymin = R_in[i].y;
        int width = R_in[i].height;
        int height;
        if (mode_ == 0)
        {
          height = R_in[i].height * 2;
        }
        else if (mode_ == 1)
        {
          height = R_in[i].height;
        }

        if (height > 0)
        {
          cv::Rect resized_rect = cv::Rect(xmin, ymin, width, height);
          float confidence = HSC_.evaluate(resized_rect, image);

          if (confidence > min_confidence) // if person found
          { // write current roi in R_out
            R_out.push_back(R_in[i]);
            C_out.push_back(confidence);
          }
        }
      }

//      // To check the bounding boxes which are classified:
//      for(unsigned int i=0;i<R_out.size();i++)
//      {
//        // Adapt ROIs to trained SVM for whole body: double height and takes width equal to half of the height
//        int xmin = int (R_out[i].x + static_cast<float>(R_out[i].width/2) - (static_cast<float>(R_out[i].height)/2));
//        int ymin = R_out[i].y;
//        int width = R_out[i].height;
//        int height = R_out[i].height * 2;
//
//        cv::rectangle(image, cv::Point(xmin, ymin), cv::Point(xmin+width, ymin+height), cv::Scalar(255,0,0), 2);
//      }
//      cv::imshow("image", image);
//      cv::waitKey(0);
    }

    void imageCb(const ImageConstPtr& image_msg,
        const DisparityImageConstPtr& disparity_msg,
        const RoisConstPtr& rois_msg)
    {
      vector<Rect> R_out;
      vector<double> C_out;
      string param_name;

      //Use CV Bridge to convert images
      CvImagePtr cv_rgb  = cv_bridge::toCvCopy(image_msg,image_encodings::BGR8);
      Mat image  = cv_rgb->image;

      // take the region of interest message and create vectors of ROIs and labels
      vector<double> C_in;   // vector of labels
      vector<Rect> R_in;  // vector of ROIs
      R_in.clear();
      C_in.clear();
      // Read ROIs message from topic:
      for(unsigned int i=0;i<rois_msg->rois.size();i++)
      {
        int x = rois_msg->rois[i].x;
        int y = rois_msg->rois[i].y;
        int w = rois_msg->rois[i].width;
        int h = rois_msg->rois[i].height;
        double c = rois_msg->rois[i].confidence;

        Rect R(x,y,w,h);
        R_in.push_back(R);
        C_in.push_back(c);
      }
      // get parameter for controlling point along ROC
      double min_confidence=0.0;
      if(!private_node_.getParam("HogSvmThreshold", min_confidence))
      {
        ROS_ERROR("HogSvmThreshold parameter not set");
      }

      // Classify all input ROIs and save "people" ROIs into R_out and L_out:
      classifyROIs (R_in, C_in, image, min_confidence, mode_, R_out, C_out);

      output_rois_.rois.clear();
      output_rois_.header.stamp = image_msg->header.stamp;
      output_rois_.header.frame_id = image_msg->header.frame_id;
      ROS_INFO("HogSvm found %d objects",(int)C_out.size());
      for(unsigned int i=0; i<R_out.size();i++)
      {
        RoiRect R;
        R.x      = R_out[i].x;
        R.y      = R_out[i].y;
        R.width  = R_out[i].width;
        R.height = R_out[i].height;
        R.label  = 1;
        R.confidence  = C_out[i];
        output_rois_.rois.push_back(R);
      }

      if (remove_overlapping_rois_)
      {
    	non_overlapping_rois_.rois.clear();
    	non_overlapping_rois_.header.stamp = image_msg->header.stamp;
    	non_overlapping_rois_.header.frame_id = image_msg->header.frame_id;
    	remove_overlap_Rois(output_rois_, max_overlap_, non_overlapping_rois_);
    	pub_rois_.publish(non_overlapping_rois_);
      }
      else
      {
    	pub_rois_.publish(output_rois_);
      }

      pub_Color_Image_.publish(image_msg);
      pub_Disparity_Image_.publish(disparity_msg);

    }
    ~HogSvmPCLNodelet()
    {
    }
  };

}// end of HogSvmPCL namespace


#include <pluginlib/class_list_macros.h>
// PLUGINLIB_DECLARE_CLASS(pkg,class_name,class_type,base_class_type)
PLUGINLIB_DECLARE_CLASS(HogSvmPCL,hogsvmpcl_nodelet, HogSvmPCL::HogSvmPCLNodelet, nodelet::Nodelet)
