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
//Used for launch file parameter parsing
#include <string>
//Used for rois message vector
#include <vector>
//#include "std_msgs/String.h"
#include <sstream>

//Included for files
#include <iostream>
#include <fstream>
#include "stdio.h"
#include "dirent.h"

//Publish Messages
//None

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

using namespace sensor_msgs;
using namespace stereo_msgs;
using namespace message_filters::sync_policies;
using namespace sensor_msgs::image_encodings;
using namespace cv;

//NOTE: Where is the best place to put these
//typedef ApproximateTime<Image, DisparityImage> ApproximatePolicy;
//typedef message_filters::Synchronizer<ApproximatePolicy> ApproximateSync;

class image_dumpNode
{
        private:
        // Define Node
        ros::NodeHandle node_;

        // Subscribe to Messages
        message_filters::Subscriber<DisparityImage> sub_disparity_;
        message_filters::Subscriber<Image> sub_image_;
        //message_filters::Subscriber<consistency::Rois> sub_rois_;

        // Define the Synchronizer
        typedef ApproximateTime<Image, DisparityImage> ApproximatePolicy;
        typedef message_filters::Synchronizer<ApproximatePolicy> ApproximateSync;
        boost::shared_ptr<ApproximateSync> approximate_sync_;

        Image output_image_;
        DisparityImage output_disparity_;

        // Launch file Parameters
        std::string imgFolderPath;
        int sampleRate;
        double disparity_scale_factor_;
	int cnt;

    public:

        explicit image_dumpNode(const ros::NodeHandle& nh):
                node_(nh)
        {

                //Read mode from launch file
                std::string mode="";
		std::string nn = ros::this_node::getName();
                node_.param(nn + "/mode", mode, std::string("none"));
		disparity_scale_factor_ = 2.0; // hard coded to match roiPlayer
		//                node_.param(nn + "/Disparity_Scale_Factor", disparity_scale_factor_, 1.0);
                ROS_INFO("Selected mode: %s Disparity Scale Factor = %f",mode.c_str(),disparity_scale_factor_);
                if(mode.compare("directory")==0){
                        ROS_INFO("MODE: directory");
	
			imgFolderPath = ".";
			sampleRate = 10;
			cnt=0;
			
                        node_.param(ros::this_node::getName()+"/imageFolderPath",imgFolderPath, std::string("."));
                        ROS_INFO("Image Folder Path: %s",imgFolderPath.c_str());
                        node_.param(ros::this_node::getName()+"/sampleRate",sampleRate,10);
                        ROS_INFO("Image Sample Rate: %d",sampleRate);

                        // Subscribe to Messages
                        sub_image_.subscribe(node_,"camera/rgb/image_color",1);
                        sub_disparity_.subscribe(node_, "camera/depth/disparity",1);

                        // Sync the Synchronizer
                        approximate_sync_.reset(new ApproximateSync(ApproximatePolicy(4),
                                                sub_image_,
                                                sub_disparity_));

                        approximate_sync_->registerCallback(boost::bind(&image_dumpNode::imageCb,
                                                                        this,
                                                                        _1,
                                                                        _2));
                }else{

                        ROS_INFO("Unknown mode:%s  Please set to {directory} in image_dump.launch",mode.c_str());
                }
        }

        void imageCb(const ImageConstPtr& image_msg,
                     const DisparityImageConstPtr& disparity_msg){

                ROS_INFO("image_dump Callback called");

		std::string timestamp;
		std::string frame_id;
                std::string imgName;
                std::string disName;
                std::string filename;
		std::stringstream ss;

		if((cnt%sampleRate)==0)
		{
        	        ROS_INFO("Sample image and disparity saved");
			//output_rois_.header.frame_id = image_msg->header.frame_id;
                        ss.str("");
		        ss<<image_msg->header.stamp.toNSec();
			timestamp = ss.str();
			frame_id = image_msg->header.frame_id;
                        imgName = imgFolderPath +"/" +timestamp+"_image.jpg";
			ROS_INFO("Image file name    : %s",imgName.c_str());

			//Use same timestamp to name the images
			//ss.str("");
			//ss<<disparity_msg->header.stamp.toNSec();
			//timestamp =ss.str();
			frame_id=disparity_msg->header.frame_id;
                        disName = imgFolderPath +"/" +timestamp+"_disparity.jpg";
		        ROS_INFO("Disparity file name: %s",disName.c_str());

                	//Use CV Bridge to convert images       
                
               		cv_bridge::CvImagePtr cv_color = cv_bridge::toCvCopy(image_msg,
                                                                     sensor_msgs::image_encodings::BGR8);
                
                	
			cv::imwrite(imgName.c_str(),cv_color->image);

                	assert(disparity_msg->image.encoding == sensor_msgs::image_encodings::TYPE_32FC1);
                	cv::Mat_<float> dmat(disparity_msg->image.height,
                                           disparity_msg->image.width,
                                           (float*) &disparity_msg->image.data[0],
                                           disparity_msg->image.step);
			// scale disparity by 2 to improve resolution when converting to integer data
			// Player should read, convert to floats, then divide by 2
			dmat = disparity_scale_factor_*dmat; 
			
			cv::Mat dmatrix = cv::Mat::zeros(disparity_msg->image.height,
                                                 disparity_msg->image.width,
                                                 CV_8U);
                	dmat.convertTo(dmatrix, CV_8U);
			cv::imwrite(disName.c_str(),dmatrix);

			//Print disparity image settings
                        //NOTE: These values are used by roiPlayer to format the disparity image from a jpg
			int h =disparity_msg->image.height;
                        int w =disparity_msg->image.width;
                        std::string e =disparity_msg->image.encoding;
                        int s =disparity_msg->image.step;
                        //disparity_msg.image.data.resize(output_disparity_.image.height * output_disparity_.image.step);
                        float f =disparity_msg->f;
                        float t =disparity_msg->T;
                        float mind=disparity_msg->min_disparity;
                        float maxd=disparity_msg->max_disparity;
		        float dd = disparity_msg->delta_d;
			
			ss.str("");
                        ss<<'\n';
	                ss<<"image.height: "<<h<<'\n';
                        ss<<"image.width: "<<w<<'\n';
			ss<<"image.encoding: "<<e<<'\n';
			ss<<"image.step: "<<s<<'\n';
                        ss<<"f: "<<f<<'\n';
                        ss<<"T: "<<t<<'\n';
                        ss<<"min_disparity: "<<mind<<'\n';
	                ss<<"max_disparity: "<<maxd<<'\n';
			ss<<"delta_d: "<<dd<<'\n';
                      	std::string dvalues = ss.str();
			ROS_INFO("Disparity Image values: %s",dvalues.c_str());

			/*
                	// Test code to display open cv images
                	cv::namedWindow("Color Image",1);
                	cv::imshow("Color Image",cv_color->image);
                	cv::waitKey(1);
			*/
		}
		cnt++;
        }
        ~image_dumpNode()
        {
        }
};

int main(int argc, char **argv)
{
        ros::init(argc, argv, "image_dump");
        ros::NodeHandle n;
        image_dumpNode image_dumpNode(n);
        ros::spin();

        return 0;
}

