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

#include "ros/ros.h" //Used for launch file parameter parsing
#include <vector>
#include <sstream>

//Included for files
#include <iostream>
#include <fstream>
#include "stdio.h"
#include "dirent.h"

//Publish Messages
#include "std_msgs/String.h"

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

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/thread.hpp> 

using namespace sensor_msgs;
using namespace stereo_msgs;
using namespace sensor_msgs::image_encodings;
using namespace cv;

class imagePublisherNode
{
	private:
	// Define Node
	ros::NodeHandle node_;

	// Messages to Publish
	ros::Publisher pub_image_;
	ros::Publisher pub_disparity_;
	ros::Publisher pub_rgb_info_;
	ros::Publisher pub_proj_info_;

    Image output_image_;
	DisparityImage output_disparity_;

	std::string imgFolderPath_;

	double replayRate_;
	int    delayStart_;
	bool images_with_timestamp_;

	// Camera info variables:
	int imgWidth_;
	int imgHeight_;
	float cx_;
	float cy_;
	float baseline_;
	float fixed_focal_length_;

	int getImageFiles(std::string dir, std::vector<std::string> &files)
	{
		DIR *dp;
		struct dirent *dirp;
		if((dp=opendir(dir.c_str())) == NULL)
		{
			return -1;
		}
		while((dirp=readdir(dp))!=NULL)
		{
			//Only add image.jpg files to the vector
			std::string filename = std::string(dirp->d_name);
			if(filename.find("image.jpg")!=std::string::npos)
			{
				files.push_back(filename);
			}
		}
		std::sort(files.begin(),files.end());

		return 0;
	}

	void load(std::string& ymlFilename)
	{
		FileStorage fs(ymlFilename.c_str(),FileStorage::READ);

		fs["imgHeight"]>>imgHeight_;
		fs["imgWidth"]>>imgWidth_;
		fs["cx"]>>cx_;
		fs["cy"]>>cy_;
		fs["baseline"]>>baseline_;
		fs["fixed_focal_length"]>>fixed_focal_length_;
	}

	public:
	
	explicit imagePublisherNode(const ros::NodeHandle& nh):
	node_(nh)
	{
		imgWidth_ = 0;
		imgHeight_ =0;
		imgFolderPath_ = ".";
		replayRate_ = 1.0;
		delayStart_  = 0;
		images_with_timestamp_ = false;

		// Published Messages
		pub_image_=node_.advertise<Image>("/camera/rgb/image_color",4);
		pub_disparity_=node_.advertise<DisparityImage>("/camera/depth/disparity",4);
		pub_rgb_info_=node_.advertise<sensor_msgs::CameraInfo>("/camera/depth_registered/camera_info",4);
		pub_proj_info_=node_.advertise<sensor_msgs::CameraInfo>("/camera/projector/camera_info",4);

		std::string imgName;
		std::string disName;
		std::string filename;
		node_.param(ros::this_node::getName()+"/imageFolderPath",imgFolderPath_, std::string("."));
		ROS_INFO("Image Folder Path: %s",imgFolderPath_.c_str());
		node_.param(ros::this_node::getName()+"/replayRate",replayRate_,1.0);
		ROS_INFO("Image Replay Rate: %g",replayRate_);
		node_.param(ros::this_node::getName()+"/delayStart",delayStart_,0);
		node_.param(ros::this_node::getName()+"/images_with_timestamp",images_with_timestamp_,false);

		//Create a vector of .jpg image file names
		std::vector<std::string>files=std::vector<std::string>();
		getImageFiles(imgFolderPath_,files);

		// Read camera_info parameters from yml file:
		string cameraInfoFilename;
		node_.param(ros::this_node::getName() +"/cameraInfoFilename", cameraInfoFilename , std::string("."));  //default filename for camera info
		load(cameraInfoFilename);

        // Create camera_info messages:
        sensor_msgs::CameraInfoPtr rgb_info_msg(new sensor_msgs::CameraInfo());     // rgb camera info message
        rgb_info_msg->header.frame_id = "/camera_rgb_optical_frame";
        rgb_info_msg->height = imgHeight_;
        rgb_info_msg->width = imgWidth_;
        rgb_info_msg->distortion_model = "plumb_bob";
        rgb_info_msg->D.resize(5,0.0);
        rgb_info_msg->K[0] = fixed_focal_length_;
        rgb_info_msg->K[4] = fixed_focal_length_;
        rgb_info_msg->K[2] = cx_;
        rgb_info_msg->K[5] = cy_;
        rgb_info_msg->K[8] = 1.0;
        rgb_info_msg->P[0] = fixed_focal_length_;
        rgb_info_msg->P[5] = fixed_focal_length_;
        rgb_info_msg->P[2] = cx_;
        rgb_info_msg->P[6] = cy_;
        rgb_info_msg->P[10] = 1.0;
        rgb_info_msg->R[0] = 1.0;
        rgb_info_msg->R[4] = 1.0;
        rgb_info_msg->R[8] = 1.0;

        sensor_msgs::CameraInfoPtr proj_info_msg(new sensor_msgs::CameraInfo());	// projector camera info message
        proj_info_msg->header.frame_id = "/camera_depth_optical_frame";
        proj_info_msg->height = imgHeight_;
        proj_info_msg->width = imgWidth_;
        proj_info_msg->distortion_model = "plumb_bob";
        proj_info_msg->D.resize(5,0.0);
        proj_info_msg->P[0] = 1.0;
        proj_info_msg->P[3] = -baseline_;

        // Display published camera info parameters:
		ROS_ERROR("imgWidth: %d", imgWidth_);
		ROS_ERROR("imgHeight: %d", imgHeight_);
		ROS_ERROR("cx: %f", cx_);
		ROS_ERROR("cy: %f", cy_);
		ROS_ERROR("baseline: %f", baseline_);
		ROS_ERROR("fixed_focal_length: %f", fixed_focal_length_);

		// wait for a while if desired before publishing
		if(delayStart_>0){
			ROS_INFO("DelayStart: %d",delayStart_);
			boost::this_thread::sleep(boost::posix_time::seconds(delayStart_));
		}

		ros::Rate loop_rate(replayRate_);

		//For each image.jpg file
		for(unsigned int i=0; i<files.size();i++)
		{
			// Example tru filename 1352474617911069665_image.tru
			// Example img filename 1352474617911069665_image.jpg
			// Example dis filename 1352474617911069665_disparity.jpg

			filename = files[i];
			imgName = imgFolderPath_ +"/" +filename;
			disName = imgFolderPath_ +"/" +filename.substr(0,filename.size()-9)+"disparity.jpg";

			cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
			cv_ptr->encoding = "bgr8";
			cv_ptr->image=cv::imread(imgName, CV_LOAD_IMAGE_COLOR);
			cv_ptr->header.frame_id=imgName.c_str();
			if (images_with_timestamp_)
			{
			  // Read time stamp from image name:
			  uint32_t sec = std::atoi(filename.substr(0, 10).c_str());
			  uint32_t nsec = std::atoi(filename.substr(10, 9).c_str());
			  cv_ptr->header.stamp=ros::Time(sec, nsec);
			}
			else
		    cv_ptr->header.stamp=ros::Time::now();

			//Generate disparity Msg
			ROS_INFO("Roi disparity file %s",disName.c_str());

			///////////////////////////////////////////////////////////////////////////////////////////

			// Copy the data from the file into matrix object CV_8U encoding ////////////
			cv::Mat dmatrix = imread(disName,CV_LOAD_IMAGE_GRAYSCALE);
			ROS_INFO("dist type = %d CV_8UC3 = %d",dmatrix.type(), CV_8UC3);

			//Convert to TYPE_32FC1 /////////////////////////////////////////////////////

			cv::Mat dmat(dmatrix.rows,dmatrix.cols,CV_32F);
			dmatrix.convertTo(dmat,CV_32F);
			// image dump scaled disparity up by 2 to improve resolution
			dmat = dmat/2.0;

			stereo_msgs::DisparityImagePtr disp_msg=boost::make_shared<stereo_msgs::DisparityImage>();

			disp_msg->header = cv_ptr->header;
			disp_msg->image.header = cv_ptr->header;
			disp_msg->image.height = dmatrix.rows;
			disp_msg->image.width = dmatrix.cols;
			disp_msg->image.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
			disp_msg->image.step=disp_msg->image.width * sizeof(float);
			disp_msg->image.data.resize(disp_msg->image.height*disp_msg->image.step);
			disp_msg->f=570.342;
			disp_msg->T=0.075;
			disp_msg->min_disparity=10.6939;
			disp_msg->max_disparity=85.5513;
			disp_msg->delta_d = 0.125;

			cv::Mat disp_image(disp_msg->image.height,
					disp_msg->image.width,
					CV_32FC1,
					reinterpret_cast<float*>(&disp_msg->image.data[0]),
					disp_msg->image.step);

			dmat.copyTo(disp_image);

			//Publish all messages:
			ROS_INFO("%s","PUBLISHED: imagePublisher->image_msg");
			pub_image_.publish(cv_ptr->toImageMsg());
			ROS_INFO("%s","PUBLISHED: imagePublisher->disparity_msg");
			pub_disparity_.publish(disp_msg);
			ROS_INFO("%s","PUBLISHED: imagePublisher->rgb_camera_info_msg");
			pub_rgb_info_.publish(rgb_info_msg);
			ROS_INFO("%s","PUBLISHED: imagePublisher->projector_camera_info_msg");
			pub_proj_info_.publish(proj_info_msg);

			//Replay Rate is the number of messages per second
			ros::spinOnce();
			loop_rate.sleep();
			
		}
	}
	~imagePublisherNode()
	{
	}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "imagePublisher");
	ros::NodeHandle n;
	imagePublisherNode imagePublisherNode(n);
	ros::spinOnce();

	return 0;
}

