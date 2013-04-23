/**
 * @file /src/main.cpp
 *
 * @brief Qt based gui.
 *
 * @date November 2010
 **/
/*****************************************************************************
** Includes
*****************************************************************************/
//SEE TIme Synchronizer Note below
//#include <QtGui>
//#include <QApplication>
//#include "../include/consistency/consistency_window.hpp"
//#include "../include/roiPlayer/consistency.hpp"
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
//#include "roiPlayer/RoiRect.h"
//#include "roiPlayer/Rois.h"
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

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/thread.hpp> 


using namespace sensor_msgs;
using namespace stereo_msgs;
using namespace message_filters::sync_policies;
using namespace sensor_msgs::image_encodings;
using namespace roi_msgs;
using namespace cv;

//NOTE: Where is the best place to put these
//typedef ApproximateTime<Image, DisparityImage> ApproximatePolicy;
//typedef message_filters::Synchronizer<ApproximatePolicy> ApproximateSync;

class roiPlayerNode
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

	// Messages to Publish
	ros::Publisher pub_roi_;
	ros::Publisher pub_rois_;
	ros::Publisher pub_image_;
	ros::Publisher pub_disparity_;

	Rois output_rois_;
	Image output_image_;
	DisparityImage output_disparity_;

	// Launch file Parameters
	int imgWidth;
	int imgHeight;

	std::string imgFolderPath;

	double replayRate;
        int    delayStart;
	int getTruFiles(std::string dir, std::vector<std::string> &files){
		DIR *dp;
		struct dirent *dirp;
		if((dp=opendir(dir.c_str())) == NULL){
			return -1;
		}
		while((dirp=readdir(dp))!=NULL){
			//Only add .tru files to the vector
			std::string filename = std::string(dirp->d_name);
			if(filename.find(".tru")!=std::string::npos){
				files.push_back(filename);
			}
		}
		std::sort(files.begin(),files.end());
	
		return 0;
	}

	int getLabeledRois(std::string truFilename, std::vector<RoiRect> &rois){
		std::ifstream f;
		f.open(truFilename.c_str());
		if(f.is_open()){
			std::string line;
			int cnt = 0;
			RoiRect roi;
			while(!f.eof()){
				getline(f,line);
				std::string value = line.substr(line.find(":")+1);
				switch(cnt%6){
		                        case 0:	roi.x =atoi(value.c_str());
                		                break;
                       			case 1: roi.y = atoi(value.c_str());
                                		break;
                        		case 2: roi.width = atoi(value.c_str())-roi.x;
                                		break;
                        		case 3: roi.height = atoi(value.c_str())-roi.y;
                                		break;
                        		case 4: roi.label = atoi(value.c_str());
                                		break;
                        		case 5: //if(line.contains("true")){
                                        	//	randomlyGenerated = true;
                                		//}else{
                                        	//	randomlyGenerated = false;
                                		//}

                                		//Roi roi;
                                		//roi.init(upperLeftX,upperLeftY,lowerRightX,lowerRightY,roiLabel,randomlyGenerated);
                                		//listRoi.append(roi);
						//ROS_INFO("ROI X value: %d",roi.x);
						rois.push_back(roi);
                                		break;
                		}
				cnt++;
			}
			f.close();
			//Test of load
		        //ROS_INFO("ROIS label value: %d",rois[0].label);	
		}
		return 0;
	}

	public:
	
	explicit roiPlayerNode(const ros::NodeHandle& nh):
		node_(nh)
	{

		imgWidth = 0;
		imgHeight =0;
		imgFolderPath = ".";
		replayRate = 1.0;
		delayStart  = 0;

		// Published Messages
    		pub_roi_ = node_.advertise<RoiRect>("output_roi",10);
		pub_rois_= node_.advertise<Rois>("output_rois",10);
		pub_image_=node_.advertise<Image>("output_image",4);
		pub_disparity_=node_.advertise<DisparityImage>("output_disparity",4);
	
                //Read mode from launch file
		std::string mode="";
                node_.param(ros::this_node::getName() + "/mode", mode, std::string("none"));
                ROS_INFO("Selected mode: %s",mode.c_str());

		if(mode.compare("directory")==0)
		{
			std::string truName;
			std::string imgName;
			std::string disName;
                        std::string filename;
			node_.param(ros::this_node::getName()+"/imageFolderPath",imgFolderPath, std::string("."));
			ROS_INFO("Image Folder Path: %s",imgFolderPath.c_str());
			node_.param(ros::this_node::getName()+"/replayRate",replayRate,1.0);
			ROS_INFO("Image Replay Rate: %g",replayRate); 
			node_.param(ros::this_node::getName()+"/delayStart",delayStart,0);
			//Create a vector of .tru file names
			//NOTE: this assumes that each .tru files has a .jpg and .disparity image to publish
 			std::vector<std::string>files=std::vector<std::string>();
			getTruFiles(imgFolderPath,files);			
	
			std::vector<RoiRect>labeledRois=std::vector<RoiRect>();
			
			// wait for a while if desired before publishing
			if(delayStart>0){
			  ROS_INFO("DelayStart: %d",delayStart); 
			  boost::this_thread::sleep(boost::posix_time::seconds(delayStart));
			}

			//For each .tru file
			for(unsigned int i=0; i<files.size();i++){
                       
                                // Example tru filename 1352474617911069665_image.tru
                                // Example img filename 1352474617911069665_image.jpg
                                // Example dis filename 1352474617911069665_disparity.jpg
				
				filename = files[i];
				truName = imgFolderPath +"/" +filename;
				imgName = imgFolderPath +"/" +filename.substr(0,filename.size()-4)+".jpg";
				disName = imgFolderPath +"/" +filename.substr(0,filename.size()-9)+"disparity.jpg";

				ROS_INFO("Roi tru file %s",truName.c_str());
				//Generate labeled roi vector
				labeledRois.clear();
				getLabeledRois(truName, labeledRois);
				//Generate image Msg
				ROS_INFO("Roi img file %s",imgName.c_str());
				cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
				cv_ptr->encoding = "bgr8";
				cv_ptr->image=cv::imread(imgName, CV_LOAD_IMAGE_COLOR);
				cv_ptr->header.stamp=ros::Time::now();
				cv_ptr->header.frame_id=imgName.c_str();
				
				//Generate disparity Msg
				ROS_INFO("Roi disparity file %s",disName.c_str());

                                ///////////////////////////////////////////////////////////////////////////////////////////

				// Copy the data from the file into matrix object CV_8U encoding ////////////
				//cv::Mat dmatrix = cv::Mat::zeros(480,640,CV_8U);
				  cv::Mat dmatrix = imread(disName,CV_LOAD_IMAGE_GRAYSCALE);
				  //				  cv::Mat dmatrix = imread(disName);
				ROS_INFO("dist type = %d CV_8UC3 = %d",dmatrix.type(), CV_8UC3);
				//Test Code for dmatrix
                                //cv::namedWindow("Disparity Image",1);
                                //cv::imshow("Disparity Image",dmatrix);
                                //cv::waitKey(1);

				//Convert to TYPE_32FC1 /////////////////////////////////////////////////////
			
				cv::Mat dmat(dmatrix.rows,dmatrix.cols,CV_32F);
				dmatrix.convertTo(dmat,CV_32F);
				// image dump scaled disparity up by 2 to improve resolution
				dmat = dmat/2.0;

                                //Test Code for dmat
                                //cv::namedWindow("Disparity Image 32FC1",1);
                                //cv::imshow("Disparity Image 32FC1",dmat);
                                //cv::waitKey(1);

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
				//dmatrix.convertTo(disp_image,-1);				
				
				//Generate Rois Msg
		                output_rois_.rois.clear();
                                //ROS_INFO("Size of ROIS: %d",output_rois_.rois.size());

               			output_rois_.header.stamp = cv_ptr->header.stamp;
               			output_rois_.header.frame_id = cv_ptr->header.frame_id;
                		//output_rois_.rois.push_back(roi);
				output_rois_.rois = labeledRois;
				//ROS_INFO("Size of ROIS: %d",output_rois_.rois.size());

				//Publish all three
				ROS_INFO("%s","PUBLISHED: roiPlayer->image_msg");
				pub_image_.publish(cv_ptr->toImageMsg());
				ROS_INFO("%s","PUBLISHED: roiPlayer->disparity_msg");
				pub_disparity_.publish(disp_msg);
                		ROS_INFO("%s","PUBLISHED: roiPlayer->rois_msg");
               			pub_rois_.publish(output_rois_);

				//Replay Rate is the number of messages per second
				ros::Rate loop_rate(replayRate);
				ros::spinOnce();
				loop_rate.sleep();
				
			}
			
		}else if(mode.compare("camera")==0){
			ROS_INFO("MODE: camera");

			//Get the image width and height			
			node_.param(ros::this_node::getName()+"/width",imgWidth,640);
			node_.param(ros::this_node::getName()+"/height",imgHeight,480);

			// Subscribe to Messages
    			sub_image_.subscribe(node_,"camera/rgb/image_color",1);
    			sub_disparity_.subscribe(node_, "camera/depth/disparity",1);
			//sub_rois_.subscribe(node_,"input_rois",1);

    			// Sync the Synchronizer
    			approximate_sync_.reset(new ApproximateSync(ApproximatePolicy(4), 
                        	                sub_image_, 
                                	        sub_disparity_));

    			approximate_sync_->registerCallback(boost::bind(&roiPlayerNode::imageCb,
                                        	                        this,
                                                	                _1,
                                                        	        _2));
		}else{

			ROS_INFO("Unknown mode:%s  Please set to {directory,camera} in roiPlayer.launch",mode.c_str());
		}
	}

	void imageCb(const ImageConstPtr& image_msg,
             	     const DisparityImageConstPtr& disparity_msg){

        	ROS_INFO("roiPlayer Callback called");

		//Use CV Bridge to convert images	
		/*
		cv_bridge::CvImagePtr cv_color = cv_bridge::toCvCopy(image_msg,
							             sensor_msgs::image_encodings::BGR8);
                
		cv_bridge::CvImagePtr cv_gray = cv_bridge::cvtColor(cv_color,
	                                                            sensor_msgs::image_encodings::MONO8);

                IplImage im2color = cv_color->image;
                IplImage im2 = cv_gray->image;		

		assert(disparity_msg->image.encoding == sensor_msgs::image_encodings::TYPE_32FC1);
                const cv::Mat_<float> dmat(disparity_msg->image.height,
                                           disparity_msg->image.width,
                                           (float*) &disparity_msg->image.data[0],
                                           disparity_msg->image.step);

		cv::Mat dmatrix = cv::Mat::zeros(disparity_msg->image.height,
                                                 disparity_msg->image.width,
                                                 CV_8U);
		dmat.convertTo(dmatrix, CV_8U);
                IplImage dispP = dmatrix;
                IplImage* disp = &dispP;

		// Test code to display open cv images
		cv::namedWindow("Color Image",1);
                cv::imshow("Color Image",cv_color->image);
		cv::waitKey(1);
		*/

		// Generate roi for the whole image
                RoiRect roi;
                roi.x = 0;
                roi.y = 0;
                roi.width = imgWidth;
                roi.height = imgHeight;
                ///con_.setRoi(roi.x,roi.y,roi.width,roi.height);
                ///ROS_INFO("%s","PUBLISHED: roiPlayer->roi.msg");
                ///pub_roi_.publish(roi);

		output_rois_.rois.clear();
                output_rois_.header.stamp = image_msg->header.stamp;
		output_rois_.header.frame_id = image_msg->header.frame_id;
		output_rois_.rois.push_back(roi);
		ROS_INFO("%s","PUBLISHED: roiPlayer->rois.msg");
		pub_rois_.publish(output_rois_);

	}
	~roiPlayerNode()
	{
	}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "roiPlayer");
	ros::NodeHandle n;
	roiPlayerNode roiPlayerNode(n);
	ros::spin();
	
	return 0;
}

