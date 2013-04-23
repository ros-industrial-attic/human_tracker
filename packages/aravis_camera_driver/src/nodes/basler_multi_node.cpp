// *******************************************************************************
//
// Copyright (C) 2011 All Right Reserved, Southwest Research Institute® (SwRI®)
//
// Contract No.  10-62987
// Contractor    Southwest Research Institute® (SwRI®)
// Address       6220 Culebra Road, San Antonio, Texas 78228-0510
// Contact       Kris Kozak <kkozak@swri.org> (210) 522-3854
//
// This code was developed as part of an internal research project fully funded
// by Southwest Research Institute®.
//
// THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
// KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
// PARTICULAR PURPOSE.
//
// *******************************************************************************

#include <algorithm>
#include <vector>
#include <map>
#include <fstream>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <aravis_camera_driver/basler_camera.h>
#include <image_transport/image_transport.h>
#include <yaml-cpp/yaml.h>
#include <boost/thread.hpp>
#include <boost/filesystem.hpp>
#include <camera_calibration_parsers/parse.h>
#include <camera_info_manager/camera_info_manager.h>
using camera_info_manager::CameraInfoManager;
using boost::shared_ptr;
struct BaslerCameraConfig
{
  std::string camera_id;
  aravis_camera_driver::ExposureMode exposure_mode;
  aravis_camera_driver::GainMode gain_mode;
  std::string capture_mode;
  ArvPixelFormat pixel_format;
  int exposure;
  int gain;
  int digital_shift;
  int frame_rate;
  int mtu;
  int bin_x;
  int bin_y;
  int roi_x;
  int roi_y;
  int roi_width;
  int roi_height;
  bool auto_white_balance_once;
  
  std::string AcquisitionStartTrigMode;
  std::string AcquisitionStartTrigSource;
  std::string AcquisitionStartTrigActivation;
  std::string FrameStartTrigMode;
  std::string FrameStartTrigSource;
  std::string FrameStartTrigActivation;

  std::string trigger_control_implementation;
  std::string line_selector;
  std::string line_mode;
  std::string line_source;
  std::string line_format;
  std::string line_output_selector;
  std::string user_output_selector;

  std::string info_filename;
};

class BaslerMultiNode
{
private:
  ros::NodeHandle node_;
  std::vector<BaslerCameraConfig> configs_;
  std::map<std::string, aravis_camera_driver::BaslerCamera> cameras_;
  std::map<std::string, ros::Publisher> color_image_pubs_;
  std::map<std::string, ros::Publisher> mono_image_pubs_;
  std::map<std::string, ros::Publisher> info_pubs_;
  std::map<std::string, sensor_msgs::CameraInfo> infos_;
  std::map<std::string, shared_ptr<camera_info_manager::CameraInfoManager> > info_mgrs_;
  ros::Timer timer_;
  image_transport::ImageTransport it_;
  int mtu_;
  bool software_synchronization;
  float software_sync_period;
  bool camera1_image_received;
  bool camera2_image_received;
  ros::Time trigger_time;

  std::string nn;
  std::string camera_ini_dir;
  std::string frame_id_;

public:

  BaslerMultiNode(ros::NodeHandle &nh) :
    node_(nh), it_(nh)
  {
    std::string configFile;
    nn = ros::this_node::getName();
    node_.param(nn + "/mtu", mtu_, 1500);
    node_.param(nn + "/config_file", configFile, std::string("./cameras.conf"));
    node_.param(nn + "/camera_ini_dir", camera_ini_dir, std::string("./ini"));
    node_.param(nn + "/Frame_ID", frame_id_, std::string("Stereo_Frame"));
    
    if (!ReadCameras(configFile))
    {
      ros::requestShutdown();
    }

    InitializeCameras();
  }

  void InitializeCameras()
  {
    for (unsigned int i = 0; i < configs_.size(); i++)
    {
      ROS_INFO("Initializing camera: %s", configs_[i].camera_id.c_str());
      std::string topic_base = "/";
      std::string id = configs_[i].camera_id;
      std::replace(id.begin(), id.end(), '-', '_' );
      topic_base += id;
      std::string image_topic = topic_base + "/image_raw";
      std::string info_topic  = topic_base + "/camera_info";

      color_image_pubs_[configs_[i].camera_id] = node_.advertise<sensor_msgs::Image>(image_topic, 1);
      info_pubs_[configs_[i].camera_id] = node_.advertise<sensor_msgs::CameraInfo>(info_topic, 1);

      aravis_camera_driver::BaslerCamera& camera = cameras_[configs_[i].camera_id];

      // TODO callback should not be set until later otherwise, uninitialized structures may exist
      camera.SetFrameCallback(&BaslerMultiNode::HandleFrame, this);
      camera.SetPacketSize(mtu_);
      camera.SetPixelFormat(configs_[i].pixel_format);

      // do the configs that must occur prior to camera.initialize()
      if (configs_[i].bin_x != -1 && configs_[i].bin_y != -1)
      {
        camera.SetBinning(configs_[i].bin_x, configs_[i].bin_y);
      }

      if (configs_[i].roi_x != -1 && configs_[i].roi_y != -1 && configs_[i].roi_width != -1 && configs_[i].roi_height != -1)
      {
        camera.SetRegion(configs_[i].roi_x, configs_[i].roi_y, configs_[i].roi_width, configs_[i].roi_height);
      }


      // initialize the camera using  camera.initialize(), then set camera parameters that need initialization first
      if (camera.Initialize(configs_[i].camera_id))
      {
        if (configs_[i].frame_rate != -1)
        {
	  ROS_ERROR("Setting Frame Rate to %d",configs_[i].frame_rate);
          camera.SetFrameRate(configs_[i].frame_rate);
        }

	ROS_ERROR("Setting Exposure %d",configs_[i].exposure);
        if (configs_[i].exposure_mode == aravis_camera_driver::CAM_EXPOSURE_MANUAL)
        {
          if (configs_[i].exposure != -1)
          {
            camera.SetExposure(configs_[i].exposure_mode, configs_[i].exposure);
          }
        }
        else if (configs_[i].exposure_mode != -1)
        {
          camera.SetExposure(configs_[i].exposure_mode);
        }

	ROS_ERROR("Setting Camera Gain to %d",configs_[i].gain);

        if (configs_[i].gain_mode == aravis_camera_driver::CAM_GAIN_MANUAL)
        {
          if (configs_[i].gain != -1)
          {
            camera.SetGain(configs_[i].gain_mode, configs_[i].gain);
          }
        }
        else if (configs_[i].gain_mode != -1)
        {
          camera.SetGain(configs_[i].gain_mode);
        }
	
	ROS_ERROR("Setting Digital Shift to %d",configs_[i].digital_shift);
        if (configs_[i].digital_shift != -1)
        {
          camera.SetDigitalShift(configs_[i].digital_shift);
        }
        
        if (configs_[i].auto_white_balance_once == true)
        {
	  ROS_ERROR("Setting auto white balance once");
           camera.SetAutoWhiteBalanceOnce(configs_[i].auto_white_balance_once);
        }       
       
	// configure the Acquisition Trigger
	camera.SetTriggerSelector("AcquisitionStart");
	ROS_ERROR("acquisition start trigger mode = %s",configs_[i].AcquisitionStartTrigMode.c_str());
	camera.SetTriggerMode(configs_[i].AcquisitionStartTrigMode);
	camera.SetTriggerSource(configs_[i].AcquisitionStartTrigSource);
	camera.SetTriggerActivation(configs_[i].AcquisitionStartTrigActivation);
	
	// configure the Frame Trigger
	camera.SetTriggerSelector("FrameStart");
	camera.SetTriggerMode(configs_[i].FrameStartTrigMode);
	camera.SetTriggerSource(configs_[i].FrameStartTrigSource);
	camera.SetTriggerActivation(configs_[i].FrameStartTrigActivation);
	
	// configure the io lines
	camera.SetLineSelector(configs_[i].line_selector);      
	camera.SetLineMode(configs_[i].line_mode);
	camera.SetLineSource(configs_[i].line_source);
	
	// TODO remove camera_calibration_parsers code and use CameraInfoManager only
	//Load the camera info file
	std::string info_fn = camera_ini_dir + "/" + configs_[i].info_filename;
	if (boost::filesystem::exists(info_fn.c_str())){
	  try{
	    std::string camera_name;
	    std::string id = configs_[i].camera_id;
	    if(!camera_calibration_parsers::readCalibration(info_fn, camera_name, infos_[id])){
	      ROS_WARN("Unable to read camera info.");
	      ROS_ASSERT(false);
	    }
	    std::string info_url = "file://" + info_fn;
	    shared_ptr<CameraInfoManager> cim(new CameraInfoManager(node_,camera_name,info_url));

	    info_mgrs_[id] = cim;
	    if(!info_mgrs_[id]->loadCameraInfo(info_url)){
	      ROS_ERROR("Could not load camera info from %s",info_url.c_str());
	    }
	  }// end try
	  catch(...){
	    ROS_WARN("Exception reading camera info.");
	    ROS_ASSERT(false);
	  }//end catch
	}
	else{// info file does not exists
	  ROS_ERROR("Camera info file does not exist: %s", info_fn.c_str());
	  ROS_ASSERT(false);
	}
      }	// end if camera initializes ok
    }	// end for each camera in configs

    // the timer callback issues a frame trigger to both cameras allowing software synchronization
    // this asumes that the camera is set up correctly.
    // Lots of different trigger configurations are possible. However, at least one of the two triggers the cameras must be
    // set to ON, not OFF, these two triggers are
    // 1. acquisition start trigger
    // 2. frame start trigger
    // Each trigger has a mode bit, either On or OFF.
    // When both triggers are in "On" mode, then both triggers must receive a trigger event to capture of an image
    // When only one trigger is in the "On" mode, then only that trigger needs to receive a trigger to capture an image
    // Each trigger can be connected to and tripped by: Software, or by Line1
    // With both on, one could be software, while the other is hardware
    // For software synchronization we require:
    // acquisition_start_trigger mode: OFF
    // frame_start_trigger mode: ON
    // frame_start_trigger source: Software
    
    if(software_synchronization){
      camera1_image_received = true;
      camera2_image_received = true;
      timer_ =   node_.createTimer( ros::Duration(software_sync_period),&BaslerMultiNode::TimerCallback, this);
    }
  }
  void TimerCallback(const ros::TimerEvent&)
  {
    int q=0;
    // wait until the images are received
    while((!camera1_image_received || !camera2_image_received) && q<10){
      q++;
    }
    if(q>0){
      if(camera1_image_received & !camera2_image_received){
	ROS_INFO("Only Camera1 Received an image.. Retrigger");
      }
      else if(camera1_image_received & !camera2_image_received){
	ROS_INFO("Only Camera2 Received an image.. Retrigger");
      }
      else if(camera1_image_received & camera2_image_received){
	ROS_INFO("Late");
      }
      else if(!camera1_image_received & !camera2_image_received){
	ROS_INFO("Neither Camera Received an image.. Retrigger");
      }
    }

    std::string c1_id = configs_[0].camera_id;
    std::string c2_id = configs_[1].camera_id;
    
    aravis_camera_driver::BaslerCamera& camera1 = cameras_[c1_id];
    aravis_camera_driver::BaslerCamera& camera2 = cameras_[c2_id];
    camera1.SetTriggerSelector("FrameStart"); // select the trigger
    camera2.SetTriggerSelector("FrameStart"); // select the trigger

    camera1_image_received = false;
    camera2_image_received = false;
    trigger_time = ros::Time::now();
    camera1.TriggerExecute(); // software trigger it
    camera2.TriggerExecute(); // software trigger it
  }


  void HandleFrame(ArvBuffer* buffer, const std::string& camera_id)
  {
    sensor_msgs::Image image;


    std::string c1_id = configs_[0].camera_id;
    std::string c2_id = configs_[1].camera_id;
    
    if (cameras_.count(camera_id) > 0 && info_mgrs_.count(camera_id) > 0)
    {
      sensor_msgs::CameraInfo info(info_mgrs_[camera_id]->getCameraInfo());

      aravis_camera_driver::BaslerCamera& camera = cameras_[camera_id];
      image.width = camera.Width();
      image.height = camera.Height();
      image.header.frame_id = frame_id_;
      image.header.stamp = ros::Time::now();

      if (buffer->pixel_format == ARV_PIXEL_FORMAT_MONO_8)
      {
        image.encoding = sensor_msgs::image_encodings::MONO8;
      }
      else if (buffer->pixel_format == ARV_PIXEL_FORMAT_MONO_16)
      {
        image.encoding = sensor_msgs::image_encodings::MONO16;
      }
      else if (buffer->pixel_format == ARV_PIXEL_FORMAT_BAYER_BG_8)
      {
        image.encoding = sensor_msgs::image_encodings::BAYER_BGGR8;
      }
      else if (buffer->pixel_format == ARV_PIXEL_FORMAT_BAYER_GB_8)
      {
        image.encoding = sensor_msgs::image_encodings::BAYER_GBRG8;
      }
      else if (buffer->pixel_format == ARV_PIXEL_FORMAT_BAYER_GR_8)
      {
        image.encoding = sensor_msgs::image_encodings::BAYER_GRBG8;
      }
      else if (buffer->pixel_format == ARV_PIXEL_FORMAT_BAYER_RG_8)
      {
        image.encoding = sensor_msgs::image_encodings::BAYER_RGGB8;
      }
      else if (buffer->pixel_format == ARV_PIXEL_FORMAT_BGR_8_PACKED)
      {
        image.encoding = sensor_msgs::image_encodings::BGR8;
      }
      else if (buffer->pixel_format == ARV_PIXEL_FORMAT_BGRA_8_PACKED)
      {
        image.encoding = sensor_msgs::image_encodings::BGRA8;
      }
      else if (buffer->pixel_format == ARV_PIXEL_FORMAT_RGBA_8_PACKED)
      {
        image.encoding = sensor_msgs::image_encodings::RGBA8;
      }
      else if (buffer->pixel_format == ARV_PIXEL_FORMAT_RGB_8_PACKED)
      {
        image.encoding = sensor_msgs::image_encodings::RGB8;
      }
      else
      {
        ROS_WARN("Unsupported pixel format: %d", buffer->pixel_format);
        return;
      }

      image.data.resize(buffer->size);
      image.data.assign(static_cast<uint8_t*>(buffer->data),static_cast<uint8_t*>(buffer->data) + buffer->size);
      image.header.stamp = trigger_time;
      info.header = image.header;
      color_image_pubs_[camera_id].publish(image);
      info_pubs_[camera_id].publish(info);
      if(camera_id == c1_id){
	camera1_image_received = true;
      }
      if(camera_id == c2_id){
	camera2_image_received = true;
      }
    }
    else
    {
      ROS_WARN("Unknown camera: %s", camera_id.c_str());
    }
  }

  static aravis_camera_driver::ExposureMode ParseExposureMode(const std::string& mode)
  {
    aravis_camera_driver::ExposureMode exposureMode = aravis_camera_driver::CAM_EXPOSURE_AUTO;

    if (mode == "fixed")
    {
      exposureMode = aravis_camera_driver::CAM_EXPOSURE_MANUAL;
    }
    else if (mode == "auto_once")
    {
      exposureMode = aravis_camera_driver::CAM_EXPOSURE_AUTO_ONCE;
    }

    return exposureMode;
  }

  static aravis_camera_driver::GainMode ParseGainMode(const std::string& mode)
  {
    aravis_camera_driver::GainMode gainMode = aravis_camera_driver::CAM_GAIN_AUTO;

    if (mode == "fixed")
    {
      gainMode = aravis_camera_driver::CAM_GAIN_MANUAL;
    }
    else if (mode == "auto_once")
    {
      gainMode = aravis_camera_driver::CAM_GAIN_AUTO_ONCE;
    }

    return gainMode;
  }

  bool ReadCameras(const std::string& filepath)
  {
    std::ifstream fin(filepath.c_str());
    if (fin.fail())
      {
	ROS_ERROR("Failed to load file: %s", filepath.c_str());
	return false;
      }
    
    try
      {
	YAML::Parser parser(fin);
	YAML::Node doc;
	parser.GetNextDocument(doc);
	
	if(const YAML::Node *sync_params = doc.FindValue("software_sync_params")){
	  (*sync_params)["software_sync"] >> software_synchronization;
	  if(software_synchronization){
	    (*sync_params)["software_sync_period"] >> software_sync_period;
	  }
	}
	   
	if(const YAML::Node *configs = doc.FindValue("cameras")){
	     
	  configs_.resize(configs->size());
	  for(unsigned int i = 0; i< configs->size();i++)
	    {
	      (*configs)[i]["camera_id"] >> configs_[i].camera_id;
		 
	      if ((*configs)[i].FindValue("pixel_format"))
		{
		  std::string format;
		  (*configs)[i]["pixel_format"] >> format;
		  configs_[i].pixel_format = ParsePixelFormat(format);
		  //		  configs_[i].pixel_format = ARV_PIXEL_FORMAT_BAYER_GR_8;
		}
	      else
		{
		  configs_[i].pixel_format = ARV_PIXEL_FORMAT_MONO_8;
		}
		 
	      if ((*configs)[i].FindValue("gain_mode"))
		{
		  std::string gainMode;
		  (*configs)[i]["gain_mode"] >> gainMode;
		  configs_[i].gain_mode = ParseGainMode(gainMode);
		}
	      else
		{
		  configs_[i].gain_mode = static_cast<aravis_camera_driver::GainMode>(-1);
		}
		 
	      if ((*configs)[i].FindValue("auto_white_balance_once"))
		{
		  (*configs)[i]["auto_white_balance_once"] >> configs_[i].auto_white_balance_once;
		}
	      else
		{
		  configs_[i].auto_white_balance_once = false;
		}
		 
	      if ((*configs)[i].FindValue("exposure_mode"))
		{
		  std::string exposureMode;
		  (*configs)[i]["exposure_mode"] >> exposureMode;
		  configs_[i].exposure_mode = ParseExposureMode(exposureMode);
		}
	      else
		{
		  configs_[i].exposure_mode = static_cast<aravis_camera_driver::ExposureMode>(-1);
		}
		 
	      if ((*configs)[i].FindValue("frame_rate"))
		{
		  (*configs)[i]["frame_rate"] >> configs_[i].frame_rate;
		}
	      else
		{
		  configs_[i].frame_rate = -1;
		}
		 
	      if ((*configs)[i].FindValue("gain"))
		{
		  (*configs)[i]["gain"] >> configs_[i].gain;
		}
	      else
		{
		  configs_[i].gain = -1;
		}
		 
	      if ((*configs)[i].FindValue("exposure"))
		{
		  (*configs)[i]["exposure"] >> configs_[i].exposure;
		}
	      else
		{
		  configs_[i].exposure = -1;
		}
		 
	      if ((*configs)[i].FindValue("digital_shift"))
		{
		  (*configs)[i]["digital_shift"] >> configs_[i].digital_shift;
		}
	      else
		{
		  configs_[i].digital_shift = -1;
		}
		 
	      if ((*configs)[i].FindValue("bin_x"))
		{
		  (*configs)[i]["bin_x"] >> configs_[i].bin_x;
		}
	      else
		{
		  configs_[i].bin_x = -1;
		}
		 
	      if ((*configs)[i].FindValue("bin_y"))
		{
		  (*configs)[i]["bin_y"] >> configs_[i].bin_y;
		}
	      else
		{
		  configs_[i].bin_y = -1;
		}
		 
	      if ((*configs)[i].FindValue("roi_x"))
		{
		  (*configs)[i]["roi_x"] >> configs_[i].roi_x;
		}
	      else
		{
		  configs_[i].roi_x = -1;
		}
		 
	      if ((*configs)[i].FindValue("roi_y"))
		{
		  (*configs)[i]["roi_y"] >> configs_[i].roi_y;
		}
	      else
		{
		  configs_[i].roi_y = -1;
		}
		 
	      if ((*configs)[i].FindValue("roi_width"))
		{
		  (*configs)[i]["roi_width"] >> configs_[i].roi_width;
		}
	      else
		{
		  configs_[i].roi_width = -1;
		}
		 
	      if ((*configs)[i].FindValue("roi_height"))
		{
		  (*configs)[i]["roi_height"] >> configs_[i].roi_height;
		}
	      else
		{
		  configs_[i].roi_height = -1;
		}
		 
	      // config parameters for the acquisition start trigger
	      if ((*configs)[i].FindValue("AcquisitionStartTrigMode")){
		(*configs)[i]["AcquisitionStartTrigMode"] >> configs_[i].AcquisitionStartTrigMode;
	      }
	      else{
		configs_[i].AcquisitionStartTrigMode = -1;
	      }
		 
	      if ((*configs)[i].FindValue("AcquisitionStartTrigSource")){
		(*configs)[i]["AcquisitionStartTrigSource"] >> configs_[i].AcquisitionStartTrigSource;
	      }
	      else{
		configs_[i].AcquisitionStartTrigSource = -1;
	      }
		 
	      if ((*configs)[i].FindValue("AcquisitionStartTrigActivation")){
		(*configs)[i]["AcquisitionStartTrigActivation"] >> configs_[i].AcquisitionStartTrigActivation;
	      }
	      else{
		configs_[i].AcquisitionStartTrigActivation = -1;
	      }
		 
	      // config parameters for the acquisition start trigger
	      if ((*configs)[i].FindValue("AcquisitionStartTrigMode")){
		(*configs)[i]["AcquisitionStartTrigMode"] >> configs_[i].AcquisitionStartTrigMode;
	      }
	      else{
		configs_[i].AcquisitionStartTrigMode = -1;
	      }

	      if ((*configs)[i].FindValue("AcquisitionStartTrigSource")){
		(*configs)[i]["AcquisitionStartTrigSource"] >> configs_[i].AcquisitionStartTrigSource;
	      }
	      else{
		configs_[i].AcquisitionStartTrigSource = -1;
	      }
		 
	      if ((*configs)[i].FindValue("AcquisitionStartTrigActivation")){
		(*configs)[i]["AcquisitionStartTrigActivation"] >> configs_[i].AcquisitionStartTrigActivation;
	      }
	      else{
		configs_[i].AcquisitionStartTrigActivation = -1;
	      }
		 
	      // config parameters for the Frame start trigger
	      if ((*configs)[i].FindValue("FrameStartTrigMode")){
		(*configs)[i]["FrameStartTrigMode"] >> configs_[i].FrameStartTrigMode;
	      }
	      else{
		configs_[i].FrameStartTrigMode = -1;
	      }
		 
	      if ((*configs)[i].FindValue("FrameStartTrigSource")){
		(*configs)[i]["FrameStartTrigSource"] >> configs_[i].FrameStartTrigSource;
	      }
	      else{
		configs_[i].FrameStartTrigSource = -1;
	      }
		 
	      if ((*configs)[i].FindValue("FrameStartTrigActivation")){
		(*configs)[i]["FrameStartTrigActivation"] >> configs_[i].FrameStartTrigActivation;
	      }
	      else{
		configs_[i].FrameStartTrigActivation = -1;
	      }
		 
	      // configs for the io lines
	      if ((*configs)[i].FindValue("line_selector")){
		(*configs)[i]["line_selector"] >> configs_[i].line_selector;
	      }
	      else{
		configs_[i].line_selector = -1;
	      }
		 
	      if ((*configs)[i].FindValue("line_mode")){
		(*configs)[i]["line_mode"] >> configs_[i].line_mode;
	      }
	      else{
		configs_[i].line_mode = -1;
	      }

	      if ((*configs)[i].FindValue("line_source")){
		(*configs)[i]["line_source"] >> configs_[i].line_source;
	      }
	      else{
		configs_[i].line_source = -1;
	      }
		 
	      if ((*configs)[i].FindValue("line_format")){
		(*configs)[i]["line_format"] >> configs_[i].line_format;
	      }
	      else{
		configs_[i].line_format = -1;
	      }

	      if ((*configs)[i].FindValue("line_output_selector")){
		(*configs)[i]["line_output_selector"] >> configs_[i].line_output_selector;
	      }
	      else{
		configs_[i].line_output_selector = -1;
	      }
		 
	      if ((*configs)[i].FindValue("user_output_selector")){
		(*configs)[i]["user_output_selector"] >> configs_[i].user_output_selector;
	      }

	      if ((*configs)[i].FindValue("info_filename")){
		(*configs)[i]["info_filename"] >> configs_[i].info_filename;
	      }

	      else{
		configs_[i].user_output_selector = -1;
	      }
	    } // end for each config
	} // end if cameras found
      }	   
    catch (YAML::ParserException& e){
      ROS_ERROR("%s", e.what());
      return false;
    }
    catch (YAML::Exception& e){
      ROS_ERROR("%s", e.what());
      return false;
    }
    
    if (configs_.size() < 1){
      ROS_ERROR("No camera configurations read in.");
      return false;
    }
	   
    return true;
 }

  ArvPixelFormat ParsePixelFormat(const std::string& format)
  {
    ArvPixelFormat value = ARV_PIXEL_FORMAT_MONO_8;

    if (format == sensor_msgs::image_encodings::MONO8)
    {
      value = ARV_PIXEL_FORMAT_MONO_8;
    }
    else if (format == sensor_msgs::image_encodings::MONO16)
    {
      value = ARV_PIXEL_FORMAT_MONO_16;
    }
    else if (format == sensor_msgs::image_encodings::BAYER_BGGR8)
    {
      value = ARV_PIXEL_FORMAT_BAYER_BG_8;
    }
    else if (format == sensor_msgs::image_encodings::BAYER_GBRG8)
    {
      value = ARV_PIXEL_FORMAT_BAYER_GB_8;
    }
    else if (format == sensor_msgs::image_encodings::BAYER_GRBG8)
    {
      value = ARV_PIXEL_FORMAT_BAYER_GR_8;
    }
    else if (format == sensor_msgs::image_encodings::BAYER_RGGB8)
    {
      value = ARV_PIXEL_FORMAT_BAYER_RG_8;
    }
    else if (format == sensor_msgs::image_encodings::BGR8)
    {
      value = ARV_PIXEL_FORMAT_BGR_8_PACKED;
    }
    else if (format == sensor_msgs::image_encodings::BGRA8)
    {
      value = ARV_PIXEL_FORMAT_BGRA_8_PACKED;
    }
    else if (format == sensor_msgs::image_encodings::RGBA8)
    {
      value = ARV_PIXEL_FORMAT_RGBA_8_PACKED;
    }
    else if (format == sensor_msgs::image_encodings::RGB8)
    {
      value = ARV_PIXEL_FORMAT_RGB_8_PACKED;
    }
    else
    {
      ROS_WARN("Unsupported pixel format: %s.  Using mono8 instead.", format.c_str());
    }

    return value;
  }

};

int main(int argc, char** argv)
{
    g_type_init();
    
    ros::init(argc, argv, "basler_multi_node");
    
    ros::NodeHandle nh;
    
    BaslerMultiNode node(nh);
    
    ros::spin();
    
    return(0);
}

