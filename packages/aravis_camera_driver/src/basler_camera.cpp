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

#include <cmath>
#include <ros/ros.h>
#include <aravis_camera_driver/basler_camera.h>

namespace aravis_camera_driver
{
  BaslerCamera::BaslerCamera() : Camera()
  {

  }

  BaslerCamera::~BaslerCamera()
  {

  }

  void BaslerCamera::ParseParameters()
  {
    ROS_INFO("Parsing genicam parameters ...");

    ParseEnumeration("TriggerControlImplementation", trigger_control_implementations_);
    ParseEnumeration("TriggerActivation", trigger_activations_);
    ParseEnumeration("TriggerMode", trigger_modes_);
    ParseEnumeration("TriggerSource", trigger_sources_);
    ParseEnumeration("TriggerSelector", trigger_selectors_);
    ParseEnumeration("LineSelector", line_selectors_);
    ParseEnumeration("LineMode", line_mode_);
    ParseEnumeration("LineSource", line_source_); 
    ParseEnumeration("LineFormat", line_format_);
    ParseEnumeration("UserOutputSelector", user_output_selector_);
  }

  void BaslerCamera::SetAutoWhiteBalanceOnce(bool on)
  {
    ROS_INFO(" Setting auto white balance once: %d", static_cast<int>(on));
    arv_device_set_integer_feature_value(device_,"BalanceWhiteAuto", static_cast<int>(on));
    int newValue = arv_device_get_integer_feature_value(device_, "BalanceWhiteAuto");
    if (newValue != static_cast<int>(on))
    {
      ROS_WARN(  "Failed to set auto white balance once: %d != %d", newValue, static_cast<int>(on));
    }
  }

  void BaslerCamera::SetDigitalShift(int shift)
  {
    ROS_INFO(" Setting digital shift: %d", shift);
    arv_device_set_integer_feature_value(device_, "DigitalShift", shift);

    int newShift = arv_device_get_integer_feature_value(device_, "DigitalShift");
    if (newShift != shift)
    {
      ROS_WARN(  "Failed to set digital shift: %d != %d", newShift, shift);
    }
  }

  void BaslerCamera::SetCaptureMode(const std::string& mode)
  {
    if (mode == "continuous")
    {
      ROS_INFO(" Setting capture mode: continuous");
      arv_device_set_integer_feature_value(device_, "TriggerMode", trigger_modes_["Off"]);

      int newMode = arv_device_get_integer_feature_value(device_, "TriggerMode");
      if (newMode != trigger_modes_["Off"])
      {
        ROS_WARN(  "Failed to set capture mode");
      }
    }
    else if (mode == "triggered")
    {
      ROS_INFO(" Setting capture mode: triggered");

      if (trigger_control_implementations_.count("Standard")  != 0)
      {
        ROS_INFO("  Setting TriggerControlImplementation: Standard");
        arv_device_set_integer_feature_value(device_, "TriggerControlImplementation", trigger_control_implementations_["Standard"]);
      }

      if (trigger_selectors_.count("FrameStart") != 0)
      {
        ROS_INFO("  Setting TriggerSelector: FrameStart");
        arv_device_set_integer_feature_value(device_, "TriggerSelector", trigger_selectors_["FrameStart"]);
      }

      if (trigger_modes_.count("On") != 0)
      {
        ROS_INFO("  Setting TriggerMode: On");
        arv_device_set_integer_feature_value(device_, "TriggerMode", trigger_modes_["On"]);

        int newMode = arv_device_get_integer_feature_value(device_, "TriggerMode");
        if (newMode != trigger_modes_["On"])
        {
          ROS_WARN("Failed to set capture mode");
        }
      }
      else
      {
        ROS_WARN("Failed to set capture mode");
      }
    }
  }
 inline  void BaslerCamera::TriggerExecute()
  {
    arv_camera_software_trigger(camera_);
  }

  void BaslerCamera::SetTriggerSelector(const std::string& trigger)
  {
    ROS_INFO(" Setting trigger selector: %s", trigger.c_str());
    if (trigger_selectors_.count(trigger) != 0)
    {
      arv_device_set_integer_feature_value(device_, "TriggerSelector", trigger_selectors_[trigger]);

      int newSource = arv_device_get_integer_feature_value(device_, "TriggerSelector");
      if (newSource != trigger_selectors_[trigger])
      {
        ROS_WARN("  Failed to set trigger selectors");
      }
    }
    else
    {
      ROS_WARN("  Failed to set trigger selector.  Unsupported value.");
    }
  }

  void BaslerCamera::SetTriggerActivation(const std::string& activation)
  {
    ROS_INFO(" Setting trigger activation: %s", activation.c_str());
    if (trigger_activations_.count(activation) != 0)
    {
      arv_device_set_integer_feature_value(device_, "TriggerActivation", trigger_activations_[activation]);

      int newSource = arv_device_get_integer_feature_value(device_, "TriggerActivation");
      if (newSource != trigger_activations_[activation])
      {
        ROS_WARN("  Failed to set trigger activation");
      }
    }
    else
    {
      ROS_WARN("  Failed to set trigger activation.  Unsupported value.");
    }
  }

  void BaslerCamera::SetTriggerMode(const std::string& mode)
  {
    ROS_INFO(" Setting trigger mode: %s", mode.c_str());
    if (trigger_modes_.count(mode) != 0)
    {
      arv_device_set_integer_feature_value(device_, "TriggerMode", trigger_modes_[mode]);

      int newMode = arv_device_get_integer_feature_value(device_, "TriggerMode");
      if (newMode != trigger_modes_[mode])
      {
        ROS_WARN("  Failed to set trigger Mode");
      }
    }
    else
    {
      ROS_WARN("  Failed to set trigger Mode.  Unsupported value.");
    }
  }

  void BaslerCamera::SetTriggerSource(const std::string& source)
  {
    ROS_INFO(" Setting trigger source: %s", source.c_str());
    if (trigger_sources_.count(source) != 0)
    {
      arv_device_set_integer_feature_value(device_, "TriggerSource", trigger_sources_[source]);

      int newSource = arv_device_get_integer_feature_value(device_, "TriggerSource");
      if (newSource != trigger_sources_[source])
      {
        ROS_WARN("  Failed to set trigger source");
      }
    }
    else
    {
      ROS_WARN("  Failed to set trigger source.  Unsupported value.");
    }
  }

  void BaslerCamera::SetLineSelector(const std::string& line)
  {
    ROS_INFO(" Setting line selector: %s", line.c_str());
    if (line_selectors_.count(line) != 0)
      {
	arv_device_set_integer_feature_value(device_, "LineSelector", line_selectors_[line]);
	
	int newLineSelector = arv_device_get_integer_feature_value(device_, "LineSelector");
	if (newLineSelector != line_selectors_[line])
	  {
	    ROS_WARN("  Failed to set line selectors");
	  }
      }
    else
      {
	ROS_WARN("  Failed to set line selector.  Unsupported value.");
      }
  }
  
  void BaslerCamera::SetLineMode(const std::string& mode)
  {
    ROS_INFO(" Setting line mode : %s", mode.c_str());
    if (line_mode_.count(mode) != 0)
      {
	arv_device_set_integer_feature_value(device_, "LineMode", line_mode_[mode]);
	
	int newLineMode = arv_device_get_integer_feature_value(device_, "LineMode");
	if (newLineMode != line_mode_[mode])
	  {
	    ROS_WARN("  Failed to set line mode ");
	  }
      }
    else
      {
	ROS_WARN("  Failed to set line mode.  Unsupported value.");
      }
  }
  
  void BaslerCamera::SetLineSource(const std::string& source)
  {
    ROS_INFO(" Setting line source : %s", source.c_str());
    if (line_source_.count(source) != 0)
      {
	arv_device_set_integer_feature_value(device_, "LineSource", line_source_[source]);
	
	int newLineSource = arv_device_get_integer_feature_value(device_, "LineSource");
	if (newLineSource != line_source_[source])
	  {
	    ROS_WARN("  Failed to set line source %d %d", line_source_[source], newLineSource);
	  }
      }
    else
      {
	ROS_WARN("  Failed to set line source.  Unsupported value.");
      }
  }
  
  
  void BaslerCamera::SetLineFormat(const std::string& format)
  {
    ROS_INFO(" Setting line format : %s", format.c_str());
    if (line_format_.count(format) != 0)
      {
	arv_device_set_integer_feature_value(device_, "LineFormat", line_format_[format]);
	
	int newLineFormat = arv_device_get_integer_feature_value(device_, "LineFormat");
	if (newLineFormat != line_format_[format])
	  {
	    ROS_WARN("  Failed to set line format %d %d",line_format_[format],newLineFormat);
	  }
      }
    else
      {
	ROS_WARN("  Failed to set line format.  Unsupported value.");
      }
  }
  

  void BaslerCamera::SetLineOutputSelector(const std::string& line)
  {
    ROS_INFO(" Setting Line Output Selector : %s", line.c_str());
    if (line_output_selector_.count(line) != 0)
      {
	arv_device_set_integer_feature_value(device_, "LineOutputSelector", line_output_selector_[line]);
	
	int newLineOutput = arv_device_get_integer_feature_value(device_, "LineOutputSelector");
	if (newLineOutput != line_output_selector_[line])
	  {
	    ROS_WARN("  Failed to set line output selector ");
	  }
      }
    else
      {
	ROS_WARN("  Failed to set line output selector.  Unsupported value.");
      }
  }

  void BaslerCamera::SetUserOutputSelector(const std::string& output)
  {
    ROS_INFO(" Setting User Output Selector : %s", output.c_str());
    if (user_output_selector_.count(output) != 0)
      {
	arv_device_set_integer_feature_value(device_, "UserOutputSelector", user_output_selector_[output]);
	
	int newUserOutput = arv_device_get_integer_feature_value(device_, "UserOutputSelector");
	if (newUserOutput != user_output_selector_[output])
	  {
	    ROS_WARN("  Failed to set user output selector ");
	  }
      }
    else
      {
	ROS_WARN("  Failed to set user output selector.  Unsupported value.");
      }
  }

}

