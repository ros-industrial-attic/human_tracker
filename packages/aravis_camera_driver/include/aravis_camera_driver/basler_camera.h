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

#ifndef ARAVIS_CAMERA_DRIVER_BASLER_CAMERA_H
#define ARAVIS_CAMERA_DRIVER_BASLER_CAMERA_H

#include <map>
#include <arv.h>
#include <aravis_camera_driver/camera.h>

namespace aravis_camera_driver
{
  class BaslerCamera : public Camera
  {
  public:
    BaslerCamera();
    ~BaslerCamera();

    virtual void SetDigitalShift(int shift);
    virtual void SetAutoWhiteBalanceOnce(bool on);

    virtual void SetCaptureMode(const std::string& mode);

    virtual void TriggerExecute();
    virtual void SetTriggerSelector(const std::string& trigger);
    virtual void SetTriggerMode(const std::string& mode);
    virtual void SetTriggerSource(const std::string& source);
    virtual void SetTriggerActivation(const std::string& activation);

    virtual void SetLineSelector(const std::string& line); 
    virtual void SetLineMode(const std::string& mode); 
    virtual void SetLineSource(const std::string& source); 
    virtual void SetLineFormat(const std::string& format); 
    virtual void SetLineOutputSelector(const std::string& line); 
    virtual void SetUserOutputSelector(const std::string& output); 


  protected:
    virtual void ParseParameters();

  private:
    std::map<std::string,int> trigger_sources_;
    std::map<std::string,int> trigger_activations_;
    std::map<std::string,int> trigger_modes_;
    std::map<std::string,int> trigger_control_implementations_;
    std::map<std::string,int> trigger_selectors_;
    std::map<std::string,int> line_selectors_;
    std::map<std::string,int> line_mode_;
    std::map<std::string,int> line_source_;
    std::map<std::string,int> line_format_;
    std::map<std::string,int> line_output_selector_;
    std::map<std::string,int> user_output_selector_;

  };
}

#endif /* ARAVIS_CAMERA_DRIVER_BASLER_CAMERA_H */
