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

#ifndef ARAVIS_CAMERA_DRIVER_CAMERA_H
#define ARAVIS_CAMERA_DRIVER_CAMERA_H

#include <stdint.h>

#include <arv.h>

namespace aravis_camera_driver
{
  enum ExposureMode
  {
    CAM_EXPOSURE_MANUAL,
    CAM_EXPOSURE_AUTO,
    CAM_EXPOSURE_AUTO_ONCE
  };

  enum GainMode
  {
    CAM_GAIN_MANUAL,
    CAM_GAIN_AUTO,
    CAM_GAIN_AUTO_ONCE
  };
  
  enum TriggerActivation
    {
      CAM_TRIG_ACTIVATION_RISING_EDGE,
      CAM_TRIG_ACTIVATION_FALLING_EDGE,
      CAM_TRIG_ACTIVATION_ANY_EDGE,
      CAM_TRIG_ACTIVATION_LEVEL_HIGH,
      CAM_TRIG_ACTIVATION_LEVEL_LOW
    };
 
 enum TriggerMode
    {
      CAM_TRIG_MODE_OFF,
      CAM_TRIG_MODE_ON
    };
 

  enum TriggerSource
    {
      CAM_TRIG_SOURCE_SOFTWARE,
      CAM_TRIG_SOURCE_LINE1,
      CAM_TRIG_SOURCE_LINE2,
      CAM_TRIG_SOURCE_LINE3,
      CAM_TRIG_SOURCE_LINE4,
      CAM_TRIG_SOURCE_LINE5,
      CAM_TRIG_SOURCE_LINE6,
      CAM_TRIG_SOURCE_LINE7,
      CAM_TRIG_SOURCE_LINE8,
      CAM_TRIG_SOURCE_CC1,
      CAM_TRIG_SOURCE_CC2,
      CAM_TRIG_SOURCE_CC3,
      CAM_TRIG_SOURCE_CC4
    };

  enum TriggerSelector
    {
      CAM_TRIG_SEL_ACQUISITION_START,
      CAM_TRIG_SEL_FRAME_START,
      CAM_TRIG_SEL_LINE_START
    };

  enum LineSelector
  {
    CAM_LINE_SEL_LINE1,
    CAM_LINE_SEL_OUT1
  };
  enum LineMode
  {
    CAM_LINE_MODE_INPUT,
    CAM_LINE_MODE_OUTPUT
  };

  enum LineSource
  {
    CAM_LINE_SOURCE_USER_OUTPUT=0,
    CAM_LINE_SOURCE_FRAME_TRIGGER_WAIT=1,
    CAM_LINE_SOURCE_EXPOSURE_ACTIVE=2,
    CAM_LINE_SOURCE_ACQUISITION_TRIGGER_WAIT=7,
    CAM_LINE_SOURCE_TIMER_ACTIVE=18,
    CAM_LINE_SOURCE_FLASH_WINDOW=25
  };

  enum LineFormat
  {
    CAM_LINE_FORMAT_OPTOCOUPLED=5
  };

  enum UserOutputSelector
  {
    CAM_USER_OUTPUT_SELECTOR_OUTPUT1=0
  };


  class Camera
  {
  public:
    Camera();
    virtual ~Camera();

    int Width() const { return width_; }
    int Height() const { return height_; }

    bool Initialize(const std::string& device, bool exact=true);

    template<class T>
    void SetFrameCallback( void(T::*fp)(ArvBuffer*, const std::string&) , T* obj)
    {
      has_callback_ = true;
      frame_callback_ = boost::bind(fp, obj, _1, _2);
    }

    void HandleFrame(ArvStream* stream);

    static void BufferCallback(ArvStream* stream, Camera* camera);

    void SetPixelFormat(ArvPixelFormat format);
    void SetPacketSize(int size);
    void SetBinning(int x, int y);
    void SetRegion(int x, int y, int width, int height);

    void SetFrameRate(double rate);
    void SetExposure(ExposureMode mode, uint32_t exp_usecs = 10000);
    void SetGain(GainMode mode, uint32_t gain = 0);

    // Camera specific functionality
    virtual void SetDigitalShift(int shift);
    virtual void SetAutoWhiteBalanceOnce(bool on);

    virtual void SetCaptureMode(const std::string& mode);
    virtual void SetTriggerActivation(const std::string& activation);
    virtual void SetTriggerSource(const std::string& source);

  protected:

    ArvDevice* device_;
    ArvGc*     genicam_;
    ArvCamera* camera_;
    ArvStream* stream_;

    bool initialized_;
    bool has_callback_;

    bool auto_socket_buffer_;
    bool no_packet_resend_;
    
    int32_t packet_timeout_;
    int32_t frame_retention_;

    ArvPixelFormat pixel_format_;

    std::string camera_id_;

    int packet_size_;

    int bin_x_;
    int bin_y_;

    int roi_x_;
    int roi_y_;
    int roi_width_;
    int roi_height_;

    int width_;
    int height_;

    boost::function<void (ArvBuffer*, const std::string&)> frame_callback_;

    virtual void ApplyPreAcquisitionParameters();
    virtual void ParseParameters();

    void ParseEnumeration(const std::string& name, std::map<std::string,int>& map);

    void SetPacketSize(ArvDevice* device, int size);
    ArvStream * CreateStream(ArvDevice *device, ArvStreamCallback callback, void *user_data, int packet_size);
  };
}

#endif /* ARAVIS_CAMERA_DRIVER_CAMERA_H */
