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
#include <aravis_camera_driver/camera.h>

// Struct needed to implement custom create stream function.
// Copied from arvgvdevice.c
typedef struct
{
  GMutex *mutex;

  guint16 packet_id;

  GSocket *socket;
  GSocketAddress  *interface_address;
  GSocketAddress  *device_address;

  GPollFD poll_in_event;

  void *buffer;

  unsigned int gvcp_n_retries;
  unsigned int gvcp_timeout_ms;

  gboolean is_controller;
} ArvGvDeviceIOData;

// Struct needed to implement custom create stream function.
// Copied from arvgvdevice.c
struct _ArvGvDevicePrivate
{
  ArvGvDeviceIOData *io_data;

  void *heartbeat_thread;
  void *heartbeat_data;

  ArvGc *genicam;

  char *genicam_xml;
  size_t genicam_xml_size;
};

namespace aravis_camera_driver
{
  Camera::Camera() :
      initialized_(false),
      has_callback_(false),
      auto_socket_buffer_(true),
      no_packet_resend_(true),
      packet_timeout_(20),
      frame_retention_(100),
      pixel_format_(ARV_PIXEL_FORMAT_MONO_8),
      packet_size_(1500),
      bin_x_(1),
      bin_y_(1),
      roi_x_(0),
      roi_y_(0),
      roi_width_(640),
      roi_height_(480),
      width_(0),
      height_(0)
  {
     g_type_init();
  }

  Camera::~Camera()
  {

  }

  bool Camera::Initialize(const std::string& device, bool exact)
  {
    if (!initialized_)
    {
      camera_id_ = device;

      arv_update_device_list ();
      int device_count = arv_get_n_devices();

      ROS_INFO("Found %d devices.", device_count);

      std::string device_id = device;
      bool match = false;
      for (int i = 0; i < device_count; i++)
      {
        std::string id = std::string(arv_get_device_id(i));
        if (device == id || (!exact && id.find(device) != std::string::npos))
        {
          match = true;
          ROS_INFO(" --> Found device %s", arv_get_device_id(i));
          device_id = id;
        }
        else
        {
          ROS_INFO("     Found device %s", arv_get_device_id(i));
        }
      }

      if (match)
      {
        camera_ = arv_camera_new(device_id.c_str());
        device_ = arv_camera_get_device(camera_);
        genicam_ = arv_device_get_genicam(device_);

        ParseParameters();

        ApplyPreAcquisitionParameters();

        ROS_INFO(" Setting pixel format(%d): %s", pixel_format_, arv_pixel_format_to_gst_caps_string (pixel_format_));
        arv_camera_set_pixel_format(camera_, pixel_format_);
        ArvPixelFormat format = arv_camera_get_pixel_format (camera_);
        if (format == pixel_format_)
        {
          // Set binning parameters
          ROS_INFO(" Setting binning: x=%d, y=%d", bin_x_, bin_y_);
          arv_camera_set_binning(camera_, bin_x_, bin_y_);
          int newX, newY;
          arv_camera_get_binning(camera_, &newX, &newY);
          if (newX != bin_x_ || newY != bin_y_)
          {
            ROS_WARN("  Failed to set binning: [%d,%d] != [%d,%d]", newX, newY, bin_x_, bin_y_);
          }

          // Set ROI parameters
          ROS_INFO(" Setting region: (%d,%d) %dx%d", roi_x_, roi_y_, roi_width_, roi_height_);
          arv_camera_set_region(camera_, roi_x_, roi_y_, roi_width_, roi_height_);

          arv_camera_get_region (camera_, NULL, NULL, &width_, &height_);
          ROS_INFO(" Image size: %dx%d", width_, height_);

          stream_ = CreateStream(device_, NULL, NULL, packet_size_);

          if (stream_ != NULL)
          {
            if (auto_socket_buffer_)
            {
              g_object_set(stream_, "socket-buffer", ARV_GV_STREAM_SOCKET_BUFFER_AUTO, "socket-buffer-size", 0, NULL);
            }
            else
            {
              // TODO(malban): ARV_GV_STREAM_SOCKET_BUFFER_FIXED
            }
            
            if (no_packet_resend_)
            {
              g_object_set(stream_, "packet-resend", ARV_GV_STREAM_PACKET_RESEND_NEVER, NULL);
            }
            else
            {
              g_object_set(stream_, "packet-resend", ARV_GV_STREAM_PACKET_RESEND_ALWAYS, NULL);
            }
            
            g_object_set (
                stream_,
                "packet-timeout", (unsigned) packet_timeout_ * 1000,
                "frame-retention", (unsigned) frame_retention_ * 1000,
                NULL);

            arv_stream_set_emit_signals (stream_, true);

            // TODO what does 50 signify
            int payload = arv_camera_get_payload (camera_);
            for (int i = 0; i < 50; i++)
            {
              arv_stream_push_buffer (stream_, arv_buffer_new (payload, NULL));
            }

            arv_camera_start_acquisition(camera_);
            g_signal_connect (stream_, "new-buffer", G_CALLBACK (Camera::BufferCallback), this);

            initialized_ = true;
          }
          else
          {
            g_object_unref(camera_);
            camera_ = NULL;
            ROS_ERROR("Failed to create camera stream.");
          }
        }
        else
        {
          g_object_unref(camera_);
          camera_ = NULL;
          ROS_ERROR("  Failed to set pixel format.");
        }
      }
    }

    return initialized_;
  }

  void Camera::HandleFrame(ArvStream* stream)
  {
    ArvBuffer *arv_buffer;

    arv_buffer = arv_stream_pop_buffer(stream);
    if (arv_buffer == NULL)
    {
      ROS_WARN("Camera buffer was unexpectedly empty.");
      return;
    }

    if (arv_buffer->status == ARV_BUFFER_STATUS_SUCCESS)
    {
      if (has_callback_)
      {
        frame_callback_(arv_buffer, camera_id_);
      }
    }
    else
    {
      ROS_INFO("Camera buffer not ready.");
    }

    arv_stream_push_buffer(stream, arv_buffer);
  }

  void Camera::BufferCallback(ArvStream* stream, Camera* camera)
  {
    camera->HandleFrame(stream);
  }

  void Camera::SetExposure(ExposureMode mode, uint32_t exp_usecs)
  {
    // TODO: need to make sure that the exposure base is actually 20 usecs
    if(!initialized_)
    {
      return;
    }

    double exp_usecs_double;
    double max_exp_usecs;
    double min_exp_usecs;
    double act_exp_time;

    switch(mode)
    {
      case CAM_EXPOSURE_MANUAL:
        ROS_INFO(" Setting exposure mode: fixed, %d microseconds", exp_usecs);
        arv_camera_set_exposure_time_auto( camera_, ARV_AUTO_OFF );
        arv_camera_get_exposure_time_bounds(camera_, &min_exp_usecs, &max_exp_usecs);

        exp_usecs_double = static_cast<double>(exp_usecs);

        exp_usecs_double = std::min(max_exp_usecs,std::max(min_exp_usecs,exp_usecs_double));
        arv_camera_set_exposure_time(camera_,exp_usecs_double);

        act_exp_time =  arv_camera_get_exposure_time(camera_);

        ROS_INFO("  Actual exposure: %f",act_exp_time);
        break;

      case CAM_EXPOSURE_AUTO:
        ROS_INFO(" Setting exposure mode: auto continuous");
        arv_camera_set_exposure_time_auto( camera_, ARV_AUTO_CONTINUOUS );

        break;

      case CAM_EXPOSURE_AUTO_ONCE:
        ROS_INFO(" Setting exposure mode: auto once");
        arv_camera_set_exposure_time_auto( camera_, ARV_AUTO_ONCE );
        break;

      default:
        break;
    }
  }

  void Camera::SetGain(GainMode mode, uint32_t gain)
  {
    if(!initialized_)
    {
      return;
    }

    if (mode == CAM_GAIN_MANUAL)
    {
      ROS_INFO(" Setting gain mode: fixed, %d", gain);
      arv_camera_set_gain_auto( camera_, ARV_AUTO_OFF);
      arv_camera_set_gain(camera_, gain);

      int newGain = arv_camera_get_gain(camera_);
      if (newGain != (int)gain)
      {
        ROS_WARN("  Failed to set gain: %d != %d", newGain, (int)gain);
      }

    }
    else if (mode == CAM_GAIN_AUTO)
    {
      ROS_INFO(" Setting gain mode: auto continuous");
      arv_camera_set_gain_auto( camera_, ARV_AUTO_CONTINUOUS);
    }
    else if (mode == CAM_GAIN_AUTO_ONCE)
    {
      ROS_INFO(" Setting gain mode: auto once");
      arv_camera_set_gain_auto( camera_, ARV_AUTO_ONCE);
    }

  }

  void Camera::SetFrameRate(double rate)
  {
    ROS_INFO(" Setting frame rate: %lf fps", rate);
    arv_camera_set_frame_rate(camera_, rate);
  }

  void Camera::SetPixelFormat(ArvPixelFormat format)
  {
    pixel_format_ = format;
  }

  void Camera::SetPacketSize(int size)
  {
    packet_size_ = size;
  }

  void Camera::SetBinning(int x, int y)
  {
    bin_x_ = x;
    bin_y_ = y;
  }

  void Camera::SetRegion(int x, int y, int width, int height)
  {
    roi_x_ = x;
    roi_y_ = y;
    roi_width_ = width;
    roi_height_ = height;
  }

  void Camera::SetPacketSize(ArvDevice* device, int size)
  {
    //ROS_INFO("Setting the packet size to %d",size);

    arv_device_write_register (device,
                               ARV_GVBS_STREAM_CHANNEL_0_PACKET_SIZE_OFFSET,
                               size);

    guint packet_size;
    arv_device_read_register (device,
                              ARV_GVBS_STREAM_CHANNEL_0_PACKET_SIZE_OFFSET,
                              &packet_size);

    if(static_cast<int>(packet_size) != size)
    {
      //ROS_ERROR("Failed to set the packet size");
    }

  }

  void Camera::SetDigitalShift(int shift)
  {
    ROS_WARN(" Failed to set camera specific digital shift parameter with generic driver.");
  }

  void Camera::SetAutoWhiteBalanceOnce(bool on)
  {
    ROS_WARN(" Failed to set camera specific auto white balance parameter with generic driver.");
  }

  void Camera::SetCaptureMode(const std::string& mode)
  {
    ROS_WARN(" Failed to set camera specific capture mode parameter with generic driver.");
  }

  void Camera::SetTriggerActivation(const std::string& activation)
  {
    ROS_WARN(" Failed to set camera specific trigger activation parameter with generic driver.");
  }

  void Camera::SetTriggerSource(const std::string& source)
  {
    ROS_WARN(" Failed to set camera specific trigger source parameter with generic driver.");
  }

  ArvStream * Camera::CreateStream(ArvDevice *device, ArvStreamCallback callback, void *user_data, int packet_size)
  {
    ArvGvDevice *gv_device = ARV_GV_DEVICE (device);
    ArvStream *stream;
    ArvGvDeviceIOData *io_data = gv_device->priv->io_data;
    const guint8 *address_bytes;
    guint32 stream_port;
    guint32 n_stream_channels;
    GInetAddress *interface_address;
    GInetAddress *device_address;

    arv_device_read_register (device, ARV_GVBS_N_STREAM_CHANNELS_OFFSET, &n_stream_channels);
    //arv_debug_device ("[GvDevice::create_stream] Number of stream channels = %d", n_stream_channels);

    if (n_stream_channels < 1)
      return NULL;

    if (!io_data->is_controller) {
      //arv_warning_device ("[GvDevice::create_stream] Can't create stream without control access");
      return NULL;
    }

    interface_address = g_inet_socket_address_get_address (G_INET_SOCKET_ADDRESS (io_data->interface_address));
    device_address = g_inet_socket_address_get_address (G_INET_SOCKET_ADDRESS (io_data->device_address));
    address_bytes = g_inet_address_to_bytes (interface_address);

    SetPacketSize(device, packet_size);

    stream = arv_gv_stream_new (device_address, 0, callback, user_data,
                                arv_gv_device_get_timestamp_tick_frequency (gv_device), packet_size);

    stream_port = arv_gv_stream_get_port (ARV_GV_STREAM (stream));

    arv_device_write_register (device, ARV_GVBS_STREAM_CHANNEL_0_IP_ADDRESS_OFFSET,
                               g_htonl(*((guint32 *) address_bytes)));
    arv_device_write_register (device, ARV_GVBS_STREAM_CHANNEL_0_PORT_OFFSET, stream_port);

    //arv_debug_device ("[GvDevice::create_stream] Stream port = %d", stream_port);

    return stream;
  }

  void Camera::ParseEnumeration(const std::string& name, std::map<std::string,int>& map)
  {
    ArvGcNode* enumeration = arv_gc_get_node(genicam_, name.c_str());
    if (enumeration != NULL)
    {
      ROS_INFO(" %s enumeration:", name.c_str());
      for (const GSList* entries = arv_gc_node_get_childs(enumeration); entries != NULL; entries = entries->next)
      {
        if (ARV_IS_GC_ENUM_ENTRY(entries->data))
        {
          std::string name = arv_gc_node_get_name(static_cast<ArvGcNode*>(entries->data));
          int value = arv_gc_enum_entry_get_value(static_cast<ArvGcEnumEntry*>(entries->data));

          ROS_INFO("  %s(%d)", name.c_str(), value);
          map[name] = value;
        }
      }
    }
  }

  void Camera::ParseParameters()
  {
    // Nothing to do for generic aravis camera.
  }

  void Camera::ApplyPreAcquisitionParameters()
  {
    // Nothing to do for generic aravis camera.
  }
}
