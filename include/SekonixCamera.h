#ifndef SEKONIX_CAMERA_GMSL_H
#define SEKONIX_CAMERA_GMSL_H

//ROS Headers
#include <ros/ros.h>

// DriveWorks Core
#include "DriveWorksApi.hpp"

#include "DeviceArguments.hpp"

#define MAX_PORTS_COUNT 4

using namespace DriveWorks;

class SekonixCamera {


public:
  explicit SekonixCamera(ros::NodeHandle &nh_in, DeviceArguments CameraArguments);

/* Destructor */
  ~SekonixCamera();

  /*
   * Start the polling threads to grab an image from the camera and publish it
   */
  void Startup();

  /*
   * Stop the polling threads to grab an image from the camera and publish it
   * Send a request to cleanup the camera connections all at once
   */
  void Shutdown();

private:
  ros::NodeHandle &nh_;

//  Struct Program Arguments
  struct ProgramArguments{
    std::string camera_type_a;
    std::string camera_type_b;
    std::string camera_type_c;
    std::string camera_type_d;
    std::string selector_mask;
    std::string tegra_slave;
    std::string cross_csi_sync;
  };
//Class parameters
  const char* groupIDNames_[MAX_PORTS_COUNT] = {"a", "b", "c", "d"};
  uint32_t m_activeCamerasPerPort_[4] = {0};
  struct ProgramArguments args_;
  // ROS Parameters: configuration
  int image_height_,image_width_;
  bool img_compressed_ = false;
  int jpeg_quality_;
  int pub_buffer_;                                                 // Image buffer for publishing

//  DriveWorks Context
  DriveWorksApi *gmsl_cam_;                                        // GMSL camera instance
  uint32_t numPort_;                                               // AGX Xavier camera port


//  Class Methods


};


#endif //SEKONIX_CAMERA_GMSL_H
