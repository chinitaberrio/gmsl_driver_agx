#ifndef SEKONIX_CAMERA_GMSL_H
#define SEKONIX_CAMERA_GMSL_H

#include <ros/ros.h>

#include "DriveWorksApi.hpp"

#include "DeviceArguments.hpp"
#include <memory>

#define MAX_PORTS_COUNT 4

using namespace DriveWorks;

class SekonixCamera {
public:
  explicit SekonixCamera(ros::NodeHandle &nh_in);

  void Shutdown();

private:
  ros::NodeHandle &nh_;

  struct ProgramArguments {
    std::string camera_type_a;
    std::string camera_type_b;
    std::string camera_type_c;
    std::string camera_type_d;
    std::string selector_mask;
    std::string tegra_slave;
    std::string cross_csi_sync;
  };

  const char *groupIDNames_[MAX_PORTS_COUNT] = {"a", "b", "c", "d"};
  uint32_t m_activeCamerasPerPort_[4] = {0};
  struct ProgramArguments args_;
  // ROS Parameters: configuration
  int image_height_, image_width_;
  bool img_compressed_ = false;
  int jpeg_quality_;
  int pub_buffer_;                                                 // Image buffer for publishing

  std::unique_ptr<DriveWorksApi> gmsl_cam_;                                        // GMSL camera instance
  uint32_t numPort_;                                               // AGX Xavier camera port

};


#endif //SEKONIX_CAMERA_GMSL_H
