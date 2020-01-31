#ifndef SEKONIX_CAMERA_GMSL_H
#define SEKONIX_CAMERA_GMSL_H

#include <ros/ros.h>
#include "DriveWorksApi.hpp"
#include "DeviceArguments.hpp"
#include <memory>

class SekonixCamera {
public:
  explicit SekonixCamera(ros::NodeHandle &nh_in);

  void Shutdown();

private:
  ros::NodeHandle &nh_;

  std::unique_ptr<DriveWorks::DriveWorksApi> driveworks_api_;
};


#endif //SEKONIX_CAMERA_GMSL_H
