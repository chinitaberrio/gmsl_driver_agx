#include <iostream>
#include <ros/ros.h>
#include "SekonixCamera.h"

// Running state
static bool volatile running = true;

void signalHandler(int sig) {
  (void) sig;
  running = false;
  ros::shutdown();
}

std::vector<DriveWorks::option_t> options =
  {
    // making pair camera config key:value
    std::make_pair("type-a", "ar0231-rccb"),
    std::make_pair("type-b", "ar0231-rccb"),
    std::make_pair("type-c", "ar0231-rccb"),
    std::make_pair("type-d", "ar0231-rccb"),
    std::make_pair("selector_mask", "11111111"),
    std::make_pair("cross_csi_sync", "0"),
    std::make_pair("fifo_size", "3"),
    std::make_pair("slave", "0"),
  };


int main(int argc, char **argv) {

  DriveWorks::DeviceArguments CameraArguments(options);

  ros::init(argc, argv, "gmsl_node");
  ros::NodeHandle nh;

  // Detect exit signals
  signal(SIGHUP, signalHandler);  // controlling terminal closed, Ctrl-D
  signal(SIGINT, signalHandler);  // Ctrl-C
  signal(SIGQUIT, signalHandler); // Ctrl-\, clean quit with core dump
  signal(SIGABRT, signalHandler); // abort() called.
  signal(SIGTERM, signalHandler); // kill command
  signal(SIGSTOP, signalHandler); // kill command


  ROS_INFO_STREAM("GMSL node started!");

  SekonixCamera gmsl_multiple_cam(nh, CameraArguments);

  while (running & ros::ok()) {
    ros::spinOnce();
  }

  std::cout << "Shutting down cameras .." << std::endl;
  gmsl_multiple_cam.Shutdown();

  ros::spin();
  return 0;
}