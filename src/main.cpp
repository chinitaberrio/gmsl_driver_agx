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


int main(int argc, char **argv) {


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

  SekonixCamera gmsl_multiple_cam(nh);

  while (running & ros::ok()) {
    ros::spinOnce();
  }

  std::cout << "Shutting down cameras .." << std::endl;
  gmsl_multiple_cam.Shutdown();

  ros::spin();
  return 0;
}