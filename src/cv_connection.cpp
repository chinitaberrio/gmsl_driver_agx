/*
 * This code has been modified from
 * 1. https://github.com/vehicularkech/gmsl-camera-ros-driver
 * 2. https://github.com/cshort101/gmsl_driver
 * 3. https://github.com/DavidTorresOcana/ros_gmsl_driver
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of Autoware nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  All rights reserved.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
/*
  This program requires ROS
  Author: Punnu Phairatt
  Initial Date: 10/05/18
*/

#include "cv_connection.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>

OpenCVConnector::OpenCVConnector(std::string topic_name,
                                 std::string camera_frame_id,
                                 std::string cam_info_file,
                                 int buffer)
    : it_(nh_),
      counter(0),
      camera_id(camera_frame_id),
      camera_info_manager(ros::NodeHandle(topic_name), camera_frame_id) {
  std::string topic_raw = topic_name + std::string("/image_raw");
  std::string topic_jpg = topic_name + std::string("/image_raw/compressed");
  record_camera_flag = false;
  pub_jpg_flag = false;
  frame_counter = 0;
  g_counter = 0; 
  pub = it_.advertise(topic_raw, buffer);
  pub_jpg = nh_.advertise<sensor_msgs::CompressedImage>(topic_jpg, buffer);
  pub_caminfo = nh_.advertise<sensor_msgs::CameraInfo>(camera_frame_id + std::string("/camera_info"), 1);
  pub_frameinfo = nh_.advertise<gmsl_frame_msg::FrameInfo>(camera_frame_id + std::string("/frame_info"), 1);

  if (camera_info_manager.validateURL(cam_info_file)) {
    camera_info_manager.loadCameraInfo(cam_info_file);
    camera_info = camera_info_manager.getCameraInfo();
  } else {
    ROS_ERROR("ERROR READING CALIBRATION FILE: %s", cam_info_file.c_str());
  }
}

void OpenCVConnector::WriteToOpenCV(unsigned char *buffer, int width_in, int height_in, int width_pub, int height_pub) {
  cv::Mat mat_img(cv::Size(width_in, height_in), CV_8UC4, buffer);    // create a cv::Mat from rgbaImage
  cv::Mat dst;

  if ((width_in != width_pub) || (height_in != height_pub)) {
    cv::resize(mat_img, dst, cv::Size(width_pub, height_pub));          // resize to the publishing size
  } else {
    dst = mat_img;
  }
  cv::Mat converted;                                                  // new cv::Mat();
  cv::cvtColor(dst, converted, cv::COLOR_RGBA2RGB);                    // COLOR_BGRA2BGR
  cv_bridge::CvImage img_bridge;
  sensor_msgs::Image img_msg;                                        // message to be sent

  std_msgs::Header header;                                            // empty header
  header.seq = counter;                                              // user defined counter
  header.stamp = ros::Time::now();                                    // time
  header.frame_id = camera_id;                                        // camera id
  img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, converted);
  img_bridge.toImageMsg(img_msg);                                    // from cv_bridge to sensor_msgs::Image
  pub.publish(img_msg);                                              // pub image

  //publish camera info
  camera_info.header = header;
  pub_caminfo.publish(camera_info);

}

void OpenCVConnector::WriteToJpeg(uint8_t *data, uint32_t compressed_size, const ros::Time &time_stamp) {
  sensor_msgs::CompressedImage img_msg_compressed;
  img_msg_compressed.data.resize(compressed_size);
  memcpy(&img_msg_compressed.data[0], data, compressed_size);
  std_msgs::Header header;                                            // empty header
  header.seq = counter;                                              // user defined counter
  header.stamp = time_stamp;                                    // time
  header.frame_id = camera_id;                                        // camera id
  img_msg_compressed.header = header;
  img_msg_compressed.format = "jpeg";
  pub_jpg.publish(img_msg_compressed);

  //publish camera info
  camera_info.header = header;
  pub_caminfo.publish(camera_info);

  g_counter++;

}

void OpenCVConnector::PubFrameInfo(const ros::Time &time_stamp, uint64_t camera_timestamp){
  std_msgs::Header header;                                            // empty header
  header.seq = counter;                                              // user defined counter
  header.stamp = time_stamp;                                    // time
  header.frame_id = camera_id;                                        // camera id
  gmsl_frame_msg::FrameInfo frame_info_msg;
  frame_info_msg.header = header;
  frame_info_msg.frame_counter = frame_counter;
  frame_info_msg.camera_timestamp = camera_timestamp;
  frame_info_msg.global_counter = g_counter;
  frame_info_msg.name = camera_id;
  frame_info_msg.ros_timestamp = time_stamp;
  pub_frameinfo.publish(frame_info_msg);
  frame_counter++;
}


void OpenCVConnector::check_for_subscribers(){
  if (!record_camera_flag && pub_frameinfo.getNumSubscribers() > 0){
    record_camera_flag = true;
    frame_counter = 0;
  }
  else if (record_camera_flag && pub_frameinfo.getNumSubscribers() < 1){
    record_camera_flag = false;
  }

  if (pub_jpg.getNumSubscribers() > 0){
    pub_jpg_flag = true;
  } else {
    pub_jpg_flag = false;
  }

}


