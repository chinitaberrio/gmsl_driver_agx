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

#ifndef _OPEN_CV_CONNECTOR_
#define _OPEN_CV_CONNECTOR_

#include <string>
#include <vector>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <camera_info_manager/camera_info_manager.h>
#include <gmsl_frame_msg/FrameInfo.h>


class OpenCVConnector {
public:
  using Ptr = std::shared_ptr<OpenCVConnector>;

  OpenCVConnector(std::string topic_name, std::string camera_frame_id, std::string cam_info_file, int buffer);

  void WriteToOpenCV(unsigned char *data, int width_in, int height_in, int width_pub, int height_pub);

  void WriteToJpeg(uint8_t *data, uint32_t compressed_size, const ros::Time &time_stamp);

  void PubFrameInfo(const ros::Time &time_stamp, uint64_t camera_timestamp);

  void check_for_subscribers();

  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Publisher pub;
  ros::Publisher pub_jpg;
  std::string topic_name;
  std::string camera_id;
  unsigned int counter;
  uint32_t frame_counter;
  uint32_t g_counter;
  bool record_camera_flag;
  bool pub_jpg_flag;
  sensor_msgs::CameraInfo camera_info;
  camera_info_manager::CameraInfoManager camera_info_manager;
  ros::Publisher pub_caminfo;
  ros::Publisher pub_frameinfo;
  std::string calib_folder;
};


#endif

