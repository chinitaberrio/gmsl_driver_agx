/*
 * This code has been modified from Nvidia SDK
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

#include "DriveWorksApi.hpp"

#include <utility>
#include <StopWatch.h>
#include <string.h>

namespace DriveWorks {
DriveWorksApi::DriveWorksApi(DeviceArguments arguments,
                             ImageConfigPub pub_image_config,
                             PrintEventHandler::Ptr print_event_handler) :
    device_arguments_(std::move(arguments)),
    pub_image_config_(std::move(pub_image_config)),
    debug_mode_(false),
    name_pretty_("DriveWorksApi"),
    print_event_handler_(std::move(print_event_handler)) {
  print_event_handler_->Print(name_pretty_, "Constructor is called.");
  InitializeContextHandle(context_handle_);
  print_event_handler_->Print(name_pretty_, "context_handle_ is initialized!");
  InitializeSalHandle(sal_handle_, context_handle_);
  print_event_handler_->Print(name_pretty_, "sal_handle_ is initialized!");
  InitializeCameraPorts(camera_ports_, count_camera_, sal_handle_, device_arguments_);
  print_event_handler_->Print(name_pretty_, "camera_ports_ is initialized!");
  WorkIt();
  print_event_handler_->Print(name_pretty_, "Constructor has finished.");
}

void DriveWorksApi::InitializeContextHandle(dwContextHandle_t &context_handle) {
  dwContextParameters context_parameters;
  memset(&context_parameters, 0, sizeof(dwContextParameters));
  dwInitialize(&context_handle, DW_VERSION, &context_parameters);
}

void DriveWorksApi::InitializeSalHandle(dwSALHandle_t &sal_handle,
                                        const dwContextHandle_t &context_handle) {
  dwStatus result;
  result = dwSAL_initialize(&sal_handle, context_handle);
  if (result != DW_SUCCESS) {
    std::cerr << "Cannot initialize SAL: " << dwGetStatusName(result)
              << std::endl;
    exit(1);
  }
}

void
DriveWorksApi::InitializeCameraPorts(std::vector<CameraPort::Ptr> &camera_ports,
                                     int &count_cameras,
                                     const dwSALHandle_t &sal,
                                     const DeviceArguments &device_arguments) {
  print_event_handler_->Print(name_pretty_, "InitializeCameraPorts is called!");

  dwStatus result;

  int idx = 0;
  int cnt[4] = {0, 0, 0, 0};
  std::string ports = "aceg";
  std::string links;

  std::string selector = device_arguments.get("selector_mask");

    // reading the mask selector to determine port and camera position
    for (size_t i = 0; i < selector.length() && i < 16; i++, idx++) {
    const char s = selector[i];
    if (s == '1') {
      cnt[idx / 4]++;
      links += std::to_string(idx % 4);
      count_cameras += 1;
    }
  }
  print_event_handler_->Print(name_pretty_, "Number of cameras in port A " + std::to_string(cnt[0]));
  print_event_handler_->Print(name_pretty_, "Number of cameras in port B " + std::to_string(cnt[1]));
  print_event_handler_->Print(name_pretty_, "Number of cameras in port C " + std::to_string(cnt[2]));
  print_event_handler_->Print(name_pretty_, "Number of cameras in port D " + std::to_string(cnt[3]));
  print_event_handler_->Print(name_pretty_, "Total cameras " + std::to_string(count_cameras) + " Cameras position " + links);
  
   // initialize all the cameras 
  for (size_t i = 0; i < 4; i++){ //check each port
    for (size_t j = 0; j < cnt[i]; j++){     //check each camera  
      std::string params;
      params += "interface=csi-" + std::string(1, ports[i]);
      params += ",camera-name=" + device_arguments.get("type-"+std::string(1, ports[i]));
      params += ",link=" + std::string(1, links[0]);
      params += ", output-format=processed";
      //camera-name=SF3324,interface=csi-a,link=0,output-format=processed
      std::cout << "DEBUG ARGS PORT:  " << std::string(1, ports[i]) << std::endl;
      std::cout << "Params: " << params << std::endl;

      dwSensorHandle_t sensor_handle = DW_NULL_HANDLE;
      dwSensorParams sensor_params;
      sensor_params.parameters = params.c_str();
      sensor_params.protocol = "camera.gmsl";

      result = dwSAL_createSensor(&sensor_handle, sensor_params, sal);
      if (result != DW_SUCCESS) {
        std::cerr << "Cannot create driver: " << sensor_params.protocol
                  << " with params: " << sensor_params.parameters << std::endl
                  << "Error: " << dwGetStatusName(result) << std::endl;

        if (result == DW_INVALID_ARGUMENT) {
          std::cerr << "It is possible the given camera is not supported. "
                    << "Please refer to the documentation for this sample."
                    << std::endl;
        }
        exit(-1);
      }

      std::string video_file = "port-"+std::string(1, ports[i]) + "_camera-"+std::string(1, links[0])+".h264"; 

      CameraPort::Ptr camera_port =
      std::make_shared<CameraPort>(sensor_handle,
                                   debug_mode_,
                                   std::string(1, ports[i]),
                                   std::string(1, links[0]),
                                   pub_image_config_.camerainfo_folder,
                                   print_event_handler_);
      
      camera_ports.push_back(camera_port);
      links.erase(0,1);
    }
  }
  if (camera_ports.empty()){
    std::cout << "camera_ports.empty()" << std::endl;
    exit(EXIT_FAILURE);
  }

  for (auto &camera_port :camera_ports){
    dwStatus status = camera_port->Start(context_handle_);
    if (status != DW_SUCCESS) {
      std::cerr << "camera_port.Start failed " << std::endl;
      exit(EXIT_FAILURE);
    }
  }
  
  std::cout << "camera_ports Start succeded" << std::endl;
}

void DriveWorksApi::WorkIt() {
  print_event_handler_->Print(name_pretty_, "WorkIt is called!");

  auto ProcessCameraStreams = [&]() {
    while (is_running_) {
      for (auto &camera_port  : camera_ports_) {
        camera_port->ProcessCameraStreams(std::ref(is_running_), context_handle_);
      }
    }
  };

  cameraStreamsFuture_ = std::async(std::launch::async,
                                    ProcessCameraStreams);

  std::cout << "WorkIt is over." << std::endl;
}

void DriveWorksApi::Shutdown() {
  print_event_handler_->Print(name_pretty_, "Shutdown is called!");

  is_running_ = false;
  print_event_handler_->Print(name_pretty_, "is_running_ is false now!");

  cameraStreamsFuture_.get();

  for (auto &camera : camera_ports_) {
    camera.reset();
  }
  print_event_handler_->Print(name_pretty_, "All camera_ports are reset.");

  dwSAL_release(sal_handle_);
  print_event_handler_->Print(name_pretty_, "dwSAL_release");

  dwRelease(context_handle_);
  print_event_handler_->Print(name_pretty_, "dwRelease");

  dwLogger_release();
  print_event_handler_->Print(name_pretty_, "dwLogger_release");
}

DriveWorksApi::~DriveWorksApi() {
  print_event_handler_->Print(name_pretty_, "Destructor is called!");
  Shutdown();
  print_event_handler_->Print(name_pretty_, "Destructor is finished!");
}
}
