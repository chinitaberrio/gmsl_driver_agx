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

namespace DriveWorks {
  DriveWorksApi::DriveWorksApi(DeviceArguments arguments,
                               ImageConfigPub pub_image_config) :
    device_arguments_(std::move(arguments)),
    pub_image_config_(std::move(pub_image_config)),
    debug_mode_(false) {
    std::cout << "DriveWorksApi::DriveWorksApi is called!" << std::endl;

    print_event_handler_ = std::make_shared<PrintEventHandler>();

    InitializeContextHandle(context_handle_);
    InitializeSalHandle(sal_handle_, context_handle_);
    InitializeCameraPorts(camera_ports_, count_camera_, sal_handle_, device_arguments_);
    WorkIt();
    std::cout << "DriveWorksApi constructor has finished." << std::endl;
  }

  void DriveWorksApi::InitializeContextHandle(dwContextHandle_t &context_handle) {
    std::cout << "InitializeContextHandle is called!" << std::endl;
    dwContextParameters context_parameters;
    memset(&context_parameters, 0, sizeof(dwContextParameters));
    dwInitialize(&context_handle, DW_VERSION, &context_parameters);
  }

  void DriveWorksApi::InitializeSalHandle(dwSALHandle_t &sal_handle,
                                          const dwContextHandle_t &context_handle) {
    std::cout << "InitializeSalHandle is called!" << std::endl;
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
    std::cout << "InitializeCameras is called!" << std::endl;
    std::string selector = device_arguments.get("selector_mask");
    dwStatus result;
    // Identify active ports
    int idx = 0;
    int cnt[4] = {0, 0, 0, 0};
    std::string port[4] = {"a", "b", "c", "d"};
    for (size_t i = 0; i < selector.length() && i < 16; i++, idx++) {
      const char s = selector[i];
      if (s == '1') {
        cnt[idx / 4]++;
      }
    }

    count_cameras = 0;
    // Iterate through camera ports
    for (size_t p = 0; p < 4; p++) {
      if (cnt[p] <= 0) {
        std::cout << "GMSL port #" << p << "is empty." << std::endl;
        continue;
      }

      std::string params;
      params += "camera-group=" + port[p];
      params += ",camera-type=" +
                device_arguments.get("type-" + port[p]);
      params += ",camera-count=4"; // when using the mask, just ask for all cameras, mask will select properly

      if (selector.size() >= p * 4) {
        params += ",camera-mask=" + selector.substr(p * 4, std::min(
          selector.size() - p * 4, size_t{4}));
      }

      params += ",slave=" + device_arguments.get("slave");
      params += ",cross-csi-sync=" + device_arguments.get("cross_csi_sync");
      params += ",fifo-size=" + device_arguments.get("fifo_size");

      std::cout << "DEBUG ARGS PORT:  " << p << std::endl;
      std::cout << params << std::endl;

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
        continue;
      }

      CameraPort::Ptr camera_port = std::make_shared<CameraPort>(sensor_handle,
                                                                 debug_mode_,
                                                                 p,
                                                                 pub_image_config_.camerainfo_folder);
      camera_ports.push_back(camera_port);
      count_cameras += camera_port->GetSiblingCount();
    }
    std::cout << "Camera count: " << count_cameras << std::endl;

    if (camera_ports.empty()) {
      std::cout << "camera_ports.empty()" << std::endl;
      exit(EXIT_FAILURE);
    }

    for (auto &camera_port : camera_ports) {
      dwStatus status = camera_port->Start(context_handle_);
      if (status != DW_SUCCESS) {
        std::cerr << "camera_port.Start failed " << std::endl;
        exit(EXIT_FAILURE);
      }
    }
    std::cout << "camera_ports Start succeded" << std::endl;
    is_running_ = true;
  }

  void DriveWorksApi::WorkIt() {
    // Start Image Publisher Consumers
    for (auto &camera_port : camera_ports_) {
      camera_port->StartConsumers(is_running_);
    }
    // Start Camera Read Producer
    std::vector<std::shared_future<void>> future_producers;
    for (auto &camera_port  : camera_ports_) {
      future_producers.push_back(camera_port->StartProducer(is_running_, context_handle_));
    }

    for (const auto &future : future_producers) {
      future.wait();
    }
    std::cout << "Wait is over." << std::endl;
  }

  void DriveWorksApi::Shutdown() {
    std::cout << "Driveworks Shutdown has started!" << std::endl;
    is_running_ = false;
    for (auto &camera : camera_ports_) {
      camera->CleanUp();
    }
    dwSAL_release(sal_handle_);
    dwRelease(context_handle_);
    dwLogger_release();
  }

}
