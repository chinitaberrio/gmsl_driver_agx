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

namespace DriveWorks {
  DriveWorksApi::DriveWorksApi(DeviceArguments arguments,
                               ImageConfigPub pub_image_config) :
    device_arguments_(std::move(arguments)),
    pub_image_config_(std::move(pub_image_config)) {
    std::cout << "DriveWorksApi::DriveWorksApi is called!" << std::endl;
    is_running_ = true;

    InitializeContextHandle(context_handle_);
    InitializeSalHandle(sal_handle_, context_handle_);
    InitializeCameraPorts(camera_ports_, count_camera_, sal_handle_, device_arguments_);
    StartCameraPortWorkers();
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
  DriveWorksApi::InitializeCameraPorts(std::vector<CameraPort> &camera_ports,
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

      dwImageProperties image_properties;
      dwSensorCamera_getImageProperties(&image_properties,
                                        DW_CAMERA_OUTPUT_NATIVE_PROCESSED,
                                        sensor_handle);

      dwCameraProperties camera_properties;
      dwSensorCamera_getSensorProperties(&camera_properties, sensor_handle);

      CameraPort camera_port(sensor_handle, image_properties, camera_properties);
      camera_ports.push_back(camera_port);
      count_cameras += camera_port.GetSiblingCount();
    }
    std::cout << "Camera count: " << count_cameras << std::endl;

    if (camera_ports.empty()) {
      std::cout << "camera_ports.empty()" << std::endl;
      exit(EXIT_FAILURE);
    }

    for (auto &camera_port : camera_ports) {
      dwStatus status = camera_port.Start(context_handle_);
      if (status != DW_SUCCESS) {
        std::cerr << "camera_port.Start failed " << std::endl;
        exit(EXIT_FAILURE);
      }
    }
    std::cerr << "camera_ports Start succeded" << std::endl;
    is_running_ = true;
  }

  void DriveWorksApi::StartCameraPortWorkers() {
    std::cout << "StartCameraPortWorkers is called!" << std::endl;
    std::vector<std::thread> camThreads;
    for (uint32_t i = 0; i < 1 ++i) {
//    for (uint32_t i = 0; i < camera_ports_.size(); ++i) {
      std::thread worker = std::thread(&DriveWorksApi::WorkerPortPipeline,
                                       this,
                                       std::ref(camera_ports_[i]),
                                       i,
                                       context_handle_);
      worker.detach();
    }
  }


  void DriveWorksApi::WorkerPortPipeline(CameraPort &camera_port,
                                         uint32_t port,
                                         const dwContextHandle_t &sdk) {
    std::cout << "Start worker for port: " << port << std::endl;
    // cv publishers
    std::vector<std::unique_ptr<OpenCVConnector>> cv_connectors;
    // init multiple cv cameras connection and topic name
    for (uint32_t cameraIdx = 0;
         cameraIdx < camera_port.GetSiblingCount(); cameraIdx++) {
      // Topic mapping e.g. gmsl_image_raw_<nvidia cam port A=0, B=1, C=2>_<sibling id 0,1,2,3> : port_0/camera_1/(image_raw,image_raw/compressed)
      const std::string topic =
        std::string("port_") + std::to_string(port) + std::string("/camera_") +
        std::to_string(cameraIdx);
      const std::string camera_frame_id =
        std::string("port_") + std::to_string(port) + std::string("/camera_") +
        std::to_string(cameraIdx);
      const std::string cam_info_file =
        std::string("file://") + std::string(pub_image_config_.camerainfo_folder) +
        std::to_string(port) + std::to_string(cameraIdx) +
        std::string("_calibration.yml");
      std::unique_ptr<OpenCVConnector> cvPtr(
        new OpenCVConnector(topic, camera_frame_id, cam_info_file, 10));
      cv_connectors.push_back(std::move(cvPtr));
    }
    std::cout << "CvConnectors are initialized for port: " << port << std::endl;

    while (is_running_ && ros::ok()) {
      // capture from all csi-ports
      // NOTE if cross-csi-synch is active, all cameras will capture at the same time
      std::cout << "bef camera_port.ReadFrames for port: " << port << std::endl;
      camera_port.ReadFrames(context_handle_);
      std::cout << "aft camera_port.ReadFrames for port: " << port << std::endl;

      for (uint32_t cameraIdx = 0; cameraIdx < camera_port.GetSiblingCount(); cameraIdx++) {
        dwImageNvMedia *frameNVMrgba;
        CameraPort::Camera& camera = camera_port.Cameras[cameraIdx];

        dwImage_getNvMedia(&frameNVMrgba, camera.ImageHandle);
        std::cout << "aft dwImage_getNvMedia for port: " << port << std::endl;
        NvMediaImageSurfaceMap surfaceMap;

        if (NvMediaImageLock(frameNVMrgba->img, NVMEDIA_IMAGE_ACCESS_READ,
                             &surfaceMap) == NVMEDIA_STATUS_OK) {
          // publish an image
          if (pub_image_config_.is_compressed) {
            // compressed
            cv_connectors[cameraIdx]->WriteToJpeg(
              camera.JpegImage,
              camera.CountByteJpeg);
          } else {
            //raw (resize if set)
            cv_connectors[cameraIdx]->WriteToOpenCV(
              (unsigned char *) surfaceMap.surface[0].mapping,
              frameNVMrgba->prop.width, frameNVMrgba->prop.height,
              pub_image_config_.image_width,
              pub_image_config_.image_height);
          }

          NvMediaImageUnlock(frameNVMrgba->img);
        } else {
          std::cout << "CANNOT LOCK NVMEDIA IMAGE - NO SCREENSHOT\n";
        }
        std::cout << "aft NvMediaImageUnlock for port: " << port << std::endl;
      }
      std::cout << "aft Publishing stuff for port: " << port << std::endl;
    }
  }

  void DriveWorksApi::releaseCameras(CameraPort *cameraSensor) {
    // release sensor
    std::cout << "Cleaning camera thread .. " << std::endl;
    {
      dwSensor_stop(cameraSensor->GetSensorHandle());
      std::cout << "Cleaning camera thread .. dwSensor " << std::endl;
      dwSAL_releaseSensor(cameraSensor->GetSensorHandle());
      std::cout << "Cleaning camera thread .. dwSAL " << std::endl;
//    dwImageFormatConverter_release(&cameraSensor->yuv2rgba);
      std::cout << "Cleaning camera thread .. dwConvert " << std::endl;
    }

    for (auto &camera : cameraSensor->Cameras) {
      dwStatus result = dwImage_destroy(camera.ImageHandle);
      if (result != DW_SUCCESS) {
        std::cerr << "Cannot destroy nvmedia: " << dwGetStatusName(result)
                  << std::endl;
        is_running_ = false;
        break;
      }
      NvMediaIJPEDestroy(camera.NvMediaIjpe);
    }
  }

  void DriveWorksApi::releaseSDK() {
    // release sdk and sal
    // release used objects in correct order
    std::cout << "Release SDK .." << std::endl;
    dwSAL_release(sal_handle_);
    dwRelease(context_handle_);
    dwLogger_release();
  }

  void DriveWorksApi::stopCameras() {
    std::cout << "Stop camera... " << std::endl;
    // loop through all camera ports to cleanup all connected cameras
    for (auto &camera : camera_ports_) {
      releaseCameras(&camera);
    }
    // sdk instances release
    releaseSDK();
    // set init and run state
    is_running_ = false;
  }

  bool DriveWorksApi::isCamReady() {
    return is_running_;
  }

}
