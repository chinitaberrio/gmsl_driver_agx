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
    InitializeCameras(cameras_, count_camera_, sal_handle_, device_arguments_);
    // Init image frames and start camera image acquisition
    initFramesStart();
    // Set values
    g_numPort = cameras_.size();
    // Set done init state
    g_initState = true;
    // Start image publishing thread
    startCameraPipline();
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
  DriveWorksApi::InitializeCameras(std::vector<CameraPort> &camera_ports,
                                   int &numCameras,
                                   const dwSALHandle_t &sal,
                                   const DeviceArguments &device_arguments) {
    std::cout << "InitializeCameras is called!" << std::endl;
    std::string selector = device_arguments.get("selector_mask");
    dwStatus result;
    // identify active ports
    int idx = 0;
    int cnt[4] = {0, 0, 0, 0};
    std::string port[4] = {"a", "b", "c", "d"};
    for (size_t i = 0; i < selector.length() && i < 16; i++, idx++) {
      const char s = selector[i];
      if (s == '1') {
        cnt[idx / 4]++;
      }
    }

    // Parse Arguments
    numCameras = 0;
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
      numCameras += camera_port.GetSiblingCount();
    }
  }

  void DriveWorksApi::initFramesStart() {
    std::cout << "Init Camera Frames .. " << std::endl;
    // check cameras connected to csi ports
    if (cameras_.size() == 0) {
      std::cerr << "Need to specify at least 1 at most 12 cameras to be used"
                << std::endl;
      exit(-1);
    }

    // allocate frameRGBA pointer
    for (size_t csiPort = 0; csiPort < cameras_.size(); csiPort++) {
      std::vector<dwImageHandle_t *> pool;
      std::vector<uint8_t *> pool_jpeg;
      std::vector<uint32_t> poolsize;
      for (size_t cameraIdx = 0;
           cameraIdx < cameras_[csiPort].GetSiblingCount(); ++cameraIdx) {
        pool.push_back(nullptr);
        pool_jpeg.push_back(nullptr);
        poolsize.push_back(0);
      }
      // assign to class variables for later use
      g_frameRGBAPtr.push_back(pool);
      g_frameJPGPtr.push_back(pool_jpeg);
      g_compressedSize.push_back(poolsize);

    }

    // init RGBA frames for each camera in the port(e.g. 4 cameras/port)
    for (size_t csiPort = 0; csiPort < cameras_.size() && is_running_; csiPort++) {
      initFrameImage(&cameras_[csiPort]);
      // record a number of connected camera
      g_numCameraPort.push_back(cameras_[csiPort].GetSiblingCount());
    }
  }

  void DriveWorksApi::initFrameImage(CameraPort *camera) {
    std::cout << "Init Camera Frame Pools .. " << std::endl;
    // RGBA image pool for conversion from YUV camera output
    // two RGBA frames per camera per sibling for a pool
    // since image streamer might hold up-to one frame when using egl stream
    dwStatus result;
    int32_t pool_size = 2;
    uint32_t numFramesRGBA = pool_size * camera->GetSiblingCount();

    // temp variable for easy access and de-reference back to camera->frameRGBA in releasing nvidia image frame read
    std::vector<dwImageHandle_t> &g_frameRGBA = camera->frameRGBA;

    g_frameRGBA.reserve(numFramesRGBA);
    {
      dwImageProperties cameraImageProperties;
      dwSensorCamera_getImageProperties(&cameraImageProperties,
                                        DW_CAMERA_OUTPUT_NATIVE_PROCESSED,
                                        camera->GetSensorHandle());
      dwImageProperties displayImageProperties = cameraImageProperties;
      displayImageProperties.format = DW_IMAGE_FORMAT_RGBA_UINT8;
      displayImageProperties.type = DW_IMAGE_NVMEDIA;

      // allocate image pool
      for (uint32_t cameraIdx = 0;
           cameraIdx < camera->GetSiblingCount(); cameraIdx++) {
        for (int32_t k = 0; k < pool_size; k++) {
          dwImageHandle_t rgba{};
          result = dwImage_create(&rgba, displayImageProperties, context_handle_);
          if (result != DW_SUCCESS) {
            std::cerr << "Cannot create nvmedia image for pool:"
                      << dwGetStatusName(result) << std::endl;
            is_running_ = false;
            break;
          }
          // don't forget to delete dwImage via g_frameRGBA when exit
          g_frameRGBA.push_back(rgba);
          camera->rgbaPool.push(&g_frameRGBA.back());
        }
      }


      // NVMedia image compression definition.
      for (uint32_t cameraIdx = 0;
           cameraIdx < camera->GetSiblingCount(); cameraIdx++) {
        NvMediaDevice *device;
        device = NvMediaDeviceCreate();
        if (!device) {
          std::cerr << "main: NvMediaDeviceCreate failed\n" << std::endl;
          is_running_ = false;
        }
        NvMediaIJPE *jpegEncoder = NULL;
        NvMediaSurfFormatAttr attrs[7];
        NVM_SURF_FMT_SET_ATTR_YUV(attrs, YUV, 422, PLANAR, UINT, 8, PL);
        NvMediaSurfaceType surface_type = NvMediaSurfaceFormatGetType(attrs, 7);
        jpegEncoder = NvMediaIJPECreate(device,
                                        surface_type,
                                        (uint8_t) 1, max_jpeg_bytes);
        if (!jpegEncoder) {
          std::cerr << "main: NvMediaIJPECreate failed\n" << std::endl;
          is_running_ = false;
        } else {
          camera->jpegEncoders.push_back(jpegEncoder);
        }
      }
      // allocate compressed image pool
      for (uint32_t cameraIdx = 0;
           cameraIdx < camera->GetSiblingCount(); cameraIdx++) {
        for (int32_t k = 0; k < pool_size; k++) {
          uint8_t *jpeg_img = (uint8_t *) malloc(max_jpeg_bytes);
          camera->jpegPool.push(jpeg_img);
        }
      }
      // start camera capturing
      is_running_ = is_running_ && dwSensor_start(camera->GetSensorHandle()) == DW_SUCCESS;
      eof = false;
    }
  }


  void DriveWorksApi::startCameraPipline() {
    std::cout << "Start camera pipline  " << std::endl;
    std::vector<std::thread> camThreads;
    for (uint32_t i = 0; i < cameras_.size(); ++i) {
      camThreads.push_back(
        std::thread(&DriveWorksApi::WorkerPortPipeline, this, &cameras_[i], i,
                    context_handle_));
    }

    // start camera threads and release
    for (uint32_t i = 0; i < cameras_.size(); ++i) {
      camThreads.at(i).detach();
    }
  }


  void DriveWorksApi::WorkerPortPipeline(CameraPort *cameraSensor, uint32_t port,
                                         dwContextHandle_t sdk) {
    std::cout << "Start worker for port: " << port << std::endl;
    // cv publishers
    std::vector<std::unique_ptr<OpenCVConnector>> cv_connectors;
    // init multiple cv cameras connection and topic name
    for (uint32_t cameraIdx = 0;
         cameraIdx < cameraSensor->GetSiblingCount(); cameraIdx++) {
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

    while (is_running_ && ros::ok()) {
      bool eofAny = false;
      // capture from all csi-ports
      // NOTE if cross-csi-synch is active, all cameras will capture at the same time
      {
        if (eof) {
          eofAny = true;
          continue;
        }

        if (cameraSensor->rgbaPool.empty()) {
          std::cerr << "Ran out of RGBA buffers, continuing" << std::endl;
          continue;
        }

        // capture from all cameras within a csi port
        for (uint32_t cameraIdx = 0; cameraIdx < cameraSensor->GetSiblingCount() &&
                                     !cameraSensor->rgbaPool.empty(); cameraIdx++) {
          // capture, convert to rgba and return it
          eof = captureCamera(cameraSensor->rgbaPool.front(),
                              cameraSensor->GetSensorHandle(),
                              port, cameraIdx,
                              cameraSensor->jpegPool.front(),
                              cameraSensor->jpegEncoders[cameraIdx]);
          g_frameRGBAPtr[port][cameraIdx] = cameraSensor->rgbaPool.front();
          cameraSensor->rgbaPool.pop();

          g_frameJPGPtr[port][cameraIdx] = cameraSensor->jpegPool.front();
          cameraSensor->jpegPool.pop();

          if (!eof) {
            cameraSensor->rgbaPool.push(g_frameRGBAPtr[port][cameraIdx]);
            cameraSensor->jpegPool.push(g_frameJPGPtr[port][cameraIdx]);
          }

          eofAny |= eof;
        }
      }

      // stop to take screenshot (will cause a delay)
      if (gTakeScreenshot) {
        {

          for (uint32_t cameraIdx = 0; cameraIdx < cameraSensor->GetSiblingCount() &&
                                       !cameraSensor->rgbaPool.empty(); cameraIdx++) {
            //copy to memory replacing by //takeScreenshot(g_frameRGBAPtr[port][cameraIdx], port, cameraIdx);

            dwImageNvMedia *frameNVMrgba;
            dwImage_getNvMedia(&frameNVMrgba, *g_frameRGBAPtr[port][cameraIdx]);
            NvMediaImageSurfaceMap surfaceMap;

            if (NvMediaImageLock(frameNVMrgba->img, NVMEDIA_IMAGE_ACCESS_READ,
                                 &surfaceMap) == NVMEDIA_STATUS_OK) {
              // publish an image
              if (pub_image_config_.is_compressed) {
                // compressed
                cv_connectors[cameraIdx]->WriteToJpeg(
                  g_frameJPGPtr[port][cameraIdx],
                  g_compressedSize[port][cameraIdx]);
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

          }
        }
        gScreenshotCount++;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
      is_running_ = is_running_ && !eofAny;
    }//end while
  }

  dwStatus DriveWorksApi::captureCamera(dwImageHandle_t *frameNVMrgba,
                                        dwSensorHandle_t cameraSensor,
                                        uint32_t port,
                                        uint32_t sibling,
                                        uint8_t *jpeg_image,
                                        NvMediaIJPE *jpegEncoder) {
    dwCameraFrameHandle_t frameHandle;
    dwImageHandle_t frameNVMyuv;

    dwStatus result = DW_FAILURE;
    result = dwSensorCamera_readFrame(&frameHandle, sibling, 300000, cameraSensor);
    if (result != DW_SUCCESS) {
      std::cerr << "readFrameNvMedia: " << dwGetStatusName(result) << std::endl;
      return result;
    }

    result = dwSensorCamera_getImage(&frameNVMyuv, DW_CAMERA_OUTPUT_NATIVE_PROCESSED, frameHandle);
    if (result != DW_SUCCESS) {
      std::cerr << "readImageNvMedia: " << dwGetStatusName(result) << std::endl;
    }

//    dwImageHandle_t frameNVrgba;
//    dwImageProperties rgbaImageProperties{};
//    dwSensorCamera_getImageProperties(&rgbaImageProperties, DW_CAMERA_OUTPUT_NATIVE_PROCESSED, cameraSensor);
//    rgbaImageProperties.format = DW_IMAGE_FORMAT_RGBA_UINT8;
//    dwImage_create(&frameNVrgba, rgbaImageProperties, sdk);

    result = dwImage_copyConvert(*frameNVMrgba, frameNVMyuv, context_handle_);

    if (result != DW_SUCCESS) {
      std::cerr << "copyConvertNvMedia: " << dwGetStatusName(result) << std::endl;
    }

//    result = dwImage_getNvMedia(&frameNVMrgba,frameNVrgba);
//
//    if( result != DW_SUCCESS )
//    {
//      std::cerr << "dwImage_getNvMedia:frameNVMrgba: " << dwGetStatusName(result) << std::endl;
//    }

    dwImageNvMedia *frameyuv_nvm;
    result = dwImage_getNvMedia(&frameyuv_nvm, frameNVMyuv);

    if (result != DW_SUCCESS) {
      std::cerr << "dwImage_getNvMedia:frameyuv_nvm: " << dwGetStatusName(result) << std::endl;
    }

    if (pub_image_config_.is_compressed) {
      NvMediaStatus nvStatus = NvMediaIJPEFeedFrame(jpegEncoder, frameyuv_nvm->img, pub_image_config_.jpeg_quality);
      if (nvStatus != NVMEDIA_STATUS_OK) {
        std::cerr << "NvMediaIJPEFeedFrameQuality failed: %x\n" << nvStatus << std::endl;
      }
      nvStatus = NvMediaIJPEBitsAvailable(jpegEncoder, &g_compressedSize[port][sibling], NVMEDIA_ENCODE_BLOCKING_TYPE_IF_PENDING, 10000);
      nvStatus = NvMediaIJPEGetBits(jpegEncoder, &g_compressedSize[port][sibling], jpeg_image, 0);
      if (nvStatus != NVMEDIA_STATUS_OK && nvStatus != NVMEDIA_STATUS_NONE_PENDING) {
        std::cerr << "main: Error getting encoded bits\n" << std::endl;
      }
    }
    result = dwSensorCamera_returnFrame(&frameHandle);
    if (result != DW_SUCCESS) {
      std::cout << "returnFrameNvMedia: " << dwGetStatusName(result) << std::endl;
    }

    return DW_SUCCESS;
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

    // cleanup nvmedia preallocate image frames
    for (dwImageHandle_t &frame : cameraSensor->frameRGBA) {
      dwStatus result = dwImage_destroy(frame);
      if (result != DW_SUCCESS) {
        std::cerr << "Cannot destroy nvmedia: " << dwGetStatusName(result)
                  << std::endl;
        is_running_ = false;
        break;
      }
    }

    // cleanup jpeg compression
    for (auto jpegEncoder_ : cameraSensor->jpegEncoders) {
      NvMediaIJPEDestroy(jpegEncoder_);
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
    for (size_t csiPort = 0; csiPort < cameras_.size(); csiPort++) {
      releaseCameras(&cameras_[csiPort]);
    }
    // sdk instances release
    releaseSDK();
    // set init and run state
    g_exitCompleted = true;
    is_running_ = false;
    g_initState = false;
  }

  bool DriveWorksApi::isCamReady() {
    return is_running_ && g_initState;
  }

  uint32_t DriveWorksApi::getNumPort() {
    return g_numPort;
  }

  std::vector<uint32_t> DriveWorksApi::getCameraPort() {
    return g_numCameraPort;
  }

  bool DriveWorksApi::isShutdownCompleted() {
    return g_exitCompleted && !is_running_;
  }

}
