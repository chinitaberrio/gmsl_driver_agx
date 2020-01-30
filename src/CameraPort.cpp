#include "CameraPort.h"
#include <dw/core/Status.h>
#include <iostream>
#include <StopWatch.h>
#include <thread>


namespace DriveWorks {
  CameraPort::CameraPort(dwSensorHandle_t sensor_handle,
                         dwImageProperties image_properties,
                         dwCameraProperties camera_properties,
                         bool debug_mode,
                         int port,
                         const std::string &caminfo_folder)
    : debug_mode(debug_mode),
      sensor_handle(sensor_handle),
      image_properties(image_properties),
      camera_properties(camera_properties),
      port(port) {
    Cameras.resize(GetSiblingCount());
    for (uint32_t cameraIdx = 0; cameraIdx < GetSiblingCount(); cameraIdx++) {
      // Topic mapping e.g. gmsl_image_raw_<nvidia cam port A=0, B=1, C=2>_<sibling id 0,1,2,3> : port_0/camera_1/(image_raw,image_raw/compressed)
      const std::string topic =
        std::string("port_") + std::to_string(port) + std::string("/camera_") +
        std::to_string(cameraIdx);
      const std::string camera_frame_id =
        std::string("port_") + std::to_string(port) + std::string("/camera_") +
        std::to_string(cameraIdx);
      const std::string cam_info_file =
        std::string("file://") + caminfo_folder +
        std::to_string(port) + std::to_string(cameraIdx) +
        std::string("_calibration.yml");
      Camera &camera = Cameras[cameraIdx];
      camera.OpenCvConnector = std::make_shared<OpenCVConnector>(topic, camera_frame_id, cam_info_file, 10);
      camera.QueueImageHandles = std::make_shared<folly::ProducerConsumerQueue<dwImageHandle_t>>(5);
    }


  }

  dwStatus CameraPort::Start(const dwContextHandle_t &context_handle) {
    dwImageProperties cameraImageProperties;
    dwStatus status = dwSensorCamera_getImageProperties(&cameraImageProperties,
                                                        DW_CAMERA_OUTPUT_NATIVE_PROCESSED,
                                                        sensor_handle);
    if (status != DW_SUCCESS) {
      std::cerr << "dwSensorCamera_getImageProperties:"
                << dwGetStatusName(status) << std::endl;
      return status;
    }
    dwImageProperties displayImageProperties = cameraImageProperties;
    displayImageProperties.format = DW_IMAGE_FORMAT_RGBA_UINT8;
    displayImageProperties.type = DW_IMAGE_NVMEDIA;
    for (size_t i = 0; i < GetSiblingCount(); ++i) {
      Camera &camera = Cameras[i];
      const uint32_t max_jpeg_bytes = 3 * 1290 * 1208;
      camera.JpegImage = (uint8_t *) malloc(max_jpeg_bytes);
      dwStatus result = dwImage_create(&camera.ImageHandle, displayImageProperties, context_handle);
      if (result != DW_SUCCESS) {
        std::cerr << "Cannot dwImage_create:"
                  << dwGetStatusName(result) << std::endl;
        return result;
      }

      camera.NvMediaDevicee = nullptr;
      camera.NvMediaDevicee = NvMediaDeviceCreate();
      if (!camera.NvMediaDevicee) {
        std::cerr << "NvMediaDeviceCreate failed." << std::endl;
        exit(EXIT_FAILURE);
      }

      camera.NvMediaIjpe = nullptr;
      NvMediaSurfFormatAttr attrs[7];
      NVM_SURF_FMT_SET_ATTR_YUV(attrs, YUV, 422, PLANAR, UINT, 8, PL);
      NvMediaSurfaceType surface_type = NvMediaSurfaceFormatGetType(attrs, 7);
      camera.NvMediaIjpe = NvMediaIJPECreate(camera.NvMediaDevicee,
                                             surface_type,
                                             (uint8_t) 1,
                                             max_jpeg_bytes);
      if (!camera.NvMediaIjpe) {
        std::cerr << "NvMediaIJPECreate failed." << std::endl;
        exit(EXIT_FAILURE);
      }

    }
    return dwSensor_start(GetSensorHandle());
  }

  int CameraPort::GetSiblingCount() {
    return camera_properties.siblings;
  }

  dwSensorHandle_t CameraPort::GetSensorHandle() const {
    return sensor_handle;
  }


  void CameraPort::ReadFramesPushImages(const dwContextHandle_t &context_handle) {
    for (int ind_camera = 0; ind_camera < 1; ind_camera++) {
//    for (int ind_camera = 0; ind_camera < Cameras.size(); ind_camera++) {
      std::cout << "Producer ReadFramesPushImages For Port: " << port
                << " Camera: " << ind_camera << std::endl;
      dwCameraFrameHandle_t camera_frame_handle;
      dwStatus status;
      status = dwSensorCamera_readFrame(&camera_frame_handle, ind_camera, DW_TIMEOUT_INFINITE, sensor_handle);
      if (status != DW_SUCCESS) {
        std::cerr << "dwSensorCamera_readFrame: " << dwGetStatusName(status) << std::endl;
        continue;
      }
      std::cout << "Producer dwSensorCamera_readFrame For Port: " << port
                << " Camera: " << ind_camera << std::endl;
      dwImageHandle_t image_handle_yuv;
      status = dwSensorCamera_getImage(&image_handle_yuv, DW_CAMERA_OUTPUT_NATIVE_PROCESSED, camera_frame_handle);
      if (status != DW_SUCCESS) {
        std::cerr << "dwSensorCamera_getImage: " << dwGetStatusName(status) << std::endl;
      }
      std::cout << "Producer dwSensorCamera_getImage For Port: " << port
                << " Camera: " << ind_camera << std::endl;

      dwImageHandle_t image_handle;

      dwStatus result = dwImage_create(&image_handle, image_properties, context_handle);

      status = dwImage_copyConvert(image_handle, image_handle_yuv, context_handle);
      if (status != DW_SUCCESS) {
        std::cerr << "dwImage_copyConvert: " << dwGetStatusName(status) << std::endl;
      }
      std::cout << "Producer dwImage_copyConvert For Port: " << port
                << " Camera: " << ind_camera << std::endl;


      status = dwSensorCamera_returnFrame(&camera_frame_handle);
      if (status != DW_SUCCESS) {
        std::cout << "dwSensorCamera_returnFrame: " << dwGetStatusName(status) << std::endl;
      }
      std::cout << "Producer dwSensorCamera_returnFrame For Port: " << port
                << " Camera: " << ind_camera << std::endl;

      Camera &camera = Cameras[ind_camera];

      std::cerr << "queue current size: " << camera.QueueImageHandles->sizeGuess() << std::endl;

      while (!camera.QueueImageHandles->write(image_handle)) {
        std::cerr << "queue is full, current size: " << camera.QueueImageHandles->sizeGuess() << std::endl;
      }
      std::cout << "Producer write_is_successfull For Port: " << port
                << " Camera: " << ind_camera << std::endl;

      result = dwImage_destroy(image_handle_yuv);
      if (result != DW_SUCCESS) {
        std::cerr << "dwImage_destroy: " << dwGetStatusName(result) << std::endl;
      }
    }
  }


  void CameraPort::StartConsumers(const bool &is_running) {
    for (size_t ind_camera = 0; ind_camera < GetSiblingCount(); ++ind_camera) {
      std::cout << "Starting Consumer For Port: " << port
                << " Camera: " << ind_camera << std::endl;
      Camera &camera = Cameras[ind_camera];
      camera.future = std::async(std::launch::async,
                                 &CameraPort::ConsumeImagesPublishMessages,
                                 this,
                                 std::ref(is_running),
                                 ind_camera);
    }
  }

  void CameraPort::ConsumeImagesPublishMessages(const bool &is_running,
                                                int ind_camera) {
    Camera &camera = Cameras[ind_camera];
    while (is_running) {
      dwImageHandle_t image_handle;
      while (!camera.QueueImageHandles->read(image_handle)) {
        //spin until we get a value
        std::this_thread::sleep_for(std::chrono::microseconds(50));
      }
      std::cout << "Consumer QueueImageHandles->read For Port: " << port
                << " Camera: " << ind_camera << std::endl;

      dwImageNvMedia *image_nvmedia;
      camera.ReadingResult = dwImage_getNvMedia(&image_nvmedia, image_handle);
      if (camera.ReadingResult != DW_SUCCESS) {
        std::cerr << "dwImage_getNvMedia: " << dwGetStatusName(camera.ReadingResult) << std::endl;
      }
      std::cout << "Consumer dwImage_getNvMedia For Port: " << port
                << " Camera: " << ind_camera << std::endl;

      NvMediaStatus nvStatus = NvMediaIJPEFeedFrame(camera.NvMediaIjpe, image_nvmedia->img, 70);
      if (nvStatus != NVMEDIA_STATUS_OK) {
        std::cerr << "NvMediaIJPEFeedFrame failed: " << nvStatus << std::endl;
      }
      std::cout << "Consumer NvMediaIJPEFeedFrame For Port: " << port
                << " Camera: " << ind_camera << std::endl;

      nvStatus = NvMediaIJPEBitsAvailable(camera.NvMediaIjpe, &camera.CountByteJpeg, NVMEDIA_ENCODE_BLOCKING_TYPE_IF_PENDING, 10000);
      std::cout << "Consumer NvMediaIJPEBitsAvailable For Port: " << port
                << " Camera: " << ind_camera << std::endl;

      nvStatus = NvMediaIJPEGetBits(camera.NvMediaIjpe, &camera.CountByteJpeg, camera.JpegImage, 0);
      if (nvStatus != NVMEDIA_STATUS_OK && nvStatus != NVMEDIA_STATUS_NONE_PENDING) {
        std::cerr << "NvMediaIJPEGetBits failed: " << nvStatus << std::endl;
      }
      std::cout << "Consumer NvMediaIJPEGetBits For Port: " << port
                << " Camera: " << ind_camera << std::endl;

      ros::Time time = ros::Time::now();
      camera.OpenCvConnector->WriteToJpeg(
        camera.JpegImage,
        camera.CountByteJpeg,
        time);
      std::cout << "Consumer Published For Port: " << port
                << " Camera: " << ind_camera << std::endl;

      dwStatus result = dwImage_destroy(image_handle);
      if (result != DW_SUCCESS) {
        std::cerr << "dwImage_destroy: " << dwGetStatusName(result) << std::endl;
      }
      std::cout << "Consumer dwImage_destroy For Port: " << port
                << " Camera: " << ind_camera << std::endl;

    }
  }


  void CameraPort::ReadFrames(const dwContextHandle_t &context_handle) {
    StopWatch watch;
    watch.Start();
    if (debug_mode)
      std::cout << "Cameras.size(): " << Cameras.size() << std::endl;
    for (int i = 0; i < Cameras.size(); i++) {
      Camera &camera = Cameras[i];
      camera.ReadingResult = DW_FAILURE;
      dwCameraFrameHandle_t camera_frame_handle;
      if (debug_mode)
        std::cout << "bef dwSensorCamera_readFrame: " << i << std::endl;
      camera.ReadingResult = dwSensorCamera_readFrame(&camera_frame_handle, i, DW_TIMEOUT_INFINITE, sensor_handle);
      if (camera.ReadingResult != DW_SUCCESS) {
        std::cerr << "dwSensorCamera_readFrame: " << dwGetStatusName(camera.ReadingResult) << std::endl;
        continue;
      }
      std::cout << "dwSensorCamera_readFrame spent: " << watch.ElapsedMilliSeconds() << " ms." << std::endl;
      if (debug_mode)
        std::cout << "aft dwSensorCamera_readFrame: " << i << std::endl;

      dwImageHandle_t image_handle_yuv;
      camera.ReadingResult = dwSensorCamera_getImage(&image_handle_yuv, DW_CAMERA_OUTPUT_NATIVE_PROCESSED, camera_frame_handle);
      if (camera.ReadingResult != DW_SUCCESS) {
        std::cerr << "dwSensorCamera_getImage: " << dwGetStatusName(camera.ReadingResult) << std::endl;
      }

      std::cout << "dwSensorCamera_getImage spent: " << watch.ElapsedMilliSeconds() << " ms." << std::endl;
      if (debug_mode)
        std::cout << "image_handle_yuv: " << image_handle_yuv << std::endl;

      camera.ReadingResult = dwImage_copyConvert(camera.ImageHandle, image_handle_yuv, context_handle);
      if (camera.ReadingResult != DW_SUCCESS) {
        std::cerr << "dwImage_copyConvert: " << dwGetStatusName(camera.ReadingResult) << std::endl;
      }
      std::cout << "dwImage_copyConvert spent: " << watch.ElapsedMilliSeconds() << " ms." << std::endl;


      camera.ReadingResult = dwSensorCamera_returnFrame(&camera_frame_handle);
      if (camera.ReadingResult != DW_SUCCESS) {
        std::cout << "dwSensorCamera_returnFrame: " << dwGetStatusName(camera.ReadingResult) << std::endl;
      }

      std::cout << "dwSensorCamera_returnFrame spent: " << watch.ElapsedMilliSeconds() << " ms." << std::endl;
      continue;

      if (debug_mode)
        std::cout << "camera.ImageHandle: " << camera.ImageHandle << std::endl;

      dwImageNvMedia *image_nvmedia;
      camera.ReadingResult = dwImage_getNvMedia(&image_nvmedia, image_handle_yuv);
      if (camera.ReadingResult != DW_SUCCESS) {
        std::cerr << "dwImage_getNvMedia: " << dwGetStatusName(camera.ReadingResult) << std::endl;
      }

      std::cout << "dwImage_getNvMedia spent: " << watch.ElapsedMilliSeconds() << " ms." << std::endl;
      NvMediaStatus nvStatus = NvMediaIJPEFeedFrame(camera.NvMediaIjpe, image_nvmedia->img, 70);
      if (nvStatus != NVMEDIA_STATUS_OK) {
        std::cerr << "NvMediaIJPEFeedFrameQuality failed: " << nvStatus << std::endl;
      }
      std::cout << "NvMediaIJPEFeedFrame spent: " << watch.ElapsedMilliSeconds() << " ms." << std::endl;
      nvStatus = NvMediaIJPEBitsAvailable(camera.NvMediaIjpe, &camera.CountByteJpeg, NVMEDIA_ENCODE_BLOCKING_TYPE_IF_PENDING, 10000);

      std::cout << "NvMediaIJPEBitsAvailable spent: " << watch.ElapsedMilliSeconds() << " ms." << std::endl;
      nvStatus = NvMediaIJPEGetBits(camera.NvMediaIjpe, &camera.CountByteJpeg, camera.JpegImage, 0);
      if (nvStatus != NVMEDIA_STATUS_OK && nvStatus != NVMEDIA_STATUS_NONE_PENDING) {
        std::cerr << "NvMediaIJPEGetBits failed: " << nvStatus << std::endl;
      }
      std::cout << "NvMediaIJPEGetBits spent: " << watch.ElapsedMilliSeconds() << " ms." << std::endl;

      camera.ReadingResult = dwSensorCamera_returnFrame(&camera_frame_handle);
      if (camera.ReadingResult != DW_SUCCESS) {
        std::cout << "dwSensorCamera_returnFrame: " << dwGetStatusName(camera.ReadingResult) << std::endl;
      }

      std::cout << "dwSensorCamera_returnFrame spent: " << watch.ElapsedMilliSeconds() << " ms." << std::endl;
    }

  }

}