#include "CameraPort.h"
#include <dw/core/Status.h>
#include <iostream>


namespace DriveWorks {
  CameraPort::CameraPort(dwSensorHandle_t sensor_handle,
                         dwImageProperties image_properties,
                         dwCameraProperties camera_properties)
    : sensor_handle(sensor_handle),
      image_properties(image_properties),
      camera_properties(camera_properties) {
  }

  dwStatus CameraPort::Start(dwContextHandle_t context_handle) {

    for (size_t i = 0; i < GetSiblingCount(); ++i) {
      Camera camera{};
      dwStatus result = dwImage_create(camera.ImageHandle, image_properties, context_handle);
      if (result != DW_SUCCESS) {
        std::cerr << "Cannot dwImage_create:"
                  << dwGetStatusName(result) << std::endl;
        break;
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
      const uint32_t max_jpeg_bytes = 3 * 1290 * 1208;
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

  void CameraPort::ReadFrames(const dwContextHandle_t &context_handle) {
    for (int i = 0; i < Cameras.size(); i++) {
      Camera &camera = Cameras[i];
      camera.ReadingResult = DW_FAILURE;
      dwCameraFrameHandle_t camera_frame_handle;
      camera.ReadingResult = dwSensorCamera_readFrame(&camera_frame_handle, i, 300000, sensor_handle);
      if (camera.ReadingResult != DW_SUCCESS) {
        std::cerr << "dwSensorCamera_readFrame: " << dwGetStatusName(camera.ReadingResult) << std::endl;
        continue;
      }
      std::cout << "dwSensorCamera_readFrame: " << i << std::endl;

      dwImageHandle_t image_handle_yuv;
      camera.ReadingResult = dwSensorCamera_getImage(&image_handle_yuv, DW_CAMERA_OUTPUT_NATIVE_PROCESSED, camera_frame_handle);
      if (camera.ReadingResult != DW_SUCCESS) {
        std::cerr << "dwSensorCamera_getImage: " << dwGetStatusName(camera.ReadingResult) << std::endl;
      }

      std::cout << "image_handle_yuv: " << image_handle_yuv << std::endl;

      camera.ReadingResult = dwImage_copyConvert(*camera.ImageHandle, image_handle_yuv, context_handle);
      if (camera.ReadingResult != DW_SUCCESS) {
        std::cerr << "dwImage_copyConvert: " << dwGetStatusName(camera.ReadingResult) << std::endl;
      }

      std::cout << "camera.ImageHandle: " << camera.ImageHandle << std::endl;

      dwImageNvMedia *image_nvmedia;
      camera.ReadingResult = dwImage_getNvMedia(&image_nvmedia, image_handle_yuv);
      if (camera.ReadingResult != DW_SUCCESS) {
        std::cerr << "dwImage_getNvMedia: " << dwGetStatusName(camera.ReadingResult) << std::endl;
      }

      NvMediaStatus nvStatus = NvMediaIJPEFeedFrame(camera.NvMediaIjpe, image_nvmedia->img, 70);
      if (nvStatus != NVMEDIA_STATUS_OK) {
        std::cerr << "NvMediaIJPEFeedFrameQuality failed: " << nvStatus << std::endl;
      }
      nvStatus = NvMediaIJPEBitsAvailable(camera.NvMediaIjpe, &camera.CountByteJpeg, NVMEDIA_ENCODE_BLOCKING_TYPE_IF_PENDING, 10000);

      nvStatus = NvMediaIJPEGetBits(camera.NvMediaIjpe, &camera.CountByteJpeg, camera.JpegImage, 0);
      if (nvStatus != NVMEDIA_STATUS_OK && nvStatus != NVMEDIA_STATUS_NONE_PENDING) {
        std::cerr << "NvMediaIJPEGetBits failed: " << nvStatus << std::endl;
      }

      camera.ReadingResult = dwSensorCamera_returnFrame(&camera_frame_handle);
      if (camera.ReadingResult != DW_SUCCESS) {
        std::cout << "dwSensorCamera_returnFrame: " << dwGetStatusName(camera.ReadingResult) << std::endl;
      }

    }

  }
}