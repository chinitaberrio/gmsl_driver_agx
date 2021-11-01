#include "CameraPort.h"
#include <dw/core/Status.h>
#include <iostream>
#include <StopWatch.h>
#include <thread>
#include <utility>

namespace DriveWorks {
CameraPort::CameraPort(
    dwSensorHandle_t sensor_handle,
    dwSensorSerializerHandle_t camera_serializer,
    bool debug_mode,
    std::string port,
    std::string ind_camera,
    const std::string &caminfo_folder,
    PrintEventHandler::Ptr printer)
    : debug_mode(debug_mode),
      sensor_handle_(sensor_handle),
      camera_serializer_(camera_serializer),
      port(port),
      ind_camera(ind_camera),
      printer_(printer) {
  std::cout << "------------------ CameraPort::CameraPort() --------------------" << std::endl;
  name_pretty_ = "Port #" + port;

  auto report_status = [&](const std::string &subject, const dwStatus &status) {
    if (status != DW_SUCCESS) {
      std::string msg_error = subject + " status: " + dwGetStatusName(status);
      printer_->Print(name_pretty_, msg_error);
      return false;
    }
  };

  dwStatus status;
  status = dwSensorCamera_getImageProperties(&image_properties_,
                                             DW_CAMERA_OUTPUT_NATIVE_PROCESSED,
                                             sensor_handle);
  report_status("dwSensorCamera_getImageProperties", status);

  status = dwSensorCamera_getSensorProperties(&camera_properties_, sensor_handle);
  report_status("dwSensorCamera_getSensorProperties", status);

  Cameras.resize(1);
  const std::string topic = "port_" + port + "/camera_" + ind_camera;
  const std::string camera_frame_id = "port_" + port + "/camera_" + ind_camera;
  const std::string cam_info_file = "file://" + caminfo_folder + port + ind_camera + "_calibration.yml";
  Camera &camera = Cameras[0];
  camera.NamePretty = name_pretty_ + " " + "Cam #" + ind_camera;
  camera.OpenCvConnector = std::make_shared<OpenCVConnector>(topic, camera_frame_id, cam_info_file, 1024);
}

dwStatus CameraPort::Start(const dwContextHandle_t &context_handle) {

  Camera &camera = Cameras[0];
  
  const uint32_t max_jpeg_bytes = 3 * 1290 * 1208;
  camera.JpegImage = (uint8_t *) malloc(max_jpeg_bytes);

  camera.NvmediaDevice = nullptr;
  camera.NvmediaDevice = NvMediaDeviceCreate();
  if (!camera.NvmediaDevice) {
    printer_->Print(camera.NamePretty, "NvMediaDeviceCreate failed.");
    exit(EXIT_FAILURE);
  }

  NvMediaSurfFormatAttr attrs[7];
  NVM_SURF_FMT_SET_ATTR_YUV(attrs, YUV, 422, PLANAR, UINT, 8, PL);
  NvMediaSurfaceType surface_type = NvMediaSurfaceFormatGetType(attrs, 7);
  camera.NvMediaIjpe = nullptr;
  camera.NvMediaIjpe = NvMediaIJPECreate(camera.NvmediaDevice,
                                          surface_type,
                                          (uint8_t) 1,
                                          max_jpeg_bytes);
  if (!camera.NvMediaIjpe) {
    printer_->Print(camera.NamePretty, "NvMediaIJPECreate failed.");
    exit(EXIT_FAILURE);
  }

  //H264 video serializer
  dwStatus status;
  status = dwSensorSerializer_start(GetSerializer());
  if (status != DW_SUCCESS) {
    std::cout << " dwSensorSerializer_start Failed " << dwGetStatusName(status) << std::endl;
    exit(EXIT_FAILURE);
  }
  // end serializer

  return dwSensor_start(GetSensorHandle());
}

dwSensorHandle_t CameraPort::GetSensorHandle() const {
  return sensor_handle_;
}

dwSensorSerializerHandle_t CameraPort::GetSerializer() const {
  return camera_serializer_;
}

void CameraPort::ProcessCameraStreams(std::atomic_bool &is_running, const dwContextHandle_t &context_handle) {

  Camera &camera = Cameras[0];
  dwStatus status;

  dwCameraFrameHandle_t camera_frame_handle;

  dwImageHandle_t image_handle;
  dwImageHandle_t image_handle_original;

  status = dwSensorCamera_readFrameNew(&camera_frame_handle, 1000000, sensor_handle_);
  if (status != DW_SUCCESS) {
    std::cout << " dwSensorCamera_readFrameNew() Failed " << dwGetStatusName(status) << std::endl;
    return;
  }
  
  camera.OpenCvConnector->check_for_subscribers();
  if (camera.OpenCvConnector->record_camera_flag){
    status = dwSensorSerializer_serializeCameraFrameAsync(camera_frame_handle, camera_serializer_);
      if (status != DW_SUCCESS) {
        std::cout << " dwSensorSerializer_serializeCameraFrameAsync) Failed " << dwGetStatusName(status) << std::endl;
        return;
      }
  }
  
  status = dwSensorCamera_getImage(&image_handle_original, DW_CAMERA_OUTPUT_NATIVE_PROCESSED, camera_frame_handle);
  if (status != DW_SUCCESS) {
    std::cout << "dwSensorCamera_getImage() Failed" << std::endl;
    is_running = false;
  }

  status = dwImage_create(&image_handle, image_properties_, context_handle);
  if (status != DW_SUCCESS) {
    std::cout << "dwImage_create() Failed" << std::endl;
    is_running = false;
  }

  status = dwImage_copyConvert(image_handle, image_handle_original, context_handle);
  if (status != DW_SUCCESS) {
    std::cout << "dwImage_copyConvert() Failed" << std::endl;
    is_running = false;
  }

  status = dwSensorCamera_returnFrame(&camera_frame_handle);
  if (status != DW_SUCCESS) {
    std::cout << "dwSensorCamera_returnFrame() Failed" << std::endl;
    is_running = false;
  }

  dwTime_t timestamp;
  status = dwImage_getTimestamp(&timestamp, image_handle_original);
  if (status != DW_SUCCESS) {
    std::cout << "dwImage_getTimestamp() Failed" << std::endl;
    is_running = false;
  }

  Camera::ImageWithStamp image_with_stamp;
  image_with_stamp.image_handle = image_handle;
  image_with_stamp.time_stamp = ros::Time((double) timestamp * 10e-7);

  dwImageNvMedia *image_nvmedia;
  status = dwImage_getNvMedia(&image_nvmedia, image_with_stamp.image_handle);
  if (status != DW_SUCCESS) {
    std::cout << "dwImage_getNvMedia() Failed" << std::endl;
    is_running = false;
  }

  NvMediaStatus nvStatus = NvMediaIJPEFeedFrame(camera.NvMediaIjpe, image_nvmedia->img, 70);
  if (nvStatus != NVMEDIA_STATUS_OK) {
    std::cout << "NvMediaIJPEFeedFrame() failed: " << std::to_string(nvStatus) << std::endl;
    is_running = false;
  }

  do {
    nvStatus = NvMediaIJPEBitsAvailable(camera.NvMediaIjpe, &camera.CountByteJpeg, NVMEDIA_ENCODE_BLOCKING_TYPE_NEVER, 0);
  } while (nvStatus != NVMEDIA_STATUS_OK);

  nvStatus = NvMediaIJPEGetBits(camera.NvMediaIjpe, &camera.CountByteJpeg, camera.JpegImage, 0);
  if (nvStatus != NVMEDIA_STATUS_OK) {
    std::cout << "NvMediaIJPEGetBits() failed: " << std::to_string(nvStatus) << std::endl;
    is_running = false;
  }

  camera.OpenCvConnector->WriteToJpeg(camera.JpegImage,
                                      camera.CountByteJpeg,
                                      image_with_stamp.time_stamp);

  status = dwImage_destroy(image_with_stamp.image_handle);
  if (status != DW_SUCCESS) {
    std::cout << "dwImage_destroy() Failed" << std::endl;
    is_running = false;
  }
}

void CameraPort::CleanUp() {
  std::cout << "------------------ CameraPort::CleanUp() --------------------" << std::endl;

  printer_->Print(name_pretty_, "clean up has started!");
  dwStatus status;
  status = dwSensor_stop(GetSensorHandle());
  if (status != DW_SUCCESS) {
    printer_->Print(name_pretty_, "dwSensor_stop: " + std::string(dwGetStatusName(status)));
  }
  printer_->Print(name_pretty_, "dwSensor_stop");
  status = dwSAL_releaseSensor(GetSensorHandle());
  if (status != DW_SUCCESS) {
    printer_->Print(name_pretty_, "dwSAL_releaseSensor: " + std::string(dwGetStatusName(status)));
  }
  printer_->Print(name_pretty_, "dwSAL_releaseSensor");
  //h264
  status = dwSensorSerializer_stop(GetSerializer());
  if (status != DW_SUCCESS) {
    printer_->Print(name_pretty_, "dwSensorSerializer_stop: " + std::string(dwGetStatusName(status)));
  }
  status = dwSensorSerializer_release(GetSerializer());
  if (status != DW_SUCCESS) {
    printer_->Print(name_pretty_, "dwSensorSerializer_release: " + std::string(dwGetStatusName(status)));
  }
  printer_->Print(name_pretty_, "dwSensorSerializer");
  //h264
  for (auto &camera : Cameras) {
    NvMediaIJPEDestroy(camera.NvMediaIjpe);
    printer_->Print(camera.NamePretty, "NvMediaIJPEDestroy");
    NvMediaDeviceDestroy(camera.NvmediaDevice);
    printer_->Print(camera.NamePretty, "NvMediaDeviceDestroy");
  }

}

CameraPort::~CameraPort() {
  std::cout << "------------------ CameraPort::~CameraPort() --------------------" << std::endl;
  printer_->Print(name_pretty_, "Destructor is called!");
  CleanUp();
}

}