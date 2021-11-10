#include "CameraPort.h"
#include <dw/core/Status.h>
#include <iostream>
#include <StopWatch.h>
#include <thread>
#include <utility>

namespace DriveWorks {
CameraPort::CameraPort(
    dwSensorHandle_t sensor_handle,
    bool debug_mode,
    std::string port,
    std::string ind_camera,
    std::string log_folder_base_name,
    const std::string &caminfo_folder,
    PrintEventHandler::Ptr printer)
    : debug_mode(debug_mode),
      sensor_handle_(sensor_handle),
      port(port),
      ind_camera(ind_camera),
      log_folder_base_name_(log_folder_base_name),
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

int CameraPort::file_existance_test (const char *filename)
{
  struct stat buffer;
  return (stat (filename, &buffer) == 0);
}

void CameraPort::InitialiseSerialiser(){
  Camera &camera = Cameras[0];
  camera_serializer_ = DW_NULL_HANDLE;
  std::string name ="port-"+ port + "_cam-"+ ind_camera; // 

  std::string latest_stream_file_name;
  std::string log_folder, log_file_name;
  camera.OpenCvConnector->nh_.param<std::string>("/log_folder", log_folder, "");
  camera.OpenCvConnector->nh_.param<std::string>("/log_file_name", log_file_name, "default-file-name");

  std::ostringstream location_name_stream;
  location_name_stream << log_folder_base_name_ << "/" << log_folder;

  std::string location_name;
  location_name = location_name_stream.str();

  DIR* dir = opendir(location_name.c_str());
  if (!dir)
  {
    std::cout << "COULD NOT OPEN folder " << location_name << std::endl;
  }
  else {
    closedir(dir);

    std::ostringstream full_file_name_candidate;
    full_file_name_candidate << location_name << "/" << log_file_name << "-" << name << ".h264.active";
    std::cout << "Test for existing file " << full_file_name_candidate.str() << std::endl;
    if (file_existance_test(full_file_name_candidate.str().c_str())) {
      // There is an existing file - construct a new filename with .partN. included in the file name
      std::cout << "H264 FILE exists " << full_file_name_candidate.str() << std::endl;
      int file_check = 1;

      for (;;) {
        std::ostringstream full_file_name_alternative;
        full_file_name_alternative << location_name << "/" << log_file_name << "-" << name << ".part" << file_check << ".h264.active";
        if (file_existance_test(full_file_name_alternative.str().c_str())) {
          std::cout << "H264 FILE exists " << full_file_name_alternative.str() << std::endl;
        }
        else {
          latest_stream_file_name = full_file_name_alternative.str();
          break;
        }
        file_check++;
      }
    }
    else {
      latest_stream_file_name = full_file_name_candidate.str();
    }

    dwSerializerParams serializerParams;
    serializerParams.parameters = "";
    std::string newParams       = "";
    newParams += std::string("format=h264");
    newParams += std::string(",serialize_bitrate=8000000");
    newParams += std::string(",framerate=30");
    newParams += std::string(",type=disk,file=") + latest_stream_file_name;
    
    serializerParams.parameters = newParams.c_str();
    serializerParams.onData     = nullptr;

    dwStatus result;
    result = dwSensorSerializer_initialize(&camera_serializer_, &serializerParams, sensor_handle_);
    if (result != DW_SUCCESS) {
      std::cout << " dwSensorSerializer_initialize Failed " << dwGetStatusName(result) << std::endl;
      exit(EXIT_FAILURE);
    }

    result = dwSensorSerializer_start(GetSerializer());
    if (result != DW_SUCCESS) {
      std::cout << " dwSensorSerializer_start Failed " << dwGetStatusName(result) << std::endl;
      exit(EXIT_FAILURE);
    }
  }
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

  //start of serialiser
  if(!init_serialiser && camera.OpenCvConnector->record_camera_flag){
    InitialiseSerialiser();
    init_serialiser = true;
    std::cout << "Opened H264 stream for port " << port <<" camera " <<  ind_camera << std::endl;
  } 
  else if(init_serialiser && !camera.OpenCvConnector->record_camera_flag) {
    StopSerialiser();
    init_serialiser = false;
    std::cout << "Closed H264 stream for port " << port <<" camera " <<  ind_camera << std::endl;
  }

  if (camera.OpenCvConnector->record_camera_flag){
    status = dwSensorSerializer_serializeCameraFrameAsync(camera_frame_handle, camera_serializer_);
      if (status != DW_SUCCESS) {
        std::cout << " dwSensorSerializer_serializeCameraFrameAsync) Failed " << dwGetStatusName(status) << std::endl;
        return;
      }
  }
  // end of serialiser

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

  if (camera.OpenCvConnector->record_camera_flag){
    camera.OpenCvConnector->PubFrameInfo(image_with_stamp.time_stamp,
                                        (uint32_t) timestamp);
  }

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

  for (auto &camera : Cameras) {
    NvMediaIJPEDestroy(camera.NvMediaIjpe);
    printer_->Print(camera.NamePretty, "NvMediaIJPEDestroy");
    NvMediaDeviceDestroy(camera.NvmediaDevice);
    printer_->Print(camera.NamePretty, "NvMediaDeviceDestroy");
  }
  
}

void CameraPort::StopSerialiser(){
  dwStatus status;
  status = dwSensorSerializer_stop(GetSerializer());
  if (status != DW_SUCCESS) {
    printer_->Print(name_pretty_, "dwSensorSerializer_stop: " + std::string(dwGetStatusName(status)));
  }
  status = dwSensorSerializer_release(GetSerializer());
  if (status != DW_SUCCESS) {
    printer_->Print(name_pretty_, "dwSensorSerializer_release: " + std::string(dwGetStatusName(status)));
  }
}

CameraPort::~CameraPort() {
  std::cout << "------------------ CameraPort::~CameraPort() --------------------" << std::endl;
  printer_->Print(name_pretty_, "Destructor is called!");
  CleanUp();
}

}