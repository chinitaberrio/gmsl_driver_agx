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
    int port,
    const std::string &caminfo_folder,
    PrintEventHandler::Ptr printer)
    : debug_mode(debug_mode),
      sensor_handle_(sensor_handle),
      port(port),
      printer_(std::move(printer)) {
    name_pretty_ = "Port #" + std::to_string(port);
    Cameras.resize(GetSiblingCount());
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

    for (uint32_t ind_camera = 0; ind_camera < GetSiblingCount(); ind_camera++) {
      const std::string topic = "port_" + std::to_string(port) + "/camera_" + std::to_string(ind_camera);
      const std::string camera_frame_id = "port_" + std::to_string(port) + "/camera_" + std::to_string(ind_camera);
      const std::string cam_info_file = "file://" + caminfo_folder + std::to_string(port) + std::to_string(ind_camera) + "_calibration.yml";
      Camera &camera = Cameras[ind_camera];
      camera.Index = ind_camera;
      camera.NamePretty = name_pretty_ + " " + "Cam #" + std::to_string(camera.Index);
      camera.OpenCvConnector = std::make_shared<OpenCVConnector>(topic, camera_frame_id, cam_info_file, 10);
      camera.QueueImageHandles = std::make_shared<folly::ProducerConsumerQueue<Camera::ImageWithStamp>>(10);
    }
  }

  dwStatus CameraPort::Start(const dwContextHandle_t &context_handle) {
    for (size_t ind_camera = 0; ind_camera < GetSiblingCount(); ++ind_camera) {
      Camera &camera = Cameras[ind_camera];
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

    }
    return dwSensor_start(GetSensorHandle());
  }

  int CameraPort::GetSiblingCount() {
    return camera_properties_.siblings;
  }

  dwSensorHandle_t CameraPort::GetSensorHandle() const {
    return sensor_handle_;
  }


  void CameraPort::StartProducer(std::atomic_bool &is_running,
                                 const dwContextHandle_t &context_handle) {
    printer_->Print(name_pretty_, "Starting Producer.");
    future_ = std::async(std::launch::async,
                         &CameraPort::ReadFramesPushImages,
                         this,
                         context_handle,
                         std::ref(is_running));
  }

  void CameraPort::ReadFramesPushImages(const dwContextHandle_t &context_handle, std::atomic_bool &is_running) {
    int timeout_counter{0};
    auto print_debug_for_cam = [&](const std::string &message, int ind_camera) {
      if (debug_mode)
        printer_->Print(name_pretty_, message + " for Cam #" + std::to_string(ind_camera));
    };
    auto print_status_error_for_cam = [&](const std::string &subject, const dwStatus &status, int ind_camera) {
      if (status != DW_SUCCESS)
        printer_->Print(name_pretty_, subject + " status: " + dwGetStatusName(status) + " for Cam #" + std::to_string(ind_camera));
    };
    auto print_error_for_cam = [&](const std::string &message, int ind_camera) {
      printer_->Print(name_pretty_, message + " for Cam #" + std::to_string(ind_camera));
    };


    while (is_running) {
      for (int ind_camera = 0; ind_camera < Cameras.size(); ind_camera++) {
        print_debug_for_cam("ReadFramesPushImages", ind_camera);
        dwCameraFrameHandle_t camera_frame_handle;
        dwStatus status;
        status = dwSensorCamera_readFrame(&camera_frame_handle, ind_camera, 35000, sensor_handle_);
        if (status != DW_SUCCESS) {
          print_status_error_for_cam("dwSensorCamera_readFrame", status, ind_camera);
          timeout_counter++;
          if (timeout_counter > 10) {
            is_running = false;
            print_error_for_cam("ReadFramesPushImages timeout_counter = " + std::to_string(timeout_counter), ind_camera);
          }
          continue;
        }
        timeout_counter = 0;

        ros::Time time_stamp = ros::Time::now();
        print_debug_for_cam("dwSensorCamera_readFrame", ind_camera);
        dwImageHandle_t image_handle_original;
        status = dwSensorCamera_getImage(&image_handle_original, DW_CAMERA_OUTPUT_NATIVE_PROCESSED, camera_frame_handle);
        print_status_error_for_cam("dwSensorCamera_getImage", status, ind_camera);
        print_debug_for_cam("dwSensorCamera_getImage", ind_camera);

        dwTime_t timestamp;
        dwImage_getTimestamp(&timestamp, image_handle_original);
        double time_nvidia_sec = (double) timestamp * 10e-7;
//      std::cout << "timestamp: " << timestamp << std::endl;
//      std::cout << "time_nvidia_sec: " << time_nvidia_sec << std::endl;
//      std::cout << "ros time - nvidia time: " << (time_nvidia_sec - time_stamp.toSec()) << std::endl << std::endl;

        ros::Time time((double) timestamp * 10e-7);

        dwImageHandle_t image_handle;

        status = dwImage_create(&image_handle, image_properties_, context_handle);
        print_status_error_for_cam("dwImage_create", status, ind_camera);

        status = dwImage_copyConvert(image_handle, image_handle_original, context_handle);
        print_status_error_for_cam("dwImage_copyConvert", status, ind_camera);
        print_debug_for_cam("dwImage_copyConvert", ind_camera);

        Camera &camera = Cameras[ind_camera];

        print_debug_for_cam("Queue Size = " + std::to_string(camera.QueueImageHandles->sizeGuess()), ind_camera);

        Camera::ImageWithStamp image_with_stamp;
        image_with_stamp.image_handle = image_handle;
        image_with_stamp.time_stamp = time;
        while (!camera.QueueImageHandles->write(image_with_stamp)) {
          print_error_for_cam("Queue Size = "
                              + std::to_string(camera.QueueImageHandles->sizeGuess())
                              + " (Full)", ind_camera);
        }
        print_debug_for_cam("write_is_successfull", ind_camera);

        status = dwSensorCamera_returnFrame(&camera_frame_handle);
        print_status_error_for_cam("dwSensorCamera_returnFrame", status, ind_camera);
        print_debug_for_cam("dwSensorCamera_returnFrame", ind_camera);
      }
    }
  }

  void CameraPort::StartConsumers(std::atomic_bool &is_running) {
    for (size_t ind_camera = 0; ind_camera < GetSiblingCount(); ++ind_camera) {
      printer_->Print(name_pretty_, "Starting Consumer For Cam #" + std::to_string(ind_camera));
      Camera &camera = Cameras[ind_camera];
      camera.future = std::async(std::launch::async,
                                 &CameraPort::ConsumeImagesPublishMessages,
                                 this,
                                 std::ref(is_running),
                                 ind_camera);
    }
  }

  void CameraPort::ConsumeImagesPublishMessages(std::atomic_bool &is_running,
                                                int ind_camera) {
    Camera &camera = Cameras[ind_camera];

    auto print = [&](const std::string &message) {
      printer_->Print(camera.NamePretty, message);
    };

    auto print_debug = [&](const std::string &message) {
      if (debug_mode)
        printer_->Print(camera.NamePretty, message);
    };

    auto print_status_error = [&](const std::string &subject, const dwStatus &status) {
      if (status != DW_SUCCESS)
        printer_->Print(camera.NamePretty, subject + " status: " + dwGetStatusName(status));
    };

    using Clock = std::chrono::steady_clock;
    using TimePoint = std::chrono::time_point<Clock>;
    TimePoint time_last_successful_capture = Clock::now();

    auto milliseconds_passed_since = [](const TimePoint &time_point) {
      return std::chrono::duration_cast<std::chrono::milliseconds>(
        Clock::now() - time_point).count();
    };

    while (is_running) {
      Camera::ImageWithStamp image_with_stamp;
      while (!camera.QueueImageHandles->read(image_with_stamp)) {
        //spin until we get a value
        std::this_thread::sleep_for(std::chrono::microseconds(500));
        if (milliseconds_passed_since(time_last_successful_capture) > 3000) {
          is_running = false;
          print("Received messages since past "
                + std::to_string(milliseconds_passed_since(time_last_successful_capture))
                + " ms.");
          break;
        }
      }
      time_last_successful_capture = Clock::now();
      print_debug("camera.QueueImageHandles->read");

      dwImageNvMedia *image_nvmedia;
      dwStatus status = dwImage_getNvMedia(&image_nvmedia, image_with_stamp.image_handle);
      print_status_error("dwImage_getNvMedia", status);
      print_debug("dwImage_getNvMedia");

      NvMediaStatus nvStatus = NvMediaIJPEFeedFrame(camera.NvMediaIjpe, image_nvmedia->img, 70);
      if (nvStatus != NVMEDIA_STATUS_OK) {
        print("NvMediaIJPEFeedFrame failed: " + std::to_string(nvStatus));
      }
      print_debug("NvMediaIJPEFeedFrame");

      nvStatus = NvMediaIJPEBitsAvailable(camera.NvMediaIjpe, &camera.CountByteJpeg, NVMEDIA_ENCODE_BLOCKING_TYPE_IF_PENDING, 10000);

      print_debug("NvMediaIJPEBitsAvailable");

      nvStatus = NvMediaIJPEGetBits(camera.NvMediaIjpe, &camera.CountByteJpeg, camera.JpegImage, 0);
      if (nvStatus != NVMEDIA_STATUS_OK && nvStatus != NVMEDIA_STATUS_NONE_PENDING) {
        print("NvMediaIJPEGetBits failed: " + std::to_string(nvStatus));
      }
      print_debug("NvMediaIJPEGetBits");

      camera.OpenCvConnector->WriteToJpeg(
        camera.JpegImage,
        camera.CountByteJpeg,
        image_with_stamp.time_stamp);
      print_debug("Published ROS Messages.");

      status = dwImage_destroy(image_with_stamp.image_handle);
      print_status_error("dwImage_destroy", status);
      NvMediaImageDestroy(image_nvmedia->img);
      print_debug("dwImage_destroy");
    }
  }

  void CameraPort::CleanUp() {
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
    for (auto &camera : Cameras) {
      camera.future.get();
      printer_->Print(camera.NamePretty, "Future is received.");
    }
    future_.get();
    printer_->Print(name_pretty_, "Future is received.");
  }

  CameraPort::~CameraPort() {
    printer_->Print(name_pretty_, "Destructor is called!");
    CleanUp();
  }

}