#include "CameraPort.h"
#include <dw/core/Status.h>
#include <iostream>
#include <StopWatch.h>
#include <thread>


namespace DriveWorks {
  CameraPort::CameraPort(dwSensorHandle_t sensor_handle,
                         bool debug_mode,
                         int port,
                         const std::string &caminfo_folder)
    : debug_mode(debug_mode),
      sensor_handle_(sensor_handle),
      port(port) {
    Cameras.resize(GetSiblingCount());
    dwStatus status = dwSensorCamera_getImageProperties(&image_properties_,
                                                        DW_CAMERA_OUTPUT_NATIVE_PROCESSED,
                                                        sensor_handle);
    if (status != DW_SUCCESS) {
      std::cerr << "dwSensorCamera_getImageProperties:"
                << dwGetStatusName(status) << std::endl;
    }
    status = dwSensorCamera_getSensorProperties(&camera_properties_, sensor_handle);
    if (status != DW_SUCCESS) {
      std::cerr << "dwSensorCamera_getSensorProperties:"
                << dwGetStatusName(status) << std::endl;
    }

    for (uint32_t ind_camera = 0; ind_camera < GetSiblingCount(); ind_camera++) {
      const std::string topic = "port_" + std::to_string(port) + "/camera_" + std::to_string(ind_camera);
      const std::string camera_frame_id = "port_" + std::to_string(port) + "/camera_" + std::to_string(ind_camera);
      const std::string cam_info_file = "file://" + caminfo_folder + std::to_string(port) + std::to_string(ind_camera) + "_calibration.yml";
      Camera &camera = Cameras[ind_camera];
      camera.Index = ind_camera;
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
        std::cerr << "NvMediaDeviceCreate failed." << std::endl;
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
        std::cerr << "NvMediaIJPECreate failed." << std::endl;
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


  std::shared_future<void> CameraPort::StartProducer(std::atomic_bool &is_running,
                                                     const dwContextHandle_t &context_handle) {
    std::cout << "Starting Producer For Port: " << port << std::endl;

    return std::async(std::launch::async,
                      &CameraPort::ReadFramesPushImages,
                      this,
                      context_handle,
                      std::ref(is_running));
  }

  void CameraPort::ReadFramesPushImages(const dwContextHandle_t &context_handle, std::atomic_bool &is_running) {
    int timeout_counter{0};
    while (is_running) {
      for (int ind_camera = 0; ind_camera < Cameras.size(); ind_camera++) {
        if (debug_mode)
          std::cout << "Producer ReadFramesPushImages For Port: " << port
                    << " Camera: " << ind_camera << std::endl;
        dwCameraFrameHandle_t camera_frame_handle;
        dwStatus status;
        status = dwSensorCamera_readFrame(&camera_frame_handle, ind_camera, 35000, sensor_handle_);
        if (status != DW_SUCCESS) {
          std::cerr << "dwSensorCamera_readFrame: " << dwGetStatusName(status) << std::endl;
          timeout_counter++;
          if (timeout_counter > 10) {
            is_running = false;
            std::cout << "Producer ReadFramesPushImages For Port: " << port
                      << " Camera: " << ind_camera << " timeout_counter = " << timeout_counter << std::endl;
          }
          continue;
        }
        timeout_counter = 0;

        ros::Time time_stamp = ros::Time::now();
        if (debug_mode)
          std::cout << "Producer dwSensorCamera_readFrame For Port: " << port
                    << " Camera: " << ind_camera << std::endl;
        dwImageHandle_t image_handle_original;
        status = dwSensorCamera_getImage(&image_handle_original, DW_CAMERA_OUTPUT_NATIVE_PROCESSED, camera_frame_handle);
        if (status != DW_SUCCESS) {
          std::cerr << "dwSensorCamera_getImage: " << dwGetStatusName(status) << std::endl;
        }
        if (debug_mode)
          std::cout << "Producer dwSensorCamera_getImage For Port: " << port
                    << " Camera: " << ind_camera << std::endl;


        dwTime_t timestamp;
        dwImage_getTimestamp(&timestamp, image_handle_original);
        double time_nvidia_sec = (double) timestamp * 10e-7;
//      std::cout << "timestamp: " << timestamp << std::endl;
//      std::cout << "time_nvidia_sec: " << time_nvidia_sec << std::endl;
//      std::cout << "ros time - nvidia time: " << (time_nvidia_sec - time_stamp.toSec()) << std::endl << std::endl;

        ros::Time time((double) timestamp * 10e-7);

        dwImageHandle_t image_handle;

        status = dwImage_create(&image_handle, image_properties_, context_handle);
        if (status != DW_SUCCESS) {
          std::cerr << "dwImage_create: " << dwGetStatusName(status) << std::endl;
        }

        status = dwImage_copyConvert(image_handle, image_handle_original, context_handle);
        if (status != DW_SUCCESS) {
          std::cerr << "dwImage_copyConvert: " << dwGetStatusName(status) << std::endl;
        }
        if (debug_mode)
          std::cout << "Producer dwImage_copyConvert For Port: " << port
                    << " Camera: " << ind_camera << std::endl;


        Camera &camera = Cameras[ind_camera];

        if (debug_mode)
          std::cerr << "queue current size: " << camera.QueueImageHandles->sizeGuess() << std::endl;

        Camera::ImageWithStamp image_with_stamp;
        image_with_stamp.image_handle = image_handle;
        image_with_stamp.time_stamp = time;
        while (!camera.QueueImageHandles->write(image_with_stamp)) {
          std::cerr << "queue is full, current size: " << camera.QueueImageHandles->sizeGuess() << std::endl;
        }
        if (debug_mode)
          std::cout << "Producer write_is_successfull For Port: " << port
                    << " Camera: " << ind_camera << std::endl;

        status = dwSensorCamera_returnFrame(&camera_frame_handle);
        if (status != DW_SUCCESS) {
          std::cout << "dwSensorCamera_returnFrame: " << dwGetStatusName(status) << std::endl;
        }
        if (debug_mode)
          std::cout << "Producer dwSensorCamera_returnFrame For Port: " << port
                    << " Camera: " << ind_camera << std::endl;
      }
    }
  }

  void CameraPort::StartConsumers(std::atomic_bool &is_running) {
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

  void CameraPort::ConsumeImagesPublishMessages(std::atomic_bool &is_running,
                                                int ind_camera) {
    Camera &camera = Cameras[ind_camera];
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
          std::cout << "Consumer For Port: " << port
                    << " Camera: " << ind_camera
                    << " no messages since past "
                    << milliseconds_passed_since(time_last_successful_capture)
                    << " ms." << std::endl;
          break;
        }
      }
      time_last_successful_capture = Clock::now();
      if (debug_mode)
        std::cout << "Consumer QueueImageHandles->read For Port: " << port
                  << " Camera: " << ind_camera << std::endl;

      dwImageNvMedia *image_nvmedia;
      dwStatus status = dwImage_getNvMedia(&image_nvmedia, image_with_stamp.image_handle);
      if (status != DW_SUCCESS) {
        std::cerr << "dwImage_getNvMedia: " << dwGetStatusName(status) << std::endl;
      }
      if (debug_mode)
        std::cout << "Consumer dwImage_getNvMedia For Port: " << port
                  << " Camera: " << ind_camera << std::endl;

      NvMediaStatus nvStatus = NvMediaIJPEFeedFrame(camera.NvMediaIjpe, image_nvmedia->img, 70);
      if (nvStatus != NVMEDIA_STATUS_OK) {
        std::cerr << "NvMediaIJPEFeedFrame failed: " << nvStatus << std::endl;
      }
      if (debug_mode)
        std::cout << "Consumer NvMediaIJPEFeedFrame For Port: " << port
                  << " Camera: " << ind_camera << std::endl;

      nvStatus = NvMediaIJPEBitsAvailable(camera.NvMediaIjpe, &camera.CountByteJpeg, NVMEDIA_ENCODE_BLOCKING_TYPE_IF_PENDING, 10000);

      if (debug_mode)
        std::cout << "Consumer NvMediaIJPEBitsAvailable For Port: " << port
                  << " Camera: " << ind_camera << std::endl;

      nvStatus = NvMediaIJPEGetBits(camera.NvMediaIjpe, &camera.CountByteJpeg, camera.JpegImage, 0);
      if (nvStatus != NVMEDIA_STATUS_OK && nvStatus != NVMEDIA_STATUS_NONE_PENDING) {
        std::cerr << "NvMediaIJPEGetBits failed: " << nvStatus << std::endl;
      }
      if (debug_mode)
        std::cout << "Consumer NvMediaIJPEGetBits For Port: " << port
                  << " Camera: " << ind_camera << std::endl;

      camera.OpenCvConnector->WriteToJpeg(
        camera.JpegImage,
        camera.CountByteJpeg,
        image_with_stamp.time_stamp);
      if (debug_mode)
        std::cout << "Consumer Published For Port: " << port
                  << " Camera: " << ind_camera << std::endl;

      status = dwImage_destroy(image_with_stamp.image_handle);
      if (status != DW_SUCCESS) {
        std::cerr << "dwImage_destroy: " << dwGetStatusName(status) << std::endl;
      }

      NvMediaImageDestroy(image_nvmedia->img);

      if (debug_mode)
        std::cout << "Consumer dwImage_destroy For Port: " << port
                  << " Camera: " << ind_camera << std::endl;

    }
  }

  void CameraPort::CleanUp() {
    std::cout << "Camera Port " << port << " clean up has started!" << std::endl;
    dwStatus status;
    status = dwSensor_stop(GetSensorHandle());
    if (status != DW_SUCCESS) {
      std::cerr << "dwSensor_stop: " << dwGetStatusName(status) << std::endl;
    }
    std::cout << "Camera Port " << port << " dwSensor_stop." << std::endl;
    status = dwSAL_releaseSensor(GetSensorHandle());
    if (status != DW_SUCCESS) {
      std::cerr << "dwSAL_releaseSensor: " << dwGetStatusName(status) << std::endl;
    }
    std::cout << "Camera Port " << port << " dwSAL_releaseSensor." << std::endl;
    for (auto &camera : Cameras) {
      NvMediaIJPEDestroy(camera.NvMediaIjpe);
      std::cout << "Camera Port " << port << "Cam " << camera.Index << " NvMediaIJPEDestroy." << std::endl;
      NvMediaDeviceDestroy(camera.NvmediaDevice);
      std::cout << "Camera Port " << port << "Cam " << camera.Index << " NvMediaDeviceDestroy." << std::endl;
    }
  }

  CameraPort::~CameraPort() {
    std::cout << "Camera Port " << port << " destructor is called!" << std::endl;
    CleanUp();
  }

}