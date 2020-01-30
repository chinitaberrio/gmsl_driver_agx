#ifndef SRC_CAMERAPORT_H
#define SRC_CAMERAPORT_H

#include <dw/sensors/Sensors.h>
#include <dw/image/Image.h>
#include <dw/sensors/camera/Camera.h>
#include <queue>
#include <drive-t186ref-linux/include/nvmedia_ijpe.h>
#include <folly/ProducerConsumerQueue.h>
#include <future>
#include "cv_connection.hpp"

namespace DriveWorks {
  class CameraPort {
  public:
    bool debug_mode{false};
    struct Camera {
      std::shared_ptr<folly::ProducerConsumerQueue<dwImageHandle_t>> QueueImageHandles;
      OpenCVConnector::Ptr OpenCvConnector;
      dwImageHandle_t ImageHandle{};
      NvMediaIJPE *NvMediaIjpe;
      NvMediaDevice *NvMediaDevicee;
      uint32_t CountByteJpeg;
      uint8_t *JpegImage;
      dwStatus ReadingResult;
      std::shared_future<void> future;
    };
    std::vector<Camera> Cameras;

    explicit CameraPort(dwSensorHandle_t sensor_handle,
                        dwImageProperties image_properties,
                        dwCameraProperties camera_properties,
                        bool debug_mode,
                        int port,
                        const std::string& caminfo_folder);

    dwStatus Start(const dwContextHandle_t &context_handle);

    void ReadFramesPushImages(const dwContextHandle_t &context_handle);

    void StartConsumers(const bool &is_running);

    void ConsumeImagesPublishMessages(const bool &is_running,
      int ind_camera);

    void ReadFrames(const dwContextHandle_t &context_handle);

    int GetSiblingCount();

    dwSensorHandle_t GetSensorHandle() const;

  private:
    dwSensorHandle_t sensor_handle;
    dwImageProperties image_properties;
    dwCameraProperties camera_properties;
    int port;
  };
}


#endif //SRC_CAMERAPORT_H
