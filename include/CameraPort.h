#ifndef SRC_CAMERAPORT_H
#define SRC_CAMERAPORT_H

#include <dw/sensors/Sensors.h>
#include <dw/image/Image.h>
#include <dw/sensors/camera/Camera.h>
#include <dw/sensors/SensorSerializer.h>
#include <drive-t186ref-linux/include/nvmedia_ijpe.h>
#include "cv_connection.hpp"
#include "PrintEventHandler.h"


namespace DriveWorks {
  class CameraPort {
  public:
    using Ptr = std::shared_ptr<CameraPort>;
    bool debug_mode{false};
    bool init_serialiser{false};
    struct Camera {
      struct ImageWithStamp {
        dwImageHandle_t image_handle;
        ros::Time time_stamp;
      };
      OpenCVConnector::Ptr OpenCvConnector;
      NvMediaIJPE *NvMediaIjpe;
      NvMediaDevice *NvmediaDevice;
      uint32_t CountByteJpeg;
      uint8_t *JpegImage;
      std::shared_future<void> future;
      std::string NamePretty;
    };
    std::vector<Camera> Cameras;

    explicit CameraPort(dwSensorHandle_t sensor_handle, bool debug_mode, std::string port, std::string ind_camera, const std::string &caminfo_folder, PrintEventHandler::Ptr printer);

    dwStatus Start(const dwContextHandle_t &context_handle);

    void ProcessCameraStreams(std::atomic_bool &is_running, const dwContextHandle_t &context_handle);

    dwSensorHandle_t GetSensorHandle() const;

    dwSensorSerializerHandle_t GetSerializer() const;

    void InitialiseSerialiser();

    void StopSerialiser();

    void CleanUp();

    virtual ~CameraPort();

  private:
    dwSensorHandle_t sensor_handle_;
    dwImageProperties image_properties_;
    dwCameraProperties camera_properties_;
    dwSensorSerializerHandle_t camera_serializer_; // serializer for video output
    std::string port;
    std::string ind_camera;
    PrintEventHandler::Ptr printer_;
    std::string name_pretty_;
  };
}


#endif //SRC_CAMERAPORT_H
