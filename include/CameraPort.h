#ifndef SRC_CAMERAPORT_H
#define SRC_CAMERAPORT_H

#include <dw/sensors/Sensors.h>
#include <dw/image/Image.h>
#include <dw/sensors/camera/Camera.h>
#include <queue>
#include <drive-t186ref-linux/include/nvmedia_ijpe.h>

namespace DriveWorks {
  class CameraPort {
  public:
    struct Camera {
//      std::vector<dwImageHandle_t> ImageHandles;
//      std::vector<NvMediaIJPE> NvMediaIjpes;
      dwImageHandle_t ImageHandle{};
      NvMediaIJPE *NvMediaIjpe;
      NvMediaDevice *NvMediaDevicee;
      uint32_t CountByteJpeg;
      uint8_t *JpegImage;
      dwStatus ReadingResult;
    };
    std::vector<Camera> Cameras;

    explicit CameraPort(dwSensorHandle_t sensor_handle,
                        dwImageProperties image_properties,
                        dwCameraProperties camera_properties);

    dwStatus Start(const dwContextHandle_t & context_handle);

    void ReadFrames(const dwContextHandle_t &context_handle);

    int GetSiblingCount();

    dwSensorHandle_t GetSensorHandle() const;

  private:
    dwSensorHandle_t sensor_handle;
    dwImageProperties image_properties;
    dwCameraProperties camera_properties;
  };
}


#endif //SRC_CAMERAPORT_H
