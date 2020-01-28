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
    std::queue<dwImageHandle_t *> rgbaPool;
    std::vector<dwImageHandle_t> frameRGBA;
    std::queue<uint8_t *> jpegPool;
    std::vector<NvMediaIJPE *> jpegEncoders;

    explicit CameraPort(dwSensorHandle_t sensor_handle,
                        dwImageProperties image_properties,
                        dwCameraProperties camera_properties);

    int GetSiblingCount();

    dwSensorHandle_t GetSensorHandle() const;

  private:
    dwSensorHandle_t sensor_handle;
    dwImageProperties image_properties;
    dwCameraProperties camera_properties;
  };
}


#endif //SRC_CAMERAPORT_H
