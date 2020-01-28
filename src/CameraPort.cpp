#include "CameraPort.h"


namespace DriveWorks {
  CameraPort::CameraPort(dwSensorHandle_t sensor_handle,
                         dwImageProperties image_properties,
                         dwCameraProperties camera_properties)
    : sensor_handle(sensor_handle),
      image_properties(image_properties),
      camera_properties(camera_properties) {
  }

  int CameraPort::GetSiblingCount() {
    return camera_properties.siblings;
  }

  dwSensorHandle_t CameraPort::GetSensorHandle() const {
    return sensor_handle;
  }
}