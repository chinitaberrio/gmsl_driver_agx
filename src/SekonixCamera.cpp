#include "SekonixCamera.h"

SekonixCamera::SekonixCamera(ros::NodeHandle &nh_in, DeviceArguments CameraArguments) : nh_(nh_in) {

  nh_in.param("/sekonix_camera_node/image_width", image_width_, int(640));
  nh_in.param("/sekonix_camera_node/image_height", image_height_, int(480));
  nh_in.param("/sekonix_camera_node/image_buffer", pub_buffer_, int(5));
  nh_in.param("/sekonix_camera_node/image_compressed", img_compressed_, true);
  nh_in.param("/sekonix_camera_node/image_compressed_quality", jpeg_quality_, int(70));


  // read ros param for camera info publish
  std::string calib_folder = "";
  nh_in.param<std::string>("calib_folder", calib_folder, "");

  // from DriveWorksApi.hpp
  ImageConfig imageConfig = {
    (uint32_t) image_width_,
    (uint32_t) image_height_,
    (uint32_t) pub_buffer_,
    img_compressed_,
    (uint32_t) jpeg_quality_,
    calib_folder,
  };

  std::string type_a_value = "";
  std::string type_b_value = "";
  std::string type_c_value = "";
  std::string type_d_value = "";
  std::string selector_mask_value = "";
  std::string cross_csi_sync_value = "";
  std::string fifo_size_value = "";
  std::string slave_value = "";

  nh_in.param<std::string>("/sekonix_camera_node/type_a", type_a_value, "ar0231-rccb-bae-sf3324");
  nh_in.param<std::string>("/sekonix_camera_node/type_b", type_b_value, "ar0231-rccb-bae-sf3324");
  nh_in.param<std::string>("/sekonix_camera_node/type_c", type_c_value, "ar0231-rccb-bae-sf3324");
  nh_in.param<std::string>("/sekonix_camera_node/type_d", type_d_value, "ar0231-rccb-bae-sf3324");
  nh_in.param<std::string>("/sekonix_camera_node/selector_mask", selector_mask_value, "0001");
  nh_in.param<std::string>("/sekonix_camera_node/cross_csi_sync", cross_csi_sync_value, "0");
  nh_in.param<std::string>("/sekonix_camera_node/fifo_size", fifo_size_value, "3");
  nh_in.param<std::string>("/sekonix_camera_node/slave", slave_value, "0");


  CameraArguments.set("type-a", type_a_value);
  CameraArguments.set("type-b", type_b_value);
  CameraArguments.set("type-c", type_c_value);
  CameraArguments.set("type-d", type_d_value);
  CameraArguments.set("selector_mask", selector_mask_value);
  CameraArguments.set("cross_csi_sync", cross_csi_sync_value);
  CameraArguments.set("fifo_size", fifo_size_value);
  CameraArguments.set("slave", slave_value);

  gmsl_cam_ = std::make_unique<DriveWorksApi>(CameraArguments, imageConfig);

  while (!(gmsl_cam_->isCamReady())) {
    sleep(1);
  }
  numPort_ = gmsl_cam_->getNumPort();
  std::cout << "Start camera threads .." << std::endl;
}

void SekonixCamera::Shutdown() {
  gmsl_cam_->stopCameras();
}

