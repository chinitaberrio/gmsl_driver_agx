#include "SekonixCamera.h"
#include <thread>
#include <chrono>

SekonixCamera::SekonixCamera(ros::NodeHandle &nh_in) : nh_(nh_in) {

  int image_width_;
  int image_height_;
  int pub_buffer_;
  bool img_compressed_;
  int jpeg_quality_;
  std::string calib_folder;
  nh_in.param<int>("/sekonix_camera_node/image_width", image_width_, 640);
  nh_in.param<int>("/sekonix_camera_node/image_height", image_height_, 480);
  nh_in.param<int>("/sekonix_camera_node/image_buffer", pub_buffer_, 5);
  nh_in.param<bool>("/sekonix_camera_node/image_compressed", img_compressed_, true);
  nh_in.param<int>("/sekonix_camera_node/image_compressed_quality", jpeg_quality_, 70);
  nh_in.param<std::string>("calib_folder", calib_folder, "");

  DriveWorks::ImageConfig imageConfig = {
    (uint32_t) image_width_,
    (uint32_t) image_height_,
    (uint32_t) pub_buffer_,
    img_compressed_,
    (uint32_t) jpeg_quality_,
    calib_folder,
  };

  std::string type_a_value;
  std::string type_b_value;
  std::string type_c_value;
  std::string type_d_value;
  std::string selector_mask_value;
  std::string cross_csi_sync_value;
  std::string fifo_size_value;
  std::string slave_value;

  nh_in.param<std::string>("/sekonix_camera_node/type_a", type_a_value, "ar0231-rccb-bae-sf3324");
  nh_in.param<std::string>("/sekonix_camera_node/type_b", type_b_value, "ar0231-rccb-bae-sf3324");
  nh_in.param<std::string>("/sekonix_camera_node/type_c", type_c_value, "ar0231-rccb-bae-sf3324");
  nh_in.param<std::string>("/sekonix_camera_node/type_d", type_d_value, "ar0231-rccb-bae-sf3324");
  nh_in.param<std::string>("/sekonix_camera_node/selector_mask", selector_mask_value, "0001");
  nh_in.param<std::string>("/sekonix_camera_node/cross_csi_sync", cross_csi_sync_value, "0");
  nh_in.param<std::string>("/sekonix_camera_node/fifo_size", fifo_size_value, "3");
  nh_in.param<std::string>("/sekonix_camera_node/slave", slave_value, "0");

  DriveWorks::DeviceArguments::VecPairStrStr options =
    {
      std::make_pair("type-a", "ar0231-rccb"),
      std::make_pair("type-b", "ar0231-rccb"),
      std::make_pair("type-c", "ar0231-rccb"),
      std::make_pair("type-d", "ar0231-rccb"),
      std::make_pair("selector_mask", "11111111"),
      std::make_pair("cross_csi_sync", "0"),
      std::make_pair("fifo_size", "3"),
      std::make_pair("slave", "0"),
    };
  DriveWorks::DeviceArguments camera_arguments(options);

  camera_arguments.set("type-a", type_a_value);
  camera_arguments.set("type-b", type_b_value);
  camera_arguments.set("type-c", type_c_value);
  camera_arguments.set("type-d", type_d_value);
  camera_arguments.set("selector_mask", selector_mask_value);
  camera_arguments.set("cross_csi_sync", cross_csi_sync_value);
  camera_arguments.set("fifo_size", fifo_size_value);
  camera_arguments.set("slave", slave_value);

  gmsl_cam_ = std::make_unique<DriveWorks::DriveWorksApi>(camera_arguments, imageConfig);

  while (!(gmsl_cam_->isCamReady())) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  std::cout << "Start camera threads .." << std::endl;
}

void SekonixCamera::Shutdown() {
  gmsl_cam_->stopCameras();
}

