#include "SekonixCamera.h"
#include <thread>
#include <chrono>

SekonixCamera::SekonixCamera(ros::NodeHandle &nh_in,
                             PrintEventHandler::Ptr print_event_handler)
    : nh_(nh_in),
      print_event_handler_(std::move(print_event_handler)),
      name_pretty_("SekonixCamera") {
  int image_width_;
  int image_height_;
  int pub_buffer_;
  bool img_compressed_;
  int jpeg_quality_;
  std::string calib_folder;
  std::string type_a_value;
  std::string type_c_value;
  std::string type_e_value;
  std::string type_g_value;
  std::string selector_mask_value;
  std::string cross_csi_sync_value;
  std::string fifo_value;
  std::string slave_value;
  std::string log_folder_base_name_value;

  nh_in.param<int>("image_width", image_width_, 1920);
  nh_in.param<int>("image_height", image_height_, 1208);
  nh_in.param<int>("image_buffer", pub_buffer_, 10);
  nh_in.param<bool>("image_compressed", img_compressed_, true);
  nh_in.param<int>("image_compressed_quality", jpeg_quality_, 70);
  nh_in.param<std::string>("calib_folder", calib_folder, "/home/nvidia/projects/adas_ws/src/sekonix_camera/calib/");
  nh_in.param<std::string>("type_a", type_a_value, "SF3324");
  nh_in.param<std::string>("type_c", type_c_value, "SF3324");
  nh_in.param<std::string>("type_e", type_e_value, "SF3325");
  nh_in.param<std::string>("type_g", type_g_value, "SF3324");
  nh_in.param<std::string>("selector_mask", selector_mask_value, "1110111011001100");
  nh_in.param<std::string>("cross_csi_sync", cross_csi_sync_value, "1");
  nh_in.param<std::string>("fifo", fifo_value, "3");
  nh_in.param<std::string>("slave", slave_value, "0");
  nh_in.param<std::string>("log_folder_base_name", log_folder_base_name_value, "/media/nvidia/Samsung_T5");

  DriveWorks::ImageConfigPub imageConfig = {
      (uint32_t) image_width_,
      (uint32_t) image_height_,
      (uint32_t) pub_buffer_,
      img_compressed_,
      (uint32_t) jpeg_quality_,
      calib_folder,
  };

  DriveWorks::DeviceArguments::VecPairStrStr options = {
    std::make_pair("type-a", "SF3324"),
    std::make_pair("type-c", "SF3324"),
    std::make_pair("type-e", "SF3325"),
    std::make_pair("type-g", "SF3324"),
    std::make_pair("selector_mask", "1110111011001100"),
    std::make_pair("cross_csi_sync", "1"),
    std::make_pair("fifo", "3"),
    std::make_pair("slave", "0"),
    std::make_pair("log_folder_base_name", "/media/nvidia/Samsung_T5"),

  };

  DriveWorks::DeviceArguments camera_arguments(options);
  camera_arguments.set("type-a", type_a_value);
  camera_arguments.set("type-c", type_c_value);
  camera_arguments.set("type-e", type_e_value);
  camera_arguments.set("type-g", type_g_value);
  camera_arguments.set("selector_mask", selector_mask_value);
  camera_arguments.set("cross_csi_sync", cross_csi_sync_value);
  camera_arguments.set("fifo", fifo_value);
  camera_arguments.set("slave", slave_value);
  camera_arguments.set("log_folder_base_name", log_folder_base_name_value);

  driveworks_api_ = std::make_unique<DriveWorks::DriveWorksApi>(camera_arguments,
                                                                imageConfig,
                                                                print_event_handler_);
}

void SekonixCamera::Shutdown() {
  print_event_handler_->Print(name_pretty_, "Shutdown is called!");
  driveworks_api_.reset();
  print_event_handler_->Print(name_pretty_, "Shutdown is finished!");
}

