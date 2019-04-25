#include "SekonixCamera.h"

SekonixCamera::SekonixCamera(ros::NodeHandle &nh_in, DeviceArguments CameraArguments) : nh_(nh_in) {

  nh_in.param("image_width", image_width_, int(640));
  nh_in.param("image_height", image_height_, int(480));
  nh_in.param("image_buffer", pub_buffer_, int(5));
  nh_in.param("image_compressed", img_compressed_, true);
  nh_in.param("image_compressed_quality", jpeg_quality_, int(70));


  // read ros param for camera info publish
  std::string calib_folder = "";
  nh_in.param<std::string>("calib_folder", calib_folder,"");

  // from DriveWorksApi.hpp
  ImageConfig imageConfig = {
    (uint32_t)image_width_,         				//publish image width
    (uint32_t)image_height_,        				//publish image height
    (uint32_t)pub_buffer_,        				//publish buffer
    img_compressed_,        		//publish raw or compressed image
    (uint32_t)jpeg_quality_,   //image compressed quality
    calib_folder,             //camera calibration folder
  };

  // read ros param here for camera configurations
  std::string type_a_value = "";
  std::string type_b_value = "";
  std::string type_c_value = "";
  std::string type_d_value = "";
  std::string selector_mask_value = "";
  std::string cross_csi_sync_value = "";
  std::string fifo_size_value = "";
  std::string slave_value = "";
  // reading new configuration
  nh_in.param<std::string>("type_a", type_a_value,"ar0231-rccb-bae-sf3324");
  nh_in.param<std::string>("type_b", type_b_value,"ar0231-rccb-bae-sf3324");
  nh_in.param<std::string>("type_c", type_c_value,"ar0231-rccb-bae-sf3324");
  nh_in.param<std::string>("type_d", type_d_value,"ar0231-rccb-bae-sf3324");
  nh_in.param<std::string>("selector_mask", selector_mask_value,"0001");
  nh_in.param<std::string>("cross_csi_sync", cross_csi_sync_value,"0");
  nh_in.param<std::string>("fifo_size", fifo_size_value,"3");
  nh_in.param<std::string>("slave", slave_value,"0");


  // setting new configurations
  CameraArguments.set("type-a", type_a_value);
  CameraArguments.set("type-b", type_b_value);
  CameraArguments.set("type-c", type_c_value);
  CameraArguments.set("type-d", type_d_value);
  CameraArguments.set("selector_mask", selector_mask_value);
  CameraArguments.set("cross_csi_sync", cross_csi_sync_value);
  CameraArguments.set("fifo_size", fifo_size_value);
  CameraArguments.set("slave", slave_value);

  // create gmsl camera instance with the arguments
  gmsl_cam_ = new DriveWorksApi(CameraArguments, imageConfig);
  // start camera frame grabber threads
  this->Startup();

}

/*
 * Destructor
 */
SekonixCamera::~SekonixCamera()
{
  if(gmsl_cam_) delete gmsl_cam_;
}
/*
 * Start the polling threads to grab an image from the camera and publish it
 */
void SekonixCamera::Startup()
{
  // After gmsl cameras init - start image publishing thread(s)
  while(!(gmsl_cam_->isCamReady()))
  {
    sleep(1);
  }
  // Ready
  numPort_ = gmsl_cam_->getNumPort();
  std::cout << "Start camera threads .." << std::endl;
}

/*
 * Stop the polling threads to grab an image from the camera and publish it
 * Send a request to cleanup the camera connections all at once
 */

void SekonixCamera::Shutdown()
{
  // Clean up camera frames & sdk all at once
  gmsl_cam_->stopCameras();
}

/* DriveWorks ns */



