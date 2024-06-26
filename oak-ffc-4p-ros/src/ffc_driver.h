#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <vector>
#include <thread>
#include <ros/ros.h>
#include <ros/rate.h>

#include "depthai/depthai.hpp"
#include <depthai/utility/Clock.hpp>
#include <chrono>

namespace OAKCAM{

class FFCDriver
{  
 public:
   struct CameraModuleConfig{
      bool show_img = false;
      bool auto_expose = false;
      bool ros_defined_freq = true;
      bool calibration_mode = true;
      bool show_img_info = false;
      bool auto_awb = false;  
      bool compressed_assemble_image = false;
      bool enable_upside_down = false;
      int32_t fps = 20.0;
      int32_t resolution = 720;
      int32_t expose_time_us = 10000;
      int32_t iso = 400;
      int32_t awb_value = 4000;
   };
   
   struct FFCColorCameraConfig{
      dai::CameraBoardSocket socket;
      dai::ColorCameraProperties::SensorResolution resolution = dai::ColorCameraProperties::SensorResolution::THE_720_P;
      std::string stream_name;
      bool is_master;
      FFCColorCameraConfig(dai::CameraBoardSocket sck,dai::ColorCameraProperties::SensorResolution res, std::string name,bool master):
         socket(sck),resolution(res),stream_name(name),is_master(master){};
   };

   struct FFCMonoCameraConfig{
      dai::CameraBoardSocket socket;
      dai::MonoCameraProperties::SensorResolution resolution = dai::MonoCameraProperties::SensorResolution::THE_720_P;
      std::string stream_name;
      bool is_master;
      FFCMonoCameraConfig(dai::CameraBoardSocket sck,dai::MonoCameraProperties::SensorResolution res, std::string name,bool master):
         socket(sck),resolution(res),stream_name(name),is_master(master){};
   };

   struct ImageNode
   {
      std::shared_ptr<dai::DataOutputQueue> data_output_q = nullptr;
      std::string topic;
      ros::Publisher ros_publisher;
      cv::Mat image;
      std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> cap_time_stamp;
      int32_t frame_counter;
      ImageNode( std::shared_ptr<dai::DataOutputQueue> data_output_q,std::string topic):data_output_q(data_output_q),topic(topic){
      }
   };
   
   std::vector<FFCColorCameraConfig> ColorCameraList = 
      {
      //  {dai::CameraBoardSocket::CAM_A,dai::MonoCameraProperties::SensorResolution::THE_720_P, std::string("CAM_A"), true}, 
       {dai::CameraBoardSocket::CAM_B,dai::ColorCameraProperties::SensorResolution::THE_720_P, std::string("CAM_B"), true},
       {dai::CameraBoardSocket::CAM_C,dai::ColorCameraProperties::SensorResolution::THE_720_P, std::string("CAM_C"), false},
      //  {dai::CameraBoardSocket::CAM_D,dai::MonoCameraProperties::SensorResolution::THE_720_P, std::string("CAM_D"),false}
      };
   std::vector<FFCMonoCameraConfig> MonoCameraList = 
      {
       {dai::CameraBoardSocket::CAM_A,dai::MonoCameraProperties::SensorResolution::THE_720_P, std::string("CAM_A"), true}, 
      //  {dai::CameraBoardSocket::CAM_B,dai::ColorCameraProperties::SensorResolution::THE_720_P, std::string("CAM_B"), false},
      //  {dai::CameraBoardSocket::CAM_C,dai::ColorCameraProperties::SensorResolution::THE_720_P, std::string("CAM_C"), false},
       {dai::CameraBoardSocket::CAM_D,dai::MonoCameraProperties::SensorResolution::THE_720_P, std::string("CAM_D"),false}
      };
   
   FFCDriver(std::shared_ptr<ros::NodeHandle>& nh);
   ~FFCDriver();
   int32_t InitPipeline();
   int32_t SetAllCameraSychron();
   int32_t SetVideoOutputQueue();
   void GetParameters(ros::NodeHandle& nh);
   void StartVideoStream();
   void GrapThreadJoin(){
      grab_thread_.join();
   };
   void StopVideoStream(){
      is_run_ = false;
      return;
   };
 private:
   void RosGrabImgThread();
   void StdGrabImgThread();
   void GrabColorImg();
   void GrabMonoImg();
   void ShowColorImg(ImageNode & image_node, std::chrono::_V2::steady_clock::time_point& time_now);
   void ShowMonoImg(ImageNode & image_node, std::chrono::_V2::steady_clock::time_point& time_now);

   ros::Publisher color_expose_time_publisher_;
   ros::Publisher mono_expose_time_publisher_;
   ros::Publisher assemble_color_image_publisher_; //to publish all camera images in
   ros::Publisher assemble_mono_image_publisher_;
   
   std::shared_ptr<dai::Pipeline> pipeline_ = nullptr;
   std::shared_ptr<dai::Device> device_ = nullptr;
   std::list<ImageNode> color_image_queue_;
   std::list<ImageNode> mono_image_queue_;
   int32_t device_is_detected_ = 0;
   int32_t pipeline_is_init_ = 0;
   CameraModuleConfig module_config_color_;
   CameraModuleConfig module_config_mono_;
   
   //config translate
   dai::ColorCameraProperties::SensorResolution color_resolution_;
   dai::MonoCameraProperties::SensorResolution mono_resolution_;

   //ros
   std::shared_ptr<ros::NodeHandle> ros_node_ = nullptr;
   ros::Timer thread_timer_;
   std::unique_ptr<ros::Rate> ros_rate_ptr_ = nullptr;

   //thread
   std::thread grab_thread_;
   bool is_run_ = true;
   //image_tmp
};

double ColorCamClearness(cv::Mat &img); //calculate clearness to manuel focus
double MonoCamClearness(cv::Mat &img);

}