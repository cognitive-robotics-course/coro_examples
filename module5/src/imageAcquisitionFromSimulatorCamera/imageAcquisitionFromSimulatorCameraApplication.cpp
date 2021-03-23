/* 
  Example use of openCV to acquire and display images from file and from a simulator camera
  -------------------------------------------------------------------------------------
 
  This application acquires image from the lynxmotion_al5d_description simulator camera.
  The Lynxmotion_al5d_description simulator sends the frames it acquired from the simulation world as a
  sensor_msg::Image message on the /lynxmotion_al5d/external_vision/image_raw ROS topic. This application subscribes
  to this ROS topic and converts the ROS message into an OpenCV Mat image inside the subscriber callback function.
  The cv::Mat object is then displayed on screen.

  Abrham Gebreselasie
  23 March 2021

*/

#include "module5/imageAcquisitionFromSimulatorCamera.h"

int main(int argc, char** argv) {

   printf("Example of how to use openCV to acquire and display images from simulator camera\n");

   printf("\nPress Esc to exit\n");
   ros::init(argc, argv, "imageAcquisitionFromSimulatorCamera");

   ros::NodeHandle nh;
   image_transport::ImageTransport it(nh);

   namedWindow(OPENCV_WINDOW_NAME);
   image_transport::Subscriber imageSubscriber = it.subscribe("/lynxmotion_al5d/external_vision/image_raw", 1, &imageMessageReceived);

   ros::spin();

   return 0;
}
