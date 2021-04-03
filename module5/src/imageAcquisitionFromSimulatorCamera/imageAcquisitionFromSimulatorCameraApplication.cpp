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
   #ifdef ROS
      // Turn off canonical terminal mode and character echoing
      static const int STDIN = 0;
      termios term, old_term;
      tcgetattr(STDIN, &old_term);
      tcgetattr(STDIN, &term);
      term.c_lflag &= ~(ICANON | ECHO);
      tcsetattr(STDIN, TCSANOW, &term);
   #endif
   char pressedKey;
   int nRead;

   printf("Example of how to use openCV to acquire and display images from simulator camera\n");

   printf("\nPress any key to exit\n");
   ros::init(argc, argv, "imageAcquisitionFromSimulatorCamera");

   ros::NodeHandle nh;
   image_transport::ImageTransport it(nh);

   namedWindow(OPENCV_WINDOW_NAME);
   image_transport::Subscriber imageSubscriber = it.subscribe("/lynxmotion_al5d/external_vision/image_raw", 1, &imageMessageReceived);

   /* Change STDIN mode to non-blocking I/O to allow taking a key press from console as well as the CV window
    * Using the likes of getchar (blocking I/O) will interfere with the drawing of the acquired images
    */
   while (true)
   {
      ros::spinOnce();
      if (_kbhit())
      {
        break;
      }
   }
   destroyAllWindows();
   #ifdef ROS
       // Reset terminal
       tcsetattr(STDIN, TCSANOW, &old_term);
   #endif

   return 0;
}
