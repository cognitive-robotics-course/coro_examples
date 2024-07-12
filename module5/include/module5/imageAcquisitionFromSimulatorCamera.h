/* 
  Example use of openCV to acquire and display images from file and from a simulator camera
  -------------------------------------------------------------------------------------

  (This is the interface file: it contains the declarations of dedicated functions to implement the application.
  These function are called by client code in the application file. The functions are defined in the implementation file.)

  Abrham Gebreselasie
  23 March 2021
      
  Ported to OpenCV 4
  David Vernon
  11 July 2024
*/


#define GCC_COMPILER (defined(__GNUC__) && !defined(__clang__))

#if GCC_COMPILER
   #ifndef ROS
       #define ROS
   #endif
   #ifndef ROS_PACKAGE_NAME
      #define ROS_PACKAGE_NAME "module5"
   #endif
#endif

#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include <ctype.h>
#include <iostream>
#include <string>

#ifndef ROS
   #include <conio.h>
#else
   #include <sys/select.h>
   #include <termios.h>
   #include <sys/ioctl.h>
   #include <fcntl.h>
   #include <unistd.h>
#endif


//opencv
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

// ncurses.h must be included after opencv2/opencv.hpp to avoid incompatibility
#include <ncurses.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#define TRUE  1
#define FALSE 0
#define MAX_STRING_LENGTH   80
#define MAX_FILENAME_LENGTH 200
#define OPENCV_WINDOW_NAME "Simulator camera video"

using namespace std;
using namespace cv;

/* function prototypes go here */

void display_image_from_file(char *filename);
void display_image_from_video(char *filename);
void display_image_from_camera(int camera_number);
void prompt_and_exit(int status);
void prompt_and_continue();
void imageMessageReceived(const sensor_msgs::ImageConstPtr& msg);

#ifdef ROS
   int _kbhit();
#endif
