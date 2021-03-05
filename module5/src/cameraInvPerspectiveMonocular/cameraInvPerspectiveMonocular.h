/* 
  Example use of openCV to compute the inverse perspective transformation from a camera model
  -------------------------------------------------------------------------------------------
   
  (This is the interface file: it contains the declarations of dedicated functions to implement the application.
  These function are called by client code in the application file. The functions are defined in the implementation file.)

  David Vernon
  14 June 2018
*/
 

#define ROS_PACKAGE_NAME "lectures"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include <ctype.h>
#include <iostream>
#include <string>
#include <sys/select.h>
#include <termios.h>
#include <stropts.h>
#include <sys/ioctl.h>

//opencv
#include <cv.h>
#include <highgui.h>
#include <opencv2/opencv.hpp>



// Must be included after opencv2/opencv.hpp to avoid incompatiability
#include <ncurses.h>
#include <ros/ros.h>
#include <ros/package.h>



#define TRUE  1
#define FALSE 0
#define MAX_STRING_LENGTH 80
#define MAX_FILENAME_LENGTH 200

using namespace std;
using namespace cv;

struct imagePointType {
   int u, v;
};

struct worldPointType {
   float x, y, z;
};


/* function prototypes go here */

void inversePerspectiveTransformation(Point2f image_sample_point, float camera_model[][4], float z, Point3f *world_sample_point);
void getSamplePoint( int event, int x, int y, int, void*);
void prompt_and_exit(int status);
void prompt_and_continue();
void pause(int milliseconds);
int _kbhit();