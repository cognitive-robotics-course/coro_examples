/* 
  Example use of openCV to perform camera calibration
  ---------------------------------------------------
 
  (This is the interface file: it contains the declarations of dedicated functions to implement the application.
  These function are called by client code in the application file. The functions are defined in the implementation file.)

  David Vernon
  10 June 2018
*/
 


#define GCC_COMPILER (defined(__GNUC__) && !defined(__clang__))

// Define ROS if on a GNU Compiler
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
   #include <stropts.h>
   #include <sys/ioctl.h>
#endif
    

//opencv
#include <cv.h>
#include <highgui.h>
#include <opencv2/opencv.hpp>

#ifdef ROS
  
   #include <ros/ros.h>
   #include <ros/package.h>
#endif 
    
 

#define TRUE  1
#define FALSE 0
#define MAX_STRING_LENGTH 80
#define MAX_FILENAME_LENGTH 200
#define MAX_NUMBER_OF_CONTROL_POINTS 100 

using namespace std;
using namespace cv;

struct imagePointType {
   int u, v;
};


struct worldPointType {
   float x, y, z;
};

/* function prototypes go here */ 
int  getImageControlPoints(string configurationFilename, int numberOfViews, int *numberOfControlPoints, imagePointType imagePoints[]);
void getSimulatedImageControlPoints(int numberOfControlPoints, float focalLength, worldPointType worldPoints[], imagePointType imagePoints[]);
void getSimulatedWorldControlPoints(int numberOfCornersWidth, int numberOfCornersHeight, int squareSize, 
                                    int numberOfViews,int originX, int originY, int originZ, 
                                    int *numberOfControlPoints, worldPointType worldPoints[]) ;
void prompt_and_exit(int status);
void prompt_and_continue();

#ifdef ROS
   int _kbhit();
#endif
   