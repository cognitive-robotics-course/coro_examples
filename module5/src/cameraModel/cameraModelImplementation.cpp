/* 
  Example use of openCV to compute the 3x4 camera model matrix
  ------------------------------------------------------------
  
  (This is the implementation file: it contains the code for dedicated functions to implement the application.
  These functions are called by client code in the application file. The functions are declared in the interface file.) 

  David Vernon
  27 March 2018
*/
 
#include "cameraModel.h"
 
 
#include <iostream>
#include <sstream>
#include <time.h>
#include <stdio.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;
using namespace std;



void computeCameraModel(int numberOfControlPoints, worldPointType worldPoints[], imagePointType imagePoints[], double cameraModel[][4]) {

   int i, j;
   bool debug = false;

   Mat c(NUMBER_OF_UNKNOWNS,1,CV_32FC1);
   Mat y(2*numberOfControlPoints,1,CV_32FC1);
   Mat X(2*numberOfControlPoints,NUMBER_OF_UNKNOWNS,CV_32FC1);
   Mat Xc(2*numberOfControlPoints,1,CV_32FC1); // X * c for validation ... should equal y

   /* y: vector of image coordinates */
   for (i=0; i<numberOfControlPoints; i++) {
      y.at<float>(2*i)   = (float) imagePoints[i].u;
      y.at<float>(2*i+1) = (float) imagePoints[i].v;
   }

   if (debug) {
      printf("y \n");
      for (i=0; i<2*numberOfControlPoints; i++) {
         printf("%3.1f ", y.at<float>(i));
      }
      printf("\n\n");
   }

   /* X: matrix */
   for (i=0; i<numberOfControlPoints; i++) {
      X.at<float>(2*i,0)   = worldPoints[i].x;
      X.at<float>(2*i,1)   = worldPoints[i].y;
      X.at<float>(2*i,2)   = worldPoints[i].z;
      X.at<float>(2*i,3)   = 1.0;
      X.at<float>(2*i,4)   = 0.0;
      X.at<float>(2*i,5)   = 0.0;
      X.at<float>(2*i,6)   = 0.0;
      X.at<float>(2*i,7)   = 0.0;
      X.at<float>(2*i,8)   = - imagePoints[i].u * worldPoints[i].x;
      X.at<float>(2*i,9)   = - imagePoints[i].u * worldPoints[i].y;
      X.at<float>(2*i,10)  = - imagePoints[i].u * worldPoints[i].z;

      X.at<float>(2*i+1,0)   = 0.0;
      X.at<float>(2*i+1,1)   = 0.0;
      X.at<float>(2*i+1,2)   = 0.0;
      X.at<float>(2*i+1,3)   = 0.0;
      X.at<float>(2*i+1,4)   = worldPoints[i].x;
      X.at<float>(2*i+1,5)   = worldPoints[i].y;
      X.at<float>(2*i+1,6)   = worldPoints[i].z;
      X.at<float>(2*i+1,7)   = 1.0;
      X.at<float>(2*i+1,8)   = - imagePoints[i].v * worldPoints[i].x;
      X.at<float>(2*i+1,9)   = - imagePoints[i].v * worldPoints[i].y;
      X.at<float>(2*i+1,10)  = - imagePoints[i].v * worldPoints[i].z;
   }

   if (debug) {
      printf("X\n");
      for (i=0; i<2*numberOfControlPoints; i++) {
         for (j=0; j<NUMBER_OF_UNKNOWNS; j++) {
             printf("%3.1f ", X.at<float>(i,j));
         }     
         printf("\n");
      }
      printf("\n");
   }

   solve(X,y,c,DECOMP_QR);  //  DECOMP_SVD

   Xc = X * c;

   if (debug) {
      printf("Xc: ");
      for (i=0; i<2*numberOfControlPoints; i++) {
         printf("%3.1f  ", Xc.at<float>(i));
      }
      
      printf("\n");

      printf("y:  ");
      for (i=0; i<2*numberOfControlPoints; i++) {
         printf("%3.1f  ", y.at<float>(i));
      }
      printf("\n\n");
   }


   cameraModel[0][0] = c.at<float>(0);
   cameraModel[0][1] = c.at<float>(1);
   cameraModel[0][2] = c.at<float>(2);
   cameraModel[0][3] = c.at<float>(3);
   cameraModel[1][0] = c.at<float>(4);
   cameraModel[1][1] = c.at<float>(5);
   cameraModel[1][2] = c.at<float>(6);
   cameraModel[1][3] = c.at<float>(7);
   cameraModel[2][0] = c.at<float>(8);
   cameraModel[2][1] = c.at<float>(9);
   cameraModel[2][2] = c.at<float>(10);
   cameraModel[2][3] = 1.0;


   /* check result */

   if (false && debug) {
      float u, v, t;
      printf("Validation\n");
      for (i=0; i<numberOfControlPoints; i++) {
          printf("Actual:  (%4.1f %4.1f %4.1f) -> (%4d %4d)\n", worldPoints[i].x,  worldPoints[i].y,  worldPoints[i].z, imagePoints[i].u, imagePoints[i].v);
          u = (float)(cameraModel[0][0]*worldPoints[i].x + cameraModel[0][1]*worldPoints[i].y + cameraModel[0][2]*worldPoints[i].z + cameraModel[0][3]*(float)1.0);
          v = (float)(cameraModel[1][0]*worldPoints[i].x + cameraModel[1][1]*worldPoints[i].y + cameraModel[1][2]*worldPoints[i].z + cameraModel[1][3]*(float)1.0);
          t = (float)(cameraModel[2][0]*worldPoints[i].x + cameraModel[2][1]*worldPoints[i].y + cameraModel[2][2]*worldPoints[i].z + cameraModel[2][3]*(float)1.0);
          printf("Computed:(%4.1f %4.1f %4.1f) -> (%5.4f %5.4f %5.4f) -> (%4.1f %4.1f)\n\n", worldPoints[i].x,  worldPoints[i].y,  worldPoints[i].z, u, v, t, u/t, v/t );
      }
      printf("\n");
   }   
}



/*=======================================================*/
/* Utility functions to prompt user to continue          */ 
/*=======================================================*/

void prompt_and_exit(int status) {
   printf("Press any key to continue and close terminal ... \n");
   getchar();
   exit(status);
}

void prompt_and_continue() {
   printf("Press any key to continue ... \n");
   getchar();
}
