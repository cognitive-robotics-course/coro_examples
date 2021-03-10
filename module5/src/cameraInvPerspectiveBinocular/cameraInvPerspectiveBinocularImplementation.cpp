/* 
  Example use of openCV to compute the inverse perspective transformation from a pair of camera models
  ----------------------------------------------------------------------------------------------------
    
  (This is the implementation file: it contains the code for dedicated functions to implement the application.
  These functions are called by client code in the application file. The functions are declared in the interface file.) 

  David Vernon
  2 April 2018
*/
 
#include "module5/cameraInvPerspectiveBinocular.h"
 
void getLeftSamplePoint( int event, int x, int y, int, void* ) {
      
   extern char*   left_window_name;
   extern Point2f left_sample_point; 
   extern int     number_of_left_sample_points;
   extern Mat     leftImage;

   Mat            leftImageCopy;
   int            crossHairSize = 10;

   if (event != EVENT_LBUTTONDOWN) {
      return;
   }
   else {
      number_of_left_sample_points = 1;
      left_sample_point.x = (float) x;
      left_sample_point.y = (float) y;

      leftImageCopy = leftImage.clone();

      line(leftImageCopy,Point(x-crossHairSize/2,y),Point(x+crossHairSize/2,y),Scalar(0, 255, 0),1, CV_AA); // Green
      line(leftImageCopy,Point(x,y-crossHairSize/2),Point(x,y+crossHairSize/2),Scalar(0, 255, 0),1, CV_AA);

      imshow(left_window_name, leftImageCopy); // show the image with the cross-hairs
      
   }
}


void getRightSamplePoint( int event, int x, int y, int, void* ) {
      
   extern char*   right_window_name;
   extern Point2f right_sample_point;  
   extern int     number_of_right_sample_points;
   extern Mat     rightImage;

   Mat            rightImageCopy;
   int            crossHairSize = 10;

   if (event != EVENT_LBUTTONDOWN) {
      return;
   }
   else {
      number_of_right_sample_points = 1;
      right_sample_point.x = (float) x;
      right_sample_point.y = (float) y;

      rightImageCopy = rightImage.clone();

      line(rightImageCopy,Point(x-crossHairSize/2,y),Point(x+crossHairSize/2,y),Scalar(0, 255, 0),1, CV_AA); // Green
      line(rightImageCopy,Point(x,y-crossHairSize/2),Point(x,y+crossHairSize/2),Scalar(0, 255, 0),1, CV_AA);

      imshow(right_window_name, rightImageCopy); // show the image with the cross-hairs
      
   }
}


void inversePerspectiveTransformation(Point2f left_sample_point, Point2f right_sample_point, 
                                      float left_camera_model[][4], float right_camera_model[][4], 
                                      Point3f *world_sample_point) {

   bool debug = true;
   int i, j;

   /* X c = y */
   Mat X(4,3,CV_32FC1);
   Mat c(3,1,CV_32FC1);
   Mat y(4,1,CV_32FC1);
   Mat Xc(2,1,CV_32FC1); // X * c for validation ... should equal y

   float a1, b1, c1, d1;
   float a2, b2, c2, d2;
   float p1, q1, r1, s1;
   float p2, q2, r2, s2;


   printf("Left camera model\n");
   if (debug) {
      for (i=0; i<3; i++) {
         for (j=0; j<4; j++) {
            printf("%f ", left_camera_model[i][j]);
         }
         printf("\n");
      }
       
      printf("\n");
      
      printf("Right camera model\n");
      for (i=0; i<3; i++) {
         for (j=0; j<4; j++) {
            printf("%f ", right_camera_model[i][j]);
         }
         printf("\n");
      }

      printf("\n");
      
      printf("Left image point:  %f, %f \n", left_sample_point.x, left_sample_point.y);
      printf("Right image point: %f, %f \n", right_sample_point.x, right_sample_point.y);
            
      printf("\n");

   }

   a1 = left_camera_model[0][0] - left_sample_point.x * left_camera_model[2][0];
   b1 = left_camera_model[0][1] - left_sample_point.x * left_camera_model[2][1];
   c1 = left_camera_model[0][2] - left_sample_point.x * left_camera_model[2][2];
   d1 = left_camera_model[0][3] - left_sample_point.x * left_camera_model[2][3];
  
   a2 = left_camera_model[1][0] - left_sample_point.y * left_camera_model[2][0];
   b2 = left_camera_model[1][1] - left_sample_point.y * left_camera_model[2][1];
   c2 = left_camera_model[1][2] - left_sample_point.y * left_camera_model[2][2];
   d2 = left_camera_model[1][3] - left_sample_point.y * left_camera_model[2][3];
  
   p1 = right_camera_model[0][0] - right_sample_point.x * right_camera_model[2][0];
   q1 = right_camera_model[0][1] - right_sample_point.x * right_camera_model[2][1];
   r1 = right_camera_model[0][2] - right_sample_point.x * right_camera_model[2][2];
   s1 = right_camera_model[0][3] - right_sample_point.x * right_camera_model[2][3];
  
   p2 = right_camera_model[1][0] - right_sample_point.y * right_camera_model[2][0];
   q2 = right_camera_model[1][1] - right_sample_point.y * right_camera_model[2][1];
   r2 = right_camera_model[1][2] - right_sample_point.y * right_camera_model[2][2];
   s2 = right_camera_model[1][3] - right_sample_point.y * right_camera_model[2][3];
  

   /* y: vector of known constants */

   y.at<float>(0)   = -d1;
   y.at<float>(1)   = -d2;
   y.at<float>(2)   = -s1;
   y.at<float>(3)   = -s2; 

   if (debug) {
      printf("y: ");
      for (i=0; i<4; i++) {
         printf("%f ", y.at<float>(i));
      }
      printf("\n\n");
   }


   /* X: matrix */
   X.at<float>(0,0) = a1;
   X.at<float>(0,1) = b1;
   X.at<float>(0,2) = c1;
   X.at<float>(1,0) = a2;
   X.at<float>(1,1) = b2;
   X.at<float>(1,2) = c2;
   X.at<float>(2,0) = p1;
   X.at<float>(2,1) = q1;
   X.at<float>(2,2) = r1;
   X.at<float>(3,0) = p2;
   X.at<float>(3,1) = q2;
   X.at<float>(3,2) = r2;
   
   if (debug) {
      printf("X\n");
      for (i=0; i<4; i++) {
         for (j=0; j<3; j++) {
             printf("%f ", X.at<float>(i,j));
         }     
         printf("\n");
      }
      printf("\n\n");
   }

   solve(X,y,c,DECOMP_SVD);

   Xc = X * c;

   if (debug) {
      printf("Xc: ");
      for (i=0; i<4; i++) {
         printf("%f  ", Xc.at<float>(i));
      }
      
      printf("\n");

      printf("y:  ");
      for (i=0; i<4; i++) {
         printf("%f  ", y.at<float>(i));
      }
      printf("\n\n");
   }

   world_sample_point->x = c.at<float>(0);
   world_sample_point->y = c.at<float>(1);
   world_sample_point->z = c.at<float>(2);

   if (debug) {

      printf("(%3d, %3d) (%3d, %3d) -> (%4.1f, %4.1f, %4.1f)\n\n", (int) left_sample_point.x,  (int) left_sample_point.y,  
                                                                 (int) right_sample_point.x,  (int) right_sample_point.y, 
                                                                 world_sample_point->x, world_sample_point->y, world_sample_point->z);

   }
}

/*=======================================================*/
/* Utility functions                                     */ 
/*=======================================================*/

void prompt_and_exit(int status) {
   printf("Press any key to continue and close terminal ... \n");
   getchar();
   
   #ifdef ROS
      endwin();
   #endif
   exit(status);
}

void prompt_and_continue() {
   printf("Press any key to continue ... \n");
   getchar();
}

/* void pause(int milliseconds) {

   _timeb tb;

   long int s1, s2;
   long int ms1, ms2;
   long elapsed;

   _ftime(&tb); 
   s1=(long) tb.time; 
   ms1=tb.millitm;

   do {
     _ftime(&tb); 
     s2=(long) tb.time; 
     ms2=tb.millitm; 
     elapsed =(s2*1000+ms2)-(s1*1000+ms1);
   } while (elapsed < milliseconds);
} */



#ifdef ROS
/**
 Linux (POSIX) implementation of _kbhit().
 Morgan McGuire, morgan@cs.brown.edu
 */
int _kbhit() {
    static const int STDIN = 0;
    static bool initialized = false;

    if (! initialized) {
        // Use termios to turn off line buffering
        termios term;
        tcgetattr(STDIN, &term);
        term.c_lflag &= ~ICANON;
        tcsetattr(STDIN, TCSANOW, &term);
        setbuf(stdin, NULL);
        initialized = true;
    }

    int bytesWaiting;
    ioctl(STDIN, FIONREAD, &bytesWaiting);
    return bytesWaiting;
}
#endif