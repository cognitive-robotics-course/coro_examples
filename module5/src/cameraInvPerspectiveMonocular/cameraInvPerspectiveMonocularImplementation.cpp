/* 
  Example use of openCV to compute the inverse perspective transformation from a camera model
  -------------------------------------------------------------------------------------------
    
  (This is the implementation file: it contains the code for dedicated functions to implement the application.
  These functions are called by client code in the application file. The functions are declared in the interface file.) 

  David Vernon
  14 June 2018
*/
 
#include "module5/cameraInvPerspectiveMonocular.h"
 
void getSamplePoint( int event, int x, int y, int, void* ) {
      
   extern char*   window_name;
   extern Point2f image_sample_point;  
   extern int     number_of_sample_points;
   extern Mat     image;

   Mat            imageCopy;
   int            crossHairSize = 10;

   if (event != EVENT_LBUTTONDOWN) {
      return;
   }
   else {
      number_of_sample_points = 1;
      image_sample_point.x = (float) x;
      image_sample_point.y = (float) y;

      imageCopy = image.clone();

      line(imageCopy,Point(x-crossHairSize/2,y),Point(x+crossHairSize/2,y),Scalar(0, 255, 0),1, CV_AA); // Green
      line(imageCopy,Point(x,y-crossHairSize/2),Point(x,y+crossHairSize/2),Scalar(0, 255, 0),1, CV_AA);

      imshow(window_name, imageCopy); // show the image with the cross-hairs
      
   }
}


void inversePerspectiveTransformation(Point2f image_sample_point,
                                      float camera_model[][4], 
                                      float z,
                                      Point3f *world_sample_point) {

   bool debug = true;
   int i, j;

   float a1, b1, c1, d1;
   float a2, b2, c2, d2;
   float x, y;


   if (false && debug) {
      printf("Camera model\n");
      for (i=0; i<3; i++) {
         for (j=0; j<4; j++) {
            printf("%f ", camera_model[i][j]);
         }
         printf("\n");
      }
      printf("\n");
      printf("Image point:  %f, %f \n\n", image_sample_point.x, image_sample_point.y);
   }

   a1 = camera_model[0][0] - image_sample_point.x * camera_model[2][0];
   b1 = camera_model[0][1] - image_sample_point.x * camera_model[2][1];
   c1 = camera_model[0][2] - image_sample_point.x * camera_model[2][2];
   d1 = camera_model[0][3] - image_sample_point.x * camera_model[2][3];
  
   a2 = camera_model[1][0] - image_sample_point.y * camera_model[2][0];
   b2 = camera_model[1][1] - image_sample_point.y * camera_model[2][1];
   c2 = camera_model[1][2] - image_sample_point.y * camera_model[2][2];
   d2 = camera_model[1][3] - image_sample_point.y * camera_model[2][3];
  

   /* inverse perspective solution for a given z value  */


   x = (z * (b1*c2 - b2*c1) + (b1*d2 - b2*d1)) / (a1*b2 - a2*b1);
   y = (z * (a2*c1 - a1*c2) + (a2*d1 - a1*d2)) / (a1*b2 - a2*b1); 

   world_sample_point->x = x;
   world_sample_point->y = y;
   world_sample_point->z = z;

   if (debug) {

      printf("(%3d, %3d) -> (%4.1f, %4.1f, %4.1f)\n", (int) image_sample_point.x,  (int) image_sample_point.y,  
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