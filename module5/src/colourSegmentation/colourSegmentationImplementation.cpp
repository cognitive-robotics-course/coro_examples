/* 
  Example use of openCV to perform colour segmentation
  ----------------------------------------------------  
    
  (This is the implementation file: it contains the code for dedicated functions to implement the application.
  These functions are called by client code in the application file. The functions are declared in the interface file.) 

  David Vernon
  24 November 2017

  Audit Trail
  --------------------
  Added _kbhit
  18 February 2021
    
*/
 
#include "module5/colourSegmentation.h"

void colourSegmentation(int, void*) {
  
   extern Mat inputBGRImage;
   extern Mat inputHLSImage;
   extern int hueRange;
   extern int saturationRange;
   extern Point2f sample_point; 
   extern char* segmented_window_name;
   extern int number_of_sample_points;

   Mat segmentedImage; 
   int row, col;
 
   int hue;
   int saturation;
   int h;
   int s;

   bool debug = false;

   if (debug) { 
      printf("colourSegmentation: %d %d\n", hueRange, saturationRange); 
   }

   /* now get the sample point */
   if (number_of_sample_points == 1) {
  
      segmentedImage = inputBGRImage.clone();
    
      hue        = inputHLSImage.at<Vec3b>((int)sample_point.y,(int)sample_point.x)[0]; // note order of indices
      saturation = inputHLSImage.at<Vec3b>((int)sample_point.y,(int)sample_point.x)[2]; // note order of indices

      if (debug) { 
         printf("Sample point (%f, %f) Hue: %d  Saturation: %d\n", sample_point.y, sample_point.x, hue, saturation); // note order of indices
         printf("Hue range: %d  Saturation range: %d\n", hueRange, saturationRange);                                 // note order of indices
      }

      /* now perform segmentation */
 	   for (row=0; row < inputBGRImage.rows; row++) {
		   for (col=0; col < inputBGRImage.cols; col++) {

            h = inputHLSImage.at<Vec3b>(row,col)[0];
            s = inputHLSImage.at<Vec3b>(row,col)[2];

            /* Note: 0 <= h <= 180 ... NOT as you'd expect: 0 <= h <= 360  */
            if ((((h >= hue     - hueRange) && (h <= hue     + hueRange)) ||
                 ((h >= hue+180 - hueRange) && (h <= hue+180 + hueRange)) ||
                 ((h >= hue-180 - hueRange) && (h <= hue-180 + hueRange))) 
                 &&
                 ((s >= (saturation - saturationRange)) && (s <= (saturation + saturationRange)))) {
               segmentedImage.at<Vec3b>(row,col)[0] = inputBGRImage.at<Vec3b>(row,col)[0];
               segmentedImage.at<Vec3b>(row,col)[1] = inputBGRImage.at<Vec3b>(row,col)[1];
               segmentedImage.at<Vec3b>(row,col)[2] = inputBGRImage.at<Vec3b>(row,col)[2];
            }
            else {
               segmentedImage.at<Vec3b>(row,col)[0] = 0;
               segmentedImage.at<Vec3b>(row,col)[1] = 0;
               segmentedImage.at<Vec3b>(row,col)[2] = 0;
            }
         }
      }
      imshow(segmented_window_name, segmentedImage);
   }

   if (debug) printf("Leaving colourSegmentation() \n");

}
 

void getSamplePoint( int event, int x, int y, int, void* ) {
      
   extern char*   input_window_name;
   extern Mat     inputBGRImage; 
   extern Point2f sample_point; 
   extern int     number_of_sample_points;
   Mat            inputImageCopy;
   int crossHairSize = 10;

   if (event != EVENT_LBUTTONDOWN) {
      return;
   }
   else {
      number_of_sample_points = 1;
      sample_point.x = (float) x;
      sample_point.y = (float) y;

      inputImageCopy = inputBGRImage.clone();

      line(inputImageCopy,Point(x-crossHairSize/2,y),Point(x+crossHairSize/2,y),Scalar(0, 255, 0),1, CV_AA); // Green
      line(inputImageCopy,Point(x,y-crossHairSize/2),Point(x,y+crossHairSize/2),Scalar(0, 255, 0),1, CV_AA);

      imshow(input_window_name, inputImageCopy); // show the image with the cross-hairs
      
      colourSegmentation(0, 0); // Show the segmented image for new colour sample and current thresholds
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