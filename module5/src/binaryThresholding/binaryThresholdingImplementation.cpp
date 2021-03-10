/* 
  Example use of openCV to perform binary thresholding
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
 
#include "module5/binaryThresholding.h"

/*
 * function binaryThresholding
 * Trackbar callback - threshold user input
*/

void binaryThresholding(int, void*) {  

   extern Mat inputImage; 
   extern int thresholdValue; 
   extern char* thresholded_window_name;
   Mat greyscaleImage;
   Mat thresholdedImage; 
   int row, col;

   if (thresholdValue < 1)  // the trackbar has a lower value of 0 which is invalid
      thresholdValue = 1;

   if (inputImage.type() == CV_8UC3) { // colour image
      cvtColor(inputImage, greyscaleImage, CV_BGR2GRAY);
   } 
   else {
      greyscaleImage = inputImage.clone();
   }

   thresholdedImage.create(greyscaleImage.size(), CV_8UC1);
   
	for (row=0; row < greyscaleImage.rows; row++) {
		for (col=0; col < greyscaleImage.cols; col++) {
         if(greyscaleImage.at<uchar>(row,col) < thresholdValue) {
            thresholdedImage.at<uchar>(row,col) = (uchar) 0;
         }
         else {
            thresholdedImage.at<uchar>(row,col) = (uchar) 255;
         }
      }
   }
   
   /* alternatively, use OpenCV */

   // threshold(greyscaleImage,thresholdedImage,thresholdValue, 255,THRESH_BINARY);
   // threshold(greyscaleImage,thresholdedImage,thresholdValue, 255,THRESH_BINARY  | THRESH_OTSU); // automatic threshold selection
 
   imshow(thresholded_window_name, thresholdedImage);
}

void prompt_and_exit(int status) {
   printf("Press any key to continue and close terminal ... \n");
   getchar();
   

   #ifdef ROS
      // Reset terminal to canonical mode
      static const int STDIN = 0;
      termios term;
      tcgetattr(STDIN, &term);
      term.c_lflag |= (ICANON | ECHO);
      tcsetattr(STDIN, TCSANOW, &term);
      exit(status);
   #endif

   exit(status);
} 

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