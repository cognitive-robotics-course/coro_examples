/* 
  Example use of openCV to identify connected components
  ------------------------------------------------------
  
  (This is the implementation file: it contains the code for dedicated functions to implement the application.
  These functions are called by client code in the application file. The functions are declared in the interface file.) 

  David Vernon
  24 November 2017

  Audit Trail
  --------------------
  Added _kbhit
  18 February 2021
     
  Ported to OpenCV 4
  David Vernon
  11 July 2024
*/
 
#include "module5/connectedComponents.h"

/*
 * function connectedComponents
 * Trackbar callback - threshold user input
*/

void connectedComponents(int, void*) {  

   extern Mat inputImage; 
   extern int thresholdValue; 
   extern char* thresholded_window_name;
   extern char* components_window_name;
   
   Mat greyscaleImage;
   Mat thresholdedImage; 
   
   vector <vector<Point> > contours;
	vector<Vec4i> hierarchy;

   if (thresholdValue < 1)  // the trackbar has a lower value of 0 which is invalid
      thresholdValue = 1;

   if (inputImage.type() == CV_8UC3) { // colour image
      cvtColor(inputImage, greyscaleImage, COLOR_BGR2GRAY);
   } 
   else {
      greyscaleImage = inputImage.clone();
   }

   threshold(greyscaleImage,thresholdedImage,thresholdValue, 255,THRESH_BINARY);

   imshow(thresholded_window_name, thresholdedImage);

	findContours(thresholdedImage,contours,hierarchy,RETR_TREE,CHAIN_APPROX_NONE);
	Mat contours_image = Mat::zeros(inputImage.size(), CV_8UC3);
	for (int contour_number=0; (contour_number<(int)contours.size()); contour_number++)
	{
        Scalar colour( rand()&0xFF, rand()&0xFF, rand()&0xFF );
        drawContours( contours_image, contours, contour_number, colour, FILLED, 8, hierarchy );
	}

   imshow(components_window_name, contours_image);

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
