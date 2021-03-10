/* 
  Example use of openCV to extract contours from a binary edge image
  ------------------------------------------------------------------
  
  (This is the implementation file: it contains the code for dedicated functions to implement the application.
  These functions are called by client code in the application file. The functions are declared in the interface file.) 

  David Vernon
  24 November 2017

  Audit Trail
  --------------------
  Added _kbhit
  18 February 2021
    
*/
 
#include "module5/contourExtraction.h"

/*
 * ContourExtraction
 * Trackbar callback - Canny hysteresis thresholds input with a ratio 1:3 and Gaussian standard deviation
 */

void ContourExtraction(int, void*) {  
   extern Mat src;
   extern Mat src_gray;
   extern Mat src_blur;
   extern Mat detected_edges;
   extern int cannyThreshold; 
   extern char* canny_window_name;
   extern char* contour_window_name;
   extern int gaussian_std_dev; 

   bool debug = true;
   int ratio = 3;
   int kernel_size = 3;
   int filter_size;
   vector <vector<Point> > contours;
	vector<Vec4i> hierarchy;
   Mat thresholdedImage; 

   filter_size = gaussian_std_dev * 4 + 1;  // multiplier must be even to ensure an odd filter size as required by OpenCV
                                            // this places an upper limit on gaussian_std_dev of 7 to ensure the filter size < 31
                                            // which is the maximum size for the Laplacian operator
   cvtColor(src, src_gray, CV_BGR2GRAY);

   GaussianBlur(src_gray, src_blur, Size(filter_size,filter_size), gaussian_std_dev);

   Canny( src_blur, detected_edges, cannyThreshold, cannyThreshold*ratio, kernel_size );

   Mat canny_edge_image_copy = detected_edges.clone();   // clone the edge image because findContours overwrites it

   /* see http://docs.opencv.org/2.4/modules/imgproc/doc/structural_analysis_and_shape_descriptors.html#findcontours */
   /* and http://docs.opencv.org/2.4/doc/tutorials/imgproc/shapedescriptors/find_contours/find_contours.html         */
	findContours(canny_edge_image_copy,contours,hierarchy,CV_RETR_TREE,CV_CHAIN_APPROX_NONE);

   Mat contours_image = Mat::zeros(src.size(), CV_8UC3);       // draw the contours on a black background
 
	for (int contour_number=0; (contour_number<(int)contours.size()); contour_number++) {
	   Scalar colour( rand()&0xFF, rand()&0xFF, rand()&0xFF );  // use a random colour for each contour
      drawContours( contours_image, contours, contour_number, colour, 1, 8, hierarchy );
	}

   if (debug) printf("Number of contours %d: \n", contours.size());

   imshow( canny_window_name, detected_edges );
   imshow( contour_window_name, contours_image );
}

void prompt_and_exit(int status) {
   printf("Press any key to continue and close terminal ... \n");
   getchar();
   
   #ifdef ROS
      endwin();
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