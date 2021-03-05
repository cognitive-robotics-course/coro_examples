/* 
  Example use of openCV to perform image segmentation with the grabCut algorithm
  ------------------------------------------------------------------------------
  
  (This is the implementation file: it contains the code for dedicated functions to implement the application.
  These functions are called by client code in the application file. The functions are declared in the interface file.) 

  David Vernon
  24 November 2017
*/
 
#include "grabCut.h"

// Global variables to allow access by the display window callback functions
// specifically the mouse callback function to acquire the control point coordinates and draw cross-hairs

Point2i control_points[4];
Rect    rect;
bool    rectState = false; // true => draw the rectangle as the mouse moves and flag the fact that the rectangle is defined

/*
 * function grabCut
 * Trackbar callback - number of iterations user input
*/

void performGrabCut(int, void*) {  

   extern Mat   inputImage; 
   extern int   numberOfIterations; 
   extern int   number_of_control_points;
   extern char* grabcut_window_name;
   Mat result;          // segmentation result 
   Mat bgModel,fgModel; // the models (hard constraints)

   if (numberOfIterations < 1)  // the trackbar has a lower value of 0 which is invalid
      numberOfIterations = 1;

   /* get two control points (top left and bottom right) and rectangle */
   do { 
      waitKey(30); 
   } while (number_of_control_points < 2);

    /* GrabCut segmentation                                                                           */
    /* see: http://docs.opencv.org/2.4/modules/imgproc/doc/miscellaneous_transformations.html#grabcut */
    grabCut(inputImage,         // input image
            result,             // segmentation result (4 values); can also be used as an input mask providing constraints
            rect,               // rectangle containing foreground 
            bgModel,fgModel,    // for internal use ... allows continuation of iterative solution on subsequent calls
            numberOfIterations, // number of iterations
            GC_INIT_WITH_RECT); // use rectangle
    
    /* Get the pixels marked as likely foreground */
    compare(result,GC_PR_FGD,result,CMP_EQ);

    /* Generate output image */
    Mat foreground(inputImage.size(),CV_8UC3,cv::Scalar(255,255,255));
    inputImage.copyTo(foreground,result); // use result to mask out the background pixels 
 
    imshow(grabcut_window_name, foreground);
}

/* Simple callback to highlight a region of interest by drawing a green rectangle                                 */
/*                                                                                                                */
/* Two parameters are returned via external variables                                                             */
/* (should rewrite so that the parameters are returned in a structure pointed to by the 5th parameter             */
/*                                                                                                                */
/*  Point2i control_points[]   contains the coordinates of the top left and bottom right corners of the rectangle */
/*  Rect rect;                 contains the rectangle data                                                        */
/*                                                                                                                */
/*  David Vernon, 18/09/2017                                                                                      */

void getControlPoints(int event, int x, int y, int, void* ) {
      
   extern char* input_window_name;
   extern Mat   inputImage; 
   extern Point2i control_points[]; 
   extern Rect rect;
   extern int number_of_control_points;
   Mat    imageCopy;
   int crossHairSize = 10;
   static bool rectState = false; // true => draw the rectangle as the mouse moves

   switch(event) {
    case EVENT_LBUTTONDOWN:  
       number_of_control_points++;

       control_points[0].x = x;
       control_points[0].y = y;

       rectState = true;
       rect = Rect( x, y, 1, 1 );

       inputImage.copyTo(imageCopy);
       rectangle(imageCopy, Point( rect.x, rect.y ), Point(rect.x + rect.width, rect.y + rect.height ), Scalar(0, 255, 0), 1); // green
       imshow(input_window_name, imageCopy); 
  
       break;

    case EVENT_LBUTTONUP:
             
       number_of_control_points++;

       control_points[1].x = x;
       control_points[1].y = y;

       if ( rectState == true ) {
          rect = Rect( Point(rect.x, rect.y), Point(x,y) );
          rectState = false;
       }
 
       inputImage.copyTo(imageCopy);
       rectangle(imageCopy, Point( rect.x, rect.y ), Point(rect.x + rect.width, rect.y + rect.height ), Scalar(0, 255, 0), 1); // green
       imshow(input_window_name, imageCopy); 

       break;

    case EVENT_MOUSEMOVE:
        if( rectState == true ) {
           rect = Rect( Point(rect.x, rect.y), Point(x,y) );
           inputImage.copyTo(imageCopy);
           rectangle(imageCopy, Point( rect.x, rect.y ), Point(rect.x + rect.width, rect.y + rect.height ), Scalar(0, 255, 0), 1); // green
           imshow(input_window_name, imageCopy); 
        }
        break;
    }
}
void prompt_and_exit(int status) {
   printf("Press any key to continue and close terminal ... \n");
   getchar();
   exit(status);
} 

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