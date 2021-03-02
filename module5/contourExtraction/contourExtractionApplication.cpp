/* 
  Example use of openCV to extract contours from a binary edge image
  ------------------------------------------------------------------
 
  This application reads a sequence lines from an input file contourExtractionInput.txt.
  Each line contains a filename of an image to be processed. 

  It is assumed that the input file is located in a data directory given by the path ../data/ 
  defined relative to the location of executable for this application.

  The edge image is generated using the Canny edge detector.

  (This is the application file: it contains the client code that calls dedicated functions to implement the application.
  The code for these functions is defined in the implementation file. The functions are declared in the interface file.)

  David Vernon
  24 November 2017
*/
 
#include "contourExtraction.h"


// Global variables to allow access by the display window callback functions

Mat src;
Mat src_blur;
Mat src_gray;
Mat detected_edges;
int cannyThreshold             = 20;         // low threshold for Canny edge detector
int gaussian_std_dev           = 3;          // default standard deviation for Gaussian filter: filter size = value * 4 + 1
char* canny_window_name        = "Canny Edge Map";
char* contour_window_name      = "Contours";
char* input_window_name        = "Input Image";

int view;

int main() {
   // Initialize screen in ncurses raw mode
   initscr(); 

         
   int end_of_file;
   bool debug = true;
   char filename[MAX_FILENAME_LENGTH];

   int const max_cannyThreshold    = 200;  // max low threshold for Canny
   int const max_gaussian_std_dev  = 7;   

   FILE *fp_in;

   printf("Example of how to use openCV to extract contours from a binary edge image.\n\n");

   if ((fp_in = fopen("../data/contourExtractionInput.txt","r")) == 0) {
	  printf("Error can't open input file contourExtractionInput.txt\n");
     prompt_and_exit(1);
   }

   do {

      end_of_file = fscanf(fp_in, "%s", filename);
      
      if (end_of_file != EOF) {

         src = imread(filename, CV_LOAD_IMAGE_COLOR);
         if(src.empty()) {
            cout << "can not open " << filename << endl;
            // end raw mode
   endwin();
   return -1;
         }
          
         printf("Press any key to continue ...\n");

         // Create a window for input and display it
         namedWindow(input_window_name, CV_WINDOW_AUTOSIZE );
         imshow(input_window_name, src);
 
         // Create a window
         namedWindow(canny_window_name, CV_WINDOW_AUTOSIZE );
         resizeWindow(canny_window_name,0,0); // this forces the trackbar to be as small as possible (and to fit in the window)
                 
         createTrackbar( "Std Dev",    canny_window_name, &gaussian_std_dev, max_gaussian_std_dev, ContourExtraction); // same callback
         createTrackbar( "Threshold:", canny_window_name, &cannyThreshold, max_cannyThreshold,     ContourExtraction );

         // Show the image
         ContourExtraction(0, 0);

         do{
            waitKey(30);                                  // Must call this to allow openCV to display the images
         } while (!_kbhit());                             // We call it repeatedly to allow the user to move the windows
                                                          // (if we don't the window process hangs when you try to click and drag

         getchar(); // flush the buffer from the keyboard hit

         destroyWindow(input_window_name);  
         destroyWindow(canny_window_name); 

      }
   } while (end_of_file != EOF);

   fclose(fp_in); 

   // end raw mode
   endwin();
   return 0;
}
