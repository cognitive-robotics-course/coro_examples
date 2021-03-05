/* 
  Example use of openCV to use the Canny edge detector
  ----------------------------------------------------
  
  This application reads a sequence lines from an input file binaryThresholdingInput.txt.
  Each line contains a filename of an image to be processed. 

  It is assumed that the input file is located in a data directory given by the path ../data/ 
  defined relative to the location of executable for this application.

  (This is the application file: it contains the client code that calls dedicated functions to implement the application.
  The code for these functions is defined in the implementation file. The functions are declared in the interface file.)

  David Vernon
  24 November 2017
*/
 
#include "cannyEdgeDetection.h"

// Global variables to allow access by the display window callback functions

Mat src;
Mat src_blur;
Mat src_gray;
Mat detected_edges;
int cannyThreshold             = 20;         // low threshold for Canny edge detector
int gaussian_std_dev           = 3;          // default standard deviation for Gaussian filter: filter size = value * 4 + 1
char* canny_window_name        = "Canny Edge Map";
char* input_window_name        = "Input Image";

int view;

int main() {
   
   string                 path;
   string                 input_filename            = "cannyEdgeDetectionInput.txt";
   string                 input_path_and_filename;
   string                 data_dir;
   string                 datafile_path_and_filename;
   data_dir = ros::package::getPath(ROS_PACKAGE_NAME); // get the package directory
   data_dir += "/data/";
   input_path_and_filename = data_dir + input_filename;
     
   // Initialize screen in ncurses raw mode
   initscr(); 

         
   int end_of_file;
   bool debug = true;
   char filename[MAX_FILENAME_LENGTH];

   int const max_cannyThreshold    = 200;  // max low threshold for Canny
   int const max_gaussian_std_dev  = 7;   

   FILE *fp_in; 

   printf("Example of how to use openCV to perform Canny edge detection.\n\n");

   if ((fp_in = fopen(input_path_and_filename.c_str(),"r")) == 0) {
	  printf("Error can't open input file cannyEdgeDetectionInput.txt\n");
     prompt_and_exit(1);
   }

   do {

      end_of_file = fscanf(fp_in, "%s", filename);
      
      if (end_of_file != EOF) {
         datafile_path_and_filename = filename;
         datafile_path_and_filename = data_dir + datafile_path_and_filename;

         src = imread(datafile_path_and_filename, CV_LOAD_IMAGE_COLOR);
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
                 
         createTrackbar( "Std Dev",    canny_window_name, &gaussian_std_dev, max_gaussian_std_dev, CannyThreshold); // same callback
         createTrackbar( "Threshold:", canny_window_name, &cannyThreshold, max_cannyThreshold,     CannyThreshold );

         // Show the image
         CannyThreshold(0, 0);

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
