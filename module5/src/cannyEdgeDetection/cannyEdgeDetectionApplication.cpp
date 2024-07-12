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
    
  Audit Trail
  --------------------
  Removed ../data/ prefix from cannyEdgeDetectionInput.txt entries
  Abrham Gebreselasie
  3 March 2021
  
  Ported to Ubuntu 16.04 and OpenCV 3.3
  Abrham Gebreselasie
  10 March 2021
  

*/
 
#include "module5/cannyEdgeDetection.h"

// Global variables to allow access by the display window callback functions

Mat src;
Mat src_blur;
Mat src_gray;
Mat detected_edges;
int cannyThreshold             = 20;         // low threshold for Canny edge detector
int gaussian_std_dev           = 3;          // default standard deviation for Gaussian filter: filter size = value * 4 + 1
const char* canny_window_name        = "Canny Edge Map";
const char* input_window_name        = "Input Image";

int view;

int main() {
   
   #ifdef ROS
      // Turn off canonical terminal mode and character echoing
      static const int STDIN = 0;
      termios term, old_term;
      tcgetattr(STDIN, &old_term);
      tcgetattr(STDIN, &term);
      term.c_lflag &= ~(ICANON | ECHO);
      tcsetattr(STDIN, TCSANOW, &term);
   #endif 
    
   const char input_filename[MAX_FILENAME_LENGTH] = "cannyEdgeDetectionInput.txt";    
   char input_path_and_filename[MAX_FILENAME_LENGTH];    
   char data_dir[MAX_FILENAME_LENGTH];
   char file_path_and_filename[MAX_FILENAME_LENGTH];
     
         
   int end_of_file;
   bool debug = true;
   char filename[MAX_FILENAME_LENGTH];

   int const max_cannyThreshold    = 200;  // max low threshold for Canny
   int const max_gaussian_std_dev  = 7;   

   FILE *fp_in; 

   printf("Example of how to use openCV to perform Canny edge detection.\n\n");

   
   #ifdef ROS   
      strcpy(data_dir, ros::package::getPath(ROS_PACKAGE_NAME).c_str()); // get the package directory
   #else
      strcpy(data_dir, "..");
   #endif
   
   strcat(data_dir, "/data/");
   strcpy(input_path_and_filename, data_dir);
   strcat(input_path_and_filename, input_filename);
   

   if ((fp_in = fopen(input_path_and_filename,"r")) == 0) {
	  printf("Error can't open input file cannyEdgeDetectionInput.txt\n");
     prompt_and_exit(1);
   }

   do {

      end_of_file = fscanf(fp_in, "%s", filename);
      
      if (end_of_file != EOF) {
         strcpy(file_path_and_filename, data_dir);
         strcat(file_path_and_filename, filename);
         strcpy(filename, file_path_and_filename);

         src = imread(filename, IMREAD_COLOR);
         if(src.empty()) {
            cout << "can not open " << filename << endl;
            return -1;
         }
          
         printf("Press any key to continue ...\n");

         // Create a window for input and display it
         namedWindow(input_window_name, WINDOW_AUTOSIZE );
         imshow(input_window_name, src);
 
         // Create a window
         namedWindow(canny_window_name, WINDOW_AUTOSIZE);
                 
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

   #ifdef ROS
      // Reset terminal
      tcsetattr(STDIN, TCSANOW, &old_term);
   #endif
   return 0;
}
