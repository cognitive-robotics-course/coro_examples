/* 
  Example use of openCV to perform binary thresholding
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

#include "module5/binaryThresholding.h"


// Global variables to allow access by the display window callback functions

Mat inputImage;
int thresholdValue            = 128; // default threshold

const char* input_window_name       = "Input Image";
const char* thresholded_window_name = "Thresholded Image";


int main() {
   termios old_settings;
   termios new_settings;

   tcgetattr( 0, &old_settings );
   new_settings = old_settings;
   new_settings.c_oflag = new_settings.c_oflag | ONLRET;
   tcsetattr( 0, TCSANOW, &new_settings );
   tcsetattr( 0, TCSANOW, &old_settings );

   const char input_filename[MAX_FILENAME_LENGTH] = "binaryThresholdingInput.txt";
   char input_path_and_filename[MAX_FILENAME_LENGTH];    
   char data_dir[MAX_FILENAME_LENGTH];
   char datafile_path_and_filename[MAX_FILENAME_LENGTH];
     
         
   int end_of_file;
   bool debug = true;
   char filename[MAX_FILENAME_LENGTH];

   int const max_threshold     = 255; 

   FILE *fp_in;

   printf("Example use of openCV to perform binary thresholding.\n\n");

   
   #ifdef ROS   
      strcpy(data_dir, ros::package::getPath(ROS_PACKAGE_NAME).c_str()); // get the package directory
   #else
      strcpy(data_dir, "..");
   #endif
   
   strcat(data_dir, "/data/");
   strcpy(input_path_and_filename, data_dir);
   strcat(input_path_and_filename, input_filename);
   
   #ifdef ROS
      // Initialize screen in ncurses raw mode
      initscr();
   #endif

   if ((fp_in = fopen(input_path_and_filename,"r")) == 0) {
	  printf("Error can't open input file binaryThresholdingInput.txt\n");
     prompt_and_exit(1);
   }

   do {

      end_of_file = fscanf(fp_in, "%s", filename);
      
      if (end_of_file != EOF) {

         inputImage = imread(filename, CV_LOAD_IMAGE_UNCHANGED);
         if(inputImage.empty()) {
            cout << "can not open " << filename << endl;
            prompt_and_exit(-1);
         }
          
         printf("Press any key to continue ...\n");

         // Create a window for input and display it
         namedWindow(input_window_name, CV_WINDOW_AUTOSIZE );
         imshow(input_window_name, inputImage);
 
         // Create a window
         namedWindow(thresholded_window_name, CV_WINDOW_AUTOSIZE );
         resizeWindow(thresholded_window_name,0,0); // this forces the trackbar to be as small as possible (and to fit in the window)

         createTrackbar( "Threshold", thresholded_window_name, &thresholdValue, max_threshold, binaryThresholding);

         // Show the image
         binaryThresholding(0, 0);

         do{
            waitKey(30);                                  // Must call this to allow openCV to display the images
         } while (!_kbhit());                             // We call it repeatedly to allow the user to move the windows
                                                          // (if we don't the window process hangs when you try to click and drag

         getchar(); // flush the buffer from the keyboard hit

         destroyWindow(input_window_name);  
         destroyWindow(thresholded_window_name); 
      }
   } while (end_of_file != EOF);

   fclose(fp_in);
    
   #ifdef ROS
      // end raw mode
      endwin();
   #endif
   return 0;
}
