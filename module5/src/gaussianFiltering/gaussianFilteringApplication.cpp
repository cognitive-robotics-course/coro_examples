/* 
  Example use of openCV to remove noise using Gaussian filtering 
  --------------------------------------------------------------
 
  This application reads a sequence lines from an input file gaussianFilteringInput.txt.
  Each line contains a filename of an image to be processed. 

  It is assumed that the input file is located in a data directory given by the path ../data/ 
  defined relative to the location of executable for this application.

  The user can interactively select the amount of noise added and the standard deviation of the Gaussian filter.
  For the standard deviation, the value used is four times the value specified using the interactive slider, plus 1.

  (This is the application file: it contains the client code that calls dedicated functions to implement the application.
  The code for these functions is defined in the implementation file. The functions are declared in the interface file.)

  David Vernon
  24 November 2017
    
  Audit Trail
  --------------------
  Removed ../data/ prefix from gaussianFilteringInput.txt entries
  Abrham Gebreselasie
  3 March 2021
  
  Ported to Ubuntu 16.04 and OpenCV 3.3
  Abrham Gebreselasie
  10 March 2021
      
  Ported to OpenCV 4
  David Vernon
  11 July 2024

*/

#include "module5/gaussianFiltering.h"


// Global variables to allow access by the display window callback functions

Mat src;
int noise_std_dev             = 0; // default standard deviation for additive Gaussian noise
int gaussian_std_dev          = 0; // default standard deviation for Gaussian filter: filter size = value * 4 + 1

const char* input_window_name     = "Input Image";
const char* processed_window_name = "Gaussian Image";

int view;

int main() {
  
  /* removing this stops a core dump on exit. DV 28/10/2021
   #ifdef ROS
      // Turn off canonical terminal mode and character echoing
      static const int STDIN = 0;
      termios term, old_term;
      tcgetattr(STDIN, &old_term);
      tcgetattr(STDIN, &term);
      term.c_lflag &= ~(ICANON | ECHO);
      tcsetattr(STDIN, TCSANOW, &term);
   #endif 
  */
  
   const char input_filename[MAX_FILENAME_LENGTH] = "gaussianFilteringInput.txt";    
   char input_path_and_filename[MAX_FILENAME_LENGTH];    
   char data_dir[MAX_FILENAME_LENGTH];
   char file_path_and_filename[MAX_FILENAME_LENGTH];
     
         
   int end_of_file;
   bool debug = true;
   char filename[MAX_FILENAME_LENGTH];

   int const max_noise_std_dev     = 50;
   int const max_gaussian_std_dev  = 5;   

   FILE *fp_in; 

   printf("Example use of openCV to remove noise using Gaussian filtering.\n\n");

   
   #ifdef ROS   
      strcpy(data_dir, ros::package::getPath(ROS_PACKAGE_NAME).c_str()); // get the package directory
   #else
      strcpy(data_dir, "..");
   #endif
   
   strcat(data_dir, "/data/");
   strcpy(input_path_and_filename, data_dir);
   strcat(input_path_and_filename, input_filename);
   

   if ((fp_in = fopen(input_path_and_filename,"r")) == 0) {
	  printf("Error can't open input file gaussianFilteringInput.txt\n");
     prompt_and_exit(1);
   }

   do {

      end_of_file = fscanf(fp_in, "%s", filename);
      
      if (end_of_file != EOF) {
         strcpy(file_path_and_filename, data_dir);
         strcat(file_path_and_filename, filename);
         strcpy(filename, file_path_and_filename);

         src = imread(filename, IMREAD_UNCHANGED);
         if(src.empty()) {
            cout << "can not open " << filename << endl;
            prompt_and_exit(-1);
         }
          
         printf("Press any key to continue ...\n");

         // Create a window for input and display it
         namedWindow(input_window_name, WINDOW_AUTOSIZE );
         imshow(input_window_name, src);

         // Create a window
         namedWindow(processed_window_name, WINDOW_AUTOSIZE );

         createTrackbar( "Noise", processed_window_name, &noise_std_dev,    max_noise_std_dev,    processNoiseAndAveraging); // same callback
         createTrackbar( "Std Dev",processed_window_name, &gaussian_std_dev, max_gaussian_std_dev, processNoiseAndAveraging); // same callback

         // Show the image
         processNoiseAndAveraging(0, 0);

         do{
            waitKey(30);                                  // Must call this to allow openCV to display the images
         } while (!_kbhit());                             // We call it repeatedly to allow the user to move the windows
                                                          // (if we don't the window process hangs when you try to click and drag

         getchar(); // flush the buffer from the keyboard hit

         destroyWindow(input_window_name);  
         destroyWindow(processed_window_name); 

      }
   } while (end_of_file != EOF);

   fclose(fp_in);

   /*  removing this stops a core dump on exit. DV 28/10/2021
   #ifdef ROS
      // Reset terminal
      tcsetattr(STDIN, TCSANOW, &old_term);
   #endif
   */
   return 0;
}
