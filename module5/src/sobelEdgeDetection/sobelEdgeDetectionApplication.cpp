/* 
  Example use of openCV to perform Sobel edge detection
  -----------------------------------------------------
 
  This application reads a sequence lines from an input file sobelEdgeDetectionInput.txt.
  Each line contains a filename of an image to be processed. 

  It is assumed that the input file is located in a data directory given by the path ../data/ 
  defined relative to the location of executable for this application.

  Using a trackbar on the image display, the user can interactively select the gradient threshold value to use when 
  detecting the edges.
  
  (This is the application file: it contains the client code that calls dedicated functions to implement the application.
  The code for these functions is defined in the implementation file. The functions are declared in the interface file.)

  David Vernon
  24 November 2017
    
  Audit Trail
  --------------------
  Removed ../data/ prefix from sobelEdgeDetectionInput.txt entries
  Abrham Gebreselasie
  3 March 2021
  
  Ported to Ubuntu 16.04 and OpenCV 3.3
  Abrham Gebreselasie
  10 March 2021
  
  Ported to OpenCV 4
  David Vernon
  11 July 2024
*/

#include "module5/sobelEdgeDetection.h"


// Global variables to allow access by the display window callback functions

Mat inputImage;
int thresholdValue            = 128; // default threshold
const char* magnitude_window_name   = "Sobel Gradient Magnitude";
const char* direction_window_name   = "Sobel Gradient Direction";
const char* edge_window_name        = "Sobel Edges";

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
    
   const char input_filename[MAX_FILENAME_LENGTH] = "sobelEdgeDetectionInput.txt";    
   char input_path_and_filename[MAX_FILENAME_LENGTH];    
   char data_dir[MAX_FILENAME_LENGTH];
   char file_path_and_filename[MAX_FILENAME_LENGTH];
     
      
   const char* input_window_name       = "Input Image";

   int end_of_file;
   bool debug = true;
   char filename[MAX_FILENAME_LENGTH];

   int const max_threshold     = 255; 

   FILE *fp_in;
   //FILE *fp_out;


   printf("Example use of openCV to perform Sobel edge detection.\n\n");

   
   #ifdef ROS   
      strcpy(data_dir, ros::package::getPath(ROS_PACKAGE_NAME).c_str()); // get the package directory
   #else
      strcpy(data_dir, "..");
   #endif
   
   strcat(data_dir, "/data/");
   strcpy(input_path_and_filename, data_dir);
   strcat(input_path_and_filename, input_filename);
   

   if ((fp_in = fopen(input_path_and_filename,"r")) == 0) {
	  printf("Error can't open input file sobelEdgeDetectionInput.txt\n");
     prompt_and_exit(1);
   }

   do {

      end_of_file = fscanf(fp_in, "%s", filename);
      
      if (end_of_file != EOF) {
         strcpy(file_path_and_filename, data_dir);
         strcat(file_path_and_filename, filename);
         strcpy(filename, file_path_and_filename);

         inputImage = imread(filename, IMREAD_GRAYSCALE);  // edge detection on greyscale images
         if(inputImage.empty()) {
            cout << "can not open " << filename << endl;
            prompt_and_exit(-1);
         }
          
         printf("Press any key to continue ...\n");

         // Create a window for input and display it
         namedWindow(input_window_name, WINDOW_AUTOSIZE );
         imshow(input_window_name, inputImage);
  
         // Create a window for gradient magnitude
         namedWindow(magnitude_window_name, WINDOW_AUTOSIZE );

         // Create a window for gradient direction
         namedWindow(direction_window_name, WINDOW_AUTOSIZE );

         // Create a window for edges
         namedWindow(edge_window_name, WINDOW_AUTOSIZE );
         createTrackbar( "Threshold", edge_window_name, &thresholdValue, max_threshold, sobelEdgeDetection);

         // Show the image
         sobelEdgeDetection(0, 0);

         do{
            waitKey(30);                                  // Must call this to allow openCV to display the images
         } while (!_kbhit());                             // We call it repeatedly to allow the user to move the windows
                                                          // (if we don't the window process hangs when you try to click and drag

         getchar(); // flush the buffer from the keyboard hit

         destroyWindow(input_window_name); 
         destroyWindow(magnitude_window_name); 
         destroyWindow(direction_window_name); 
         destroyWindow(edge_window_name); 
      }
   } while (end_of_file != EOF);

   fclose(fp_in);
    
   #ifdef ROS
      // Reset terminal
      tcsetattr(STDIN, TCSANOW, &old_term);
   #endif
   return 0;
}
