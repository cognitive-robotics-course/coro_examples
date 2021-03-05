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
*/

#include "sobelEdgeDetection.h"


// Global variables to allow access by the display window callback functions

Mat inputImage;
int thresholdValue            = 128; // default threshold
char* magnitude_window_name   = "Sobel Gradient Magnitude";
char* direction_window_name   = "Sobel Gradient Direction";
char* edge_window_name        = "Sobel Edges";

int main() {
   
   string                 path;
   string                 input_filename            = "sobelEdgeDetectionInput.txt";
   string                 input_path_and_filename;
   string                 data_dir;
   string                 datafile_path_and_filename;
   data_dir = ros::package::getPath(ROS_PACKAGE_NAME); // get the package directory
   data_dir += "/data/";
   input_path_and_filename = data_dir + input_filename;
     
   // Initialize screen in ncurses raw mode
   initscr(); 

      
   char* input_window_name       = "Input Image";

   int end_of_file;
   bool debug = true;
   char filename[MAX_FILENAME_LENGTH];

   int const max_threshold     = 255; 

   FILE *fp_in;
   //FILE *fp_out;


   printf("Example use of openCV to perform Sobel edge detection.\n\n");

   if ((fp_in = fopen(input_path_and_filename.c_str(),"r")) == 0) {
	  printf("Error can't open input file sobelEdgeDetectionInput.txt\n");
     prompt_and_exit(1);
   }

   do {

      end_of_file = fscanf(fp_in, "%s", filename);
      
      if (end_of_file != EOF) {
         datafile_path_and_filename = filename;
         datafile_path_and_filename = data_dir + datafile_path_and_filename;

         inputImage = imread(datafile_path_and_filename, CV_LOAD_IMAGE_GRAYSCALE);  // edge detection on greyscale images
         if(inputImage.empty()) {
            cout << "can not open " << filename << endl;
            prompt_and_exit(-1);
         }
          
         printf("Press any key to continue ...\n");

         // Create a window for input and display it
         namedWindow(input_window_name, CV_WINDOW_AUTOSIZE );
         imshow(input_window_name, inputImage);
  
         // Create a window for gradient magnitude
         namedWindow(magnitude_window_name, CV_WINDOW_AUTOSIZE );

         // Create a window for gradient direction
         namedWindow(direction_window_name, CV_WINDOW_AUTOSIZE );

         // Create a window for edges
         namedWindow(edge_window_name, CV_WINDOW_AUTOSIZE );
         resizeWindow(edge_window_name,0,0); // this forces the trackbar to be as small as possible (and to fit in the window)

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
    
   // end raw mode
   endwin();
   return 0;
}
