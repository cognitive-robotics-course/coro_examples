/* 
  Example use of openCV to identify connected components
  ------------------------------------------------------
 
  This application reads a sequence lines from an input file connectedComponentsInput.txt.
  Each line contains a filename of an image to be processed. 

  It is assumed that the input file is located in a data directory given by the path ../data/ 
  defined relative to the location of executable for this application.

  Each image is thresholded using a user-specified threshold before performing connected component analysis

  (This is the application file: it contains the client code that calls dedicated functions to implement the application.
  The code for these functions is defined in the implementation file. The functions are declared in the interface file.)

  David Vernon
  24 November 2017
*/

#include "connectedComponents.h"


// Global variables to allow access by the display window callback functions

Mat inputImage;
int thresholdValue            = 128; // default threshold

char* input_window_name       = "Input Image";
char* thresholded_window_name = "Thresholded Image";
char* components_window_name  = "Connected Components";


int main() {
   
   string                 path;
   string                 input_filename            = "connectedComponentsInput.txt";
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

   int const max_threshold     = 255; 

   FILE *fp_in;

   printf("Example use of openCV to identify connected components.\n\n");

   if ((fp_in = fopen(input_path_and_filename.c_str(),"r")) == 0) {
	  printf("Error can't open input file connectedComponentsInput.txt\n");
     prompt_and_exit(1);
   }

   do {

      end_of_file = fscanf(fp_in, "%s", filename);
      
      if (end_of_file != EOF) {
         datafile_path_and_filename = filename;
         datafile_path_and_filename = data_dir + datafile_path_and_filename;


         inputImage = imread(datafile_path_and_filename, CV_LOAD_IMAGE_UNCHANGED);
         if(inputImage.empty()) {
            cout << "can not open " << filename << endl;
            prompt_and_exit(-1);
         }
          
         printf("Press any key to continue ...\n");

         // Create a window for input image and show it
         namedWindow(input_window_name, CV_WINDOW_AUTOSIZE );
         imshow(input_window_name, inputImage);
  

         // Create a window for the thresholded image
         namedWindow(thresholded_window_name, CV_WINDOW_AUTOSIZE );
         resizeWindow(thresholded_window_name,0,0); // this forces the trackbar to be as small as possible (and to fit in the window)

         createTrackbar( "Threshold", thresholded_window_name, &thresholdValue, max_threshold, connectedComponents);

         // Create a window for the connected components image
         namedWindow(components_window_name, CV_WINDOW_AUTOSIZE );
 

         // Show the image
         connectedComponents(0, 0);

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
    
   // end raw mode
   endwin();
   return 0;
}
