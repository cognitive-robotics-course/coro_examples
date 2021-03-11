/* 
  Example use of openCV to perform image segmentation with the grabCut algorithm
  ------------------------------------------------------------------------------

  This application reads a sequence lines from an input file grabCutInput.txt.
  Each line contains a filename of an image to be processed. 

  It is assumed that the input file is located in a data directory given by the path ../data/ 
  defined relative to the location of executable for this application.

  The user interactively specifies the rectangular region of interest containing the object to be segmented 
  by clicking on the top-left corner and dragging the cursor.
  
  The user can also interactively specify the number of iterations of the grabCut algorithm to apply.
  
  For a more sophisticated example, see http://docs.opencv.org/trunk/de/dd0/grabcut_8cpp-example.html

  (This is the application file: it contains the client code that calls dedicated functions to implement the application.
  The code for these functions is defined in the implementation file. The functions are declared in the interface file.)

  David Vernon
  24 November 2017
    
  Audit Trail
  --------------------
  Removed ../data/ prefix from grabCutInput.txt entries
  Abrham Gebreselasie
  3 March 2021
  
  Ported to Ubuntu 16.04 and OpenCV 3.3
  Abrham Gebreselasie
  10 March 2021
  

*/

#include "module5/grabCut.h"

// Global variables to allow access by the display window callback functions

Mat inputImage;
int numberOfIterations        = 1; // default number of iterations
int number_of_control_points  = 0;

const char* input_window_name       = "Input Image";
const char* grabcut_window_name     = "GrabCut Image";


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
    
   const char input_filename[MAX_FILENAME_LENGTH] = "grabCutInput.txt";    
   char input_path_and_filename[MAX_FILENAME_LENGTH];    
   char data_dir[MAX_FILENAME_LENGTH];
   char file_path_and_filename[MAX_FILENAME_LENGTH];
     
         
   int end_of_file;
   bool debug = true;
   char filename[MAX_FILENAME_LENGTH];
   int const max_iterations  = 5; 
   FILE *fp_in; 

   printf("Example use of openCV to perform image segmentation using the grabCut algorithm\n\n");

   
   #ifdef ROS   
      strcpy(data_dir, ros::package::getPath(ROS_PACKAGE_NAME).c_str()); // get the package directory
   #else
      strcpy(data_dir, "..");
   #endif
   
   strcat(data_dir, "/data/");
   strcpy(input_path_and_filename, data_dir);
   strcat(input_path_and_filename, input_filename);
   

   if ((fp_in = fopen(input_path_and_filename,"r")) == 0) {
	  printf("Error can't open input file grabCutInput.txt\n");
     prompt_and_exit(1);
   }

   do {

      end_of_file = fscanf(fp_in, "%s", filename);
      
      if (end_of_file != EOF) {
         strcpy(file_path_and_filename, data_dir);
         strcat(file_path_and_filename, filename);
         strcpy(filename, file_path_and_filename);

         inputImage = imread(filename, CV_LOAD_IMAGE_UNCHANGED);
         if(inputImage.empty()) {
            cout << "can not open " << filename << endl;
            prompt_and_exit(-1);
         }
          
         printf("Press any key to continue ...\n");

         // Create a window for input and display it
         namedWindow(input_window_name, CV_WINDOW_AUTOSIZE );
         setMouseCallback(input_window_name, getControlPoints);    // use this callback to get the coordinates of the control points

         // Create a window
         namedWindow(grabcut_window_name, CV_WINDOW_AUTOSIZE );
         resizeWindow(grabcut_window_name,0,0); // this forces the trackbar to be as small as possible (and to fit in the window)
         numberOfIterations       = 1;          // reset each time
         createTrackbar( "Iterations", grabcut_window_name, &numberOfIterations, max_iterations, performGrabCut);

         Mat blankImage(inputImage.size(),CV_8UC3,cv::Scalar(255,255,255));
         imshow(input_window_name,inputImage);  
         imshow(grabcut_window_name,blankImage);  

         // process the image
         number_of_control_points = 0; // don't segment until the region in interest is specified
         performGrabCut(0, 0);

         do{
            waitKey(30);                
         } while (!_kbhit());                            

         getchar(); // flush the buffer from the keyboard hit

         destroyWindow(input_window_name);  
         destroyWindow(grabcut_window_name); 
      }
   } while (end_of_file != EOF);

   fclose(fp_in);

   #ifdef ROS
      // Reset terminal
      tcsetattr(STDIN, TCSANOW, &old_term);
   #endif
   return 0;
}
