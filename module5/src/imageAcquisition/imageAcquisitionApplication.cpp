/* 
  Example use of openCV to acquire and display images from file and from a video camera
  -------------------------------------------------------------------------------------
 
  This application reads a sequence lines from an input file imageAcquisitionInput.txt.
  Every triplet of lines contains 
  
  1.  a filename of an image to be displayed
  2.  a filename of a video to be displayed
  3.  a camera number (typically 0 for the internal webcam and 1 for an external USB camera)

  After the video is displayed, images from the video camera are displayed.

  When display from the camera is terminated by the user (by pressing any key), 
  the last image to be acquired is written to a file 'camera_image.png'
  and the cycle repeats until all filenames from the input file have been read.

  It is assumed that the input file is located in a data directory given by the path ../data/ 
  defined relative to the location of executable for this application.

  (This is the application file: it contains the client code that calls dedicated functions to implement the application.
  The code for these functions is defined in the implementation file. The functions are declared in the interface file.)

  David Vernon
  24 November 2017
    
  Audit Trail
  --------------------
  Removed ../data/ prefix from imageAcquisitionInput.txt entries
  Abrham Gebreselasie
  3 March 2021
  
  Ported to Ubuntu 16.04 and OpenCV 3.3
  Abrham Gebreselasie
  10 March 2021
  


  Added camera_number as an input
  DV 27 February 2019
*/

 
#include "module5/imageAcquisition.h"

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
    
   const char input_filename[MAX_FILENAME_LENGTH] = "imageAcquisitionInput.txt";    
   char input_path_and_filename[MAX_FILENAME_LENGTH];    
   char data_dir[MAX_FILENAME_LENGTH];
   char file_path_and_filename[MAX_FILENAME_LENGTH];
     
      
   int end_of_file;
   bool debug = true;
   char filename[MAX_FILENAME_LENGTH];
   int  camera_number;

   FILE *fp_in;
   
   
   #ifdef ROS   
      strcpy(data_dir, ros::package::getPath(ROS_PACKAGE_NAME).c_str()); // get the package directory
   #else
      strcpy(data_dir, "..");
   #endif
   
   strcat(data_dir, "/data/");
   strcpy(input_path_and_filename, data_dir);
   strcat(input_path_and_filename, input_filename);
   

   if ((fp_in = fopen(input_path_and_filename,"r")) == 0) {
	  printf("Error can't open input imageAcquisitionInput.txt\n");
     prompt_and_exit(1);
   }

   printf("Example of how to use openCV to acquire and display images\n");
   printf("from three sources: image file, video file, and USB camera.\n\n");
   
   do {

      end_of_file = fscanf(fp_in, "%s", filename);
      
      if (end_of_file != EOF) { 
         printf("\nDisplaying image from image file %s \n",filename);

         strcpy(file_path_and_filename, data_dir);
         strcat(file_path_and_filename, filename);
         strcpy(filename, file_path_and_filename);

         display_image_from_file(filename);

         end_of_file = fscanf(fp_in, "%s", filename);
         if (end_of_file != EOF) {
            printf("\nDisplaying image from video file %s \n",filename);
            
            strcpy(file_path_and_filename, data_dir);
            strcat(file_path_and_filename, filename);
            strcpy(filename, file_path_and_filename);
            display_image_from_video(filename);
         }

		 end_of_file = fscanf(fp_in, "%d", &camera_number);
         if (end_of_file != EOF) {
            printf("\nDisplaying image from USB camera ... \n");
            display_image_from_camera(camera_number);
         }


      }
   } while (end_of_file != EOF);

   fclose(fp_in);
  
   #ifdef ROS
      // Reset terminal
      tcsetattr(STDIN, TCSANOW, &old_term);
   #endif
   return 0;
}

