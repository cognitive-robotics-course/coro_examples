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

  Added camera_number as an input
  DV 27 February 2019
*/

 
#include "imageAcquisition.h"

int main() {
   // Initialize screen in ncurses raw mode
   initscr(); 

      
   int end_of_file;
   bool debug = true;
   char filename[MAX_FILENAME_LENGTH];
   int  camera_number;

   FILE *fp_in;
   
   if ((fp_in = fopen("../data/imageAcquisitionInput.txt","r")) == 0) {
	  printf("Error can't open input imageAcquisitionInput.txt\n");
     prompt_and_exit(1);
   }

   printf("Example of how to use openCV to acquire and display images\n");
   printf("from three sources: image file, video file, and USB camera.\n\n");
   
   do {

      end_of_file = fscanf(fp_in, "%s", filename);
      
      if (end_of_file != EOF) { 
         printf("\nDisplaying image from image file %s \n",filename);
         display_image_from_file(filename);

         end_of_file = fscanf(fp_in, "%s", filename);
         if (end_of_file != EOF) {
            printf("\nDisplaying image from video file %s \n",filename);
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
  
   // end raw mode
   endwin();
   return 0;
}

