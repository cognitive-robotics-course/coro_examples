/* 
  Example use of openCV to perform camera calibration
  ---------------------------------------------------

  This application reads a sequence lines from an input file cameraCalibrationInput.txt.
  Each line contains a filename of an xml file that contains the specification of the camera calibration setup 

  It is assumed that the input file is located in a data directory given by the path ../data/ 
  defined relative to the location of executable for this application.

  (This is the application file: it contains the client code that calls dedicated functions to implement the application.
  The code for these functions is defined in the implementation file. The functions are declared in the interface file.)

  David Vernon
  24 November 2017
*/
 
#include "cameraCalibration.h"

int main() {
   // Initialize screen in ncurses raw mode
   initscr(); 


   int end_of_file;
   bool debug = false;
   char filename[MAX_FILENAME_LENGTH];

   FILE *fp_in;
   
   if ((fp_in = fopen("../data/cameraCalibrationInput.txt","r")) == 0) {
	  printf("Error can't open input cameraCalibrationInput.txt\n");
     prompt_and_exit(1);
   }

   printf("Example of how to use openCV to perform the camera calibration.\n\n");
   
   do {

      end_of_file = fscanf(fp_in, "%s", filename);
      
      if (end_of_file != EOF) {
         if (debug) {
            printf ("%s\n",filename);
            prompt_and_continue();
         }

         printf("\nPerforming camera calibration on %s \n",filename);

		   CameraCalibration( string(filename) );
      }
   } while (end_of_file != EOF);

   fclose(fp_in); 
   
   // end raw mode
   endwin();
   return 0;
}

