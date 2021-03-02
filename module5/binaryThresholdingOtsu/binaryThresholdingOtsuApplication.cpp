/* 
  Example use of openCV to perform automatic binary thresholding using the Otsu algorithm
  ---------------------------------------------------------------------------------------
  
  This application reads a sequence lines from an input file binaryThresholdingOtsuInput.txt.
  Each line contains a filename of an image to be processed. 

  It is assumed that the input file is located in a data directory given by the path ../data/ 
  defined relative to the location of executable for this application.

  (This is the application file: it contains the client code that calls dedicated functions to implement the application.
  The code for these functions is defined in the implementation file. The functions are declared in the interface file.)

  David Vernon
  24 November 2017
*/

#include "binaryThresholdingOtsu.h"

int main() {
   // Initialize screen in ncurses raw mode
   initscr(); 

   // Initialize screen in ncurses raw mode
   initscr();
         
   int end_of_file;
   bool debug = true;
   char filename[MAX_FILENAME_LENGTH];

   FILE *fp_in;
  
   printf("Example use of openCV to perform automatic binary thresholding using the Otsu algorithm.\n\n");

   if ((fp_in = fopen("../data/binaryThresholdingOtsuInput.txt","r")) == 0) {
	  printf("Error can't open input file binaryThresholdingOtsuInput.txt\n");
     prompt_and_exit(1);
   }

   do {

      end_of_file = fscanf(fp_in, "%s", filename);
      
      if (end_of_file != EOF) {
          
         printf("\nPerforming binary thresholding using the Otsu algorithm on %s \n",filename);
         binaryThresholdingOtsu(filename);
    
      }
    } while (end_of_file != EOF);

   fclose(fp_in);

   endwin(); 
   // end raw mode
   endwin();
   return 0;
}
