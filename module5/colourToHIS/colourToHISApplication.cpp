/* 
  Example use of openCV to convert a colour image to hue, intensity, and saturation
  ---------------------------------------------------------------------------------
 
  This application reads a sequence lines from an input file colourToHISInput.txt.
  Each line contains a filename of an image to be processed. 

  It is assumed that the input file is located in a data directory given by the path ../data/ 
  defined relative to the location of executable for this application.

  (This is the application file: it contains the client code that calls dedicated functions to implement the application.
  The code for these functions is defined in the implementation file. The functions are declared in the interface file.)

  David Vernon
  24 November 2017
*/
 
#include "colourToHIS.h"

int main() {
   // Initialize screen in ncurses raw mode
   initscr(); 


   int end_of_file;
   bool debug = true;
   char filename[MAX_FILENAME_LENGTH];

   FILE *fp_in, *fp_out;
   
   if ((fp_in = fopen("../data/colourToHISInput.txt","r")) == 0) {
	  printf("Error can't open input colourToHISInput.txt\n");
     prompt_and_exit(1);
   }

   printf("Example of how to use openCV to convert from colour to HIS.\n\n");
   
   do {

      end_of_file = fscanf(fp_in, "%s", filename);
      
      if (end_of_file != EOF) {
          
         printf("\nPerforming log-polar transform on %s \n",filename);
         colourToHIS(filename);
      }
   } while (end_of_file != EOF);

   fclose(fp_in);
   
   // end raw mode
   endwin();
   return 0;
}

