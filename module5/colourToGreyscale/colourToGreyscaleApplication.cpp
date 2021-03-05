/* 
  Example use of openCV to convert a colour image to greyscale
  ------------------------------------------------------------
 
  This application reads a sequence lines from an input file colourToGreyscaleInput.txt.
  Each line contains a filename of an image to be processed. 

  It is assumed that the input file is located in a data directory given by the path ../data/ 
  defined relative to the location of executable for this application.

  (This is the application file: it contains the client code that calls dedicated functions to implement the application.
  The code for these functions is defined in the implementation file. The functions are declared in the interface file.)

  David Vernon
  24 November 2017
*/
 
#include "colourToGreyscale.h"

int main() {
   
   string                 path;
   string                 input_filename            = "colourToGreyscaleInput.txt";
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

   FILE *fp_in;
   
   if ((fp_in = fopen(input_path_and_filename.c_str(),"r")) == 0) {
	  printf("Error can't open input colourToGreyscaleInput.txt\n");
     prompt_and_exit(1);
   }

   printf("Example of how to use openCV to converta colour image to a greyscale image.\n\n");
   
   do {

      end_of_file = fscanf(fp_in, "%s", filename);
      
      if (end_of_file != EOF) {
         datafile_path_and_filename = filename;
         datafile_path_and_filename = data_dir + datafile_path_and_filename;

         strcpy(filename, datafile_path_and_filename.c_str());
         printf("\nConverting to greyscale a colour image in %s \n",filename);
         colourToGreyscale(filename);
      }
   } while (end_of_file != EOF);


   fclose(fp_in);
   
   // end raw mode
   endwin();
   return 0;
}

