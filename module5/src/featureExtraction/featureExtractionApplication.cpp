/* 
  Example use of openCV to perform 2D feature extraction
  ------------------------------------------------------
 
  This application reads a sequence lines from an input file featureExtractionInput.txt.
  Each line contains a filename of an image to be processed. 

  It is assumed that the input file is located in a data directory given by the path ../data/ 
  defined relative to the location of executable for this application.

  The imput image is converted to greyscale, automatically thresholded, and the connected components are identified.
  The following features are then computed using find_contours():

     Bounding rectangle
     Convex hull
     Convex cavities
     Hu moments

  The features for each image are written to the output file.

  (This is the application file: it contains the client code that calls dedicated functions to implement the application.
  The code for these functions is defined in the implementation file. The functions are declared in the interface file.)

  David Vernon
  24 November 2017
*/
 
#include "featureExtraction.h"

int main() {
   
   string                 path;
   string                 input_filename            = "featureExtractionInput.txt";
   string                 output_filename            = "featureExtractionOutput.txt";
   string                 input_path_and_filename;
   string                 output_path_and_filename;
   string                 data_dir;
   string                 datafile_path_and_filename;
   data_dir = ros::package::getPath(ROS_PACKAGE_NAME); // get the package directory
   data_dir += "/data/";
   input_path_and_filename = data_dir + input_filename;
   output_path_and_filename = data_dir + output_filename;

   // Initialize screen in ncurses raw mode
   initscr(); 


   int end_of_file;
   bool debug = true;
   char filename[MAX_FILENAME_LENGTH];

   FILE *fp_in;
   FILE *fp_out;
   
   if ((fp_in = fopen(input_path_and_filename.c_str(),"r")) == 0) {
	  printf("Error can't open input featureExtractionInput.txt\n");
     prompt_and_exit(1);
   }
   
   if ((fp_out = fopen(output_path_and_filename.c_str(),"w")) == 0) {
	  printf("Error can't open output featureExtractionOutput.txt\n");
     prompt_and_exit(1);
   }

   printf("Example of how to use openCV to perform 2D feature extraction.\n\n");
   
   do {

      end_of_file = fscanf(fp_in, "%s", filename);
      
      if (end_of_file != EOF) {
         //if (debug) printf ("%s\n",filename);
         datafile_path_and_filename = filename;
         datafile_path_and_filename = data_dir + datafile_path_and_filename;

         strcpy(filename, datafile_path_and_filename.c_str());
         printf("\nPerforming 2D feature extraction on %s \n",filename);
         featureExtraction(filename, fp_out);
      }
   } while (end_of_file != EOF);


   fclose(fp_in);
   fclose(fp_out);
   
   // end raw mode
   endwin();
   return 0;
}

