/* 
  Example use of openCV to perform face detection using Haar features and boosted classification
  ----------------------------------------------------------------------------------------------
 
  This application reads a sequence lines from an input file faceDetectionInput.txt.
  Each line contains either a filename of an image to be processed or the keyword "camera"
  If the keyword "camera" is present, the images to be processed are streamed live from the video camera.

  It is assumed that the input file is located in a data directory given by the path ../data/ 
  defined relative to the location of executable for this application.

  (This is the application file: it contains the client code that calls dedicated functions to implement the application.
  The code for these functions is defined in the implementation file. The functions are declared in the interface file.)

  David Vernon
  24 November 2017

  Audit Trail
  --------------------
  Removed ../data/ prefix from faceDetectionInput.txt entries
  David Vernon
  31 October 2022
  
  Ported to Ubuntu 16.04 and OpenCV 3.3
  David Vernon
  1 november 2022

*/
 
#include "module5/faceDetection.h"

int main() {

   const char input_filename[MAX_FILENAME_LENGTH] = "faceDetectionInput.txt";    
   char input_path_and_filename[MAX_FILENAME_LENGTH];    
   char data_dir[MAX_FILENAME_LENGTH];
   char file_path_and_filename[MAX_FILENAME_LENGTH];
       
   int end_of_file;
   bool debug = true;
   char filename[MAX_FILENAME_LENGTH];

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
     printf("Error can't open input faceDetectionInput.txt\n");
     prompt_and_exit(1);
   }
   
   printf("Example of how to use openCV to perform face detection using Haar features and boosed classification.\n\n");
   
   /* ---------------------------------------------------------------------------------------------
    * This code is provided as part of "A Practical Introduction to Computer Vision with OpenCV"
    * by Kenneth Dawson-Howe © Wiley & Sons Inc. 2014.  All rights reserved.
    */

   	
   // Load Haar Cascade(s)

   char file_location[MAX_FILENAME_LENGTH];  // DV Porting to ROS
   strcpy(file_location, data_dir);
   strcat(file_location, "/Media/");

	vector<CascadeClassifier> cascades;
	char* cascade_files[] = { 
		"haarcascades/haarcascade_frontalface_alt.xml" };
	int number_of_cascades = sizeof(cascade_files)/sizeof(cascade_files[0]);
	for (int cascade_file_no=0; (cascade_file_no < number_of_cascades); cascade_file_no++)
	{
		CascadeClassifier cascade;
		string filename(file_location);
		filename.append(cascade_files[cascade_file_no]);
		if( !cascade.load( filename ) )
		{
			cout << "Cannot load cascade file: " << filename << endl;
			return -1;
		}
		else cascades.push_back(cascade);
	}

   /* --------------------------------------------------------------------------------------------- */

   do {

      end_of_file = fscanf(fp_in, "%s", filename);
      
      if (end_of_file != EOF) {          
         printf("Performing face detection using Haar features and boosed classification on %s \n",filename);

         faceDetection(filename, cascades[HAAR_FACE_CASCADE_INDEX]);
      }
   } while (end_of_file != EOF);

   fclose(fp_in);

   return 0;
}

