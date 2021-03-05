/* 
  Example use of openCV to generate image control point data for computing the camera model
  ---------------------------------------------------------------------------------------------------

  This application acquires the image control points which provide input to the cameraModel application.
  The world control points are provided by other means, such as manual measurement, with or without 
  the aid of a robot to tie them to the robot's frame of reference.
 
  In general, calibration requires that the calibration grid be presented in (at least) two different poses. 
  However, in some applications, the z values of all the world points are constrained  so that they are all equal 
  and constant (e.g. such as the situation where objects sit on a flat table) and you need only calibrate for 
  a single plane rather than for the full 3D volume. In this case, it is sufficient to have a single view of 
  the calibration grid resting on the surface to be calibrated. Note, however, that a camera model derived 
  from this single view will only yield valid inverse perspective mapping for the z value that 
  was used during calibration.

  The image control points are computed from either a single view of a calibration grid or two views.
  The first line of the input file (camerModelDataInput.txt) should have either the number 1 or 2 
  to identify the number of views to be used. 
  
  This is followed by two filenames, one input and one output.

  The first filename identifies an input .xml file that contains the specification of the camera calibration setup, 
  e.g. the number of points in the x and y direction in the calibration grid, the grid spacing, and the camera number.
  
  The second filename identifies the output .txt file where the 2D image control point coordinates are written.  
  This file will be used by the cameraModel application.

  The application then captures one or two images of a calibration grid using the camera, identifies the corner points, 
  and writes their coordinates to the output file.

  It is assumed that the input file is located in a data directory given by the path ../data/ 
  defined relative to the location of executable for this application.

  When the application is launched, images of the calibration grid are acquired with the camera and the 
  detected control points are displayed.   You must press the 'g' key to save the control points.
  If two views have been specified in the input file, you must press the 'g' key again to save the control
  points for the second view.

  For the standard 9x7 chessboard pattern calibration grid (see ../data/Media/chessboard_pattern.jpg), 
  the control points are ordered as shown in the figure shown below. 
  
    1   2   3   4   5   6   7   8

    9  10  11  12  13  14  15  16

   17  18  19  20  21  22  23  24

   25  26  27  28  29  30  31  32

   33  34  35  36  37  38  39  40

   41  142  43  44  45 46  47  48

  You need to know this ordering when identifying the 3D coordinates corresponding world control points; 
  see also the robotCalibration project. 


  (This is the application file: it contains the client code that calls dedicated functions to implement the application.
  The code for these functions is defined in the implementation file. The functions are declared in the interface file.)

  David Vernon
  9 June 2018
*/
 
#include "cameraModelData.h"

int main() {
   
   string                 path;
   string                 input_filename            = "cameraModelDataInput.txt";
   string                 input_path_and_filename;
   string                 data_dir;
   string                 datafile_path_and_filename;
   data_dir = ros::package::getPath(ROS_PACKAGE_NAME); // get the package directory
   data_dir += "/data/";
   input_path_and_filename = data_dir + input_filename;
     
   // Initialize screen in ncurses raw mode
   initscr(); 


   int end_of_file;
   bool debug = false;
   char configurationPathAndFilename[MAX_FILENAME_LENGTH];
   char controlPointsPathAndFilename[MAX_FILENAME_LENGTH];
   char filename[MAX_FILENAME_LENGTH];
   int i;
   int numberOfViews;

   FILE *fp_in;
   FILE *fp_control_points;

   imagePointType imagePoints[MAX_NUMBER_OF_CONTROL_POINTS];
   int numberOfControlPoints;

   if ((fp_in = fopen(input_path_and_filename.c_str(),"r")) == 0) {
	  printf("Error can't open input cameraModelDataInput.txt\n");
     prompt_and_exit(1);
   }

   printf("Example of how to use openCV to generate image control points for computing the camera model.\n\n");
   
   numberOfViews = 0;
   fscanf(fp_in, "%d", &numberOfViews);
   if (debug) {
      printf ("number of views: %d\n",numberOfViews);
   }   

   if (numberOfViews > 0) {

      end_of_file = fscanf(fp_in, "%s", filename);
      
      if (end_of_file != EOF) {
         strcpy(configurationPathAndFilename, data_dir.c_str());
         strcat(configurationPathAndFilename, filename);
         if (debug) {
            printf ("%s\n", configurationPathAndFilename);
           //prompt_and_continue();
         }

         end_of_file = fscanf(fp_in, "%s", controlPointsPathAndFilename);

         if (end_of_file != EOF) {
            strcpy(controlPointsPathAndFilename, data_dir.c_str());
            strcat(controlPointsPathAndFilename, filename);
            if (debug) {
               printf ("%s\n", controlPointsPathAndFilename);
               //prompt_and_continue();
            }
       
            /* write out the image control points */

            if ((fp_control_points = fopen(controlPointsPathAndFilename, "w")) == 0) {
	            printf("Error can't open input %s\n", controlPointsPathAndFilename);
               prompt_and_exit(1);
            }
            else {

               /* get the image control points */

               printf("\nCollecting image control points.\n");

               getImageControlPoints(string(configurationPathAndFilename),
                                     numberOfViews,
                                     &numberOfControlPoints,
                                     imagePoints);
                               
               if (debug) { 
                  for (i=0; i<numberOfControlPoints; i++) {
                      printf("%d %d\n", imagePoints[i].u, imagePoints[i].v);
                  }
               }

               for (i=0; i<numberOfControlPoints; i++) {
                  fprintf(fp_control_points, "%d %d\n", imagePoints[i].u, imagePoints[i].v);
               }
  
               fprintf(fp_control_points,"\n");
              
            }
         }
      }  
   }

   fclose(fp_in); 
   if (debug) prompt_and_continue();
   // end raw mode
   endwin();
   return 0;
}

