/* 
  Example use of openCV to generate image control point data for computing the camera model
  ---------------------------------------------------------------------------------------------------

  This application acquires the image control points from the lynxmotion_al5d_description simulator
  camera which provide input to the cameraModel application.
  In addition, it also writes the world control points corresponding to the image control points from a calibration
  grid placed at different heights in the simulated world.

  In general, calibration requires that the calibration grid be presented in (at least) two different poses. 
  However, in some applications, the z values of all the world points are constrained  so that they are all equal 
  and constant (e.g. such as the situation where objects sit on a flat table) and you need only calibrate for 
  a single plane rather than for the full 3D volume. In this case, it is sufficient to have a single view of 
  the calibration grid resting on the surface to be calibrated. Note, however, that a camera model derived 
  from this single view will only yield valid inverse perspective mapping for the z value that 
  was used during calibration.

  The first line of the input file (camerModelDataROSInput.txt) should be a number between 1 and 10
  to identify the number of views to be used in calibration.

  The second line of the input file has three numbers that specify the x, y and z coordinate of the camera in that order.

  This is followed by four filenames.

  The first filename identifies the SDF file specifying the calibration grid to be placed in the simulator.

  The second filename identifies an input .xml file that contains the specification of the camera calibration setup,
  e.g. the number of points in the x and y direction in the calibration grid, the grid spacing, and another XML file
  specifying the image(s) to be used for calibration. The number of points to be used is best set to the number of
  inner corners in the checkerboard pattern in the x and y direction. The XML specifying images to be used for
  calibration is called simulCameraCalibImages.xml and the ABSOLUTE PATH to this filename must be set as the input
  in the camera calibration setup xml. The <Square_size> parameter in the XML file must be set to the box size used
  in the SDF file loaded in millimeters. The sizes in SDF are measured in meters.
  
  The third filename identifies the output .txt file where the 2D image control point coordinates are written.
  This file will be used by the cameraModel application.

  The fourth filename identifies the output .txt file where the 3D image control point coordinates are written.
  This file will be used by the cameraModel application.

  Before running this application ensure that the calibration grid is not occluded by the robot. A
  program that moves the robot out of the field of view of the camera is provided as part of module5 of coro_examples
  repository (https://github.com/cognitive-robotics-course/coro_examples) and can be run by the command
  'rosrun module5 moveRobot'

  It is assumed that the input file is located in a data directory given by the path ../data/
  defined relative to the location of source code for this application.

  When this application is run, it places the checkerboard pattern provided in the floor of the simulator. The x and y
  coordinates of the center of the placed checkerboard are always the same as the camera coordinates provided. The
  application then takes 3 images per view, saves the images to <DATA_DIR>/<MEDIA> folder and loads them for calibration.
  The spawned calibration grid is removed from the simulator and is spawned at a position 20 millimeters higher and the
  image acquisition and calibration progress in the aforementioned manner.

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

  AUDIT TRAIL
  ----------------
  Calibration code extended to use a Lynxmotional5d_description simulator camera.
  Abrham Gebreselasie
  13 March 2021



*/

#include "module5/cameraModelDataSimulator.h"

Mat scene_image;
int imageCount = 0;

int main(int argc, char** argv) {
   #ifdef ROS
       // Turn off canonical terminal mode and character echoing
         static const int STDIN = 0;
         termios term, old_term;
         tcgetattr(STDIN, &old_term);
         tcgetattr(STDIN, &term);
         term.c_lflag &= ~(ICANON | ECHO);
         tcsetattr(STDIN, TCSANOW, &term);
   #endif


   const char*                 input_filename = "cameraModelDataSimulatorInput.txt";
   char                 input_path_and_filename[MAX_FILENAME_LENGTH];
   char                 data_dir[MAX_FILENAME_LENGTH];
   char                 file_path_and_filename[MAX_FILENAME_LENGTH];
   char                 checkerboard_sdf_filename[MAX_FILENAME_LENGTH];


   int end_of_file;
   bool debug = false;
   char configurationPathAndFilename[MAX_FILENAME_LENGTH];
   char controlPointsPathAndFilename[MAX_FILENAME_LENGTH];
   char worldPointsPathAndFilename[MAX_FILENAME_LENGTH];
   char filename[MAX_FILENAME_LENGTH];
   int i;
   int numberOfViews;
   float cameraX, cameraY, cameraZ;
   float boardZ;

   FILE *fp_in;
   FILE *fp_control_points;
   FILE *fp_world_points;

   imagePointType imagePoints[MAX_NUMBER_OF_CONTROL_POINTS];
   int numberOfControlPoints;

   strcpy(data_dir, ros::package::getPath(ROS_PACKAGE_NAME).c_str()); // get the package directory
   strcat(data_dir, "/data/");

   strcpy(input_path_and_filename, data_dir);
   strcat(input_path_and_filename, input_filename);

   ros::init(argc, argv, "cameraModelDataSimulator");

   ros::NodeHandle nh;
   image_transport::ImageTransport it(nh);
   image_transport::Subscriber sub = it.subscribe("/lynxmotion_al5d/external_vision/image_raw", 1, &imageMessageReceived);


   if ((fp_in = fopen(input_path_and_filename,"r")) == 0) {
	  printf("Error can't open input cameraModelDataSimulatorInput.txt\n");
     prompt_and_exit(1);
   }

   printf("Example of how to use openCV to generate image control points from simulator camera for computing the camera model.\n\n");

   numberOfViews = 0;

   fscanf(fp_in, "%d", &numberOfViews);
   if (debug) {
      printf ("number of views: %d\n", numberOfViews);
   }   

   if (numberOfViews > 0) {

      end_of_file = fscanf(fp_in, "%f %f %f", &cameraX, &cameraY, &cameraZ);

      if (end_of_file != EOF) {
          if (debug) {
              printf("Camera coordinates: (%f, %f, %f)", cameraX, cameraY, cameraZ);
          }

          end_of_file = fscanf(fp_in, "%s", filename);
          if (end_of_file != EOF)
          {
              strcpy(checkerboard_sdf_filename, data_dir);
              strcat(checkerboard_sdf_filename, filename);

              end_of_file = fscanf(fp_in, "%s", filename);
              if (end_of_file != EOF) {
                  strcpy(configurationPathAndFilename, data_dir);
                  strcat(configurationPathAndFilename, filename);

                  if (debug) {
                      printf ("%s\n", configurationPathAndFilename);
                      //prompt_and_continue();
                  }

                  end_of_file = fscanf(fp_in, "%s", filename);

                  if (end_of_file != EOF) {
                      strcpy(controlPointsPathAndFilename, data_dir);
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

                      end_of_file = fscanf(fp_in, "%s", filename);
                      if (end_of_file != EOF) {
                          strcpy(worldPointsPathAndFilename, data_dir);
                          strcat(worldPointsPathAndFilename, filename);

                          if (debug) {
                              printf("%s\n", controlPointsPathAndFilename);
                              //prompt_and_continue();
                          }

                          /* write out the image control points */

                          if ((fp_world_points = fopen(worldPointsPathAndFilename, "w")) == 0) {
                              printf("Error can't open input %s\n", worldPointsPathAndFilename);
                              prompt_and_exit(1);
                          }
                          else {
                              printf("Move the robot if necessary then press return to continue.\n");
                              getchar();

                              for (int h = 0; h < numberOfViews; h++)
                              {
                                  boardZ = h * 20.0; // board z-coordinate in millimeters
                                  spawn_checkerboard(checkerboard_sdf_filename, cameraX / 1000, cameraY / 1000,
                                                     boardZ / 1000, 0, 0, 0);

                                  printf("Calbiration grid now placed at Z = %3.4f mm\n", boardZ);
                                  printf("Waiting for images\n");
                                  imageCount = 0;

                                  // Use a sequence of three images for each view
                                  while (imageCount < 3) {
                                      ros::spinOnce();
                                  }

                                  /* get the image control points */

                                  printf("\nCollecting image control points.\n");

                                  // Use 1 image per view
                                  getImageControlPoints(configurationPathAndFilename,
                                                        1,
                                                        &numberOfControlPoints,
                                                        imagePoints, fp_world_points,
                                                        cameraX, cameraY, boardZ);

                                  delete_checkerboard();

                                  if (debug) {
                                      for (i = 0; i < numberOfControlPoints; i++) {
                                          printf("%d %d\n", imagePoints[i].u, imagePoints[i].v);
                                      }
                                  }

                                  for (i = 0; i < numberOfControlPoints; i++) {
                                      fprintf(fp_control_points, "%d %d\n", imagePoints[i].u, imagePoints[i].v);
                                  }

                              }
                              fprintf(fp_control_points, "\n");

                          }
                      }
                  }
              }
          }

      }
   }

   fclose(fp_in);
   fclose(fp_control_points);
   fclose(fp_world_points);
   if (debug) prompt_and_continue();

   #ifdef ROS
       // Reset terminal
       tcsetattr(STDIN, TCSANOW, &old_term);
   #endif
   return 0;
}