/* 
  Example use of openCV to generate world control point data for computing the camera model
  ---------------------------------------------------------------------------------------------------

  This application acquires the world control points that account for robot errors from the lynxmotion_al5d_description
  simulator camera which provide input to the cameraModel application and writes them to file. Because of different
  factors, when the simulated robot is instructed to go to a position (x, y, z) it will actually go to
  (x + e_x, y + e_y, z + e_z). That is, to actually place it at (x, y, z) it must be instructed to go to some coordinate
  (x + u_x, y + u_y, z + u_z).

  Let (px, py) be the image point coordinate corresponding to the world coordinate (x, y, z).
  Let the world coordinates of the form (x, y, z) be used to obtain a camera model.
  If this camera model is used for inverse persepective transformation, the inverse perspective model will output (x, y, z)
  as the world coordinate corresponding to (px, py). However, since the robot has errors it will end up going to (x + e_x, y + e_y, z + e_z).
  Thus, the coordinates (x + u_x, y + u_y, z + u_z) are used to obtain a camera model. That is, the coordinates that
  actually take the robot to (x, y, z).

  The corresponding image control points must first be acquired using the cameraModelDataSimulator application found
  in the coro_examples repository. Ensure that the same calibration grid, same box size setting, and board size width
  and height settings are used for this application and the cameraModelDataSimulator run that obtained the image control
  points. The number of views (the first line of the input file for cameraModelDataSimulator) must be set to 1.

  The first line of the input file (robotCamerModelDataSimulatorInput.txt) is the filename of the robot configuration
  file to use for the lynxmotion_al5d robot.

  The second line of the input file has three numbers that specify the x, y and z coordinate of the camera in that order.

  The third line identifies the SDF file specifying the calibration grid to be placed in the simulator.

  The fourth line contains three numbers which specify the board size width, board size height and square size parameters.
  These parameters must be set to the values used in the XML configuration file used when running cameraModelDataSimulator.

  The fifth line specifies the filename of a file where the computed world coordinates of the control points will be
  written to.

  When this application is run, it spawns the calibration grid, and a vertical line that passes through the top left
  control point. The robot is then moved to the coordinates of the control point. However, because of execution errors
  the robot will not be exactly above the control point. Thus, it is necessary to move the robot so it is directly above
  the top left control point. The following steps are recommended when making the fine adjustments.

  1. Hide the checkerboard by pressing SPACE
  2. Go to Gazebo and change the view to "The view from the bottom" option by
     - Clicking on change the view angle (Box with orange front icon)
     - Clicking the "View from the bottom"
     - Changing the "perspective" to "orthographic" from the drop down box
  3. Zoom in and move the robot so the red line passes through the opening in the robot's wrist
  4. Press q to save and move to the bottom right control point

  Once the above steps are completed for the top-left and bottom-right control points the coordinates of the remaining
  points are computed using linear interpolation and written to file.

*/

#include "module5/robotCameraModelDataSimulator.h"

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

   extern robotConfigurationDataType robotConfigurationData;

   const char*                 input_filename = "robotCameraModelDataSimulatorInput.txt";
   char                 input_path_and_filename[MAX_FILENAME_LENGTH];
   char                 data_dir[MAX_FILENAME_LENGTH];
   char                 file_path_and_filename[MAX_FILENAME_LENGTH];
   char                 checkerboard_sdf_filename[MAX_FILENAME_LENGTH];
   char                 cylinder_sdf_filename[MAX_FILENAME_LENGTH];

   int end_of_file;
   char robot_configuration_filename[MAX_FILENAME_LENGTH];
   bool debug = false;
   char controlPointsPathAndFilename[MAX_FILENAME_LENGTH];
   char worldPointsPathAndFilename[MAX_FILENAME_LENGTH];
   char filename[MAX_FILENAME_LENGTH];

   float cameraX, cameraY, cameraZ;
   float boardZ;
   float gripperX, gripperY, gripperZ;
   float stepSize = 5.0;
   float boxsize;
   float controlPointCoordinates[2][2];

   int nPointsX, nPointsY;

   bool showCheckerboard = true;

   FILE *fp_in;
   FILE *fp_control_points;
   FILE *fp_world_points;


   strcpy(data_dir, ros::package::getPath(ROS_PACKAGE_NAME).c_str()); // get the package directory
   strcat(data_dir, "/data/");

   strcpy(input_path_and_filename, data_dir);
   strcat(input_path_and_filename, input_filename);

   strcpy(cylinder_sdf_filename, data_dir);
   strcat(cylinder_sdf_filename, "cylinder.sdf");

   ros::init(argc, argv, "robotCameraModelDataSimulator");


   if ((fp_in = fopen(input_path_and_filename,"r")) == 0) {
	  printf("Error can't open input robotCameraModelDataSimulatorInput.txt\n");
     prompt_and_exit(1);
   }

   printf("Example of how to use openCV to generate image control points from simulator camera for computing the camera model.\n\n");

   reset();

   end_of_file = fscanf(fp_in, "%s", robot_configuration_filename); // read the configuration filename
   if (end_of_file != EOF) {
       if (debug) printf("Robot configuration filename %s\n", robot_configuration_filename);
       strcpy(filename, robot_configuration_filename);
       strcpy(robot_configuration_filename, data_dir);
       strcat(robot_configuration_filename, filename);

       readRobotConfigurationData(robot_configuration_filename);

       end_of_file = fscanf(fp_in, "%f %f %f", &cameraX, &cameraY, &cameraZ);
       if (end_of_file != EOF) {
           if (debug) {
               printf("Camera coordinates: (%f, %f, %f)", cameraX, cameraY, cameraZ);
           }

           end_of_file = fscanf(fp_in, "%s", filename);
           if (end_of_file != EOF) {
               strcpy(checkerboard_sdf_filename, data_dir);
               strcat(checkerboard_sdf_filename, filename);

               end_of_file = fscanf(fp_in, "%d %d %f", &nPointsX, &nPointsY, &boxsize);
               if (end_of_file != EOF) {

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

                       spawn_checkerboard(checkerboard_sdf_filename, cameraX / 1000, cameraY / 1000,
                                          boardZ / 1000, 0, 0, 0);

                       controlPointCoordinates[0][0] = cameraX - (nPointsX - 1) * boxsize / 2;
                       controlPointCoordinates[0][1] = cameraY + (nPointsY - 1) * boxsize / 2;
                       controlPointCoordinates[1][0] = cameraX + (nPointsX - 1) * boxsize / 2;
                       controlPointCoordinates[1][1] = cameraY - (nPointsY - 1) * boxsize / 2;

                       printf("\n*************************************************************\n");
                       printf("* Please ensure CAPS LOCK is OFF                            *\n");
                       printf("* Press key w to move to in the positive Y direction        *\n");
                       printf("* Press key s to move to in the negative Y direction        *\n");
                       printf("* Press key d to move to in the positive X direction        *\n");
                       printf("* Press key a to move to in the negative X direction        *\n");
                       printf("* Press key Shift + w to move to in the positive Z direction*\n");
                       printf("* Press key Shift + s to move to in the negative Z direction*\n");
                       printf("* Press e/c to increase decrease step size.                 *\n");
                       printf("* Press SPACE to show/hide checkerboard.                    *\n");
                       printf("* Press q to save and continue                              *\n");
                       printf("*************************************************************\n");

                       printf("\n Step size is currently %f\n", stepSize);

                       for (int i = 0; i < 2; i++)
                       {
                           gripperX = controlPointCoordinates[i][0];
                           gripperY = controlPointCoordinates[i][1];
                           gripperZ = 0;

                           spawn_model(cylinder_sdf_filename, LINE_MODEL_NAME, gripperX / 1000, gripperY / 1000,
                                       0.5, 0, 0, 0);

                           moveToCoordinates(gripperX, gripperY, 0, 0);


                           char c;

                           do {
                               printf("Use wasd to move robot:\n");
                               c = getchar();
                               //printf("%c", c);int
                               switch (c)
                               {
                                   case 'w':
                                       printf("Moving to positive y\n");
                                       fineAdjustmentMove(gripperX, gripperY + stepSize, gripperZ, 0);
                                       gripperY += stepSize;
                                       break;
                                   case 'a':
                                       printf("Moving to negative x\n");
                                       fineAdjustmentMove(gripperX - stepSize, gripperY, gripperZ, 0);
                                       gripperX -= stepSize;
                                       break;
                                   case 's':
                                       printf("Moving to positive x\n");
                                       fineAdjustmentMove(gripperX, gripperY - stepSize, gripperZ, 0);
                                       gripperY -= stepSize;
                                       break;
                                   case 'd':
                                       printf("Moving to negative y\n");
                                       fineAdjustmentMove(gripperX + stepSize, gripperY, gripperZ, 0);
                                       gripperX += stepSize;
                                       break;
                                   case 'W':
                                       printf("Moving to positive z\n");
                                       fineAdjustmentMove(gripperX, gripperY, gripperZ + stepSize, 0);
                                       gripperZ += stepSize;
                                       break;
                                   case 'S':
                                       printf("Moving to negative z\n");
                                       fineAdjustmentMove(gripperX, gripperY, gripperZ - stepSize, 0);
                                       gripperZ -= stepSize;
                                       break;
                                   case 'e':
                                       stepSize += 1;
                                       printf("\nStep size now set to %f\n", stepSize);
                                       break;
                                   case 'c':
                                       stepSize -= 1;
                                       if (stepSize < 1)
                                       {
                                           stepSize = 1;
                                       }
                                       printf("\nStep size now set to %f\n", stepSize);
                                       break;
                                   case ' ':
                                       if (showCheckerboard)
                                       {
                                           delete_checkerboard();
                                           showCheckerboard = false;
                                       }
                                       else
                                       {
                                           spawn_checkerboard(checkerboard_sdf_filename, cameraX / 1000, cameraY / 1000,
                                                              boardZ / 1000, 0, 0, 0);
                                           showCheckerboard = true;
                                       }
                                       break;

                               }
                           } while (c != 'q');
                           // printf("(%f, %f)", gripperX, gripperY);
                           controlPointCoordinates[i][0] = gripperX;
                           controlPointCoordinates[i][1] = gripperY;

                           delete_model(LINE_MODEL_NAME);
                       }
                       if (showCheckerboard)
                       {
                           delete_checkerboard();
                       }
                       // printf("(%f, %f)\n", (controlPointCoordinates[1][0] - controlPointCoordinates[0][0])/ 7,
                       //       (controlPointCoordinates[1][1] - controlPointCoordinates[0][1])/ 5);
                       Size2f adjustedBoxSize((controlPointCoordinates[1][0] - controlPointCoordinates[0][0]) / (nPointsX - 1),
                                       (controlPointCoordinates[1][1] - controlPointCoordinates[0][1]) / (nPointsY - 1));
                       writeWorldCoordinatesToFile(fp_world_points, controlPointCoordinates[0][0],
                                              controlPointCoordinates[0][1], adjustedBoxSize, Size(8, 6));
                       }
                   }

           }
       }

   }


   fclose(fp_in);
   fclose(fp_world_points);
   if (debug) prompt_and_continue();

   #ifdef ROS
       // Reset terminal
       tcsetattr(STDIN, TCSANOW, &old_term);
   #endif

    return 0;
}