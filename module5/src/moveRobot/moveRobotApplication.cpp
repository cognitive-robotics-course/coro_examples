/*******************************************************************************************************************
*   Example code to move simulator robot out of field of view
*   ---------------------------------------------------------------
*   This program moves the Lynxmotion AL5D robot in the lynxmotion_al5d_description simulator out of the field of view
*   of the simulator camera.

*******************************************************************************************************************/

#include <stdlib.h>
#include <time.h>
#ifdef WIN32
    #include "pickAndPlace.h"
#else
    #include <module5/moveRobot.h>
#endif

Mat scene_image;

int main(int argc, char ** argv) {
  
   #ifdef ROS
       ros::init(argc, argv, "moveRobot"); // Initialize the ROS system
   #endif

   extern robotConfigurationDataType robotConfigurationData;
   
   bool debug = true;
   
   FILE *fp_in;                    // pickAndPlace input file
   int  end_of_file; 
   char robot_configuration_filename[MAX_FILENAME_LENGTH];
   char filename[MAX_FILENAME_LENGTH] = {};
   char directory[MAX_FILENAME_LENGTH] = {};



   
   /* Set the filename. Different directories for ROS and Windows versions */
   
#ifdef ROS
    strcat(directory, (ros::package::getPath(ROS_PACKAGE_NAME) + "/data/").c_str());
#else
    strcat(directory, "../data/");// On Windows the exec is in bin, so we go in the parent directory first
#endif

    strcpy(filename, directory);
    strcat(filename, "moveRobotInput.txt"); // Input filename matches the application name
   if ((fp_in = fopen(filename, "r")) == 0) {
      printf("Error can't open input moveRobotInput.txt\n");
      prompt_and_exit(0);
   }

   
   /* get the robot configuration data */
   /* -------------------------------- */
   strcpy(robot_configuration_filename, "robot_3_config.txt");

   strcpy(filename, robot_configuration_filename);
   strcpy(robot_configuration_filename, directory);
   strcat(robot_configuration_filename, filename);

   readRobotConfigurationData(robot_configuration_filename);


   
   /* get the destination pose data */
   /* ----------------------------- */
   printf("Moving robot out of field of view\n");
   leave_field_of_view();
   printf("Moved robot out of field of view\n");
   goHome(); // this returns the robot to the home position so that when it's switched off
             // it's in a pose that is close to the one that the servo-controller uses as its initial state
             // could also do this with a move() as show above

   return 0;
}
