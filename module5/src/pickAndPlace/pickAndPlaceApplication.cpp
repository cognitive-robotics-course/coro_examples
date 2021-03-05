/*******************************************************************************************************************
*   Example pick-and-place program for a LynxMotion AL5D robot arm
*   ---------------------------------------------------------------
*
*   This application implements a simple robot program to grasp a simple object (a block),
*   lift it up, and place it somewhere else.  
*   
*   The position and orientation (pose) of the object and the goal position are specified in the input file.
*   (The pickAndPlaceVision application uses a camera to determine the object pose.)
*
*   The program uses task-level programming using frames to specify the object, robot, and gripper poses.
*
*   This application reads three lines from an input file pickAndPlace.txt.
*
*   The first line contains a filename of the file with the robot calibration data, i.e. for the inverse kinematic solution.
*   This allows the program to be used with different robots (by specifying the corresponding calibration data file). 
*
*   The second line contains the object pose, i.e. the x, y, and z coordinates and the phi angle of the object (i.e. rotation about z).
*
*   The third line contains the destination pose, i.e. the x, y, and z coordinates and the phi angle of the destination (i.e. rotation about z).
*
*   It is assumed that the input file is located in a data directory given by the path ../data/ 
*   defined relative to the location of executable for this application.
*
*
*   David Vernon, Carnegie Mellon University Africa
*   4 February 2020
*
*   Audit Trail
*   -----------
*   Renamed approach and grasp frames to object_approach and object_grasp
*   Renamed setGripper() to grasp()
*   These changes were required to facilitate new re-factored implementation code
*   David Vernon
*   27 July 2020
*
*******************************************************************************************************************/

#include "pickAndPlace.h"

int main(int argc, char ** argv) {

   extern robotConfigurationDataType robotConfigurationData;
   bool debug = true;
   FILE *fp_in;                    // pickAndPlace input file
   int  end_of_file; 
   char robot_configuration_filename[MAX_FILENAME_LENGTH];

   /* Frame objects */
   
   Frame E;
   Frame Z;
   Frame T6;
   Frame object;
   Frame object_grasp;
   Frame object_approach;
   Frame destination;

   /* data variables */

   float effector_length;           // this is initialized from robot configuration file

   float object_x          = -40;   // default values; actual values are read from the input file
   float object_y          = 150;   //                         
   float object_z          =   0;   //                         
   float object_phi        = -90;   // rotation in degrees about the z (vertical) axis 
      
   float destination_x     =  40;   // default values; actual values are read from the input file                        
   float destination_y     = 150;
   float destination_z     =   0;
   float destination_phi   = -90;   // rotation in degrees about the z (vertical) axis 

   float grasp_x           =   0;   // grasp pose relative to object and destination poses                        
   float grasp_y           =   0;
   float grasp_z           =  10;
   float grasp_theta       = 180;   // rotation in degrees about the y axis 
      

   float approach_distance = 100;   // approach and departure distance from grasp pose in -z direction
  
   /* open the input file */
   /* ------------------- */

   if ((fp_in = fopen("../data/pickAndPlaceInput.txt","r")) == 0) {
	  printf("Error can't open input pickAndPlaceInput.txt\n");
      prompt_and_exit(0);
   }

   /* get the robot configuration data */
   /* -------------------------------- */

   end_of_file = fscanf(fp_in, "%s", robot_configuration_filename); // read the configuration filename   
   if (end_of_file == EOF) {   
	  printf("Fatal error: unable to read the robot configuration filename\n");
      prompt_and_exit(1);
   }
   if (debug) printf("Robot configuration filename %s\n", robot_configuration_filename);


   readRobotConfigurationData(robot_configuration_filename);

   /* get the object pose data */
   /* ------------------------ */

   end_of_file = fscanf(fp_in, "%f %f %f %f", &object_x, &object_y, &object_z, &object_phi);
   if (end_of_file == EOF) {   
	  printf("Fatal error: unable to read the object position and orientation\n");
      prompt_and_exit(1);
   }
   if (debug) printf("Object pose %f %f %f %f\n", object_x, object_y, object_z, object_phi);

   /* get the destination pose data */
   /* ----------------------------- */

   end_of_file = fscanf(fp_in, "%f %f %f %f", &destination_x, &destination_y, &destination_z, &destination_phi);
   if (end_of_file == EOF) {   
	  printf("Fatal error: unable to read the destination position and orientation\n");
      prompt_and_exit(1);
   }
   if (debug) printf("Destination pose %f %f %f %f\n", destination_x, destination_y, destination_z, destination_phi);
    
   /* now start the pick and place task */
   /* --------------------------------- */

   effector_length = (float) robotConfigurationData.effector_z; // initialized from robot configuration data
  
   E               = trans(0.0, 0.0, effector_length);                                           // end-effector (gripper) frame
   Z               = trans(0.0 ,0.0, 0.0);                                                       // robot base frame
   object          = trans(object_x,      object_y,      object_z)      * rotz(object_phi);      // object pose
   destination     = trans(destination_x, destination_y, destination_z) * rotz(destination_phi); // destination pose
   object_grasp    = trans(grasp_x,       grasp_y,       grasp_z)       * roty(grasp_theta);     // object grasp frame w.r.t. both object and destination frames
   object_approach = trans(0,0,-approach_distance);                                              // frame defined w.r.t. grasp frame
 
   /* close the gripper */
   /* ----------------- */

   grasp(GRIPPER_OPEN);     

   wait(1000);  //  1 second

   /* move to initial approach pose */
   /* ----------------------------- */
   
   if (debug) printf("\nInitial approach pose\n");

   T6 = inv(Z) * object * object_grasp * object_approach * inv(E);

   if (move(T6) == false) display_error_and_exit("move error ... quitting\n");;

   wait(4000);  // 2 seconds

 
   /* move to the grasp pose */
   /* ---------------------- */
      
   if (debug) printf("\nGrasp pose\n");

   T6 = inv(Z) * object * object_grasp * inv(E);

   if (move(T6) == false) display_error_and_exit("move error ... quitting\n");

   wait(2000);   // 2 seconds

   /* close the gripper */
   /* ----------------- */

   grasp(GRIPPER_CLOSED);     
   wait(2000); 
         
   /* move back to initial approach pose */
   /* ---------------------------------- */

   if (debug) printf("\nInitial approach pose (departing)\n");

   T6 = inv(Z) * object * object_grasp * object_approach * inv(E);

   if (move(T6) == false) display_error_and_exit("move error ... quitting\n");

   wait(3000);  // 2 seconds
   
   /* move to destination approach pose */
   /* --------------------------------- */

   if (debug) printf("\nDestination approach pose\n");

   T6 = inv(Z) * destination * object_grasp * object_approach * inv(E);

   if (move(T6) == false) display_error_and_exit("move error ... quitting\n");

   wait(3000); // 2 seconds
  
   /* move to the destination pose */
   /* ---------------------------- */
       
   if (debug) printf("\n pose\n");

   T6 = inv(Z) * destination * object_grasp * inv(E);

   if (move(T6) == false) display_error_and_exit("move error ... quitting\n");;
 
   wait(2000);  // 2 seconds  

   /* open the gripper */
   /* ---------------- */

   grasp(GRIPPER_OPEN);     
   wait(2000);  // 1 second

   /* move back to initial approach pose */
   /* ---------------------------------- */

   if (debug) printf("\nDestination approach pose (departing)\n");

   T6 = inv(Z) * destination * object_grasp * object_approach * inv(E);

   if (move(T6) == false) display_error_and_exit("move error ... quitting\n");;

   wait(3000);  // 2 seconds
    
   goHome(); // this // end raw mode
   endwin();
   returns the robot to the home position so that when it's switched off 
             // it's in a pose that is close to the one that the servo-controller uses as its initial state
             // could also do this with a move() as show above

   if (debug)
	   prompt_and_exit(0);

   // end raw mode
   endwin();
   return 0;
}