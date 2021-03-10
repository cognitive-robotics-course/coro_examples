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
*   Ported to ROS for use with the Lynxmotion_AL5D simulator in Gazebo
*   Vinny Adjibi
*   9 February 2021
*
*   The option to spawn and kill a brick in the simulator to aid with debugging is now available
*   To switch this option on, set the create_brick variable to true
*   David Vernon
*   25 February 2021
*
*   Implemented piece-wise continuous path control for approach and depart phases of both the pick and the place actions
*   To switch this option on, set the continuous_path variable to true 
*   David Vernon
*   26 February 2021
*
*   Fixed a bug with the type of the color and name strings: changed from const char* to string 
*   so that they are compatible with the parameters of the spawn_brick() and kill_brick() functions
*   David Vernon
*   4 March 2021
*
*******************************************************************************************************************/

#include <stdlib.h>
#include <time.h>
#ifdef WIN32
    #include "pickAndPlace.h"
#else
    #include <module4/pickAndPlace.h>
#endif

int main(int argc, char ** argv) {
  
   #ifdef ROS
       ros::init(argc, argv, "pickAndPlace"); // Initialize the ROS system
   #endif

   extern robotConfigurationDataType robotConfigurationData;
   
   bool debug = true;
   
   FILE *fp_in;                    // pickAndPlace input file
   int  end_of_file; 
   char robot_configuration_filename[MAX_FILENAME_LENGTH];
   char filename[MAX_FILENAME_LENGTH] = {};
   char directory[MAX_FILENAME_LENGTH] = {};

   /* Frame objects */
   
   Frame E;
   Frame Z;
   Frame T6;
   Frame object;
   Frame object_grasp;
   Frame object_approach;
   Frame object_depart;
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
   float grasp_z           =   5;
   float grasp_theta       = 180;   // rotation in degrees about the y axis 
      
   float approach_distance;         // approach  distance from grasp pose in -z direction
   float depart_distance;           // departure distance from grasp pose in -z direction
   float initial_approach_distance; // start the approach from this distance
   float final_depart_distance;     // start the approach from this distance
   float delta;                     // increment in approach and depart distance 

   bool continuous_path = true;    // if true, implement approximation of continuous path control
                                    // when approaching and departing the grasp pose
                                    // otherwise just move directly from the initial approach pose to the grasp pose
                                    // and directly from the grasp pose to the final depart pose 

#ifdef ROS   
   bool create_brick = true;       // if true, spawn a brick at the specified location
   
   string name       = "brick1";    // name and colors for option to spawn and kill a brick
   string colors[3]  = {"red", "green", "blue"};
#endif

   
   /* open the input file */
   /* ------------------- */

   /* Set the filename. Different directories for ROS and Windows versions */
   
#ifdef ROS
    strcat(directory, (ros::package::getPath(ROS_PACKAGE_NAME) + "/data/").c_str());
#else
    strcat(directory, "../data/");// On Windows the exec is in bin, so we go in the parent directory first
#endif

    strcpy(filename, directory);
    strcat(filename, "pickAndPlaceInput.txt"); // Input filename matches the application name
   if ((fp_in = fopen(filename, "r")) == 0) {
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

   strcpy(filename, robot_configuration_filename);
   strcpy(robot_configuration_filename, directory);
   strcat(robot_configuration_filename, filename);

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

   
#ifdef ROS

   /* If we are using the simulator on ROS, we can instantiate the brick here to help with debugging */
   /* Normally, we would instantiate the brick using the terminal to mimic the way we would do it    */
   /* when using the physical robot, i.e. manually positioning it for the robot to pick and place    */

   if (create_brick) {
     
       /* Spawn the brick at the specified position */
       /* Randomly pick a color                     */

       srand(time(NULL));

       if (debug) {
          printf("Spawning brick with name %s at position (%.2f %.2f %.2f %.2f)\n",
	         name.c_str(), object_x, object_y, object_z, object_phi);
       }
     
       /* Call the utility function */
       
       spawn_brick(name, colors[rand() % 3], object_x, object_y, object_z, object_phi);
   }
#endif     
   
   /* now start the pick and place task */
   /* --------------------------------- */

   effector_length = (float) robotConfigurationData.effector_z; // initialized from robot configuration data
   initial_approach_distance = 20;
   final_depart_distance     = 20;
   delta = 2;
  
   E               = trans(0.0, 0.0, effector_length);                                           // end-effector (gripper) frame 
   Z               = trans(0.0 ,0.0, 0.0);                                                       // robot base frame
   object          = trans(object_x,      object_y,      object_z)      * rotz(object_phi);      // object pose
   destination     = trans(destination_x, destination_y, destination_z) * rotz(destination_phi); // destination pose
   object_grasp    = trans(grasp_x,       grasp_y,       grasp_z)       * roty(grasp_theta);     // object grasp frame w.r.t. both object and destination frames
   object_approach = trans(0,0,-initial_approach_distance);                                      // frame defined w.r.t. grasp frame
   object_depart   = trans(0,0,-final_depart_distance);                                          // frame defined w.r.t. grasp frame
 
   /* open the gripper */
   /* ----------------- */

   if (debug) printf("Opening gripper\n");
   
   grasp(GRIPPER_OPEN);

#ifdef ROS
      wait(5000); // wait to allow the simulator to go to the home pose before beginning
                  // we need to do this because the simulator does not initialize in the home pose
#endif

   

   /* move to the pick approach pose */
   /* ------------------------------ */
   
   if (debug) printf("Moving to object approach pose\n");

   T6 = inv(Z) * object * object_grasp * object_approach * inv(E);

   if (move(T6) == false) display_error_and_exit("move error ... quitting\n");;

   wait(2000); 


   if (continuous_path) {

      /* incrementally decrease the approach distance */
     
      approach_distance = initial_approach_distance - delta;
   
      while (approach_distance >= 0) {
	
	 object_approach   = trans(0,0,-approach_distance);
	 
         T6 = inv(Z) * object * object_grasp * object_approach * inv(E);
	 
         if (move(T6) == false) display_error_and_exit("move error ... quitting\n");  

         approach_distance = approach_distance - delta;                              
      }
   }

   
   /* move to the pick pose */
   /* --------------------- */
      
   if (debug) printf("Moving to object pose\n");

   T6 = inv(Z) * object * object_grasp * inv(E);

   if (move(T6) == false) display_error_and_exit("move error ... quitting\n");

   wait(1000); 

   
   /* close the gripper */
   /* ----------------- */

   if (debug) printf("Closing gripper\n");

   grasp(GRIPPER_CLOSED);

   wait(1000);

           
   /* move to pick depart pose */
   /* ------------------------ */

   if (debug) printf("Moving to object depart pose\n");

   if (continuous_path) {

      /* incrementally increase depart distance */

      depart_distance = delta;
      
      while (depart_distance <= final_depart_distance) {

	 object_depart   = trans(0,0,-depart_distance);

         T6 = inv(Z) * object * object_grasp * object_depart * inv(E);          
                                                                              
         if (move(T6) == false) display_error_and_exit("move error ... quitting\n"); 

         depart_distance = depart_distance + delta;
      }
   }

   
   T6 = inv(Z) * object * object_grasp * object_depart * inv(E);

   if (move(T6) == false) display_error_and_exit("move error ... quitting\n");;

   wait(2000);

   
   /* move to destination approach pose */
   /* --------------------------------- */

   if (debug) printf("Moving to destination approach pose\n");

   object_approach = trans(0,0,-initial_approach_distance);
 
   T6 = inv(Z) * destination * object_grasp * object_approach * inv(E);

   if (move(T6) == false) display_error_and_exit("move error ... quitting\n");

   wait(2000);

      
   /* move to the destination pose */
   /* ---------------------------- */

   if (debug) printf("Moving to destination pose\n");
      
   if (continuous_path) {

      /* incrementally decrease approach distance */

      approach_distance = initial_approach_distance - delta;
   
      while (approach_distance >= 0) {
	
         object_approach   = trans(0,0,-approach_distance);
	
         T6 = inv(Z) * destination * object_grasp * object_approach * inv(E);   
                                                                                  
         if (move(T6) == false) display_error_and_exit("move error ... quitting\n");  

         approach_distance = approach_distance - delta;
      }
   }


   T6 = inv(Z) * destination * object_grasp * inv(E);

   if (move(T6) == false) display_error_and_exit("move error ... quitting\n");;
 
   wait(1000);

   
   /* open the gripper */
   /* ---------------- */

   if (debug) printf("Opening gripper\n");

   grasp(GRIPPER_OPEN);     
   wait(1000); 

   
   /* move to depart pose */
   /* ------------------- */

   if (debug) printf("Moving to destination depart pose\n");

   if (continuous_path) {

      depart_distance = delta;
      
      while (depart_distance <= final_depart_distance) {

	 object_depart   = trans(0,0,-depart_distance);

         T6 = inv(Z) * destination * object_grasp * object_depart * inv(E);          
                                                                              
         if (move(T6) == false) display_error_and_exit("move error ... quitting\n"); 

         depart_distance = depart_distance + delta;
      }
   }
   
   object_depart   = trans(0,0,-final_depart_distance);
      
   T6 = inv(Z) * destination * object_grasp * object_depart * inv(E);

   if (move(T6) == false) display_error_and_exit("move error ... quitting\n");

   wait(1000);

    
   goHome(); // this returns the robot to the home position so that when it's switched off 
             // it's in a pose that is close to the one that the servo-controller uses as its initial state
             // could also do this with a move() as show above

#ifdef ROS


   if (create_brick) {

       prompt_and_continue();
	
       /* Remove the brick for the next time  */
       /* ----------------------------------- */

       if (debug) {
	  printf("Killing brick named %s\n",name.c_str());
       }
     
       /* Call the utility function */
       
       kill_brick(name);
   }
#endif
   
   return 0;
}
