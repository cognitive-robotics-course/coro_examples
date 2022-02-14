/*******************************************************************************************************************
*   Example pick-and-place program for a LynxMotion AL5D robot arm
*   ---------------------------------------------------------------
*
*   This application implements a simple robot program to grasp a simple object (a Lego brick),
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
*   It is assumed that the input file is located in a data directory. The location of this directory depends on whether 
*   we are running on Ubuntu with ROS or on Windows.
*   This application implements robot programming with frames (homogeneous transformations)
*   using the Frame and Vector classes and auxilliary friend functions, all defined in the interface file
*  
*   The frames are defined in a Cartesian frame of reference and use an inverse kinematic model 
*   to map to the robot joint space.
*
*   Different AL5D robots are accommodated by reading the required calibration data from a robot-specific configuration file.
*   The configuration filename is provided in the robotProgrammingInput.txt file.
*   This allows the application to be easily adapted to the different robots used in class exercises.
*   The configuration file contains the following data, specified using key-value pairs
*
*   COM       <serial port name>
*   BAUD      <rate>
*   SPEED     <value>
*   CHANNEL   <servo 1 pin number> <servo 2 pin number> <servo 3 pin number> <servo 4 pin number> <servo 5 pin number> <servo 6 pin number> 
*   HOME      <servo 1 setpoint>   <servo 2 setpoint>   <servo 3 setpoint>   <servo 4 setpoint>   <servo 5 setpoint>   <servo 6 setpoint> 
*   DEGREE    <servo 1 pulses>     <servo 2 pulses>     <servo 3 pulses>     <servo 4 pulses>     <servo 5 pulses>     <servo 6 pulses>
*   WRIST     <wrist type>
*   CURRENT   <joint 1 angle> <joint 2 angle> <joint 3 angle> <joint 4 angle> <joint 5 angle> <gripper distance>
*   SIMULATOR <flag value>
*
*   The serial port name is assigned automatically by the operating system when the robot's USB-to-serial port is connected to the computer
*   e.g., COM0, COM6, ... 
*
*   The baud rate is also assigned automatically by the operating system when the robot's USB-to-serial port is connected to the computer
*   e.g., 9600
*
*   The speed value determines the default servo speed.  This is specified microseconds per second, e.g. 500
*   "For a better understanding of the speed argument, consider that 1000uS of travel will result in around 90° of rotation. 
*   A speed value of 100uS per second means the servo will take 10 seconds (divide 1000 by 100) to move 90°. 
*   Alternately, a speed value of 2000uS per second equates to 500mS (half a second) to move 90° (divide 1000 by 2000)."
*   Taken from lynxmotion_ssc-32u_usb_user_guide.pdf
*
*   The channel data specifies which pins on the SSC-32U servo-controller board are used for each servo motor.
*   Normally, servo 1 is controlled using pin 0, servo 2 using pin 1, ...   
*   However, sometimes different pins are used, e.g. in the case of the servo-controller for robot 1 where pin 2 malfunctions
*   and instead pin 6 is used to control servo 3.
*
*   The home data specifies the servo setpoint value required to position the robot in the pre-defined home position with 
*   joint angles 0, 1.57 radians, -1.57 radians, 0, 0, respectively, and gripper fully open (0.030 m).
*   These setpoint values are specified in microseconds: 
*   typically 500 microseconds corresponds to the servo-motor positioned at one extreme and 2500 microseconds to the other extreme.
*
*   The degree data specifies the pulse width per degree for each servo. In theory, these should be identical for all servos 
*   but in practice they are not and the value specified are determined using a calibration exercise. 
*
*   The wrist data specifies whether the wrist is a lightweight wrist or a heavy duty wrist. The key values are "lightweight" or "heavyduty"
*   This information is used to determine which direction the roll servo should turn (the heavy duty wrist reverses the direction)
*   
*   The current data specifies the initial joint angles in radians and the gripper distance in metres
*   This information is updated every time either setJointAngles() or grasp() is called 
*   and is used in constructing the message to be published on the ROS topic when using ROS to control the robot or the simulator
*
*   The simulator data specifies whether to use the simulator or the physical robot.  The key values are "TRUE" or "FALSE".
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
*   Changed the initialization of the E frame to use the x, y, and z values read from the configuration file, not just the z value
*   David Vernon
*   21 March 2021
*
*   Changed the gripper distance to 15 mm when grasping the object and changed the approach distance to 40 mm
*   David Vernon
*   18 October 2021
*
*   Added a flag to indicate whether or not the simulator is being used
*   David Vernon
*   28 October 2021
* 
*   Moved the simulator flag to the robotConfigurationData data structure and added added functionality to the 
*   readRobotConfigurationData() function to read the flag from the configuration file.
*   Removed the ROS conditional compilation (i.e. no longer focussing on backward compatability with Windows).
*   It's all ROS from now on.
*   David Vernon
*   11 February 2022
*
*******************************************************************************************************************/

#include <stdlib.h>
#include <time.h>
#include <module4/pickAndPlace.h>
#include <module4/lynxmotionUtilities.h>


int main(int argc, char ** argv) {
  
   ros::init(argc, argv, "pickAndPlace"); // Initialize the ROS system

   extern robotConfigurationDataType robotConfigurationData;

   
   bool debug = true;              // set this to false for silent mode

   FILE *fp_in;                    // pickAndPlace input file
   int  end_of_file; 
   char robot_configuration_filename[MAX_FILENAME_LENGTH];
   char filename[MAX_FILENAME_LENGTH]  = {};
   char directory[MAX_FILENAME_LENGTH] = {};
   int  small_delay = 200;
   
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

   bool continuous_path = false;     // if true, implement approximation of continuous path control
                                    // when approaching and departing the grasp pose
                                    // otherwise just move directly from the initial approach pose to the grasp pose
                                    // and directly from the grasp pose to the final depart pose 

   bool   create_brick = true;     // if true, spawn a brick at the specified location when using the simulator
   string name       = "brick1";    // name and colors for option to spawn and kill a brick
   string colors[3]  = {"red", "green", "blue"};

   
   /* open the input file */
   /* ------------------- */

   /* Set the filename */
   
    strcat(directory, (ros::package::getPath(ROS_PACKAGE_NAME) + "/data/").c_str());
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

   
   /* If we are using the simulator on ROS, we can instantiate the brick here to help with debugging */
   /* Normally, we would instantiate the brick using the terminal to mimic the way we would do it    */
   /* when using the physical robot, i.e. manually positioning it for the robot to pick and place    */

   if (robotConfigurationData.simulator) {
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
   }
   
   
   /* now start the pick and place task */
   /* --------------------------------- */

   initial_approach_distance = 60; // 40
   final_depart_distance     = 60; // 40
   delta                     = 5;
  
   E               = trans((float) robotConfigurationData.effector_x,                            // end-effector (gripper) frame
	                   (float) robotConfigurationData.effector_y,                            // is initialized from data
	                   (float) robotConfigurationData.effector_z);                           // in the robot configuration file
   
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

   if (robotConfigurationData.simulator) {
      wait(2000); // wait to allow the simulator to go to the home pose before beginning
                  // we need to do this because the simulator does not initialize in the home pose
   }
   

   /* move to the pick approach pose */
   /* ------------------------------ */
   
   if (debug) printf("Moving to object approach pose\n");

   T6 = inv(Z) * object * object_grasp * object_approach * inv(E);

   if (move(T6) == false) display_error_and_exit("move error ... quitting\n");;

   wait(5000); 


   if (continuous_path) {

      /* incrementally decrease the approach distance */
     
      approach_distance = initial_approach_distance - delta;
   
      while (approach_distance >= 0) {
	
	 object_approach   = trans(0,0,-approach_distance);
	 
         T6 = inv(Z) * object * object_grasp * object_approach * inv(E);
	 
         if (move(T6) == false) display_error_and_exit("move error ... quitting\n");  

	 wait(small_delay);
	 
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

   grasp(GRIPPER_CLOSED); // just less than the width of the brick, in mm, to apply some lateral pressure

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

	 wait(small_delay);

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
	 
	 wait(small_delay);
	 
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

	 wait(small_delay);
		 
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


   if (robotConfigurationData.simulator)  {
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
   }
      
   return 0;
}
