/*******************************************************************************************************************
*   Task-level robot programming for a LynxMotion AL5D robot arm
*   ------------------------------------------------------------
*
*   This application implements a simple robot program with frames (homogeneous transformations)
*   to position the end-effector at the top-left and the bottom-right control points of the camera model calibration grid.
*  
*   The control points are ordered as shown in the figure shown below.  
*   This program causes the robot to reach for control points 1 and 48.
*   
*    1   2   3   4   5   6   7   8
*
*    9  10  11  12  13  14  15  16
*
*   17  18  19  20  21  22  23  24
*
*   25  26  27  28  29  30  31  32
*
*   33  34  35  36  37  38  39  40
*
*   41  142  43  44  45 46  47  48
*
*   For a detailed description and example of the robot programming techniques employed, see the robotProgramming application.
*   
*   David Vernon, Carnegie Mellon University Africa
*   12 June 2018
*
*   Audit Trail
*   -----------
*   After replacing the heavy duty wrist with the light weight wrist,
*   changed the points reached for to 1 and 48 instead of 41 and 8.
*   David Vernon, 19 Febrary 2019
*
*******************************************************************************************************************/
#include "robotCalibration.h"

int main(int argc, char ** argv) {

   extern robotConfigurationDataType robotConfigurationData;


  /***********************************************************************************

   Robot Number 3 calibration
      
   Coordinates of the top-left and bottom-right control points on the calibration grid

   ************************************************************************************/

   float top_left_x   = -95;      
   float top_left_y   = 235;                                
   float top_left_z   = 0;

   float bottom_right_x    = 75;   
   float bottom_right_y    = 105;
   float bottom_right_z    = 0;



   bool debug = true;

   FILE *fp_in;                        // robotCalibration input file
   int end_of_file; 
   char filename[MAX_FILENAME_LENGTH];


   Frame E;
   Frame Z;
   Frame T5;
   Frame top_left;
   Frame bottom_right;
   Frame approach; 
   Frame home;

   float approach_distance;


   /*******************************************************************************

   Get the name of the file containing the robot configuration and calibration data 
   from the input file robotProgrammingInput.txt

   ********************************************************************************/

   if ((fp_in = fopen("../data/robotCalibrationInput.txt","r")) == 0) {
	   printf("Error can't open input robotCalibrationInput.txt\n");
      prompt_and_exit(0);
   }

   end_of_file = fscanf(fp_in, "%s", filename); // read the configuration filename
      
   if (end_of_file != EOF) {  // only proceed if there is a configuration file

      readRobotConfigurationData(filename);
    
      E = trans(0.0, 0.0, (float) robotConfigurationData.effector_z);      // end-effector (gripper) frame
      Z = trans(0.0 ,0.0, 0.0);                                            // robot base frame
      approach_distance = 100;

      home         = trans(0, 187, 216-(float) robotConfigurationData.effector_z) * roty(180.0) * rotz(-90.0); 
      top_left     = trans(top_left_x,     top_left_y,     top_left_z)     * roty(180.0) * rotz(-90.0); // top left control point frame
      bottom_right = trans(bottom_right_x, bottom_right_y, bottom_right_z) * roty(180.0) * rotz(-90.0); // bottom right control point frame
      approach     = trans(0,0,-approach_distance);                                                     // frame defined wrt control point frame


      /* home */
            
      T5 = inv(Z) * home * inv(E);

      if (move(T5) == false) 
         display_error_and_exit("move error ... quitting\n");;

      grasp(GRIPPER_CLOSED);     

      wait(3000);  // wait for 5 seconds
   

      /* top-left control point */

      T5 = inv(Z) * top_left * approach * inv(E);

      if (move(T5) == false) 
         display_error_and_exit("move error ... quitting\n");;

      wait(3000);  // wait for 5 seconds
   

      /* move to the control point  */
       
      T5 = inv(Z) * top_left * inv(E);

      if (move(T5) == false) 
         display_error_and_exit("move error ... quitting\n");;
 
      wait(3000);    


      /* move to  approach pose */

      T5 = inv(Z) * top_left * approach * inv(E);

      if (move(T5) == false) 
         display_error_and_exit("move error ... quitting\n");;

      wait(3000);  // wait for 5 seconds
   

      /* bottom-right control point */

      T5 = inv(Z) * bottom_right * approach * inv(E);

      if (move(T5) == false) 
         display_error_and_exit("move error ... quitting\n");;

      wait(3000);  
   

      /* move to the control point  */
       
      T5 = inv(Z) * bottom_right * inv(E);

      if (move(T5) == false) 
         display_error_and_exit("move error ... quitting\n");;
 
      wait(3000);    


      /* move to  approach pose */

      T5 = inv(Z) * bottom_right * approach * inv(E);

      if (move(T5) == false) 
         display_error_and_exit("move error ... quitting\n");;

      wait(2000);   

    
      /* home */
            
      T5 = inv(Z) * home * inv(E);

      if (move(T5) == false) 
         display_error_and_exit("move error ... quitting\n");;

      wait(3000);   
   
      /* park the arm in a pose that will cause the least movement when powered off and on again */

      goHome();
   }
   return 0;
}