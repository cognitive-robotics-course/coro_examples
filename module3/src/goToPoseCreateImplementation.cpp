/*******************************************************************************************************************
*   
*  Exercise 1: Implementation of the divide-and-conquer go-to-position algorithm (goto1) 
*              and the MIMO go-to-position algorithm (goto2)
*              on the iRobot Create 2 mobile robot
*
*   This is the implementation file.
*   For documentation, please see the application file
*
*   David Vernon
*   1 November 2021
*
*   Audit Trail
*   -----------
*
*******************************************************************************************************************/

#include <module3/goToPoseCreate.h> 
  

/******************************************************************************

global variables with the current robot pose

*******************************************************************************/

/* global variables with the current robot pose */


float                  current_x        = 0; 
float                  current_y        = 0; 
float                  current_theta    = 0;
float                  odom_x           = 0;
float                  odom_y           = 0;
float                  odom_theta       = 0;
float                  adjustment_x     = 0;
float                  adjustment_y     = 0;
float                  adjustment_theta = 0;



/******************************************************************************

odomMessageReceived

Callback function, executed each time a new pose message arrives 

*******************************************************************************/


void odomMessageReceived(const nav_msgs::Odometry& msg) {
  bool debug = true;

   float x, y;
  
   odom_x     = msg.pose.pose.position.x;
   odom_y     = msg.pose.pose.position.y;
   odom_theta = 2 * atan2(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w);

   /* change frame of reference from arbitrary odometry frame of reference to the world frame of reference */
   
   /* translation of origin */
  
   x = odom_x + adjustment_x;
   y = odom_y + adjustment_y;
   
   /* rotation about origin */

   current_x = x * cos(adjustment_theta) + y * -sin(adjustment_theta);
   current_y = x * sin(adjustment_theta) + y * cos(adjustment_theta);
   
   current_theta = odom_theta + adjustment_theta;

   /* check to ensure theta is still in the range -PI to +PI */
   
   if (current_theta < - PI)
     current_theta += 2*PI;
   else if (current_theta > PI)
     current_theta -= 2*PI;
   
   
   // printf("odom_x,y,theta %5.3f %5.3f %5.3f; adjustment_x,y,theta  %5.3f %5.3f %5.3f; x, y %5.3f %5.3f; current_x,y,theta %5.3f %5.3f %5.3f\n",  odom_x, odom_y, odom_theta, adjustment_x, adjustment_y, adjustment_theta, x, y, current_x, current_y, current_theta);
   
   
   if (debug) {
     printf("Odometry: position = (%5.3f, %5.3f) orientation = %5.3f\n",current_x,current_y,current_theta);
   }
}



/*******************************************************************************

readLocomotionParameterData

Read locomotion parameters from file     

*******************************************************************************/

void readLocomotionParameterData(char filename[], struct locomotionParameterDataType *locomotionParameterData) {
      
   bool debug = false;
   int i; 
   int j;

   keyword keylist[NUMBER_OF_KEYS] = {
      "position_tolerance",
      "angle_tolerance_orienting",
      "angle_tolerance_going",
      "position_gain_dq",
      "angle_gain_dq",
      "position_gain_mimo",
      "angle_gain_mimo",
      "min_linear_velocity",
      "max_linear_velocity",
      "min_angular_velocity",
      "max_angular_velocity"
   };

   keyword key;                  // the key string when reading parameters
   keyword value;                // the value string, used for the WRIST key

   char input_string[STRING_LENGTH];
   FILE *fp_config;       

   if ((fp_config = fopen(filename,"r")) == 0) {
      printf("Error can't open locomition parameter file %s\n",filename);
      prompt_and_exit(0);
   }

   /*** set default values ***/

   locomotionParameterData->position_tolerance        = 0.025;
   locomotionParameterData->angle_tolerance_orienting = 0.075;
   locomotionParameterData->angle_tolerance_going     = 0.075; 
   locomotionParameterData->position_gain_dq          = 0.3;
   locomotionParameterData->angle_gain_dq             = 0.3;
   locomotionParameterData->position_gain_mimo        = 0.2;
   locomotionParameterData->angle_gain_mimo           = 0.5;
   locomotionParameterData->min_linear_velocity       = 0.015;
   locomotionParameterData->max_linear_velocity       = 0.5;
   locomotionParameterData->min_angular_velocity      = 0.09;  
   locomotionParameterData->max_angular_velocity      = 1.0;


   /*** get the key-value pairs ***/

   for (i=0; i<NUMBER_OF_KEYS; i++) {
		
      fgets(input_string, STRING_LENGTH, fp_config);
      //if (debug)  printf ("Input string: %s",input_string);

      /* extract the key */

      sscanf(input_string, " %s", key);

      for (j=0; j < (int) strlen(key); j++)
         key[j] = tolower(key[j]);
       
      //if (debug)  printf ("key: %s\n",key);

      for (j=0; j < NUMBER_OF_KEYS; j++) {
         if (strcmp(key,keylist[j]) == 0) {
            switch (j) {
            case 0:  sscanf(input_string, " %s %f", key, &(locomotionParameterData->position_tolerance));         // position_tolerance
                     break;
            case 1:  sscanf(input_string, " %s %f", key, &(locomotionParameterData->angle_tolerance_orienting));  // angle_tolerance_orienting
                     break;
	    case 2:  sscanf(input_string, " %s %f", key, &(locomotionParameterData->angle_tolerance_going));      // angle_tolerance_going
                     break;
	    case 3:  sscanf(input_string, " %s %f", key, &(locomotionParameterData->position_gain_dq));           // position_gain_dq)
                     break;
            case 4:  sscanf(input_string, " %s %f", key, &(locomotionParameterData->angle_gain_dq));              // angle_gain_dq
                     break;
            case 5:  sscanf(input_string, " %s %f", key, &(locomotionParameterData->position_gain_mimo));         // position_gain_mimo
                     break;
	    case 6:  sscanf(input_string, " %s %f", key, &(locomotionParameterData->angle_gain_mimo));            // angle_gain_mimo
                     break;
	    case 7:  sscanf(input_string, " %s %f", key, &(locomotionParameterData->min_linear_velocity));        // min_linear_velocity
                     break;
            case 8:  sscanf(input_string, " %s %f", key, &(locomotionParameterData->max_linear_velocity));        // max_linear_velocity
                     break;
            case 9:  sscanf(input_string, " %s %f", key, &(locomotionParameterData->min_angular_velocity));       // min_angular_velocity
                     break;
	    case 10: sscanf(input_string, " %s %f", key, &(locomotionParameterData->max_angular_velocity));       // max_angular_velocity
                     break;
            }
         }
      }
   }

   if (debug) { 
      printf("POSITION_TOLERANCE:        %f\n",locomotionParameterData->position_tolerance);
      printf("ANGLE_TOLERANCE_ORIENTING: %f\n",locomotionParameterData->angle_tolerance_orienting);
      printf("ANGLE_TOLERANCE_GOING:     %f\n",locomotionParameterData->angle_tolerance_going);
      printf("POSITION_GAIN_DQ:          %f\n",locomotionParameterData->position_gain_dq); 
      printf("ANGLE_GAIN_DQ:             %f\n",locomotionParameterData->angle_gain_dq); 
      printf("POSITION_GAIN_MIMO:        %f\n",locomotionParameterData->position_gain_mimo); 
      printf("ANGLE_GAIN_MIMO:           %f\n",locomotionParameterData->angle_gain_mimo);
      printf("MIN_LINEAR_VELOCITY:       %f\n",locomotionParameterData->min_linear_velocity);
      printf("MAX_LINEAR_VELOCITY:       %f\n",locomotionParameterData->max_linear_velocity);
      printf("MIN_ANGULAR_VELOCITY:      %f\n",locomotionParameterData->min_angular_velocity);
      printf("MAX_ANGULAR_VELOCITY:      %f\n",locomotionParameterData->max_angular_velocity);
   }
}




/*********************************************************************************

findMinimumVelocities

Find the minimum linear and angular velocities that alter the odometry data   

**********************************************************************************/

void findMinimumVelocities(ros::Publisher pub, ros::Rate rate, float max_linear_velocity,  float max_angular_velocity) {
  
   geometry_msgs::Twist msg;
   
   float                min_angular_velocity;
   float                min_linear_velocity;

   float                temp_x;
   float                temp_y;
   float                temp_theta;
   
   float                step = 0.01;  // the decrement we use when identifying the minimum linear and angular velocities.

   sleep(1); // allow time for messages to be published on the odom topic
   ros::spinOnce();
      
   min_linear_velocity = max_linear_velocity; // this is the maximum allowable linear velocity
   
   do {
      temp_x = current_x;
      temp_y = current_y;
 
      min_linear_velocity = min_linear_velocity - step;
      
      msg.linear.x = min_linear_velocity;       
      msg.angular.z = 0;;
        
      pub.publish(msg);              // Publish the message

      rate.sleep(); // Wait until it's time for another iteration

      sleep(1);
      ros::spinOnce();

      printf("temp_x,y %f, %f; current_x,y %f %f; min_linear_velocity %f\n", temp_x, temp_y, current_x, current_y, min_linear_velocity);
      
   } while ((temp_x != current_x) || (temp_y != current_y));

   min_linear_velocity += step; // use the last value that produced a change in either x or y
   
   printf("Minimum linear velocity = %f\n", min_linear_velocity);

   
   min_angular_velocity = max_angular_velocity; // this is the maximum allowable angular velocity

   do {
      temp_theta = current_theta;

      min_angular_velocity = min_angular_velocity - step;
      
      msg.linear.x = 0;       
      msg.angular.z = min_angular_velocity;
        
      pub.publish(msg); // Publish the message

      rate.sleep(); // Wait until it's time for another iteration

      sleep(1);
      ros::spinOnce();

      printf("temp_theta %f; current_theta %f; min_angular_velocity %f\n", temp_theta, current_theta,  min_angular_velocity);
      
   } while (temp_theta != current_theta);

   min_angular_velocity += step; // use the last value that produced a change in either x or y

   printf("Minimum angular velocity = %f\n", min_angular_velocity);

}


/*********************************************************************************

setOdometryPose

Initialize the pose returned by the callback that services the subscription to the odom topic         
                                                                                                       
Odometry provides relative position orientation. Since we can't assume what odometry data is published
on the odom topic on start up, we use two extra sets of variables for the x, y, and theta values:     
adjustment variables and current variables.                                                           
                                                                                                      
We set the values of the adjustment variables to be the difference between                            

(a) the values associated with the start pose, and                                                    
(b) the values published on the odom topic on start up (or whenever we reinitialize the odometry),    
                                                                                                      
The callback then sets the values of the current variables as follows.

- the current x and y values are set to the sum of the adjustment x and y values and the odom x and y values 
  (this effectively translates the odom x and y values by the adjustment x and y values) 

- these translated values are then rotated about the Z axis by an angle equal to the difference 
  between the start theta value  and the odom theta value

- the current theta value is set to be the sum of the adjustment theta value and the odom theta value                                                            

**********************************************************************************/

void setOdometryPose(float x, float y, float theta) {

  bool debug = false;
  
   sleep(1); // allow time for messages to be published on the odom topic
   ros::spinOnce();
   
   adjustment_x     = x     - odom_x;
   adjustment_y     = y     - odom_y;
   adjustment_theta = theta - odom_theta;
      
   sleep(1); // allow time for adjusted  messages to be published on the odom topic
      
   if (debug) {
      printf("odom_x,y,theta %f %f %f  adjustment_x, y, theta %f %f %f\n", odom_x, odom_y, odom_theta,
                                                                           adjustment_x,  adjustment_y,  adjustment_theta);
   }
}

/******************************************************************************

goToPoseDQ

Use the divide and conquer algorithm to drive the robot to a given pose

*******************************************************************************/

void goToPoseDQ(float x, float y, float theta, locomotionParameterDataType locomotionParameterData, ros::Publisher pub, ros::Rate rate) {

   bool                 debug = false;
  
   geometry_msgs::Twist msg; 

   float                start_x;
   float                start_y;
   float                start_theta;

   float                goal_x;
   float                goal_y;
   float                goal_theta;

   float                goal_direction;
   
   float                position_error;
   float                angle_error;
 
   float                angular_velocity;
   float                linear_velocity;
   float                current_linear_velocity = 0;

   int                  number_of_ramp_up_steps = 20;
   
   int                  mode; // GOING or ORIENTING
   
   goal_x     = x;
   goal_y     = y;
   goal_theta = theta;

   mode = ORIENTING;  // divide and conquer always starts by adjusing the heading
   
   do {

      /* get the current pose */

      ros::spinOnce();      // Let ROS take over to handle the callback and publish the pose on the odom topic

      position_error = sqrt((goal_x - current_x)*(goal_x - current_x) +
		            (goal_y - current_y)*(goal_y - current_y));


      goal_direction = atan2((goal_y - current_y),(goal_x - current_x));
      angle_error = goal_direction - current_theta;

      /* The absolute error in direction can not be greater than Pi because the robot can rotate in two directions,             */
      /* positive and negative. Thus it can eliminate any angular error by rotating Pi radians or less in the correct direction */
      /* If the difference in directions is greater than +Pi, subtract  2 Pi; if it is less than -Pi, add 2 Pi                  */

      if (angle_error > PI) {
         angle_error = angle_error - 2 * PI;
      }
      else if (angle_error < -PI) {
         angle_error = angle_error + 2 * PI;
      }
	    
      //if (fabs(angle_error) > locomotionParameterData.angle_tolerance_orienting) {
      if (((mode == ORIENTING) && (fabs(angle_error) > locomotionParameterData.angle_tolerance_orienting)) ||  // low angular tolerance when orienting to get the best initial heading
	  ((mode == GOING)     && (fabs(angle_error) > locomotionParameterData.angle_tolerance_going)) ) {     // high angular tolerance when going so we don't have to correct the heading too often
	  
         /* if the robot is not oriented correctly, adjust the heading */

         if (debug) printf("Orienting\n");

         mode = ORIENTING;  // reset mode from GOING to ORIENTING to ensure we use the lower angular tolerance when reorienting 
	 
         /* set linear and angular velocities, taking care not to use values that exceed maximum values */
         /* or use values that are less than minimum values needed to produce a response in the robot   */
	    
         msg.linear.x  = 0;

         angular_velocity = locomotionParameterData.angle_gain_dq * angle_error;
	       
         if (fabs(angular_velocity) < locomotionParameterData.min_angular_velocity)
             msg.angular.z = locomotionParameterData.min_angular_velocity * signnum(angular_velocity);
         else if (fabs(angular_velocity) > locomotionParameterData.max_angular_velocity)
             msg.angular.z = locomotionParameterData.max_angular_velocity * signnum(angular_velocity);
         else
             msg.angular.z = angular_velocity;
      }
      else if (position_error > locomotionParameterData.position_tolerance) {

 	 mode = GOING;
	
         /* if the robot has not reached the goal, adjust the distance */

         if (debug) printf("Going\n");

         /* set linear and angular velocities, taking care not to use values that exceed maximum values */
         /* or use values that are less than minimum values needed to produce a response in the robot   */
	    
	 linear_velocity = locomotionParameterData.position_gain_dq * position_error;
	       
         if (linear_velocity < locomotionParameterData.min_linear_velocity)
	    linear_velocity = locomotionParameterData.min_linear_velocity;
         else if (linear_velocity > locomotionParameterData.max_linear_velocity)
     	    linear_velocity = locomotionParameterData.max_linear_velocity;

	 /* if stopped, ramp up to the required velocity ... don't attempt an infinite acceleration to the required velocity */

	 if (current_linear_velocity == 0) {
	   
	   for (int i=1; i < number_of_ramp_up_steps; i++) {
              msg.linear.x = (float) linear_velocity * ((float) i / (float) number_of_ramp_up_steps);
	      msg.angular.z = 0;

              if (debug) {
	         printf("Ramping up velocity\n");
                 //printf("Current pose:         %5.3f %5.3f %5.3f\n", current_x, current_y, current_theta);
                 //printf("Goal, heading, theta: %5.3f, %5.3f, %5.3f\n", goal_theta, goal_direction, current_theta);
                 printf("Error:                %5.3f, %5.3f\n", position_error, angle_error);
                 printf("velocity command:     %5.3f, %5.3f\n", msg.linear.x, msg.angular.z);
              }
	      
	      pub.publish(msg);              // Publish the message
	     
              rate.sleep();                  // Wait until it's time for another iteration
           }
	   current_linear_velocity = linear_velocity;
         }
      
         msg.linear.x = linear_velocity;	       
	 msg.angular.z = 0;
      }

      if (debug) {
         //printf("Current pose:         %5.3f %5.3f %5.3f\n", current_x, current_y, current_theta);
         //printf("Goal, heading, theta: %5.3f, %5.3f, %5.3f\n", goal_theta, goal_direction, current_theta);
         printf("Error:                %5.3f, %5.3f\n", position_error, angle_error);
	 printf("velocity command:     %5.3f, %5.3f\n\n", msg.linear.x, msg.angular.z);
      }
	    
      pub.publish(msg);              // Publish the message

      rate.sleep(); // Wait until it's time for another iteration
	    
   } while ((position_error > locomotionParameterData.position_tolerance) && ros::ok());

	      
   /* the robot has reached the destination so             */
   /* adjust the orientation to match the goal orientation */
	 
   do {
	   
      if (debug) printf("Orienting\n");

      /* get the current pose */

      ros::spinOnce();      // Let ROS take over to handle the callback

      angle_error = goal_theta - current_theta;

      /* The absolute error in direction can not be greater than Pi because the robot can rotate in two directions,            */
      /* positive and negative. Thus it can eliminate any angular error by rotating Pi radian or less in the correct direction */
      /* If the difference in directions is greater than +Pi, subtract  2 Pi; if it is less than -Pi, add 2 Pi                 */

      if (angle_error > PI) {
         angle_error = angle_error - 2 * PI;
      }
      else if (angle_error < -PI) {
         angle_error = angle_error + 2 * PI;
      }
	    
      msg.linear.x  = 0;          

      /* set linear and angular velocities, taking care not to use values that exceed maximum values */
      /* or use values that are less than minimum values needed to produce a response in the robot   */
	    
      angular_velocity = locomotionParameterData.angle_gain_dq * angle_error;
	       
      if (fabs(angular_velocity) < locomotionParameterData.min_angular_velocity)
          msg.angular.z = locomotionParameterData.min_angular_velocity * signnum(angular_velocity);
      else if (fabs(angular_velocity) > locomotionParameterData.max_angular_velocity)
          msg.angular.z = locomotionParameterData.max_angular_velocity * signnum(angular_velocity);
      else
          msg.angular.z = angular_velocity;

      if (debug) {
 	 printf("Orienting\n");
         //printf("Current pose:         %5.3f %5.3f %5.3f\n", current_x, current_y, current_theta);
         //printf("Goal, heading, theta: %5.3f, %5.3f, %5.3f\n", goal_theta, goal_direction, current_theta);
         printf("Error:                %5.3f, %5.3f\n", position_error, angle_error);
         printf("velocity command:     %5.3f, %5.3f\n\n", msg.linear.x, msg.angular.z);
     }
	    
     pub.publish(msg);              // Publish the message

     rate.sleep(); // Wait until it's time for another iteration
	    
   } while( (fabs(angle_error) > locomotionParameterData.angle_tolerance_orienting) && ros::ok());
 }


 
/**********************************************************************************************************************

goToPoseMIMO1

Use the MIMO algorithm to drive the robot to a given position and then adjust orientation to achieve the required  pose

***********************************************************************************************************************/

void goToPoseMIMO1(float x, float y, float theta, locomotionParameterDataType locomotionParameterData, ros::Publisher pub, ros::Rate rate) {

   bool                 debug = false;
  
   geometry_msgs::Twist msg; 

   float                start_x;
   float                start_y;
   float                start_theta;

   float                goal_x;
   float                goal_y;
   float                goal_theta;

   float                goal_direction;
   
   float                position_error;
   float                angle_error;
 
   float                angular_velocity;
   float                linear_velocity;

   float                current_linear_velocity = 0;

   int                  number_of_ramp_up_steps = 20;
   
   goal_x     = x;
   goal_y     = y;
   goal_theta = theta;
	 
   do {

      /* get the current pose */

      ros::spinOnce();      // Let ROS take over to handle the callback

      position_error = sqrt((goal_x - current_x)*(goal_x - current_x) +
		            (goal_y - current_y)*(goal_y - current_y));

      goal_direction = atan2((goal_y - current_y),(goal_x - current_x));
      angle_error = goal_direction - current_theta;


      /* The absolute error in direction can not be greater than Pi because the robot can rotate in two directions,            */
      /* positive and negative. Thus it can eliminate any angular error by rotating Pi radian or less in the correct direction */
      /* If the difference in directions is greater than +Pi, subtract  2 Pi; if it is less than -Pi, add 2 Pi                 */

      if (angle_error > PI) {
         angle_error = angle_error - 2 * PI;
      }
      else if (angle_error < -PI) {
         angle_error = angle_error + 2 * PI;
      }

      /* set linear and angular velocities, taking care not to use values that exceed maximum values */
      /* or use values that are less than minimum values needed to produce a response in the robot   */
	    
      angular_velocity = locomotionParameterData.angle_gain_mimo * angle_error;
	       
      if (fabs(angular_velocity) < locomotionParameterData.min_angular_velocity)
         msg.angular.z = locomotionParameterData.min_angular_velocity * signnum(angular_velocity);
      else if (fabs(angular_velocity) > locomotionParameterData.max_angular_velocity)
         msg.angular.z = locomotionParameterData.max_angular_velocity * signnum(angular_velocity);
      else
         msg.angular.z = angular_velocity;

   
      linear_velocity = locomotionParameterData.position_gain_mimo * position_error;

      if (linear_velocity < locomotionParameterData.min_linear_velocity)
         msg.linear.x =locomotionParameterData.min_linear_velocity;
      else if (linear_velocity > locomotionParameterData.max_linear_velocity)
         msg.linear.x = locomotionParameterData.max_linear_velocity;


      /* if currently stopped, ramp up to the required velocity ... don't attempt an infinite acceleration to the required velocity */

      if (current_linear_velocity == 0) {
	   
	 for (int i=1; i < number_of_ramp_up_steps; i++) {
            msg.linear.x = (float) linear_velocity * ((float) i / (float) number_of_ramp_up_steps);
	    msg.angular.z = 0;

            if (debug) {
	       printf("Ramping up velocity\n");
               //printf("Current pose:         %5.3f %5.3f %5.3f\n", current_x, current_y, current_theta);
               //printf("Goal, heading, theta: %5.3f, %5.3f, %5.3f\n", goal_theta, goal_direction, current_theta);
               printf("Error:                %5.3f, %5.3f\n", position_error, angle_error);
               printf("velocity command:     %5.3f, %5.3f\n\n", msg.linear.x, msg.angular.z);
            }
	      
	    pub.publish(msg);              // Publish the message
	     
            rate.sleep();                  // Wait until it's time for another iteration
         }
	 current_linear_velocity = linear_velocity;
      }
   
      msg.linear.x = linear_velocity;

      if (debug) {
         printf("Going\n");
         //printf("Current pose:         %5.3f %5.3f %5.3f\n", current_x, current_y, current_theta);
         //printf("Goal, heading, theta: %5.3f, %5.3f, %5.3f\n", goal_theta, goal_direction, current_theta);
         printf("Error:                %5.3f, %5.3f\n", position_error, angle_error);
         printf("velocity command:     %5.3f, %5.3f\n\n", msg.linear.x, msg.angular.z);
      }
	    
      pub.publish(msg);              // Publish the message

      rate.sleep(); // Wait until it's time for another iteration
	    
   } while( (fabs(position_error) > locomotionParameterData.position_tolerance) && ros::ok());

   do {
	   
      /* if the robot has reached the destination             */
      /* adjust the orientation to match the goal orientation */

      /* get the current pose */

      ros::spinOnce();      // Let ROS take over to handle the callback

      /* set linear and angular velocities, taking care not to use values that exceed maximum values */
      /* or use values that are less than minimum values needed to produce a response in the robot   */
	    
      angle_error = goal_theta - current_theta;

      msg.linear.x = 0;
	       
      angular_velocity = locomotionParameterData.angle_gain_mimo * angle_error;
	       
      if (fabs(angular_velocity) < locomotionParameterData.min_angular_velocity)
          msg.angular.z = locomotionParameterData.min_angular_velocity * signnum(angular_velocity);
      else if (fabs(angular_velocity) > locomotionParameterData.max_angular_velocity)
          msg.angular.z = locomotionParameterData.max_angular_velocity * signnum(angular_velocity);
      else
          msg.angular.z = angular_velocity;
           
	    
      if (debug) {
         printf("Orienting\n");
         //printf("Current pose:         %5.3f %5.3f %5.3f\n", current_x, current_y, current_theta);
         //printf("Goal, heading, theta: %5.3f, %5.3f, %5.3f\n", goal_theta, goal_direction, current_theta);
         printf("Error:                %5.3f, %5.3f\n", position_error, angle_error);
         printf("velocity command:     %5.3f, %5.3f\n\n", msg.linear.x, msg.angular.z);
      }
	    
      pub.publish(msg);              // Publish the message

      rate.sleep(); // Wait until it's time for another iteration
	    
   } while( (fabs(angle_error) > locomotionParameterData.angle_tolerance_orienting) && ros::ok());
}





 
/*=======================================================*/
/* Utility functions                                     */ 
/*=======================================================*/

/* return the sign of a number as +/- 1 */

int signnum(float x) {
  if (x >= 0.0) return 1;
  if (x < 0.0) return -1;
}

void display_error_and_exit(char error_message[]) {
   printf("%s\n", error_message);
   printf("Hit any key to continue >>");
   getchar();
   exit(1);
}

void prompt_and_exit(int status) {
   printf("Press any key to terminate the program ... \n");
   getchar();
   exit(status);
}

void prompt_and_continue() {
   printf("Press any key to continue ... \n");
   getchar();
}


void print_message_to_file(FILE *fp, char message[]) {
   fprintf(fp,"The message is: %s\n", message);
}

