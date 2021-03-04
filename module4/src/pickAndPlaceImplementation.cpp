/*******************************************************************************************************************
 *   Example pick-and-place program for a LynxMotion AL5D robot arm
 *   --------------------------------------------------------------------
 *
 *   Implementation file
 *
 *   Audit Trail
 *   -----------
 *   28 June 2020: re-factored code to separate calculation of the joint angles using the inverse kinematics,  
 *                 from the calculation of servomotor setpoint values.  
 *                 This was done to allow the simulator to be controlled by publishing joint angles on the 
 *                 ROS /lynxmotion_al5d/joints_positions/command topic 
 *
 *******************************************************************************************************************/

#ifdef WIN32
#include "pickAndPlace.h"
#else
#include <module4/pickAndPlace.h>
#endif

/******************************************************************************

 Robot configuration data: global to allow access from implementation functions

*******************************************************************************/
    
struct robotConfigurationDataType robotConfigurationData;

 
/***********************************************************************************************************************

   Frame and vector classes to support task-level robot programming 
   ----------------------------------------------------------------

   Interface to the Frame and Vector classes and auxilliary friend functions required to implement a robot control program
   by specifying the position and orientation of objects and the robot manipulator using homogeneous transformations
   
   Author: David Vernon, Carnegie Mellon University Africa, Rwanda
   Date:   22/02/2017

***********************************************************************************************************************/
float joint_state_[6]; // Array to store the joint states

Vector::Vector(double x, double y, double z, double w) { 
   coefficient[0] = x; 
   coefficient[1] = y;
   coefficient[2] = z;
   coefficient[3] = w;
}

void Vector::setValues(double x, double y, double z, double w) { 
   coefficient[0] = x; 
   coefficient[1] = y;
   coefficient[2] = z;
   coefficient[3] = w;
}

void Vector::getValues(double &x, double &y, double &z, double &w) { 
   x = coefficient[0]; 
   y = coefficient[1];
   z = coefficient[2];
   w = coefficient[3];
}

void Vector::printVector()const {
  int i;
   
   for (i=0; i<4; i++) {
     printf("%4.1f ",coefficient[i]);
   }
   printf("\n\n");
}
 
Vector operator+(Vector &a, Vector &b) { 
   return Vector(a.coefficient[0] / a.coefficient[3] + b.coefficient[0] / b.coefficient[3], 
                 a.coefficient[1] / a.coefficient[3] + b.coefficient[1] / b.coefficient[3], 
                 a.coefficient[2] / a.coefficient[3] + b.coefficient[2] / b.coefficient[3],
                 1); //friend access
}

double dotProduct(Vector &a, Vector &b) {

   double result;

   result = a.coefficient[0] / a.coefficient[3] * b.coefficient[0] / b.coefficient[3] +
            a.coefficient[1] / a.coefficient[3] * b.coefficient[1] / b.coefficient[3] +
            a.coefficient[2] / a.coefficient[3] * b.coefficient[2] / b.coefficient[3]; //friend access;

    return result;
}

Frame::Frame() { 
   int i, j;
   
   for (i=0; i<4; i++) {
      for (j=0; j<4; j++) {
         if (i==j)
            coefficient[i][j] = 1; 
         else
            coefficient[i][j] = 0; 
      }
   }
}

void Frame::printFrame()const {
   int i, j;
   
   printf("\n");
   for (i=0; i<4; i++) {
      for (j=0; j<4; j++) {
         printf("%4.1f ",coefficient[i][j]);
      }
      printf("\n");
   }
   printf("\n");
}
 
Frame &Frame::operator*(Frame const& h) { 

   Frame result;
   double temp;
   int i, j, k;
   bool debug = false;

   if (debug) {
      printf("Operator *\n");
      this->printFrame();
      h.printFrame();
   }

   for (i=0; i<4; i++) {
      for (j=0; j<4; j++) {
         temp = 0;
         for (k=0; k<4; k++) { 
            temp = temp + (this->coefficient[i][k]) * (h.coefficient[k][j]);
         } 
         result.coefficient[i][j] = temp; 
      }
   } 

   for (i=0; i<4; i++) {
      for (j=0; j<4; j++) {
         this->coefficient[i][j] = result.coefficient[i][j]; 
      }
   } 

   if (debug) this->printFrame();

   return *this;
}

Frame &Frame::operator=(Frame const& h) { 
   int i, j;
   bool debug = false;

   if (debug) {
      printf("Operator =\n");
      h.printFrame();
   }

   for (i=0; i<4; i++) {
      for (j=0; j<4; j++) {
         this->coefficient[i][j] = h.coefficient[i][j];
      }
   }

  if (debug) this->printFrame();

   return *this;
}

/* translation by vector (x, y, z) */

Frame trans(float x, float y, float z) {
      
   Frame result;

   int i, j;
   bool debug = false;

   if (debug) {
      printf("trans %f %f %f\n",x, y, z);
   }

   for (i=0; i<4; i++) {
      for (j=0; j<4; j++) {
         result.coefficient[i][j] = 0;
      }
   }

   result.coefficient[0][0] = 1;
   result.coefficient[1][1] = 1;
   result.coefficient[2][2] = 1;
   result.coefficient[3][3] = 1;

   result.coefficient[0][3] = x;
   result.coefficient[1][3] = y;
   result.coefficient[2][3] = z;

  if (debug) result.printFrame();

  return result;
}


/* rotation about x axis by theta degrees */

Frame rotx(float theta) {
         
   Frame result;
   double thetaRadians;
   int i, j;
   bool debug = false;

   if (debug) {
      printf("rotx %f\n",theta);
   }

   /* convert theta to radians */

   thetaRadians = (3.14159 * theta) / 180.0;

   for (i=0; i<4; i++) {
      for (j=0; j<4; j++) {
         result.coefficient[i][j] = 0;
      }
   }

   result.coefficient[0][0] = 1;
   result.coefficient[0][1] = 0;
   result.coefficient[0][2] = 0;

   result.coefficient[1][0] = 0;
   result.coefficient[1][1] = cos(thetaRadians);
   result.coefficient[1][2] = -sin(thetaRadians);;

   result.coefficient[2][0] = 0;
   result.coefficient[2][1] = sin(thetaRadians);
   result.coefficient[2][2] = cos(thetaRadians);

   result.coefficient[3][3] = 1;

  if (debug) result.printFrame();

  return result;
}


/* rotation about y axis by theta degrees */

Frame roty(float theta) {
         
   Frame result;
   double thetaRadians;
   int i, j;
   bool debug = false;

   if (debug) {
      printf("roty %f\n",theta);
   }

   /* convert theta to radians */

   thetaRadians = (3.14159 * theta) / 180.0;

   for (i=0; i<4; i++) {
      for (j=0; j<4; j++) {
         result.coefficient[i][j] = 0;
      }
   }

   result.coefficient[0][0] = cos(thetaRadians);
   result.coefficient[0][1] = 0;
   result.coefficient[0][2] = sin(thetaRadians);

   result.coefficient[1][0] = 0;
   result.coefficient[1][1] = 1;
   result.coefficient[1][2] = 0;

   result.coefficient[2][0] = -sin(thetaRadians);
   result.coefficient[2][1] = 0;
   result.coefficient[2][2] = cos(thetaRadians);

   result.coefficient[3][3] = 1;

  if (debug) result.printFrame();

  return result;
}


Frame inv(Frame h) { 

   Vector u, v;
   Frame result;

   int i, j;
   //double dp;
   bool debug = false;

   if (debug) {
      printf("inv \n");
      h.printFrame();
   }

   for (i=0; i<3; i++) {
      for (j=0; j<3; j++) {
         result.coefficient[j][i] = h.coefficient[i][j]; // transpose the rotation part
      }
   }

   for (j=0; j<4; j++) {
      result.coefficient[3][j] = h.coefficient[3][j];    // copy the scaling part
   }

   
   v.setValues(h.coefficient[0][3], h.coefficient[1][3], h.coefficient[2][3], h.coefficient[3][3]); // p

   u.setValues(h.coefficient[0][0], h.coefficient[1][0], h.coefficient[2][0], 1); // n
   result.coefficient[0][3] = -dotProduct(u,v);
 
   u.setValues(h.coefficient[0][1], h.coefficient[1][1], h.coefficient[2][1], 1); // o 
   result.coefficient[1][3] = -dotProduct(u,v);
 
   u.setValues(h.coefficient[0][2], h.coefficient[1][2], h.coefficient[2][2], 1); // a
   result.coefficient[2][3] = -dotProduct(u,v);
 
   result.coefficient[3][3] = 1;

   if (debug) result.printFrame();

   return result;
}


/* rotation about z axis by theta degrees */

Frame rotz(float theta) {
         
   Frame result;
   double thetaRadians;
   int i, j;
   bool debug = false;

   if (debug) {
      printf("rotz %f\n",theta);
   }

   /* convert theta to radians */

   thetaRadians = (3.14159 * theta) / 180.0;

   for (i=0; i<4; i++) {
      for (j=0; j<4; j++) {
         result.coefficient[i][j] = 0;
      }
   }

   result.coefficient[0][0] = cos(thetaRadians);
   result.coefficient[0][1] = -sin(thetaRadians);
   result.coefficient[0][2] = 0;

   result.coefficient[1][0] = sin(thetaRadians);
   result.coefficient[1][1] = cos(thetaRadians);
   result.coefficient[1][2] = 0;

   result.coefficient[2][0] = 0;
   result.coefficient[2][1] = 0;
   result.coefficient[2][2] = 1;

   result.coefficient[3][3] = 1;

  if (debug) result.printFrame();

  return result;
}


/* move(Frame t5)                                                                                   */
/*                                                                                                  */
/* 1. Extract the pose parameters from the T5 frame and servo the robot to the associated pose      */
/* 2. Call computeJointAngles() to compute the joint angles in radians using the inverse kinematics */
/* 3. Call setJointAngles() to servo to robot arm to the required joint angles                      */
/*                                                                                                  */
/*                                                                                                  */
/* Fixed bug in computation of pitch parameter                                                      */
/* David Vernon 8/6/2018                                                                            */
/*                                                                                                  */
/* Refactored code to use computeJointAngles() and setJointAngles()                                 */
/* David Vernon 28/6//2020                                                                          */


bool move(Frame T5) {

   bool debug = false;

   double ax, ay, az; // components of approach vector
   double ox, oy, oz; // components of orientation vector
   double px, py, pz; // components of position vector

   double tolerance = 0.001;
   double r;
   double pitch;
   double roll;

   double jointAngles[6]; // six angles: five for the pose (joints 1 to 5), all in radians, and one for the gripper in metres 

   /* check to see if the pose is achievable:                                                                */
   /* the approach vector must be aligned with (i.e. in same plane as) the vector from the base to the wrist */
   /* (unless the approach vector is directed vertically up or vertically down                               */

   // T5.printFrame();

   ox = T5.coefficient[0][1];
   oy = T5.coefficient[1][1];
   oz = T5.coefficient[2][1];

   ax = T5.coefficient[0][2];
   ay = T5.coefficient[1][2];
   az = T5.coefficient[2][2];

   px = T5.coefficient[0][3];
   py = T5.coefficient[1][3];
   pz = T5.coefficient[2][3];


   if (debug) {
         T5.printFrame();
         // printf("move(): px,py %4.1f %4.1f  ax,ay %4.1f %4.1f angles  %4.1f %4.1f \n", px, py, ax, ay, 180*atan2(py, px)/3.14159, 180*atan2(ay, ax)/3.14159);
   }

   if (( ax > -tolerance && ax < tolerance && ay > -tolerance && ay < tolerance)  // vertical approach vector
       ||
       (abs(atan2(ay, ax) - atan2(py, px)) < tolerance)) {  

      /* achievable pose                           */
      /* extract the pitch and roll angles from T5 */

      /* pitch */
      r = sqrt(ax*ax + ay*ay);

      if (r < tolerance) {      // vertical orientation vector
         pitch = 0;
         if (az < 0) {          // get direction right
            pitch = -180;
         }
      }
      else {
         pitch = -degrees(atan2(r, az)); // DV change to negative angle since pitch range is -180 to 0     8/6/2018
      }
       
      /* roll */
      roll =   degrees(atan2(ox, oy));

      if (debug) {
         printf("move(): x, y, z, pitch, roll: %4.1f %4.1f %4.1f %4.1f %4.1f \n", px, py, pz, pitch, roll);
      }

      //gotoPose((float) px, (float) py, (float) pz, (float) pitch, (float) roll);

	  computeJointAngles(px, py, pz, pitch, roll, jointAngles);

	  setJointAngles(jointAngles);

      return true;
   }
   else {

      printf("move(): pose not achievable: approach vector and arm are not aligned \n");
      printf("        atan2(py, px) %f; atan2(ay, ax)  %f\n",180*atan2(py, px)/3.14159, 180*atan2(ay, ax)/3.14159);

      return false; // approach vector and arm are not aligned ... pose is not achievable
   }
}


/*********************************************************************/
/*                                                                   */
/* Inverse kinematics for LynxMotion AL5D robot manipulator          */
/*                                                                   */
/*********************************************************************/

double degrees(double radians)
{
    double degrees = radians * (double) 180.0 / (double) M_PI; // David Vernon ... cast to float
    return degrees;
}

double radians(double degrees)
{
    double radians = degrees / ((double) 180.0 / (double) M_PI); // David Vernon ... cast to float
    return radians;
}
 

/* computeJointAngles()
   
   Transform from wrist pose (x, y, z, pitch, and roll in degrees) to joint angles using the inverse kinematics

   If resulting arm position is physically unreachable, return false.
   Otherwise, return the corresponding joint angles in radians.

   This code is a modified version of the code written by Oleg Mazurov and Eric Goldsmith (see header)
   The revisions include the removal of the dependency on the end-effector (gripper) distance
   so that it computes the inverse kinematics for T5, the position and orientation of the wrist
   Also, the roll angle was added.  Minor changes have been made to the variable names; more are necessary
   to make it readable and consistent with Denavit-Hartenberg notation 

   David Vernon
   3 March 2017
   
   Audit trail
   -----------

   7 June 2018: modified to use robot configuration data reflecting the calibration of individual robots DV
   8 June 2018: fixed a number of bugs in the inverse kinematic solution
   28 June 2020: refactored code into computeJointAngles() and computeJointPositions(), removing getPose()
*/

bool computeJointAngles(double x, double y, double z, double pitch_angle_d, double roll_angle_d, double joint_angles[]) {
 
    bool debug = false; 
  
    if (debug) printf("computeJointAngles(): x %4.1f, y %4.1f, z %4.1f, pitch %4.1f, roll %4.1f\n", x, y, z, pitch_angle_d, roll_angle_d);

    double hum_sq;
    double uln_sq;
    double wri_roll_angle_d;

    hum_sq = A3 * A3;
    uln_sq = A4 * A4;


    //grip angle in radians for use in calculations
    double pitch_angle_r = radians(pitch_angle_d);  // David Vernon uncommented 
    double roll_angle_r = radians(roll_angle_d);    // David Vernon uncommented


    // Base angle and radial distance from x,y coordinates
	// ---------------------------------------------------
    double bas_angle_r = atan2(x, y);
    double bas_angle_d = degrees(bas_angle_r);

    double rdist = sqrt((x * x) + (y * y));

    // rdist is y coordinate for the arm
    y = (float) rdist; //BAD PRACTICE IN ORIGINAL CODE: OVERWRITING A PARAMETER! Noted by David Vernon

    // Wrist position
    double wrist_z = z - D1;
    double wrist_y = y;

    // Shoulder to wrist distance (AKA sw)
    double s_w = (wrist_z * wrist_z) + (wrist_y * wrist_y);
    double s_w_sqrt = sqrt(s_w);

    // s_w angle to ground
    double a1 = atan2(wrist_z, wrist_y);  // David Vernon ... alpha in notes

    // s_w angle to A3
    double a2 = (float) acos(((hum_sq - uln_sq) + s_w) / (2 * A3 * s_w_sqrt));  // David Vernon ... cast to float ... this is angle beta in notes

    // Shoulder angle
	//---------------
    double shl_angle_r = a1 + a2; // David Vernon ... theta_2 = alpha + beta in notes
    // If result is NAN or Infinity, the desired arm position is not possible

    if (std::isnan(shl_angle_r) || std::isinf(shl_angle_r))
        return false;             // David Vernon ... not a valid pose 

    double shl_angle_d = degrees(shl_angle_r);

    double a1_d = degrees(a1);    // David Vernon ... alpha in notes
    double a2_d = degrees(a2);    // David Vernon ... beta  in notes


	// Elbow angle
	//------------
	/* Changed original solution to use the negative value of the original angle                                                           */
	/* This is necessary because the shoulder angle is computed using a1 + a2 (theta_2 = alpha + beta in notes)                            */
	/* The original value is correct for the a1 - a2 (theta_2 = alpha - beta in notes)                                                     */
	/* The final servo value was correct because a negative value was used in that calculation                                             */
	/* David Vernon                                                                                                                        */
	/* 28 June 2020                                                                                                                        */           

    //double elb_angle_r = acos((hum_sq + uln_sq - s_w) / (2 * A3 * A4)); // David Vernon ... cast to float
    double elb_angle_r = acos((s_w - hum_sq - uln_sq) / (2 * A3 * A4)); // Innocent ... calculating the supplementary angle
    
    // If result is NAN or Infinity, the desired arm position is not possible
    if (std::isnan(elb_angle_r) || std::isinf(elb_angle_r)) 
        return false;            // David Vernon ... not a valid pose 

    elb_angle_r = -elb_angle_r;  // David Vernon ... use negative value when shoulder angle = a1 + a2 (theta_2 = alpha + beta in notes)
    //double elb_angle_d = degrees(elb_angle_r);

    //double elb_angle_dn = -((double)180.0 - elb_angle_d);  // Commented in David Vernon's code.

    // double elb_angle_dn = -((double)180.0 + elb_angle_d);    // David Vernon ... adjusted for negative value 

    double elb_angle_dn = degrees(elb_angle_r); // Innocent ... converting the supplementary angle to degrees


    // Wrist angles
    //-------------
    /* Changed original solution by adding 90 degrees to rotation about y axis (pitch) and about z axis (roll) to ensure that              */
    /* the z axis of the wrist (i.e. the approach vector) is aligned with the z axis of the frame in base of the robot, i.e. frame Z       */  
    /* and the y axis of the wrist (i.e. the orientation vector) is aligned with the y axis of frame Z                                     */
    /* Thus, if T5 is pure translation the the wrist is pointing vertically upward and the plane of the gripper is aligned with the y axis */ 
    /* David Vernon                                                                                                                        */
	/* 24 April 2018                                                                                                                       */
                               
	/* Note that this is necessary because the kinematic specification given in M. A. Qassem, I. Abuhadrous, and H. Elaydi,                */
	/* “Modeling and Simulation of 5 DOF educational robot arm”, 2nd International Conference on Advanced Computer Control (ICACC), 2010.  */
	/* has the T5 axes aligned in different directions to the base frame                                                                   */
    /* David Vernon                                                                                                                        */
	/* 25 June 2020                                                                                                                        */         

    //float wri_pitch_angle_d = (pitch_angle_d - elb_angle_dn) - shl_angle_d; // original code
    double wri_pitch_angle_d = (pitch_angle_d - elb_angle_dn) - shl_angle_d + 90; // David Vernon ... added 90

    // if (((int)pitch_angle_d == -90) || ((int)pitch_angle_d == 90)) // original code
    if (((int) pitch_angle_d == 0))  // directed vertically up
    {

        /* special case: we can adjust the required roll to compensate for the base rotation */

        // wri_roll_angle_d = roll_angle_d - bas_angle_d;   // original code
        wri_roll_angle_d = roll_angle_d + bas_angle_d + 90; // gripper orientation aligned with y axis

    }
    else if (((int) pitch_angle_d == -180) || ((int) pitch_angle_d == 180))  // directed vertically down
    {

        /* special case: we can adjust the required roll to compensate for the base rotation */

        // wri_roll_angle_d = roll_angle_d - bas_angle_d;   // original code
        wri_roll_angle_d = roll_angle_d - bas_angle_d + 90; // gripper orientation aligned with y axis
    }
    else
    {
        // should really throw an exception here because this isn't correct
        wri_roll_angle_d = roll_angle_d; // original code
        wri_roll_angle_d = roll_angle_d + 90;
    }

	joint_angles[0] = bas_angle_r;
	joint_angles[1] = shl_angle_r;
	joint_angles[2] = elb_angle_r;
	joint_angles[3] = radians(wri_pitch_angle_d);
	joint_angles[4] = radians(wri_roll_angle_d);

    if (debug) {
      // printf("Joint 1 (degrees): %4.2f \n",degrees(bas_angle_r));
      // printf("Joint 2 (degrees): %4.2f \n",shl_angle_d);
      // printf("Joint 3 (degrees): %4.2f \n",elb_angle_d);
      // printf("Joint 4 (degrees): %4.2f \n",wri_pitch_angle_d);
      // printf("Joint 5 (degrees): %4.2f \n",wri_roll_angle_d);
	   printf("\n");
       printf("Joint 1 (radians): %4.2f \n",joint_angles[0]);
       printf("Joint 2 (radians): %4.2f \n",joint_angles[1]);
       printf("Joint 3 (radians): %4.2f \n",joint_angles[2]);
       printf("Joint 4 (radians): %4.2f \n",joint_angles[3]);
       printf("Joint 5 (radians): %4.2f \n",joint_angles[4]);
       printf("\n");
    }

    return true; // David Vernon ... valid pose
}


/* setJointAngles()
   
   Servo the robot by setting the joint angles.

   If using ROS, this is effected by publishing the joint angles on the /lynxmotion_al5d/joints_positions/command topic 

   If not using ROS but controlling the robot from Windows, this is effected by 
   transforming from joint angles to servo position values and writing the servo position values to the COM port
 
   David Vernon
   28 June 2020

*/

bool setJointAngles(double joint_angles[]) {

	bool debug = false; 
    int positions[6];       
    bool valid_pose;   

    if (debug) printf("setJointAngles(): angles %4.2f %4.2f %4.2f %4.2f %4.2f\n", joint_angles[0], joint_angles[1],joint_angles[2],joint_angles[3],joint_angles[4]);

#ifdef ROS

	/* ROS version: PUT ROS PUBLISHER CODE HERE */

	/* copy joint angles to the globally-accessible structure so that they can be used when constucting the topic message in the grasp() function */

	for (int i=0; i<5; i++) {
		robotConfigurationData.current_joint_value[i] = joint_angles[i];
	}

	/* now construct the topic message with all six joint values: five for the joint angles and one for the gripper */
	/* use the values in robotConfigurationData.current_joint_value[]                                               */

		ros::NodeHandle n;                        // Become a node
        ros::Publisher pub_ = n.advertise<std_msgs::Float64MultiArray>("/lynxmotion_al5d/joints_positions/command", 1000);
	ros::Rate rate(2); // Loop at 2Hz until the node is shut down

	/* Create a subscriber object */
	ros :: Subscriber sub = n.subscribe("/lynxmotion_al5d/joint_states", 1000, &jointStates);

	// MultiArray message
	std_msgs::Float64MultiArray msg;

	std::vector<float> joints_values;

	
	/* publish the message on the topic */

	joints_values = {robotConfigurationData.current_joint_value[0], robotConfigurationData.current_joint_value[1], robotConfigurationData.current_joint_value[2], robotConfigurationData.current_joint_value[3], robotConfigurationData.current_joint_value[4], robotConfigurationData.current_joint_value[5]};

	   
	   msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
	   msg.layout.dim[0].size = joints_values.size();
	   msg.layout.dim[0].stride = 1;
	   msg.layout.dim[0].label = "joints";

	   msg.data.clear();
	   msg.data.insert(msg.data.end(), joints_values.begin(), joints_values.end());


	   /* Waiting for the publisher to be ready to publish the message */
	   while(pub_.getNumSubscribers()<1)
	     {
	       // waiting for a connection to publisher
	     }


	   
	   pub_.publish(msg); // publishing the message





#else

	/* Windows version */

	valid_pose = computeServoPositions(joint_angles, positions);

    if (valid_pose) {
        
       if (debug) printf("setJointAngles(): servo positions %d %d %d %d %d \n", positions[0], positions[1],  positions[2], positions[3], positions[4]);

       executeCommand(robotConfigurationData.channel, positions, robotConfigurationData.speed, 5);

       return 1;
    }
    else {
       printf("setJointAngles() error: not a valid pose for this robot\n");
       return 0;
    }

#endif 
}

/* computeServoPositions()
   
   Transform from joint angles to servo position values
 
   David Vernon
   28 June 2020

*/

bool computeServoPositions(double joint_angles[], int positions[]) {

    int homeOffset[6];  

    bool debug = false; 
    int i;

    if (debug) 
	   printf("computeServoPositions(): joint angles %4.2f %4.2f %4.2f %4.2f %4.2f\n", joint_angles[0], joint_angles[1],joint_angles[2],joint_angles[3],joint_angles[4]);

    double bas_pos;
    double shl_pos;
    double elb_pos;
    double wri_pitch_pos;
    double wri_roll_pos;

    /* Set the servo angles corresponding to joint angles for the home position */

    for (i=0; i<5; i++) {
       homeOffset[i] = (int) ((float)robotConfigurationData.home[i] / robotConfigurationData.degree[i]);
    }


    // Calculate servo positions

    bas_pos       = degrees(joint_angles[0])                  + homeOffset[0];
    shl_pos       = (degrees(joint_angles[1]) - (float) 90.0) + homeOffset[1];  // joint angle after compensating for the -90 degrees for the home configuration
    elb_pos       = (-degrees(joint_angles[2]) - (float) 90.0) + homeOffset[2];  // joint angle after compensating for the -90 degrees for the home configuration
    wri_pitch_pos = degrees(joint_angles[3])                  + homeOffset[3];
    if (robotConfigurationData.lightweightWrist == true) {
       wri_roll_pos  = - degrees(joint_angles[4])             + homeOffset[4];            
    }
    else {
       wri_roll_pos  =   degrees(joint_angles[4])             + homeOffset[4];  // roll servo rotates in reverse with the medium duty wrist attachment
    }

    //if (wri_roll_pos > 90) wri_roll_pos = wri_roll_pos - 180; // NEW CHECK
    //if (wri_roll_pos < 90) wri_roll_pos = wri_roll_pos + 180; // NEW CHECK
	
    positions[0] = (int)(bas_pos       * robotConfigurationData.degree[0]);
    positions[1] = (int)(shl_pos       * robotConfigurationData.degree[1]);
    positions[2] = (int)(elb_pos       * robotConfigurationData.degree[2]);
    positions[3] = (int)(wri_pitch_pos * robotConfigurationData.degree[3]);
    positions[4] = (int)(wri_roll_pos  * robotConfigurationData.degree[4]);

    if (debug) {
	   printf("computeServoPositions(): servo positions ");
	   for (i=0; i<=4; i++) {   
		  printf("%4d ",positions[i]);
	   }
       printf("\n");
    }

    return true;  
}

void grasp(int d) // d is distance between finger tips:  0 <= d <= 30 mm
{

  /* The gripper is controlled by servo 6
   *
   * The calibration data are stored in element 5 of the home[], degree[], and channel[] arrays
   * in the global robotConfigurationData structure
   *
   * The gripper is approximately 30mm apart (i.e. fully open) when at the servo setpoint given by home[5]
   *
   * A (closing) distance of 1 mm is given by the PW defined by degree[5] array
   * and, a gripper opening distance of d mm is given by home[5] + (30-d) * degree[5]
   */

   bool debug = true;

   int pw;

#ifdef ROS

	/* ROS version: PUT ROS PUBLISHER CODE HERE */

	/* copy the gripper distance angles to the globally-accessible structure so that it can be used when constucting the topic message in the setJointAngles() function */

   robotConfigurationData.current_joint_value[5] = ((double) d) / 1000;
   
	

	/* now construct the topic message with all six joint values: five for the joint angles and one for the gripper */
	/* use the values in robotConfigurationData.current_joint_value[]                                               */

		ros::NodeHandle n;                        // Become a node
        ros::Publisher pub = n.advertise<std_msgs::Float64MultiArray>("/lynxmotion_al5d/joints_positions/command", 1000);
	ros::Rate rate(2); // Loop at 2Hz until the node is shut down

	/* Create a subscriber object */
	ros :: Subscriber sub = n.subscribe("/lynxmotion_al5d/joint_states", 1000, jointStates);

	// MultiArray message
	std_msgs::Float64MultiArray msg;

	std::vector<float> joints_values;



	/* publish the message on the topic */

	joints_values = {robotConfigurationData.current_joint_value[0], robotConfigurationData.current_joint_value[1], robotConfigurationData.current_joint_value[2], robotConfigurationData.current_joint_value[3], robotConfigurationData.current_joint_value[4], robotConfigurationData.current_joint_value[5]};
	

	   msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
	   msg.layout.dim[0].size = joints_values.size();
	   msg.layout.dim[0].stride = 1;
	   msg.layout.dim[0].label = "joints";

	   msg.data.clear();
	   msg.data.insert(msg.data.end(), joints_values.begin(), joints_values.end());

	   /* Waiting for the publisher to be ready to publish the message */
	   while(pub.getNumSubscribers()<1)
	     {
	       // Waiting for connection to publisher
	     }


	   pub.publish(msg); // publishing the message

	   



#else

   /* Windows version */

   pw = robotConfigurationData.home[5] + (int) (float (30-d) * robotConfigurationData.degree[5]);

   if (debug) {
      printf("grasp: %d\n",d);
     // printf("grasp: d %d  PW %d\n", d, pw);
   }
   executeCommand(robotConfigurationData.channel[5], pw, robotConfigurationData.speed * 2);   

#endif

}


int pose_within_working_env(float x, float y, float z)
{
    if((int)x <= MAX_X && (int) x > MIN_X && (int)y <= MAX_Y && (int) y > MIN_Y && (int)z <= MAX_Z && (int) z > MIN_Z ) return 1;
    else return 0;
}

#ifdef ROS
void jointStates(const sensor_msgs::JointState::ConstPtr& msg)
{
  
    /* The messages are echoed starting from the gripper then the joints */
    joint_state_[5] = msg->position[0];
    joint_state_[0] = msg->position[1];
    joint_state_[1] = msg->position[2];
    joint_state_[2] = msg->position[3];
    joint_state_[3] = msg->position[4];
    joint_state_[4] = msg->position[5];
   
}
#endif




#ifdef COMPILE_LEGACY_VERSION

/* Arm positioning routine using inverse kinematics
 
   Z is height, Y is distance from base center out, X is side to side. Y, Z can only be positive.
   Input dimensions are for the WRIST 
   If resulting arm position is physically unreachable, return false.

   This code is an extended version of the code written by Oleg mazurov and Eric Goldsmith (see header)
   The revisions include the removal of the dependency on the end-effector (gripper) distance
   so that it computes the inverse kinematics for T5, the position and orientation of the wrist
   Also, the roll angle was added.  Minor changes have been made to the variable names; more are necessary
   to make it readable and consistent with Denavit-Hartenberg notation 

   David Vernon
   3 March 2017
   
   Audit trail
   -----------

   7 June 2018: modified to use robot configuration data reflecting the calibration of individual robots DV
   8 June 2018: fixed a number of bugs in the inverse kinematic solution
*/

bool getJointPositions(float x, float y, float z, float pitch_angle_d, float roll_angle_d, int positions[]) {
 
    int homeOffset[6];  

    bool debug = true; 
    int i;

    if (false && debug) printf("getJointPositions(): %4.1f %4.1f %4.1f %4.1f %4.1f\n", x, y, z, pitch_angle_d, roll_angle_d);

    double hum_sq;
    double uln_sq;
    double wri_roll_angle_d;
    double bas_pos;
    double shl_pos;
    double elb_pos;
    double wri_pitch_pos;
    double wri_roll_pos;

    hum_sq = A3 * A3;
    uln_sq = A4 * A4;

    /* Set the servo angles corresponding to joint angles for the home position */

    for (i=0; i<5; i++) {
       homeOffset[i] = (int) ((float)robotConfigurationData.home[i] / robotConfigurationData.degree[i]);
    }

    //grip angle in radians for use in calculations
    double pitch_angle_r = radians(pitch_angle_d);  // David Vernon uncommented 
    double roll_angle_r = radians(roll_angle_d);    // David Vernon uncommented

    // Base angle and radial distance from x,y coordinates
    double bas_angle_r = atan2(x, y);
    double bas_angle_d = degrees(bas_angle_r);

    double rdist = sqrt((x * x) + (y * y));

    // rdist is y coordinate for the arm
    y = (float) rdist; //DV BAD PRACTICE IN ORIGINAL CODE: OVERWRITING A PARAMETER!

    // Wrist position
    double wrist_z = z - D1;
    double wrist_y = y;

    // Shoulder to wrist distance (AKA sw)
    double s_w = (wrist_z * wrist_z) + (wrist_y * wrist_y);
    double s_w_sqrt = sqrt(s_w);

    // s_w angle to ground
    double a1 = atan2(wrist_z, wrist_y);  // David Vernon ... alpha in notes

    // s_w angle to A3
    double a2 = (float) acos(((hum_sq - uln_sq) + s_w) / (2 * A3 * s_w_sqrt));  // David Vernon ... cast to float ... this is angle beta in notes

    // Shoulder angle
    double shl_angle_r = a1 + a2; // David Vernon ... theta_2 = alpha + beta in notes
    // If result is NAN or Infinity, the desired arm position is not possible

    if (std::isnan(shl_angle_r) || std::isinf(shl_angle_r))
        return false;             // David Vernon ... not a valid pose 

    double shl_angle_d = degrees(shl_angle_r);

    double a1_d = degrees(a1);    // David Vernon ... alpha in notes
    double a2_d = degrees(a2);    // David Vernon ... beta  in notes


	// Elbow angle
	//------------
	/* Changed original solution to use the negative value of the original angle                                                           */
	/* This is necessary because the shoulder angle is computed using a1 + a2 (theta_2 = alpha + beta in notes)                            */
	/* The original value is correct for the a1 - a2 (theta_2 = alpha - beta in notes)                                                     */
	/* The final servo value was correct because a negative value is used in that calculation                                              */
	/* David Vernon                                                                                                                        */
	/* 28 June 2020                                                                                                                        */           

    double elb_angle_r = acos((hum_sq + uln_sq - s_w) / (2 * A3 * A4)); // David Vernon ... cast to float
    // If result is NAN or Infinity, the desired arm position is not possible
    if (std::isnan(elb_angle_r) || std::isinf(elb_angle_r)) 
        return false;            // David Vernon ... not a valid pose 

	elb_angle_r = -elb_angle_r;  // David Vernon ... use negative value when shoulder angle = a1 + a2 (theta_2 = alpha + beta in notes) 

    double elb_angle_d = degrees(elb_angle_r);
    //double elb_angle_dn = -((double)180.0 - elb_angle_d);  
    double elb_angle_dn = -((double)180.0 + elb_angle_d);    // David Vernon ... adjusted for negative value 


    // Wrist angles
    //-------------
    /* Changed original solution by adding 90 degrees to rotation about y axis (pitch) and about z axis (roll) to ensure that              */
    /* the z axis of the wrist (i.e. the approach vector) is aligned with the z axis of the frame in base of the robot, i.e. frame Z       */  
    /* and the y axis of the wrist (i.e. the orientation vector) is aligned with the y axis of frame Z                                     */
    /* Thus, if T5 is pure translation the the wrist is pointing vertically upward and the plane of the gripper is aligned with the y axis */ 
    /* David Vernon                                                                                                                        */
	/* 24 April 2018                                                                                                                       */
                               
	/* Note that this is necessary because the kinematic specification given in M. A. Qassem, I. Abuhadrous, and H. Elaydi,                */
	/* “Modeling and Simulation of 5 DOF educational robot arm”, 2nd International Conference on Advanced Computer Control (ICACC), 2010.  */
	/* has the T5 axes aligned in different directions to the base frame                                                                   */
    /* David Vernon                                                                                                                        */
	/* 25 June 2020                                                                                                                        */         

    //float wri_pitch_angle_d = (pitch_angle_d - elb_angle_dn) - shl_angle_d; // original code
    double wri_pitch_angle_d = (pitch_angle_d - elb_angle_dn) - shl_angle_d + 90; // David Vernon ... added 90

    // if (((int)pitch_angle_d == -90) || ((int)pitch_angle_d == 90)) // original code
    if (((int) pitch_angle_d == 0))  // directed vertically up
    {

        /* special case: we can adjust the required roll to compensate for the base rotation */

        // wri_roll_angle_d = roll_angle_d - bas_angle_d;   // original code
        wri_roll_angle_d = roll_angle_d + bas_angle_d + 90; // gripper orientation aligned with y axis

    }
    else if (((int) pitch_angle_d == -180) || ((int) pitch_angle_d == 180))  // directed vertically down
    {

        /* special case: we can adjust the required roll to compensate for the base rotation */

        // wri_roll_angle_d = roll_angle_d - bas_angle_d;   // original code
        wri_roll_angle_d = roll_angle_d - bas_angle_d + 90; // gripper orientation aligned with y axis
    }
    else
    {
        // should really throw an exception here because this isn't correct
        wri_roll_angle_d = roll_angle_d; // original code
        wri_roll_angle_d = roll_angle_d + 90;
    }


    // Calculate servo positions

    bas_pos       = bas_angle_d                  + homeOffset[0];
    shl_pos       = (shl_angle_d - (float) 90.0) + homeOffset[1];  // joint angle after compensating for the -90 degrees for the home configuration
    elb_pos       = (elb_angle_d + (float) 90.0) + homeOffset[2];  // joint angle after compensating for the -90 degrees for the home configuration
    wri_pitch_pos = wri_pitch_angle_d            + homeOffset[3];
    if (robotConfigurationData.lightweightWrist == true) {
       wri_roll_pos  = - wri_roll_angle_d        + homeOffset[4];            
    }
    else {
       wri_roll_pos  =   wri_roll_angle_d        + homeOffset[4];  // roll servo rotates in reverse with the medium duty wrist attachment
    }

    //if (wri_roll_pos > 90) wri_roll_pos = wri_roll_pos - 180; // NEW CHECK
    //if (wri_roll_pos < 90) wri_roll_pos = wri_roll_pos + 180; // NEW CHECK
	
    positions[0] = (int)(bas_pos       * robotConfigurationData.degree[0]);
    positions[1] = (int)(shl_pos       * robotConfigurationData.degree[1]);
    positions[2] = (int)(elb_pos       * robotConfigurationData.degree[2]);
    positions[3] = (int)(wri_pitch_pos * robotConfigurationData.degree[3]);
    positions[4] = (int)(wri_roll_pos  * robotConfigurationData.degree[4]);


    if (debug) {
       printf("x:         %5.1f \n",x);
       printf("y:         %5.1f \n",y); // DV THIS HAS BEEN OVERWRITTEN IN ORIGINAL CODE
       printf("z:         %5.1f \n",z);
       printf("pitch:     %5.1f \n",pitch_angle_d);
       printf("roll:      %5.1f \n",roll_angle_d);
       printf("Base Pos:  %5.1f \n",bas_pos);
       printf("Shld Pos:  %5.1f \n",shl_pos);
       printf("Elbw Pos:  %5.1f \n",elb_pos);
       printf("Pitch Pos: %5.1f \n",wri_pitch_pos);
       printf("Roll Pos:  %5.1f \n",wri_roll_pos);
       printf("bas angle: %5.1f \n",degrees(bas_angle_r));
       printf("shl angle: %5.1f \n",shl_angle_d);
       printf("elb angle: %5.1f \n",elb_angle_d);
       printf("Pitch d:   %5.1f \n",wri_pitch_angle_d);
       printf("Roll  d:   %5.1f \n",wri_roll_angle_d);
	   printf("positions: ");
	   for (i=0; i<=4; i++) {   
		  printf("%d ",positions[i]);
	   }
       printf("\n\n");
    }

    return true; // David Vernon ... valid pose
}


int gotoPose(float x, float y, float z, float pitch, float roll)
{
    bool debug = false; 
    int pos[6];       
    bool valid_pose;   

    if (!pose_within_working_env(x, y, z)) {
       printf("gotoPose() error: %4.1f %4.1f %4.1f not in working envelope\n", x, y, z);
       //return 0;
    }

    if (debug) printf("gotoPose(): %4.1f %4.1f %4.1f %4.1f %4.1f\n", x, y, z, pitch, roll);

    valid_pose = getJointPositions(x, y, z, pitch, roll, pos);

    if (valid_pose) {
        
       if (debug) printf("gotoPose(): %d %d %d %d %d \n", pos[0], pos[1],  pos[2], pos[3], pos[4]);

        executeCommand(robotConfigurationData.channel, pos, robotConfigurationData.speed, 5);

        return 1;
    }
    else {
       printf("gotoPose() error: not a valid pose for this robot\n");
       return 0;
    }
}

#endif

/*=======================================================*/
/* Robot configuration function                          */ 
/*=======================================================*/

void readRobotConfigurationData(char filename[]) {
      
   bool debug = false;
   int i; 
   int j;
   int k;

   keyword keylist[NUMBER_OF_KEYS] = {
      "com",
      "baud",
      "speed",
      "channel",
      "home",
      "degree",
      "effector",
      "wrist",
      "current"
   };

   keyword key;                  // the key string when reading parameters
   keyword value;                // the value string, used for the WRIST key

   char input_string[STRING_LENGTH];
   FILE *fp_config;       

   if ((fp_config = fopen(filename,"r")) == 0) {
	   printf("Error can't open robot configuration file %s\n",filename);
      prompt_and_exit(0);
   }

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
            case 0:  sscanf(input_string, " %s %s", key, robotConfigurationData.com);     // com  
                     break;
            case 1:  sscanf(input_string, " %s %d", key, &(robotConfigurationData.baud));  // baud
                     break;
            case 2:  sscanf(input_string, " %s %d", key, &(robotConfigurationData.speed)); // speed
                     break;
            case 3:  sscanf(input_string, " %s %d %d %d %d %d %d", key,                    // channel
                                                                   &(robotConfigurationData.channel[0]), 
                                                                   &(robotConfigurationData.channel[1]), 
                                                                   &(robotConfigurationData.channel[2]), 
                                                                   &(robotConfigurationData.channel[3]), 
                                                                   &(robotConfigurationData.channel[4]), 
                                                                   &(robotConfigurationData.channel[5]));
                     break;
            case 4:  sscanf(input_string, " %s %d %d %d %d %d %d", key,                    // home
                                                                   &(robotConfigurationData.home[0]), 
                                                                   &(robotConfigurationData.home[1]), 
                                                                   &(robotConfigurationData.home[2]), 
                                                                   &(robotConfigurationData.home[3]), 
                                                                   &(robotConfigurationData.home[4]), 
                                                                   &(robotConfigurationData.home[5]));                 
                     break;
            case 5:  sscanf(input_string, " %s %f %f %f %f %f %f", key,                    // degree
                                                                   &(robotConfigurationData.degree[0]), 
                                                                   &(robotConfigurationData.degree[1]), 
                                                                   &(robotConfigurationData.degree[2]), 
                                                                   &(robotConfigurationData.degree[3]), 
                                                                   &(robotConfigurationData.degree[4]), 
                                                                   &(robotConfigurationData.degree[5]));  
                     break;
            case 6:  sscanf(input_string, " %s %d %d %d", key,                             // effector
                                                          &(robotConfigurationData.effector_x),  
                                                          &(robotConfigurationData.effector_y), 
                                                          &(robotConfigurationData.effector_z));                                                    
                     break;
            case 7:  sscanf(input_string, " %s %s ", key, value);                          // wrist
                     for (j=0; j < (int) strlen(value); j++)
                        value[j] = tolower(value[j]);
                     if (strcmp(value,"lightweight")==0) {
                        robotConfigurationData.lightweightWrist = true;
                     }
                     else {
                        robotConfigurationData.lightweightWrist = false;
                     }
                     break;
		     case 8: sscanf(input_string, " %s %f %f %f %f %f %f", key,                    // current_joint_value
                                                                   &(robotConfigurationData.current_joint_value[0]), 
                                                                   &(robotConfigurationData.current_joint_value[1]), 
                                                                   &(robotConfigurationData.current_joint_value[2]), 
                                                                   &(robotConfigurationData.current_joint_value[3]), 
                                                                   &(robotConfigurationData.current_joint_value[4]), 
                                                                   &(robotConfigurationData.current_joint_value[5]));  
                     break;

            }
         }
      }
   }

   if (debug) { 
      printf("COM:      %s\n",robotConfigurationData.com);
      printf("BAUD:     %d\n",robotConfigurationData.baud);
      printf("SPEED:    %d\n",robotConfigurationData.speed);
      printf("CHANNEL:  "); for (k=0; k<6; k++) printf("%d ", robotConfigurationData.channel[k]);   printf("\n");
      printf("HOME:     "); for (k=0; k<6; k++) printf("%d ", robotConfigurationData.home[k]);      printf("\n");
      printf("DEGREE:   "); for (k=0; k<6; k++) printf("%3.1f ", robotConfigurationData.degree[k]); printf("\n");
      printf("EFFECTOR: "); printf("%d %d %d \n", robotConfigurationData.effector_x, robotConfigurationData.effector_y, robotConfigurationData.effector_z);
      printf("WRIST:    "); printf("%s \n", value);
      printf("CURRENT:  "); for (k=0; k<6; k++) printf("%4.3f ", robotConfigurationData.current_joint_value[k]); printf("\n");
   }
}


/******************************************************************************
   
   Serial port interface 

   Based on code written by Victor Akinwande, Carnegie Mellon University Africa
   
   Modified by: David Vernon, Carnegie Mellon University Africa

*******************************************************************************/

/*
 *  intialize the port, baud rate, and speed global variables
 */


void goHome() {

    executeCommand(robotConfigurationData.channel, robotConfigurationData.home, robotConfigurationData.speed, 6);
}


/* execute command for multiple servo motors */

void executeCommand(int *channel, int *pos, int speed, int number_of_servos) {

    char command[COMMAND_SIZE] = {0};

    for(int i =0; i< number_of_servos; i++) {

        char temp[COMMAND_SIZE] = {0};
        sprintf(temp, " #%dP%d", channel[i], pos[i]); // David Vernon ... added space before #; without this port 0 is not affected

        strcat(command, temp);

        sprintf(temp, "S%d", robotConfigurationData.speed); // David Vernon ... append the speed argument to each servo command 
        strcat(command, temp);                              // David Vernon

        strcat(command, " ");
  
    }
     
    sendToSerialPort(command);
}


/* execute command for single servo motor */

void executeCommand(int channel, int pos, int speed) {

    char command[200];

    sprintf(command, " #%dP%dS%d ", channel, pos, speed); // David Vernon ... added space before # ... without this port 0 is not affected
                                                          // also removed the <CR> after the command
    sendToSerialPort(command);
}

/* send the command to the serial port with the echo OS primitive */
/* do this with the system() function                             */

void sendToSerialPort(char *command)
{
    bool debug = false;  
    char execcommand[COMMAND_SIZE];

    if (debug && false) printf("execute(): %s \n", command);

    sprintf(execcommand, "echo \"%s\" > %s", command, robotConfigurationData.com);

    if (debug) printf("%s\n", execcommand);
        
    system(execcommand);

    wait(DEFAULT_SLEEP_TIME); 
}

//helper methods

/**
 * Print failure message and return
 * @param message, output message
 */

void fail(char *message)
{
    printf("%s\n", message);
    printf("Enter any character to finish >>");
    getchar();
    exit(1);
}


/*=======================================================*/
/* Utility functions                                     */ 
/*=======================================================*/


void display_error_and_exit(const char *error_message) {
   printf("%s\n", error_message);
   printf("Hit any key to continue >>");
   getchar();
   exit(0);
}


void prompt_and_exit(int status) {
   printf("Press any key to continue and close terminal ... \n");
   getchar();
   exit(status);
}

void prompt_and_continue() {
   printf("Press any key to continue ... \n");
   getchar();
}


void wait(int ms)
{
#ifdef ROS
    usleep(ms * 1000);
#else
    Sleep(ms);
#endif
}

void print_message_to_file(FILE *fp, char message[]) {
   fprintf(fp,"The message is: %s\n", message);
}

#ifdef ROS
int spawn_brick(std::string name, std::string color, double x, double y, double z, double phi) {

    bool debug = false;

    if (debug) {
       printf("spawn_brick:  %s with color %s at position (%.2f %.2f %.2f %.2f)\n",
	      name.c_str(), color.c_str(), x, y, z, phi);
    }
  
    // The values are expected to be in mm and degrees
    ros::NodeHandle nh;
    ros::service::waitForService("/lynxmotion_al5d/spawn_brick");
    ros::ServiceClient client = nh.serviceClient<lynxmotion_al5d_description::SpawnBrick>("/lynxmotion_al5d/spawn_brick");
    lynxmotion_al5d_description::SpawnBrick srv;

    srv.request.name  = name;
    srv.request.color = color;
    srv.request.pose.position.x = x / 1000.0;
    srv.request.pose.position.y = y / 1000.0;
    srv.request.pose.position.z = z / 1000.0;
    srv.request.pose.orientation.yaw = radians(phi);

    if (client.call(srv))
    {
      if (debug) ROS_INFO("Spawned brick [%s] of color [%s] at position (%.3f %.3f %.3f %.2f %.3f %.3f)", srv.response.name.c_str(), color.c_str(), (x/1000.0), (y/1000.0), (z/1000.0), 0.0, 0.0, radians(phi));
      wait(1000); // pause before continuing
    }
    else
    {
        ROS_ERROR("Failed to call the service");
        return 1;
    }
    
    return 0;
}    

int kill_brick(std::string name) {

    bool debug = false;

    if (debug) {
       printf("kill_brick: %s\n",name.c_str());
    }
  
    ros::NodeHandle nh;
    ros::service::waitForService("/lynxmotion_al5d/kill_brick");
    ros::ServiceClient client = nh.serviceClient<lynxmotion_al5d_description::KillBrick>("/lynxmotion_al5d/kill_brick");
    lynxmotion_al5d_description::KillBrick srv;

    srv.request.name  = name.c_str();

    if (client.call(srv))
    {
        if (debug) ROS_INFO("Killed brick [%s]", name.c_str());
    }
    else
    {
        ROS_ERROR("Failed to call the service");
        return 1;
    }
    
    return 0;
}    

#endif
