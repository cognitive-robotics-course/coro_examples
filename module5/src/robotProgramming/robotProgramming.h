/*******************************************************************************************************************
*   Task-level robot programming for a LynxMotion AL5D robot arm
*
*   Interface file
*
*   Refer to the application file for documentation
*
********************************************************************************************************************/



/* Uncomment the line below if compiling for use with ROS. 

   ROS code will then be compiled in 

   setJointAngles()  
   grasp() 
   
   to publish the joint angles on the /lynxmotion_al5d/joints_positions/command topic with ROS

   If left commented out, these two functions write the servocontrol setpoints to the COM port on Windows

   NOTE: the ROS publisher code has not yet been written
*/

//#define ROS  




#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <cmath>  
#include <ctype.h>
#include <iostream>

using namespace std;

#define _USE_MATH_DEFINES   
#include <math.h>         
#include <float.h>         
#include <Windows.h>  
#define wait Sleep

#if defined(_MSC_VER)      // from https://www.christophlassner.de/collection-of-msvc-gcc-compatibility-tricks.html
#if _MSC_VER < 1800
namespace std {
  template <typename T>
  bool isnan(const T &x) { if(_isnan(x) == 0) return false; else return true; } // modified by David Vernon to return explicit Boolean value

  template <typename T>
  bool isinf(const T &x) { return !_finite(x); }
}
#endif
#endif



/***************************************************************************************************************************

   Definitions for reading robot configuration data 

****************************************************************************************************************************/

#define MAX_FILENAME_LENGTH 80
#define STRING_LENGTH 200
#define KEY_LENGTH 20
#define NUMBER_OF_KEYS 9
typedef char keyword[KEY_LENGTH];
    

/***************************************************************************************************************************

   Definitions for servo control 

****************************************************************************************************************************/

#define DEFAULT_SLEEP_TIME 5   
#define COMMAND_SIZE 200
#define MAX_SERVOS 32
#define MIN_PW 750  //lowest pulse width
#define MAX_PW 2250 //highest pulsewidth

// working envelope in mm

#define MIN_X -130
#define MAX_X  130
#define MIN_Y   80
#define MAX_Y  330
#define MIN_Z    0
#define MAX_Z  380

#define MAX(x, y) (((x) > (y)) ? (x) : (y))
#define MIN(x, y) (((x) < (y)) ? (x) : (y))

/***************************************************************************************************************************

   Frame and vector classes to support task-level robot programming 
   ----------------------------------------------------------------

   Interface to the Frame and Vector classes and auxilliary friend functions required to implement a robot control program
   by specifying the position and orientation of objects and the robot manipulator using homogeneous transformations
   
   Author: David Vernon, Carnegie Mellon University Africa, Rwanda
   Date:   22/02/2017

****************************************************************************************************************************/

#define MAX_MESSAGE_LENGTH 81

class Vector {
public:
   Vector(double x=0, double y=0, double z=0, double w=1);
   void setValues(double x, double y, double z, double w);
   void getValues(double &x, double &y, double &z, double &w);
   void printVector()const;
   friend Vector operator+(Vector &a, Vector &b);
   friend double dotProduct(Vector &a, Vector &b);
private:
   double coefficient[4];
};


class Frame {
public:
   Frame();
   void printFrame()const;
   Frame        &operator*(Frame &h);
   Frame        &operator=(Frame &h);
   friend Frame trans(float x, float y, float z);
   friend Frame rotx(float theta);
   friend Frame roty(float theta);
   friend Frame rotz(float theta);
   friend Frame inv(Frame h);
   friend bool  move(Frame h);
private:
   double coefficient[4][4];
};

/* function prototypes */

Vector operator+(Vector &a, Vector &b);
double dotProduct(Vector &a, Vector &b);
Frame trans(float x, float y, float z);
Frame rotx(float theta);
Frame roty(float theta);
Frame rotz(float theta);
Frame inv(Frame h);

bool move(Frame T5);
void grasp(int d);

void prompt_and_exit(int status);
void print_message_to_file(FILE *fp, char message[]);



/****************************************************************************************************************************
   Serial port interface 
   ---------------------

   Author: Victor Akinwande, Carnegie Mellon University Africa
   Modified by: David Vernon, Carnegie Mellon University Africa

****************************************************************************************************************************/

/** 
 * set the positions of the servos connected to 
 * the controller to their default positions
 */ 

void goHome();
void sendToSerialPort(char *command);
void executeCommand(int channel, int pos, int speed);                           // single servo motor
void executeCommand(int * channel, int * pos, int speed, int number_of_servos); // multiple servo motors

// Helper function prototypes
void fail(char *message);


/***************************************************************************************************************************

   Inverse kinematics for LynxMotion AL5D robot manipulator

*****************************************************************************************************************************/

/* Kinematics: arm dimensions (mm) for AL5D arm */

#define D1 70              // Base height to X/Y plane
#define A3 146.0           // Shoulder-to-elbow "bone"
#define A4 187.0           // Elbow-to-wrist "bone"
#define EZ 100             // Gripper length
#define GRIPPER_OPEN    30 // mm
#define GRIPPER_CLOSED  0  // mm


bool computeJointAngles(double x, double y, double z, double pitch, double roll, double joint_angles[]);
bool setJointAngles(double joint_angles[]);
bool computeServoPositions(double joint_angles[], int servo_positions[]);

//int  gotoPose(float x, float y, float z, float pitch, float roll);

void sig(int s);
int  pose_within_working_env(float x, float y, float z);
void display_error_and_exit(char error_message[]);
double degrees(double radians);
double radians(double degrees);


/***************************************************************************************************************************

   Robot Configuration 

****************************************************************************************************************************/
  
struct robotConfigurationDataType {
   char  com[13];                     // USB com port ID on windows ... need to check this in the control panel > device manager or use the SSC-32 servo sequencer utility
   int   baud;                        // Baud Rate
   int   speed;                       // Servo speed
   int   channel[6];                  // pin / channel numbers for each servo
   int   home[6];                     // setpoint for home position of each servo
   float degree[6];                   // pulse width per degree (in microseconds)
   int   effector_x;                  // x offset of end-effector w.r.t.  T5
   int   effector_y;                  // y offset of end-effector w.r.t.  T5
   int   effector_z;                  // z offset of end-effector w.r.t.  T5
   bool  lightweightWrist;            // one robot has a heavy duty wrist and it reverses the direction of the roll angle
   float current_joint_value[6];      // Integrate the five joint angles wth the gripper distance to facilitate the constuction of the ROS topic message
};

void readRobotConfigurationData(char filename[]);


/***************************************************************************************************************************

   Utility functions 

****************************************************************************************************************************/

void prompt_and_exit(int status);
void prompt_and_continue();
