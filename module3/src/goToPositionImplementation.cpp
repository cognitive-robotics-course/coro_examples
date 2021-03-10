/*******************************************************************************************************************
*   
*   Module 3 example: implementation of the divide-and-conquer go-to-position algorithm 
*
*   This is the implementation file.
*   For documentation, please see the application file
*
*   David Vernon
*   10 March 2021
*
*   Audit Trail
*   -----------
*
*******************************************************************************************************************/

#include <module3/goToPosition.h> 

  
/* global variables with the current turtle pose */

extern float         current_x; 
extern float         current_y; 
extern float         current_theta;

/* Callback function, executed each time a new pose message arrives */

void poseMessageReceived(const turtlesim::Pose& msg) {
  bool debug = false;

   if (debug) {ROS_INFO_STREAM(std::setprecision(2) << std::fixed <<
	                       "position=(" << msg.x << "," << msg.y << ")" <<
		               " direction=" << msg.theta);
   }
   
   current_x = msg.x;
   current_y = msg.y;
   current_theta = msg.theta;
}

/*=======================================================*/
/* Utility functions                                     */ 
/*=======================================================*/


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

