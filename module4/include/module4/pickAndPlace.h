/*******************************************************************************************************************
*
*   Interface file for the LynxMotion AL5D robot pick-and-place implementation functions
*
*   Audit Trail
*   -----------
*   Extracted from the original implementation interface file 
*   David Vernon
*   11 February 2022
*
********************************************************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <cmath>  
#include <ctype.h>
#include <iostream>
#include <math.h>
#include <vector>
#include <ros/ros.h>

using namespace std;


/***************************************************************************************************************************

   Utility functions 

****************************************************************************************************************************/

void prompt_and_exit(int status);
void prompt_and_continue();
void display_error_and_exit(const char *error_message);
void print_message_to_file(FILE *fp, char message[]);
void fail(char *message);
void wait(int ms);
