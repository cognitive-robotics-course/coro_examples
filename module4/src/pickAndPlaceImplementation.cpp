/*******************************************************************************************************************
 *   Task-level robot programming for a LynxMotion AL5D robot arm
 *   --------------------------------------------------------------------
 *
 *   Implementation file
 *
 *   Audit Trail
 *   -----------
 *   11 February 2022: factored out the Lynxmotion utility functions into a separate file (lynxmotionUtilities.cpp)
 *
 *
 *******************************************************************************************************************/

#include <module4/pickAndPlace.h>

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
   usleep(ms * 1000);
}

void print_message_to_file(FILE *fp, char message[]) {
   fprintf(fp,"The message is: %s\n", message);
}

void fail(char *message) {
    printf("%s\n", message);
    printf("Enter any character to finish >>");
    getchar();
    exit(1);
}


/*=======================================================*/
/* Other functions                                     */ 
/*=======================================================*/

/* if you add functionality to the application, put the relevant functions here */
/* and don't forget to declare them in the interface file, pickAndPlace.h       */
