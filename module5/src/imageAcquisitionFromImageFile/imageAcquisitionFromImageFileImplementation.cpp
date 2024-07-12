/* 
  Example use of openCV to acquire and display images from file and from a video camera
  -------------------------------------------------------------------------------------
  
  (This is the implementation file: it contains the code for dedicated functions to implement the application.
  These functions are called by client code in the application file. The functions are declared in the interface file.) 

  David Vernon
  24 November 2017

  Audit Trail
  --------------------
  Added _kbhit
  18 February 2021
    
  Changed to C++ version for camera acquisition. 
  DV 27/2/2018
   
  Ported to OpenCV 4
  David Vernon
  11 July 2024
*/
 
#include "module5/imageAcquisitionFromImageFile.h"

/*================================================*/
/* Display images from a file in an openCV window */
/* pass the filename as a parameter               */
/*================================================*/

void display_image_from_file(char *filename) {
  
   string inputWindowName     = "Input Image"; 
  
   Mat image;
   Mat processedImage;
    
   namedWindow(inputWindowName, WINDOW_AUTOSIZE);// create the window  

   image = imread(filename, IMREAD_COLOR);       // Read the file

   if (!image.data) {                               // Check for invalid input
      printf("Error: failed to read image\n");
      prompt_and_exit(-1);
   }

   printf("Press any key to stop image display\n");

   imshow(inputWindowName, image );                 // show our image inside it.
    
   do{
      waitKey(30);                                  // Must call this to allow openCV to display the images
   } while (!_kbhit());                             // We call it repeatedly to allow the user to move the windows
                                                    // (if we don't the window process hangs when you try to click and drag)
   
   getchar(); // flush the buffer from the keyboard hit

   destroyWindow(inputWindowName);   
}


/*=======================================================*/
/* Utility functions to prompt user to continue          */ 
/*=======================================================*/

void prompt_and_exit(int status) {
   printf("Press any key to continue and close terminal ... \n");
   getchar();
   

   #ifdef ROS
      // Reset terminal to canonical mode
      static const int STDIN = 0;
      termios term;
      tcgetattr(STDIN, &term);
      term.c_lflag |= (ICANON | ECHO);
      tcsetattr(STDIN, TCSANOW, &term);
      exit(status);
   #endif

   exit(status);
}

void prompt_and_continue() {
   printf("Press any key to continue ... \n");
   getchar();
}



#ifdef ROS
/**
 Linux (POSIX) implementation of _kbhit().
 Morgan McGuire, morgan@cs.brown.edu
 */
int _kbhit() {
    static const int STDIN = 0;
    static bool initialized = false;

    if (! initialized) {
        // Use termios to turn off line buffering
        termios term;
        tcgetattr(STDIN, &term);
        term.c_lflag &= ~ICANON;
        tcsetattr(STDIN, TCSANOW, &term);
        setbuf(stdin, NULL);
        initialized = true;
    }

    int bytesWaiting;
    ioctl(STDIN, FIONREAD, &bytesWaiting);
    return bytesWaiting;
}
#endif
