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
 
#include "module5/imageAcquisitionFromUSBCamera.h"


/*===================================================*/
/* display images from a camera in an openCV window  */
/* pass the index of the camera as a parameter       */
/*===================================================*/

void display_image_from_camera(int cameraNum) {

   VideoCapture camera;            //  the camera device
   Mat frame;					        //  save an image read from a camera
   vector<int> compressionParams;  // parameters for image write

   char windowName[MAX_STRING_LENGTH]; 
   char cameraNumber[MAX_STRING_LENGTH]; 

   strcpy(windowName,"Camera");
   sprintf(cameraNumber, " %d", cameraNum);

   namedWindow(windowName,     WINDOW_AUTOSIZE); // create the window  
    
   if (camera.open(cameraNum) == true) {          // open the camera input 

      printf("Press any key to stop image display\n");

      camera >> frame;                             // read a frame from the camera to get the image size ... this is actually C++

      // printf("Camera image is %d x %d\n", frame.cols, frame.rows);
   
      do {
         camera >> frame;                          // read a frame from the camera                    
         imshow(windowName, frame);
         waitKey(30); // this is essential as it allows openCV to handle the display event ... 
                      // the argument is the number of milliseconds to wait 
      } while (!_kbhit());

      getchar(); // flush the buffer from the keyboard hit
      
      compressionParams.push_back(IMWRITE_PNG_COMPRESSION);
      compressionParams.push_back(9);                                  // 9 implies maximum compression

      imwrite("../data/camera_image.png", frame, compressionParams);   // write the image to a file just for fun

      destroyWindow(windowName);   
   }
   else {
      printf("Failed to open camera %d\n",cameraNum);
      prompt_and_continue();
   }
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
