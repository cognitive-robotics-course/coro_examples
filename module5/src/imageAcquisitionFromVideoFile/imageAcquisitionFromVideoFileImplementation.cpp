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
    

  Changed to C++ version for camera acquisition. DV 27/2/2018
*/
 
#include "module5/imageAcquisitionFromVideoFile.h"


/*=======================================================*/
/* display images from a video file in an openCV window  */
/* pass the filename of the video as a parameter         */
/*=======================================================*/

void display_image_from_video(char *filename) {
  
   VideoCapture video;     //  the video device
   Mat frame;					//  an image read from a camera
   Mat processedImage;     //  a processed image
   string inputWindowName  = "Input Image"; 
   
   namedWindow(inputWindowName, CV_WINDOW_AUTOSIZE); // create the window  

   video.open(filename);                     // open the video input 
   if (video.isOpened()){
      printf("Press any key to stop image display\n");
	   
      do {
         video >> frame;                     // read a frame from the video 

         if (!frame.empty()) {
            imshow(inputWindowName, frame);  // show our image inside it.
            waitKey(30);                     // this is essential as it allows openCV to handle the display event ... 
                                             // the argument is the number of milliseconds to wait 
         }
      } while ((!_kbhit()) && (!frame.empty()));
      
      getchar(); // flush the buffer from the keyboard hit
      destroyWindow(inputWindowName);   
   }
   else {
      printf("Failed to open video file\n");
      prompt_and_continue();
      return;
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