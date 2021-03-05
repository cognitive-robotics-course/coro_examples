/* 
  Example use of openCV to acquire and display images from file and from a video camera
  -------------------------------------------------------------------------------------
  
  (This is the implementation file: it contains the code for dedicated functions to implement the application.
  These functions are called by client code in the application file. The functions are declared in the interface file.) 

  David Vernon
  24 November 2017

  Changed to C++ version for camera acquisition. DV 27/2/2018
*/
 
#include "imageAcquisition.h"

/*================================================*/
/* Display images from a file in an openCV window */
/* pass the filename as a parameter               */
/*================================================*/

void display_image_from_file(char *filename) {
  
   string inputWindowName     = "Input Image"; 
  
   Mat image;
   Mat processedImage;
    
   namedWindow(inputWindowName, CV_WINDOW_AUTOSIZE);// create the window  

   image = imread(filename, CV_LOAD_IMAGE_COLOR);   // Read the file

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

   namedWindow(windowName,     CV_WINDOW_AUTOSIZE); // create the window  
    
   if (camera.open(cameraNum) == true) {          // open the camera input 

      printf("Press any key to stop image display\n");

      camera >> frame;                             // read a frame from the camera to get the image size ... this is actually C++

      // printf("Camera image is %d x %d\n", frame.cols, frame.rows);
   
	   do {
         camera >> frame;                          // read a frame from the camera                    
         imshow(windowName, frame);
         cvWaitKey(30); // this is essential as it allows openCV to handle the display event ... 
                        // the argument is the number of milliseconds to wait 
      } while (!_kbhit());

      getchar(); // flush the buffer from the keyboard hit
      
      compressionParams.push_back(CV_IMWRITE_PNG_COMPRESSION);
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
   exit(status);
}

void prompt_and_continue() {
   printf("Press any key to continue ... \n");
   getchar();
}

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
