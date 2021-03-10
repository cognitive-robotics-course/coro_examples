/* 
  Example use of openCV to perform automatic binary thresholding using the Otsu algorithm
  ---------------------------------------------------------------------------------------
  
  (This is the implementation file: it contains the code for dedicated functions to implement the application.
  These functions are called by client code in the application file. The functions are declared in the interface file.) 

  David Vernon
  24 November 2017
*/
 
#include "binaryThresholdingOtsu.h"

void binaryThresholdingOtsu(char *filename) {  

   Mat inputImage;
   Mat greyscaleImage;
   Mat thresholdedImage; 

   int thresholdValue            = 128; // default threshold

   char* input_window_name       = "Input Image";
   char* thresholded_window_name = "Thresholded Image";
 
   inputImage = imread(filename, CV_LOAD_IMAGE_UNCHANGED);
   if (inputImage.empty()) {
      cout << "can not open " << filename << endl;
      prompt_and_exit(-1);
   }
          
   printf("Press any key to continue ...\n");

   // Create a window for input and display it
   namedWindow(input_window_name, CV_WINDOW_AUTOSIZE );
   imshow(input_window_name, inputImage);
  
   // Create a window for thresholded image
   namedWindow(thresholded_window_name, CV_WINDOW_AUTOSIZE );
 
   if (inputImage.type() == CV_8UC3) { // colour image
      cvtColor(inputImage, greyscaleImage, CV_BGR2GRAY);
   } 
   else {
      greyscaleImage = inputImage.clone();
   }

   //thresholdedImage.create(greyscaleImage.size(), CV_8UC1);

   threshold(greyscaleImage,thresholdedImage,thresholdValue, 255,THRESH_BINARY  | THRESH_OTSU); // automatic threshold selection
 
   imshow(thresholded_window_name, thresholdedImage);
         
   do {
      waitKey(30);                                  // Must call this to allow openCV to display the images
   } while (!_kbhit());                             // We call it repeatedly to allow the user to move the windows
                                                   // (if we don't the window process hangs when you try to click and drag

   getchar(); // flush the buffer from the keyboard hit

   destroyWindow(input_window_name);  
   destroyWindow(thresholded_window_name); 
}

void prompt_and_exit(int status) {
   printf("Press any key to continue and close terminal ... \n");
   getchar();
   exit(status);
} 