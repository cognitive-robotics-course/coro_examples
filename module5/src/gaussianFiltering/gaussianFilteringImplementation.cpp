/* 
  Example use of openCV to remove noise using Gaussian filtering 
  --------------------------------------------------------------
  
  (This is the implementation file: it contains the code for dedicated functions to implement the application.
  These functions are called by client code in the application file. The functions are declared in the interface file.) 

  David Vernon
  24 November 2017
*/
 
#include "gaussianFiltering.h"

/*
 * function processNoiseAndAveraging
 * Trackbar callback - add Gaussian noise with standard deviation input from user
 * Trackbar callback - remove noise with local averaging using filter size input from user
*/

void processNoiseAndAveraging(int, void*) {  

   extern Mat src; 
   extern int noise_std_dev;
   extern int gaussian_std_dev; 
   extern char* processed_window_name;

   Mat noisy_image;
   Mat filtered_image; 

   int filter_size;

   filter_size = gaussian_std_dev * 4 + 1;
	noisy_image = src.clone();

	addGaussianNoise(noisy_image, 0.0, (double)noise_std_dev); 
	
   GaussianBlur(noisy_image,filtered_image,Size(filter_size,filter_size),gaussian_std_dev);
 
   imshow(processed_window_name, filtered_image);
 }

void prompt_and_exit(int status) {
   printf("Press any key to continue and close terminal ... \n");
   getchar();
   exit(status);
}
 

/************************************************************************************************/

/*
 * This code is provided as part of "A Practical Introduction to Computer Vision with OpenCV"
 * by Kenneth Dawson-Howe � Wiley & Sons Inc. 2014.  All rights reserved.
 */

void addGaussianNoise(Mat &image, double average, double standard_deviation)
{
	// We need to work with signed images (as noise can be negative as well as positive).
	// We chose 16 bit signed images as if we converted an 8 bits unsigned image to a
	// signed version we would lose precision.

	/* Amended by David Vernon to faciliate images with four channels - 24/5/2017 */
   
   //int image_type = (image.channels() == 3) ? CV_16SC3 : CV_16SC1;

   int image_type;

   if (image.channels() == 1)
      image_type = CV_16SC1;
   else if (image.channels() == 3)
      image_type = CV_16SC3;
   else if (image.channels() == 4)
      image_type = CV_16SC4;

   /* end of amended code */

	Mat noise_image(image.size(), image_type);
   randn(noise_image, Scalar::all(average), Scalar::all(standard_deviation));
	
   Mat temp_image;
   image.convertTo(temp_image,image_type); 

	addWeighted(temp_image,1.0,noise_image,1.0,0.0,temp_image);
	temp_image.convertTo(image,image.type());
}

/************************************************************************************************/

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