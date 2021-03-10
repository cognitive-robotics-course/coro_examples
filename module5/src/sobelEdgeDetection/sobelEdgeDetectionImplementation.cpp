/* 
  Example use of openCV to perform Sobel edge detection
  -----------------------------------------------------
  
  (This is the implementation file: it contains the code for dedicated functions to implement the application.
  These functions are called by client code in the application file. The functions are declared in the interface file.) 

  David Vernon
  24 November 2017
*/
 
#include "sobelEdgeDetection.h"

/*
 * function sobelEdgeDetection
 * Trackbar callback - threshold user input
*/

void sobelEdgeDetection(int, void*) {  

   extern Mat inputImage; 
   extern int thresholdValue; 
   extern char* magnitude_window_name;
   extern char* direction_window_name;
   extern char* edge_window_name;

   Mat greyscaleImage;
   Mat edgeImage;  
   Mat horizontal_partial_derivative;
   Mat vertical_partial_derivative;
	Mat l2norm_gradient;
   Mat orientation; 

   if (inputImage.type() == CV_8UC3) { // colour image
      cvtColor(inputImage, greyscaleImage, CV_BGR2GRAY);
   } 
   else {
      greyscaleImage = inputImage.clone();
   }
    
   /*******************************************************************************************************/
   /*
    * This code is provided as part of "A Practical Introduction to Computer Vision with OpenCV"
    * by Kenneth Dawson-Howe © Wiley & Sons Inc. 2014.  All rights reserved.
    */

	Sobel(greyscaleImage,horizontal_partial_derivative,CV_32F,1,0);
	Sobel(greyscaleImage,vertical_partial_derivative,CV_32F,0,1);
	cartToPolar(horizontal_partial_derivative,vertical_partial_derivative,l2norm_gradient,orientation); 
	Mat l2norm_gradient_gray = convert_32bit_image_for_display( l2norm_gradient );
   Mat l2norm_gradient_mask, display_orientation;
	l2norm_gradient.convertTo(l2norm_gradient_mask,CV_8U);
	threshold(l2norm_gradient_mask,edgeImage,thresholdValue,255,THRESH_BINARY); // DV thresholdValue edgeImage
	orientation.copyTo(display_orientation, edgeImage);
	Mat orientation_gray = convert_32bit_image_for_display(display_orientation, 0.0, 255.0/(2.0*PI) ); 

   imshow(magnitude_window_name, l2norm_gradient_gray); // DV
   imshow(direction_window_name, orientation_gray);     // DV
   imshow(edge_window_name, edgeImage);                 // DV
}

void prompt_and_exit(int status) {
   printf("Press any key to continue and close terminal ... \n");
   getchar();
   exit(status);
} 


/*******************************************************************************************************/
/*
 * This code is provided as part of "A Practical Introduction to Computer Vision with OpenCV"
 * by Kenneth Dawson-Howe © Wiley & Sons Inc. 2014.  All rights reserved.
 */

Mat convert_32bit_image_for_display(Mat& passed_image, double zero_maps_to/*=0.0*/, double passed_scale_factor/*=-1.0*/ )
{
	Mat display_image;
	double scale_factor = passed_scale_factor;
	if (passed_scale_factor == -1.0)
	{
		double minimum,maximum;
		minMaxLoc(passed_image,&minimum,&maximum);
		scale_factor = (255.0-zero_maps_to)/max(-minimum,maximum);
	}
	passed_image.convertTo(display_image, CV_8U, scale_factor, zero_maps_to);
	return display_image;
}

/*******************************************************************************************************/