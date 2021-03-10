/* 
  Example use of openCV to perform 2D feature extraction
  ------------------------------------------------------
  
  (This is the implementation file: it contains the code for dedicated functions to implement the application.
  These functions are called by client code in the application file. The functions are declared in the interface file.) 

  David Vernon
  24 November 2017

  Audit Trail
  --------------------
  Added _kbhit
  18 February 2021
    
*/
 
#include "module5/featureExtraction.h"
 
void featureExtraction(char *filename, FILE *fp_out) {
  
   char inputWindowName[MAX_STRING_LENGTH]              = "Input Image";
   char outputWindowName[MAX_STRING_LENGTH]             = "Contour Image";
 
   Mat inputImage;
 
   namedWindow(inputWindowName,   CV_WINDOW_AUTOSIZE);  
   namedWindow(outputWindowName,  CV_WINDOW_AUTOSIZE);

   inputImage = imread(filename, CV_LOAD_IMAGE_COLOR); // Read the file

   if (!inputImage.data) {                            // Check for invalid input
      printf("Error: failed to read image %s\n",filename);
      prompt_and_exit(-1);
   }

   printf("Press any key to continue ...\n");

   fprintf(fp_out,"%s \n",filename);  // file write added by David Vernon 

/*
 * The following is based on code provided as part of "A Practical Introduction to Computer Vision with OpenCV"
 * by Kenneth Dawson-Howe © Wiley & Sons Inc. 2014.  All rights reserved.
 */

   /* convert the input image to a binary image */
	Mat gray;
   Mat binary;

	cvtColor(inputImage, gray, CV_BGR2GRAY);
	//threshold(gray,binary,128,255,THRESH_BINARY_INV);
   threshold(gray,binary,128, 255,THRESH_BINARY_INV  | THRESH_OTSU); // David Vernon: substituted in automatic threshold selection
 
   /* extract the contours of the objects in the binary image */
   vector <vector<Point> > contours;
	vector<Vec4i>         hierarchy;

   /* David Vernon: see http://docs.opencv.org/2.4.10/modules/imgproc/doc/structural_analysis_and_shape_descriptors.html#findcontours */
	findContours(binary,contours,hierarchy,CV_RETR_TREE,CV_CHAIN_APPROX_NONE);

   /* extract features from the contours */
	Mat contours_image = Mat::zeros(binary.size(), CV_8UC3);
	contours_image = Scalar(255,255,255);
	
   //binary.copyTo(contours_image,binary);

	/* Prepare to do some processing on all contours (objects and holes!) by declaring appropriate data-structures */
	vector<RotatedRect>   min_bounding_rectangle(contours.size());   // bounding rectangles
	vector <vector<Point> > hulls(contours.size());                    // convex hulls
	vector <vector<int> >   hull_indices(contours.size());             // indices of convex hulls
	vector <vector<Vec4i> > convexity_defects(contours.size());        // convex cavities
	vector<Moments>       contour_moments(contours.size());          // Hu moments

	for (int contour_number=0; (contour_number<(int)contours.size()); contour_number++) {
		if (contours[contour_number].size() > 10) { // only consider contours of appreciable length

         /* David Vernon: see http://docs.opencv.org/2.4.10/modules/imgproc/doc/structural_analysis_and_shape_descriptors.html#boundingrect */
			min_bounding_rectangle[contour_number] = minAreaRect(contours[contour_number]);

         /* David Vernon: see http://docs.opencv.org/2.4.10/modules/imgproc/doc/structural_analysis_and_shape_descriptors.html#convexhull */
			convexHull(contours[contour_number], hulls[contour_number]);
			convexHull(contours[contour_number], hull_indices[contour_number]);

         /* David Vernon: see http://docs.opencv.org/2.4.10/modules/imgproc/doc/structural_analysis_and_shape_descriptors.html#convexitydefects */
			convexityDefects(contours[contour_number], hull_indices[contour_number], convexity_defects[contour_number]);

         /* David Vernon: see http://docs.opencv.org/2.4.10/modules/imgproc/doc/structural_analysis_and_shape_descriptors.html#moments*/
			contour_moments[contour_number] = moments( contours[contour_number] );
		}
	}

   /* for all contours */
	for (int contour_number=0; (contour_number>=0); contour_number=hierarchy[contour_number][0]) {

      /* only consider contours of appreciable length */
		if (contours[contour_number].size() > 10) {
         Scalar colour(rand()&0x7F, rand()&0x7F, rand()&0x7F );                                    // generate a random colour 
         drawContours(contours_image, contours, contour_number, colour, CV_FILLED, 8, hierarchy ); // draw the contour
		
         char output[500];

         // David Vernon: Ken Dawson-Howe adjusts area as it seems to be underestimated by half the number of pixels on the perimeter
		   double area = contourArea(contours[contour_number]) + contours[contour_number].size()/2 + 1; 

		   // Process any holes (removing the area from the area of the enclosing contour)
         for (int hole_number=hierarchy[contour_number][2]; (hole_number>=0); hole_number=hierarchy[hole_number][0]) {
           
            // David Vernon: Ken Dawson-Howe adjusts area as it seems to be underestimated by half the number of pixels on the perimeter
            area -= (contourArea(contours[hole_number]) - contours[hole_number].size()/2 + 1); 
            
            Scalar colour( rand()&0x7F, rand()&0x7F, rand()&0x7F );
            drawContours( contours_image, contours, hole_number, colour, CV_FILLED, 8, hierarchy );

            sprintf(output,"Area=%.0f", contourArea(contours[hole_number]) -contours[hole_number].size()/2+1); 

            /* write to file added by David Vernon */
            fprintf(fp_out,"Object %d, Hole %d: Area = %.0f\n",
                    contour_number, hole_number, contourArea(contours[hole_number]) - contours[hole_number].size()/2 + 1);

            Point location( contours[hole_number][0].x + 20, contours[hole_number][0].y + 5 );  
            putText( contours_image, output, location, FONT_HERSHEY_SIMPLEX, 0.4, colour );
         }

         /* Draw the minimum bounding rectangle */
         Point2f bounding_rect_points[4];
         min_bounding_rectangle[contour_number].points(bounding_rect_points);
         line(contours_image, bounding_rect_points[0], bounding_rect_points[1], Scalar(0, 0, 127));
         line(contours_image, bounding_rect_points[1], bounding_rect_points[2], Scalar(0, 0, 127));
         line(contours_image, bounding_rect_points[2], bounding_rect_points[3], Scalar(0, 0, 127));
         line(contours_image, bounding_rect_points[3], bounding_rect_points[0], Scalar(0, 0, 127));

         float bounding_rectangle_area = min_bounding_rectangle[contour_number].size.area();
		
         /* Draw the convex hull */
         drawContours(contours_image, hulls, contour_number, Scalar(255,0,255) );  // purple
		
         /* Highlight any convexities */
         int largest_convexity_depth=0;
		
         for (int convexity_index=0; convexity_index < (int)convexity_defects[contour_number].size(); convexity_index++) {
			   if (convexity_defects[contour_number][convexity_index][3] > largest_convexity_depth)
               largest_convexity_depth = convexity_defects[contour_number][convexity_index][3];
			
            if (convexity_defects[contour_number][convexity_index][3] > 256*2) {
               line( contours_image, contours[contour_number][convexity_defects[contour_number][convexity_index][0]], 
                                     contours[contour_number][convexity_defects[contour_number][convexity_index][2]], Scalar(0,0, 255));
               line( contours_image, contours[contour_number][convexity_defects[contour_number][convexity_index][1]], 
                                     contours[contour_number][convexity_defects[contour_number][convexity_index][2]], Scalar(0,0, 255));
            }
         }

         //sprintf(output,"Perimeter=%d, Area=%.0f, BArea=%.0f, CArea=%.0f", contours[contour_number].size(),area,min_bounding_rectangle[contour_number].size.area(),contourArea(hulls[contour_number]));
         /* David Vernon: alternative as area seems to be underestimated by half the number of pixels on the perimeter */
         sprintf(output,"Perimeter=%d, Area=%.0f, BArea=%.0f, CArea=%.0f", contours[contour_number].size(),
                                                                           area,
                                                                           min_bounding_rectangle[contour_number].size.area() + contours[contour_number].size()/2 + 1,
                                                                           contourArea(hulls[contour_number])+ contours[contour_number].size()/2 + 1);
         /* file write added by David Vernon */
         /* David Vernon: area seems to be underestimated by half the number of pixels on the perimeter */
         fprintf(fp_out,"Object %d: perimeter = %d, object area = %.0f, bounding rectangle area = %.0f, convex hull area = %.0f \n",
                 contour_number, 
                 contours[contour_number].size(),
                 area,
                 min_bounding_rectangle[contour_number].size.area() + contours[contour_number].size()/2 + 1,
                 contourArea(hulls[contour_number]) + contours[contour_number].size()/2 + 1);

         Point location( contours[contour_number][0].x, contours[contour_number][0].y-3 );
         putText(contours_image, output, location, FONT_HERSHEY_SIMPLEX, 0.4, colour );
		
         /* David Vernon: see http://docs.opencv.org/2.4.10/modules/imgproc/doc/structural_analysis_and_shape_descriptors.html#humoments */
         double hu_moments[7];
         HuMoments(contour_moments[contour_number], hu_moments );

         sprintf(output,"HuMoments = %.2f, %.2f, %.2f", hu_moments[0],hu_moments[1],hu_moments[2]);
         Point location2( contours[contour_number][0].x+100, contours[contour_number][0].y-3+15 );
         putText(contours_image, output, location2, FONT_HERSHEY_SIMPLEX, 0.4, colour );

         /* filewrite added by David Vernon */
         fprintf(fp_out,"Object %d: HuMoments = %.2f, %.2f, %.2f \n\n", contour_number, hu_moments[0],hu_moments[1],hu_moments[2]);
      }
      fprintf(fp_out,"\n"); //file write added by David Vernon 
	}

   imshow(inputWindowName,  inputImage );        
   imshow(outputWindowName, contours_image);  

   do{
      waitKey(30);           
   } while (!_kbhit());              
                                    
   getchar(); // flush the buffer from the keyboard hit

   destroyWindow(inputWindowName);  
   destroyWindow(outputWindowName); 
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