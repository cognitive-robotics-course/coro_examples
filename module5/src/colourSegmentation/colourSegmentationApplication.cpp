/* 
  Example use of openCV to perform colour segmentation
  ----------------------------------------------------
  
  This application reads a sequence lines from an input file colourSegmentationInput.txt.
  Each line contains a filename of an image to be processed. 

  It is assumed that the input file is located in a data directory given by the path ../data/ 
  defined relative to the location of executable for this application.

  For each image, The user must interactively select the colour sample that will form the basis of the segmentation by clicking on a selected pixel.
  The user can also adjust the hue and saturation tolerances on that sample.

  (This is the application file: it contains the client code that calls dedicated functions to implement the application.
  The code for these functions is defined in the implementation file. The functions are declared in the interface file.)

  David Vernon
  24 November 2017
*/

 
#include "colourSegmentation.h"

// Global variables to allow access by the display window callback functions

Mat inputBGRImage;
Mat inputHLSImage;
int hueRange            = 10; // default range
int saturationRange     = 10; // default range
Point2f sample_point; 
int number_of_sample_points;

char* input_window_name       = "Input Image";
char* segmented_window_name   = "Segmented Image";

int main() {
   
   string                 path;
   string                 input_filename            = "colourSegmentationInput.txt";
   string                 input_path_and_filename;
   string                 data_dir;
   string                 datafile_path_and_filename;
   data_dir = ros::package::getPath(ROS_PACKAGE_NAME); // get the package directory
   data_dir += "/data/";
   input_path_and_filename = data_dir + input_filename;
     
   // Initialize screen in ncurses raw mode
   initscr(); 


   int end_of_file;
   bool debug = false;
   char filename[MAX_FILENAME_LENGTH];
   int max_hue_range = 180;
   int max_saturation_range = 128;
   Mat outputImage;

   FILE *fp_in;
   
   if ((fp_in = fopen(input_path_and_filename.c_str(),"r")) == 0) {
	  printf("Error can't open input colourSegmentationInput.txt\n");
     prompt_and_exit(1);
   }

   printf("Example of how to use openCV to perform colour segmentation.\n\n");
   
   do {
      end_of_file = fscanf(fp_in, "%s", filename);

      if (end_of_file != EOF) {
         datafile_path_and_filename = filename;
         datafile_path_and_filename = data_dir + datafile_path_and_filename;

         inputBGRImage = imread(datafile_path_and_filename, CV_LOAD_IMAGE_UNCHANGED);
         if(inputBGRImage.empty()) {
            cout << "can not open " << filename << endl;
            prompt_and_exit(-1);
         }

         CV_Assert(inputBGRImage.type() == CV_8UC3 ); // make sure we are dealing with a colour image

         printf("Click on a sample point in the input image.\n");
         printf("When finished with this image, press any key to continue ...\n");

         /* Create a window for input and display it */
         namedWindow(input_window_name, CV_WINDOW_AUTOSIZE );
         setMouseCallback(input_window_name, getSamplePoint);    // use this callback to get the colour components of the sample point
         imshow(input_window_name, inputBGRImage);
  
         /* convert the BGR image to HLS to facilitate hue-saturation segmentation */
         cvtColor(inputBGRImage, inputHLSImage, CV_BGR2HLS);

         /* Create a window for segmentation based on hue and saturation thresholding */
         namedWindow(segmented_window_name, CV_WINDOW_AUTOSIZE );
         resizeWindow(segmented_window_name,0,0); // this forces the trackbar to be as small as possible (and to fit in the window)
         createTrackbar( "Hue Range", segmented_window_name, &hueRange,        max_hue_range,        colourSegmentation);
         createTrackbar( "Sat Range", segmented_window_name, &saturationRange, max_saturation_range, colourSegmentation);

         /* display a zero output */
         outputImage = Mat::zeros(inputBGRImage.rows, inputBGRImage.cols, inputBGRImage.type()); 
         imshow(segmented_window_name, outputImage); 

         /* now wait for user interaction - mouse click to change the colour sample or trackbar adjustment to change the thresholds */
         number_of_sample_points = 0;
         do {
            waitKey(30);                             
         } while (!_kbhit());      
         
         getchar(); // flush the buffer from the keyboard hit

         destroyWindow(input_window_name);  
         destroyWindow(segmented_window_name); 
      }
   } while (end_of_file != EOF);

   fclose(fp_in);
   
   // end raw mode
   endwin();
   return 0;
}

