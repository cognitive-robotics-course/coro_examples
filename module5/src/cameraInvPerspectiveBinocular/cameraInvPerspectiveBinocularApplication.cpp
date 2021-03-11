/* 
  Example use of openCV to compute the inverse perspective transformation from a pair of camera models
  ----------------------------------------------------------------------------------------------------
   
  This application reads four lines from an input file inversePerspectiveInput.txt.
  Each line comprise a filename for files containing, respectively:

  1. The camera model for the left camera
  2. The camera model for the right camera
  3. An image from the left camera
  4. An image from the right camera

  It is assumed that the input file is located in a data directory given by the path ../data/ 
  defined relative to the location of executable for this application.

  It is also assumed that the images are in a data directory given by the path ../data/Media/
  However, the camera model files are in a data directory given by the path ../data/ 

  After computing the inverse perspective transformation, the user can then interactively select a point in the left image
  and a corresponding point in the right image.  The application then uses the inverse perspective transformation to 
  compute the world x, y, and z coordinates of the selected point.
 



  NB: THIS APPLICATION HAS NOT BEEN TESTED WITH CORRECTLY CALIBRATED CAMERA MODELS



  (This is the application file: it contains the client code that calls dedicated functions to implement the application.
  The code for these functions is defined in the implementation file. The functions are declared in the interface file.)

  David Vernon
  2  April 2018
*/

 
#include "module5/cameraInvPerspectiveBinocular.h"

// Global variables to allow access by the display window callback functions

Point2f left_sample_point; 
Point2f right_sample_point; 
int number_of_left_sample_points;
int number_of_right_sample_points;
Mat leftImage;
Mat rightImage;

const char* left_window_name       = "Left Image";
const char* right_window_name      = "Right Image";

int main() {
   
   #ifdef ROS
      // Turn off canonical terminal mode and character echoing
      static const int STDIN = 0;
      termios term, old_term;
      tcgetattr(STDIN, &old_term);
      tcgetattr(STDIN, &term);
      term.c_lflag &= ~(ICANON | ECHO);
      tcsetattr(STDIN, TCSANOW, &term);
   #endif 
    
   const char input_filename[MAX_FILENAME_LENGTH] = "cameraInvPerspectiveBinocularInput.txt";    
   char input_path_and_filename[MAX_FILENAME_LENGTH];    
   char data_dir[MAX_FILENAME_LENGTH];
   char file_path_and_filename[MAX_FILENAME_LENGTH];
     

   int end_of_file;
   bool debug = false;
   char left_camera_model_filename[MAX_FILENAME_LENGTH];
   char right_camera_model_filename[MAX_FILENAME_LENGTH];
   char left_image_filename[MAX_FILENAME_LENGTH];
   char right_image_filename[MAX_FILENAME_LENGTH];

   int i, j;

   Mat leftImageCopy;
   Mat rightImageCopy;

   Point3f world_sample_point;
   Point2f text_coordinates; 

   Scalar colour(0,255,0);

   char coordinates[20];

   FILE *fp_in;
   FILE *fp_left_camera_model;
   FILE *fp_right_camera_model;

   float    left_camera_model[3][4];
   float    right_camera_model[3][4];

   printf("Example of how to use openCV to compute the inverse perspective transformation.\n");
   printf("Click on a point in the left image and click on the corresponding point in the right image.\n\n");   
   printf("Press any key to finish ...\n\n");


   
   #ifdef ROS   
      strcpy(data_dir, ros::package::getPath(ROS_PACKAGE_NAME).c_str()); // get the package directory
   #else
      strcpy(data_dir, "..");
   #endif
   
   strcat(data_dir, "/data/");
   strcpy(input_path_and_filename, data_dir);
   strcat(input_path_and_filename, input_filename);
   

   if ((fp_in = fopen(input_path_and_filename,"r")) == 0) {
	  printf("Fatal error can't open input cameraInvPerspectiveBinocularInput.txt\n");
     prompt_and_exit(1);
   }

 
   end_of_file = fscanf(fp_in, "%s", left_camera_model_filename);
   if (end_of_file == EOF) {
      printf("Fatal error: unable to read left camera model filename\n");
     prompt_and_exit(1);
   }

   end_of_file = fscanf(fp_in, "%s", right_camera_model_filename);
   if (end_of_file == EOF) {
      printf("Fatal error: unable to read right camera model filename\n");
     prompt_and_exit(1);
   }

   end_of_file = fscanf(fp_in, "%s", left_image_filename);
   if (end_of_file == EOF) {
      printf("Fatal error: unable to read left image filename\n");
     prompt_and_exit(1);
   }

   end_of_file = fscanf(fp_in, "%s", right_image_filename);
   if (end_of_file == EOF) {
      printf("Fatal error: unable to read right image filename\n");
      prompt_and_exit(1);
   }

   /* get the left and right camera models */
   strcpy(file_path_and_filename, data_dir);
   strcat(file_path_and_filename, left_camera_model_filename);
   strcpy(left_camera_model_filename, file_path_and_filename);

   if ((fp_left_camera_model = fopen(left_camera_model_filename,"r")) == 0) {
	   printf("Error can't open left camera model for input %s\n",left_camera_model_filename);
      prompt_and_exit(1);
   }
    
   for (i=0; i<3; i++) {
      for (j=0; j<4; j++) {
         fscanf(fp_left_camera_model, "%f ", &(left_camera_model[i][j]));
      }
   }

   strcpy(file_path_and_filename, data_dir);
   strcat(file_path_and_filename, right_camera_model_filename);
   strcpy(right_camera_model_filename, file_path_and_filename);

   if ((fp_right_camera_model = fopen(right_camera_model_filename,"r")) == 0) {
	   printf("Error can't open right camera model for input %s\n",right_camera_model_filename);
      prompt_and_exit(1);
   }
    
   for (i=0; i<3; i++) {
      for (j=0; j<4; j++) {
         fscanf(fp_right_camera_model, "%f ", &(right_camera_model[i][j]));
      }
   }

   /* get the left and right images */
   strcpy(file_path_and_filename, data_dir);
   strcat(file_path_and_filename, left_image_filename);
   strcpy(left_image_filename, file_path_and_filename);

   leftImage = imread(left_image_filename, CV_LOAD_IMAGE_UNCHANGED);
   if (leftImage.empty()) {
      cout << "can not open " << left_image_filename << endl;
      prompt_and_exit(-1);
   }

   strcpy(file_path_and_filename, data_dir);
   strcat(file_path_and_filename, right_image_filename);
   strcpy(right_image_filename, file_path_and_filename);

   rightImage = imread(right_image_filename, CV_LOAD_IMAGE_UNCHANGED);
   if (rightImage.empty()) {
      cout << "can not open " << right_image_filename << endl;
      prompt_and_exit(-1);
   }

   /* Create a window for left and display it */
   namedWindow(left_window_name, CV_WINDOW_AUTOSIZE );
   setMouseCallback(left_window_name, getLeftSamplePoint);    // use this callback to get the coordinates of the sample point
   imshow(left_window_name, leftImage);
  
   /* Create a window for right and display it */
   namedWindow(right_window_name, CV_WINDOW_AUTOSIZE );
   setMouseCallback(right_window_name, getRightSamplePoint);  // use this callback to get the coordinates of the sample point
   imshow(right_window_name, rightImage);

   /* now wait for user interaction - mouse click of left and right images */
   number_of_left_sample_points = 0;
   number_of_right_sample_points = 0;
   do {
      waitKey(30);   
      if (number_of_left_sample_points == 1 && number_of_right_sample_points == 1) {
                                                         
         inversePerspectiveTransformation(left_sample_point, right_sample_point, left_camera_model, right_camera_model, &world_sample_point);

         leftImageCopy  = leftImage.clone();
         rightImageCopy = rightImage.clone();

         sprintf(coordinates, "+ (%3.1f, %3.1f, %3.1f)", world_sample_point.x, world_sample_point.y, world_sample_point.z);

         text_coordinates.x = left_sample_point.x-7; // offset the graphic text message so that the + character is centred on the image sample point
         text_coordinates.y = left_sample_point.y+4;
         putText(leftImageCopy,  coordinates, text_coordinates, FONT_HERSHEY_SIMPLEX, 0.5, colour, 1 );

         text_coordinates.x = right_sample_point.x-7; // offset the graphic text message so that the + character is centred on the image sample point
         text_coordinates.y = right_sample_point.y+4;
         putText(rightImageCopy, coordinates, text_coordinates, FONT_HERSHEY_SIMPLEX, 0.5, colour, 1 );

         imshow(left_window_name, leftImageCopy);
         imshow(right_window_name, rightImageCopy);

         number_of_left_sample_points =  0;  // reset to allow another sample
         number_of_right_sample_points = 0; 
      }
   } while (!_kbhit());      
         

   destroyWindow(left_window_name);  
   destroyWindow(right_window_name); 

   fclose(fp_in);
   fclose(fp_left_camera_model);
   fclose(fp_right_camera_model);

   exit(0);

}

