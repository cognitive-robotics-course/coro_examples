/* 
  Example use of openCV to compute the inverse perspective transformation from a camera model
  -------------------------------------------------------------------------------------------
   
  This application reads two lines from an input file cameraInvPerspectiveMonocular.txt.
  Each line comprise a filename for files containing, respectively:

  1. The camera model for the camera
  2. An image from the camera

  It is assumed that the input file is located in a data directory given by the path ../data/ 
  defined relative to the location of executable for this application.

  It is also assumed that the images are in a data directory given by the path ../data/Media/
  However, the camera model files are in a data directory given by the path ../data/ 

  After computing the inverse perspective transformation, the user can then interactively select a point in the image.
  The application then uses the inverse perspective transformation to compute the world x and y coordinates of the selected point.
  It assumes the z coordinate is zero.
 
 (This is the application file: it contains the client code that calls dedicated functions to implement the application.
  The code for these functions is defined in the implementation file. The functions are declared in the interface file.)

  David Vernon
  14 June 2018
*/

 
#include "module5/cameraInvPerspectiveMonocular.h"

// Global variables to allow access by the display window callback functions

Point2f image_sample_point; 
int number_of_sample_points;
Mat image;

const char* window_name       = "Image";

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
    
   const char input_filename[MAX_FILENAME_LENGTH] = "cameraInvPerspectiveMonocularInput.txt";    
   char input_path_and_filename[MAX_FILENAME_LENGTH];    
   char data_dir[MAX_FILENAME_LENGTH];
   char file_path_and_filename[MAX_FILENAME_LENGTH];
     

   int end_of_file;
   bool debug = false;
   char camera_model_filename[MAX_FILENAME_LENGTH];
   char image_filename[MAX_FILENAME_LENGTH];

   int i, j;
   float z;

   Mat imageCopy;

   Point3f world_sample_point;
   Point2f text_coordinates; 

   Scalar colour(0,255,0);

   char coordinates[20];

   FILE *fp_in;
   FILE *fp_camera_model;

   float    camera_model[3][4];

   printf("Example of how to use openCV to compute the inverse perspective transformation.\n");
   printf("Click on a point in the image to compute the world coordinates.\n\n");   
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
	  printf("Fatal error can't open input cameraInvPerspectiveMonocularInput.txt\n");
     prompt_and_exit(1);
   }
 
   end_of_file = fscanf(fp_in, "%s", camera_model_filename);
   if (end_of_file == EOF) {
      printf("Fatal error: unable to read camera model filename\n");
     prompt_and_exit(1);
   }

   end_of_file = fscanf(fp_in, "%s", image_filename);
   if (end_of_file == EOF) {
      printf("Fatal error: unable to read image filename\n");
     prompt_and_exit(1);
   }

   /* get the left and right camera models */
   strcpy(file_path_and_filename, data_dir);
   strcat(file_path_and_filename, camera_model_filename);
   strcpy(camera_model_filename, file_path_and_filename);

   if ((fp_camera_model = fopen(camera_model_filename,"r")) == 0) {
	   printf("Error can't open camera model for input %s\n",camera_model_filename);
      prompt_and_exit(1);
   }
    
   for (i=0; i<3; i++) {
      for (j=0; j<4; j++) {
         fscanf(fp_camera_model, "%f ", &(camera_model[i][j]));
      }
   }


   /* get the image */
   strcpy(file_path_and_filename, data_dir);
   strcat(file_path_and_filename, image_filename);
   strcpy(image_filename, file_path_and_filename);

   image = imread(image_filename, IMREAD_UNCHANGED);
   if (image.empty()) {
      cout << "can not open " << image_filename << endl;
      prompt_and_exit(-1);
   }

   z = 0; // set the depth value for the inverse perspective transformation

   /* Create a window for image and display it */
   namedWindow(window_name, WINDOW_AUTOSIZE );
   setMouseCallback(window_name, getSamplePoint);    // use this callback to get the coordinates of the sample point
   imshow(window_name, image);

   /* now wait for user interaction - mouse click on image */
   number_of_sample_points = 0;
   do {
      waitKey(30);   
      if (number_of_sample_points == 1) {
                                                         
         inversePerspectiveTransformation(image_sample_point, camera_model, z, &world_sample_point);

         text_coordinates.x = image_sample_point.x-7; // offset the graphic text message so that the + character is centred on the image sample point
         text_coordinates.y = image_sample_point.y+4;

         imageCopy  = image.clone(); 
         sprintf(coordinates, "+ (%3.1f, %3.1f, %3.1f)", world_sample_point.x, world_sample_point.y, world_sample_point.z);
         putText(imageCopy,  coordinates, text_coordinates, FONT_HERSHEY_SIMPLEX, 0.5, colour, 1 );
         imshow(window_name, imageCopy);
         number_of_sample_points =  0;  // reset to allow another sample
      }
   } while (!_kbhit());      
         

   destroyWindow(window_name);  

   fclose(fp_in);
   fclose(fp_camera_model);
   #ifdef ROS
      // Reset terminal
      tcsetattr(STDIN, TCSANOW, &old_term);
   #endif

   exit(0);

}

