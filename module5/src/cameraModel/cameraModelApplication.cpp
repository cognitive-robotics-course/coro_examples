/* 
  Example use of openCV to compute the 3x4 camera model matrix
  ------------------------------------------------------------

  cameraModelApplication reads three filenames from an input file cameraModelInput.txt.

  The first filename is for a input file that contains the image control points. 
  The second filename is for a input file that contains the corresponding world control points. 
  The third filename is for an output file to which the twelve floating point elements of the camera model matrix are written.

  It is assumed that the input files are located in a data directory given by the path ../data/ 
  defined relative to the location of executable for this application.

  NB: use the cameraModelData application to acquire image control points which provide input to this application.
  The world control points are provided by other means, such as manual measurement, with or without 
  the aid of a robot to tie them to the robot's frame of reference.

  (This is the application file: it contains the client code that calls dedicated functions to implement the application.
  The code for these functions is defined in the implementation file. The functions are declared in the interface file.)

  David Vernon
  9 June 2018
*/
 
#include "cameraModel.h"

int main() {
   
   string                 path;
   string                 input_filename            = "cameraModelInput.txt";
   string                 input_path_and_filename;
   string                 data_dir;
   string                 datafile_path_and_filename;
   data_dir = ros::package::getPath(ROS_PACKAGE_NAME); // get the package directory
   data_dir += "/data/";
   input_path_and_filename = data_dir + input_filename;
     
   // Initialize screen in ncurses raw mode
   initscr(); 


   int end_of_file;
   bool debug = true;
   int i, j;
   double u, v, t;
   double x, y, z;
   char imageControlPointsPathAndFilename[MAX_FILENAME_LENGTH];
   char worldControlPointsPathAndFilename[MAX_FILENAME_LENGTH];
   char cameralModelPathAndFilename[MAX_FILENAME_LENGTH];
   char filename[MAX_FILENAME_LENGTH];
   FILE *fp_in;
   FILE *fp_image_control_points;
   FILE *fp_world_control_points;
   FILE *fp_camera_model;

   imagePointType imagePoints[MAX_NUMBER_OF_CONTROL_POINTS];
   worldPointType worldPoints[MAX_NUMBER_OF_CONTROL_POINTS];
   double        cameraModel[3][4];
   int           numberOfImageControlPoints;
   int           numberOfWorldControlPoints;

   if ((fp_in = fopen(input_path_and_filename.c_str(),"r")) == 0) {
	  printf("Error can't open input cameraModelInput.txt\n");
     prompt_and_exit(1);
   }

   printf("Computing the camera model\n\n");
   
   end_of_file = fscanf(fp_in, "%s", filename);
      
   if (end_of_file != EOF) {
      strcpy(imageControlPointsPathAndFilename, data_dir.c_str());
      strcat(imageControlPointsPathAndFilename, filename);

      if (debug) {
         printf ("%s\n", imageControlPointsPathAndFilename);
      }

      end_of_file = fscanf(fp_in, "%s", filename);
      
      if (end_of_file != EOF) {
         strcpy(worldControlPointsPathAndFilename, data_dir.c_str());
         strcat(worldControlPointsPathAndFilename, filename);

         if (debug) {
            printf ("%s\n", worldControlPointsPathAndFilename);
         }

         end_of_file = fscanf(fp_in, "%s", filename);

         if (end_of_file != EOF) {
            strcpy(cameralModelPathAndFilename, data_dir.c_str());
            strcat(cameralModelPathAndFilename, filename);

            if (debug) {
               printf ("%s\n", cameralModelPathAndFilename);
            }

            /* read the image control points */

            if ((fp_image_control_points = fopen(imageControlPointsPathAndFilename, "r")) == 0) {
	            printf("Error can't open input %s\n", imageControlPointsPathAndFilename);
               prompt_and_exit(1);
            }

            if ((fp_world_control_points = fopen(worldControlPointsPathAndFilename, "r")) == 0) {
	            printf("Error can't open input %s\n", worldControlPointsPathAndFilename);
               prompt_and_exit(1);
            }

            numberOfImageControlPoints = 0;
            do {
               end_of_file = fscanf(fp_image_control_points, "%d %d", &(imagePoints[numberOfImageControlPoints].u), &(imagePoints[numberOfImageControlPoints].v));
               if (end_of_file != EOF) numberOfImageControlPoints++;
            } while (end_of_file != EOF);
                  
            numberOfWorldControlPoints = 0;
            do {
               end_of_file = fscanf(fp_world_control_points, "%f %f %f", &(worldPoints[numberOfWorldControlPoints].x), &(worldPoints[numberOfWorldControlPoints].y), &(worldPoints[numberOfWorldControlPoints].z));
               if (end_of_file != EOF) numberOfWorldControlPoints++;
            } while (end_of_file != EOF);
                

            if (debug) {
               printf("Number of image control points %d\n", numberOfImageControlPoints);
               for (i=0; i<numberOfImageControlPoints; i++) {
                  printf("%d %d \n", imagePoints[i].u, imagePoints[i].v);
               }
               printf("\n");

               printf("Number of world control points %d\n", numberOfWorldControlPoints);
               for (i=0; i<numberOfWorldControlPoints; i++) {
                  printf("%f %f %f \n", worldPoints[i].x, worldPoints[i].y, worldPoints[i].z);
               }
               printf("\n");
            }

            if (numberOfImageControlPoints != numberOfWorldControlPoints) {
               printf("Fatal error: number of image and world control points is not same\n");
               prompt_and_exit(0);
            }
            else {
               
               if (debug) printf("\nComputing camera model ... \n\n");

               computeCameraModel(numberOfImageControlPoints, worldPoints, imagePoints, cameraModel);
  
               /* check result */

               if (debug) {
                  printf("Validation:\n");

                  z= worldPoints[0].z;
                  for (x = worldPoints[0].x; x < worldPoints[numberOfWorldControlPoints-1].x; x+=10) {
                     
                     //for (y = worldPoints[0].y; y < worldPoints[numberOfWorldControlPoints-1].y; y+=20) {
                     for (y = worldPoints[numberOfWorldControlPoints-1].y; y < worldPoints[0].y; y+=20) { // control point 0 has maximum y; last point has minimum y
                        u = cameraModel[0][0]*x + cameraModel[0][1]*y + cameraModel[0][2]*z + cameraModel[0][3]*(float)1.0;
                        v = cameraModel[1][0]*x + cameraModel[1][1]*y + cameraModel[1][2]*z + cameraModel[1][3]*(float)1.0;
                        t = cameraModel[2][0]*x + cameraModel[2][1]*y + cameraModel[2][2]*z + cameraModel[2][3]*(float)1.0;
                        printf("(%4.1f %4.1f %4.1f) -> (%4.1f %4.1f)\n", x,  y,  z,  u/t, v/t );
                     }
                  }
                  printf("\n");

                  for (i=0; i<numberOfImageControlPoints; i++) {
                     printf("Actual:  (%4.1f %4.1f %4.1f) -> (%4d %4d)\n", worldPoints[i].x,  worldPoints[i].y,  worldPoints[i].z, imagePoints[i].u, imagePoints[i].v);
                     u = cameraModel[0][0]*worldPoints[i].x + cameraModel[0][1]*worldPoints[i].y + cameraModel[0][2]*worldPoints[i].z + cameraModel[0][3]*(float)1.0;
                     v = cameraModel[1][0]*worldPoints[i].x + cameraModel[1][1]*worldPoints[i].y + cameraModel[1][2]*worldPoints[i].z + cameraModel[1][3]*(float)1.0;
                     t = cameraModel[2][0]*worldPoints[i].x + cameraModel[2][1]*worldPoints[i].y + cameraModel[2][2]*worldPoints[i].z + cameraModel[2][3]*(float)1.0;
                     printf("Computed:(%4.1f %4.1f %4.1f) -> (%4.1f %4.1f)\n", worldPoints[i].x,  worldPoints[i].y,  worldPoints[i].z, u/t, v/t );
                  }
                  printf("\n");
               }  

               /* write out the camera model */

               if (debug) {
                  for (i=0; i<3; i++) {
                     for (j=0; j<4; j++) {
                        printf("%f ", cameraModel[i][j]);
                     }
                     printf("\n");
                  }
               }


               if ((fp_camera_model = fopen(cameralModelPathAndFilename, "w")) == 0) {
	               printf("Error can't open output %s\n", cameralModelPathAndFilename);
                  prompt_and_exit(1);
               }

               for (i=0; i<3; i++) {
                  for (j=0; j<4; j++) {
                     fprintf(fp_camera_model, "%f ", cameraModel[i][j]);
                  }
                  fprintf(fp_camera_model,"\n");
               }
            }
         }
      }
   }

   fclose(fp_in); 
   
   if (debug) prompt_and_continue();

   // end raw mode
   endwin();
   return 0;
}