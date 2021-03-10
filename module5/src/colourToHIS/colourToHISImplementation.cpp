/* 
  Example use of openCV to convert a colour image to hue, intensity, and saturation
  ---------------------------------------------------------------------------------
  
  (This is the implementation file: it contains the code for dedicated functions to implement the application.
  These functions are called by client code in the application file. The functions are declared in the interface file.) 

  David Vernon
  24 November 2017

  Audit Trail
  --------------------
  Added _kbhit
  18 February 2021
    
*/
 
#include "module5/colourToHIS.h"
 
void colourToHIS(char *filename) {
  
   char inputWindowName[MAX_STRING_LENGTH]         = "Input Image";
   char hueWindowName[MAX_STRING_LENGTH]           = "Hue Image";
   char intensityWindowName[MAX_STRING_LENGTH]     = "Intensity Image";
   char saturationWindowName[MAX_STRING_LENGTH]    = "Saturation Image";

   Mat colourImage;
   Mat hueImage;   
   Mat intensityImage;  
   Mat saturationImage;

   int row;
   int col;
   unsigned char red;
   unsigned char green;
   unsigned char blue;
   float hue;
   float saturation;
   float intensity;

   namedWindow(inputWindowName,      CV_WINDOW_AUTOSIZE);  
   namedWindow(hueWindowName,        CV_WINDOW_AUTOSIZE);
   namedWindow(intensityWindowName,  CV_WINDOW_AUTOSIZE);
   namedWindow(saturationWindowName, CV_WINDOW_AUTOSIZE);

   colourImage = imread(filename, CV_LOAD_IMAGE_COLOR); // Read the file

   if (!colourImage.data) {                            // Check for invalid input
      printf("Error: failed to read image %s\n",filename);
      prompt_and_exit(-1);
   }

   printf("Press any key to continue ...\n");

   imshow(inputWindowName, colourImage );        
  
   CV_Assert(colourImage.type() == CV_8UC3 );

   // convert to HIS by explicit access to colour image pixels
   // we do this simply as an example of one way to access individual pixels
   // see changeQuantisation() for a more efficient method that accesses pixel using pointers
  
   hueImage.create(colourImage.size(), CV_8UC1);
   saturationImage.create(colourImage.size(), CV_8UC1);
   intensityImage.create(colourImage.size(), CV_8UC1);

	for (row=0; row < colourImage.rows; row++) {
		for (col=0; col < colourImage.cols; col++) {
	      blue  = colourImage.at<Vec3b>(row,col)[0];
         green = colourImage.at<Vec3b>(row,col)[1];
         red   = colourImage.at<Vec3b>(row,col)[2];

         rgb2hsi(red, green, blue, &hue, &saturation, &intensity);
         hueImage.at<uchar>(row,col)        = (char)  (255.0  * (hue/360.0));
         saturationImage.at<uchar>(row,col) = (char)  (saturation * 255);
         intensityImage.at<uchar>(row,col)  = (char)  (intensity * 255);
 
      }
   }
 
   imshow(hueWindowName,       hueImage); 
   imshow(intensityWindowName, intensityImage); 
   imshow(saturationWindowName,saturationImage); 

   do{
      waitKey(30);                                  // Must call this to allow openCV to display the images
   } while (!_kbhit());                             // We call it repeatedly to allow the user to move the windows
                                                    // (if we don't the window process hangs when you try to click and drag

   getchar(); // flush the buffer from the keyboard hit

   destroyWindow(inputWindowName);  
   destroyWindow(hueWindowName); 
   destroyWindow(intensityWindowName); 
   destroyWindow(saturationWindowName); 
}
 

// -----------------------------------------------------------------------------------------------
// rgb2hsi
//
// convert an RGB triple to a HSI triple
//
// The transform is based on "The Taming of the Hue, Saturation and Brightness Colour Space", Allan Hanbury, Proc. CVWW, [Hanbury02]
// 
// -----------------------------------------------------------------------------------------------

void rgb2hsi(unsigned char red, unsigned char green, unsigned char blue, float *hue, float *saturation, float *intensity){

	double y, h, h_star, c, c1, c2,  s, r, g, b; 
   int min =256;

   //  0 <= hue <= 2 pi
   //  0 <= saturation <= 1

   r = (float) red   / 256;
   g = (float) green / 256;
   b = (float) blue  / 256;

   y  = 0.2125 * r + 0.7154 * g + 0.0721 * b;
   c1 =          r - 0.5    * g - 0.5    * b;
   c2 =            - 0.8660 * g + 0.8660 * b;


   // chroma c: [0,1]

   c = sqrt(c1*c1 + c2*c2);


   // hue h: [0,360]

   if (c == 0) { // h and s are undefined
      *hue        = (float) 0;
      *saturation = (float) 0;
   }
   else {
      if(c2 <= 0) {
         h = acos (c1/c);
      }
      else {
         h = 2*3.14159  - acos (c1/c);
      }

      h = 360 * (h / (2 * 3.14159)); // convert to degrees


      // saturation: [0,1]

      h_star =  (int) h - (int) (60 * (  ((int) h) / 60));  // convert to interval 0,60


      s = (2 * c * sin( 2 * 3.14159 * ((120 - h_star) / 360.0))) / 1.73205;


      //*hue        = (float)  ((h / 360) * 2 * 3.14159); // convert to radians ... for the moment anyway
      *hue        = (float)  h;  
      *saturation = (float)  s;
   }

 	*intensity  = (float)  (r+g+b)/3;

  // printf("rgb2hsi: (%d, %d, %d) -> (%3.1f, %3.1f, %3.1f)\n", red, green, blue, *hue, *saturation, *intensity);

}

/*=======================================================*/
/* Utility functions to prompt user to continue          */ 
/*=======================================================*/

void prompt_and_exit(int status) {
   printf("Press any key to continue and close terminal ... \n");
   getchar();
   
   #ifdef ROS
      endwin();
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