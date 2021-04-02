/* 
  Example use of openCV to collect image control points for computing the camera model
  ------------------------------------------------------------------------------------

  (This is the implementation file: it contains the code for dedicated functions to implement the application.
  These functions are called by client code in the application file. The functions are declared in the interface file.) 

  David Vernon
  29 March 2018

  AUDIT TRAIL
  -------------------
  Added the functions spawn_checkerboard, delete_checkerboard, fsize, imageMessageReceived and writeWorldCoordinatesToFile.
  Added new parameters to getImageControlPoints to allow world coordinate generation for control points.
  Abrham Gebreselasie
  13 March 2021


*/
 
#include "module5/cameraModelDataSimulator.h"
 
/*
 * This camera calibration software has been derived from software provided
 * as part of OpenCV 2.4.5 documentation.  It was downloaded from
 * http://docs.opencv.org/doc/tutorials/calib3d/camera_calibration/camera_calibration.html?highlight=camera%20calibration
 *
 * This version of the code is provided as part of
 * "A Practical Introduction to Computer Vision with OpenCV"
 * by Kenneth Dawson-Howe � Wiley & Sons Inc. 2014.  All rights reserved.
 */
#include <iostream>
#include <sstream>
#include <time.h>
#include <stdio.h>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>

using namespace cv;
using namespace std;

extern Mat scene_image;
extern int imageCount;

class Settings
{
public:
    Settings() : goodInput(false) {}
    enum Pattern { NOT_EXISTING, CHESSBOARD, CIRCLES_GRID, ASYMMETRIC_CIRCLES_GRID };
    enum InputType {INVALID, CAMERA, VIDEO_FILE, IMAGE_LIST};

    void write(FileStorage& fs) const                        //Write serialization for this class
    {
        fs << "{" << "BoardSize_Width"  << boardSize.width
                  << "BoardSize_Height" << boardSize.height
                  << "Square_Size"         << squareSize
                  << "Calibrate_Pattern" << patternToUse
                  << "Calibrate_NrOfFrameToUse" << nrFrames
                  << "Calibrate_FixAspectRatio" << aspectRatio
                  << "Calibrate_AssumeZeroTangentialDistortion" << calibZeroTangentDist
                  << "Calibrate_FixPrincipalPointAtTheCenter" << calibFixPrincipalPoint

                  << "Write_DetectedFeaturePoints" << bwritePoints
                  << "Write_extrinsicParameters"   << bwriteExtrinsics
                  << "Write_outputFileName"  << outputFileName

                  << "Show_UndistortedImage" << showUndistorsed

                  << "Input_FlipAroundHorizontalAxis" << flipVertical
                  << "Input_Delay" << delay
                  << "Input" << input
           << "}";
    }
    void read(const FileNode& node)                          //Read serialization for this class
    {
        node["BoardSize_Width" ] >> boardSize.width;
        node["BoardSize_Height"] >> boardSize.height;
        node["Calibrate_Pattern"] >> patternToUse;
        node["Square_Size"]  >> squareSize;
        node["Calibrate_NrOfFrameToUse"] >> nrFrames;
        node["Calibrate_FixAspectRatio"] >> aspectRatio;
        node["Write_DetectedFeaturePoints"] >> bwritePoints;
        node["Write_extrinsicParameters"] >> bwriteExtrinsics;
        node["Write_outputFileName"] >> outputFileName;
        node["Calibrate_AssumeZeroTangentialDistortion"] >> calibZeroTangentDist;
        node["Calibrate_FixPrincipalPointAtTheCenter"] >> calibFixPrincipalPoint;
        node["Input_FlipAroundHorizontalAxis"] >> flipVertical;
        node["Show_UndistortedImage"] >> showUndistorsed;
        node["Input"] >> input;
        node["Input_Delay"] >> delay;
        interprate();
    }
    void interprate()
    {
        goodInput = true;
        if (boardSize.width <= 0 || boardSize.height <= 0)
        {
            cerr << "Invalid Board size: " << boardSize.width << " " << boardSize.height << endl;
            goodInput = false;
        }
        if (squareSize <= 10e-6)
        {
            cerr << "Invalid square size " << squareSize << endl;
            goodInput = false;
        }
        if (nrFrames <= 0)
        {
            cerr << "Invalid number of frames " << nrFrames << endl;
            goodInput = false;
        }

        if (input.empty())      // Check for valid input
                inputType = INVALID;
        else
        {
            if (input[0] >= '0' && input[0] <= '9')
            {
                stringstream ss(input);
                ss >> cameraID;
                inputType = CAMERA;
            }
            else
            {
                if (readStringList(input, imageList))
                    {
                        printf("Read imagelist file\n");
                        inputType = IMAGE_LIST;
                        nrFrames = (nrFrames < (int)imageList.size()) ? nrFrames : (int)imageList.size();
                    }
                else
                    inputType = VIDEO_FILE;
            }
            if (inputType == CAMERA)
			{
                inputCapture.open(cameraID);
				inputCapture.set(CV_CAP_PROP_FRAME_WIDTH, 640);
				inputCapture.set(CV_CAP_PROP_FRAME_HEIGHT, 480);
			}
            if (inputType == VIDEO_FILE)
                inputCapture.open(input);
            if (inputType != IMAGE_LIST && !inputCapture.isOpened())
                    inputType = INVALID;
        }
        if (inputType == INVALID)
        {
            cerr << " Inexistent input: " << input;
            goodInput = false;
        }

        flag = 0;
        if(calibFixPrincipalPoint) flag |= CV_CALIB_FIX_PRINCIPAL_POINT;
        if(calibZeroTangentDist)   flag |= CV_CALIB_ZERO_TANGENT_DIST;
        if(aspectRatio)            flag |= CV_CALIB_FIX_ASPECT_RATIO;


        calibrationPattern = NOT_EXISTING;
        if (!patternToUse.compare("CHESSBOARD")) calibrationPattern = CHESSBOARD;
        if (!patternToUse.compare("CIRCLES_GRID")) calibrationPattern = CIRCLES_GRID;
        if (!patternToUse.compare("ASYMMETRIC_CIRCLES_GRID")) calibrationPattern = ASYMMETRIC_CIRCLES_GRID;
        if (calibrationPattern == NOT_EXISTING)
            {
                cerr << " Inexistent camera calibration mode: " << patternToUse << endl;
                goodInput = false;
            }
        atImageList = 0;

    }
    Mat nextImage()
    {
        // Use absolute path instead of relative path on ROS
        string path_and_filename;
        string data_dir;
        data_dir = ros::package::getPath(ROS_PACKAGE_NAME); // get the package directory
        data_dir += "/data/";

        Mat result;
        if( inputCapture.isOpened() )
        {
            Mat view0;
            inputCapture >> view0;
            view0.copyTo(result);
        }
        else if( atImageList < (int)imageList.size() )
        {
            path_and_filename = data_dir + imageList[atImageList];
            atImageList++;
            result = imread(path_and_filename, CV_LOAD_IMAGE_COLOR);
        }

        return result;
    }

    static bool readStringList( const string& filename, vector<string>& l )
    {
        l.clear();
        FileStorage fs(filename, FileStorage::READ);
        if( !fs.isOpened() )
            return false;
        FileNode n = fs.getFirstTopLevelNode();
        if( n.type() != FileNode::SEQ )
            return false;
        FileNodeIterator it = n.begin(), it_end = n.end();
        for( ; it != it_end; ++it )
            l.push_back((string)*it);
        return true;
    }
public:
    Size boardSize;            // The size of the board -> Number of items by width and height
    Pattern calibrationPattern;// One of the Chessboard, circles, or asymmetric circle pattern
    float squareSize;          // The size of a square in your defined unit (point, millimeter,etc).
    int nrFrames;              // The number of frames to use from the input for calibration
    float aspectRatio;         // The aspect ratio
    int delay;                 // In case of a video input
    bool bwritePoints;         //  Write detected feature points
    bool bwriteExtrinsics;     // Write extrinsic parameters
    bool calibZeroTangentDist; // Assume zero tangential distortion
    bool calibFixPrincipalPoint;// Fix the principal point at the center
    bool flipVertical;          // Flip the captured images around the horizontal axis
    string outputFileName;      // The name of the file where to write
    bool showUndistorsed;       // Show undistorted images after calibration
    string input;               // The input ->



    int cameraID;
    vector<string> imageList;
    VideoCapture inputCapture;
    InputType inputType;
    bool goodInput;
    int flag;

private:
    string patternToUse;
    int atImageList;

};

static void read(const FileNode& node, Settings& x, const Settings& default_value = Settings())
{
    if(node.empty())
        x = default_value;
    else
        x.read(node);
}

enum { DETECTION = 0, CAPTURING = 1, CALIBRATED = 2 };

bool runCalibrationAndSave(Settings& s, Size imageSize, Mat&  cameraMatrix, Mat& distCoeffs,
                           vector<vector<Point2f> > imagePoints );

static double computeReprojectionErrors( const vector<vector<Point3f> >& objectPoints,
                                         const vector<vector<Point2f> >& imagePoints,
                                         const vector<Mat>& rvecs, const vector<Mat>& tvecs,
                                         const Mat& cameraMatrix , const Mat& distCoeffs,
                                         vector<float>& perViewErrors)
{
    vector<Point2f> imagePoints2;
    int i, totalPoints = 0;
    double totalErr = 0, err;
    perViewErrors.resize(objectPoints.size());

    for( i = 0; i < (int)objectPoints.size(); ++i )
    {
        projectPoints( Mat(objectPoints[i]), rvecs[i], tvecs[i], cameraMatrix,
                       distCoeffs, imagePoints2);
        err = norm(Mat(imagePoints[i]), Mat(imagePoints2), CV_L2);

        int n = (int)objectPoints[i].size();
        perViewErrors[i] = (float) std::sqrt(err*err/n);
        totalErr        += err*err;
        totalPoints     += n;
    }

    return std::sqrt(totalErr/totalPoints);
}

static void calcBoardCornerPositions(Size boardSize, float squareSize, vector<Point3f>& corners,
                                     Settings::Pattern patternType /*= Settings::CHESSBOARD*/)
{
    corners.clear();

    switch(patternType)
    {
    case Settings::CHESSBOARD:
    case Settings::CIRCLES_GRID:
        for( int i = 0; i < boardSize.height; ++i )
            for( int j = 0; j < boardSize.width; ++j )
                corners.push_back(Point3f(float( j*squareSize ), float( i*squareSize ), 0));
        break;

    case Settings::ASYMMETRIC_CIRCLES_GRID:
        for( int i = 0; i < boardSize.height; i++ )
            for( int j = 0; j < boardSize.width; j++ )
                corners.push_back(Point3f(float((2*j + i % 2)*squareSize), float(i*squareSize), 0));
        break;
    default:
        break;
    }
}

static bool runCalibration( Settings& s, Size& imageSize, Mat& cameraMatrix, Mat& distCoeffs,
                            vector<vector<Point2f> > imagePoints, vector<Mat>& rvecs, vector<Mat>& tvecs,
                            vector<float>& reprojErrs,  double& totalAvgErr)
{
    cameraMatrix = Mat::eye(3, 3, CV_64F);
    if( s.flag & CV_CALIB_FIX_ASPECT_RATIO )
        cameraMatrix.at<double>(0,0) = 1.0;

    distCoeffs = Mat::zeros(8, 1, CV_64F);

    vector<vector<Point3f> > objectPoints(1);
    calcBoardCornerPositions(s.boardSize, s.squareSize, objectPoints[0], s.calibrationPattern);

    objectPoints.resize(imagePoints.size(),objectPoints[0]);

    //Find intrinsic and extrinsic camera parameters
    
    double rms = calibrateCamera(objectPoints, imagePoints, imageSize, cameraMatrix,
                                 distCoeffs, rvecs, tvecs, s.flag|CV_CALIB_FIX_K4|CV_CALIB_FIX_K5);

    bool ok = checkRange(cameraMatrix) && checkRange(distCoeffs);

    totalAvgErr = computeReprojectionErrors(objectPoints, imagePoints,
                                             rvecs, tvecs, cameraMatrix, distCoeffs, reprojErrs);
   
    return ok;
}

// Print camera parameters to the output file
static void saveCameraParams( Settings& s, Size& imageSize, Mat& cameraMatrix, Mat& distCoeffs,
                              const vector<Mat>& rvecs, const vector<Mat>& tvecs,
                              const vector<float>& reprojErrs, const vector<vector<Point2f> >& imagePoints,
                              double totalAvgErr )
{
    FileStorage fs( s.outputFileName, FileStorage::WRITE );

    time_t tm;
    time( &tm );
    struct tm *t2 = localtime( &tm );
    char buf[1024];
    strftime( buf, sizeof(buf)-1, "%c", t2 );

    fs << "calibration_Time" << buf;

    if( !rvecs.empty() || !reprojErrs.empty() )
        fs << "nrOfFrames" << (int)std::max(rvecs.size(), reprojErrs.size());
    fs << "image_Width" << imageSize.width;
    fs << "image_Height" << imageSize.height;
    fs << "board_Width" << s.boardSize.width;
    fs << "board_Height" << s.boardSize.height;
    fs << "square_Size" << s.squareSize;

    if( s.flag & CV_CALIB_FIX_ASPECT_RATIO )
        fs << "FixAspectRatio" << s.aspectRatio;

    if( s.flag )
    {
        sprintf( buf, "flags: %s%s%s%s",
            s.flag & CV_CALIB_USE_INTRINSIC_GUESS ? " +use_intrinsic_guess" : "",
            s.flag & CV_CALIB_FIX_ASPECT_RATIO ? " +fix_aspectRatio" : "",
            s.flag & CV_CALIB_FIX_PRINCIPAL_POINT ? " +fix_principal_point" : "",
            s.flag & CV_CALIB_ZERO_TANGENT_DIST ? " +zero_tangent_dist" : "" );
        cvWriteComment( *fs, buf, 0 );

    }

    fs << "flagValue" << s.flag;

    fs << "Camera_Matrix" << cameraMatrix;
    fs << "Distortion_Coefficients" << distCoeffs;

    fs << "Avg_Reprojection_Error" << totalAvgErr;
    if( !reprojErrs.empty() )
        fs << "Per_View_Reprojection_Errors" << Mat(reprojErrs);

    if( !rvecs.empty() && !tvecs.empty() )
    {
        CV_Assert(rvecs[0].type() == tvecs[0].type());
        Mat bigmat((int)rvecs.size(), 6, rvecs[0].type());
        for( int i = 0; i < (int)rvecs.size(); i++ )
        {
            Mat r = bigmat(Range(i, i+1), Range(0,3));
            Mat t = bigmat(Range(i, i+1), Range(3,6));

            CV_Assert(rvecs[i].rows == 3 && rvecs[i].cols == 1);
            CV_Assert(tvecs[i].rows == 3 && tvecs[i].cols == 1);
            //*.t() is MatExpr (not Mat) so we can use assignment operator
            r = rvecs[i].t();
            t = tvecs[i].t();
        }
        cvWriteComment( *fs, "a set of 6-tuples (rotation vector + translation vector) for each view", 0 );
        fs << "Extrinsic_Parameters" << bigmat;
    }

    if( !imagePoints.empty() )
    {
        Mat imagePtMat((int)imagePoints.size(), (int)imagePoints[0].size(), CV_32FC2);
        for( int i = 0; i < (int)imagePoints.size(); i++ )
        {
            Mat r = imagePtMat.row(i).reshape(2, imagePtMat.cols);
            Mat imgpti(imagePoints[i]);
            imgpti.copyTo(r);
        }
        fs << "Image_points" << imagePtMat;
    }
}

bool runCalibrationAndSave(Settings& s, Size imageSize, Mat&  cameraMatrix, Mat& distCoeffs,vector<vector<Point2f> > imagePoints )
{
    vector<Mat> rvecs, tvecs;
    vector<float> reprojErrs;
    double totalAvgErr = 0;

    bool ok = runCalibration(s,imageSize, cameraMatrix, distCoeffs, imagePoints, rvecs, tvecs,
                             reprojErrs, totalAvgErr);
    cout << (ok ? "Calibration succeeded" : "Calibration failed")
         << ".  Average re-projection error = "  << totalAvgErr << endl;

    /*
    if( ok )
        saveCameraParams( s, imageSize, cameraMatrix, distCoeffs, rvecs ,tvecs, reprojErrs,
                            imagePoints, totalAvgErr);
    */
    return ok;
}


/* int CameraCalibration( string passed_settings_filename  ) */ // original function
int getImageControlPoints(string passed_settings_filename, int numberOfViews, int *numberOfControlPoints, imagePointType imagePointsArray[],
                            FILE* fp_world_points, float cameraX, float cameraY, float boardZ)
{

    bool debug = false; // David Vernon

    Settings s;
    FileStorage fs(passed_settings_filename, FileStorage::READ); // Read the settings

    if (!fs.isOpened())
    {
       cout << "Could not open the configuration file: \"" << passed_settings_filename << "\"" << endl;
       return -1;
    }
    fs["Settings"] >> s;
    fs.release();                                         // close Settings file

    if (!s.goodInput)
    {
        cout << "Invalid settings file detected.  Exiting calibration. " << endl;
        return -1;
    }

    vector<vector<Point2f> > imagePoints;
    Mat cameraMatrix, distCoeffs;
    Size imageSize;
    int mode = s.inputType == Settings::IMAGE_LIST ? CAPTURING : DETECTION;
    clock_t prevTimestamp = 0;
    const Scalar RED(0,0,255), GREEN(0,255,0);
    const char ESC_KEY = 27;

    Mat view;
    for(int i = 0;;++i)
    {
      bool blinkOutput = false;

      view = s.nextImage();

      //-----  If no more image, or got enough, then stop calibration and show result -------------
      if( mode == CAPTURING && imagePoints.size() >= (unsigned)s.nrFrames )
      {
          if( runCalibrationAndSave(s, imageSize,  cameraMatrix, distCoeffs, imagePoints))
              mode = CALIBRATED;
          else
              mode = DETECTION;
      }

      if(view.empty())          // If no more images then run calibration, save and stop loop.
      {
            if( imagePoints.size() > 0 )
                runCalibrationAndSave(s, imageSize,  cameraMatrix, distCoeffs, imagePoints);
            break;
      }


        imageSize = view.size();  // Format input image.
        if( s.flipVertical )    flip( view, view, 0 );

        vector<Point2f> pointBuf;

        bool found;
        switch( s.calibrationPattern ) // Find feature points on the input format
        {
        case Settings::CHESSBOARD:
            found = findChessboardCorners( view, s.boardSize, pointBuf,
                CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);
            break;
        case Settings::CIRCLES_GRID:
            found = findCirclesGrid( view, s.boardSize, pointBuf );
            break;
        case Settings::ASYMMETRIC_CIRCLES_GRID:
            found = findCirclesGrid( view, s.boardSize, pointBuf, CALIB_CB_ASYMMETRIC_GRID );
            break;
        default:
            found = false;
            break;
        }

        if ( found )                // If done with success,
        {
              // improve the found corners' coordinate accuracy for chessboard
                if( s.calibrationPattern == Settings::CHESSBOARD)
                {
                    Mat viewGray;
                    cvtColor(view, viewGray, CV_BGR2GRAY);
                    cornerSubPix( viewGray, pointBuf, Size(11,11),
                        Size(-1,-1), TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ));
                }
                
                if( mode == CAPTURING &&  // For camera only take new samples after delay time
                    (!s.inputCapture.isOpened() || clock() - prevTimestamp > s.delay*1e-3*CLOCKS_PER_SEC) )
                {
                    imagePoints.push_back(pointBuf);
                    prevTimestamp = clock();
                    blinkOutput = s.inputCapture.isOpened();
                }

                // Draw the corners.
                drawChessboardCorners( view, s.boardSize, Mat(pointBuf), found );

                /* David Vernon .... extract the control point coordinates, display, and exit */

                if (mode == CAPTURING) { // g has been pressed
                                   
                   /* store the control point from the first view */

                   for (int i=0; i < (int) pointBuf.size(); i++) {
                      imagePointsArray[i].u = (int) pointBuf[i].x;
                      imagePointsArray[i].v = (int) pointBuf[i].y;
                      if (debug) printf("%d %d\n", imagePointsArray[i].u, imagePointsArray[i].v);
                   }
                   if (debug) printf("\n");
                   *numberOfControlPoints = (int) pointBuf.size();

                   imshow("Image View", view);
                   waitKey(2000);

                   if (numberOfViews == 1) {
                      /* job done */
                      break;
                   }
                   else {

                      /* prompt the user to say when the second view is ready */

                      printf("Press 'g' again to use the detected control points as the second view ... \r");

                      char key = (char) waitKey(s.delay);

                      if (tolower(key) == 'g') {
                         
                         /* store the control point from the first view */

                         for (int i=0; i < (int) pointBuf.size(); i++) {
                            imagePointsArray[i+(*numberOfControlPoints)].u = (int) pointBuf[i].x;
                            imagePointsArray[i+(*numberOfControlPoints)].v = (int) pointBuf[i].y;
                            if (debug) printf("%d %d\n", imagePointsArray[i+(*numberOfControlPoints)].u, imagePointsArray[i+(*numberOfControlPoints)].v);
                         }
                         if (debug) printf("\n");
                         *numberOfControlPoints = *numberOfControlPoints + (int) pointBuf.size();

                         imshow("Image View", view);
                         waitKey(50);

                         break;

                      }
                   }
                }


                /* David Vernon ... end of new code */
        }
 
        
        //----------------------------- Output Text ------------------------------------------------
        string msg = (mode == CAPTURING) ? "100/100" :
                      mode == CALIBRATED ? (s.showUndistorsed ? "Showing corrected image" : "Showing original image") :
					      "Press 'g' to start";
        int baseLine = 0;
        Size textSize = getTextSize(msg, 1, 1, 1, &baseLine);
        Point textOrigin(5, view.rows - 2*baseLine - 10);

        if( mode == CAPTURING )
        {
            if(s.showUndistorsed)
                msg = format( "%d/%d Undistorted", (int)imagePoints.size(), s.nrFrames );
            else
                msg = format( "%d/%d", (int)imagePoints.size(), s.nrFrames );
        }

        putText( view, msg, textOrigin, 1, 1, mode == CALIBRATED ?  GREEN : RED);

        if( blinkOutput )
            bitwise_not(view, view);
        

        
        //------------------------- Video capture  output  undistorted ------------------------------
        if( mode == CALIBRATED && s.showUndistorsed )
        {
            Mat temp = view.clone();
            undistort(temp, view, cameraMatrix, distCoeffs);
        }
        
        //------------------------------ Show image and check for input commands -------------------
        imshow("Image View", view);
       
        char key = (char)waitKey(s.inputCapture.isOpened() ? 50 : s.delay);

        if( key  == ESC_KEY )
            break;

        if( key == 'u' && mode == CALIBRATED )
           s.showUndistorsed = !s.showUndistorsed;

        if( s.inputCapture.isOpened() && key == 'g' )
        {
            mode = CAPTURING;
            imagePoints.clear();
        }


    }

  
 // -----------------------Show the undistorted image for the image list ------------------------
    if( s.inputType == Settings::IMAGE_LIST && s.showUndistorsed )
    {
        Mat view, rview, map1, map2;
        initUndistortRectifyMap(cameraMatrix, distCoeffs, Mat(),
            getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1, imageSize, 0),
            imageSize, CV_16SC2, map1, map2);

        for(int i = 0; i < (int)s.imageList.size(); i++ )
        {
            view = imread(s.imageList[i], 1);
            if(view.empty())
                continue;
            remap(view, rview, map1, map2, INTER_LINEAR);
            imshow("Image View", rview);
            char c = (char)waitKey();
            if( c  == ESC_KEY || c == 'q' || c == 'Q' )
                break;
        }
    }
    imshow("Image View", view);
    waitKey(2000);

    writeWorldCoordinatesToFile(fp_world_points, cameraX, cameraY, boardZ, s.squareSize, s.boardSize);
    return 0;
}


/* simulate the acquisition of the image control point coordinates by generating them from the perspective transformation */

void getSimulatedImageControlPoints(int numberOfControlPoints, float focalLength, worldPointType worldPoints[], imagePointType imagePoints[]) {
   int i; 
   bool debug = false;

   if (debug) {
      printf("number of control points %d\n", numberOfControlPoints);
   }

   for (i=0; i<numberOfControlPoints; i++) {
      imagePoints[i].u = (int) (focalLength * worldPoints[i].x / worldPoints[i].z);
      imagePoints[i].v = (int) (focalLength * worldPoints[i].y / worldPoints[i].z);

      if (debug) {
         printf("getSimulatedImageControlPoints: %4.1f %4.1f %4.1f -> %d %d\n", worldPoints[i].x, worldPoints[i].y, worldPoints[i].z, imagePoints[i].u, imagePoints[i].v);
      }
   }
 
}



/* simulate the acquisition of the world control point coordinates by generating them from the position, orientation, and geometry of the calibration grid */

void getSimulatedWorldControlPoints(int numberOfCornersWidth, int numberOfCornersHeight, int squareSize, 
                                    int numberOfViews, int originX, int originY, int originZ, 
                                    int *numberOfControlPoints, worldPointType worldPoints[]) {
            
   float offsetX;
   float angle;
   float angleOfIncline;
         
   int i, j; 
   int viewOffset;
   bool debug = true;
   int viewNumber;

   offsetX = (float) squareSize;

   for (viewNumber = 1; viewNumber <= numberOfViews; viewNumber++) {

      if (viewNumber == 1) {
         angleOfIncline = 0;
      }
      else {
         angleOfIncline = 45;
      }

      angle = ((float) CV_PI * (float) angleOfIncline)/ (float)  180.0;

      viewOffset = (viewNumber-1)*(numberOfCornersWidth*numberOfCornersHeight);

      worldPoints[0+viewOffset].x = (float) originX;
      worldPoints[0+viewOffset].y = (float) originY;
      worldPoints[0+viewOffset].z = (float) originZ;

      for (i = 1; i<numberOfCornersWidth; i++) {
         worldPoints[i+viewOffset].x = worldPoints[0+viewOffset].x + i * offsetX ;
         worldPoints[i+viewOffset].y = worldPoints[0+viewOffset].y;
         worldPoints[i+viewOffset].z = worldPoints[0+viewOffset].z;
      }

      for (j = 1; j<numberOfCornersHeight; j++) { 
         for (i = 0; i<numberOfCornersWidth; i++) { 
            worldPoints[viewOffset + i + j * numberOfCornersWidth].x = worldPoints[viewOffset + i + (j-1) * numberOfCornersWidth].x ;
            worldPoints[viewOffset + i + j * numberOfCornersWidth].y = worldPoints[viewOffset + i + (j-1) * numberOfCornersWidth].y +  squareSize * cos(angle);
            worldPoints[viewOffset + i + j * numberOfCornersWidth].z = worldPoints[viewOffset + i + (j-1) * numberOfCornersWidth].z +  squareSize * sin(angle);
         }
      }    

      if (debug) {

         for (j = 0; j<numberOfCornersHeight; j++) { 
            for (i = 0; i<numberOfCornersWidth; i++) { 
               printf("%4.1f %4.1f %4.1f \n", worldPoints[viewOffset + i + j * numberOfCornersWidth].x, 
                                              worldPoints[viewOffset + i + j * numberOfCornersWidth].y, 
                                              worldPoints[viewOffset + i + j * numberOfCornersWidth].z);
            }
            printf("\n");
         }
      }
   }   
   
   *numberOfControlPoints = numberOfViews * (numberOfCornersHeight * numberOfCornersWidth);
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

void spawn_checkerboard(const char* sdf_filename, float x, float y, float z, float pitch, float yaw, float roll)
{
    ros::NodeHandle nh;
    FILE* fp_sdf;
    int end_of_file;
    long num_bytes;
    char* sdf_content;

    fp_sdf = fopen(sdf_filename, "r");

    if (fp_sdf == NULL)
    {
        printf("Could not open %s\n", sdf_filename);
        prompt_and_exit(1);
    }
    num_bytes = fsize(fp_sdf);

    sdf_content = (char*) malloc(num_bytes + 1);
    fread(sdf_content, 1, num_bytes, fp_sdf);

    ros::ServiceClient gazebo_spawn_client = nh.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_sdf_model");
    gazebo_msgs::SpawnModel srv;

    srv.request.model_name = CHECKERBOARD_MODEL_NAME;
    srv.request.model_xml = string(sdf_content);
    srv.request.initial_pose.position.x = x;
    srv.request.initial_pose.position.y = y;
    srv.request.initial_pose.position.z = z;

    gazebo_spawn_client.call(srv);

    free(sdf_content);
}
void delete_checkerboard()
{
    ros::NodeHandle nh;
    ros::ServiceClient gazebo_delete_client = nh.serviceClient<gazebo_msgs::DeleteModel>("/gazebo/delete_model");
    gazebo_msgs::DeleteModel srv;

    srv.request.model_name = CHECKERBOARD_MODEL_NAME;

    gazebo_delete_client.call(srv);

}

size_t fsize(FILE *fp)
{
    size_t size;
    fseek(fp, 0, SEEK_END);
    size = ftell(fp);
    fseek(fp, 0, SEEK_SET);
    return size;
}

void imageMessageReceived(const sensor_msgs::ImageConstPtr& msg)
{

    string                 data_dir;
    char filename[MAX_FILENAME_LENGTH];
    cv_bridge::CvImagePtr cv_ptr;
    data_dir = ros::package::getPath(ROS_PACKAGE_NAME); // get the package directory
    data_dir += "/data/";
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    scene_image = cv_ptr->image;
    sprintf(filename, "%sMedia/%d.jpg", data_dir.c_str(), imageCount + 1);
    printf("Writing %s\n", filename);
    imwrite(filename, scene_image);
    imageCount++;
//    waitKey(500);
}

void writeWorldCoordinatesToFile(FILE *fp_world_points, float cameraX, float cameraY, float boardZ, float boxsize, Size size) {
    // Assumes the edges of the checkerboard are parallel to the x- and y- axes of the world coordinate frame
    int npointsX = size.width;
    int npointsY = size.height;

    float sizeX = (npointsX - 1) * boxsize; // size of the grid without the outer boxes
    float sizeY = (npointsY - 1) * boxsize;

    float topleftX = cameraX - sizeX / 2; // Only internal corners count actual top left not considered
    float topleftY = cameraY + sizeY / 2;

    for (int j = 0; j < npointsY; j++) {
        for (int i = 0; i < npointsX; i++) {
            // printf("%2d: (%3.4f, %3.4f)\n", i * (npointsY - 1) + j, topleftX + boxsize * i, topleftY + boxsize * j);
            fprintf(fp_world_points, "%3.4f %3.4f %3.4f\n", topleftX + boxsize * i, topleftY - boxsize * j, boardZ);
        }
    }
    //fprintf(fp_world_points, "\n");
}

