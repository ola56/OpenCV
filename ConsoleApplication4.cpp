
#include <opencv/cvaux.h>
#include <opencv/highgui.h>
#include <opencv/cxcore.h>
#include <stdio.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <math.h>
#include <float.h>
#include <limits.h>
#include <time.h>
#include <ctype.h>

#include <opencv/cv.h>
//#include <highgui.h>
#include <iostream>
#include <cmath>
using namespace std;
int H_MIN = 0;//odcien
int S_MIN = 0;//nasycenie
int V_MIN = 0;//jaskrawosc
int H_MAX = 256;
int S_MAX = 256;
int V_MAX = 256;
const string trackbarWindowName = "Kalibracja";
#define bluewhite  CV_RGB(0,162,232)
#define bluedark CV_RGB(63,72,204)
#define green   CV_RGB(34,177,76)
#define red   CV_RGB(237,28,36)
#define yellow   CV_RGB(255,242,0)
#define white CV_RGB(255,255,255)
#define black CV_RGB(0,0,0)

void on_trackbar( int, void* )
{//This function gets called whenever a
	// trackbar position is changed





}
void createTrackbars(){
	//create window for trackbars

cvNamedWindow( "Kalibracja", CV_WINDOW_AUTOSIZE );
	//create memory to store trackbar name on window
	char TrackbarName[50];
	sprintf( TrackbarName, "H_MIN", H_MIN);
	sprintf( TrackbarName, "H_MAX", H_MAX);
	sprintf( TrackbarName, "S_MIN", S_MIN);
	sprintf( TrackbarName, "S_MAX", S_MAX);
	sprintf( TrackbarName, "V_MIN", V_MIN);
	sprintf( TrackbarName, "V_MAX", V_MAX);
	//create trackbars and insert them into window
	//3 parameters are: the address of the variable that is changing when the trackbar is moved(eg.H_LOW),
	//the max value the trackbar can move (eg. H_HIGH), 
	//and the function that is called whenever the trackbar is moved(eg. on_trackbar)
	//                                  ---->    ---->     ---->      
	cv::createTrackbar( "H_MIN", trackbarWindowName, &H_MIN, H_MAX, on_trackbar );
	cv::createTrackbar( "H_MAX", trackbarWindowName, &H_MAX, H_MAX, on_trackbar );
	cv::createTrackbar( "S_MIN", trackbarWindowName, &S_MIN, S_MAX, on_trackbar );
	cv::createTrackbar( "S_MAX", trackbarWindowName, &S_MAX, S_MAX, on_trackbar );
	cv::createTrackbar( "V_MIN", trackbarWindowName, &V_MIN, V_MAX, on_trackbar );
	cv::createTrackbar( "V_MAX", trackbarWindowName, &V_MAX, V_MAX, on_trackbar );




}

IplImage* GetThresholdedImage(IplImage* img, CvScalar& lowerBound, CvScalar& upperBound)
{
    // Convert the image into an HSV image
    IplImage* imgHSV = cvCreateImage(cvGetSize(img), 8, 3);
    cvCvtColor(img, imgHSV, CV_BGR2HSV);

    IplImage* imgThreshed = cvCreateImage(cvGetSize(img), 8, 1);

    cvInRangeS(imgHSV, lowerBound, upperBound, imgThreshed);

    cvReleaseImage(&imgHSV);
    return imgThreshed;
}

int main(int argc, char* argv[])
{
	int numerek = 0;

	bool calibrationMode = true;
	if(calibrationMode){
		//create slider bars for HSV filtering
		createTrackbars();
	}

	// Default capture size - 640x480
    CvSize size = cvSize(640,480);
    // Open capture device. 0 is /dev/video0, 1 is /dev/video1, etc.
    CvCapture* capture = cvCaptureFromCAM( 0 );
    if( !capture )
    {
            fprintf( stderr, "ERROR: capture is NULL \n" );
            getchar();
            return -1;
    }
    // Create a window in which the captured images will be presented
    cvNamedWindow( "Kamera", CV_WINDOW_AUTOSIZE );
    //cvNamedWindow( "HSV", CV_WINDOW_AUTOSIZE );
	cvNamedWindow( "Kalibracja", CV_WINDOW_AUTOSIZE );
	cvNamedWindow( "Rysuj", CV_WINDOW_AUTOSIZE );
    //cvNamedWindow( "EdgeDetection", CV_WINDOW_AUTOSIZE );
	IplImage* imgColorPanel = 0;
    imgColorPanel = cvLoadImage( "cvPanel.png",  CV_LOAD_IMAGE_COLOR );  
	//IplImage *imgColorPanel2 = cvLoadImage("cvPanel.png",  CV_LOAD_IMAGE_COLOR );
	if(!imgColorPanel)
    {
		fprintf( stderr, "Nie znaleziono panelu !!! \n" );
        return -1;
    }
	int confirm_close = 10, confirm_clear = 10;
	//int posX = 0;
    //int posY = 0;
	char buffer [50];
	double area_limit = 700;

	CvScalar lineColor = white;
	CvScalar lastColor = white;

    //Detect a red ball
    //CvScalar hsv_min =cv::Scalar(H_MIN,S_MIN,V_MIN);
    //CvScalar hsv_max = cv::Scalar(H_MAX,S_MAX,V_MAX);
	//CvScalar hsv_min = cvScalar(0, 0, 130, 0);
    //CvScalar hsv_max = cvScalar(0, 0, 255, 0);

    IplImage *  hsv_frame    = cvCreateImage(size, IPL_DEPTH_8U, 3);
    IplImage*  thresholded   = cvCreateImage(size, IPL_DEPTH_8U, 1);
	IplImage *  rysuj   = cvCreateImage(size, IPL_DEPTH_8U, 3);
	//cvZero(rysuj);
	cvSet(rysuj, white);
	int lastX=-1;
    int lastY=-1;
	while( (calibrationMode == true) )
    {
   // while( 1 )
   // { 

    CvScalar hsv_min =cv::Scalar(H_MIN,S_MIN,V_MIN);
    CvScalar hsv_max = cv::Scalar(H_MAX,S_MAX,V_MAX);
        // Get one frame
        IplImage* frame = cvQueryFrame( capture );
        if( !frame )
        {
			 
                fprintf( stderr, "ERROR: frame is null...\n" );
                getchar();
                break;
        }
		cvFlip(frame,NULL,1);
        // Covert color space to HSV as it is much easier to filter colors in the HSV color-space.
        cvCvtColor(frame, hsv_frame, CV_BGR2HSV);
        // Filter out colors which are out of range.
        cvInRangeS(hsv_frame, hsv_min, hsv_max, thresholded);
        // Memory for hough circles
        CvMemStorage* storage = cvCreateMemStorage(0);


        // hough detector works better with some smoothing of the image
        cvSmooth( thresholded, thresholded, CV_GAUSSIAN, 9, 9 );

		IplImage* imgThresh = GetThresholdedImage(frame,hsv_min,hsv_max);

        // Calculate the moments to estimate the position of the object
        CvMoments *moments = (CvMoments*)malloc(sizeof(CvMoments));
        cvMoments(imgThresh, moments, 1);
		CvFont font, fontbig;
		cvInitFont( &font, CV_FONT_HERSHEY_COMPLEX, 1, .6, 0, 2, CV_AA);
		cvInitFont( &fontbig, CV_FONT_HERSHEY_COMPLEX, 3, .6, 0, 3, CV_AA);
		CvPoint cvpoint;  // location of the text
		
        // The actual moment values
        double moment10 = cvGetSpatialMoment(moments, 1, 0);
        double moment01 = cvGetSpatialMoment(moments, 0, 1);
        double area = cvGetCentralMoment(moments, 0, 0);
		// Holding the last and current positions
        

        //posX = 0;
        //posY = 0;


        if(moment10/area>=0 && moment10/area < 1280 && moment01/area >=0 && moment01/area < 1280
          && area>area_limit )
		//if(area>10000)
        {
            int possX = moment10/area;
            int possY = moment01/area;
			//lastX = posX;
         //lastY = posY;
		//double diff_X = lastX-posX;
		//double diff_Y = lastY-posY;
        //double magnitude = sqrt(   pow(diff_X,2) + pow(diff_Y,2)   );
        // We want to draw a line only if its a valid position
        if(lastX>=0 && lastY>=0 && possX>=0 && possY>=0)
        //if(magnitude > 0 && magnitude < 100 && posX > 120 && posX<530)
        {
            // Draw a line from the previous point to the current point
            cvLine(rysuj, cvPoint(possX, possY), cvPoint(lastX, lastY), lineColor, 4);
        }
		lastX = possX;
         lastY = possY;
        }
		 int posX = moment10/area;
            int posY = moment01/area;
		 if((posX  > 11 && posX < 96) && ( posY > 365 && posY <442))  // bluedark
        {
            lastColor = bluedark;
			lineColor = white;
			cvpoint = cvPoint(102,399);
            cvPutText( frame, "Wybrano kolor ciemnoniebieski.", cvpoint, &font, black );
        }
		 else if((posX  > 12 && posX < 96) && ( posY > 282 && posY <358))  // bluewhite
        {
			cvpoint = cvPoint(102,319);
            lastColor = bluewhite;
			lineColor = white;
            cvPutText( frame, "Wybrano kolor jasnoniebieski.", cvpoint, &font, black );
        }
		 else if((posX  > 12 && posX < 96) && ( posY > 199 && posY <275))  // green
        {
			cvpoint = cvPoint(102,235);
            lastColor = green;
			lineColor = white;
            cvPutText( frame, "Wybrano kolor zielony.", cvpoint, &font, black );
        }
		 else if((posX  > 12 && posX < 96) && ( posY > 116 && posY <193))  // red
        {
			cvpoint = cvPoint(102,151);
            lastColor = red;
			lineColor = white;
            cvPutText( frame, "Wybrano kolor czerwony.", cvpoint, &font, black );
        }
		 else if((posX  > 11 && posX < 96) && ( posY > 34 && posY <110))  // yellow
        {
			cvpoint = cvPoint(102,73);
            lastColor = yellow;
			lineColor = white;
            cvPutText( frame, "Wybrano kolor zolty.", cvpoint, &font, black );
        }
		  else if((posX > 565 && posX < 625) && (posY >15 && posY < 70))
		  {
			  calibrationMode = false;
		  }
		  else if((posX > 380 && posX < 442) && (posY >13 && posY < 68))
		  {
			  cvSet(rysuj, white);
			  lineColor = white;
			  lastColor = white;
		  }
		  else if((posX > 475 && posX < 540) && ( posY > 12 && posY < 68))
		  {
			  cvpoint = cvPoint(102,73);
			  sprintf (buffer, "Obrazek00%d.png",numerek++);
			  cvSaveImage(buffer ,rysuj);
			  cvPutText( frame, "Zapisano obrazek.", cvpoint, &font, black );
			  
		  }
		  else lineColor = lastColor;

        CvSeq* circles = cvHoughCircles(thresholded, storage, CV_HOUGH_GRADIENT, 2,thresholded->height/4, 100, 50, 10, 400);
		if (circles->total == 0)
		{
			lastX = posX;
			lastY = posY;
		}
        for (int i = 0; i < circles->total; i++)
        {
            float* p = (float*)cvGetSeqElem( circles, i );
            //printf("Ball! x=%f y=%f r=%f\n\r",p[0],p[1],p[2] );
            cvCircle( frame, cvPoint(cvRound(p[0]),cvRound(p[1])),3, CV_RGB(0,255,0), -1, 8, 0 );
            cvCircle( frame, cvPoint(cvRound(p[0]),cvRound(p[1])), cvRound(p[2]), CV_RGB(255,0,0), 3, 8, 0 );
			//cvLine(rysuj, cvPoint(cvRound(p[0]), cvRound(p[1])), cvPoint(lastX, lastY), lineColor, 4);
        }
		
		
		 //Combine everything in frame
         //cvAnd(frame, frame, frame);
         //cvAnd(imgColorPanel, frame, frame);

		cvAnd(imgColorPanel, frame, frame);
		cvAnd(rysuj, frame, frame);
		//cvAnd(imgColorPanel2, frame, frame);
		cvShowImage( "Rysuj", rysuj);
        cvShowImage( "Kamera", frame ); // Original stream with detected ball overlay
        //cvShowImage( "HSV", hsv_frame); // Original stream in the HSV color space
        cvShowImage( "Okno", thresholded ); // The stream after color filtering
		
      
        cvReleaseMemStorage(&storage);
        // Do not release the frame!
        //If ESC key pressed, Key=0x10001B under OpenCV 0.9.7(linux version),
        //remove higher bits using AND operator
        if( (cvWaitKey(10) & 255) == 27 ) break;
		//}
	}
     // Release the capture device housekeeping
     cvReleaseCapture( &capture );
     cvDestroyWindow( "mywindow" );
     return 0;
   }

   
