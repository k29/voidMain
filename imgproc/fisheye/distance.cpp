//OpenCV Headers
#include <opencv/cv.h>
#include <opencv/highgui.h>
//Input-Output
#include <stdio.h>
//Blob Library Headers
#include <cvblob.h>

#include <ueye.h>
//Definitions
#define WIDTHd 752
#define HEIGHTd 480
#define HEIGHTu 752
#define WIDTHu 480
#define PI 3.14159265359
#define rad2deg(x) x*180/PI
//NameSpaces
using namespace std;
using namespace cvb;
//using namespace std;

char* imgPointer = NULL;
int imgMemPointer;

typedef struct{
	double x, y, z;
}Point3D;

int initializeCam(HIDS hCam)
{	
	char* errMsg = (char*)malloc(sizeof(char)*200);
	int err = 0;

	int nRet = is_InitCamera (&hCam, NULL);
	if (nRet != IS_SUCCESS)
	{
		is_GetError (hCam, &err, &errMsg);
		printf("Camera Init Error %d: %s\n",err,errMsg);
		return EXIT_FAILURE;
	}

	nRet = is_SetColorMode(hCam, IS_CM_BGR8_PACKED);
    if (nRet != IS_SUCCESS) 
    {
         	is_GetError (hCam, &err, &errMsg);
			printf("Color Mode Error %d: %s\n",err,errMsg);
			return EXIT_FAILURE;  
    }

    nRet = is_SetHardwareGain(hCam, 100, 4, 0, 13);
    if (nRet != IS_SUCCESS) 
    {
         	is_GetError (hCam, &err, &errMsg);
			printf("Hardware Gain Error %d: %s\n",err,errMsg);
			return EXIT_FAILURE;
    }

	return EXIT_SUCCESS;
}

int setImgMem(HIDS hCam)
{
	char* errMsg = (char*)malloc(sizeof(char)*200);
	int err = 0;

	int nRet = is_AllocImageMem(hCam, 752, 480, 24, &imgPointer, &imgMemPointer);
	if(nRet != IS_SUCCESS)
	{
		is_GetError (hCam, &err, &errMsg);
		printf("MemAlloc Unsuccessful %d: %s\n",err,errMsg);
		return EXIT_FAILURE;
	}

	nRet = is_SetImageMem (hCam, imgPointer, imgMemPointer);
	if(nRet != IS_SUCCESS)
	{
		is_GetError (hCam, &err, &errMsg);
		printf("Could not set/activate image memory %d: %s\n",err, errMsg);
		return EXIT_FAILURE;
	}

	return EXIT_SUCCESS;
}

void getFrame(HIDS hCam, IplImage* frame)
{
	char* errMsg = (char*)malloc(sizeof(char)*200);
	int err = 0;


	int nRet = is_FreezeVideo (hCam, IS_WAIT) ;
	if(nRet != IS_SUCCESS)
	{
		is_GetError (hCam, &err, &errMsg);
		printf("Could not grab image %d: %s\n",err,errMsg);
		//return EXIT_FAILURE;
	}
		
	//fill in the OpenCV imaga data 
	//IplImage* img_rgb = cvCreateImage(cvSize(752,480),8,3);
	memcpy(frame->imageData, imgPointer, 752*480 * 3);
	//img_rgb->imageData = imgPointer;
	//return img_rgb;
}

int exitCam(HIDS hCam)
{
	int	nRet = is_ExitCamera (hCam);
	if(nRet != IS_SUCCESS)
	{
		printf("Could not exit camera \n");
		return EXIT_FAILURE;
	}
}

void getLinearCoords(int xd, int yd, int* xu, int* yu)
{
	double ax = -7e-06;
//	double id,jd,iu,ju;
	
	// id = (double)xd - WIDTHd/2;
	// jd = (double)-yd + HEIGHTd/2;
	// double r2 = id*id + jd*jd;

	// iu = id/(1+ax*r2);
	// ju = jd/(1+ax*r2);

 // 	*xu = iu + WIDTHu/2;
 // 	*yu = -ju + HEIGHTu/2;

	double r2 = xd*xd +yd*yd;
	*xu = xd/(1+ax*r2);
	*yu = yd/(1+ax*r2);
}

Point3D getPt(int x, int y)
{
	double a = 5*0.04; 
	Point3D p;
	//p.z = camHeight/y*a;
	//p.y = 67.5;	//cm
	return p; 
}

int main()
{
	//INITIALIZE CAMERA
	HIDS hCam = 1;
	initializeCam(hCam);
	setImgMem(hCam);

	//Windows
	cvNamedWindow("Live",CV_WINDOW_AUTOSIZE);
	cvNamedWindow("Threshed", CV_WINDOW_AUTOSIZE);


	//Image Variables
	IplImage* frame=cvCreateImage(cvSize(752, 480), 8, 3);//fisheye image
	IplImage* img_hsv=cvCreateImage(cvSize(752, 480), 8, 3);//Image in HSV color space
	IplImage* threshy = cvCreateImage(cvSize(752, 480), 8, 1);
	IplImage* labelImg=cvCreateImage(cvSize(752, 480),IPL_DEPTH_LABEL,1);//Image Variable for blobs


	CvBlobs blobs;

	int xu, yu;//coordinates of undistorted image
	int xd, yd;//coordinates in distorted image

	Point3D ypt;


		while(1)
		{
			//Getting the current frame
			getFrame(hCam, frame);
			//If failed to get break the loop
			if(!frame)
				break;
			
				
			cvCvtColor(frame,img_hsv,CV_BGR2HSV);
			//Thresholding the frame for yellow
			cvInRangeS(img_hsv, cvScalar(20, 100, 20), cvScalar(30, 255, 255), threshy);
			//Filtering the frame
			cvSmooth(threshy,threshy,CV_MEDIAN,7,7);
			//Finding the blobs
			unsigned int result=cvLabel(threshy,labelImg,blobs);
			//Filtering the blobs
			cvFilterByArea(blobs,100,10000);
			//Rendering the blobs
			cvRenderBlobs(labelImg,blobs,frame,frame);

//int x1, y1;
			for (CvBlobs::const_iterator it=blobs.begin(); it!=blobs.end(); ++it)
			{
				double moment10 = it->second->m10;
				double moment01 = it->second->m01;
				double area = it->second->area;

				//Calculating the current position
				xd = moment10/area;
				yd = moment01/area;
				
				xd = xd - WIDTHd/2;
				yd = -yd + HEIGHTd/2;

				cout<<"non-linear coords: xd="<<xd<<"     yd="<<yd<<endl;

				getLinearCoords(xd, yd, &xu, &yu);

				cout<<"linear coords: x="<<xu<<"     y="<<yu<<endl;
				
				//ypt = getPt(xu, yu);
				//cout<<"x= "<<ypt.x<<"\ty= "<<ypt.y<<"\tz= "<<ypt.z<<endl;
			}
			//Showing the images
			cvShowImage("Live",frame);
			cvShowImage("Threshed",threshy);

			//Escape Sequence
			char c=cvWaitKey(5);
			if(c==27)
				break;
		}
	//Cleanup
	cvReleaseImage(&frame);
	cvReleaseImage(&threshy);
	cvReleaseImage(&img_hsv);
	cvReleaseImage(&labelImg);
	cvDestroyAllWindows();
	exitCam(hCam);
	return 0;
}