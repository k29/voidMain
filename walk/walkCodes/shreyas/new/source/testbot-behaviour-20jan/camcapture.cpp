#include <opencv2/opencv.hpp>
#include <FlyCapture2.h>
#include "camcapture.h"
#include "imgproc.h"
#include <cstring>

//pass true as parameter to load luts, false to skip loading luts
CamCapture::CamCapture(bool param, int percent, int percent2)
{
	isInit = false;
    doLUT = param;
    small_percent = percent;
    small_percent2 = percent2;
    width_var = (640*percent)/100;
    height_var = (480*percent)/100;
    width_var_small = (640*percent2)/100;
    height_var_small = (480*percent2)/100;
    width_var_full = 640;
    height_var_full = 480;
}



CamCapture::~CamCapture()
{
	if(isInit==false)
		return;

	cvReleaseImage(&bayerimg);
    cvReleaseImage(&rgbimg);
    // Stop capturing images
    error = cam.StopCapture();
    if (error != FlyCapture2::PGRERROR_OK)
    {
        error.PrintErrorTrace();
        return;
    }      
    
    // Disconnect the camera
    error = cam.Disconnect();
    if (error != FlyCapture2::PGRERROR_OK)
    {
        error.PrintErrorTrace();
        return;
    }
}



CamError CamCapture::init()
{
    if(isInit==true)
        return CAM_SUCCESS;

    //Loading lookup table
    if(doLUT == true)
    {
        printf("Loading lookup tables...");
        if(loadLUT(REDC)==false||loadLUT(BLUEC)==false||loadLUT(YELLOWC)==false||loadLUT(GREENC)==false||loadLUT(WHITEC)==false||loadLUT(BLACKC)==false)
        {
            printf("Unable to open LUT\n");
            return CAM_FAILURE;
        }
        printf("Loaded.\n");
    }
    //Initializing camera
	error = busMgr.GetCameraFromIndex(0, &guid);
    if (error != FlyCapture2::PGRERROR_OK)
    {
        error.PrintErrorTrace();
        return CAM_FAILURE;
    }

  	vm = FlyCapture2::VIDEOMODE_640x480Y8;
    fr = FlyCapture2::FRAMERATE_60;

    error = cam.Connect(&guid);
    if (error != FlyCapture2::PGRERROR_OK)
    {
        error.PrintErrorTrace();
        return CAM_FAILURE;
    }

    cam.SetVideoModeAndFrameRate(vm, fr);
    //Starting the capture
    error = cam.StartCapture();
    if (error != FlyCapture2::PGRERROR_OK)
    {
        error.PrintErrorTrace();
       	return CAM_FAILURE;
    }

    cam.RetrieveBuffer(&rawImage);
    // Size variables cannot dynamically changed. Must be defined in constructor
    // width_var_full = rawImage.GetCols();
    // height_var_full = rawImage.GetRows();
    bayerimg = cvCreateImage(cvSize(rawImage.GetCols(), rawImage.GetRows()), 8, 1);
    rgbimg_full = cvCreateImage(cvSize(rawImage.GetCols(), rawImage.GetRows()), 8, 3);
    rgbimg = cvCreateImage(cvSize((rawImage.GetCols()*small_percent)/100, (rawImage.GetRows()*small_percent)/100), 8, 3);
    rgbimg_small = cvCreateImage(cvSize((rawImage.GetCols()*small_percent2)/100, (rawImage.GetRows()*small_percent2)/100), 8, 3);

#ifdef CAMCAPTURE_DEBUG
    showSeg = cvCreateImage(cvSize(rgbimg->width, rgbimg->height), 8, 3);
#endif

    // width_var = rgbimg->width;
    // height_var = rgbimg->height;
    isInit = true;
    return CAM_SUCCESS;

}



CamError CamCapture::getImage()
{
	if(isInit==false)
		return CAM_FAILURE;
    // Start capturing images
    cam.RetrieveBuffer(&rawImage);

    // Get the raw image dimensions
    //TODO: are the following 3 lines required?
    //Have been removed for now.
    /*
    FlyCapture2::PixelFormat pixFormat;
    unsigned int rows, cols, stride;
    rawImage.GetDimensions( &rows, &cols, &stride, &pixFormat );
    */

    //Copy the image into the IplImage of OpenCV
    //TODO
    /*
        Maybe make this more efficient by removing the
        need to copy image or something? eg making bayerimg just point to
        the data in rawImage?
        TODO: need to check whether this function gives at least
        60 fps on normal operation ie without any other
        processing taking place.
    */
    memcpy(bayerimg->imageData, rawImage.GetData(), rawImage.GetDataSize());

    if(!bayerimg)
    	return CAM_FAILURE;

    //TODO: see if this image conversion can be removed, and if
    //bayer image can directly be used for segmentation using lut
    cvCvtColor(bayerimg, rgbimg_full, CV_BayerBG2BGR);
    if(small_perc