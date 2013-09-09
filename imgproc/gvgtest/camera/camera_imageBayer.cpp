/** @file
 **
 */

#include "camera_imageBayer.h"

#include "comm.h"
#include "robot.h"
#include "vision/color.h"

#include <arpa/inet.h>
#include <zlib.h>


#include "opencv/highgui.h"

/*------------------------------------------------------------------------------------------------*/

/**
 **
 */

CameraImageBayer::CameraImageBayer(uint16_t _imageWidth, uint16_t _imageHeight)
		: CameraImage(_imageWidth, _imageHeight)
{
}


/*------------------------------------------------------------------------------------------------*/

/** Returns a new OpenCV image object in YUV format.
 **
 ** @param newImageWidth    Width of image to create
 ** @param newImageHeight   Height of image to create
 **
 ** @return new OpenCV image object in YUV format, caller is responsible of freeing resources
 */

IplImage* CameraImageBayer::getImageAsYUV(uint16_t newImageWidth, uint16_t newImageHeight) const {
	// we do not support scaling up
	if (newImageWidth > imageWidth)
		newImageWidth = imageWidth;
	if (newImageHeight > imageHeight)
		newImageHeight = imageHeight;

	// create OpenCV image object
	CvSize s;
	s.width  = newImageWidth;
	s.height = newImageHeight;
	IplImage *img = cvCreateImage(s, IPL_DEPTH_8U, 3);

	// calculate scaling ratio
	uint8_t stepX = imageWidth  / newImageWidth;
	uint8_t stepY = imageHeight / newImageHeight;

	for (uint16_t y=0; y < imageHeight; y += stepY) {
		uint8_t* rowDataStart = (uint8_t*)img->imageData + 3*(y/stepY * newImageWidth);
		for(uint16_t x=0; x < imageWidth; x += stepX) {
			uint8_t* imageData = rowDataStart + 3 * (x/stepX);
			getPixelAsYUV(x, y, imageData, imageData+1, imageData+2);
		}
	}

	return img;
}



/*------------------------------------------------------------------------------------------------*/

/** Returns a new OpenCV image object in RGB format.
 **
 ** @param newImageWidth    Width of image to create
 ** @param newImageHeight   Height of image to create
 ** @param colorMgr         Optional color manager
 **
 ** @return new OpenCV image object in RGB format, caller is responsible of freeing resources
 */

IplImage* CameraImageBayer::getImageAsRGB(uint16_t newImageWidth, uint16_t newImageHeight, const ColorManager* colorMgr, bool bgr, PixelEnhanceMode enhanceMode) const {
	// we do not support scaling up
	if (newImageWidth > imageWidth)
		newImageWidth = imageWidth;
	if (newImageHeight > imageHeight)
		newImageHeight = imageHeight;

	// create OpenCV image object
	IplImage *img = cvCreateImage(cvSize(newImageWidth, newImageHeight), IPL_DEPTH_8U, 3);

	if (enhanceMode == NONE) {
		IplImage* currentImage = cvCreateImageHeader(cvSize(imageWidth, imageHeight), IPL_DEPTH_8U, 1);
		cvSetData(currentImage, currentData, imageWidth);
		if (imageWidth > newImageWidth) {
			IplImage * img2 = cvCreateImage(cvSize(imageWidth, imageHeight), IPL_DEPTH_8U, 3);
			if (bgr) {
				cvCvtColor(currentImage, img2, CV_BayerBG2BGR);
			} else {
				cvCvtColor(currentImage, img2, CV_BayerBG2RGB);
			}
			cvResize(img2, img, CV_INTER_LINEAR);
			cvReleaseImage(&img2);
		} else {
			if (bgr) {
				cvCvtColor(currentImage, img, CV_BayerBG2BGR);
			} else {
				cvCvtColor(currentImage, img, CV_BayerBG2RGB);
			}
		}
		cvReleaseImageHeader(&currentImage);
		return img;
	}

	// calculate scaling ratio (only natural numbers supported)
	uint8_t stepX = imageWidth  / newImageWidth;
	uint8_t stepY = imageHeight / newImageHeight;

	uint8_t* rowDataStart = (uint8_t*)img->imageData;
	for (uint16_t y=0; y < imageHeight; y += stepY, rowDataStart += img->widthStep) {
		uint8_t* imageData = rowDataStart;
		for(uint16_t x=0; x < imageWidth; x += stepX, imageData += 3) {

			if (colorMgr) {
				uint8_t rgb_r, rgb_b, rgb_g;
				getPixelAsRGB(x, y, &rgb_r, &rgb_b, &rgb_g, enhanceMode);

				Color color = colorMgr->getColor(rgb_r, rgb_b, rgb_g);

				if (bgr)
					colorMgr->getRGBColor(color, *(imageData+2), *(imageData+1), *imageData);
				else
					colorMgr->getRGBColor(color, *imageData, *(imageData+1), *(imageData+2));
			} else if (bgr)
				getPixelAsRGB(x, y, imageData+2, imageData+1, imageData, enhanceMode);
			else
				get