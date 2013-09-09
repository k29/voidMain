/** @file
 **
 */

#include "camera_image.h"
#include "comm.h"
#include "robot.h"
#include "theCameraModel.h"
#include "vision/vision.h"

#include <opencv/highgui.h>
#include <stdio.h>
#include <dirent.h>

#include <sstream>
#include <iomanip>

uint32_t CameraImage::pixelCounterGradient = 0;
uint32_t CameraImage::pixelCountergetPixelAsXXX = 0;

PixelEnhanceMode CameraImage::autoPixelEnhanceMode = AUTO;

/*------------------------------------------------------------------------------------------------*/

/**
 **
 */

CameraImage::CameraImage(uint16_t _imageWidth, uint16_t _imageHeight)
	: imageWidth(_imageWidth)
	, imageHeight(_imageHeight)
	, imageOffsetX(0)
	, imageOffsetY(0)
	, fullImageWidth(_imageWidth)
	, fullImageHeight(_imageHeight)
	, centerX(_imageWidth/2)
	, centerY(_imageHeight/2)
	, camera_height(0)
	, camera_pitch(0)
	, camera_roll(0)
	, currentData(0)
	, currentDataLength(0)
{
}


/*------------------------------------------------------------------------------------------------*/

/**
 **
 */

CameraImage::~CameraImage() {
}


/*------------------------------------------------------------------------------------------------*/

/** Sets the subset of the image we are interested in
 **
 ** @param offsetX        Horizontal offset into the full image data (in pixel)
 ** @param offsetY        Vertical offset into the full image data (in pixel)
 ** @param subsetWidth    Width (in pixel) of the subset image (divisible by 4)
 ** @param subsetHeight   Height (in pixel) of the subset image
 */

void CameraImage::setImageAsSubset(
		  uint16_t offsetX
		, uint16_t offsetY
		, uint16_t subsetWidth
		, uint16_t subsetHeight
) {
	if (offsetX < fullImageWidth)
		imageOffsetX = offsetX;
	else {
		WARNING("Image-ROI offset (X) exceeds image width, ignoring");
		imageOffsetX = 0;
	}

	if (offsetY < fullImageHeight)
		imageOffsetY = offsetY;
	else {
		WARNING("Image-ROI offset (Y) exceeds image width, ignoring");
		imageOffsetY = 0;
	}

	if (offsetX + subsetWidth <= fullImageWidth)
		imageWidth = subsetWidth;
	else {
		WARNING("Image-ROI width too large, limiting to end of image");
		imageWidth = fullImageWidth - offsetX;
	}

	if (offsetY + subsetHeight <= fullImageHeight)
		imageHeight = subsetHeight;
	else {
		WARNING("Image-ROI height too large, limiting to end of image");
		imageHeight = fullImageHeight - offsetY;
	}

	// adjust image center coordinates
	centerX = imageWidth/2;
	centerY = imageHeight/2;
}


/*------------------------------------------------------------------------------------------------*/

/**
 **
 */

void CameraImage::setImage(void* newImageData, int newImageDataLength, int16_t newImageWidth, int16_t newImageHeight) {
	currentData = newImageData;
	currentDataLength = newImageDataLength;

	// take the gyro roll and pitch, offsets to the camera are used in the camera model
	robottime_t time = getCurrentTime();
	GyroValues pitchAndRoll = robot.getHead().getGyroPitchRoll(time);
	robot.getHead().update();
	int16_t headAngle = robot.getHead().getAngleX();
	float pitch = pitchAndRoll.pitch;
	float roll  = pitchAndRoll.roll;

	// update the head angles right after the frame has been captured
	// to keep image and angles somehow synchronized
	setImagePosition( ROBOT_EYE_HEIGHT,  (int16_t) round(pitch), (int16_t) round(roll), headAngle );
	TheCameraModel::getInstance().setAngles( pitch, roll, headAngle );
	TheCameraModel::getInstance().setTimeDiff(pitchAndRoll.timediff);

	if (newImageWidth > 0) {
		fullImageWidth  = newImageWidth;
		fullImageHeight = newImageHeight;

		// reset ROI
		imageOffsetX = imageOffsetY = 0;
		imageWidth   = fullImageWidth;
		imageHeight  = fullImageHeight;
//		setImageAsSubset(imageOffsetX, imageOffsetY, imageWidth, imageHeight);
	}
}


/*------------------------------------------------------------------------------------------------*/

/**
 **
 */

void CameraImage::setImagePosition(uint16_t height, int16_t pitch, int16_t roll, int16_t headAngle) {
	camera_height    = height;
	camera_pitch     = pitch;
	camera_roll      = roll;
	camera_headAngle = headAngle;
}


/*------------------------------------------------------------------------------------------------*/

/**
 **
 */

void CameraImage::freeImageData() {
	if (currentData)
		free(currentData);
	currentData = 0;
	currentDataLength = 0;
}


/*------------------------------------------------------------------------------------------------*/

/** Returns a new OpenCV image object in YUV format.
 **
 ** @param scale      Scaling factor. Must be 1/1, 1/2, 1/3, ...
 **
 ** @return new OpenCV image object in YUV format, caller is responsible of freeing resources
 */

IplImage* CameraImage::getImageAsYUV(double scale) const {
	if (scale > 1)
		return 0;

	return getImageAsYUV((uint16_t)(imageWidth * scale), (uint16_t)(imageHeight * scale));
}


/*------------------------------------------------------------------------------------------------*/

/** Returns a new OpenCV image object in RGB format.
 **
 ** @param scale     Scaling factor. Must be 1/1, 1/2, 1/3, ...
 ** @param colorMgr  Optional color manager
 **
 ** @return new OpenCV image object in RGB format, caller is responsible of freeing resources
 */

IplImage* CameraImage::getImageAsRGB(double scale, const ColorManager* colorMgr, bool bgr, PixelEnhanceMode enhanceMode) const {
	if (scale > 1)
		return 0;

	return getImageAsRGB((uint16_t)(imageWidth * scale), (uint16_t)(imageHeight * scale), colorMgr, bgr, enhanceMode);
}


/*------------------------------------------------------------------------------------------------*/

/** Get the gradient image
 **
 ** @param scale
 ** @return gradient image
 */

IplImage* CameraImage::getGradientImage(double scale) const {
	if (scale > 1)
		return 0;

	// calculate image size
	uint16_t newImageWidth  = (uint16_t)(imageWidth * scale);
	uint16_t newImageHeight = (uint16_t)(imageHeight * scale);

	// create OpenCV image object
	CvSize s;
	s.width  = newImageWidth;
	s.height = newImageHeight;
	IplImage *img = cvCreateImage(s, IPL_DEPTH_8U, 3);

	// calculate scaling ratio (only natural numbers supported)
	uint8_t stepX = imageWidth  / newImageWidth;
	uint8_t stepY = imageHeight / newImageHeight;

	const uint8_t border = 2;
	for (uint16_t y=border; y < imageHeight-border; y += stepY) {
		uint8_t* rowDataStart = (uint8_t*)img->imageData + 3*(y/stepY * newImageWidth);
		for(uint16_t x=border; x < imageWidth-border; x += stepX) {
			uint8_t* imageData = rowDataStart + 3 * (x/stepX);

			int16_t gradX, gradY;
			uint16_t magnitude;
			getGradient(x, y, &gradX, &gradY, &magnitude, 2);

			int magnitudeThreshold = Vision::getInstance().calibration.getMagnitudeThreshold();
			if(magnitudeThreshold < 0)
				magnitudeThreshold = 20; // default value
			if(magnitude < magnitudeThreshold) {
				gradY = -8*128;
				gradX = -8*128;
			}

			*imageData = 0;
			*(imageData+2) = gradY/8 + 128;
			*(imageData+1) = gradX/8 + 128;
			*(imageData+0)=0;
		}
	}

	return img;
}


/*------------------------------------------------------------------------------------------------*/

/**
 **
 */

void CameraImage::saveImage(uint8_t scale, std::string path, std::string filename) {
	IplImage *cv_image = getImageAsRGB(1.0/scale);

	if (path.length() == 0)
		path = ".";

	std::string full_filename;
	if (filename.length() > 0)
		full_filename = path + "/" + filename;
	else {
		std::stringstream ss;
		int count = 0;

		do {
			ss.seekp(0, std::ios::beg);
			ss << "image-" << robot.getName();

			robottime_t nowSeconds = getCurrentTime() / 1000;
			uint16_t hours    = (nowSeconds / 60 / 60);
			uint16_t minutes  = (nowSeconds / 60) % 60;
			uint16_t seconds  = nowSeconds % 60;

			ss << "-" <<  std::setfill('0') << std::setw(2) << hours << std::setw(2) << minutes << std::setw(2) << seconds;
			ss << "-pitch-" << camera_pitch << "-roll-" << camera_roll;

			// add extension
			if (count > 0)
				ss << "-" + count;
			count++;
			ss << ".bmp";
		}  while (fileExists(ss.str().c_str()));

		full_filename = ss.str();
		INFO("Saving image %s", full_filename.c_str());
	}
	cvSaveImage(full_filename.c_str(), cv_image);
	INFO("Image saved");
	cvReleaseImage(&cv_image);
}
