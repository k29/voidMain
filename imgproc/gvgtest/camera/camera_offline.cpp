/*
 * camera_simulator.cpp
 *
 */

#include "camera_offline.h"

#include "camera_imageYUV422.h"
#include "vision/image.h"
#include "comm/protobuf/msg_image.pb.h"
#include "Utils.h"

#include <opencv/highgui.h>

/*------------------------------------------------------------------------------------------------*/

/**
 * Init image index
 */
int CameraOffline::imageIdx = 0;

/*------------------------------------------------------------------------------------------------*/

/**
 **
 **
**/

CameraOffline::CameraOffline()
	: imageCV(0), cameraImageFileName("")
{}


/*------------------------------------------------------------------------------------------------*/

/**
 **
 **
**/

bool CameraOffline::openCamera(const char* deviceName, uint16_t requestedImageWidth, uint16_t requestedImageHeight) {
	image = new IMAGETYPE(requestedImageWidth, requestedImageHeight);
	return image != 0;
}


/*------------------------------------------------------------------------------------------------*/

/**
 ** Reads image from given command line argument --camera.device
 **
 **/
bool CameraOffline::capture() {
	delay(50);

	std::string cameraImageFile = robot.getConfig().getStrValue("camera.device", "camera.png");

	if (imageCV)
		cvReleaseImage(&imageCV);

	std::string extension = "";
	std::vector<std::string> files;
	int dotPos = cameraImageFile.find_last_of(".");

	// if dot exists and is not the first character
	if (dotPos != (int)std::string::npos && dotPos != 0)
		extension = cameraImageFile.substr(dotPos + 1);

	// test if path is a directory
	if (extension == "") {
		bool hasFiles = getFilesInDir(cameraImageFile, files);
		if (!hasFiles) {
			ERROR("Empty folder");
			return false;
		}
		std::string file = "";
		uint invalidFiles = 0;

		// sort alphabetically
		std::sort(files.begin(), files.end());

		// find valid file name
		while (true) {
			if (invalidFiles == files.size()){
				ERROR("No valid files in folder");
				return false;
			}
			// ensure that imageIdx is in range
			if (imageIdx >= (int)files.size())
				imageIdx = 0;
			else if (imageIdx < 0)
				imageIdx = files.size()-1;

			file = files[imageIdx];

			// ignore other folders and files that start or end with a dot
			dotPos = file.find_last_of(".");
			if (dotPos == (int)std::string::npos
					|| dotPos == 0
					|| dotPos == (int)file.length()-1) {
				imageIdx++;
				invalidFiles++;
			}
			else break;
		}

		// store current image file name
		cameraImageFileName = file;

		// update extension
		extension = file.substr(dotPos + 1);

		// combine folder and file name
		if (cameraImageFile[cameraImageFile.length()-1] != '/')
			cameraImageFile += '/';
		cameraImageFile += file;
	}


	// try reading the image file
	if (extension == "pbi") {
		bool error = !readImageFromPBI(cameraImageFile);
		if (error) {
			ERROR("Failed to load PBI image");
			return false;
		}
	} else {
		bool error = !readImage(cameraImageFile);
		if (error) {
			ERROR("OpenCV failed to load image");
			return false;
		}
	}

	if (imageCV)
		cvReleaseImage(&imageCV);

	totalFrames++;
	return true;
}

/**
 * Reads an image from a protobuf file.
 * @param cameraImageFile the string name of the image file to read
 * @return true if image could be read, false otherwise
 */
bool CameraOffline::readImageFromPBI(std::string cameraImageFile) {
	de::fumanoids::message::Image pbImage;

	std::ifstream file(cameraImageFile.c_str(), std::ios::in | std::ios::binary);
	if (file.fail())
		return false;

	pbImage.ParseFromIstream(&file);
	if (pbImage.IsInitialized() == false) {
		ERROR("Offline image not properly constructed");
		return false;
	}

	// sets the image
	bool foundRawData = false;
	for (int i=0; i < pbImage.imagedata_size(); i++) {
		const de::fumanoids::message::ImageData &pbImageData = pbImage.imagedata(i);
#if defined IMAGEFORMAT_YUV422
		if (pbImageData.format() == de::fumanoids::message::YUV422_IMAGE) {
#else
		if (pbImageData.format() == de::fumanoids::message::BAYER_IMAGE) {
#endif
			imageWidth  = pbImageData.width();
			imageHeight = pbImageData.height();

			// sets the pitch / roll and head angle data
			image->setImagePosition( pbImage.eyeheight(), pbImage.pitch(), pbImage.roll(), pbImage.headangle() );
			robot.getHead().setGyro(pbImage.pitch(), pbImage.roll());
			robot.getHead().setManualAngleX(pbImage.headangle());

			void* newData = malloc(pbImageData.data().size());
			memcpy(newData, pbImageData.data().c_str(), pbImageData.data().size());
			image->freeImageData();
			image->setImage(newData, imageWidth*imageHeight, imageWidth, imageHeight);
			//printf("set new pbi image (%d bytes, %dx%d)\n", (int)pbImageData.data().size(), pbImageData.width(), pbImageData.height());
			foundRawData = true;
		}
	}

	if (foundRawData == false) {
#if defined IMAGEFORMAT_YUV422
		ERROR("The pbi file '%s' does not contain raw image data for YUV422, try another format/build or use a PNG image", cameraImageFile.c_str());
#else
		ERROR("The pbi file '%s' does not contain raw image data for Bayer, try another format/build or use a PNG image", cameraImageFile.c_str());
#endif
		return false;
	}
	return true;
}

/**
 * Reads the image by using OpenCV
 * @param cameraImageFile the string name of the image file to read
 * @return true if image could be read, false otherwise
 */
bool CameraOffline::readImage(std::string cameraImageFile) {
	imageCV = cvLoadImage(cameraImageFile.c_str());

	if (0 == imageCV) {
		return false;
	}

	imageWidth  = imageCV->width;
	imageHeight = imageCV->height;

#if defined IMAGEFORMAT_YUV422
	IplImage* yuv = cvCreateImage(cvSize(imageWidth, imageHeight), IPL_DEPTH_8U, 3);
	cvCvtColor(imageCV, yuv, CV_BGR2YCrCb);

	uint8_t *yuv422 = (uint8_t*)malloc(imageWidth*imageHeight*2);
	if (!yuv422) {
		ERROR("Could not allocate memory for image conversion.");
		return false;
	}

	// convert image to yuv422
	for (int x=0; x < imageWidth; x += 2) {
		for (int y=0; y < imageHeight; y++) {
			uint8_t y1 = ((uchar *)(yuv->imageData + y*yuv->widthStep))[x*yuv->nChannels + 0];
			uint8_t u1 = ((uchar *)(yuv->imageData + y*yuv->widthStep))[x*yuv->nChannels + 1];
			uint8_t v1 = ((uchar *)(yuv->imageData + y*yuv->widthStep))[x*yuv->nChannels + 2];

			uint8_t y2 = ((uchar *)(yuv->imageData + y*yuv->widthStep))[(x+1)*yuv->nChannels + 0];
			uint8_t u2 = ((uchar *)(yuv->imageData + y*yuv->widthStep))[(x+1)*yuv->nChannels + 1];
			uint8_t v2 = ((uchar *)(yuv->imageData + y*yuv->widthStep))[(x+1)*yuv->nChannels + 2];

			yuv422[2*y*imageWidth + 2*x + 0] = y1;
			yuv422[2*y*imageWidth + 2*x + 1] = (v1 + v2)/2;
			yuv422[2*y*imageWidth + 2*x + 2] = y2;
			yuv422[2*y*imageWidth + 2*x + 3] = (u1 + u2)/2;
		}
	}

	cvReleaseImage(&yuv);
	image->freeImageData();
	image->setImage(yuv422, imageWidth*imageHeight*2, imageWidth, imageHeight);
#else
	uint8_t *bayer = (uint8_t*)malloc(imageWidth*imageHeight);

	for (int x=0; x<imageWidth-1; x++) {
		for (int y=0; y<imageHeight-1; y++) {
			uchar* pixel = (bayer + y*imageWidth + x);

			CvScalar p1 = cvGet2D(imageCV, (y & ~1),   (x & ~1)  );
			CvScalar p2 = cvGet2D(imageCV, (y & ~1),   (x & ~1)+1);
			CvScalar p3 = cvGet2D(imageCV, (y & ~1)+1, (x & ~1)  );
			CvScalar p4 = cvGet2D(imageCV, (y & ~1)+1, (x & ~1)+1);

			// TODO: the green1 / green2 calculation is only approximate
			if (x & 1) {
				if (y & 1) { // x odd, y odd -> blue
					*pixel = (uint8_t)((p1.val[0] + p2.val[0] + p3.val[0] + p4.val[0]) / 4);
				} else { // x odd, y even -> green1
					*pixel = (uint8_t)((p1.val[1] + p2.val[1] + p3.val[1] + p4.val[1]) / 4);
				}
			} else {
				if (y & 1) { // x even, y odd -> green2
					*pixel = (uint8_t)((p1.val[1] + p2.val[1] + p3.val[1] + p4.val[1]) / 4);
				} else { // x even, y even -> red
					*pixel = (uint8_t)((p1.val[2] + p2.val[2] + p3.val[2] + p4.val[2]) / 4);
				}
			}
		}
	}

	image->freeImageData();
	image->setImage(bayer, imageWidth*imageHeight, imageWidth, imageHeight);
#endif

	// try to determine pitch and roll from the filename
	{
		// get current pitch/roll values
		int16_t pitch = image->getImagePositionPitch();
		int16_t roll  = image->getImagePositionRoll();

		// get the manually set values
		int preconfigured_pitch = robot.getConfig().getIntValue("Pitch", -999);
		int preconfigured_roll  = robot.getConfig().getIntValue("Roll",  -999);

		// determine where in the filename string the pitch/roll values would be
		size_t pitchStrOffset = cameraImageFile.find("-pitch-");
		size_t rollStrOffset  = cameraImageFile.find("-roll-");

		// extract the pitch/roll values from the filename
		int image_pitch_parameter = -999;
		int image_roll_parameter  = -999;
		if (pitchStrOffset != std::string::npos)
			image_pitch_parameter = atoi(cameraImageFile.c_str() + pitchStrOffset + 7);
		if (rollStrOffset != std::string::npos)
			image_roll_parameter = atoi(cameraImageFile.c_str() + rollStrOffset + 6);

		// if the pitch or roll was not manually set, use the image pitch or roll
		// value (if any)
		if (preconfigured_pitch == -999 && image_pitch_parameter != -999)
			pitch = image_pitch_parameter;
		if (preconfigured_roll == -999 && image_roll_parameter != -999)
			roll  = image_roll_parameter;

		// TODO: determine head turn angle from filename
		image->setImagePosition( ROBOT_EYE_HEIGHT, pitch, roll, 0);
		robot.getHead().setGyro(pitch, roll);
	}

	return true;
}
