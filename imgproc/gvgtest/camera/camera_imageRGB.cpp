/** @file
 **
 */

#include "camera_imageRGB.h"

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

CameraImageRGB::CameraImageRGB(uint16_t _imageWidth, uint16_t _imageHeight)
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

IplImage* CameraImageRGB::getImageAsYUV(uint16_t newImageWidth, uint16_t newImageHeight) const {
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

IplImage* CameraImageRGB::getImageAsRGB(uint16_t newImageWidth, uint16_t newImageHeight, const ColorManager* colorMgr, bool bgr, PixelEnhanceMode enhanceMode) const {
	// we do not support scaling up
	if (newImageWidth > imageWidth)
		newImageWidth = imageWidth;
	if (newImageHeight > imageHeight)
		newImageHeight = imageHeight;

	IplImage *img = cvCreateImage(cvSize(newImageWidth, newImageHeight), IPL_DEPTH_8U, 3);

	if (enhanceMode == NONE) {
		IplImage* currentImage = cvCreateImageHeader(cvSize(imageWidth, imageHeight), IPL_DEPTH_8U, 3);
		cvSetData(currentImage, currentData, imageWidth*3);

		if (imageWidth > newImageWidth) {
			IplImage * img2 = cvCreateImage(cvSize(imageWidth, imageHeight), IPL_DEPTH_8U, 3);
			if (bgr) {
				cvCvtColor(currentImage, img2, CV_RGB2BGR);
			} else {
				cvCopyImage(currentImage, img2);
			}
			cvResize(img2, img, CV_INTER_LINEAR);
			cvReleaseImage(&img2);
		} else {
			if (bgr) {
				cvCvtColor(currentImage, img, CV_RGB2BGR);
			} else {
				cvCopyImage(currentImage, img);
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
				getPixelAsRGB(x, y, imageData, imageData+1, imageData+2, enhanceMode);
		}
	}

	return img;
}



/*------------------------------------------------------------------------------------------------*/

/** Send raw image (well, kinda, we send the processed RGB, sigh)
 **
 ** @param fd        File descriptor to send data to
 ** @param compress  Whether to compress the file
 */

bool CameraImageRGB::sendRawImage(Transport *transport, bool compress) const {
	/*
	  uint16_t (BE)  imageWidth
	  uint16_t (BE)  imageHeight
	  uint32_t (BE)  uncompressedImageSize
	  uint32_t (BE)  compressedImageSize
	  uint8_t[]      imageData (YUV422)
	*/

	IplImage* image = getImageAsRGB(1, 0, false);

	uint32_t imageSizeInBytes = image->imageSize;

	// send the first two values right away
	uint16_t value16;
	value16 = htons(imageWidth);  transport->write((uint8_t*)&value16, 2);
	value16 = htons(imageHeight); transport->write((uint8_t*)&value16, 2);

	if (compress) {
		uLongf uncompressedSize = imageSizeInBytes;
		uLongf compressedSize   = compressBound(uncompressedSize);

		// reserve memory for compressed image
		void* uncompressedImage = image->imageData;
		void* compressedImage   = malloc(compressedSize);

		if (compressedImage == 0)
			compress = false; // failure getting enough memory, send uncompressed

		// if we are still setup for sending the image compressed, do so now
		if (compress) {
			// compress
			int res = ::compress2((Bytef*)compressedImage, &compressedSize, (Bytef*)uncompressedImage, uncompressedSize, Z_BEST_COMPRESSION);

			if (res == Z_OK) {
				INFO("Image compression finished (compressed to %d (%d%%) bytes)", compressedSize, compressedSize*100/uncompressedSize);

				uint32_t value32;
				value32 = htonl(uncompressedSize); transport->write((uint8_t*)&value32, 4);
				value32 = htonl(compressedSize);   transport->write((uint8_t*)&value32, 4);

				uint32_t bytesWritten = transport->write((uint8_t*)compressedImage, compressedSize);
				if (bytesWritten != compressedSize) {
					ERROR("Image transmission failed, wrote only %d out of %d bytes", bytesWritten, compressedSize);
				}
			} else
				compress = false;

			if (compressedImage != uncompressedImage) {
				free(compressedImage);
				compressedImage = 0;
			}
		}

		// if we still have 'compress' set to true, we must have succeeded
		if (compress) {
			INFO("Image sent");
			return true;
		}
	}

	uint32_t value32;
	value32 = htonl(imageSizeInBytes); transport->write((uint8_t*)&value32, 4);
	value32 = htonl(0);                transport->write((uint8_t*)&value32, 4);

	uint32_t bytesWritten = 0;
	while (bytesWritten < imageSizeInBytes) {
		int written = transport->write((uint8_t*)image->imageData + bytesWritten, imageSizeInBytes - bytesWritten);
		if (written < 0)
			return false;

		bytesWritten += written;
	}

	INFO("Sent image (uncompressed, %d bytes)", imageSizeInBytes);
	cvReleaseImage(&image);
	return true;
}
