/** @file
 **
 ** Abstract base class to access the image data.
 **
 ** This class needs to be derived for the different image formats.
 **
 ** Note on image access
 ** --------------------
 **
 ** The image data should be accessed using accessor functions. It is recommended
 ** that each subclass implements the following functions:
 **
 **    inline void getPixelAsYUV(const uint16_t &xPos, const uint16_t &yPos, uint8_t *y, uint8_t *u, uint8_t *v) const;
 **    inline void getPixelAsRGB(const uint16_t &xPos, const uint16_t &yPos, uint8_t *r, uint8_t *g, uint8_t *b) const;
 **
 ** This will require that calling code is aware of the subclass type, so it breaks
 ** the abstraction quite a bit. Unfortunately the alternative to use a virtual function
 ** would have a huge impact on performance as inlining would not be possible. So, for
 ** performance-critical functions where we want inlining to occur, code will need to
 ** be aware of the actual image format. This should be kept to as few functions as possible.
 **
 ** Non-performance critical functions (i.e. where the overhead of a function call is minimal
 ** compared to the number of times the function is called or the complexity of the code)
 ** will be virtual.
 **
 ** Performance measurements:
 **    Retrieving and adding up the YUV values for each pixel of a 640x480 image, repeated
 **    100 times on a Gumstix 600 MHz, takes the following times:
 **           20 seconds - no function call, extracting in-place (RoboCup 2009 implementation)
 **           20 seconds - inlined function call (current implementation)
 **           27 seconds - direct function call
 **           30 seconds - virtual function call (ideal implementation for abstraction)
 **
 **
 ** Note on image size
 ** ------------------
 **
 ** Mainly due to the 2009 camera setup (stereo vision), the image class supports having the
 ** available image as a subset of the captured image (e.g. only one camera image of two
 ** recorded for stereo vision and spliced together).
 **
 ** For this reason, the following setup is used:
 **
 **    +-----------------------+
 **    | captured image        |
 **    |                       |
 **    |   +---------+         |
 **    |   | image   |         |
 **    |   |         |         |
 **    |   |         |         |
 **    |   +---------+         |
 **    |                       |
 **    +-----------------------+
 **
 ** Definitions:
 **    captured image = image retrieved from camera
 **    image = subset of captured image
 **
 */

#ifndef CAMERA_IMAGE_H_
#define CAMERA_IMAGE_H_

#include "transport/transport.h"

#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include <opencv/cv.h>

#include <string>

typedef enum {
	  IMAGE_YUV422 // class ImageYUV422
	, IMAGE_BAYER  // class ImageBayer
	, IMAGE_RGB    // class ImageRGB
} ImageFormat;

typedef enum {
	ORIGINAL = 0,
	NONE     = 0,
	ENHANCE  = 1,
	AUTO     = 2
} PixelEnhanceMode;

class ColorManager;

/*------------------------------------------------------------------------------------------------*/

class CameraImage {
	friend class Vision;

public:
	CameraImage(uint16_t _imageWidth, uint16_t _imageHeight);
	virtual ~CameraImage();

	/// Sets the subset of the image we are interested in
	virtual void setImageAsSubset(uint16_t offsetX, uint16_t offsetY, uint16_t subsetWidth, uint16_t subsetHeight);

	/// returns the type of image
	virtual ImageFormat getImageFormat() const = 0;

	/// return the width of the image (possibly subset of full image)
	inline uint16_t getImageWidth() const  { return imageWidth;  }

	/// return the height of the image (possibly subset of full image)
	inline uint16_t getImageHeight() const  { return imageHeight; }

	/// return the width of the full image
	inline uint16_t getFullImageWidth() const  { return fullImageWidth;  }

	/// return the height of the full image
	inline uint16_t getFullImageHeight() const  { return fullImageHeight; }

	/// return the x-offset into the full image
	inline uint16_t getOffsetX() const { return imageOffsetX; }

	/// return the y-offset into the full image
	inline uint16_t getOffsetY() const { return imageOffsetY; }

	/// return the x-coordinate of the image's center
	inline uint16_t getCenterX() const { return centerX; }

	/// return the y-coordinate of the image's center
	inline uint16_t getCenterY() const { return centerY; }

	/// get image data (points to full image!) (DO NOT USE!)
//	inline const void* getImageData() const { return currentData; }

	/// get length (in bytes) of the (full) image data (DO NOT USE!)
//	inline int getImageDataLength() const   { return currentDataLength; }

	/// set image
	virtual void setImage(void* newImageData, int newImageDataLength, int16_t newImageWidth=0, int16_t newImageHeight=0);

	/// free the memory
	void freeImageData();

	/// calculate the gradient
	virtual void getGradient(uint16_t xPos, uint16_t yPos , int16_t *grad_x, int16_t *grad_y, uint16_t *Mag, const uint8_t scale=1) const = 0;

	/// retrieve an OpenCV image
	virtual IplImage* getImageAsYUV(double scale=1) const;
	virtual IplImage* getImageAsYUV(uint16_t newImageWidth, uint16_t newImageHeight) const = 0;
	virtual IplImage* getImageAsRGB(double scale=1, const ColorManager* colorMgr=0, bool bgr=true, PixelEnhanceMode enhanceMode=AUTO) const;
	virtual IplImage* getImageAsRGB(uint16_t newImageWidth, uint16_t newImageHeight, const ColorManager* colorMgr=0, bool bgr=true, PixelEnhanceMode enhanceMode=AUTO) const = 0;
	virtual IplImage* getGradientImage(double scale) const;

	/// set/get image position in space
	void setImagePosition(uint16_t height, int16_t pitch, int16_t roll, int16_t headAngle);
	uint16_t getImagePositionHeight()    { return camera_height;    }
	int16_t  getImagePositionPitch()     { return camera_pitch;     }
	int16_t  getImagePositionRoll()      { return camera_roll;      }
	int16_t  getImagePositionHeadAngle() { return camera_headAngle; }

	/// save image
	virtual void saveImage(uint8_t scale=1, std::string path="", std::string filename="");

	virtual bool sendRawImage(Transport *transport, bool compress) const = 0;

	static uint32_t pixelCounterGradient;
	static uint32_t pixelCountergetPixelAsXXX;

	static PixelEnhanceMode autoPixelEnhanceMode;

protected:
	// size of the image (may be a subset of the captured image!)
	uint16_t imageWidth;
	uint16_t imageHeight;

	// if we are only interested in a subset of the captured image, this is the
	// offset into the captured image
	uint16_t imageOffsetX;
	uint16_t imageOffsetY;

	// size of the captured image
	uint16_t fullImageWidth;
	uint16_t fullImageHeight;

	// center of the image
	uint16_t centerX;
	uint16_t centerY;

	// point of view in space
	uint16_t camera_height;
	int16_t  camera_pitch;
	int16_t  camera_roll;
	int16_t  camera_headAngle;

	void* currentData;
	int   currentDataLength;
};

#endif
