/** @file
 **
 ** Class to access the image data.
 **
 ** It must be taken big care that any instance of this class is not used past
 ** its lifetime (data will become invalid as soon as the next image is captured)
 */

#ifndef IMAGEYUV422_H_
#define IMAGEYUV422_H_

#include "camera_image.h"
#include "math/Fixed.h"
#include "vision/colorConverter.h"

#include "transport/transport.h"

class ColorManager;

/*------------------------------------------------------------------------------------------------*/

class CameraImageYUV422 : public CameraImage {
	friend class Vision;

public:
	CameraImageYUV422(uint16_t _imageWidth, uint16_t _imageHeight)
		: CameraImage(_imageWidth, _imageHeight)
	{
	}

	virtual ~CameraImageYUV422() {}

	virtual ImageFormat getImageFormat() const {
		return IMAGE_YUV422;
	}

	virtual IplImage* getImageAsYUV(double scale=1) const {
		return CameraImage::getImageAsYUV(scale);
	}
	virtual IplImage* getImageAsYUV(uint16_t newImageWidth, uint16_t newImageHeight) const;
	virtual IplImage* getImageAsRGB(double scale=1, const ColorManager* colorMgr=0, bool bgr=true, PixelEnhanceMode enhanceMode=AUTO) const {
		return CameraImage::getImageAsRGB(scale, colorMgr, bgr, enhanceMode);
	}
	virtual IplImage* getImageAsRGB(uint16_t newImageWidth, uint16_t newImageHeight, const ColorManager* colorMgr=0, bool bgr=true, PixelEnhanceMode enhanceMode=AUTO) const;

	virtual bool sendRawImage(Transport *transport, bool compress) const;

	inline void getGradient(uint16_t xPos, uint16_t yPos, int16_t *grad_x, int16_t *grad_y, uint16_t *Mag, uint8_t scale) const {
		using namespace FixedPointMath;

		yPos += imageOffsetY;
		xPos += imageOffsetX;

		if (xPos<scale) xPos=scale;
		if (yPos<scale) yPos=scale;
		if (xPos>imageWidth-scale) xPos=imageWidth-scale;
		if (yPos>imageHeight-scale) yPos=imageHeight-scale;

		register uint32_t pos1 = xPos + yPos * fullImageWidth;

		/* calculates now the gradient with this method:
		 * http://en.wikipedia.org/wiki/Edge_detection#Other_first-order_methods
		 *
		 * -----------------------
		 * |      | 1/2up  |     |
		 * |------+--------+-----|
		 * | 1/2l |        |-1/2r|
		 * |------+--------+-----|
		 * |      |-1/2down|     |
		 * -----------------------
		 */

		register uint32_t pos = pos1 - scale*fullImageWidth;
		register uint32_t up = ((uint32_t*) currentData)[pos / 2];

		pos = pos1 + scale*fullImageWidth;
		register uint32_t down = ((uint32_t*) currentData)[pos / 2];

		pos = pos1 - scale;
		register uint32_t left = ((uint32_t*) currentData)[pos / 2];

		pos = pos1 + scale;
		register uint32_t right = ((uint32_t*) currentData)[pos / 2];


		int16_t grad_yx, grad_yy;

		if ((pos1 & 1) == 0 /* even */ ) {
			grad_yx = (-((right >> 16) & 255) + ((left >> 16) & 255)) / 2;
			grad_yy = (((up >> 16) & 255) - ((down >> 16) & 255)) / 2;

		} else {
			grad_yx = (-(right & 255) + (left & 255)) / 2;
			grad_yy = ((up & 255) - (down & 255)) / 2;
		}
		int16_t grad_ux = (-((right >> 8) & 255) + ((left >> 8) & 255)) / 2;
		int16_t grad_uy = (((up >> 8) & 255) - ((down >> 8) & 255)) / 2;

		int16_t grad_vx = (-((right >> 24) & 255) + ((left >> 24) & 255)) / 2;
		int16_t grad_vy = (((up >> 24) & 255) - ((down >> 24) & 255)) / 2;

		// magnitude for grad_y
		uint16_t mag_y = (uint16_t) (abs(grad_yx) + abs(grad_yy));

		// magnitude for grad_v
		uint16_t mag_v = (uint16_t) (abs(grad_vx) + abs(grad_vy));

		// magnitude for grad_u
		uint16_t mag_u = (uint16_t) (abs(grad_ux) + abs(grad_uy));

		*Mag = mag_y + mag_v + mag_u;

		if (mag_y > mag_v) {
			if (mag_y > mag_u) {
//				*Mag = mag_y;
				*grad_x = grad_yx;
				*grad_y = grad_yy;

			} else {
//				*Mag = mag_u;
				*grad_x = grad_ux;
				*grad_y = grad_uy;
			}
		} else {
			if (mag_v > mag_u) {
//				*Mag = mag_v;
				*grad_x = grad_vx;
				*grad_y = grad_vy;
			} else {
//				*Mag = mag_u;
				*grad_x = grad_ux;
				*grad_y = grad_uy;
			}
		}

		++pixelCounterGradient;
	}


	/** Retrieve the YUV values of a pixel
	 **
	 ** @param xPos        horizontal pixel position (in subset image)
	 ** @param yPos        vertical pixel position (in subset image)
	 ** @param y           Pointer to where to store the Y value
	 ** @param u           Pointer to where to store the U value
	 ** @param v           Pointer to where to store the V value
	 ** @param averageY    True iff the y value should be averaged between two pixels
	 */
	inline void getPixelAsYUV(uint16_t xPos, uint16_t yPos, uint8_t *y, uint8_t *u, uint8_t *v, bool averageY=false) const {
		yPos += imageOffsetY;
		xPos += imageOffsetX;

		if (xPos > imageWidth - 1) xPos = imageWidth - 1;
		if (yPos > imageHeight - 1) yPos = imageHeight - 1;

		register uint32_t pos = xPos + yPos * fullImageWidth;
		register uint32_t buffer = ((uint32_t*)currentData)[ pos / 2];

		if ( (pos & 1) == 0 /* even */) {
			*y = buffer;
			*u = buffer>>8;
			*v = buffer>>24;
		} else {
			*y = buffer>>16;
			*u = buffer>>8;
			*v = buffer>>24;
		}

		pixelCountergetPixelAsXXX += 1;

		if (averageY)
			*y = ((uint8_t)buffer + (uint8_t)(buffer>>16))/2;
	}


	/** Retrieve brightness of a pixel (Y value in YUV space)
	 **
	 ** @param xPos      x coordinate
	 ** @param yPos      y-coordinate
	 ** @param averageY  whether to average the brightness values in YUV422
	 **
	 ** @return brightness (0..255)
	 */

	inline uint8_t getPixelBrightness(uint16_t xPos, uint16_t yPos, bool averageY=false) {
		register uint32_t pos = (imageOffsetX + xPos) + (yPos + imageOffsetY) * fullImageWidth;
		register uint32_t buffer = ((uint32_t*)currentData)[ pos / 2];

		if (averageY)
			return ((uint8_t)buffer + (uint8_t)(buffer>>16))/2;
		else if ( (pos & 1) == 0 /* even */)
			return (uint8_t)buffer;
		else
			return (uint8_t)(buffer>>16);
	}



	/** Retrieve the RGB values of a pixel
	 **
	 ** @param xPos        horizontal pixel position (in subset image)
	 ** @param yPos        vertical pixel position (in subset image)
	 ** @param r           Pointer to where to store the R value
	 ** @param g           Pointer to where to store the G value
	 ** @param b           Pointer to where to store the B value
	 */
	inline void getPixelAsRGB(uint16_t xPos, uint16_t yPos, uint8_t *r, uint8_t *g, uint8_t *b, bool enhance=true) const {
		uint8_t y, u, v;
		getPixelAsYUV(xPos, yPos, &y, &u, &v);
		ColorConverter::yuv2rgb(y, u, v, r, g, b);
	}

	/** Retrieve the HSV values of a pixel
	 **
	 ** @param xPos        horizontal pixel position (in subset image)
	 ** @param yPos        vertical pixel position (in subset image)
	 ** @param h           Will be set to the H value
	 ** @param s           Will be set to the S value
	 ** @param v           Will be set to the V value
	 */
	inline void getPixelAsHSV(uint16_t xPos, uint16_t yPos, uint16_t& h, uint8_t& s, uint8_t& v, PixelEnhanceMode enhanceMode=AUTO) {
		uint8_t r, g, b;
		getPixelAsRGB(xPos, yPos, &r, &g, &b, enhanceMode);
		ColorConverter::rgb2hsv(r,g,b,h,s,v);
	}



protected:
};

#endif
