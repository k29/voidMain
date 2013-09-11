/** @file
 **
 ** Class to access the image data in RGB format.
 **
 ** It must be taken big care that any instance of this class is not used past
 ** its lifetime (data will become invalid as soon as the next image is captured)
 */

#ifndef IMAGERGB_H_
#define IMAGERGB_H_

#include "camera_image.h"
#include "vision/colorConverter.h"
#include "math/Fixed.h"

class ColorManager;


/*------------------------------------------------------------------------------------------------*/

class CameraImageRGB : public CameraImage {
	friend class Vision;

public:
	CameraImageRGB(uint16_t _imageWidth, uint16_t _imageHeight);

	virtual ~CameraImageRGB() {}

	virtual ImageFormat getImageFormat() const {
		return IMAGE_RGB;
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
	inline void getGradient(uint16_t xPos, uint16_t yPos , int16_t *grad_x, int16_t *grad_y, uint16_t *Mag, uint8_t scale=1) const {

		pixelCounterGradient++;

		xPos &= ~1;
		yPos &= ~1;
		scale &= ~1;
		if (xPos > imageWidth-scale)  xPos = imageWidth-scale;
		if (yPos > imageHeight-scale - 2) yPos = imageHeight-scale-2;

		if (scale<=2){
			uint8_t* pos = (uint8_t*)currentData + yPos * imageWidth;
			uint8_t g1, g2, g3, g4;

			g1 = *(pos + xPos + 1);
			g2 = *(pos + xPos + 3);
			g3 = *(pos + imageWidth*2 + xPos + 1);
			g4 = *(pos + imageWidth*2 + xPos + 3);

			int16_t grad_gx = ((int16_t)g1-g2+g3-g4);
			int16_t grad_gy = ((int16_t)g1+g2-g3-g4);

			uint16_t mag_g = (uint16_t) (abs(grad_gx) + abs(grad_gy));
			*Mag = mag_g;
			*grad_x = grad_gx;
			*grad_y = grad_gy;

			return;
		}

		/* calculates know the gradient with this method:
		 * http://en.wikipedia.org/wiki/Edge_detection#Other_first-order_methods
		 *
		 * -----------------------
		 * |      |  1/2up |     |
		 * |------+--------+-----|
		 * | 1/2l |        |-1/2r|
		 * |------+--------+-----|
		 * |      |-1/2down|     |
		 * -----------------------
		 */

		uint16_t up_rg, up_gb, left_rg, left_gb, right_rg, right_gb, down_rg, down_gb;

		uint32_t lineDifferenceWithScale = scale * imageWidth;
		uint8_t* pos = (uint8_t*)currentData + yPos * imageWidth + xPos;


		up_rg = *((uint16_t *)(pos - lineDifferenceWithScale));
		up_gb = *((uint16_t *)(pos - lineDifferenceWithScale + imageWidth));

		left_rg = *((uint16_t *)(pos - scale));
		left_gb = *((uint16_t *)(pos + imageWidth - scale));

		right_rg = *((uint16_t *)(pos + scale));
		right_gb = *((uint16_t *)(pos + imageWidth + scale));

		down_rg = *((uint16_t *)(pos + lineDifferenceWithScale));
		down_gb = *((uint16_t *)(pos + lineDifferenceWithScale + imageWidth));

		int16_t grad_rx = (-(int16_t) (right_rg & 255) + (int16_t) (left_rg & 255)) / 2;
		int16_t grad_ry = ((int16_t) (up_rg & 255)	- (int16_t) (down_rg & 255)) / 2;

		int16_t grad_gx = (-(int16_t) (right_gb & 255) + (int16_t) (left_gb & 255)) / 2;
		int16_t grad_gy = ((int16_t) (up_gb & 255) - (int16_t) (down_gb & 255)) / 2;

		int16_t grad_bx = (-(int16_t) ((right_gb >> 8) & 255) + (int16_t) ((left_gb >> 8) & 255)) / 2;
		int16_t grad_by = ((int16_t) ((up_gb >> 8) & 255)	- (int16_t) ((down_gb >> 8) & 255)) / 2;

		int16_t intensityDiffy = 0;//(grad_ry+grad_gy+grad_by)/3;
		int16_t intensityDiffx = 0;//(grad_rx+grad_gx+grad_bx)/3;

		// magnitude for grad_r
//		uint16_t mag_r = (uint16_t) FixedPointMath::isqrt(grad_rx * grad_rx + grad_ry * grad_ry);
		uint16_t mag_r = (uint16_t) (abs(grad_rx-intensityDiffx) + abs(grad_ry-intensityDiffy));

		// magnitude for grad_g
//		uint16_t mag_g = (uint16_t) FixedPointMath::isqrt(grad_gx * grad_gx + grad_gy * grad_gy);
		uint16_t mag_g = (uint16_t) (abs(grad_gx-intensityDiffx) + abs(grad_gy-intensityDiffy));

		// magnitude for grad_b
//		uint16_t mag_b = (uint16_t) FixedPointMath::isqrt(grad_bx * grad_bx + grad_by * grad_by);
		uint16_t mag_b = (uint16_t) (abs(grad_bx-intensityDiffx) + abs(grad_by-intensityDiffy));

//		*Mag = mag_g*4+mag_b+mag_r;
//		*grad_x = grad_gx+grad_bx+grad_rx;
//		*grad_y = grad_gy+grad_by+grad_ry;

		if (mag_g > mag_r) {
			if (mag_g > mag_b) {
				*Mag = mag_g;
				*grad_x = grad_gx;
				*grad_y = grad_gy;

			} else {
				*Mag = mag_b;
				*grad_x = grad_bx;
				*grad_y = grad_by;
			}
		} else {
			if (mag_r > mag_b) {
				*Mag = mag_r;
				*grad_x = grad_rx;
				*grad_y = grad_ry;
			} else {
				*Mag = mag_b;
				*grad_x = grad_bx;
				*grad_y = grad_by;
			}
		}

	}

	/** Retrieve the YUV values of a pixel
	 **
	 ** @param xPos        horizontal pixel position (in subset image)
	 ** @param yPos        vertical pixel position (in subset image)
	 ** @param y           Pointer to where to store the Y value
	 ** @param u           Pointer to where to store the U value
	 ** @param v           Pointer to where to store the V value
	 ** @param averageY    (unused)
	 */
	inline void getPixelAsYUV(uint16_t xPos, uint16_t yPos, uint8_t *y, uint8_t *u, uint8_t *v, bool averageY=false) const {
		uint8_t r, g, b;
		getPixelAsRGB(xPos, yPos, &r, &g, &b);
		ColorConverter::rgb2yuv(r,g,b,y,u,v);
	}


	/** Retrieve brightness of a pixel (Y value in YUV space)
	 **
	 ** @param xPos      x coordinate
	 ** @param yPos      y-coordinate
	 ** @param averageY  (unused)
	 **
	 ** @return brightness (0..255)
	 */

	inline uint8_t getPixelBrightness(uint16_t xPos, uint16_t yPos, bool averageY=false) {
		uint8_t r, g, b;
		getPixelAsRGB(xPos, yPos, &r, &g, &b);

		using namespace FixedPointMath;
		register fixedpoint r_fixed = toFixed(r);
		register fixedpoint g_fixed = toFixed(g);
		register fixedpoint b_fixed = toFixed(b);
		return toInt(fmul(r_fixed, toFixed(0.299)) + fmul(g_fixed, toFixed(0.587)) + fmul(b_fixed, toFixed(0.114)));
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


	/** Retrieve the RGB values of a pixel
	 **
	 ** @param xPos        horizontal pixel position (in subset image)
	 ** @param yPos        vertical pixel position (in subset image)
	 ** @param r           Pointer to where to store the R value
	 ** @param g           Pointer to where to store the G value
	 ** @param b           Pointer to where to store the B value
	 */

	inline void getPixelAsRGB(uint16_t xPos, uint16_t yPos, uint8_t *r, uint8_t *g, uint8_t *b, PixelEnhanceMode enhanceMode=AUTO) const {
		uint8_t* pos = (uint8_t*)currentData + 3 * (yPos * imageWidth + xPos);

		*r = pos[0];
		*g = pos[1];
		*b = pos[2];

		if (enhanceMode == NONE || (enhanceMode == AUTO && autoPixelEnhanceMode == NONE))
			return;

		int tr = *r, tg = *g, tb = *g;

		int intensity = ((int)tr + tg + tb)/3;
		tr = (int)tr * 3 - 2*intensity;
		tg = (int)tg * 3 - 2*intensity;
		tb = (int)tb * 3 - 2*intensity;
		if (tr>255) tr=255;
		if (tr<0) tr=0;
		if (tg>255) tg=255;
		if (tg<0) tg=0;
		if (tb>255) tb=255;
		if (tb<0) tb=0;
		*r=tr;
		*g=tg;
		*b=tb;
	}

protected:
};

#endif
