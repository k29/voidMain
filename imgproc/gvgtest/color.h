/** @file
 **
 ** Class managing the color representation.
 ** @ingroup vision
 */

#ifndef __COLOR_H__
#define __COLOR_H__

#include "singleton.h"
#include "math/Fixed.h"
#include "comm/protobuf/msg_calibration.pb.h"
#include "fieldColorExtractor.h"
#include "image.h"

#include <inttypes.h>
#include <math.h>
#include <opencv/cv.h>

#include <map>
#include <string>
#include <fstream>


/*------------------------------------------------------------------------------------------------*/

typedef enum {
	  Unknown     = 0
	, Ball        = 1
	, Field       = 2
	, YellowGoal  = 3
	, BlueGoal    = 4
	, Obstacle    = 5
	, Cyan        = 6
	, Magenta     = 7
	, White       = 8
	, MAX_COLOR   = White
	, MIN_COLOR   = Ball
} Color;


// the lookup table can be set to assign a value to every single possible color triple
// by using a 256x256x256 table resulting in a 16 MB file; or it can discard the lower
// bits of each channel with almost no difference to the result but significantly smaller
// file sizes (discarding 2 bits per channel results in just a 0.25 MB file)

#define FULLCOLORLOOKUPTABLE 0

#if FULLCOLORLOOKUPTABLE
#define LUTCHANNELDIVISOR 1
#define LUT_BPC           8
#else
#define LUTCHANNELDIVISOR 4
#define LUT_BPC           6
#endif

typedef uint8_t ColorLookupTable[256/LUTCHANNELDIVISOR][256/LUTCHANNELDIVISOR][256/LUTCHANNELDIVISOR];


/*------------------------------------------------------------------------------------------------*/

class ColorManager {
public:
	ColorManager();
	virtual ~ColorManager();

	void load(de::fumanoids::message::Calibration &calibration);


	/*------------------------------------------------------------------------------------------------*/

	/**
	 **
	**/

	inline void setColor(uint8_t channel1, uint8_t channel2, uint8_t channel3, Color color) {
		if (color != Unknown)
			(*lookupTable)[channel1/LUTCHANNELDIVISOR][channel2/LUTCHANNELDIVISOR][channel3/LUTCHANNELDIVISOR] = color;
	}


	/*------------------------------------------------------------------------------------------------*/

	/**
	 ** Returns the color class to which the given color values are mapped, e.g
	 ** field, ball, obstacle etc.
	 **
	 ** @param channel1   Value of the first color channel (e.g. R in RGB or Y in YUV)
	 ** @param channel2   Value of the second color channel (e.g. G in RGB)
	 ** @param channel3   Value of the third color channel (e.g. V in YUV)
	 **
	 ** @return the color class
	 */

	inline Color getColor(uint8_t channel1, uint8_t channel2, uint8_t channel3) const {
		// first use lookup table
		Color c = (Color)(*lookupTable)[channel1/LUTCHANNELDIVISOR][channel2/LUTCHANNELDIVISOR][channel3/LUTCHANNELDIVISOR];

		if (FieldColorExtractor::getInstance().isActive()) {
			if (c == Field || c == Unknown || c == White || c == Obstacle) {
				// check if color is in field color range
				if (FieldColorExtractor::getInstance().isFieldColor(channel1, channel2, channel3))
					return Field;
				else if (c == Field)
					return Unknown; // ignore if LUT says Field
				else
					return c;
			}
		}

		return c;
	}


	/*------------------------------------------------------------------------------------------------*/

	/** Get the color class of a pixel in the image.
	 **
	 ** @param image    Image
	 ** @param xPos     x coordinate
	 ** @param yPos     y coordinate
	 **
	 ** @return color class of pixel (xPos,yPos) in image
	**/

	inline Color getPixelColor(const CameraImage &image, uint16_t xPos, uint16_t yPos) const {
		uint8_t channel1, channel2, channel3;

#if defined IMAGEFORMAT_BAYER or defined IMAGEFORMAT_RGB
		((IMAGETYPE&)image).getPixelAsRGB(xPos, yPos, &channel1, &channel2, &channel3);
#elif defined IMAGEFORMAT_YUV422
		((IMAGETYPE&)image).getPixelAsYUV(xPos, yPos, &channel1, &channel2, &channel3);
#else
#error "Unknown image type."
#endif
		return getColor(channel1, channel2, channel3);
	}

	void getRGBColor(Color color, uint8_t &r, uint8_t &g, uint8_t &b) const;
	void getYUVColor(Color color, uint8_t &y, uint8_t &u, uint8_t &v) const;

	std::map<Color, std::string> colorNames;

protected:
	ColorLookupTable *lookupTable;

	uint16_t getLUTIndex(de::fumanoids::message::Calibration &calibration);

private:
	std::string lutFileName;

};

#endif
