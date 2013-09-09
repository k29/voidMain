/** @file
 **
 */

#include "color.h"
#include "image.h"

#include <stdio.h>

#include <iostream>
#include <fstream>
#include <sstream>

#include "comm.h"
#include "robot.h"

using namespace de::fumanoids::message;


/*------------------------------------------------------------------------------------------------*/

/** Constructor
**/

ColorManager::ColorManager() {
	colorNames[Ball]       = "orange (ball)";
	colorNames[Field]      = "green (field)";
	colorNames[YellowGoal] = "yellow (goal)";
	colorNames[BlueGoal]   = "blue (goal)";
	colorNames[Magenta]    = "magenta";
	colorNames[Cyan]       = "cyan";
	colorNames[White]      = "white";
	colorNames[Obstacle]   = "black";

	lookupTable =   0;
}


/*------------------------------------------------------------------------------------------------*/

/** Destructor
**/

ColorManager::~ColorManager() {
}


/*------------------------------------------------------------------------------------------------*/

/** Find the correct LUT in the calibration data, or create a new one if none exists
**/

uint16_t ColorManager::getLUTIndex(de::fumanoids::message::Calibration &calibration) {
	int16_t yuv_index = -1;
	int16_t rgb_index = -1;

	// find YUV and RGB lookup tables
	for (int i=0; i < calibration.lut_size(); i++) {
		if (calibration.lut(i).bpc() == LUT_BPC) {
			if (calibration.lut(i).lutformat() == YUV)
				yuv_index = i;
			else if (calibration.lut(i).lutformat() == RGB)
				rgb_index = i;
		}
	}

	// if we found a suitable lookup table, let's use it
#if defined IMAGEFORMAT_RGB || defined IMAGEFORMAT_BAYER
	if (rgb_index != -1)
		return rgb_index;
#elif defined IMAGEFORMAT_YUV422
	if (yuv_index != -1)
		return yuv_index;
#else
#error "Unknown or unsupported color space."
#endif

	// if no suitable lookup table exists, create a new one
	LUT *lut = calibration.add_lut();
	lut->set_bpc(LUT_BPC);
	lut->set_compressed(false);
	lut->mutable_lut()->resize(256*256*256 / LUTCHANNELDIVISOR / LUTCHANNELDIVISOR / LUTCHANNELDIVISOR, Unknown);
#if defined IMAGEFORMAT_RGB || defined IMAGEFORMAT_BAYER
	lut->set_lutformat(RGB);
#elif defined IMAGEFORMAT_YUV422
	lut->set_lutformat(YUV);
#endif

	// if we did not find any lookup table, go on using the empty one
	if (rgb_index == -1 && yuv_index == -1) {
		WARNING("Found no suitable LUT");

	} else {
		// so we seem to have a calibraton, but of the wrong type. Maybe we can do a rough conversion
#if defined IMAGEFORMAT_RGB || defined IMAGEFORMAT_BAYER
		WARNING("Found YUV LUT, converting to RGB -> not really fast and not very good!");
		int lutIndex = yuv_index;
#elif defined IMAGEFORMAT_YUV422
		WARNING("Found RGB LUT, converting to YUV -> not really fast and not very good!");
		int lutIndex = rgb_index;
#endif
		ColorLookupTable *newLUT = (ColorLookupTable*)lut->mutable_lut()->c_str();
		ColorLookupTable *oldLUT = (ColorLookupTable*)calibration.mutable_lut(lutIndex)->mutable_lut()->c_str();
		for (int c1_old=0; c1_old<256; c1_old += LUTCHANNELDIVISOR) {
			for (int c2_old=0; c2_old<256; c2_old += LUTCHANNELDIVISOR) {
				for (int c3_old=0; c3_old<256; c3_old += LUTCHANNELDIVISOR) {
					Color c = (Color)(*oldLUT)[c1_old/LUTCHANNELDIVISOR][c2_old/LUTCHANNELDIVISOR][c3_old/LUTCHANNELDIVISOR];
					if (c != Unknown) {
						uint8_t c1_new, c2_new, c3_new;
#if defined IMAGEFORMAT_RGB || defined IMAGEFORMAT_BAYER
						ColorConverter::yuv2rgb(c1_old, c2_old, c3_old, &c1_new, &c2_new, &c3_new);
#elif defined IMAGEFORMAT_YUV422
						ColorConverter::rgb2yuv(c1_old, c2_old, c3_old, &c1_new, &c2_new, &c3_new);
#endif
						(*newLUT)[c1_new/LUTCHANNELDIVISOR][c2_new/LUTCHANNELDIVISOR][c3_new/LUTCHANNELDIVISOR] = c;
					}
				}
			}
		}

		// try to convert remaining
		for (int c1_new=0; c1_new<256; c1_new += LUTCHANNELDIVISOR) {
			for (int c2_new=0; c2_new<256; c2_new += LUTCHANNELDIVISOR) {
				for (int c3_new=0; c3_new<256; c3_new += LUTCHANNELDIVISOR) {
					if ((*newLUT)[c1_new/LUTCHANNELDIVISOR][c2_new/LUTCHANNELDIVISOR][c3_new/LUTCHANNELDIVISOR] == Unknown) {
						uint8_t c1_old, c2_old, c3_old;
#if defined IMAGEFORMAT_RGB || defined IMAGEFORMAT_BAYER
						ColorConverter::rgb2yuv(c1_new, c2_new, c3_new, &c1_old, &c2_old, &c3_old);
#elif defined IMAGEFORMAT_YUV422
						ColorConverter::yuv2rgb(c1_new, c2_new, c3_new, &c1_old, &c2_old, &c3_old);
#endif
						(*newLUT)[c1_new/LUTCHANNELDIVISOR][c2_new/LUTCHANNELDIVISOR][c3_new/LUTCHANNELDIVISOR] =
							(*oldLUT)[c1_old/LUTCHANNELDIVISOR][c2_old/LUTCHANNELDIVISOR][c3_old/LUTCHANNELDIVISOR];
					}
				}
			}
		}
	}

	return calibration.lut_size() - 1;
}


/*------------------------------------------------------------------------------------------------*/

/** Load lookup table
 **
 ** @param
**/

void ColorManager::load(de::fumanoids::message::Calibration &calibration) {
	lookupTable = (ColorLookupTable*)calibration.mutable_lut( getLUTIndex(calibration) )->mutable_lut()->c_str();
}

/*------------------------------------------------------------------------------------------------*/

/** Returns the RGB color values for a supported color.
 **
 ** @param color    Color to get RGB values for
 ** @param r        Will be set to the red value
 ** @param g        Will be set to the green value
 ** @param b        Will be set to the blue value
 */

void ColorManager::getRGBColor(Color color, uint8_t &r, uint8_t &g, uint8_t &b) const {
	switch (color) {
	case Ball:
		r = 255; g = b = 0;
		break;
	case Field:
		r = 0; g = 255; b = 0;
		break;
	case YellowGoal:
		r = 255; g = 255; b = 0;
		break;
	case BlueGoal:
		r = g = 0; b = 255;
		break;
	case Cyan:
		r = 0; g = 255; b = 255;
		break;
	case Magenta:
		r = 255; g = 0; b = 255;
		break;
	case Obstacle:
		r = 128; g = 128; b = 128;
		break;
	case White:
		r = 255; g = 255; b = 255;
		break;
	default:
		r = g = b = 0;
		break;
	}
}


/*------------------------------------------------------------------------------------------------*/

/** Returns the YUV color values for a supported color.
 **
 ** @param color    Color to get YUV values for
 ** @param y
 ** @param u
 ** @param v
 */

void ColorManager::getYUVColor(Color color, uint8_t &y, uint8_t &u, uint8_t &v) const {
	switch (color) {
	case Ball:
		y =  81; u =  90; v = 240;
		break;
	case Field:
		y = 145; u =  54; v =  34;
		break;
	case YellowGoal:
		y = 210; u =  16; v = 146;
		break;
	case BlueGoal:
		y =  41; u = 240; v = 110;
		break;
	case Cyan:
		y = 170; u = 166; v =  16;
		break;
	case Magenta:
		y = 106; u = 202; v = 222;
		break;
	case Obstacle:
		y = 128; u = 128; v = 128;
		break;
	default:
		y = u = v = 0;
		break;
	}
}
