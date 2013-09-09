/*
 *  fieldColorExtractor.cpp
 *
 *  Created on: 16.03.2011
 * 	    Author: lisa
 */

#include "image.h"
#include "fieldColorExtractor.h"
#include "debug.h"
#include "math/fixedPointMath.h"
#include "config/configRegistry.h"
#include "vision.h"
#include "colorConverter.h"

// for debugging: use rgb with image format yuv
#define USE_RGB true
#define USE_YUV false

REGISTER_OPTION("vision.fieldcolorextractor",1,"Activate FieldColorExtractor");

/**
 * Constructor. Create histograms.
 * Only called once, because of singleton pattern.
 */
FieldColorExtractor::FieldColorExtractor() {
	image = 0;
	histo = new uint16_t[HISTO_COLOR_LENGTH];
	histoMax = new uint16_t[HISTO_VALUE_LENGTH];
}

/**
 * Destructor. Delete histograms.
 */
FieldColorExtractor::~FieldColorExtractor() {
	image = 0;
	delete[] histo;
	delete[] histoMax;
	histo = 0;
	histoMax = 0;
}

/**
 * Computes the color threshold values and caches them for later use in
 * FieldColorExtractor::isFieldColor()
 */
void FieldColorExtractor::extract() {
	createBrightnessHistogram();
	createColorHistogram();
	colorMaxLevel = getWeightedMaxLevel();

	using namespace FixedPointMath;
	uint16_t stDevColor = stDev(histo, HISTO_COLOR_LENGTH, colorMaxLevel, colorMaxLevel-50);
	int32_t thresh = toInt(fmul(toFixed(stDevColor), toFixed(2.5)));

	// cache the thresholds, define lower and upper bounds for thresholds
	thresh = std::min(std::max(thresh, 10), 192);
	colorLowThresh = std::max(colorMaxLevel - thresh, 10);
	colorHighThresh = colorMaxLevel + thresh;
}

void FieldColorExtractor::createBrightnessHistogram() {
	#if USE_YUV
		createBrightnessHistogramYUV();
	#elif USE_RGB
		createBrightnessHistogramRGB();
	#else
	#error "Unknown image type."
	#endif
}

/**
 * Create histogram of image value ("value" in the sense of HSV) to estimate
 * the mean brightness of the image
 */
void FieldColorExtractor::createBrightnessHistogramRGB() {
	//clear histogramm
	memset(histoMax, 0, sizeof(uint16_t) * HISTO_VALUE_LENGTH);
	uint32_t sumMax = 0, sumMin = 0, pixels = 0;

	for(uint16_t y = SAMPLE_DIST; y < image->getImageHeight(); y+=SAMPLE_DIST) {
		uint16_t x_start, x_end;
		Vision::getInstance().getLineInCenterCircle(y,&x_start,&x_end);

		for(uint16_t x = x_start; x < x_end; x+=SAMPLE_DIST) {
			uint8_t r,g,b, maxVal, minVal;
			image->getPixelAsRGB(x, y, &r, &g, &b);
			maxVal = ::max(r,g,b);
			minVal = ::min(r,g,b);

#if defined ROBOT2011
			if(g < b || g < b-35 || maxVal < 20 || maxVal > 220 || maxVal-minVal < 20)
				continue;
#endif

			sumMax += maxVal;
			sumMin += minVal;
			++histoMax[maxVal];
			++pixels;
		}
	}
	uint16_t meanOfMax = (pixels == 0)? 0: sumMax/pixels;
	uint16_t meanOfMin = (pixels == 0)? 0: sumMin/pixels;
	uint16_t stDevMax = stDev(histoMax, HISTO_VALUE_LENGTH, meanOfMax, 0);

	// define thresholds for saturation and min and max lightness
	using namespace FixedPointMath;
	saturation = (meanOfMax-meanOfMin);
	valueLowThresh = meanOfMax-2*stDevMax;
	valueHighThresh = toInt(fmul(toFixed(2.0),toFixed(stDevMax))) + meanOfMax;
	meanBrightness = meanOfMax;
}

/**
 * Create histogram of y component to estimate the mean brightness of the image.
 */
void FieldColorExtractor::createBrightnessHistogramYUV() {
	memset(histoMax, 0, sizeof(uint16_t) * HISTO_VALUE_LENGTH);
	uint32_t sumBright = 0, sumSat = 0, pixels = 0;

	for(uint16_t yPos = SAMPLE_DIST; yPos < image->getImageHeight(); yPos+=SAMPLE_DIST) {
		uint16_t x_start, x_end;
		Vision::getInstance().getLineInCenterCircle(yPos,&x_start,&x_end);

		for(uint16_t xPos = x_start; xPos < x_end; xPos+=SAMPLE_DIST) {
			uint8_t y,u,v;
			image->getPixelAsYUV(xPos, yPos, &y, &u, &v);

			if(u < 150 && v < 128) {
				sumBright += y;
				sumSat += getSaturation(y,u,v);
				++histoMax[y];
				++pixels;
			}
		}
	}

	uint16_t meanBright = (pixels == 0)? 0: sumBright/pixels;
	uint16_t meanSat    = (pixels == 0)? 0: sumSat/pixels;
	uint16_t stDevBright = stDev(histoMax, HISTO_VALUE_LENGTH, meanBright, 0);

	using namespace FixedPointMath;
	saturation = meanSat;
	valueLowThresh = meanBright-stDevBright;
	valueHighThresh = toInt(fmul(toFixed(1.5),toFixed(stDevBright))) + meanBright;
	meanBrightness = meanBright;
}

/**
 * Create histogram of greenish like colors in the image. The closer a
 * color of a pixel is to green, the more it is placed to the right of the
 * histogram (higher value).
 */
void FieldColorExtractor::createColorHistogram() {
	// clear histogram
	memset(histo, 0, sizeof(uint16_t) * HISTO_COLOR_LENGTH);

	for(uint16_t y = SAMPLE_DIST; y < image->getImageHeight(); y+=SAMPLE_DIST) {
		uint16_t x_start, x_end;
		Vision::getInstance().getLineInCenterCircle(y,&x_start,&x_end);

		for(uint16_t x = x_start; x < x_end; x+=SAMPLE_DIST) {
			uint8_t c1=0,c2=0,c3=0;
			uint16_t weightedVal = 0;
			#if USE_YUV
				image->getPixelAsYUV(x, y, &c1, &c2, &c3);
				weightedVal = getWeightedYUV(c1,c2,c3);
			#elif USE_RGB
				image->getPixelAsRGB(x, y, &c1, &c2, &c3);
				weightedVal = getWeightedRGB(c1,c2,c3);
			#else
			#error "Unknown image type."
			#endif

			if (weightedVal > 0)
				++histo[weightedVal];
		}
	}
}

/**
 * Returns the most common value in the histogram. The bigger the value, the
 * closer it is to pure green and thus it is weighted stronger.
 */
uint16_t FieldColorExtractor::getWeightedMaxLevel() {
	uint32_t maxLevel = 0, maxVal = 0;

	for (uint16_t i=0; i < HISTO_COLOR_LENGTH; ++i) {
		// the larger the value the better
		if ((uint32_t)(histo[i]*i) > maxVal) {
			maxLevel = i;
			maxVal = histo[i]*i;
		}
	}
	return maxLevel;
}


/**
 * Returns a weighted sum of the given color components according to their
 * color space. Currently only RGB and YUV are supported.
 * @param c1 The first color component
 * @param c2 The second color component
 * @param c3 The third color component
 * @return The weighted sum. Higher value means closer to green.
 */
uint16_t FieldColorExtractor::getWeightedColor(uint8_t c1, uint8_t c2, uint8_t c3) {

	#if USE_YUV and (defined IMAGEFORMAT_BAYER or defined IMAGEFORMAT_RGB)
		// convert to yuv
		uint8_t y,u,v;
		ColorConverter::rgb2yuv(c1,c2,c3,&y,&u,&v);
		return getWeightedYUV(y,u,v);
	#elif USE_YUV and defined IMAGEFORMAT_YUV422
		return getWeightedYUV(c1,c2,c3);
	#elif USE_RGB and defined IMAGEFORMAT_YUV422
		// convert to rgb
		uint8_t r,g,b;
		ColorConverter::yuv2rgb(c1,c2,c3,&r,&g,&b);
		return getWeightedRGB(r,g,b);
	#elif USE_RGB
		return getWeightedRGB(c1,c2,c3);
	#else
	#error "Unknown image type."
	#endif
}

/**
 * Returns the weighted sum of the RGB components. The closer the pixel's color
 * is to pure green the higher is the returned weight. Pixels that are unlikely to
 * belong to the field color are filtered out beforehand.
 * @param r The red component
 * @param g The green component
 * @param b The blue component
 * @return weight in [0, 893]
 */
uint16_t FieldColorExtractor::getWeightedRGB(uint8_t r, uint8_t g, uint8_t b) {

	uint8_t max = std::max(std::max(r,g),b);
	uint8_t min = std::min(std::min(r,g),b);

	// ignore pixels without enough color information
	if (max < 20 || max > 220) return 0;

	// allow more green-blue shades than green-red shades
	if (g < r || g < b-35) {
		return 0;
	}

	// ignore gray, very dark and very light pixels
	if((max-min < saturation && (max < 96 || max > 192))
			|| max < valueLowThresh
			|| max > valueHighThresh)
		return 0;

	// weigh small r-values and small b-values high and large g-values high
	uint8_t weightedRed = (r < 128)? 128 : 255-r;  //[0,255] -> [128,0]
	uint16_t weightedGreen = std::max((g-r) + (g-b), 0); //[510,0]
	uint8_t weightedBlue = (b > 128)? 128 : 255-b; //[0,255] -> [255,128]

	return weightedRed + weightedGreen + weightedBlue;
}

/**
 * IMPORTANT: don't use this right now, RGB yields better results
 * -> set USE_RGB to true, even for YUV images.
 *
 * Returns the weighted sum of the YUV components. The closer the pixel's color
 * is to pure green the higher is the returned weight.
 * @param y
 * @param u
 * @param v
 * @return weight in [0,893]
 */
uint16_t FieldColorExtractor::getWeightedYUV(uint8_t y, uint8_t u, uint8_t v) {
	if (u > 150 || v > 128) return 0;
	uint16_t sat = getSaturation(y,u,v);

	// ignore pixels without enough color information
	if((sat < saturation && (y < meanBrightness-20 || y > meanBrightness+20))
			|| y < valueLowThresh
			|| y > valueHighThresh)
		return 0;

	uint16_t weightedU = ::max(0,128-u);     // smaller u -> closer to green, larger weight
	uint16_t weightedV = ::max(0,128-v) * 2; // smaller v -> closer to green, larger weight
	uint16_t weightedY = 255 - abs(128-y);
	uint16_t uvLength = (abs(u-128) + abs(v-128));

	return weightedU + weightedV + weightedY + uvLength;
}

/**
 * Computes the standard deviation for the histogram with the given mean.
 * @param histo The histogram
 * @param length The number of bins in the histogram
 * @param mean The mean of the histogram, i.e. the most common value
 * @param start Where to start in the histogram [start, histo.length]
 * @return The standard deviation
 */
int32_t FieldColorExtractor::stDev(uint16_t* histo, uint16_t length, uint16_t mean, uint16_t start) {
	uint32_t pixels = 0;
	uint32_t sum = 0;

	for (int16_t i = start; i < length; ++i) {
		if (i < 0) continue;
		pixels += histo[i];
		sum += histo[i] * (i-mean) * (i-mean);
	}

	if(pixels == 0)	return 0;
	return FixedPointMath::isqrt(sum/pixels);
}

/**
 * Checks if the FieldColorExtractor is active.
 * @return True, if the FieldColorExtractor is activated in the config and
 * false otherwise.
 */
bool FieldColorExtractor::isActive() {
	return Vision::getInstance().activeFieldColorExtractor == 1;
}

/**
 * Returns true, if the given color is classified as field color by the
 * FieldColorExtractor. That is the case if the additive combined value lies
 * between lowThresh and highThresh. The combined value is returned by
 * @see FieldExtractor::getWeightedColor().
 * @return True, if (c1,c2,c3) is field color
 */
bool FieldColorExtractor::isFieldColor(uint8_t c1, uint8_t c2, uint8_t c3) {
	uint16_t weightedColor = getWeightedColor(c1,c2,c3);
	return weightedColor >= colorLowThresh && weightedColor <= colorHighThresh;
}
