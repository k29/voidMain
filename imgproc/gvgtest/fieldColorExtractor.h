
/**
 * @{
 * @ingroup vision
 *
 ** Class for extracting the field color range from an image.
 **
 ** @file fieldColorExtractor.h
 ** @date 16.03.2011
 ** @author lisa
 */

#ifndef FIELDCOLOREXTRACTOR_H_
#define FIELDCOLOREXTRACTOR_H_


#include "singleton.h"
#include <inttypes.h>

struct IMAGETYPE; // forward declaration to prevent cyclic includes

/**
 * Class that extracts the most dominant greenish color range from the image and
 * classifies it as the field color. Greenish pixels are weighted higher than
 * other pixels to make sure to find the right maximum in the histogram.
 */
SINGLETON(FieldColorExtractor, void)
	public:
		virtual ~FieldColorExtractor();

		/**
		 * Sets the image to be worked with
		 * @param img
		 */
		inline void setImage(IMAGETYPE *img) {
				image = img;
		}

		bool isActive();
		bool isFieldColor(uint8_t c1, uint8_t c2, uint8_t c3);
		void extract();

	protected:
		IMAGETYPE *image;
		uint16_t *histo;
		uint16_t *histoMax;

		uint16_t colorLowThresh;    //!< smaller values are no field pixels
		uint16_t colorHighThresh;   //!< greater values are no field pixels
		int16_t saturation;         //!< mean saturation of image
		int16_t valueLowThresh;     //!< smaller brightness is no field pixel
		int16_t valueHighThresh;    //!< higher brightness is no field pixel
		int16_t meanBrightness;     //!< mean brightness of image
		int16_t colorMaxLevel;      //!< most dominant weighted color

		void createBrightnessHistogram();
		void createBrightnessHistogramYUV();
		void createBrightnessHistogramRGB();

		void createColorHistogram();

		uint16_t getWeightedRGB(uint8_t r, uint8_t g, uint8_t b);
		uint16_t getWeightedYUV(uint8_t y, uint8_t u, uint8_t v);
		uint16_t getWeightedColor(uint8_t c1, uint8_t c2, uint8_t c3);

		uint16_t getWeightedMaxLevel();
		int32_t stDev(uint16_t* histo, uint16_t length, uint16_t mean, uint16_t start);

		/**
		 * Approx saturation of given YUV color
		 * @param y The luma component
		 * @param u The first chrominance component
		 * @param v The second chrominance component
		 * @return saturation in [0,100]
		 */
		inline uint16_t getSaturation(uint8_t y, uint8_t u, uint8_t v) {
			uint16_t sat = FixedPointMath::isqrt((u-128)*(u-128)+(v-128)*(v-128));
			return sat;
		}

		/// max 4*256 histogram values vor color histogram
		static const uint16_t HISTO_COLOR_LENGTH = 1024;
		/// length for the brightness histogram
		static const uint16_t HISTO_VALUE_LENGTH = 256;
		/// Distance between sample points (used for x and y directions)
		static const uint8_t SAMPLE_DIST = 32;

};

/**
 * @}
 */

#endif /* FIELDCOLOREXTRACTOR_H_ */
