#ifndef COLOR_CONVERTER_H__
#define COLOR_CONVERTER_H__

#include "math/Fixed.h"
#include <inttypes.h>


/*------------------------------------------------------------------------------------------------*/

/**
 * Converts colors between different color spaces
 * @ingroup vision
 */
class ColorConverter {
public:
	/** convert a yuv value to an rgb one
	 *
	 * @param y
	 * @param u
	 * @param v
	 * @param r
	 * @param g
	 * @param b
	 */
	static inline void yuv2rgb(
				const uint8_t &y, const uint8_t &u, const uint8_t &v,
				uint8_t *r, uint8_t* g, uint8_t *b
			)
	{
		int dr = 1000 + (351*v/256) - 175;
		int dg = 1000 - (179*v/256) + 90 - (86*u/256) + 43;
		int db = 1000 + (443*u/256) - 222;

		int r1 = y + dr;
		int g1 = y + dg;
		int b1 = y + db;

		if (b1<1000) b1 = 1000;
		if (b1>1255) b1 = 1255;
		if (g1<1000) g1 = 1000;
		if (g1>1255) g1 = 1255;
		if (r1<1000) r1 = 1000;
		if (r1>1255) r1 = 1255;

		*b = b1-1000;
		*g = g1-1000;
		*r = r1-1000;
	}

	/**
	 * Converts a rgb color to a yuv color
	 * @param r
	 * @param g
	 * @param b
	 * @param y
	 * @param u
	 * @param v
	 */
	static inline void rgb2yuv(
			const uint8_t &r, const uint8_t &g, const uint8_t &b,
			uint8_t *y, uint8_t *u, uint8_t *v)
	{
		using namespace FixedPointMath;
		register fixedpoint r_fixed = toFixed(r);
		register fixedpoint g_fixed = toFixed(g);
		register fixedpoint b_fixed = toFixed(b);

		// Old conversion: is actually rgb to YCbCr!
//		*y = toInt(toFixed( 16) + fmul(r_fixed, toFixed(0.257)) + fmul(g_fixed, toFixed(0.504)) + fmul(b_fixed, toFixed(0.098)));
//		*u = toInt(toFixed(128) - fmul(r_fixed, toFixed(0.148)) - fmul(g_fixed, toFixed(0.291)) + fmul(b_fixed, toFixed(0.439)));
//		*v = toInt(toFixed(128) + fmul(r_fixed, toFixed(0.439)) - fmul(g_fixed, toFixed(0.368)) - fmul(b_fixed, toFixed(0.071)));

		/*
		 * Current conversion rgb to yuv
		 * @see http://softpixel.com/~cwright/programming/colorspace/yuv/
		 * Y = R *  .299 + G *  .587 + B *  .114
		 * U = R * -.169 + G * -.331 + B *  .500 + 128
		 * V = R *  .500 + G * -.418 + B * -.081 + 128
		 *
		 * seems to be inverse to our current yuv2rgb()
		 */

		*y = toInt(               fmul(r_fixed, toFixed(0.299)) + fmul(g_fixed, toFixed(0.587)) + fmul(b_fixed, toFixed(0.114)));
		*u = toInt(toFixed(128) - fmul(r_fixed, toFixed(0.169)) - fmul(g_fixed, toFixed(0.331)) + fmul(b_fixed, toFixed(0.500)));
		*v = toInt(toFixed(128) + fmul(r_fixed, toFixed(0.500)) - fmul(g_fixed, toFixed(0.418)) - fmul(b_fixed, toFixed(0.081)));
	}

	/**
	 * Convert RGB to HSV (Hue, Saturation, Value)
	 * @param r - red value
	 * @param g - green value
	 * @param b - blue value
	 * @param h - is set to the hue [0,360)
	 * @param s - is set to the saturation [0,100]
	 * @param v - is set to the value [0,100]
	 */
	static inline void rgb2hsv(const uint8_t &r, const uint8_t &g, const uint8_t &b,
			uint16_t &h, uint8_t &s, uint8_t &v) {
		using namespace FixedPointMath;

		fixedpoint red   = fdiv(toFixed(r),toFixed(255));
		fixedpoint green = fdiv(toFixed(g),toFixed(255));
		fixedpoint blue  = fdiv(toFixed(b),toFixed(255));
		fixedpoint max = std::max(std::max(red,green),blue);
		fixedpoint min = std::min(std::min(red,green),blue);
		fixedpoint delta = max-min;
		fixedpoint h_fixed = 0;

		if (delta == 0) h_fixed = 0;
		else if (red   == max)  h_fixed = fmul(fdiv(green-blue,delta), toFixed(60));
		else if (green == max)  h_fixed = fmul(toFixed(2) + fdiv(blue-red,delta), toFixed(60));
		else if (blue  == max)  h_fixed = fmul(toFixed(4) + fdiv(red-green,delta),toFixed(60));

		int16_t h_int = toInt(h_fixed);

		if (h_int < 0) h = h_int + 360;
		else h = h_int;

		fixedpoint s_fixed = (max == 0)? 0 : fdiv(delta, max);

		s = toInt(fmul(s_fixed, toFixed(100)));
		v = toInt(fmul(max, toFixed(100)));
	}
};


#endif
