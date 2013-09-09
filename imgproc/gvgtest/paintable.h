#ifndef PAINTABLE_H_
#define PAINTABLE_H_

#include <opencv/cv.h>
#include "opencv/highgui.h"

static CvScalar blue    = cvScalar(255,   0,   0);
static CvScalar red     = cvScalar(  0,   0, 255);
static CvScalar yellow  = cvScalar(  0, 255, 255);
static CvScalar orange  = cvScalar(  0, 163, 255);
static CvScalar black   = cvScalar(  0,   0,   0);
static CvScalar magenta = cvScalar(255,   0, 255);
static CvScalar cyan    = cvScalar(255, 255,   0);
static CvScalar white   = cvScalar(255, 255, 255);
static CvScalar green   = cvScalar(  0, 255,   0);
static CvScalar gray    = cvScalar(128, 128, 128);

/**
 * Abstract class defining a paintable object.
 *
 * @ingroup vision
 */
class Paintable {
public:

	virtual ~Paintable() {}

	/**
	 * Paints the object an the given image.
	 *
	 * @param img
	 * @param scale scaling factor
	 */
	virtual void paint(IplImage *img, int scale) = 0;
};

#endif /* PAINTABLE_H_ */
