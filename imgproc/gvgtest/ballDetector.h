/**
 * @author Tobias Langner
 * ball detection without lookup table
 */
#ifndef BALLDETECTOR_H_
#define BALLDETECTOR_H_

#include "blob.h"
#include "potentialBall.h"
#include "camera/camera_image.h"
#include "object.h"
#include "image.h"

#include <opencv/cxcore.h>
#include <vector>

class BallDetector {
public:
	BallDetector();
	~BallDetector();
	void operator()(IMAGETYPE const * image);

	std::vector<PotentialBall> const & getPotentialBalls() const {
		return mBallCandidates;
	}
	inline RectangleObject* getBall() const {
		return ball;
	}

	inline int getSaliency(int x, int y, int mode = 0) const {
		uint8_t red, green, blue;
		mImage->getPixelAsRGB(x, y, &red, &green, &blue, NONE);
		if (mode==0) {
			int const gb = (green/2) + (blue/2);
			if (red > 80 && blue < green && red > gb) {
				return 255 - gb * 255 / red;
			} else {
				return 0;
			}
		} else {

			int const MIN_RED_INTENSITY = 80;
			int const GREEN_THRESHOLD = 180;

			if (red < MIN_RED_INTENSITY) {
				return 0;
			}
			int const blue2 = blue*255/red/2;
			int const green2 = green*255/red/2;
			int value = 255 - green2 - blue2;
			// penalize if green is to large
			if (green2 > GREEN_THRESHOLD) {
				value -= (green2 - GREEN_THRESHOLD);
			}
			// penalize if blue is larger than 0.8*green
			if (4*green2 < 5*blue2) {
				value += (4*green2 - 5*blue2);
			}
			if (value >=0 && value <=255) {
				return value;
			} else {
				return 0;
			}
		}
	}

private:
	RectangleObject *ball;

	std::vector<Blob> mBlobs, mClusteredBlobs;
	std::vector<PotentialBall> mBallCandidates;
	std::vector<int> mLabels;
	Comp mCompareBlobs;


	IMAGETYPE const * mImage;

	void decideForBall();
};

#endif /* BALLDETECTOR_H_ */
