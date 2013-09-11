#include "ballDetector.h"
#include "objectExtractor_field.h"
#include "position.h"
#include "vision.h"

#include "partition.h"

#include "debug.h"

#include <vector>

#include <algorithm>

BallDetector::BallDetector() {
	ball = new RectangleObject(BALL_OBJECT, Ball);
}

BallDetector::~BallDetector() {
	delete ball;
	ball = 0;
}

void BallDetector::operator()(IMAGETYPE const * image) {
	mImage = image;

	// detect blobs
	int const INTENSITY_THRESHOLD = 60;
	int const MAGNITUDE_THRESHOLD = 20;
	mBlobs.clear();

	int minStep = 4;
	int maxStep = 32;
	int const width  = image->getImageWidth();
	int const height = image->getImageHeight();
	int counter = 0;
	bool f = true;
	for (int y=minStep; y<height-minStep; y+=minStep) {
		if (f && y>height/2) {
			minStep = 8;
			maxStep = 32;
			f = false;
		}
		for (int x=minStep; x<width-minStep; x+=minStep) {
			int val = getSaliency(x, y, 0);
			if (val >= INTENSITY_THRESHOLD) {
				counter++;
				int s = 0;
				for (s=minStep; s<=maxStep; s*=2) {
					int top = getSaliency(x, y-s, 0);
					int left = getSaliency(x-s, y, 0);
					int right = getSaliency(x+s, y, 0);
					int bottom = getSaliency(x, y+s, 0);
					int const max1 = left > right ? left : right;
					int const max2 = top > bottom ? top : bottom;
					int const min = max1 > max2 ? val - max1 : val - max2;
					if (min >= MAGNITUDE_THRESHOLD) {
						mBlobs.push_back(Blob(x, y, min, val, s));
					}
				}
			}
		}
	}

	mLabels.clear();
	mClusteredBlobs.clear();
	int n = partition(mBlobs, mLabels, mCompareBlobs);

	// FIXME opencv 1 doesn't knows this ...
//	int n = cv::partition(mBlobs, mLabels, mCompareBlobs);
	mClusteredBlobs.resize(n);
	for (unsigned int i=0; i<mLabels.size(); ++i) {
		int index = mLabels[i];
		if (mBlobs[i].minMagnitude > mClusteredBlobs[index].minMagnitude) {
			mClusteredBlobs[index] = mBlobs[i];
		}
	}

	// sort blobs and pick the n best
	unsigned int const MAX_BLOBS = 20;
	if (mClusteredBlobs.size() <= MAX_BLOBS) {
		std::sort(mClusteredBlobs.begin(), mClusteredBlobs.end(), std::greater<Blob>());
	} else {
		std::partial_sort(mClusteredBlobs.begin(), mClusteredBlobs.begin() + MAX_BLOBS, mClusteredBlobs.end(), std::greater<Blob>());
		mClusteredBlobs.resize(MAX_BLOBS);
	}

	// refine location and radius of the blobs
	mBallCandidates.clear();
	unsigned int const MAX_BALLS = 4;
	int const INTENSITY_THRESH = 50;
	int const MIN_RADIUS = 1;
	std::vector<Blob>::const_iterator it = mClusteredBlobs.begin();
	for (; it != mClusteredBlobs.end() && mBallCandidates.size() < MAX_BALLS; ++it) {

		int const x0 = it->x;
		int const y0 = it->y;
		int scale = it->scale;
		int xmin = std::max(0, x0 - scale);
		int xmax = std::min(image->getImageWidth()-1, x0 + scale);
		int ymin = std::max(0, y0 - scale);
		int ymax = std::min(image->getImageHeight()-1, y0 + scale);
		int x1 = x0;
		int x2 = x0;
		int y1 = y0;
		int y2 = y0;
		while (x1 >= xmin) {
			int val = getSaliency(x1, y0, 0);
			if (val > INTENSITY_THRESH) {
				--x1;
			} else {
				break;
			}
		}
		while (x2 <= xmax) {
			int val = getSaliency(x2, y0, 0);
			if (val > INTENSITY_THRESH) {
				++x2;
			} else {
				break;
			}
		}
		while (y1 >= ymin) {
			int val = getSaliency(x0, y1, 0);
			if (val > INTENSITY_THRESH) {
				--y1;
			} else {
				break;
			}
		}
		while (y2 <= ymax) {
			int val = getSaliency(x0, y2, 0);
			if (val > INTENSITY_THRESH) {
				++y2;
			} else {
				break;
			}
		}

		int const r = ((x2-x1) + (y2-y1)) / 4;
		if (x1 < x2 && y1 < y2 && r >= MIN_RADIUS) {
			mBallCandidates.push_back(PotentialBall((x1+x2)/2, (y1+y2)/2, r));
		}
	}

	// decide for right ball out of the candidates
	decideForBall();

}

void BallDetector::decideForBall() {

	for (uint32_t i = 0; i < mBallCandidates.size(); ) {

		// check whether it overlaps with any other ball candidates (if yes, ...?)
		for (uint32_t j = i + 1; j < mBallCandidates.size(); ) {
			PotentialBall &firstBall  = mBallCandidates[i];
			PotentialBall &secondBall = mBallCandidates[j];

			int dx = firstBall.x - secondBall.x;
			int dy = firstBall.y - secondBall.y;
			int delta = (int)sqrt(dx*dx + dy*dy);

			if ( delta < firstBall.radius + secondBall.radius) {
				firstBall.x += dx/2;
				firstBall.y += dy/2;
				firstBall.radius = delta/2 + std::max(firstBall.radius, secondBall.radius);

				mBallCandidates.erase( mBallCandidates.begin() + j );

				continue;
			}

			// next
			j++;
		}

		// check whether ball candidate is in field (if not, remove)
		if (Vision::getInstance().getFieldExtractor().getHeighestFieldCoordinate(mBallCandidates[i].x) > mBallCandidates[i].y) {
			mBallCandidates.erase( mBallCandidates.begin() + i );
			continue;
		}

		// check ball size
		// TODO

		// next ball
		i++;
	}

	// get closest ball
	int closestDistance = INT_MAX;
	int selectedBallIndex = -1;
	for (uint32_t i=0; i < mBallCandidates.size(); ++i) {
		PositionImage ballPosition( mBallCandidates[i].x, mBallCandidates[i].y );
		int r = ballPosition.translateToRelative().getAsPolar().getR();
		if (r < closestDistance) {
			selectedBallIndex = i;
			closestDistance = r;
		}
	}

	// TODO enlarge bounding box

	if (selectedBallIndex != -1) {
		ball->rectangle.x  = mBallCandidates[selectedBallIndex].x - mBallCandidates[selectedBallIndex].radius;
		ball->rectangle.y  = mBallCandidates[selectedBallIndex].y - mBallCandidates[selectedBallIndex].radius;
		ball->rectangle.width  = mBallCandidates[selectedBallIndex].radius*2;
		ball->rectangle.height = mBallCandidates[selectedBallIndex].radius*2;

		ball->basePoint.x = (ball->rectangle.x + ball->rectangle.x + ball->rectangle.width) / 2;
		ball->basePoint.y = ball->rectangle.y + ball->rectangle.height;
	}
	else {
		ball->clear();
	}

}
