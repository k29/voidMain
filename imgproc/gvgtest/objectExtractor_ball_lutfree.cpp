/*
 * objectExtractor_ball_lutfree.cpp
 *
 *  Created on: 05.05.2011
 *      Author: lisa
 */

#include "objectExtractor_ball_lutfree.h"
#include "color.h"
#include "vision.h"
#include "math.h"
#include "math/utils.h"
#include "config/configRegistry.h"

REGISTER_OPTION("vision.ballextractor.lutfree",1,"Set to 1, if LUT free ball detection (by Lisa) should be used");

/**
 * The standard constructor. Creates a new ball object.
 */
BallExtractorLutFree::BallExtractorLutFree() {
	ball = new RectangleObject(BALL_OBJECT, Ball);
	image = 0;

	currBallHueMean = HUE_BALL_MEAN;

	// init debugging variables
	frames = 0;
	avgTime = 0;
	pixels = 0;
}

/**
 * Destructor. Deletes the ball and clears the list of ball candidates.
 */
BallExtractorLutFree::~BallExtractorLutFree() {
	delete ball;
	ball = 0;
	possibleBalls.clear();
}

/**
 * Extract the ball from the image.
 * @return True, if at least one possible ball was found, false otherwise
 */
bool BallExtractorLutFree::extract() {
	if (image == 0) {
		ERROR("objectExtractor_ball_lutfree: No input image provided.");
		return false;
	}

	++frames;
	pixels = 0;

	ball-> clear();
	possibleBalls.clear();
	cells.clear();

	robottime_t start = 0, ballTime = 0;
	start = getCurrentMicroTime();

	PositionRelative topPos(TOP_BORDER, 0);
	PositionRelative middlePos(MIDDLE_BORDER, 0);
	topInImg = TheCameraModel::getInstance().projectOnImage(topPos);
	middleInImg = TheCameraModel::getInstance().projectOnImage(middlePos);

	bool found = findPossibleBalls();

	ballTime = getCurrentMicroTime()-start;
	avgTime += ballTime;

	if((frames%10) == 0) {
//		INFO("*** (%d) avg ball time: %.2f, mean hue %d, cells %d, pixels %d", frames,
//				(float)avgTime/frames, currBallHueMean, (int)cells.size(), pixels);
	} if (frames >= 500) {
		// reset
		avgTime = 0;
		frames = 0;
	}

	pixels = 0;

	// early return if no possible ball was found
	if (!found) return false;

	PossibleBall& bestBall = possibleBalls.front();

	// if even the best ball has high error, discard
	if(bestBall.error > PossibleBall::ALLOWED_BALL_ERROR) return false;

	// balls near our feet must have even smaller error
	if(bestBall.getRelativeDistance() < MIDDLE_BORDER && bestBall.error > 55)
		return false;

	// set best possible ball as current ball
	ball->rectangle.x = bestBall.upperLeftX;
	ball->rectangle.y = bestBall.upperLeftY;
	ball->rectangle.width = bestBall.width;
	ball->rectangle.height = bestBall.height;

	// point of ball that touches the ground
	ball->basePoint = bestBall.getBasePoint();

	// adapt mean hue of ball according to last best ball
	if (bestBall.error >= 0 && bestBall.error < 40) {
		if(bestBall.meanHue > currBallHueMean) {
			currBallHueMean = min(currBallHueMean+1, HUE_BALL_MEAN+HUE_BALL_STD);
		}
		else if(bestBall.meanHue < currBallHueMean) {
			currBallHueMean = max(currBallHueMean-1, HUE_BALL_MEAN-HUE_BALL_STD);
		}
	}

	return true;
}

/**
 * Finds possible balls by trying to locate clusters of reddish pixels.
 * @return true, if at least one possible ball was found
 */
bool BallExtractorLutFree::findPossibleBalls() {
//	robottime_t botTime = 0, midTime = 0, topTime = 0, tmpTime = 0;

//	tmpTime = getCurrentMicroTime();
	findBallsInBottom();
//	botTime = getCurrentMicroTime()-tmpTime;

	if(!possibleBalls.empty() && possibleBalls.front().error < 40) {
//		printf("** bottom: %d, middle: %d, top: %d\n", (int)botTime, (int)midTime, (int)topTime);
		return true;
	}

//	tmpTime = getCurrentMicroTime();
	findBallsInMiddle();
//	midTime = getCurrentMicroTime()-tmpTime;

	if(!possibleBalls.empty() && possibleBalls.front().error < 55) {
//		printf("** bottom: %d, middle: %d, top: %d\n", (int)botTime, (int)midTime, (int)topTime);
		return true;
	}

//	tmpTime = getCurrentMicroTime();
	findBallsInTop();
//	topTime = getCurrentMicroTime()-tmpTime;
//	printf("** bottom: %d, middle: %d, top: %d\n", (int)botTime, (int)midTime, (int)topTime);

	return !possibleBalls.empty();
}

/**
 * Searches the top part of the image for a ball.
 */
void BallExtractorLutFree::findBallsInTop() {
	int16_t horizonY = TheCameraModel::getInstance().horizon(900);

	// ignore top
	uint16_t y_start = ::max(0,horizonY - (horizonY&(GRID_CELL_SIZE-1)));
	uint16_t y_end = getTopInPixels();

	bool useEdges = true;
#ifdef ROBOT2011
	useEdges = false;
#endif

	scanForBall(y_start, y_end, useEdges);

	processPossibleBalls();
}

/**
 * Searches the middle part of the image for a ball.
 */
void BallExtractorLutFree::findBallsInMiddle() {
	uint16_t y_start = getTopInPixels();
	uint16_t y_end = getMiddleInPixels();

	scanForBall(y_start, y_end, true);

	processPossibleBalls();
}

/**
 * Searches the bottom part of the image for a ball.
 */
void BallExtractorLutFree::findBallsInBottom() {
	uint16_t y_start = getMiddleInPixels();
	uint16_t y_end = image->getImageHeight();

	scanForBall(y_start, y_end, true);

	processPossibleBalls();
}

/**
 * Scan the image for possible balls between y_start und y_end. useEdges=true
 * speeds up the scan significantly because only grid cells that have an edge
 * and some reddishPixels are further processed. With useEdges=false the image
 * is scanned line-by-line with a stepsize determined by the distance to the
 * robot's feet. This method is more accurate but computationally expensive.
 * @param y_start The y coordinate to start with.
 * @param y_end   The y coordinate to end.
 * @param useEdges
 */
void BallExtractorLutFree::scanForBall(uint16_t y_start, uint16_t y_end, bool useEdges) {
	uint16_t interestStepX, interestStepY;
	uint8_t x_step, y_step;
	uint8_t windowSize = maxSizeMid;

	interestStepX = GRID_CELL_SIZE;
	interestStepY = GRID_CELL_SIZE;

	// move window across image in interestStepX/Y steps
	for(uint16_t y = y_start; y < y_end;) {

		uint16_t x_start, x_end;
		Vision::getInstance().getLineInCenterCircle(y, &x_start, &x_end);
		// make divisble by GRID_CELL_SIZE
		x_start = ::max(0, x_start - (x_start&(GRID_CELL_SIZE-1)));
		x_end   = ::max(0, x_end - (x_end&(GRID_CELL_SIZE-1)));

		for(uint16_t x = x_start; x < x_end;) {
			int16_t maxY = Vision::getInstance().getFieldExtractor()
					.getHeighestFieldCoordinate(x);

			// don't search outside of field
			if (y < maxY-windowSize) {
				x+=interestStepX;
				continue;
			}

			// continue if cell has no edge
			if(useEdges) {
				GridCell* cell = Vision::getInstance().getGVG()
						.getGridCell(x/GRID_CELL_SIZE,y/GRID_CELL_SIZE);

				// only look at cell, if it has possible ball edges
				if (!hasPossibleBallEdge(cell) || !cell->hasRed) {
					x += GRID_CELL_SIZE; //next cell
					continue;
				}
			}

			PositionImage pos(x,y);
			// distance to our feet
			int16_t dist = pos.translateToRelative().getAsPolar().getR();

			// define expected ball size based on distance to our feet,
			// e.g. balls in the front are bigger and thus can be detected
			// by larger steps
			if (dist > TOP_BORDER) {
				windowSize = maxSizeTop;
				x_step = 2;
				y_step = 2;
			} else if(dist > MIDDLE_BORDER) {
				windowSize = maxSizeMid;
				x_step = 2;
				y_step = 4;
			} else {
				windowSize = maxSizeBot;
				x_step = 4;
				y_step = 6;
			}

			if(!useEdges) {
				interestStepX = x_step;
				interestStepY = y_step;
			}

			if(useEdges) {
				GridCell* cell = Vision::getInstance().getGVG()
						.getGridCell(x/GRID_CELL_SIZE,y/GRID_CELL_SIZE);

				Edge* edge = getPossibleBallEdge(cell);
				uint16_t cellStartX = x, cellStartY = y;

				if (edge != NULL) {
					// center cell at edge point
					EdgePoint ep = edge->linePoints.front();
					cellStartX = ep.getX() - GRID_CELL_SIZE/2;
					cellStartY = ep.getY() - GRID_CELL_SIZE/2;
				}

				findSeedPointInCell(cellStartX,cellStartY,GRID_CELL_SIZE,windowSize,x_step,y_step);
				cells.insert(std::make_pair(cvPoint(cellStartX,cellStartY),GRID_CELL_SIZE));

				// go to next cell
				x += interestStepX;
			} else {
				// scan line-by-line due to faster memory access pattern
				bool found = findSeedPointAtPixel(x,y,windowSize);
				if(!found) {
					x+=interestStepX;
				} else {
					x+=windowSize/3;
				}
			}
		}
		y+=interestStepY;
	}
}

/**
 * Searches for a blob of reddish pixels in the given grid cell of size
 * interestRegion times interestRegion.
 * @param x The x coordinate of the cell
 * @param y The y coordinate of the cell
 * @param interestRegion The cell size is interestRegion times interestRegion
 * @param windowSize The expected ball size
 * @param x_step The step size in x direction
 * @param y_step The step size in y direction
 * @return True if a PossibleBall was found and added to the list, false otherwise
 */
bool BallExtractorLutFree::findSeedPointInCell(uint16_t x, uint16_t y, uint8_t interestRegion,
		uint8_t windowSize, uint8_t x_step, uint8_t y_step) {

	uint16_t reddishPixels = 0;
	uint32_t redLikelihood = 0;
	uint32_t pixelsInRegion = 0;
	CvPoint center = cvPoint(0,0);

	uint8_t minReddishPixels = 2;
	if(windowSize > maxSizeMid) minReddishPixels = 3;
	else if(windowSize > maxSizeBot) minReddishPixels = 5;

	for(uint8_t l = 0; l < interestRegion; l+=y_step) {
		uint16_t absY = y+l;
		uint8_t k = 0;

		for(k = 0; k < interestRegion; k+=x_step) {
			++pixelsInRegion;
			uint16_t absX = x+k;
			uint16_t likelihood = 0;

			if(isBallPixelFast(absX,absY,likelihood)) {
				++reddishPixels;
				redLikelihood += likelihood;
				center.x += absX;
				center.y += absY;
			}
			// break if we found enough pixels in this region
			if(reddishPixels >= minReddishPixels) break;
		}
		// break if prior loop was left early
		if (k < interestRegion) break;
	}
	if (reddishPixels < minReddishPixels || pixelsInRegion == 0) {
		return false;
	}

	center.x /= reddishPixels;
	center.y /= reddishPixels;

	uint16_t height = windowSize;
	uint16_t width = windowSize;
	uint16_t upperLeftX = center.x - width/2;
	uint16_t upperLeftY = center.y - height/2;

	// make sure error is in [0,100]
	int16_t error = (100*reddishPixels-redLikelihood)/reddishPixels;

	assert(error >= 0 && error <= 100);

	// add ball with error
	possibleBalls.push_back(PossibleBall(upperLeftX, upperLeftY, width,
			height, error));

	return true;
}

/**
 * Checks if pixel (x,y) is ball pixel. If yes, a new PossibleBall object
 * with size windowSize times windowSize is added to the list.
 * @param x The x coordinate of the pixel
 * @param y The y coordinate of the pixel
 * @param windowSize Potential size of ball
 * @return True, if pixel (x,y) might form a ball.
 */
bool BallExtractorLutFree::findSeedPointAtPixel(uint16_t x, uint16_t y, uint8_t windowSize) {
	uint16_t likelihood = 0;

	if(isBallPixelFast(x,y,likelihood)) {
		uint16_t height = windowSize;
		uint16_t width = windowSize;
		uint16_t upperLeftX = x - width/2;
		uint16_t upperLeftY = y - height/2;

		// make sure error is in [0,100]
		int16_t error = (100-likelihood);
		assert(error >= 0 && error <= 100);

		// add ball with error
		possibleBalls.push_back(PossibleBall(upperLeftX, upperLeftY, width,
				height, error));
		return true;
	}
	return false;
}

/**
 * Checks for overlapping balls, keeps the one with min error, e.g maximum
 * reddish pixels in the center, discards others.
 */
void BallExtractorLutFree::combineBalls() {
	for(uint8_t i=0; i<possibleBalls.size(); ) {
		for(uint8_t j=i+1; j<possibleBalls.size(); ) {

			PossibleBall& ball1 = possibleBalls[i];
			PossibleBall& ball2 = possibleBalls[j];

			if (PossibleBall::detectCollision(ball1, ball2)) {

				// always keep bigger ball
				if(ball1.width < ball2.width) {
					// keep ball2
					possibleBalls.erase(possibleBalls.begin()+i);
					j = i+1;
					continue;
				}
				if(ball1.width > ball2.width) {
					// keep ball1
					possibleBalls.erase(possibleBalls.begin()+j);
					continue;
				}

				// if ball sizes are equal i.e. they lie in the same image
				// partition keep better ball (less error)
				if (ball1.error > ball2.error)  {
					// keep ball2
					possibleBalls.erase(possibleBalls.begin()+i);
					j = i+1;
					continue;
				} else {
					// keep ball1
					possibleBalls.erase(possibleBalls.begin()+j);
					continue;
				}
			}
			++j;
		}
		++i;
	}
}

/**
 * Grow each ball candidate in 8 directions. Discard balls that are too big,
 * too small or have the wrong shape.
 */
void BallExtractorLutFree::adjustBallRegion() {
	std::vector<PossibleBall>::iterator iter = possibleBalls.begin();
	for(;iter < possibleBalls.end(); ) {

		PossibleBall& ball = *iter;
		if(ball.processed) {
			++iter;
			continue;
		}

		// init, but set later
		uint8_t minSize = 0;
		uint8_t notGrowing = 0;
		uint8_t maxMissedPixels = 0;

		uint16_t const size = max(ball.width, ball.height);

		// determine how much the ball is allowed to grow
		// based on its original size
		uint16_t const maxSize = size;
		uint16_t const searchRadius = size + size/2;

		if(size <= maxSizeTop) {
			minSize = 3;
			maxMissedPixels = 2;
		}
		else if(size <= maxSizeMid) {
			minSize = 5;
			maxMissedPixels = 3;
		}
		else {
			minSize = 12;
			maxMissedPixels = 4;
		}


		// define step size: larger steps in the front
		uint8_t step = 1;
		if(size > maxSizeMid) step = 2;

		CvPoint center = ball.getCenter();

		uint16_t missedPixels[8];
		memset(missedPixels, 0, sizeof(uint16_t) * 8);

		CvPoint borders[8];
		memset(borders, 0, sizeof(CvPoint) * 8);

		for(uint8_t i=0; i<8;++i) {
			borders[i] = cvPoint(center.x,center.y);
		}

		CvPoint& lastMaxX     = borders[0];
		CvPoint& lastMaxXMinY = borders[1];
		CvPoint& lastMinY     = borders[2];
		CvPoint& lastMinXY    = borders[3];
		CvPoint& lastMinX     = borders[4];
		CvPoint& lastMinXMaxY = borders[5];
		CvPoint& lastMaxY     = borders[6];
		CvPoint& lastMaxXY    = borders[7];

		/* search for ball borders in eight directions
		 *
		 *                  minY
		 *    minXY --------------------  maxXminY
		 *          | .      .      .  |
		 *          |   .    .    .    |
		 *          |     .  .  .      |
		 *          |       ...        |
		 *     minX | . . . ... . . .  |  maxX
		 *          |       ...        |
		 *          |     .  .  .      |
		 *          |   .    .    .    |
		 *          | .      .      .  |
		 * minXmaxY -------------------- maxXY
		 *                  maxY
		 */

		for (int i = 1; i < searchRadius; i+=step) {

			CvPoint candidates[8];
			// correct order is important
			candidates[0] = cvPoint(center.x+i,center.y);
			candidates[1] = cvPoint(center.x+i,center.y-i);
			candidates[2] = cvPoint(center.x  ,center.y-i);
			candidates[3] = cvPoint(center.x-i,center.y-i);
			candidates[4] = cvPoint(center.x-i,center.y);
			candidates[5] = cvPoint(center.x-i,center.y+i);
			candidates[6] = cvPoint(center.x  ,center.y+i);
			candidates[7] = cvPoint(center.x+i,center.y+i);

			notGrowing = 0;

			// for each of the 8 directions
			for (int k = 0; k < 8; ++k) {
				if (missedPixels[k] > maxMissedPixels) {
					++notGrowing;
					continue;
				}

				int similar = isSimilar(candidates[k]);

				switch(similar) {
					case 0: // not similar
						++missedPixels[k];
						break;
					case 1: // similar
						borders[k].x = candidates[k].x;
						borders[k].y = candidates[k].y;
						break;
					default: // do nothing
						break;
				}
			}
			if (notGrowing == 8) break;
		}

		//enlarge or shrink ball
		uint16_t maxBorderX = max(lastMaxX.x, lastMaxXMinY.x, lastMaxXY.x);
		uint16_t minBorderX = min(lastMinX.x, lastMinXY.x, lastMinXMaxY.x);

		uint16_t maxBorderY = max(lastMaxY.y, lastMinXMaxY.y, lastMaxXY.y);
		uint16_t minBorderY = min(lastMinY.y, lastMinXY.y, lastMaxXMinY.y);

		uint16_t newWidth  = maxBorderX - minBorderX;
		uint16_t newHeight = maxBorderY - minBorderY;

		ball.upperLeftX = minBorderX;
		ball.upperLeftY = minBorderY;
		ball.width = newWidth;
		ball.height = newHeight;

#ifdef ROBOT2011
		// with the new camera position we see parts of our own body
		// make sure that we don't see a ball there
		if(ball.upperLeftY > (image->getImageHeight()-3*GRID_CELL_SIZE)) {
			iter = possibleBalls.erase(iter);
			continue;
		}
#endif

		// discard balls that are too large or too small,
		// have the wrong shape
		if (std::max(ball.width, ball.height) > maxSize
				|| std::min(ball.width, ball.height) < minSize
				|| (ball.width  > ball.height*2 && (abs(ball.width-ball.height)) > 10)
				|| (ball.height > ball.width*2  && (abs(ball.width-ball.height)) > 10)) {
			iter = possibleBalls.erase(iter);
			continue;
		}
		// enlarge a little to permit shadows at border
		if(ball.upperLeftX + ball.width < image->getImageWidth()-1
			&& ball.upperLeftY + ball.height < image->getImageHeight()-1) {
			ball.width  += 2;
			ball.height += 2;
			ball.upperLeftX -= 1;
			ball.upperLeftY -= 1;
		}
		++iter;

	}
}

/**
 * Recalculate the error for each possible ball as the sum of each pixel's
 * likelihood to be a ball pixel. Sort ball list so that the best
 * ball is at the front.
 */
void BallExtractorLutFree::rateBalls() {
	std::vector<PossibleBall>::iterator iter = possibleBalls.begin();

	for(; iter != possibleBalls.end(); ++iter) {
		PossibleBall& ball = *iter;
		if(ball.processed) continue;

		uint16_t error = weighRegion(ball);
		ball.error = error;
		ball.processed = true;
	}

	// sort the balls ascending by error value
	if (!possibleBalls.empty())
		std::sort(possibleBalls.begin(), possibleBalls.end(), PossibleBall::compare);

}

/**
 * Computes an error for the given ball that reflects how large the possibility
 * is to be a valid ball. Lower error means higher likelihood.
 *
 * The error is computed as the weighted sum of the gaussian weighted mean hue,
 * mean saturation and mean value of the ball's region.
 * @param ball
 * @return The error rate in [0,100].
 */
int16_t BallExtractorLutFree::weighRegion(PossibleBall& ball) {
	uint8_t const maxError = 100;

	uint16_t redLikelihood = 0;
	uint16_t pixelsInRegion = 0;
	uint16_t reddishPixels = 0;
	int32_t meanHue = 0;
	uint32_t meanVal = 0;
	uint32_t meanSat = 0;

	uint16_t x_start = ball.upperLeftX;
	uint16_t y_start = ball.upperLeftY;
	uint16_t x_end = ball.upperLeftX+ball.width;
	uint16_t y_end = ball.upperLeftY+ball.height;
	uint8_t x_step = 1, y_step = 1;

	// determine step size based on ball size and shape
	x_step = ::max(ball.width / 8, 1);
	y_step = ::max(ball.height / 8, 1);

	for(int y=y_start; y < y_end; y+=y_step) {
		for(int x=x_start; x < x_end; x+=x_step) {
			++pixelsInRegion;
			uint16_t hue;
			uint8_t sat,val;
			image->getPixelAsHSV(x,y,hue,sat,val);
			int16_t hueTemp = hue;
			if (hue > 180) hueTemp -= HUE_MAX;

			meanVal += val;
			meanSat += sat;

			if (isBallPixel(x,y,hue,sat,val)) {
				meanHue += hueTemp;
				redLikelihood += gaussianWeight(hueTemp,currBallHueMean,HUE_BALL_STD);
				++reddishPixels;
			}
		}
	}

	if(pixelsInRegion == 0 || reddishPixels == 0 || reddishPixels*100/pixelsInRegion < 40)
		return maxError;

	meanHue /= reddishPixels;
	meanVal /= pixelsInRegion;
	meanSat /= pixelsInRegion;
	ball.meanHue = meanHue;

	// weigh according to color, value and saturation
	int16_t valWeight = maxError - gaussianWeight(meanVal, 70, 20);
	int16_t satWeight = maxError - gaussianWeight(meanSat, 90, 10);
	int16_t hueWeight = (maxError*pixelsInRegion-redLikelihood)/pixelsInRegion;
	int16_t weight = hueWeight/2 + satWeight/3 + valWeight/6;

	assert(weight >= 0 && weight <= maxError);

	return weight;
}

/**
 * Returns true if newPoint is similar according to some measure.
 * @param newPoint
 * @return 0 - false, stop growing
 *         1 - true, continue growing from newPoint
 *         2 - ignore, continue growing from last point
 */
int BallExtractorLutFree::isSimilar(CvPoint newPoint) {
	if(newPoint.x < 0 || newPoint.y < 0
			|| newPoint.x > image->getImageWidth()-1
			|| newPoint.y > image->getImageHeight()-1)
		return 0;

	uint16_t hue;
	uint8_t sat,val;
	image->getPixelAsHSV(newPoint.x,newPoint.y,hue,sat,val);

	if (isBallPixel(newPoint.x,newPoint.y,hue,sat,val))
		return 1;

	// step over highlights
	if(hue <= 50 && val > 85)
		return 2;

	return 0;
}
