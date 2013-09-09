#include "objectExtractor_ball.h"
#include "objectExtractor.h"
#include "objectExtractor_field.h"
#include "position.h"
#include "robot.h"
#include "vision.h"


BallExtractor::BallExtractor() {
	ball = new RectangleObject(BALL_OBJECT, Ball);
}

BallExtractor::~BallExtractor() {
	delete ball;
	ball = 0;
}

/**
 * Extracts the ball from the given edges.
 * All ball edges are taken into account. From the edges,
 * the bounding boxes get extracted an the best one is then chosen as ball.
 *
 * @param edges
 */
bool BallExtractor::extract(const EdgeVector &edges) {
	clear();

	if(edges.empty()) { // nothing to do
		return false;
	}

	if(image == 0) return false;

	int16_t imgWidth = image->getImageWidth();
	int16_t imgHeigth = image->getImageHeight();

	// use just the needed edges
	EdgeVector ballEdges;
	for(uint16_t i = 0; i < edges.size(); ++i) {
		Edge *e = edges[i];
		// add to ball edges, if it's a red/green or red/white edge
		if((e->mostFrequentColor1 == Ball && (e->mostFrequentColor2 == White || e->mostFrequentColor2 == Field))
				|| (e->mostFrequentColor2 == Ball && (e->mostFrequentColor1 == White || e->mostFrequentColor1 == Field))) {
			ballEdges.push_back(e);
		}
	}

	std::list<BoundingBox> bbBall;
	createBoundingBoxes(ballEdges, bbBall, 40, 40);

	// iterate through all balls and choose the best
	BoundingBox biggestBall;
	int16_t biggestBallIndex = -1;
	int16_t i = -1;
	int size = 0;

	UNUSED(biggestBallIndex);

	// iterate through all possible ball regions
	std::list<BoundingBox>::iterator bbIter;
	for (bbIter = bbBall.begin(); bbIter != bbBall.end(); ++bbIter) {
		i++;
		BoundingBox &box = *bbIter;
		int16_t xx = (int16_t) box.rectangle.x;
		int16_t yy = (int16_t) box.rectangle.y;
		int16_t w = (int16_t) box.rectangle.width;
		int16_t h = (int16_t) box.rectangle.height;
		int16_t s = w * h;

#if VISION_DEBUG
		// generate protobuf objects
		de::fumanoids::message::ObjectPosition *ballObjStatus = protobufExtractorStatus->add_objectpositions();
		ballObjStatus->set_type(::de::fumanoids::message::ObjectPosition_ObjectType_Ball);
		de::fumanoids::message::Position *ballObjStatusPos = ballObjStatus->mutable_position();
		ballObjStatusPos->set_positiontype(de::fumanoids::message::Position_PositionType_IMAGE);
		ballObjStatusPos->set_x((xx + xx + w) / 2);
		ballObjStatusPos->set_y(yy + h);
#endif

		// verify ball by counting all red pixels in the bounding box (and a little bit bigger to enlarge it) {
		int16_t minBallY = 10000;
		int16_t minBallX = 10000;
		int16_t maxBallY = -1;
		int16_t maxBallX = -1;
		int16_t ballCounter = 0;

		for (int16_t y = yy - 15 < 0 ? 0 : yy - 15; y < yy + h + 15 && y < imgHeigth; y += 2) {
			if (y < 0 || y > imgHeigth - 1)
				continue;

			for (int16_t x = xx - 15 < 0 ? 0 : xx - 15; x < xx + w + 15 && x < imgWidth ; x += 2) {
				if (x < 0 || x > imgWidth - 1)
					continue;

				Color c = colorMgr->getPixelColor(*image, x, y);
				if (c == Ball) {
					++ballCounter;
					maxBallX = max(x, maxBallX);
					maxBallY = max(y, maxBallY);
					minBallX = min(x, minBallX);
					minBallY = min(y, minBallY);
				}
			}
		}

		if(ballCounter == 0) { // this should not happen, we found nothing inside the bounding box
			continue;
		}

		s = (maxBallX - minBallX) * (maxBallY - minBallY);

		// test, if the density of red pixels is high enough in this new region
		if (s == 0 || fdiv(toFixed(ballCounter), toFixed(s)) > toFixed(0.1)) {
			BoundingBox newBox(cvRect(minBallX, minBallY, maxBallX - minBallX,
					maxBallY - minBallY), cvPoint((maxBallX + minBallX) / 2, maxBallY), ballCounter);
			*bbIter = newBox;
			w = newBox.rectangle.width;
			h = newBox.rectangle.height;
		} else {
#if VISION_DEBUG
			ballObjStatus->set_comment("Ball color density too low");
#endif
			continue;
		}

		// calculate distance to ball and discard, if too far away (and we can't see it)
		PositionImage imgPos((minBallX + maxBallX) / 2, maxBallY);
		Polar polarPos = imgPos.translateToRelative().getAsPolar();

		if(polarPos.getR() > 350) {
//			INFO("Discard ball, because it's to far away");
#if VISION_DEBUG
			ballObjStatus->set_comment("Too far away");
#endif
		}

		// discard ball, which is near the feetspace but too small or in general to small
		if ((s < 100 && (*bbIter).distance(feetSpace) < 20)	|| (w == 0 && h == 0)) {
//			INFO("Discard ball at (%d, %d) w: %d h %d", (*bbIter).rectangle.x, (*bbIter).rectangle.y, (*bbIter).rectangle.width, (*bbIter).rectangle.height);
#if VISION_DEBUG
			ballObjStatus->set_comment("Not expected size");
#endif
			continue;
		}

		// if the size of this ball region is bigger then the current found one,
		// then replace the it with this new one
		if (s > size) {
			size = s;
			biggestBall = *bbIter;
			biggestBallIndex = i;
		}
	}

	if(biggestBall.rectangle.x != -1) { // we found a ball

		// enlarge bounding box to the ground, until we see green/ white or obstacle
		int16_t newLowestY = -1;
		for(int16_t y = biggestBall.basePoint.y; y < (int16_t) image->getImageHeight(); ++y) {
			Color c = colorMgr->getPixelColor(*image, biggestBall.basePoint.x, y);

			if(c == Obstacle || c == Cyan || c == Magenta || c == Field || c == White) {
				newLowestY = y - 3; // don't use the lowest, to omit shadow under the ball
				break;
			}
		}

		if(newLowestY != -1) {
			biggestBall.rectangle.height += newLowestY - biggestBall.basePoint.y;
			biggestBall.basePoint.y = newLowestY;
		}

		ball->set(biggestBall.rectangle, biggestBall.basePoint);
//		comm.logDebug(LOG_VISION,"Found Ball: (%d, %d) w: %d h: %d",
//				ball->rectangle.x, ball->rectangle.y,
//				ball->rectangle.width, ball->rectangle.height);
#if VISION_DEBUG
		protobufExtractorStatus->mutable_objectpositions(biggestBallIndex)->set_isused(true);
#endif
		return true;
	}

	return false;
}
