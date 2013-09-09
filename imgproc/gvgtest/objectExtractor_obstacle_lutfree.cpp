/*
 * objectExtractor_obstacle_lutfree.cpp
 *
 *  Created on: 30.06.2011
 *      Author: lisa
 *
 */

#include "objectExtractor_obstacle_lutfree.h"
#include "vision.h"
#include "config/configRegistry.h"

// currently not used by default
REGISTER_OPTION("vision.obstacles.lutfree",0,"Set to 1, if LUT free obstacle detection should be used");

ObstacleExtractorLutFree::ObstacleExtractorLutFree() {
	image = 0;
}

ObstacleExtractorLutFree::~ObstacleExtractorLutFree() {
	image = 0;
	clear();
}

/**
 * Extracts the dark obstacles in the image and pushes them into a list.
 */
bool ObstacleExtractorLutFree::extract() {
	clear();

	if(image == 0) {
		ERROR("objectExtractor_obstacle_lutfree: No input image provided.");
		return false;
	}

	FieldExtractor& fe = Vision::getInstance().getFieldExtractor();
	std::vector<CvPoint>& originalContour = fe.getOriginalFieldContour();
	int16_t horizonY = TheCameraModel::getInstance().horizon(900);
	std::list<BoundingBox> tmp;

	// copy point
	CvPoint& tmpPoint = *(originalContour.begin()+1);
	CvPoint prevPoint = cvPoint(tmpPoint.x,tmpPoint.y);

	for(std::vector<CvPoint>::iterator iter = originalContour.begin()+2;
			iter != originalContour.end()-1; ++iter) {

		tmpPoint = *iter;
		CvPoint point = cvPoint(tmpPoint.x,tmpPoint.y);

		uint8_t distX = point.x - prevPoint.x;

		if(distX > FieldExtractor::X_STEP) {
			// a point is missing, we have an obstacle here
			uint8_t steps = distX/FieldExtractor::X_STEP;
			// interpolate
			point.x = prevPoint.x + FieldExtractor::X_STEP;
			point.y = prevPoint.y + (point.y-prevPoint.y)/steps;
			--iter;
		}

		prevPoint.x = point.x;
		prevPoint.y = point.y;

		uint16_t filteredContourY = fe.getHeighestFieldCoordinate(point.x);

		if(!(point.y > filteredContourY + 16)) continue;

		// find first real obstacle point
		for(int16_t y = point.y; y > filteredContourY; y -= 2) {
			if(isObstaclePixel(point.x,y)) {
				point.y = y;
				break;
			}
		}

		if(point.y <= filteredContourY) continue;
			possibleObstacles.push_back(point);

		int16_t missed = 0;

		// find highest obstacle point
		CvPoint lastPoint = cvPoint(-1,-1);

		// grow obstacle
		for(int16_t y = point.y; y > horizonY; y -= 2) {
			if (y < 0) break;

			if(isObstaclePixel(point.x,y)) {
				missed = 0;
				lastPoint.y = y;
//			} else if(isTeamMarkerPixel(point.x,y)) {
//				// step over, do nothing
			} else {
				missed++;
				if(missed > 10) break;
			}
		}

		if (lastPoint.y == -1) continue;

		// throw away if to close to circle
		uint16_t lineStart, lineEnd;
		Vision::getInstance().getLineInCenterCircle(point.y, &lineStart, &lineEnd);

		if(point.x-lineStart < 32 || lineEnd-point.x < 32)
			continue;

		uint16_t width = 3 * FieldExtractor::X_STEP/2;
		// make BoundingBox
		BoundingBox box;
		box.basePoint = cvPoint(point.x, point.y);
		box.rectangle.width = width;
		box.rectangle.height = point.y - lastPoint.y;

		box.rectangle.x = point.x - width/2;
		box.rectangle.y = point.y - box.rectangle.height;

		int16_t lastPointY = -1;
		uint8_t missedObstacle = 0;

		// pull base point to ground, i.e. the field,
		// but feetspace is maximum
		for(uint16_t k = box.basePoint.y; k < feetSpace.rectangle.y; k+=2) {

			if(isObstaclePixel(box.basePoint.x, k)) {
				lastPointY = k;
				missedObstacle = 0;
			} else {
				++missedObstacle;
			}

			if (missedObstacle > 5) break;
		}

		if(lastPointY != -1) {
			// enlarge the box
			box.basePoint.y = lastPointY;
			box.rectangle.height = lastPointY-box.rectangle.y;
		}

		if(box.rectangle.height < 32) continue;

		tmp.push_back(box);
	}

	if(!tmp.empty()) {
		BoundingBox::mergeRectsInDistance(tmp,blackObstacleBoxes,0,10);
	}

	return true;
}

/**
 * Checks if pixel (x,y) might be an obstacle pixel. Therefore it is determined
 * if it's of dark color.
 * @param x The x coordinate of the pixel
 * @param y The y coordinate of the pixel
 * @return True, if pixel might belong to an obstacle.
 */
bool ObstacleExtractorLutFree::isObstaclePixel(uint16_t x, uint16_t y) {
	uint8_t r,g,b;
	image->getPixelAsRGB(x,y,&r,&g,&b);

	uint8_t max = ::max(r,g,b);
	uint8_t min = ::min(r,g,b);

	return (max < 60 || (max-min < 40 && max < 100));
}

/// TODO unused at the moment, find better description
bool ObstacleExtractorLutFree::isTeamMarkerPixel(uint16_t x, uint16_t y) {
	uint8_t r,g,b;
	image->getPixelAsRGB(x,y,&r,&g,&b);
	uint8_t max = ::max(r,g,b);
	return ((max == b && b-g > b-r) || (max == r && r-b > r-g));
}
