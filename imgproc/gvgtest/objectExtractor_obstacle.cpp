#include "objectExtractor_obstacle.h"
#include "robot.h"
#include "vision.h"
#include "math/fixedPointMath.h"

ObstacleExtractor::ObstacleExtractor() { }

ObstacleExtractor::~ObstacleExtractor() {
	cyanObstacleBoxes.clear();
	magentaObstacleBoxes.clear();
	blackObstacleBoxes.clear();
}

/**
 *
 * @param edges
 * @return
 */
bool ObstacleExtractor::extract(const EdgeVector &edges) {
	return extractBoxes(edges);
}

/**
 * Cluster obstacles edges using bounding boxes.
 * @param edges
 * @return true, if obstacles where found, otherwise false
 */
bool ObstacleExtractor::extractBoxes(const EdgeVector &edges) {
	clear();

	EdgeVector cyan, magenta, black;


	for(uint16_t i = 0; i < edges.size(); ++i) {

		Edge *e = edges[i];
		// ignore everything with ball, blue goal or yellow goal
		if(e->mostFrequentColor1 == Ball || e->mostFrequentColor2 == Ball ||
				e->mostFrequentColor1 == BlueGoal || e->mostFrequentColor2 == BlueGoal ||
				e->mostFrequentColor1 == YellowGoal || e->mostFrequentColor2 == YellowGoal) {
			continue;
		}
		// then ignore everything, which has not obstacle color
		if(e->mostFrequentColor1 != Magenta && e->mostFrequentColor1 != Cyan && e->mostFrequentColor1 != Obstacle &&
				e->mostFrequentColor2 != Magenta && e->mostFrequentColor2 != Cyan && e->mostFrequentColor2 != Obstacle) {
			continue;
		}

		// discard obstacles, which are too near to the circle
		const EdgePoint &first = e->getFirstPoint();
		const EdgePoint &last = e->getLastPoint();
		int16_t centerX = Vision::getInstance().getImageCenterX();
		int16_t centerY = Vision::getInstance().getImageCenterY();
		int16_t radius = Vision::getInstance().getImageF();

		int16_t distanceFirst = abs(centerX - first.getX()) + abs(centerY - first.getY());
		int16_t distanceLast = abs(centerX - last.getX()) + abs(centerY - last.getY());
		if(min(radius - distanceFirst, radius - distanceLast) < 30) {
			continue;
		}

		// skip edges near to the feetspace box
		bool skip = false;
		for(std::vector<EdgePoint>::const_iterator iter = e->linePoints.begin(); iter != e->linePoints.end(); ++iter) {
			const EdgePoint &ep = *iter;
			if(feetSpace.distance(cvPoint(ep.getX(), ep.getY())) <= 5) {
				skip = true;
				break;
			}
		}
		if(skip)
			continue;

		// divide remaining obstacle points into sublists by color
		if(e->mostFrequentColor1 == Magenta || e->mostFrequentColor2 == Magenta) {
			magenta.push_back(e);
		}
		else if(e->mostFrequentColor1 == Cyan || e->mostFrequentColor2 == Cyan) {
			cyan.push_back(e);
		}
		else if (e->mostFrequentColor1 == Obstacle || e->mostFrequentColor2 == Obstacle) {
			black.push_back(e);
		}

	}

	std::list<BoundingBox> tmpMagentaBoxes, tmpCyanBoxes, tmpBlackBoxes;

	createBoundingBoxes(magenta, tmpMagentaBoxes, 20, 50);
	createBoundingBoxes(cyan, tmpCyanBoxes, 20, 50);
	createBoundingBoxes(black, tmpBlackBoxes, 20, 50);


	// enlarge bounding boxes to the ground
	for(std::list<BoundingBox>::iterator iter = tmpMagentaBoxes.begin(); iter != tmpMagentaBoxes.end(); ) {

		BoundingBox &box = *iter;
		CvPoint center = { ( box.rectangle.x + box.rectangle.x + box.rectangle.width ) / 2, ( box.rectangle.y + box.rectangle.y + box.rectangle.height ) / 2 };
		CvPoint seed = { -1, -1 };
		for(int16_t i = -1; i <= 1; ++i) {
			for(int16_t j = -1; j <= 1; ++j) {
				Color c = colorMgr->getPixelColor(*image, center.x + i, center.y + j);
				if (c == Magenta) {
					seed.x = center.x + i;
					seed.y = center.y + j;
					break;
				}
			}
		}

		// region growing
		if(seed.x != -1) {
			BoundingBox newbox = regionGrowing(seed.x, seed.y, Magenta, 100, 100);
			if ( newbox.intersects(feetSpace) ) {
				newbox.rectangle.height = feetSpace.rectangle.y - newbox.rectangle.y;
				newbox.basePoint.y = feetSpace.rectangle.y;
			}

			if(box.rectangle.width * box.rectangle.height < newbox.rectangle.width * newbox.rectangle.height) {
				box.set(newbox.rectangle, newbox.basePoint, 0);
			}
		}
		else { // clear box if we can't grow
			box.clear();
		}

		if(box.rectangle.width == 0 || box.rectangle.height == 0 || box.rectangle.width * box.rectangle.height < 500) {
			iter = tmpMagentaBoxes.erase(iter);
			continue;
		}

		++iter;
	}

	for(std::list<BoundingBox>::iterator iter = tmpCyanBoxes.begin(); iter != tmpCyanBoxes.end(); ) {
		BoundingBox &box = *iter;

		CvPoint center = { ( box.rectangle.x + box.rectangle.x + box.rectangle.width ) / 2, ( box.rectangle.y + box.rectangle.y + box.rectangle.height ) / 2 };
		CvPoint seed = { -1, -1 };
		for(int16_t i = -1; i <= 1; ++i) {
			for(int16_t j = -1; j <= 1; ++j) {
				Color c = colorMgr->getPixelColor(*image, center.x + i, center.y + j);
				if (c == Cyan) {
					seed.x = center.x + i;
					seed.y = center.y + j;
					break;
				}
			}
		}

		// region growing
		if(seed.x != -1) {
			BoundingBox newbox = regionGrowing(seed.x, seed.y, Cyan, 100, 100);
			if ( newbox.intersects(feetSpace) ) {
				newbox.rectangle.height = feetSpace.rectangle.y - newbox.rectangle.y;
				newbox.basePoint.y = feetSpace.rectangle.y;
			}

			if(box.rectangle.width * box.rectangle.height < newbox.rectangle.width * newbox.rectangle.height) {
				box.set(newbox.rectangle, newbox.basePoint, 0);
			}
		}
		else { // clear box if we can't grow
			box.clear();
		}

		if(box.rectangle.width == 0 || box.rectangle.height == 0 || box.rectangle.width * box.rectangle.height < 500) {
			iter = tmpCyanBoxes.erase(iter);
			continue;
		}

		++iter;
	}

	for(std::list<BoundingBox>::iterator iter = tmpBlackBoxes.begin(); iter != tmpBlackBoxes.end(); ) {
		BoundingBox &box = *iter;

		CvPoint center = { ( box.rectangle.x + box.rectangle.x + box.rectangle.width ) / 2, ( box.rectangle.y + box.rectangle.y + box.rectangle.height ) / 2 };
		CvPoint seed = { -1, -1 };
		for(int16_t i = -1; i <= 1; ++i) {
			for(int16_t j = -1; j <= 1; ++j) {
				Color c = colorMgr->getPixelColor(*image, center.x + i, center.y + j);
				if (c == Obstacle) {
					seed.x = center.x + i;
					seed.y = center.y + j;
					break;
				}
			}
		}

		// region growing
		if(seed.x != -1) {
			BoundingBox newbox = regionGrowing(seed.x, seed.y, Obstacle, 100, 100);
			if ( newbox.intersects(feetSpace) ) {
				newbox.rectangle.height = feetSpace.rectangle.y - newbox.rectangle.y;
				newbox.basePoint.y = feetSpace.rectangle.y;
			}

			if(box.rectangle.width * box.rectangle.height < newbox.rectangle.width * newbox.rectangle.height) {
				box.set(newbox.rectangle, newbox.basePoint, 0);
			}
		}
		else { // clear box if we can't grow
			box.clear();
		}

		if(box.rectangle.width == 0 || box.rectangle.height == 0 || box.rectangle.width * box.rectangle.height < 500) {
			iter = tmpBlackBoxes.erase(iter);
			continue;
		}

		++iter;
	}

	// merge new boxes to eleminate intersection of boxes of the same color
	BoundingBox::mergeRectsInDistance(tmpBlackBoxes, blackObstacleBoxes, 0, 0);
	BoundingBox::mergeRectsInDistance(tmpCyanBoxes, cyanObstacleBoxes, 0, 0);
	BoundingBox::mergeRectsInDistance(tmpMagentaBoxes, magentaObstacleBoxes, 0, 0);

	// merge cyan with black boxes and magenta with black boxes
	for (std::list<BoundingBox>::iterator miter = magentaObstacleBoxes.begin(); miter != magentaObstacleBoxes.end(); ++miter) {
		BoundingBox &magentaBox = *miter;

		for(std::list<BoundingBox>::iterator biter = blackObstacleBoxes.begin(); biter != blackObstacleBoxes.end(); ) {
			BoundingBox &blackBox = *biter;

			BoundingBox newbox;
			if (blackBox.intersects(magentaBox)) {
				newbox = BoundingBox::merge(blackBox, magentaBox);
				magentaBox.set(newbox.rectangle, newbox.basePoint, 0);

				biter = blackObstacleBoxes.erase(biter);
				continue;
			}

			++biter;
		}
	}

	for (std::list<BoundingBox>::iterator citer = cyanObstacleBoxes.begin(); citer != cyanObstacleBoxes.end(); ++citer) {
		BoundingBox &cyanBox = *citer;

		for(std::list<BoundingBox>::iterator biter = blackObstacleBoxes.begin(); biter != blackObstacleBoxes.end(); ) {
			BoundingBox &blackBox = *biter;

			BoundingBox newbox;
			if (blackBox.intersects(cyanBox)) {
				newbox = BoundingBox::merge(blackBox, cyanBox);
				cyanBox.set(newbox.rectangle, newbox.basePoint, 0);

				biter = blackObstacleBoxes.erase(biter);
				continue;
			}

			++biter;
		}
	}

	if(magentaObstacleBoxes.empty() && cyanObstacleBoxes.empty() && blackObstacleBoxes.empty()) {
		return false;
	}
	return true;
}


/**
 * Founds the point on the ground (= on green carpet) of the given bounding box.
 *
 * @param box
 * @param lowestPoint  The result is stored into it
 */
void ObstacleExtractor::getGroundPoint(const BoundingBox &box, CvPoint &lowestPoint) {
	uint16_t lowestY = box.basePoint.y;
	uint16_t middleX = (box.rectangle.x + box.rectangle.x + box.rectangle.width) / 2;

	bool goLower = true;
	uint16_t step = 5;

	while(goLower && lowestY < image->getImageHeight()-step) {
		int16_t colorCounter[9] = {0};

		uint16_t minX = max(0, middleX - step);
		uint16_t maxX = min(image->getImageWidth(), (uint16_t) (middleX + step));

		for(uint16_t x = minX; x < maxX; ++x) {
			Color c = colorMgr->getPixelColor(*image, x, lowestY);
			colorCounter[(int)c]++;
		}

		// get most frequent color
		int16_t max = 0;
		int16_t maxColor = 0;
		for(int16_t i = 0; i < 9; ++i) {
			if(colorCounter[i] > max) {
				max = colorCounter[i];
				maxColor = i;
			}
		}

		// on the ground, there is only green ...
		if((Color) maxColor == Field || (Color) maxColor == Ball ||
				lowestY >= feetSpace.rectangle.y) {
			goLower = false;
		}
		else {
			lowestY += step;
		}
	}

	lowestPoint.x = middleX;
	lowestPoint.y = lowestY;

}
