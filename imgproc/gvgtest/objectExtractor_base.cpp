#include "objectExtractor_base.h"
#include "camera/theCameraModel.h"

/**
 * Founds the point on the ground (= on green carpet) of the given bounding box.
 *
 * @param box
 * @param lowestPoint  The result is stored into it
 */
void Extractor::getGroundPoint(const BoundingBox &box, CvPoint &lowestPoint) {
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
		if((Color) maxColor == Field ) {
			goLower = false;
		}
		else {
			lowestY += step;
		}
	}

	lowestPoint.x = middleX;
	lowestPoint.y = lowestY;

}

/**
 * Starts a region growing from the given seed point, on the given color.
 * The dimension of the surrounding bounding box can be limited.
 *
 * @param seedX
 * @param seedY
 * @param color
 * @param maxWidth
 * @param maxHeight
 * @return bounding box of the region
 */
BoundingBox Extractor::regionGrowing(int16_t seedX, int16_t seedY, Color color, int16_t maxWidth, int16_t maxHeight) {
	std::vector<CvPoint> region;
	std::vector<CvPoint> seedPoints;
	BoundingBox regionBB;

	int16_t centerX = TheCameraModel::getInstance().centerX();
	int16_t centerY = TheCameraModel::getInstance().centerY();
	int16_t radius = (int16_t) TheCameraModel::getInstance().focalLengthX();

	seedPoints.push_back( cvPoint(seedX, seedY) );
	regionBB.set(cvRect(seedX, seedY, 0, 0), cvPoint(seedX, seedY), 0);

	while( ! seedPoints.empty()) {

		CvPoint p = seedPoints.back();
		seedPoints.pop_back();
		region.push_back(p);
		BoundingBox bb(cvRect(p.x, p.y, 0, 0), cvPoint(p.x, p.y));
		regionBB = BoundingBox::merge(regionBB, bb);

		if ( regionBB.rectangle.height > maxHeight || regionBB.rectangle.width > maxWidth ) {
			break;
		}

		if(FixedPointMath::isqrt( (p.x - centerX) * (p.x - centerX) + (p.y - centerY) * (p.y - centerY) ) >= radius) {
			continue;
		}

		if ( colorMgr->getPixelColor(*image, p.x + 3, p.y) == color ) {
			bool alreadyInRegion = false;
			for (std::vector<CvPoint>::iterator iter = region.begin(); iter != region.end(); ++iter) {
				if (iter->x == p.x + 3 && iter->y == p.y) {
					alreadyInRegion = true;
					break;
				}
			}
			if ( ! alreadyInRegion) {
				seedPoints.push_back( cvPoint(p.x + 3, p.y) );
			}
		}

		if ( colorMgr->getPixelColor(*image, p.x - 3, p.y) == color ) {
			bool alreadyInRegion = false;
			for (std::vector<CvPoint>::iterator iter = region.begin(); iter != region.end(); ++iter) {
				if (iter->x == p.x - 3 && iter->y == p.y) {
					alreadyInRegion = true;
					break;
				}
			}
			if ( ! alreadyInRegion) {
				seedPoints.push_back( cvPoint(p.x - 3, p.y) );
			}
		}

		if ( colorMgr->getPixelColor(*image, p.x, p.y + 3) == color ) {
			bool alreadyInRegion = false;
			for (std::vector<CvPoint>::iterator iter = region.begin(); iter != region.end(); ++iter) {
				if (iter->x == p.x && iter->y == p.y + 3) {
					alreadyInRegion = true;
					break;
				}
			}
			if ( ! alreadyInRegion) {
				seedPoints.push_back( cvPoint(p.x, p.y + 3) );
			}
		}

		if ( colorMgr->getPixelColor(*image, p.x, p.y - 3) == color ) {
			bool alreadyInRegion = false;
			for (std::vector<CvPoint>::iterator iter = region.begin(); iter != region.end(); ++iter) {
				if (iter->x == p.x && iter->y == p.y - 3) {
					alreadyInRegion = true;
					break;
				}
			}
			if ( ! alreadyInRegion) {
				seedPoints.push_back( cvPoint(p.x, p.y - 3) );
			}
		}

	}

//	BoundingBox regionBB(cvRect(region.front().x, region.front().y, 0, 0), cvPoint(region.front().x, region.front().y));
//
//	for (std::vector<CvPoint>::const_iterator iter = region.begin() + 1; iter != region.end(); ++iter) {
//		BoundingBox bb(cvRect(iter->x, iter->y, 0, 0), cvPoint(iter->x, iter->y));
//		regionBB = BoundingBox::merge(regionBB, bb);
//	}
	regionBB.basePoint.x = (regionBB.rectangle.x + regionBB.rectangle.x + regionBB.rectangle.width) / 2;
	regionBB.basePoint.y = regionBB.rectangle.y + regionBB.rectangle.height;

	return regionBB;
}

/**
 * Creates one ore more bounding boxes of the edges
 *
 * @param edges         The edges which match the criteria for an object like, goal, pole, teamplayers etc.
 * @param boundingBoxes The list with the resulting bounding boxes
 * @param distanceX     Maximum distance in x direction of two bounding boxes, in which they will be merged
 * @param distanceY     Maximum distance in y direction of two bounding boxes, in which they will be merged
 *
 */
void Extractor::createBoundingBoxes(const EdgeVector& edges, std::list<BoundingBox> &boundingBoxes, int16_t distanceX, int16_t distanceY) {
	std::list<BoundingBox> tmp;

	for (uint16_t i = 0; i < edges.size(); ++i) {
		BoundingBox bb = edges[i]->getBoundingBox();
		tmp.push_back(bb);
	}

	BoundingBox::mergeRectsInDistance(tmp, boundingBoxes, distanceX, distanceY);
}

/**
 * Fills the colorArray (needs size 9!) with the number of pixels
 * with the corresponding color in the 8-neighbourhood of the given pixel
 *
 * @param x
 * @param y
 * @param colorArray
 */
void Extractor::getColorsInNeighbourhood(int16_t x, int16_t y, uint8_t *colorArray) {

	for(uint8_t c = 0; c < 9; ++c) {
		colorArray[c] = 0;
	}

	for(int16_t yy = y - 1; yy <= y + 1; ++yy) {
		for(int16_t xx = x - 1; xx <= x + 1; ++xx) {

			if(xx == x && yy == y)
				continue;

			Color c = colorMgr->getPixelColor(*image, xx, yy);
			colorArray[(int) c]++;

		}
	}

}
