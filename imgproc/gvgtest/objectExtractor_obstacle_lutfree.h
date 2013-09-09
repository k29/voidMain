/*
 * objectExtractor_obstacle_lutfree.h
 *
 *  Created on: 30.06.2011
 *      Author: lisa
 */

#ifndef OBJECTEXTRACTOR_OBSTACLE_LUTFREE_H_
#define OBJECTEXTRACTOR_OBSTACLE_LUTFREE_H_

#include "objectExtractor_base.h"


/**
 * @{
 * @ingroup vision
 *
 * A class to detect obstacles in an image. The idea is to compare the
 * original (unfiltered) field contour with the filtered field contour to
 * identify points that lie significantly below the real contour. These point
 * might belong to an obstacle and thus are further processed.
 * Starting from this point a box is grown upwards of a fixed width. Boxes
 * are then merged if they overlap or their base points are close together.
 */
class ObstacleExtractorLutFree {
public:
	ObstacleExtractorLutFree();
	~ObstacleExtractorLutFree();

	bool extract();
	bool isObstaclePixel(uint16_t x,uint16_t y);
	bool isTeamMarkerPixel(uint16_t x, uint16_t y);

	/**
	 * Clear boxes.
	 */
	inline void clear() {
		blackObstacleBoxes.clear();
		possibleObstacles.clear();
	}

	/**
	 * Get black obstacles.
	 * @return The list of obstacles
	 */
	inline std::list<BoundingBox>& getBlackObstacleBoxes() {
		return blackObstacleBoxes;
	}

	/**
	 * Get the base points of possible obstacles.
	 * @return The list of points
	 */
	inline std::vector<CvPoint>& getPossibleObstaclePoints() {
		return possibleObstacles;
	}

	/**
	 * Sets the image to works with.
	 * @param img
	 */
	inline void setImage(IMAGETYPE *img) {
		image = img;
	}

	/**
	 * Set the robots feet space.
	 * @param fs The BoundingBox of the feet space
	 */
	inline void setFeetSpace(const BoundingBox &fs) {
		feetSpace.rectangle.x = fs.rectangle.x;
		feetSpace.rectangle.y = fs.rectangle.y;
		feetSpace.rectangle.width = fs.rectangle.width;
		feetSpace.rectangle.height = fs.rectangle.height;
		feetSpace.basePoint.x = (fs.rectangle.x + fs.rectangle.x + fs.rectangle.width) / 2;
		feetSpace.basePoint.y = fs.rectangle.y + fs.rectangle.height;
	}

protected:
	IMAGETYPE* image;

	std::list<BoundingBox> blackObstacleBoxes;
	std::vector<CvPoint> possibleObstacles;

	BoundingBox feetSpace;

};

/**
 * @}
 */

#endif /* OBJECTEXTRACTOR_OBSTACLE_LUTFREE_H_ */
