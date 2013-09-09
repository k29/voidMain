#ifndef OBJECTEXTRACTOR_OBSTACLE_H_
#define OBJECTEXTRACTOR_OBSTACLE_H_

#include "objectExtractor_base.h"
#include "../worldModel/worldModel.h"

/**
 * Defines the extraction of obstacles.
 *
 * @ingroup vision
 */
class ObstacleExtractor: public Extractor {
public:
	ObstacleExtractor();
	virtual ~ObstacleExtractor();

	/**
	 * Clears the data
	 */
	inline void clear() {
		cyanObstacleBoxes.clear();
		magentaObstacleBoxes.clear();
		blackObstacleBoxes.clear();
	}

	bool extract(const EdgeVector &edges);

	inline std::list<BoundingBox>& getCyanTeamBoxes() {
		return cyanObstacleBoxes;
	}

	inline std::list<BoundingBox>& getMagentaTeamBoxes() {
		return magentaObstacleBoxes;
	}

	inline std::list<BoundingBox>& getBlackObstacleBoxes() {
		return blackObstacleBoxes;
	}

	inline void setFeetSpace(const BoundingBox &fs) {
		feetSpace.rectangle.x = fs.rectangle.x;
		feetSpace.rectangle.y = fs.rectangle.y;
		feetSpace.rectangle.width = fs.rectangle.width;
		feetSpace.rectangle.height = fs.rectangle.height;
		feetSpace.basePoint.x = (fs.rectangle.x + fs.rectangle.x + fs.rectangle.width) / 2;
		feetSpace.basePoint.y = fs.rectangle.y + fs.rectangle.height;
	}

protected:
	// obstacles clustered as bounding boxes

	std::list<BoundingBox> cyanObstacleBoxes;
	std::list<BoundingBox> magentaObstacleBoxes;
	std::list<BoundingBox> blackObstacleBoxes;

	BoundingBox feetSpace;

	bool extractBoxes(const EdgeVector &edges);

	virtual void getGroundPoint(const BoundingBox &box, CvPoint &lowestPoint);

};

#endif /* OBJECTEXTRACTOR_OBSTACLE_H_ */
