#ifndef OBJECTEXTRACTOR_POLE_H_
#define OBJECTEXTRACTOR_POLE_H_

#include "objectExtractor_base.h"
#include "object.h"

/**
 * Extractor for the side poles.
 * There are two methods implemented, one using the field contour and the information of possible
 * side poles given by the GoalExtractor.
 * The other method relays on the edges (and the colors of them).
 *
 * @ingroup vision
 */
class PoleExtractor: public Extractor {
public:
	PoleExtractor();
	virtual ~PoleExtractor();

	/**
	 * Clears the data
	 */
	inline void clear() {
		ybyPole->clear();
		bybPole->clear();
	}

	bool extract(const EdgeVector &edges);

	inline RectangleObject* getYBYPole() {
		return ybyPole;
	}

	inline RectangleObject* getBYBPole() {
		return bybPole;
	}

	inline bool ybyPoleSeen() {
		return ybyPole->rectangle.x != -1;
	}

	inline bool bybPoleSeen() {
		return bybPole->rectangle.x != -1;
	}

	inline void setYBYPoleBoundingBoxes(const std::list<BoundingBox> &ybyPoleBoundingBoxes) {
		this->ybyPoleBoundingBoxes = ybyPoleBoundingBoxes;
	}

	inline void setBYBPoleBoundingBoxes(const std::list<BoundingBox> &bybPoleBoundingBoxes) {
		this->bybPoleBoundingBoxes = bybPoleBoundingBoxes;
	}


protected:
	RectangleObject *ybyPole;
	RectangleObject *bybPole;

	std::list<BoundingBox> ybyPoleBoundingBoxes;
	std::list<BoundingBox> bybPoleBoundingBoxes;

	bool extractEdgeColorBased(const EdgeVector &edges);
	bool extractContourBased();

	void setProtobuf(RectangleObject *obj);
};

#endif /* OBJECTEXTRACTOR_POLE_H_ */
