#ifndef EDGE_H_
#define EDGE_H_

#include <inttypes.h>
#include <opencv/cv.h>
#include <vector>
#include "math/Fixed.h"
#include "color.h"
#include "paintable.h"
#include "boundingBox.h"
#include "position.h"

/**
 * @{
 * @ingroup vision
 */

/**
 * A single image edge point, represented by position and orientation
 */
class EdgePoint : public PositionImage {
public:
	EdgePoint() :
		dirX(0), dirY(0) {
	}

	EdgePoint(int16_t _x, int16_t _y, FixedPointMath::fixedpoint _dirX, FixedPointMath::fixedpoint _dirY) :
		PositionImage(_x, _y), dirX(_dirX), dirY(_dirY) {

	}

	inline FixedPointMath::fixedpoint getDirX() const {
		return dirX;
	}

	inline FixedPointMath::fixedpoint getDirY() const {
		return dirY;
	}

	inline void setDirX(FixedPointMath::fixedpoint _dirX) {
		dirX = _dirX;
	}

	inline void setDirY(FixedPointMath::fixedpoint _dirY) {
		dirY = _dirY;
	}

protected:
	FixedPointMath::fixedpoint dirX;   //!< orientation in this point in x direction
	FixedPointMath::fixedpoint dirY;   //!< orientation in this point in y direction
};

/**
 * This class represents an edge, which consists of multiple line points.
 * An edge represents structure of the real world in the image domain.
 */
class Edge  : public Paintable {
public:
	FixedPointMath::fixedpoint dirX;        //!< orientation of edge in x direction
	FixedPointMath::fixedpoint dirY;        //!< orientation of edge in y direction
	std::vector<EdgePoint> linePoints;      //!< vector of linepoints

	int16_t colorCounter[9];                //!< accumulates the frequency of the colors
	Color mostFrequentColor1;               //!< most frequent color on both sides of edge
	Color mostFrequentColor2;               //!< second most frequent color on both sides of edge

	Edge() {
		dirX = toFixed(0);
		dirY = toFixed(0);
		mostFrequentColor1 = Unknown;
		mostFrequentColor2 = Unknown;
	}
	virtual ~Edge() { linePoints.clear(); }

	int distance(const Edge &edge);
	inline EdgePoint getFirstPoint() const { return linePoints.front(); }
	inline EdgePoint getLastPoint() const { return linePoints.back(); }

	BoundingBox getBoundingBox();

	void paint(IplImage *img, int scale);

	static std::vector<const Edge*> edgesNearToPoint(int16_t x, int16_t y, int16_t distance, const std::vector<Edge*> &edges);
	static Edge* getEdgeWithPoint(int16_t x, int16_t y, std::vector<Edge*> &edges);
};

typedef std::vector<Edge*> EdgeVector;

/**
 * @}
 */

#endif /* EDGE_H_ */
