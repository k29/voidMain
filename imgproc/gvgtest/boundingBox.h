#ifndef BOUNDINGBOX_H_
#define BOUNDINGBOX_H_

#include <opencv/cv.h>
#include <vector>
#include <list>
#include <inttypes.h>
#include "math/utils.h"
#include "paintable.h"


/**
 * Class representing bounding boxes used by vision.
 *
 * @ingroup vision
 */
class BoundingBox : public Paintable {
public:
	/// rectangle describing the bounds of this box
	CvRect rectangle;
	/// the basepoint, extracted from the edges, which are building this box
	CvPoint basePoint;
	/// the number of edge points creating this box
	int16_t numberOfEdgePoints;

	BoundingBox();
	BoundingBox(const CvRect &r, const CvPoint &p, const int16_t nrEdgePoints=0);

	virtual ~BoundingBox();

	/**
	 * Sets the bounding box to the rectangle
	 *
	 * @param r rectangle
	 * @param p basepoint
	 */
	inline void set(const CvRect &r, const CvPoint &p, const int16_t nrEdgePoints) {
		rectangle.x = r.x;
		rectangle.y = r.y;
		rectangle.width = r.width;
		rectangle.height = r.height;

		basePoint.x = p.x;
		basePoint.y = p.y;

		numberOfEdgePoints = nrEdgePoints;
	}

	/**
	 * Resets the bounding box
	 */
	inline void clear() {
		rectangle.x = -1;
		rectangle.y = -1;
		rectangle.width = 0;
		rectangle.height = 0;

		basePoint.x = -1;
		basePoint.y = -1;
	}

	/**
	 * Tests, if the bounding box contains the point p
	 *
	 * @param p
	 * @return
	 */
	inline bool contains(const CvPoint &p) {
		if(rectangle.x + rectangle.width < p.x || p.x < rectangle.x) {
			return false;
		}
		if(rectangle.y + rectangle.height < p.y || p.y < rectangle.y ) {
			return false;
		}

		return true;
	}

	/**
	 * Test, if the current rectangle intersects with the given rectangle.
	 *
	 * @param r
	 * @return
	 */
	inline bool intersects(const BoundingBox &r) {
		if (rectangle.x >= r.rectangle.x + r.rectangle.width || r.rectangle.x >= rectangle.x + rectangle.width) {
			return false;
		}
		if (rectangle.y >= r.rectangle.y + r.rectangle.height || r.rectangle.y >= rectangle.y + rectangle.height) {
			return false;
		}
		return true;
	}

	/**
	 * Returns the distance from the bounding box to point p
	 *
	 * @param p
	 * @return
	 */
	inline int16_t distance(const CvPoint &p) {
		if(this->contains(p)) {
			return 0;
		}

		int16_t distX = 0;
		if (rectangle.x > p.x) {
			distX = rectangle.x - p.x;
		} else if (p.x > rectangle.x + rectangle.width) {
			distX = p.x - (rectangle.x + rectangle.width);
		}

		int16_t distY = 0;
		if (rectangle.y > p.y) {
			distY = rectangle.y - p.y;
		} else if (p.y > rectangle.y + rectangle.height) {
			distY = p.y - (rectangle.y + rectangle.height);
		}

		return max(distX, distY);
	}

	/**
	 * Returns the maximum distance between the rectangles (a kind of Manhattan distance)
	 *
	 * @param r
	 * @return
	 */
	inline int16_t distance(const BoundingBox &r) {
		return max(this->distanceX(r), this->distanceY(r));
	}

	/**
	 * Returns the  distance in x direction between the rectangles (a kind of Manhattan distance)
	 *
	 * @param r
	 * @return
	 */
	inline int16_t distanceX(const BoundingBox &r) {
		int16_t distX = 0;
		if (rectangle.x > r.rectangle.x + r.rectangle.width) {
			distX = rectangle.x - (r.rectangle.x + r.rectangle.width);
		} else if (r.rectangle.x > rectangle.x + rectangle.width) {
			distX = r.rectangle.x - (rectangle.x + rectangle.width);
		}

		return distX;
	}

	/**
	 * Returns the  distance in y direction between the rectangles (a kind of Manhattan distance)
	 *
	 * @param r
	 * @return
	 */
	inline int16_t distanceY(const BoundingBox &r) {
		int16_t distY = 0;
		if (rectangle.y > r.rectangle.y + r.rectangle.height) {
			distY = rectangle.y - (r.rectangle.y + r.rectangle.height);
		} else if (r.rectangle.y > rectangle.y + rectangle.height) {
			distY = r.rectangle.y - (rectangle.y + rectangle.height);
		}

		return distY;
	}

	void paint(IplImage *img, int scale);

	static BoundingBox merge(const BoundingBox &r1, const BoundingBox &r2);
	static void mergeRectsInDistance(std::list<BoundingBox> &inputRects, std::list<BoundingBox> &outputRects, int16_t _distanceX, int16_t _distanceY);
};

#endif /* BOUNDINGBOX_H_ */
