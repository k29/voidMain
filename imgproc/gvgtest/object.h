#ifndef OBJECT_H_
#define OBJECT_H_

#include "paintable.h"
#include "color.h"

/**
 * @{
 * @ingroup vision
 */

/**
 * Defines the object type.
 */
typedef enum {
	NO_OBJECT,               //!< NO_OBJECT
	LINE_OBJECT,             //!< LINE_OBJECT
	GOAL_POLE_UNKNOWN_OBJECT,//!< GOAL_POLE_UNKNOWN_OBJECT
	GOAL_POLE_RIGHT_OBJECT,  //!< GOAL_POLE_RIGHT_OBJECT
	GOAL_POLE_LEFT_OBJECT,   //!< GOAL_POLE_LEFT_OBJECT
	BALL_OBJECT,             //!< BALL_OBJECT
	PLAYER_OBJECT,           //!< PLAYER_OBJECT
	POLE_OBJECT,             //!< POLE_OBJECT
	OBSTACLE_OBJECT          //!< OBSTACLE_OBJECT
} ObjectType;

typedef std::vector<CvPoint> PointVector; //!< defines a vector of CVPoint

/**
 * Defines the base vision-object.
 * An object is defined by its type (like ball, goal etc.) and its color.
 *
 * @ingroup vision
 */
class Object : public Paintable {
public:
	Object();
	virtual ~Object();

	ObjectType type;   //!< type of object
	Color color;       //!< color of object

	virtual void paint(IplImage *img, int scale) { }
};

/**
 * Defines an vision-object which can be represented as a rectangle on the image.
 * The object is defined by the bounding box (convex hull) in the image and
 * a base point, which is the lowest point of the object (on the ground).
 *
 * @ingroup vision
 */
class RectangleObject : public Object {
public:
	RectangleObject() {
		type = NO_OBJECT;
		color = Unknown;
		clear();
	}

	RectangleObject(ObjectType _type, Color _color) {
		type = _type;
		color = _color;
		clear();
	}

	/// Rectangle describing/ containing the object
	CvRect rectangle;

	/**
	 * Lowest point of object in image coordinates, should be the point
	 * where the object touches the ground/field
	 */
	CvPoint basePoint;

	/**
	 * Sets the object to bounding box r and basepoint p
	 *
	 * @param r bounding box
	 * @param p basepoint
	 */
	inline void set(const CvRect &r, const CvPoint &p) {
		rectangle.x = r.x;
		rectangle.y = r.y;
		rectangle.width = r.width;
		rectangle.height = r.height;

		basePoint.x = p.x;
		basePoint.y = p.y;
	}

	/**
	 * Resets the object to its default values
	 */
	inline void clear() {
		rectangle.x = -1;
		rectangle.y = -1;
		rectangle.width = 0;
		rectangle.height = 0;
		basePoint.x = -1;
		basePoint.y = -1;
	}

	void paint(IplImage *img, int scale);
};

/**
 * Defines a vision object which can be represented as a cluster of lines.
 *
 * @ingroup vision
 */
class ClusterObject : public Object {
public:
	virtual ~ClusterObject() {
		for(uint i = 0; i < cluster.size(); ++i) {
			cluster[i].clear();
		}
		cluster.clear();
	}

	std::vector<PointVector> cluster;  //!< object lines

	inline void clear() {
		cluster.clear();
	}

	void paint(IplImage *img, int scale);
};

/**
 * @}
 */

#endif /* OBJECT_H_ */
