#ifndef OBJECTEXTRACTOR_BALL_H_
#define OBJECTEXTRACTOR_BALL_H_

#include "objectExtractor_base.h"
#include "object.h"

/**
 * This class defines the extraction algorithm for the ball.
 *
 * @ingroup vision
 */
class BallExtractor : public Extractor {
public:
	BallExtractor();
	virtual ~BallExtractor();

	/**
	 * Clears all data
	 */
	inline void clear() {
		ball->clear();
	}

	bool extract(const EdgeVector &edges);

	/**
	 * Returns the ball
	 * @return current ball
	 */
	inline RectangleObject* getBall() {
		return ball;
	}

	/**
	 * Is the ball currently seen?
	 * @return
	 */
	inline bool ballSeen() {
		return ball->rectangle.x != -1;
	}

	/**
	 * Sets the feetspace
	 * @param fs
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
	RectangleObject *ball;   //!< the ball object
	BoundingBox feetSpace;   //!< the feetspace
};

#endif /* OBJECTEXTRACTOR_BALL_H_ */
