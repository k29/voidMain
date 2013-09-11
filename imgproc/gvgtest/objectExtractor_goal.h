#ifndef OBJECTEXTRACTOR_GOAL_H_
#define OBJECTEXTRACTOR_GOAL_H_

#include "objectExtractor_base.h"
#include "object.h"

/**
 * Defines the goal extraction algorithm.
 * The algorithm uses the extracted field contour (so the field extraction needs to be done before the goal extraction starts!)
 * to determine the position of the goals.
 * The idea is that the field contour crosses in some points the goal poles,
 * because the goals are standing at the border of the field.
 * A histogram is build for both goal colors along the field contour. The peaks are potential goal positions.
 * By further analysis of this interesting points, we can determine exactly the goal position.
 *
 * @ingroup vision
 */
class GoalExtractor : public Extractor {
public:
	GoalExtractor();
	virtual ~GoalExtractor();

	void clear();
	bool extract(const EdgeVector &edges);

	inline bool blueGoalLeftSeen() {
		return blueGoalLeft->rectangle.x != -1 && blueGoalLeft->type == GOAL_POLE_LEFT_OBJECT;
	}

	inline bool blueGoalRightSeen() {
		return blueGoalRight->rectangle.x != -1;
	}

	inline bool blueGoalUnknownSeen() {
		return blueGoalLeft->rectangle.x != -1 && blueGoalLeft->type == GOAL_POLE_UNKNOWN_OBJECT;
	}

	/**
	 * Checks, if the whole blue goal was seen
	 * @return
	 */
	inline bool blueGoalSeen() {
		return blueGoalLeftSeen() && blueGoalRightSeen();
	}

	inline bool yellowGoalLeftSeen() {
		return yellowGoalLeft->rectangle.x != -1 && yellowGoalLeft->type == GOAL_POLE_LEFT_OBJECT;
	}

	inline bool yellowGoalRightSeen() {
		return yellowGoalRight->rectangle.x != -1;
	}

	inline bool yellowGoalUnknownSeen() {
		return yellowGoalLeft->rectangle.x != -1 && yellowGoalLeft->type == GOAL_POLE_UNKNOWN_OBJECT;
	}

	/**
	 * Checks, if the whole yellow goal was seen
	 * @return
	 */
	inline bool yellowGoalSeen() {
		return yellowGoalLeftSeen() && yellowGoalRightSeen();
	}

	inline RectangleObject* getBlueGoalLeft() {
		return blueGoalLeft;
	}

	inline RectangleObject* getBlueGoalRight() {
		return blueGoalRight;
	}

	inline RectangleObject* getYellowGoalLeft() {
		return yellowGoalLeft;
	}

	inline RectangleObject* getYellowGoalRight() {
		return yellowGoalRight;
	}

	inline std::list<BoundingBox>& getYBYPoleBoundingBoxes() {
		return ybyPoleBoundingBoxes;
	}

	inline std::list<BoundingBox>& getBYBPoleBoundingBoxes() {
		return bybPoleBoundingBoxes;
	}

	inline void reset(RectangleObject *obj) {
		if(obj == yellowGoalLeft || obj == yellowGoalRight ||
				obj == blueGoalLeft || obj == blueGoalRight) {
			obj->clear();
		}
	}


protected:
	friend class ImagePresenter;

	RectangleObject *yellowGoalLeft;              //!< left goal pole of yellow goal (also used, if only one unknown pole is seen)
	RectangleObject *yellowGoalRight;             //!< right goal pole of yellow goal
	RectangleObject *blueGoalLeft;                //!< left goal pole of blue goal (also used, if only one unknown pole is seen)
	RectangleObject *blueGoalRight;               //!< right goal pole of blue goal

	std::vector<int16_t> potentialBlueGoalX;
	std::vector<int16_t> potentialYellowGoalX;

	int16_t *histogramBlueContour;                //!< histogram of blue occurrences along the field contour
	int16_t *histogramYellowContour;              //!< histogram of yellow occurrences along the field contour
	uint16_t histogramSize;                         //!< size of the histograms (corresponds to the image width)

	std::list<BoundingBox> ybyPoleBoundingBoxes;  //!< possible YBY-pole positions (can be used by the PoleExtractor)
	std::list<BoundingBox> bybPoleBoundingBoxes;  //!< possible BYB-pole positions (can be used by the PoleExtractor)

	bool extractFieldContour();
	void generatePotentialPositionsWithContourHistogram(int16_t *histogramColor1, int16_t *histogramColor2, std::vector<int16_t> &potentialPositionsColor1, std::vector<int16_t> &potentialPositionsColor2, Color color1, Color color2);
	void createBoundingBoxesAndClassify(const std::vector<int16_t> &possiblePositions, Color color1, Color color2, std::list<BoundingBox> &possiblePole1, std::list<BoundingBox> &possiblePole2, std::list<BoundingBox> &possibleGoal);

	int16_t getCenterXFromPole(int16_t oldCenterX, int16_t y, Color c1, Color c2);

	void setProtobuf(RectangleObject *object);
};

#endif /* OBJECTEXTRACTOR_GOAL_H_ */
