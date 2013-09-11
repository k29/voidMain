#include "objectExtractor_goal.h"
#include "objectExtractor.h"
#include "robot.h"
#include "vision.h"
#include "objectExtractor_field.h"

GoalExtractor::GoalExtractor() {
	blueGoalLeft = new RectangleObject(GOAL_POLE_LEFT_OBJECT, BlueGoal);
	blueGoalRight = new RectangleObject(GOAL_POLE_RIGHT_OBJECT, BlueGoal);
	yellowGoalLeft = new RectangleObject(GOAL_POLE_LEFT_OBJECT, YellowGoal);
	yellowGoalRight = new RectangleObject(GOAL_POLE_RIGHT_OBJECT, YellowGoal);

	histogramBlueContour = 0;
	histogramYellowContour = 0;
	histogramSize = 0;
}

GoalExtractor::~GoalExtractor() {
	delete blueGoalLeft;
	delete blueGoalRight;
	delete yellowGoalLeft;
	delete yellowGoalRight;
	blueGoalLeft = 0;
	blueGoalRight = 0;
	yellowGoalLeft = 0;
	yellowGoalRight = 0;

	delete[] histogramBlueContour;
	histogramBlueContour = 0;
	delete[] histogramYellowContour;
	histogramYellowContour = 0;
}

/**
 * Clear all data
 */
void GoalExtractor::clear() {
	yellowGoalRight->clear();
	yellowGoalLeft->clear();
	yellowGoalLeft->type = GOAL_POLE_LEFT_OBJECT;

	blueGoalRight->clear();
	blueGoalLeft->clear();
	blueGoalLeft->type = GOAL_POLE_LEFT_OBJECT;

	memset(histogramBlueContour, 0, sizeof(int16_t) * histogramSize);
	memset(histogramYellowContour, 0, sizeof(int16_t) * histogramSize);
	ybyPoleBoundingBoxes.clear();
	bybPoleBoundingBoxes.clear();
	potentialYellowGoalX.clear();
	potentialBlueGoalX.clear();
}

/**
 * Extracts the goals with help of the field contour.
 * @param edges
 * @return
 */
bool GoalExtractor::extract(const EdgeVector &edges) {

	uint16_t width = image->getImageWidth();

	if(width != histogramSize) {
		histogramSize = width;
		if(histogramBlueContour != 0) {
			delete[] histogramBlueContour;
			histogramBlueContour = 0;
		}
		if(histogramYellowContour != 0) {
			delete[] histogramYellowContour;
			histogramYellowContour = 0;
		}
		histogramBlueContour = new int16_t[histogramSize];
		histogramYellowContour = new int16_t[histogramSize];
	}

	clear();

	bool goalFound = extractFieldContour();
/*
	uint16_t widthFifth = TheCameraModel::getInstance().focalLengthX() / 5;//width / 5;

	BoundingBox yl(yellowGoalLeft->rectangle, yellowGoalLeft->basePoint);
	BoundingBox yr(yellowGoalRight->rectangle, yellowGoalRight->basePoint);
	BoundingBox bl(blueGoalLeft->rectangle, blueGoalLeft->basePoint);
	BoundingBox br(blueGoalRight->rectangle, blueGoalRight->basePoint);

	// check, if we see both goals
	if((yellowGoalLeftSeen() || yellowGoalRightSeen() || yellowGoalUnknownSeen()) && (blueGoalLeftSeen() || blueGoalRightSeen() || blueGoalUnknownSeen())) {

		// we see the whole yellow goal
		if(yellowGoalSeen()) {

			if(blueGoalSeen() && min(yl.distance(br), yr.distance(bl)) < 2* widthFifth) {
				// check distance between goals -> it must be higher than 2/5 of the whole image width!
				// otherwise delete goals
				yellowGoalLeft->clear();
				yellowGoalRight->clear();
				blueGoalLeft->clear();
				blueGoalRight->clear();
			}
			else if((blueGoalLeftSeen() || blueGoalUnknownSeen()) && min(yl.distance(bl), yr.distance(bl)) < 2* widthFifth) {
				// check distance between goals -> it must be higher than 2/5 of the whole image width!
				// otherwise delete the smaller goal (with just one pole)
				blueGoalLeft->clear();
			}
			else if(blueGoalRightSeen() && min(yl.distance(br), yr.distance(br)) < 2* widthFifth) {
				// check distance between goals -> it must be higher than 2/5 of the whole image width!
				// otherwise delete the smaller goal (with just one pole)
				blueGoalRight->clear();
			}
		}

		else if (yellowGoalLeftSeen()) {

			if(blueGoalSeen() && min(yl.distance(br), yl.distance(bl)) < 2* widthFifth) {
				// check distance between goals -> it must be higher than 2/5 of the whole image width!
				// otherwise delete smaller goal (with just one pole)
				yellowGoalLeft->clear();
			}
			else if(blueGoalLeftSeen() && yl.distance(bl) < 2* widthFifth) {
				// check distance between goals -> it must be higher than 2/5 of the whole image width!
				// otherwise delete both poles // TODO check for BYB or YBY pole
				blueGoalLeft->clear();
				yellowGoalLeft->clear();
			}
			else if(blueGoalRightSeen() && yl.distance(br) < 2* widthFifth) {
				// check distance between goals -> it must be higher than 2/5 of the whole image width!
				// otherwise delete both poles // TODO check for BYB or YBY pole
				blueGoalRight->clear();
				yellowGoalLeft->clear();
			}
			else if (blueGoalUnknownSeen() && yl.distance(bl) < 2* widthFifth) {
				// check distance between goals -> it must be higher than 2/5 of the whole image width!
				// otherwise delete unknown pole
				blueGoalLeft->clear();
			}
		}

		else if (yellowGoalRightSeen()) {

			if(blueGoalSeen() && min(yr.distance(br), yr.distance(bl)) < 2*widthFifth) {
				// check distance between goals -> it must be higher than 2/5 of the whole image width!
				// otherwise delete smaller goal (with just one pole)
				yellowGoalRight->clear();
			}
			else if(blueGoalLeftSeen() && yr.distance(bl) < 2* widthFifth) {
				// check distance between goals -> it must be higher than 2/5 of the whole image width!
				// otherwise delete both poles // TODO check for BYB or YBY pole
				blueGoalLeft->clear();
				yellowGoalRight->clear();
			}
			else if(blueGoalRightSeen() && yr.distance(br) < 2* widthFifth) {
				// check distance between goals -> it must be higher than 2/5 of the whole image width!
				// otherwise delete both poles // TODO check for BYB or YBY pole
				blueGoalRight->clear();
				yellowGoalRight->clear();
			}
			else if(blueGoalUnknownSeen() && yr.distance(bl) < 2* widthFifth) {
				// check distance between goals -> it must be higher than 2/5 of the whole image width!
				// otherwise delete unknown pole // TODO check for BYB or YBY pole
				blueGoalLeft->clear();
			}
		}

		else if (yellowGoalUnknownSeen()) {

			if(blueGoalSeen() && min(yl.distance(br), yl.distance(bl)) < 2* widthFifth) {
				// check distance between goals -> it must be higher than 2/5 of the whole image width!
				// otherwise delete smaller goal (with just one pole)
				yellowGoalLeft->clear();
			}
			else if(blueGoalLeftSeen() && yl.distance(bl) < 2* widthFifth) {
				// check distance between goals -> it must be higher than 2/5 of the whole image width!
				// otherwise delete unknown pole
				yellowGoalLeft->clear();
			}
			else if(blueGoalRightSeen() && yl.distance(br) < 2* widthFifth) {
				// check distance between goals -> it must be higher than 2/5 of the whole image width!
				// otherwise delete unknown pole
				yellowGoalLeft->clear();
			}
			else if (blueGoalUnknownSeen() && yl.distance(bl) < 2 * widthFifth) {
				// check distance between goals -> it must be higher than 2/5 of the whole image width!
				// otherwise delete both unknown poles // TODO check for BYB or YBY pole
				if(bl.rectangle.width * bl.rectangle.height > yl.rectangle.width * yl.rectangle.height) {
					yellowGoalLeft->clear();
				}
				else {
					blueGoalLeft->clear();
				}
			}
		}
	}
	*/

	// dismiss poles, if to near to the border of the image
//	if(blueGoalLeft->basePoint.x != -1 && (blueGoalLeft->basePoint.x < widthFifth || blueGoalLeft->basePoint.x > 4 * widthFifth)) {
//		blueGoalLeft->clear();
//	}
//	if(yellowGoalLeft->basePoint.x != -1 && (yellowGoalLeft->basePoint.x < widthFifth || yellowGoalLeft->basePoint.x > 4 * widthFifth)) {
//		yellowGoalLeft->clear();
//	}
//	if(blueGoalRight->basePoint.x != -1 && (blueGoalRight->basePoint.x < widthFifth || blueGoalRight->basePoint.x > 4 * widthFifth)) {
//		blueGoalRight->clear();
//	}
//	if(yellowGoalRight->basePoint.x != -1 && (yellowGoalRight->basePoint.x < widthFifth || yellowGoalRight->basePoint.x > 4 * widthFifth)) {
//		yellowGoalRight->clear();
//	}


	if(yellowGoalLeftSeen() || yellowGoalUnknownSeen()) {
		setProtobuf(yellowGoalLeft);
	}
	if(yellowGoalRightSeen()) {
		setProtobuf(yellowGoalRight);
	}
	if(blueGoalLeftSeen() || blueGoalUnknownSeen()) {
		setProtobuf(blueGoalLeft);
	}
	if(blueGoalRightSeen()) {
		setProtobuf(blueGoalRight);
	}

	return goalFound;
}


/**
 * Extract the goals using the interpolated field contour.
 * We build a color histogram along the field contour and analyze this histogram for peaks (with threshold).
 * Then try to find goal / poles near the peaks.
 *
 * @return
 */
bool GoalExtractor::extractFieldContour() {
	if(image == 0)
		return false;

	// generate histograms and potential positions
	generatePotentialPositionsWithContourHistogram(histogramBlueContour, histogramYellowContour, potentialBlueGoalX, potentialYellowGoalX, BlueGoal, YellowGoal);

	// find goals/ poles near the maxima and create bounding boxes
	std::list<BoundingBox> blueBoundingBoxes, yellowBoundingBoxes;
	createBoundingBoxesAndClassify(potentialBlueGoalX, YellowGoal, BlueGoal, ybyPoleBoundingBoxes, bybPoleBoundingBoxes, blueBoundingBoxes);
	createBoundingBoxesAndClassify(potentialYellowGoalX, BlueGoal, YellowGoal, bybPoleBoundingBoxes, ybyPoleBoundingBoxes, yellowBoundingBoxes);

	// select the biggest ones, if we have more the two

	BoundingBox blueMax1, blueMax2;
	BoundingBox yellowMax1, yellowMax2;


	// merge rectangles
	std::list<BoundingBox> mergedYellow, mergedBlue;
	BoundingBox::mergeRectsInDistance(yellowBoundingBoxes, mergedYellow, 10, 10);
	BoundingBox::mergeRectsInDistance(blueBoundingBoxes, mergedBlue, 10, 10);

	std::list<BoundingBox>::iterator iter = mergedYellow.begin();
	while(iter != mergedYellow.end()) {
		const BoundingBox &b = *iter;

		int bsize = b.rectangle.height * b.rectangle.width;
		if(bsize == 0) {
			bsize = b.rectangle.height + b.rectangle.width;
		}

		if((bsize < 10 && b.rectangle.height != 0 && b.rectangle.width != 0) || max(b.rectangle.height, b.rectangle.width) < 5) {
			// ignore
		}
		else if(bsize > yellowMax1.rectangle.width * yellowMax1.rectangle.height || bsize > yellowMax1.rectangle.width + yellowMax1.rectangle.height) { // bigger then max
			yellowMax2.set(yellowMax1.rectangle, yellowMax1.basePoint, yellowMax1.numberOfEdgePoints);
			yellowMax1.set(b.rectangle, b.basePoint, b.numberOfEdgePoints);
		}
		else if(bsize > yellowMax2.rectangle.width * yellowMax2.rectangle.height || bsize > yellowMax2.rectangle.width + yellowMax2.rectangle.height) { // bigger then 2nd max
			yellowMax2.set(b.rectangle, b.basePoint, b.numberOfEdgePoints);
		}
		++iter;
	}

	iter = mergedBlue.begin();
	while(iter != mergedBlue.end()) {
		const BoundingBox &b = *iter;

		int bsize = b.rectangle.height * b.rectangle.width;
		if(bsize == 0) {
			bsize = b.rectangle.height + b.rectangle.width;
		}

		if((bsize < 10 && b.rectangle.height != 0 && b.rectangle.width != 0) || max(b.rectangle.height, b.rectangle.width) < 5) {
			// ignore
		}
		else if(bsize > blueMax1.rectangle.width * blueMax1.rectangle.height || bsize > blueMax1.rectangle.width + blueMax1.rectangle.height) { // bigger then max
			blueMax2.set(blueMax1.rectangle, blueMax1.basePoint, blueMax1.numberOfEdgePoints);
			blueMax1.set(b.rectangle, b.basePoint, b.numberOfEdgePoints);
		}
		else if(bsize > blueMax2.rectangle.width * blueMax2.rectangle.height || bsize > blueMax2.rectangle.width * blueMax2.rectangle.height) { // bigger then 2nd max
			blueMax2.set(b.rectangle, b.basePoint, b.numberOfEdgePoints);
		}
		++iter;
	}

	// set objects

	bool found = false;
	// we have left and right pole
	if(blueMax2.rectangle.x != -1) {

		if(blueMax1.basePoint.x < blueMax2.basePoint.x) { // blue1 is left
			blueGoalLeft->set(blueMax1.rectangle, blueMax1.basePoint);
			blueGoalRight->set(blueMax2.rectangle, blueMax2.basePoint);
		}
		else {
			blueGoalLeft->set(blueMax2.rectangle, blueMax2.basePoint);
			blueGoalRight->set(blueMax1.rectangle, blueMax1.basePoint);
		}
		found = true;
	} // we have just one pole
	else if (blueMax1.rectangle.x != -1) { // just one pole
		blueGoalLeft->set(blueMax1.rectangle, blueMax1.basePoint);
		blueGoalLeft->type = GOAL_POLE_UNKNOWN_OBJECT;
		found = true;
	}

	// we have left and right yellow poles
	if(yellowMax2.rectangle.x != -1) {

		if(yellowMax1.basePoint.x < yellowMax2.basePoint.x) { // blue1 is left
			yellowGoalLeft->set(yellowMax1.rectangle, yellowMax1.basePoint);
			yellowGoalRight->set(yellowMax2.rectangle, yellowMax2.basePoint);
		}
		else {
			yellowGoalLeft->set(yellowMax2.rectangle, yellowMax2.basePoint);
			yellowGoalRight->set(yellowMax1.rectangle, yellowMax1.basePoint);
		}
		found = true;
	} // just one pole
	else if(yellowMax1.rectangle.x != -1) { // just one pole
		yellowGoalLeft->set(yellowMax1.rectangle, yellowMax1.basePoint);
		yellowGoalLeft->type = GOAL_POLE_UNKNOWN_OBJECT;
		found = true;
	}

	return found;
}

/**
 * Generates potential goal / pole positions by generating a histogram of the number of color-hits along the field contour
 * The histograms of both colors get filtered and only the peaks are then the potential positions.
 *
 * @param histogramColor1
 * @param histogramColor2
 * @param potentialPositionsColor1 x position on the field contour of a potential goal / pole of color1
 * @param potentialPositionsColor2 x position on the field contour of a potential goal / pole of color1
 * @param color1
 * @param color2
 */
void GoalExtractor::generatePotentialPositionsWithContourHistogram(int16_t *histogramColor1, int16_t *histogramColor2, std::vector<int16_t> &potentialPositionsColor1, std::vector<int16_t> &potentialPositionsColor2, Color color1, Color color2) {
	uint16_t width = image->getImageWidth();

	// follow the field contour and analyze the colors on the way
	// and build a histogram.
	// each value in the histogram represents, how many pixels with the right color where in front of it.

	uint8_t colorNeighbourhood[9] = { 0 };

	for(int16_t x = 1; x < width; ++x) {
		int16_t y = Vision::getInstance().getFieldExtractor().getHeighestFieldCoordinate(x);

		Color c = colorMgr->getPixelColor(*image, x, y);

		Color c2 = Unknown;
		if(y - 10 >= 0) {
			c2 = colorMgr->getPixelColor(*image, x, y - 10);
		}
		else {
			c2 = c;
		}

		if ( c == color1 || c2 == color1 ) {
			getColorsInNeighbourhood(x, y, colorNeighbourhood);
			int count1 = colorNeighbourhood[(int) color1];

			getColorsInNeighbourhood(x, y - 10, colorNeighbourhood);
			int count2 = colorNeighbourhood[(int) color1];

			if (count1 > 3 || count2 > 3) {
				histogramColor1[x] = histogramColor1[x - 1] + 1;
			}
		}
		else {
			histogramColor1[x] = histogramColor1[x - 1] - 1;
			if(histogramColor1[x] < 0) {
				histogramColor1[x] =  0;
			}
		}
		if(c == color2 || c2 == color2) {
			getColorsInNeighbourhood(x, y, colorNeighbourhood);
			int count1 = colorNeighbourhood[(int) color2];

			getColorsInNeighbourhood(x, y - 10, colorNeighbourhood);
			int count2 = colorNeighbourhood[(int) color2];

			if (count1 > 3 || count2 > 3) {
				histogramColor2[x] = histogramColor2[x - 1] + 1;
			}
		}
		else {
			histogramColor2[x] = histogramColor2[x - 1] -1;
			if(histogramColor2[x] < 0) {
				histogramColor2[x] = 0;
			}
		}
	}

	// search maxima of histogram (with threshold)

	// TODO peaks need certain distance
	int16_t peakThreshold = 1;
	for(int16_t i = 1; i < width - 1; ++i) {
		if(histogramColor1[i] >  peakThreshold) {
			if(histogramColor1[i] > histogramColor1[i - 1] && histogramColor1[i] > histogramColor1[i + 1]) {
				potentialPositionsColor1.push_back(i);
			}
		}

		if(histogramColor2[i] > peakThreshold) {
			if(histogramColor2[i] > histogramColor2[i - 1] && histogramColor2[i] > histogramColor2[i+1]) {
				potentialPositionsColor2.push_back(i);
			}
		}
	}
}

/**
 * Create bounding boxes out of the possible positions and classify them as goal or pole
 *
 * @param possiblePositions  potential x coordinates (corresponding to the contour histogram) for a goal of color2
 * @param color1
 * @param color2
 * @param possiblePole1 list of possible poles color1-color2-color1
 * @param possiblePole2 list of possible poles color2-color1-color2
 * @param possibleGoal list of possible goals for color2
 */
void GoalExtractor::createBoundingBoxesAndClassify(const std::vector<int16_t> &possiblePositions, Color color1, Color color2, std::list<BoundingBox> &possiblePole1, std::list<BoundingBox> &possiblePole2, std::list<BoundingBox> &possibleGoal) {
	uint16_t width = image->getImageWidth();
	uint16_t height = image->getImageHeight();

	for(int16_t i = 0; i < (int16_t) possiblePositions.size(); ++i) {

		int16_t xRight = possiblePositions.at(i);
		int16_t xLeft = xRight;

		// calculate right bounds
		int16_t y = Vision::getInstance().getFieldExtractor().getHeighestFieldCoordinate(xRight);
		int16_t missWidth = 0;
		for(int16_t x = xRight; x >= 0; x -= 2) {
			Color c = colorMgr->getPixelColor(*image, x, y);
			if(c == color2) {
				xLeft = x;
			}
			else {
				++missWidth;
				if(missWidth > 3) {
					break;
				}
			}
		}
		for(int16_t x = xRight + 2; x < width; x += 2) {
			Color c = colorMgr->getPixelColor(*image, x, y);
			if(c == color2) {
				xRight = x;
			}
			else {
				++missWidth;
				if(missWidth > 3) {
					break;
				}
			}
		}

		int16_t width = xRight - xLeft;
		int16_t xCenter = xLeft + width / 2;
		int16_t originalXCenter = xCenter;
		int16_t yMax = Vision::getInstance().getFieldExtractor().getHeighestFieldCoordinate(xCenter);
		int16_t yMin = -1;
		bool inFirstSegment = true;

		int16_t countColor1SegmentUp = 0;
		int16_t countColor2Segment1Up = 0;
		int16_t countColor2Segment2Up = 0;

		// go up as long as we see color1 or color2
		int16_t miss = 0;
		for(int16_t y = yMax; y >= 0; y -= 3) {
			Color c = colorMgr->getPixelColor(*image, xCenter, y);
			if(c != color1 && c != color2) {
				++miss;
				if(miss > 3) {
					break;
				}
			}
			else {
				if(c == color2 && inFirstSegment) {
					++countColor2Segment1Up;
				}
				else if (c == color2 && !inFirstSegment){
					++countColor2Segment2Up;
				}
				if (c == color1) { // we reached the end of the color2 segment -> it's a pole
					inFirstSegment = false;
					++countColor1SegmentUp;
				}
				miss = 0;
				yMin = y;

				xCenter = getCenterXFromPole(xCenter, y, color1, color2);

				// get boundary
				if (xCenter > xRight) {
					xRight = xCenter;
				}
				if (xCenter < xLeft) {
					xLeft = xCenter;
				}
			}
		}

		// go down as long as we see color1 or color2
		int16_t countColor1SegmentDown = 0;
		int16_t countColor2Segment1Down = 0;
		int16_t countColor2Segment2Down = 0;
		miss = 0;
		xCenter = originalXCenter;
		inFirstSegment = true;
		for(int16_t y = yMax + 3; y < height; y += 3) {
			Color c = colorMgr->getPixelColor(*image, xCenter, y);
			if(c != color1 && c != color2) {
				++miss;
				if(miss > 3) {
					break;
				}
			}
			else {
				if(c == color2 && inFirstSegment) {
					++countColor2Segment1Down;
				}
				else if (c == color2 && !inFirstSegment){
					++countColor2Segment2Down;
				}
				if (c == color1) { // we reached the end of the color2 segment -> it's a pole
					inFirstSegment = false;
					++countColor1SegmentDown;
				}
				miss = 0;
				yMax = y;

				xCenter = getCenterXFromPole(xCenter, y, color1, color2);

				// get boundary
				if (xCenter > xRight) {
					xRight = xCenter;
				}
				if (xCenter < xLeft) {
					xLeft = xCenter;
				}
			}
		}

		BoundingBox bb(cvRect(xLeft, yMin, xRight - xLeft, yMax - yMin), cvPoint(xCenter, yMax));

		int countColor2Segment1 = countColor2Segment1Down + countColor2Segment1Up;
		int countColor2Segment2 = countColor2Segment2Down + countColor2Segment2Up;
		int countColor1Segment = countColor1SegmentDown + countColor1SegmentUp;

		if (countColor2Segment1 > 0 && countColor2Segment2 > 0 && countColor1Segment > 0) { // color2-color1-color2 pole
			if(countColor2Segment1 + countColor2Segment2 < countColor1Segment * 3) {
				possiblePole2.push_back(bb);
			}
		}
		else if (countColor2Segment1 > 0 && countColor2Segment2 == 0 && countColor1Segment > 0) { // color1-color2-color1 pole
			possiblePole1.push_back(bb);
		}
		else if (countColor2Segment1 > 0) { // goal with color2
			possibleGoal.push_back(bb);
		}

	}
}

/**
 * Due to the high distortion of the fish-eye-lense, we need to adjust the center point of the
 * poles while analyzing it for potential side-poles.
 *
 * @param oldCenterX old center x position
 * @param y corresponding y position
 * @param c1 colors which are allowed
 * @param c2 colors which are allowed
 * @return new center x position
 */
int16_t GoalExtractor::getCenterXFromPole(int16_t oldCenterX, int16_t y, Color c1, Color c2) {
	uint16_t width = image->getImageWidth();

	int16_t xLeft = oldCenterX;
	int16_t xRight = oldCenterX;

	int16_t miss = 0;
	for(int16_t x = oldCenterX - 2; x >= 0; x -= 2) {
		Color c = colorMgr->getPixelColor(*image, x, y);
		if(c == c1 || c == c2) {
			xLeft = x;
		}
		else {
			++miss;
			if(miss > 3) {
				break;
			}
		}
	}
	miss = 0;
	for(int16_t x = oldCenterX + 2; x < width; x += 2) {
		Color c = colorMgr->getPixelColor(*image, x, y);
		if(c == c1 || c == c2) {
			xRight = x;
		}
		else {
			++miss;
			if(miss > 3) {
				break;
			}
		}
	}

	return (xLeft + xRight) / 2;
}

void GoalExtractor::setProtobuf(RectangleObject *obj) {
#if VISION_DEBUG
	de::fumanoids::message::ObjectPosition *goalObjStatus = protobufExtractorStatus->add_objectpositions();
	if(obj->color == BlueGoal)
		goalObjStatus->set_type(de::fumanoids::message::ObjectPosition_ObjectType_GoalBlue);
	else
		goalObjStatus->set_type(de::fumanoids::message::ObjectPosition_ObjectType_GoalYellow);
	de::fumanoids::message::Position *goalObjStatusPos = goalObjStatus->mutable_position();
	goalObjStatusPos->set_positiontype(de::fumanoids::message::Position_PositionType_IMAGE);
	goalObjStatusPos->set_x(obj->basePoint.x);
	goalObjStatusPos->set_y(obj->basePoint.y);
	goalObjStatus->set_isused(true);
	if(obj->type == GOAL_POLE_UNKNOWN_OBJECT)
		goalObjStatus->set_comment("Unknown");
	else if(obj->type == GOAL_POLE_LEFT_OBJECT)
		goalObjStatus->set_comment("Left");
	else
		goalObjStatus->set_comment("Right");
#endif
}
