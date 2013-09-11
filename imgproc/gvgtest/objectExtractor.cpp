#include "objectExtractor.h"
#include "robot.h"
#include "localization/localization.h"
#include "vision.h"
#include "gvg.h"
#include "objectExtractor_field.h"

#include "comm/protobuf/msg_vision.pb.h"

ObjectExtractor::ObjectExtractor() {
	image = 0;
	frameCounter = 0;
	criticalSection.setName("ObjectExtractor");
}

ObjectExtractor::~ObjectExtractor() { }


void ObjectExtractor::clear() {
	criticalSection.enter();

	// clear own data
	blueGoalLeft.clear();
	blueGoalRight.clear();
	yellowGoalLeft.clear();
	yellowGoalRight.clear();
	bybPole.clear();
	ybyPole.clear();
	fieldLineFeatures.clear();
	fieldLineCluster.clear();
	magentaObstacleBoxes.clear();
	cyanObstacleBoxes.clear();
	blackObstacleBoxes.clear();

	criticalSection.leave();

	ballExtractor.clear();
	goalExtractor.clear();
	poleExtractor.clear();
	fieldLineFeatureExtractor.clear();
	obstacleExtractor.clear();
	obstacleExtractorLutFree.clear();
}

/**
 * Extracts the objects from the given edges.
 * Calls all registered object extractors.
 *
 * @param edges
 */
void ObjectExtractor::extractObjects(const EdgeVector& edges) {

	if (image == 0) return;

	frameCounter++;

#if VISION_DEBUG
	de::fumanoids::message::Message protobufMessage;
	protobufExtractorStatus = protobufMessage.mutable_extractorstatus();
	ballExtractor.setProtobufExtractorStatus(protobufExtractorStatus);
	goalExtractor.setProtobufExtractorStatus(protobufExtractorStatus);
	poleExtractor.setProtobufExtractorStatus(protobufExtractorStatus);
	obstacleExtractor.setProtobufExtractorStatus(protobufExtractorStatus);
	Vision::getInstance().getFieldExtractor().setProtobufExtractorStatus(protobufExtractorStatus);
#endif

	// EXTRACT OBJECTS

	// ATTENTION: field contour extraction should have been done before!!!!

	robottime_t tmpTime, goalTime, poleTime, ballTime, obstacleTime, fieldLinesTime;

	// ball
	tmpTime = getCurrentTime();
	int useLUTFreeBall = robot.getConfig().getIntValue("vision.ballextractor.lutfree", 1);

	if (useLUTFreeBall == 1) {
		ballExtractorLutFree.extract();
	}
	else {
		ballExtractor.extract(edges);
	}

	ballTime = getCurrentTime() - tmpTime;
	DEBUG_TABLE("vision.runtimes", "Ball extraction time", ballTime);

	// goals
	tmpTime = getCurrentTime();
	goalExtractor.extract(edges);
	goalTime = getCurrentTime() - tmpTime;
	DEBUG_TABLE("vision.runtimes", "Goal extraction time", goalTime);

	// poles
	tmpTime = getCurrentTime();
	poleExtractor.setBYBPoleBoundingBoxes(goalExtractor.getBYBPoleBoundingBoxes());
	poleExtractor.setYBYPoleBoundingBoxes(goalExtractor.getYBYPoleBoundingBoxes());

	poleExtractor.extract(edges);
	poleTime = getCurrentTime() - tmpTime;
	DEBUG_TABLE("vision.runtimes", "Pole extraction time", poleTime);

	// obstacles
	int useLutFreeObstacles = robot.getConfig().getIntValue("vision.obstacles.lutfree", 0);
	tmpTime = getCurrentTime();

	if(useLutFreeObstacles == 1) {
		obstacleExtractorLutFree.extract();
	}
	// still use lut detector for cyan/magenta boxes
	obstacleExtractor.extract(edges);

	obstacleTime = getCurrentTime() - tmpTime;
	DEBUG_TABLE("vision.runtimes", "Obstacle extraction time", obstacleTime);

	//	uint16_t widthFifth = TheCameraModel::getInstance().focalLengthX() / 5;//width / 5;
	//	printf("widthFifth %u\n", widthFifth);

	// test if poles and goals are too near to each other

	BoundingBox yby(poleExtractor.getYBYPole()->rectangle, poleExtractor.getYBYPole()->basePoint);
	BoundingBox byb(poleExtractor.getBYBPole()->rectangle, poleExtractor.getBYBPole()->basePoint);

	BoundingBox yl(goalExtractor.getYellowGoalLeft()->rectangle, goalExtractor.getYellowGoalLeft()->basePoint);
	BoundingBox yr(goalExtractor.getYellowGoalRight()->rectangle, goalExtractor.getYellowGoalRight()->basePoint);
	BoundingBox bl(goalExtractor.getBlueGoalLeft()->rectangle, goalExtractor.getBlueGoalLeft()->basePoint);
	BoundingBox br(goalExtractor.getBlueGoalRight()->rectangle, goalExtractor.getBlueGoalRight()->basePoint);

	if (yby.intersects(yl)) {
		yby = BoundingBox::merge(yby, yl);
		poleExtractor.getYBYPole()->set(yby.rectangle, yby.basePoint);
		goalExtractor.getYellowGoalLeft()->clear();
	}
	if (yby.intersects(yr)) {
		yby = BoundingBox::merge(yby, yr);
		poleExtractor.getYBYPole()->set(yby.rectangle, yby.basePoint);
		goalExtractor.getYellowGoalRight()->clear();
	}
	if (yby.intersects(bl)) {
		yby = BoundingBox::merge(yby, bl);
		poleExtractor.getYBYPole()->set(yby.rectangle, yby.basePoint);
		goalExtractor.getBlueGoalLeft()->clear();
	}
	if (yby.intersects(br)) {
		yby = BoundingBox::merge(yby, br);
		poleExtractor.getYBYPole()->set(yby.rectangle, yby.basePoint);
		goalExtractor.getBlueGoalRight()->clear();
	}

	if (byb.intersects(yl)) {
		byb = BoundingBox::merge(byb, yl);
		poleExtractor.getBYBPole()->set(byb.rectangle, byb.basePoint);
		goalExtractor.getYellowGoalLeft()->clear();
	}
	if (byb.intersects(yr)) {
		byb = BoundingBox::merge(byb, yr);
		poleExtractor.getBYBPole()->set(byb.rectangle, byb.basePoint);
		goalExtractor.getYellowGoalRight()->clear();
	}
	if (byb.intersects(bl)) {
		byb = BoundingBox::merge(byb, bl);
		poleExtractor.getBYBPole()->set(byb.rectangle, byb.basePoint);
		goalExtractor.getBlueGoalLeft()->clear();
	}
	if (byb.intersects(br)) {
		byb = BoundingBox::merge(byb, br);
		poleExtractor.getBYBPole()->set(byb.rectangle, byb.basePoint);
		goalExtractor.getBlueGoalRight()->clear();
	}
/*
	if((goalExtractor.blueGoalLeftSeen() || goalExtractor.blueGoalRightSeen() || goalExtractor.blueGoalUnknownSeen()) && poleExtractor.ybyPoleSeen()) {
		if((goalExtractor.blueGoalLeftSeen() || goalExtractor.blueGoalUnknownSeen()) && (bl.distance(yby) < widthFifth)) {
			goalExtractor.reset(goalExtractor.getBlueGoalLeft());
		}
		if(goalExtractor.blueGoalRightSeen() && br.distance(yby) < widthFifth) {
			goalExtractor.reset(goalExtractor.getBlueGoalRight());
		}
	}
	if((goalExtractor.blueGoalLeftSeen() || goalExtractor.blueGoalRightSeen() || goalExtractor.blueGoalUnknownSeen()) && poleExtractor.bybPoleSeen()) {
		if((goalExtractor.blueGoalLeftSeen() || goalExtractor.blueGoalUnknownSeen()) && (bl.distance(byb) < widthFifth)) {
			goalExtractor.reset(goalExtractor.getBlueGoalLeft());
		}
		if(goalExtractor.blueGoalRightSeen() && br.distance(byb) < widthFifth) {
			goalExtractor.reset(goalExtractor.getBlueGoalRight());
		}
	}

	if((goalExtractor.yellowGoalLeftSeen() || goalExtractor.yellowGoalRightSeen() || goalExtractor.yellowGoalUnknownSeen()) && poleExtractor.ybyPoleSeen()) {
		if((goalExtractor.yellowGoalLeftSeen() || goalExtractor.yellowGoalUnknownSeen()) && (yl.distance(yby) < widthFifth)) {
			goalExtractor.reset(goalExtractor.getYellowGoalLeft());
		}
		if(goalExtractor.yellowGoalRightSeen() && yr.distance(yby) < widthFifth) {
			goalExtractor.reset(goalExtractor.getYellowGoalRight());
		}
	}
	if((goalExtractor.yellowGoalLeftSeen() || goalExtractor.yellowGoalRightSeen() || goalExtractor.yellowGoalUnknownSeen()) && poleExtractor.bybPoleSeen()) {
		if((goalExtractor.yellowGoalLeftSeen() || goalExtractor.yellowGoalUnknownSeen()) && (yl.distance(byb) < widthFifth)) {
			goalExtractor.reset(goalExtractor.getYellowGoalLeft());
		}
		if(goalExtractor.yellowGoalRightSeen() && yr.distance(byb) < widthFifth) {
			goalExtractor.reset(goalExtractor.getYellowGoalRight());
		}
	}
	*/
	// extract field line features

	tmpTime = getCurrentTime();
	EdgeVector &gridFieldLineEdges = Vision::getInstance().getGVG().getFieldLineEdges();
	fieldLineFeatureExtractor.extract(gridFieldLineEdges);
	fieldLinesTime = getCurrentTime() - tmpTime;
	DEBUG_TABLE("vision.runtimes", "Field line extraction time", fieldLinesTime);


	// critical section to copy all extracted objects at once

	criticalSection.enter();

	if(useLUTFreeBall == 1) {
		ball = *ballExtractorLutFree.getBall();
	} else {
		ball = *ballExtractor.getBall();
	}
	blueGoalLeft = *goalExtractor.getBlueGoalLeft();
	blueGoalRight = *goalExtractor.getBlueGoalRight();
	yellowGoalLeft = *goalExtractor.getYellowGoalLeft();
	yellowGoalRight = *goalExtractor.getYellowGoalRight() ;
	bybPole = *poleExtractor.getBYBPole();
	ybyPole = *poleExtractor.getYBYPole();
	fieldLineFeatures = fieldLineFeatureExtractor.getLineFeatures();
	fieldLineCluster = *fieldLineFeatureExtractor.getFieldLineCluster();

	// get magenta/cyan boxes from LUT method
	magentaObstacleBoxes = obstacleExtractor.getMagentaTeamBoxes();
	cyanObstacleBoxes = obstacleExtractor.getCyanTeamBoxes();

	if(useLutFreeObstacles == 1) {
		// use different method for black boxes
		blackObstacleBoxes = obstacleExtractorLutFree.getBlackObstacleBoxes();
	} else {
		blackObstacleBoxes = obstacleExtractor.getBlackObstacleBoxes();
	}

	roll = TheCameraModel::getInstance().roll();
	pitch = TheCameraModel::getInstance().pitch();
	headAngle = TheCameraModel::getInstance().headAngle();

	criticalSection.leave();

#if VISION_DEBUG
	comm.sendMessage(protobufMessage);
#endif
}

/**
 * Saves the relevant object image positions and roll/ pitch/ head angle values for localization
 */
void ObjectExtractor::saveDataForLocalization() {
	using namespace de::fumanoids::message;

	LocalizationInput_VisionData* visionData = Vision::getInstance().dataForLocalization.add_visiondata();

	visionData->set_framenumber(frameCounter);
	visionData->set_frametime(getCurrentTime());
	visionData->set_pitch(pitch);
	visionData->set_roll(roll);
	visionData->set_headangle(headAngle);

	visionData->set_forward(localization.getMotionModel().forward());
	visionData->set_sideward(localization.getMotionModel().sideward());
	visionData->set_rotation(localization.getMotionModel().rotation());

	ObjectExtractorStatus *objectExtractorStatus = visionData->mutable_objectextractorstatus();

	// add goals
	if(blueGoalLeft.basePoint.x != -1) {
		ObjectPosition *bgl = objectExtractorStatus->add_objectpositions();
		de::fumanoids::message::Position *pos = bgl->mutable_position();
		pos->set_positiontype(Position_PositionType_IMAGE);
		pos->set_x(blueGoalLeft.basePoint.x);
		pos->set_y(blueGoalLeft.basePoint.y);

		if(blueGoalLeft.type == GOAL_POLE_LEFT_OBJECT) {
			bgl->set_type(ObjectPosition_ObjectType_GoalBlueL);
		}
		else {
			bgl->set_type(ObjectPosition_ObjectType_GoalBlueU);
		}
	}
	if(blueGoalRight.basePoint.x != -1) {
		ObjectPosition *bgr = objectExtractorStatus->add_objectpositions();
		de::fumanoids::message::Position *pos = bgr->mutable_position();
		pos->set_positiontype(Position_PositionType_IMAGE);
		pos->set_x(blueGoalRight.basePoint.x);
		pos->set_y(blueGoalRight.basePoint.y);

		bgr->set_type(ObjectPosition_ObjectType_GoalBlueR);
	}
	if(yellowGoalLeft.basePoint.x != -1) {
		ObjectPosition *ygl = objectExtractorStatus->add_objectpositions();
		de::fumanoids::message::Position *pos = ygl->mutable_position();
		pos->set_positiontype(Position_PositionType_IMAGE);
		pos->set_x(yellowGoalLeft.basePoint.x);
		pos->set_y(yellowGoalLeft.basePoint.y);

		if(yellowGoalLeft.type == GOAL_POLE_LEFT_OBJECT) {
			ygl->set_type(ObjectPosition_ObjectType_GoalYellowL);
		}
		else {
			ygl->set_type(ObjectPosition_ObjectType_GoalYellowU);
		}
	}
	if(yellowGoalRight.basePoint.x != -1) {
		ObjectPosition *ygr = objectExtractorStatus->add_objectpositions();
		de::fumanoids::message::Position *pos = ygr->mutable_position();
		pos->set_positiontype(Position_PositionType_IMAGE);
		pos->set_x(yellowGoalRight.basePoint.x);
		pos->set_y(yellowGoalRight.basePoint.y);

		ygr->set_type(ObjectPosition_ObjectType_GoalYellowR);
	}

	// add side poles
	if(bybPole.basePoint.x != -1) {
		ObjectPosition *byb = objectExtractorStatus->add_objectpositions();
		de::fumanoids::message::Position *pos = byb->mutable_position();
		pos->set_positiontype(Position_PositionType_IMAGE);
		pos->set_x(bybPole.basePoint.x);
		pos->set_y(bybPole.basePoint.y);

		byb->set_type(ObjectPosition_ObjectType_BYBPole);
	}
	if(ybyPole.basePoint.x != -1) {
		ObjectPosition *yby = objectExtractorStatus->add_objectpositions();
		de::fumanoids::message::Position *pos = yby->mutable_position();
		pos->set_positiontype(Position_PositionType_IMAGE);
		pos->set_x(ybyPole.basePoint.x);
		pos->set_y(ybyPole.basePoint.y);

		yby->set_type(ObjectPosition_ObjectType_YBYPole);
	}

	// add field line features
	for(std::vector<FieldLineFeature>::iterator iter = fieldLineFeatures.begin(); iter != fieldLineFeatures.end(); ++iter) {
		const FieldLineFeature &f = *iter;
		ObjectPosition *lineFeature = objectExtractorStatus->add_objectpositions();
		if(f.type == XCrossingFieldLine) {
			lineFeature->set_type(ObjectPosition_ObjectType_FieldLineX);
		}
		else if(f.type == TCrossingFieldLine) {
			lineFeature->set_type(ObjectPosition_ObjectType_FieldLineT);
		}
		else if(f.type == LCrossingFieldLine) {
			lineFeature->set_type(ObjectPosition_ObjectType_FieldLineL);
		}
		else {
			// shouldn't be seen ...
			continue;
		}

		de::fumanoids::message::Position *pos = lineFeature->mutable_position();
		pos->set_positiontype(Position_PositionType_IMAGE);
		pos->set_x(f.getX());
		pos->set_y(f.getY());
	}

	// add field lines
	for(std::vector<std::vector<CvPoint> >::iterator iter = fieldLineCluster.cluster.begin(); iter != fieldLineCluster.cluster.end(); ++iter) {
		const std::vector<CvPoint> &edge = *iter;
		if (edge.empty()) {
			WARNING("empty edge");
			continue;
		}

		VisionEdge *visionEdge = objectExtractorStatus->add_fieldlines();

		for(std::vector<CvPoint>::const_iterator eiter = edge.begin(); eiter != edge.end(); ++eiter) {
			VisionEdge_Edgel *edgel = visionEdge->add_edge();
			de::fumanoids::message::Position *pos = edgel->mutable_position();
			pos->set_positiontype(Position_PositionType_IMAGE);
			pos->set_x(eiter->x);
			pos->set_y(eiter->y);
		}
	}

	if( ! visionData->IsInitialized()) {
		WARNING("protobuf vision data for localization not initialized");
	}

}
