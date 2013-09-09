#ifndef OBJECTEXTRACTOR_H_
#define OBJECTEXTRACTOR_H_

#include <opencv/cv.h>
#include <vector>
#include <list>
#include "color.h"
#include "boundingBox.h"
#include "object.h"
#include "edge.h"
#include "image.h"

#include "objectExtractor_ball.h"
#include "objectExtractor_goal.h"
#include "objectExtractor_pole.h"
#include "objectExtractor_obstacle.h"
#include "objectExtractor_fieldlinefeature.h"
#include "objectExtractor_ball_lutfree.h"
#include "objectExtractor_obstacle_lutfree.h"

#include "comm/protobuf/msg_vision.pb.h"

/**
 * This class acts as interface between the Vision main class and all the specific object extractors.
 *
 * All the object extractors get called, using the edge information provided by the GradientVectorGriding algorithm and
 * the field countour provided by the FieldExtractor.
 *
 * @ingroup vision
 */
class ObjectExtractor {
public:
	ObjectExtractor();
	virtual ~ObjectExtractor();

	inline void setImage(IMAGETYPE *img) {
		image = img;
		ballExtractor.setImage(img);
		ballExtractorLutFree.setImage(img);
		goalExtractor.setImage(img);
		poleExtractor.setImage(img);
		obstacleExtractor.setImage(img);
		obstacleExtractorLutFree.setImage(img);
		fieldLineFeatureExtractor.setImage(img);
	}

	inline void setColorManager(ColorManager *cm) {
		ballExtractor.setColorManager(cm);
		obstacleExtractor.setColorManager(cm);
		goalExtractor.setColorManager(cm);
		poleExtractor.setColorManager(cm);
		fieldLineFeatureExtractor.setColorManager(cm);
	}

	inline void setFeetSpace(const BoundingBox &fs) {
		ballExtractor.setFeetSpace(fs);
		obstacleExtractor.setFeetSpace(fs);
		obstacleExtractorLutFree.setFeetSpace(fs);
	}

	void clear();

	void extractObjects(const EdgeVector &edges);

	// get objects (threadsafe)

	inline RectangleObject& getBall() {
		CriticalSectionLock csl(criticalSection);
		return ball;
	}
	inline RectangleObject& getBlueGoalLeft() {
		CriticalSectionLock csl(criticalSection);
		return blueGoalLeft;
	}
	inline RectangleObject& getBlueGoalRight() {
		CriticalSectionLock csl(criticalSection);
		return blueGoalRight;
	}
	inline RectangleObject& getYellowGoalLeft() {
		CriticalSectionLock csl(criticalSection);
		return yellowGoalLeft;
	}
	inline RectangleObject& getYellowGoalRight() {
		CriticalSectionLock csl(criticalSection);
		return yellowGoalRight;
	}
	inline RectangleObject& getYBYPole() {
		CriticalSectionLock csl(criticalSection);
		return ybyPole;
	}
	inline RectangleObject& getBYBPole() {
		CriticalSectionLock csl(criticalSection);
		return bybPole;
	}
	inline std::vector<FieldLineFeature>& getFieldLineFeatures() {
		CriticalSectionLock csl(criticalSection);
		return fieldLineFeatures;
	}
	inline bool hasFieldLineCluster() {
		CriticalSectionLock csl(criticalSection);
		return fieldLineCluster.cluster.size() > 0;
	}
	inline ClusterObject& getFieldLineCluster() {
		CriticalSectionLock csl(criticalSection);
		return fieldLineCluster;
	}
	inline std::list<BoundingBox>& getCyanTeamBoxes() {
		CriticalSectionLock csl(criticalSection);
		return cyanObstacleBoxes;
	}
	inline std::list<BoundingBox>& getMagentaTeamBoxes() {
		CriticalSectionLock csl(criticalSection);
		return magentaObstacleBoxes;
	}
	inline std::list<BoundingBox>& getBlackObstacleBoxes() {
		CriticalSectionLock csl(criticalSection);
		return blackObstacleBoxes;
	}
	inline float getRoll() {
		CriticalSectionLock csl(criticalSection);
		return roll;
	}
	inline float getPitch() {
		CriticalSectionLock csl(criticalSection);
		return pitch;
	}
	inline int16_t getHeadAngle() {
		CriticalSectionLock csl(criticalSection);
		return headAngle;
	}

	// get extractors

	inline FieldLineFeatureExtractor& getFieldLineFeatureExtractor() {
		return fieldLineFeatureExtractor;
	}

	inline ObstacleExtractor& getObstacleExtractor() {
		return obstacleExtractor;
	}

	inline ObstacleExtractorLutFree& getObstacleExtractorLutFree() {
		return obstacleExtractorLutFree;
	}

	inline BallExtractorLutFree& getBallExtractorLutFree() {
		return ballExtractorLutFree;
	}

	inline GoalExtractor& getGoalExtractor() {
		return goalExtractor;
	}

	inline PoleExtractor& getPoleExtractor() {
		return poleExtractor;
	}

	inline de::fumanoids::message::ObjectExtractorStatus* getProtobufExtractorStatus() {
		return protobufExtractorStatus;
	}

	void saveDataForLocalization();

protected:
	friend class ImagePresenter;
	friend class Vision;
	IMAGETYPE *image;

	// extractors for the different objects

	BallExtractor ballExtractor;
	GoalExtractor goalExtractor;
	PoleExtractor poleExtractor;
	ObstacleExtractor obstacleExtractor;
	FieldLineFeatureExtractor fieldLineFeatureExtractor;

	// lut-free detection methods
	BallExtractorLutFree ballExtractorLutFree;
	ObstacleExtractorLutFree obstacleExtractorLutFree;

	// objects

	RectangleObject ball;
	RectangleObject blueGoalLeft;
	RectangleObject blueGoalRight;
	RectangleObject yellowGoalLeft;
	RectangleObject yellowGoalRight;
	RectangleObject bybPole;
	RectangleObject ybyPole;
	std::vector<FieldLineFeature> fieldLineFeatures;
	ClusterObject fieldLineCluster; //!< all field lines are clustered together into one object
	std::list<BoundingBox> cyanObstacleBoxes;
	std::list<BoundingBox> magentaObstacleBoxes;
	std::list<BoundingBox> blackObstacleBoxes;

	// roll, pitch and head angle of the current image -> needed for synchronization with localization
	float roll, pitch;
	int16_t headAngle;

	int frameCounter;

	CriticalSection criticalSection;
	de::fumanoids::message::ObjectExtractorStatus *protobufExtractorStatus;
};

#endif /* OBJECTEXTRACTOR_H_ */
