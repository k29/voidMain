/** @file
 **
 ** Interface for camera models
 */

#ifndef CAMERAMODEL_H_
#define CAMERAMODEL_H_

#include "position.h"
#include "robot.h"
#include <vector>
#include <map>
#include <string>
#include <iostream>
#include <fstream>

class CameraModel
{
protected:
	CameraModel() {};
	virtual ~CameraModel() {};
public:
	virtual void setHeight(uint16_t height) = 0;
	virtual void setPitch(float degrees) = 0;
	virtual void setRoll(float degrees) = 0;
	virtual void setHeadAngle(int16_t degrees) = 0;
	virtual void setAngles(float pitch, float roll, int16_t headAngle) = 0;

	virtual void setTimeDiff(robottime_t timeDiff) = 0;
	virtual robottime_t getTimeDiff() = 0;

	virtual float pitch() const = 0;
	virtual float roll() const = 0;
	virtual int16_t headAngle() const = 0;
	virtual uint16_t centerX() const = 0;
	virtual uint16_t centerY() const = 0;
	virtual uint16_t focalLengthX() const = 0;
	virtual uint16_t focalLengthY() const = 0;

	virtual void setCameraPitch(int16_t camPitch) = 0;
	virtual void setCameraRoll(int16_t camRoll) = 0;

	virtual void setCenterX(uint16_t center_x) = 0;
	virtual void setCenterY(uint16_t center_y) = 0;
	virtual void setCenterFocalLengthX(uint16_t focalLength_x) = 0;
	virtual void setCenterFocalLengthY(uint16_t focalLength_y) = 0;

	virtual void setOffsetX(uint16_t offset_x) = 0;
	virtual void setOffsetY(uint16_t offset_y) = 0;
	virtual int16_t getOffsetX() = 0;
	virtual int16_t getOffsetY() = 0;

	virtual void setPitchIdle(float pitchIdle) = 0;
	virtual void setRollIdle(float rollIdle) = 0;
	virtual float getPitchIdle() = 0;
	virtual float getRollIdle() = 0;

	virtual int16_t horizon(uint16_t maxDistance) = 0;
	virtual PositionImage projectOnImage(const PositionRelative& pos, int height = -ROBOT_EYE_HEIGHT) = 0;
	virtual PositionRelative inversePerspectiveProjection(const PositionImage& pos) = 0;
	virtual PositionRelative inversePerspectiveProjection(const PositionImage& pos, FixedPointMath::fixedpoint pitch, FixedPointMath::fixedpoint roll, FixedPointMath::fixedpoint headAngle) = 0;

	virtual PositionImage undistortImagePoint(const PositionImage &positionImage) const = 0;
};

#endif /* CAMERAMODEL_H_ */

