/** @file
 **
 ** this class encapsulates how image coordinates
 ** are transformed into relative field coordinates
 */

#ifndef FISHEYELENSMODEL_H_
#define FISHEYELENSMODEL_H_

#include "cameraModel.h"
#include "position.h"
#include "math/fixedPointMath.h"
#include "comm.h"
#include "thread.h"
#include "events.h"

#include <robot.h>
#include <vector>

using namespace FixedPointMath;

class FishEyeLensModel : public CameraModel, public OperationCallback, public EventCallback {
	friend class TheCameraModel;
	friend class TestCameraModel;
protected:
	FishEyeLensModel();
public:
	virtual ~FishEyeLensModel();

	void setHeight(uint16_t height);
	void setPitch(float degrees);
	void setRoll(float degrees);
	void setHeadAngle(int16_t degrees);
	void setAngles(float pitch, float roll, int16_t headAngle);
	void updateTransformationMatrix();

	uint16_t centerX() const;
	uint16_t centerY() const;
	uint16_t focalLengthX() const; 	/// focalLengthX == focalLengthY
	uint16_t focalLengthY() const;

	virtual float pitch() const;
	virtual float roll() const;
	virtual int16_t headAngle() const;

	virtual void setCenterX(uint16_t center_x);
	virtual void setCenterY(uint16_t center_y);
	virtual void setCenterFocalLengthX(uint16_t focalLength_x); /// focalLengthX == focalLengthY
	virtual void setCenterFocalLengthY(uint16_t focalLength_y);
	virtual void setOffsetX(uint16_t offset_x);
	virtual void setOffsetY(uint16_t offset_y);
	virtual void setCameraPitch(int16_t camPitch);
	virtual void setCameraRoll(int16_t camRoll);

	virtual int16_t getOffsetX() { return mOffsetX; }
	virtual int16_t getOffsetY() { return mOffsetY; }

	virtual void setPitchIdle(float pitchIdle) { mIdlePitchFixed = toFixed(pitchIdle); }
	virtual void setRollIdle(float rollIdle) { mIdleRollFixed = toFixed(rollIdle); }
	virtual float getPitchIdle() { return toFloat(mIdlePitchFixed); }
	virtual float getRollIdle() { return toFloat(mIdleRollFixed); }

	int16_t horizon(uint16_t maxDistance);
	PositionImage projectOnImage(const PositionRelative& pos, int height = -ROBOT_EYE_HEIGHT);
	PositionRelative inversePerspectiveProjection(const PositionImage& pos);
	PositionRelative inversePerspectiveProjection(const PositionImage& pos, FixedPointMath::fixedpoint pitch, FixedPointMath::fixedpoint roll, FixedPointMath::fixedpoint headAngle);

	PositionImage undistortImagePoint(const PositionImage &positionImage) const;

	bool operationCallback(
		OPERATION operation,
		uint8_t   flags,
		uint8_t  *data,
		uint16_t  dataLen,
		struct sockaddr_in *remoteAddress);

	void eventCallback(EventType evtType, void*);

	void setTimeDiff(robottime_t timeDiff) {
		this->timeDiff = timeDiff;
	}
	robottime_t getTimeDiff() {
		return timeDiff;
	}

private:
	robottime_t timeDiff;  //!< timediff between setting the gyro roll/pitch values and reading it

	/** intrinsic parameters **/
	uint16_t mFocalLength;         //!< approx the radius of the circle
	fixedpoint mFocalLengthFixed;  //!< fixed variant of radius of the circle
	uint16_t mCenterX;             //!< optical image Center x
	uint16_t mCenterY;             //!< optical image Center x

	/** extrinsic parameters **/
	uint16_t mHeight;           //!< height of camera in cm
	fixedpoint mPitchFixed;     //!< pitch value of gyro in degrees
	fixedpoint mRollFixed;      //!< pitch value of gyro in degrees
	fixedpoint mHeadAngleFixed; //!< in degrees
	int16_t mCameraPitch;       //!< pitch offset from camera to gyro
	int16_t mCameraRoll;        //!< roll offset from camera to gyro
	Mat3x3 mTransformation;
	Mat3x3 mTransformationInv;
	Mat3x3 mCamToGyro;          //!< transformation matrix from camera to gyro
	Mat3x3 mGyroToField;        //!< transformation matrix from gyro to field
	Mat3x3 mHeadRotation;
	bool mUpdate;

	/// static x and y offset for field coordinates
	int16_t mOffsetX;
	int16_t mOffsetY;

	fixedpoint mIdlePitchFixed;
	fixedpoint mIdleRollFixed;

	void init();
	void updateCamToGyroTransformation();
	void calculateDynamicOffset(fixedpoint pitch, fixedpoint roll, int16_t &dynOffsetX, int16_t &dynOffsetY);
};

#endif /* FISHEYELENSMODEL_H_ */
