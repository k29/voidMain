#include "fishEyeLensModel.h"
#include "Utils.h"
#include "vision/vision.h"
#include "math/fixedPointMath.h"
#include "math/Math.h"
#include "commandLine.h"
#include "config/configRegistry.h"
#include "robot.h"
#include "motorbus/motorbus.h"


/*------------------------------------------------------------------------------------------------*/

class OriginCmdLineCallback : public CommandLineInterface {
public:
	virtual bool commandLineCallback(CommandLine &cmdLine) {
		std::string cmd = cmdLine.getCommand(0);
		if (cmd == "origin") {
			printf("Calibrating X and YOFFSET (center of coordinate system\n");
			printf("\n");
			printf("Please make sure the vision sees the ball and place the ball\n");
			printf("so it touches BOTH feet.\n");
			printf("\n");
			printf("Press RETURN to continue.\n\n");

			motors.enableTorque(MOTOR_HEAD_TURN, true);

			int xOffset = 0, yOffset = 0;
			float pitchIdle = 0.0f, rollIdle = 0.0f;
			while (getKey(250) == 0) {
				worldModelOutput cWM = wm.getCurrentWorldModel();
				PositionRelative p        = cWM->ballRelVision;
				robottime_t      lastSeen = cWM->ballLastSeenVision;

				int x = p.getX();
				int y = p.getY();

				pitchIdle = TheCameraModel::getInstance().pitch();
				rollIdle = TheCameraModel::getInstance().roll();

				xOffset = TheCameraModel::getInstance().getOffsetX() - x;
				yOffset = TheCameraModel::getInstance().getOffsetY() - y;

				if (lastSeen + 250 < getCurrentTime())
					printf(" Ball position: \033[31mBall not seen\033[0m                                           \r");
				else if (abs(x) <= 1 && abs(y) <= 1) {
					printf(" Ball position: \033[32m( %- 2d, %- 2d)\033[0m  (suggest XOFFSET=%d, YOFFSET=%d)       \r", x, y, xOffset, yOffset);
				} else {
					printf(" Ball position: \033[31m( %- 2d, %- 2d)\033[0m  (suggest XOFFSET=%d, YOFFSET=%d)       \r", x, y, xOffset, yOffset);
				}
				fflush(stdout);
			}

			TheCameraModel::getInstance().setPitchIdle(pitchIdle);
			TheCameraModel::getInstance().setRollIdle(rollIdle);

			TheCameraModel::getInstance().setOffsetX(xOffset);
			TheCameraModel::getInstance().setOffsetY(yOffset);

			printf("\n\nSaving ...");
			fflush(stdout);
			robot.getConfig().save();
			printf(" done\n");

			motors.enableTorque(MOTOR_HEAD_TURN, false);

			return true;
		}

		return false;
	}
};

static OriginCmdLineCallback originCmdLineCallback;
REGISTER_COMMAND("origin",  "Calibrate coordinate center",  false, true, false, &originCmdLineCallback);


/*------------------------------------------------------------------------------------------------*/

REGISTER_OPTION("camera.center.f",       320, "radius of the camera's fish eye view");
REGISTER_OPTION("camera.center.x",       320, "x-value of the camera's center");
REGISTER_OPTION("camera.center.y",       240, "y-value of the camera's center");
REGISTER_OPTION("camera.xoffset",          0, "x offset (to adjust feet/ball relation)");
REGISTER_OPTION("camera.yoffset",          0, "y offset (to adjust feet/ball relation)");
REGISTER_OPTION("camera.idle.pitch",     0.0f, "pitch value of gyro in the idle position");
REGISTER_OPTION("camera.idle.roll",      0.0f, "roll value of gyro in the idle position");


/*------------------------------------------------------------------------------------------------*/

/**
 **
 **
 */

FishEyeLensModel::FishEyeLensModel()
	: timeDiff(0)
	, mOffsetX(0)
	, mOffsetY(0)

{
	Events::getInstance().registerForEvent(EVT_CONFIGURATION_LOADED, this);
	Events::getInstance().registerForEvent(EVT_BEFORE_CONFIG_SAVE, this);

	// as we may be constructed AFTER the configuration is loaded,
	// let's trigger a configuration initialization now
	eventCallback(EVT_CONFIGURATION_LOADED, &robot.getConfig());

	// angle offset between gyro and camera
#ifdef ROBOT2011
	mCameraPitch = robot.getConfig().getIntValue("head.pitchoffset", 45);
	mCameraRoll = robot.getConfig().getIntValue("head.rolloffset", 0);
#else
	mCameraPitch = robot.getConfig().getIntValue("head.pitchoffset", 22);
	mCameraRoll = robot.getConfig().getIntValue("head.rolloffset", 0);
#endif

	updateCamToGyroTransformation();

	comm.registerOperationCallback(this, OP_GETIMAGECENTER, 0,  0);
	comm.registerOperationCallback(this, OP_SETIMAGECENTER, 8, -1);

	setHeight(ROBOT_EYE_HEIGHT);
	setAngles(0, 0, 0);
	updateTransformationMatrix();
}


/*------------------------------------------------------------------------------------------------*/

/**
 **
 **
 */

FishEyeLensModel::~FishEyeLensModel() {
	Events::getInstance().unregisterForEvent(EVT_CONFIGURATION_LOADED, this);
	Events::getInstance().unregisterForEvent(EVT_BEFORE_CONFIG_SAVE, this);
}


/*------------------------------------------------------------------------------------------------*/

/**
 **
 **
 */

void FishEyeLensModel::setHeight(uint16_t height) {
	mHeight = height;
	mUpdate = true;
}

void FishEyeLensModel::setPitch(float pitch) {
	mPitchFixed = toFixed(pitch);
	mUpdate = true;
}

void FishEyeLensModel::setRoll(float roll) {
	mRollFixed = toFixed(roll);
	mUpdate = true;
}


/*------------------------------------------------------------------------------------------------*/

/**
 **
 **
 */

void FishEyeLensModel::setHeadAngle(int16_t angleX){
	mHeadAngleFixed = toFixed(angleX);
	mUpdate = true;
}


/*------------------------------------------------------------------------------------------------*/

/**
 **
 **
 */

void FishEyeLensModel::setAngles(float pitch, float roll, int16_t headAngle) {
	mPitchFixed = toFixed(pitch);
	mRollFixed = toFixed(roll);
	mHeadAngleFixed = toFixed(headAngle);
	mUpdate = true;
}


/*------------------------------------------------------------------------------------------------*/

/**
 **
 **
 */

void FishEyeLensModel::updateTransformationMatrix() {
	using namespace FixedPointMath;
	fixedpoint const sinPitch = fsin(mPitchFixed);
	fixedpoint const cosPitch = fcos(mPitchFixed);
	fixedpoint const sinRoll = fsin(mRollFixed);
	fixedpoint const cosRoll = fcos(mRollFixed);
	Vec3 const x(cosPitch, 0, sinPitch);
	Vec3 const y(0, cosRoll, sinRoll);
	Vec3 const z = crossProduct(x, y);

	mGyroToField = Mat3x3(x[0], x[1], x[2], y[0], y[1], y[2], z[0], z[1], z[2]);
	mHeadRotation = getRotMat(Z, mHeadAngleFixed);
	mTransformation = composeMat(composeMat(mGyroToField, mHeadRotation), mCamToGyro);
	mTransformationInv = mTransformation.getTransposed();
	mUpdate = false;
}


/*------------------------------------------------------------------------------------------------*/

/**
 **
 **
 */

uint16_t FishEyeLensModel::centerX() const {
	return mCenterX;
}

uint16_t FishEyeLensModel::centerY() const {
	return mCenterY;
}

uint16_t FishEyeLensModel::focalLengthX() const {
	return mFocalLength;
}

uint16_t FishEyeLensModel::focalLengthY() const {
	return mFocalLength;
}


/*------------------------------------------------------------------------------------------------*/

/**
 **
 **
 */

float FishEyeLensModel::pitch() const {
	return toFloat(mPitchFixed);
}


/*------------------------------------------------------------------------------------------------*/

/**
 **
 **
 */

float FishEyeLensModel::roll() const {
	return toFloat(mRollFixed);
}


/*------------------------------------------------------------------------------------------------*/

/**
 **
 **
 */

int16_t FishEyeLensModel::headAngle() const {
	return toInt(mHeadAngleFixed);
}


/*------------------------------------------------------------------------------------------------*/

/**
 **
 **
 */

void FishEyeLensModel::setOffsetX(uint16_t offset_x) {
	mOffsetX = offset_x;
}

void FishEyeLensModel::setOffsetY(uint16_t offset_y) {
	mOffsetY = offset_y;
}

void FishEyeLensModel::setCenterX(uint16_t center_x) {
	mCenterX = center_x;
}

void FishEyeLensModel::setCenterY(uint16_t center_y) {
	mCenterY = center_y;
}


/*------------------------------------------------------------------------------------------------*/

/**
 **
 **
 */

void FishEyeLensModel::setCenterFocalLengthX(uint16_t focalLength_x) {
	mFocalLength = focalLength_x;
	mFocalLengthFixed = toFixed(mFocalLength);
}


/*------------------------------------------------------------------------------------------------*/

/**
 **
 **
 */

void FishEyeLensModel::setCenterFocalLengthY(uint16_t focalLength_y) {
	mFocalLength = focalLength_y;
	mFocalLengthFixed = toFixed(mFocalLength);
}

void FishEyeLensModel::setCameraPitch(int16_t camPitch) {
	mCameraPitch = camPitch;
	updateCamToGyroTransformation();
}

void FishEyeLensModel::setCameraRoll(int16_t camRoll) {
	mCameraRoll = camRoll;
	updateCamToGyroTransformation();
}

void FishEyeLensModel::updateCamToGyroTransformation() {
	// rotation camera to gyro
	fixedpoint const sinCameraPitch = fsin(toFixed(mCameraPitch));
	fixedpoint const cosCameraPitch = fcos(toFixed(mCameraPitch));
	fixedpoint const sinCameraRoll = fsin(toFixed(mCameraRoll));
	fixedpoint const cosCameraRoll = fcos(toFixed(mCameraRoll));
	Vec3 const x(cosCameraPitch, 0, sinCameraPitch);
	Vec3 const y(0, cosCameraRoll, sinCameraRoll);
	Vec3 const z = crossProduct(x, y);
	mCamToGyro = Mat3x3(x[0], x[1], x[2], y[0], y[1], y[2], z[0], z[1], z[2]);

	updateTransformationMatrix();
}

/*------------------------------------------------------------------------------------------------*/

/**
 **
 **
 */

int16_t FishEyeLensModel::horizon(uint16_t maxDistance) {
	using namespace FixedPointMath;
	if(mUpdate) {
		updateTransformationMatrix();
	}
	PositionRelative p (maxDistance, 0);
	p.rotateAsVectorByAngle(toInt(mHeadAngleFixed));
	return projectOnImage(p).getY();
}


/*------------------------------------------------------------------------------------------------*/

/**
 **
 **
 */

PositionImage FishEyeLensModel::projectOnImage(const PositionRelative& pos, int height) {
	using namespace FixedPointMath;
	if(mUpdate){
		updateTransformationMatrix();
	}

	int16_t dynOffsetX, dynOffsetY;
	calculateDynamicOffset(mPitchFixed, mRollFixed, dynOffsetX, dynOffsetY);

	// divide the values by 256 to avoid an overflow
	fixedpoint const x = toFixed(pos.getX() - mOffsetX - dynOffsetX) / 256;
	fixedpoint const y = toFixed(pos.getY() - mOffsetY - dynOffsetY) / 256;
	fixedpoint const z = toFixed(height)                / 256;
	fixedpoint const invLength = finvSqrt(fmul(x,x) + fmul(y,y) + fmul(z,z));
	Vec3 v(fmul(x, invLength), fmul(y, invLength), fmul(z, invLength));
	v = matMul(mTransformationInv, v);
	return PositionImage(toInt(fmul(-v[1], mFocalLengthFixed)) + mCenterX, toInt(fmul(-v[2], mFocalLengthFixed)) + mCenterY);
}


/*------------------------------------------------------------------------------------------------*/

/**
 * Calculates the dynamic offset in relative coordinates between the camera position and the feet
 * @param pitch
 * @param roll
 * @param dynOffsetX offset in x direction gets here
 * @param dynOffsetY offset in y direction gets here
 */
void FishEyeLensModel::calculateDynamicOffset(fixedpoint pitch, fixedpoint roll, int16_t &dynOffsetX, int16_t &dynOffsetY) {
	using namespace FixedPointMath;
	fixedpoint pitchDiffToIdle = pitch - mIdlePitchFixed;
	fixedpoint rollDiffToIdle = roll - mIdleRollFixed;

	dynOffsetX = toInt( fmul( fsin(pitchDiffToIdle), toFixed(mHeight) ) );
	dynOffsetY = toInt( fmul( fsin(rollDiffToIdle), toFixed(mHeight) ) );
}

/**
 **
 **
 */

PositionRelative FishEyeLensModel::inversePerspectiveProjection(const PositionImage& pos) {
	using namespace FixedPointMath;
	if(mUpdate) {
		updateTransformationMatrix();
	}

	fixedpoint const rx = toFixed( pos.getX() - mCenterX);
	fixedpoint const ry = toFixed( pos.getY() - mCenterY);
	Vec3 v(0, (fdiv(-rx, mFocalLengthFixed)), (fdiv(-ry, mFocalLengthFixed)));
	if (abs(v[1]) > toFixed(1) || abs(v[2]) > toFixed(1)) { // image coordinates are outside the circle
			return PositionRelative(32767, 32767);
	}
	// finvSqrt() is much more accurate for small numbers than fsqrt()
	fixedpoint const inv = finvSqrt(toFixed(1) -fmul(v[1], v[1]) - fmul(v[2], v[2]));
	if (inv < 0){ // image coordinates are outside the circle
		return PositionRelative(32767, 32767);
	}
	if (inv == toFixed(0)) {
		v[0] = 0;
	} else {
		v[0] = fdiv(toFixed(1), inv);
	}	
	v = matMul(mTransformation, v);
	fixedpoint const s = -v[2];
	if (s <= toFixed(0)) { // the ray does intersect the ground behind the robot
		return PositionRelative(32767, 32767);
	}
	fixedpoint const h = toFixed(mHeight);
	v = Vec3(fdiv(fmul(h, v[0]), s), fdiv(fmul(h, v[1]), s), fdiv(fmul(h, v[2]), s));

	int16_t dynOffsetX, dynOffsetY;
	calculateDynamicOffset(mPitchFixed, mRollFixed, dynOffsetX, dynOffsetY);

	return PositionRelative(toInt(v[0]) + mOffsetX + dynOffsetX, toInt(v[1]) + mOffsetY + dynOffsetY);
}

PositionRelative FishEyeLensModel::inversePerspectiveProjection(const PositionImage& pos, FixedPointMath::fixedpoint pitch, FixedPointMath::fixedpoint roll, FixedPointMath::fixedpoint headAngle) {
	using namespace FixedPointMath;

	// calculate tranformation matrix for given roll/ pitch/ headangle
	fixedpoint const sinPitch = fsin(pitch);
	fixedpoint const cosPitch = fcos(pitch);
	fixedpoint const sinRoll = fsin(roll);
	fixedpoint const cosRoll = fcos(roll);
	Vec3 const x(cosPitch, 0, sinPitch);
	Vec3 const y(0, cosRoll, sinRoll);
	Vec3 const z = crossProduct(x, y);

	Mat3x3 gyroToField = Mat3x3(x[0], x[1], x[2], y[0], y[1], y[2], z[0], z[1], z[2]);
	Mat3x3 headRotation = getRotMat(Z, headAngle);
	Mat3x3 transformation = composeMat(composeMat(gyroToField, headRotation), mCamToGyro);


	fixedpoint const rx = toFixed( pos.getX() - mCenterX);
	fixedpoint const ry = toFixed( pos.getY() - mCenterY);
	Vec3 v(0, (fdiv(-rx, mFocalLengthFixed)), (fdiv(-ry, mFocalLengthFixed)));
	if (abs(v[1]) > toFixed(1) || abs(v[2]) > toFixed(1)) { // image coordinates are outside the circle
			return PositionRelative(32767, 32767);
	}
	// finvSqrt() is much more accurate for small numbers than fsqrt()
	fixedpoint const inv = finvSqrt(toFixed(1) -fmul(v[1], v[1]) - fmul(v[2], v[2]));
	if (inv < 0){ // image coordinates are outside the circle
		return PositionRelative(32767, 32767);
	}
	if (inv == toFixed(0)) {
		v[0] = 0;
	} else {
		v[0] = fdiv(toFixed(1), inv);
	}
	v = matMul(transformation, v);
	fixedpoint const s = -v[2];
	if (s <= toFixed(0)) { // the ray does intersect the ground behind the robot
		return PositionRelative(32767, 32767);
	}
	fixedpoint const h = toFixed(mHeight);
	v = Vec3(fdiv(fmul(h, v[0]), s), fdiv(fmul(h, v[1]), s), fdiv(fmul(h, v[2]), s));

	int16_t dynOffsetX, dynOffsetY;
	calculateDynamicOffset(pitch, roll, dynOffsetX, dynOffsetY);

	return PositionRelative(toInt(v[0]) + mOffsetX + dynOffsetX, toInt(v[1]) + mOffsetY + dynOffsetY);
}

/*------------------------------------------------------------------------------------------------*/

/**
 * Undistort the image point relative to the calibrated center point
 * @param positionImage
 * @return
 */
PositionImage FishEyeLensModel::undistortImagePoint(const PositionImage &positionImage) const {

	int distX = positionImage.getX() - mCenterX;
	int distY = positionImage.getY() - mCenterY;

	double r = sqrt(distX * distX + distY * distY);

	fixedpoint alpha = fatan2(toFixed(distY), toFixed(distX));
	fixedpoint theta = fasin( fdiv( toFixed(r), mFocalLengthFixed) );

	fixedpoint rUndistorted = fmul( mFocalLengthFixed , ftan(theta) );

	int xUndistorted = mCenterX + toInt( fmul(rUndistorted, fcos(alpha)) );
	int yUndistorted = mCenterY + toInt( fmul(rUndistorted, fsin(alpha)) );

	return PositionImage(xUndistorted, yUndistorted);
}

/*------------------------------------------------------------------------------------------------*/

/** Handle vision related operations
 **
 ** @param operation       Operation
 ** @param flags           Operation flags
 ** @param data            Received payload
 ** @param dataLen         Number of bytes received in payload
 ** @param remoteAddress   Information about who sent this operation
 **
 ** @return true iff operation was handled successfully
 */

bool FishEyeLensModel::operationCallback(
	OPERATION operation,
	uint8_t   flags,
	uint8_t  *data,
	uint16_t  dataLen,
	struct sockaddr_in *remoteAddress) {
	if (operation == OP_GETIMAGECENTER) {
		uint16_t settings[4];
		settings[0] = htons(mCenterX);
		settings[1] = htons(mCenterY);
		settings[2] = htons(mFocalLength);
		settings[3] = htons(mOffsetY);
		comm.sendMessage(OP_GETIMAGECENTER, FLAG_IS_ANSWER, (uint8_t*)settings, sizeof settings, remoteAddress);

	} else if (operation == OP_SETIMAGECENTER) {
		uint16_t *settings = (uint16_t*)data;
		mCenterX           = ntohs(settings[0]);
		mCenterY           = ntohs(settings[1]);
		mFocalLength       = ntohs(settings[2]);
		mFocalLengthFixed  = toFixed(mFocalLength);
		printf("image center set to: (%d, %d), radius %d\n", mCenterX, mCenterY, mFocalLength);
	} else
		return false;

	return true;
}


/*------------------------------------------------------------------------------------------------*/

/**
 **
 **
 */

void FishEyeLensModel::eventCallback(EventType evtType, void*) {
	Config &config = robot.getConfig();

	// if configuration is not yet loaded, abort
	if (&config == 0)
		return;

	if (evtType == EVT_CONFIGURATION_LOADED) {
		// set default values
		mCenterX     = config.getIntValue("camera.center.x", Vision::getInstance().getImageWidth()  / 2);
		mCenterY     = config.getIntValue("camera.center.y", Vision::getInstance().getImageHeight() / 2);
		mFocalLength = config.getIntValue("camera.center.f", Vision::getInstance().getImageWidth()  / 2);
		mOffsetX     = config.getIntValue("camera.xOffset");
		mOffsetY     = config.getIntValue("camera.yOffset");
		mIdlePitchFixed = toFixed( config.getFloatValue("camera.idle.pitch") );
		mIdleRollFixed = toFixed( config.getFloatValue("camera.idle.roll") );

		#ifdef ROBOT2010
		// the new robots have an aperture of 150°
		// sin(75°) == 0.966 == 1/1.035
		mFocalLengthFixed = toFixed(1.035f*mFocalLength);
		#else
		mFocalLengthFixed = toFixed(mFocalLength);
		#endif

	} else if (evtType == EVT_BEFORE_CONFIG_SAVE) {
		config.setValue("camera.center.x",       mCenterX);
		config.setValue("camera.center.y",       mCenterY);
		config.setValue("camera.center.f",       mFocalLength);
		config.setValue("camera.xOffset",        mOffsetX);
		config.setValue("camera.yOffset",        mOffsetY);
		config.setValue("camera.idle.pitch",     toFloat(mIdlePitchFixed));
		config.setValue("camera.idle.roll",      toFloat(mIdleRollFixed));
	}
}
