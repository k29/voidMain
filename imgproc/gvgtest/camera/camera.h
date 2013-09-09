/** @file
 **
 ** Abstract base class for camera handling. The vision will
 ** only work on objects of this type, unaware of the actual
 ** camera hardware.
 */


#ifndef CAMERA_H_
#define CAMERA_H_

#include <inttypes.h>

#include <fstream>
#include <string>
#include <vector>
#include <map>

#include "camera_image.h"
#include "comm.h"

#include "comm/protobuf/msg_calibration.pb.h"

/*------------------------------------------------------------------------------------------------*/

typedef enum {
	  CAMERA_GAIN
	, CAMERA_R_GAIN
	, CAMERA_G_GAIN
	, CAMERA_G2_GAIN
	, CAMERA_B_GAIN
	, CAMERA_AUTO_WHITE_BALANCE
	, CAMERA_WHITE_BALANCE_TEMP
	, CAMERA_HFLIP_IMAGE
	, CAMERA_VFLIP_IMAGE
	, CAMERA_AUTO_EXPOSURE
	, CAMERA_AUTO_EXPOSURE_PRIO
	, CAMERA_EXPOSURE
	, CAMERA_HUE
	, CAMERA_SATURATION
	, CAMERA_CONTRAST
	, CAMERA_BACKLIGHT_COMP
	, CAMERA_BRIGHTNESS
	, CAMERA_SHARPNESS
	, CAMERA_GAMMA
	, CAMERA_REGISTER
	, CAMERA_R_GAIN_A_MUL
	, CAMERA_G1_GAIN_A_MUL
	, CAMERA_G2_GAIN_A_MUL
	, CAMERA_B_GAIN_A_MUL
	, CAMERA_R_GAIN_D
	, CAMERA_G1_GAIN_D
	, CAMERA_G2_GAIN_D
	, CAMERA_B_GAIN_D
	, CAMERA_GREEN1_OFFSET
	, CAMERA_GREEN2_OFFSET
	, CAMERA_RED_OFFSET
	, CAMERA_BLUE_OFFSET
	, CAMERA_BLC_MANUAL
	, CAMERA_ROTATE_IMAGE
	, CAMERA_START_X
	, CAMERA_START_Y
	, CAMERA_WIDTH
	, CAMERA_HEIGHT
	, CAMERA_POWER_FREQUENCY
} CAMERA_SETTING;


/*------------------------------------------------------------------------------------------------*/

typedef struct {
	CAMERA_SETTING setting;
	uint32_t       id;
	const char*    name;
	int32_t        minValue;
	int32_t        maxValue;
	int32_t        defaultValue;
	int32_t        currentValue;
} CameraSettingsData;


/*------------------------------------------------------------------------------------------------*/

class Camera : public OperationCallback {
public:
	Camera();

	virtual ~Camera();

	virtual std::string getCameraName() = 0;

	virtual bool openCamera(const char* deviceName, uint16_t requestedImageWidth, uint16_t requestedImageHeight) = 0;
	virtual void closeCamera() = 0;
	virtual bool isOpen() = 0;

	/// return the width of the images that this camera will capture
	virtual uint16_t getImageWidth()  { return imageWidth;  }

	/// return the height of the images that this camera will capture
	virtual uint16_t getImageHeight() { return imageHeight; }

	/// capture an image
	virtual bool capture() = 0;

	/// add a supported setting to the list
	virtual void addSupportedSetting(
			CAMERA_SETTING setting,
			unsigned long int id,
			const char* description,
			int32_t defaultValue,
			int32_t minValue,
			int32_t maxValue);

	/// returns true iff the camera supports this setting
	virtual bool supports(CAMERA_SETTING setting);

	/// set a camera setting
	virtual void setSetting(CAMERA_SETTING setting, int32_t value) = 0;

	/// get a camera setting
	virtual int32_t getSetting(CAMERA_SETTING setting);

	/// get the valid range for a setting
	virtual bool getValueRange(CAMERA_SETTING setting, int32_t &low, int32_t &high);

	/// get the list of available camera settings
	virtual uint8_t* getListOfSettings(uint32_t *size);

	/** Get current camera image data.
	 **
	 ** It must be taken big care that the returned object is not used past
	 ** its lifetime (data will become invalid as soon as the next image is captured).
	 */
	CameraImage* getImage() {
		return image;
	}

	virtual bool configure(const de::fumanoids::message::CameraSettings &parameters);

	virtual bool operationCallback(
				OPERATION operation,
				uint8_t   flags,
				uint8_t  *data,
				uint16_t  dataLen,
				struct sockaddr_in *remoteAddress);

	virtual void autoWhiteBalance(int16_t startX, int16_t endX, int16_t startY, int16_t endY);

protected:
	CameraImage *image;

	uint16_t imageWidth;  /// width of the images that this camera will capture
	uint16_t imageHeight; /// height of the images that this camera will capture

	friend class Vision;

	std::vector<CameraSettingsData> supportedSettings;

	int16_t getSettingIndex(CAMERA_SETTING setting);

	virtual void defaultConfiguration();

	uint32_t totalFrames;
};

#endif /* CAMERA_H_ */
