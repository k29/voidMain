/** @file
 **
 **
 */


#include "camera.h"
#include "robot.h"
#include "vision/image.h"

#include "math/utils.h"

#include <functional>
#include <algorithm>
#include <string>
#include <map>


/*------------------------------------------------------------------------------------------------*/

/**
 */

Camera::Camera()
	: image(0)
	, totalFrames(0)
{
	comm.registerOperationCallback(this, OP_GETLISTOFCAMERASETTINGS, 0, 0);
	comm.registerOperationCallback(this, OP_GETCAMERASETTING,        4, 4);
	comm.registerOperationCallback(this, OP_SETCAMERASETTING,        8, 8);
	comm.registerOperationCallback(this, OP_WHITECALIBRATION,        8, 8);
}


/*------------------------------------------------------------------------------------------------*/

/**
 */

Camera::~Camera() {
	comm.unregisterOperationCallback(this, OP_GETLISTOFCAMERASETTINGS);
	comm.unregisterOperationCallback(this, OP_GETCAMERASETTING);
	comm.unregisterOperationCallback(this, OP_SETCAMERASETTING);

	if (image)
		delete image;
	image = 0;
}


/*------------------------------------------------------------------------------------------------*/

/**
 */

void Camera::addSupportedSetting(CAMERA_SETTING setting, unsigned long int id, const char* description, int32_t defaultValue, int32_t minValue, int32_t maxValue) {
	CameraSettingsData data;
	data.setting      = setting;
	data.id           = id;
	data.name         = description;
	data.minValue     = minValue;
	data.maxValue     = maxValue;
	data.defaultValue = defaultValue;
	data.currentValue = defaultValue;

	supportedSettings.push_back(data);
}


/*------------------------------------------------------------------------------------------------*/

/** Checks whether a certain setting is supported.
 **
 ** @return true iff setting is supported
 */

bool Camera::supports(CAMERA_SETTING setting) {
	return getSettingIndex(setting) != -1;
}


/*------------------------------------------------------------------------------------------------*/

/** Determines the index of the setting in the lookup table.
 **
 ** @param setting  Setting to look for in the table
 ** @return index in the lookup table, -1 if not found
 */

int16_t Camera::getSettingIndex(CAMERA_SETTING setting) {
	for (uint16_t i=0; i < supportedSettings.size(); i++) {
		if (supportedSettings[i].setting == setting) {
			return i;
		}
	}

	return -1;
}


/*------------------------------------------------------------------------------------------------*/

/**
 */

int32_t Camera::getSetting(CAMERA_SETTING setting) {
	int16_t index = getSettingIndex(setting);
	if (index >= 0)
		return supportedSettings[index].currentValue;
	else
		return -1;
}


/*------------------------------------------------------------------------------------------------*/

/** Configure the camera.
 **
 ** @param parameters  List of named parameters
 **
 ** @return true iff configuration succeeded
 */

bool Camera::configure(const de::fumanoids::message::CameraSettings &parameters) {
	//INFO("configure camera (%d settings)", parameters.cameraparameters_size());
	for (int i=0; i < parameters.cameraparameters_size(); i++) {
		const de::fumanoids::message::CameraParameter &p = parameters.cameraparameters(i);

		//printf("checking %s\n", p.name().c_str());
		for (uint16_t j=0; j < supportedSettings.size(); j++) {
			if (p.name().compare(supportedSettings[j].name) == 0) {
				setSetting(supportedSettings[j].setting, p.value());
				printf("configured %s as %d\n", p.name().c_str(), p.value());
			}
		}
	}

	return true;
}


/*------------------------------------------------------------------------------------------------*/

/** Get list of available camera settings.
 **
 ** @return
 */

uint8_t* Camera::getListOfSettings(uint32_t *size) {
	uint32_t answerLen = 0;

	for (uint32_t i=0; i < supportedSettings.size(); i++)
		answerLen += 12 /* id, min, max */ + strlen(supportedSettings[i].name) + 1 /* trailing 0 */;

	uint8_t *answer = (uint8_t*)malloc(answerLen);
	if (answer != NULL) {
		memset(answer, 0, answerLen);

		int offset = 0;
		for (uint32_t i=0; i < supportedSettings.size(); i++) {
			uint32_t id  = htonl(supportedSettings[i].setting);
			int32_t min = htonl(supportedSettings[i].minValue);
			int32_t max = htonl(supportedSettings[i].maxValue);

			memcpy(answer + offset, &id, 4);
			offset += 4;

			memcpy(answer + offset, supportedSettings[i].name, strlen(supportedSettings[i].name) + 1);
			offset += strlen(supportedSettings[i].name) + 1;

			memcpy(answer + offset, &min, 4);
			offset += 4;

			memcpy(answer + offset, &max, 4);
			offset += 4;
		}
	}

	*size = answerLen;
	return answer;
}


/*------------------------------------------------------------------------------------------------*/

/**
 */

bool Camera::getValueRange(CAMERA_SETTING setting, int32_t &low, int32_t &high) {
	if (supports(setting) == false)
		return false;

	low  = supportedSettings[ getSettingIndex(setting) ].minValue;
	high = supportedSettings[ getSettingIndex(setting) ].maxValue;
	return true;
}


/*------------------------------------------------------------------------------------------------*/

/**
 */

void Camera::defaultConfiguration() {
	for (uint32_t i=0; i < supportedSettings.size(); i++)
		setSetting(supportedSettings[i].setting, supportedSettings[i].defaultValue);
}


/*------------------------------------------------------------------------------------------------*/

/** Handle camera related operations
 **
 ** @param operation       Operation
 ** @param flags           Operation flags
 ** @param data            Received payload
 ** @param dataLen         Number of bytes received in payload
 ** @param remoteAddress   Information about who sent this operation
 **
 ** @return true iff operation was handled successfully
 */

bool Camera::operationCallback(
	OPERATION operation,
	uint8_t   flags,
	uint8_t  *data,
	uint16_t  dataLen,
	struct sockaddr_in *remoteAddress)
{
	if (operation == OP_GETLISTOFCAMERASETTINGS) {
		uint32_t size = 0;
		uint8_t* list = getListOfSettings(&size);

		if (list != 0) {
			comm.sendMessage(OP_GETLISTOFCAMERASETTINGS, FLAG_IS_ANSWER, list, size, remoteAddress);
			free(list);
			list = 0;
		} else
			ERROR("Out of memory assembling list of camera settings.");
	} else if (operation == OP_GETCAMERASETTING) {
		int32_t value[2];

		memcpy(value, data, 4);
		CAMERA_SETTING setting = (CAMERA_SETTING)ntohl(value[0]);
		value[1] = htonl(getSetting(setting));
		comm.sendMessage(OP_GETLISTOFCAMERASETTINGS, FLAG_IS_ANSWER, (uint8_t*)value, 8, remoteAddress);
	} else if (operation == OP_SETCAMERASETTING) {
		int32_t settingID, value;
		memcpy(&settingID, data,   4);
		memcpy(&value,     data+4, 4);

		CAMERA_SETTING setting = (CAMERA_SETTING)ntohl(settingID);
		value = ntohl(value);

		setSetting(setting, value);
	} else if (operation == OP_WHITECALIBRATION) {
		uint32_t startX, endX, startY, endY;
		memcpy(&startX, data   , 2);
		memcpy(&endX,   data + 2, 2);
		memcpy(&startY, data + 4, 2);
		memcpy(&endY,   data + 6, 2);
		autoWhiteBalance(startX, endX, startY, endY);
	} else
		return false;

	return true;
}


/*------------------------------------------------------------------------------------------------*/

/** White-balance the camera
 **
 */

class GainSettings : public std::map<CAMERA_SETTING, uint32_t> {
	Camera* camera;

public:
	GainSettings(Camera* _camera)
		: camera(_camera)
	{}

	void addIfAvailable(CAMERA_SETTING setting) {
		if (camera->supports(setting)) {
			(*this)[setting] = camera->getSetting(setting);
		}
	}

	void adjust(CAMERA_SETTING setting, int32_t value, int32_t goal) {
		int32_t settingsValue = (*this)[setting];

		if (value < goal - 10)
			settingsValue += 3;
		else if (value > goal + 10)
			settingsValue -= 3;
		else {
			if (value < goal - 5)
				settingsValue += 1;
			else if (value > goal + 5)
				settingsValue -= 1;
		}

		int32_t low, high;
		camera->getValueRange(setting, low, high);

		if (camera->supports(setting)) {
			settingsValue = limited(settingsValue, low, high);
			camera->setSetting(setting, settingsValue);
			(*this)[setting] = settingsValue;
		}
	}

	void apply() {
		GainSettings::iterator it;
		for (it = begin(); it != end(); it++) {
			if (camera->supports(it->first)) {
				camera->setSetting(it->first, it->second);
			}
		}
	}
};


void Camera::autoWhiteBalance(int16_t startX, int16_t endX, int16_t startY, int16_t endY) {
	GainSettings gainSettings(this);
	gainSettings.addIfAvailable(CAMERA_G_GAIN);
	gainSettings.addIfAvailable(CAMERA_G2_GAIN);
	gainSettings.addIfAvailable(CAMERA_R_GAIN);
	gainSettings.addIfAvailable(CAMERA_B_GAIN);

	gainSettings[CAMERA_G2_GAIN] = gainSettings[CAMERA_G_GAIN];
	gainSettings.apply();

	if (gainSettings.size() <= 1)
		return;

	// white calibration
	uint32_t startFrame = totalFrames + 1;
	for (int i=0; i < 50; i++) {
		uint32_t rsum = 0, gsum = 0, bsum = 0;

		while (totalFrames != startFrame + i)
			delay(5);

		for (int32_t y = startY + 1; y < endY; y++) {
			for (int32_t x = startX + 1; x < endX; x++) {
				uint8_t r, g, b;
				((IMAGETYPE*)image)->getPixelAsRGB(x, y, &r, &g, &b, NONE);

				rsum += r;
				gsum += g;
				bsum += b;
			}
		}

		uint32_t numberOfPixels = (endY - startY) * (endX - startX);
		uint32_t rmid = rsum / numberOfPixels;
		uint32_t gmid = gsum / numberOfPixels;
		uint32_t bmid = bsum / numberOfPixels;

		if ((::abs(rmid - bmid) < 8) && (::abs(rmid - gmid) < 8) && (::abs(bmid - gmid) < 8)) {
			INFO("White-Calibration seems to be finished");
			break; // done
		}

		uint32_t meanValue = 0;
		if (rmid <= gmid) {
			if (gmid <= bmid)
				meanValue = gmid;
			else
				meanValue = (rmid <= bmid) ? bmid : rmid;
		} else { //gmid <= rmid
			if (rmid <= bmid)
				meanValue = rmid;
			else
				meanValue = (gmid <= bmid) ? bmid : gmid;
		}

		gainSettings.adjust(CAMERA_R_GAIN, rmid, meanValue);
		gainSettings.adjust(CAMERA_B_GAIN, bmid, meanValue);
	}

/*
	// brightness adjustment
	startFrame = totalFrames + 1;
	int32_t gainValue = getSetting(CAMERA_GAIN);
	for (int i=0; i < 200; i++) {
		uint32_t rsum = 0, gsum = 0, bsum = 0;

		while (totalFrames != startFrame + i)
			delay(5);

		for (int32_t y = startY + 1; y < endY; y++) {
			for (int32_t x = startX + 1; x < endX; x++) {
				uint8_t r, g, b;
				((IMAGETYPE*)image)->getPixelAsRGB(x, y, &r, &g, &b);

				rsum += r;
				gsum += g;
				bsum += b;
			}
		}

		uint32_t numberOfPixels = (endY - startY) * (endX - startX);
		uint32_t rmid = rsum / numberOfPixels;
		uint32_t gmid = gsum / numberOfPixels;
		uint32_t bmid = bsum / numberOfPixels;

		uint32_t maxvalue = max( max(rmid, gmid), bmid);

		if (maxvalue < 200 - 10) {
			gainValue += 10;
		} else if (maxvalue > 200 + 10) {
			gainValue -= 10;
		} else
			break;

		printf("set gain to %d\n", gainValue);
		setSetting(CAMERA_GAIN, gainValue);
	}
*/
}
