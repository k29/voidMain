#ifndef CAMERA_OFFLINE_H_
#define CAMERA_OFFLINE_H_

#include "camera.h"
#include "robot.h"

#include <string>

class CameraOffline : public Camera {
public:
	CameraOffline();
	virtual ~CameraOffline() {};

	virtual std::string getCameraName() {
		return "Offline/Image";
	}

	virtual bool openCamera(const char* deviceName, uint16_t requestedImageWidth, uint16_t requestedImageHeight);
	virtual void closeCamera() {}
	virtual bool isOpen() { return image != 0; }

	/// capture an image
	virtual bool capture();

	/// set a camera setting
	virtual void setSetting(CAMERA_SETTING setting, int32_t value) {}

	inline std::string getCurrentImageName() {
		return cameraImageFileName;
	}

	static void incrImageIdx() { ++imageIdx; }

	static void decrImageIdx() {--imageIdx; }

protected:
	IplImage *imageCV;

	/// name of currently loaded image file
	std::string cameraImageFileName;

	bool readImageFromPBI(std::string cameraImageFile);
	bool readImage(std::string cameraImageFile);

	static int imageIdx; // index of current image if a list of images is used

private:

};

#endif /* CAMERA_SIMULATOR_H_ */
