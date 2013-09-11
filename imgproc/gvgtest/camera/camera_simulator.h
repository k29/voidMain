/*
 * camera_simulator.h
 *
 *  Created on: 25.10.2009
 */

#ifndef CAMERA_SIMULATOR_H_
#define CAMERA_SIMULATOR_H_

#include "camera.h"
#include "simulation/sim_cameraDevice.h"
#include "../robot.h"

#include <string>

class CameraSimulator : public Camera {
public:
	CameraSimulator();
	virtual ~CameraSimulator() {};

	virtual std::string getCameraName() {
		return "Simulator";
	}

	virtual bool openCamera(const char* deviceName, uint16_t requestedImageWidth, uint16_t requestedImageHeight);
	virtual void closeCamera() {}; // TODO close device
	virtual bool isOpen() { return scd->running(); };

	/// capture an image
	virtual bool capture();
	virtual CameraImage* createImage();

	/// set a camera setting
	virtual void setSetting(CAMERA_SETTING setting, int32_t value) {};

protected:
	virtual bool initDevice(uint16_t requestedImageWidth, uint16_t requestedImageHeight);
	virtual void uninitDevice();
	virtual bool readFrame();

private:

	SimCameraDevice *scd;
	struct Buffer* buffers;
	unsigned int n_buffers;
	int frameNumber;
};

#endif /* CAMERA_SIMULATOR_H_ */
