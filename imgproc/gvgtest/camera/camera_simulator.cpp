/*
 * camera_simulator.cpp
 *
 *  Created on: 25.10.2009
 */

#include "camera_simulator.h"
#include "simulation/sim_cameraDevice.h"

// TODO: what happens if the vision expects Bayer data? -> FUmanoid should be compiled with the Desktop-Debug-YUV422 configuration (for now at least)
#include "camera_imageYUV422.h"
#include "vision/image.h"

#include "../robot.h"

// TODO remove
#include <opencv/highgui.h>


CameraSimulator::CameraSimulator() {
	// New Device Thread
	scd = &SimCameraDevice::getInstance();
	frameNumber = 0;
}

bool CameraSimulator::openCamera(const char* deviceName, uint16_t requestedImageWidth,
		uint16_t requestedImageHeight) {

	if(scd->startCamera(requestedImageWidth, requestedImageHeight) > 0) {

		if(initDevice(requestedImageWidth, requestedImageHeight)) {
			return true;
		} else {
			ERROR("initDevice failed: can't set image size");
			return false;
		}
	}

	ERROR("startCamera() failed");
	return false;
}

/**
 * Capturing a new image == get image from Sim_CameraDevice through readFrame()
 *
 * @return true when new frame was read
 */
bool CameraSimulator::capture() {
	if (readFrame()) {
		//INFO("capture(): Got Frame from Camera Device");
		totalFrames++;
		return true;
	}

	return false;
}

/**
 * read last Frame from simulated CameraDevice
 *
 */

bool CameraSimulator::readFrame() {
	if (scd->frameCapturedEvent.wait(50)) {
		image->setImage(scd->getBuffer(), scd->getImageSize());
		frameNumber = scd->getFrameCounter();

//		INFO("frame-no.: %d", scd->getFrameCounter());

		return true;
	}

	//INFO("readFrame() >> no new Frame available");
	return false;
}

/**
 * TODO config should set ImageSize parameters
 */
bool CameraSimulator::initDevice(uint16_t requestedImageWidth, uint16_t requestedImageHeight) {

	if(scd->running()) {
		// TODO tell Simulator what kind of pictures are needed!
		imageWidth = requestedImageWidth;
		imageHeight = requestedImageHeight;
	} else {
		return false;
	}

	// Create CameraImage Object
	image = createImage();
	return true;
}

/**
 *
 * @return
 */
CameraImage* CameraSimulator::createImage() {
	return new IMAGETYPE(imageWidth, imageHeight);
}

/**
 *
 */
void CameraSimulator::uninitDevice() {

}
