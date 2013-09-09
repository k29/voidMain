#include "vision.h"
#include "robot.h"
#include "comm.h"
#include "events.h"
#include "feet.h"

#include "camera/camera_gumstix.h"
#include "camera/camera_logitech_quickcam.h"
#include "camera/camera_philips_spc630nc.h"
#include "camera/camera_simulator.h"
//#include "camera/camera_VCSBC6210.h"
#include "camera/camera_icube.h"
#include "camera/camera_offline.h"

#include "imagePresenter.h"
#include "commandLine.h"

#include "comm/protobuf/msg_fumanoid.pb.h"
#include "comm/protobuf/msg_image.pb.h"
#include "transport/transport_file.h"
#include "config/configRegistry.h"
#include "debug.h"

#include <zlib.h>

#include <algorithm>
#include <string>
#include <map>
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>


/*------------------------------------------------------------------------------------------------*/

#define VISION_STATISTICS_INTERVAL  5 // seconds

#if defined VERDEX
#define DEFAULTCAMERA "Gumstix"
#elif defined OVERO
#define DEFAULTCAMERA "iCube"
#elif defined ROBOT2011
#define DEFAULTCAMERA "quickcam"
#elif defined IMAGEFORMAT_YUV422
#define DEFAULTCAMERA "quickcam"
#else
#define DEFAULTCAMERA "quickcam"
#endif

/*------------------------------------------------------------------------------------------------*/

REGISTER_OPTION("camera.device",                "",                    "Camera device or image to use");
REGISTER_OPTION("camera.type",                  DEFAULTCAMERA,         "Camera class to use");
REGISTER_OPTION("camera.updatefirmware",        1,                     "Install latest firmware to device if required");

REGISTER_OPTION("vision.lut",                   "calibration.dat",     "Color lookup table file");
REGISTER_OPTION("vision.savelocalizationdata",  0,                     "Save vision data for localization");
REGISTER_OPTION("vision.save.enabled",          0,                     "Save images");
REGISTER_OPTION("vision.save.path",             "",                    "Path (relative or absolute) for image storage");
REGISTER_OPTION("vision.save.interval",         100,                   "Interval in which to save images");
//REGISTER_OPTION("vision.save.format",           "pbi",                 "Image format (three letter, e.g. pbi, png)");

DEBUG_REGISTER("vision.runtimes", TABLE, BASIC);
DEBUG_REGISTER("vision.objects", TABLE, BASIC);

static const PositionImage unknownPosition(-1,-1);

/*------------------------------------------------------------------------------------------------*/

/**
 ** Constructor
**/
Vision::Vision()
	: cam(0)
{
	cameraType = "";
	activeFieldColorExtractor = 0;
	cs.setName("Vision");
	initialized.reset();
}


/*------------------------------------------------------------------------------------------------*/

/**
 ** Destructor
**/
Vision::~Vision() {
	comm.unregisterOperationCallback(this, OP_SAVEVISIONSETTINGS);
	comm.unregisterOperationCallback(this, OP_IMAGECAPTURE);

	if (isRunning())
		cancel(true);

	if (cam)
		delete cam;

	if (colorMgr)
		delete colorMgr;
}


/*------------------------------------------------------------------------------------------------*/

/**
 ** Initializes the camera.
 **
 ** @return true iff camera was initialized successfully
 */

bool Vision::initCamera() {
	// if camera is already open, close it now
	if (cam != 0) {
		cam->closeCamera();
		cam = 0;
	}

	cameraType                = robot.getConfig().getStrValue("camera.type",    DEFAULTCAMERA);
	std::string cameraDevice  = robot.getConfig().getStrValue("camera.device",  "/dev/video0");
	int configuredResolutionX = robot.getConfig().getIntValue("camera.width",   640);
	int configuredResolutionY = robot.getConfig().getIntValue("camera.height",  480);

	std::transform(cameraType.begin(), cameraType.end(), cameraType.begin(), ::tolower);

	// create correct camera instance
	if (cameraType == "icube") {
#ifndef VERDEX
		INFO("Loading iCube camera");
		cam = new CameraICube();
#else
		ERROR("iCube camera is not supported on Gumstix verdex");
		return false;
#endif
	} else if (cameraType == "gumstix") {
		INFO("Loading Gumstix camera");
		cam = new CameraGumstix();
	} else if (cameraType == "generic") {
		INFO("Loading generic V4L2 camera");
		cam = new CameraV4L2();
	} else if (cameraType == "quickcam") {
		INFO("Loading Logitech Quickcam camera");
		cam = new CameraLogitechQuickcam();
	} else if (cameraType == "philips") {
		INFO("Loading Philips SPC camera");
		cam = new CameraPhilipsSPC();
	} else if (cameraType == "simulator") {
		INFO("Loading Simulator (socket based) camera");
		cam = new CameraSimulator();
//	} else if (cameraType == "network") {
//		INFO("Loading network camera");
//		cam = new CameraVCSBC6210();
	} else if (cameraType == "offline") {
		INFO("Loading Offline Camera (image file based) camera");
		cam = new CameraOffline();
	} else if (cameraType == "disabled") {
		INFO("Loading no camera");
		return true;
	} else {
		ERROR("Unknown camera.");
		return false;
	}

	// open camera
	if (false == cam->openCamera(cameraDevice.c_str(), configuredResolutionX, configuredResolutionY))
		return false;

	// retrieve image - because of performance issues, we need to know the exact
	// type (see comment in image.h for more details)
	image = (IMAGETYPE*)cam->getImage();
	return true;
}


/*------------------------------------------------------------------------------------------------*/

/**
 ** Initializes the vision system and starts the vision thread.
 **
 ** @return true iff thread was started (this implies initialization succeeded)
 */

bool Vision::init() {
	// init camera
	initCamera();

	// start thread
	run();

	// we were successful
	return true;
}


/*------------------------------------------------------------------------------------------------*/

/**
 ** Thread init. Initializes remaining (more time consuming) subsystems.
 **
 */
void Vision::threadInit() {
	// let's try to take a bit more CPU time
	setNiceness(-2);

	// create color manager
	colorMgr = new ColorManager();

	// load calibration
	calibration.load( robot.getConfig().getStrValue("vision.lut", "calibration.dat").c_str() );
	colorMgr->load(calibration.calibration);

	cam->configure(calibration.calibration.camerasettings());

	// register operations we support
	comm.registerOperationCallback(this, OP_SAVEVISIONSETTINGS, 0, 0);
	comm.registerOperationCallback(this, OP_IMAGECAPTURE,       4, -1);
}


/*------------------------------------------------------------------------------------------------*/

/**
 ** Thread main function.
 **
 ** Calls the image processing routines and the object extraction.
 **
**/
void Vision::threadMain() {
	if (cam == 0)
		return;

	threadInit();

	robottime_t start = getCurrentTime();
	robottime_t lastFrameTime = 0, currentFrameTime = 0;


	Events::getInstance().trigger(EVT_VISION_STARTED, 0);
	initialized.trigger();

	double avg_fps = 0.0;
	int fpscounter = 0;
	int locCounter = 0;
	robottime_t lastLocalizationTime = 0;

	robottime_t gridTime                  = 0,
	            objectExtractionTime      = 0,
	            fieldExtractionTime       = 0,
	            fieldColorExtractionTime = 0;
	robottime_t tmpTime;

	bool firstImage = true;

	// loop until we quit
	while (isRunning()) {
		// make sure we are still connected to a camera
		if (false == cam->isOpen()) {
			WARNING("Camera not active, trying to re-initiate");
			if (initCamera() == false) {
				ERROR("No camera ... vision won't do anything.");
				delay(250);
				continue;
			}

			// re-apply settings
			cam->configure(calibration.calibration.camerasettings());
		}

		// update frame
		cs.enter();
		bool imageCaptured = cam->capture();
		cs.leave();

		if (imageCaptured) {
			// notify any other objects that want to know about this new image for processing
			Events::getInstance().trigger(EVT_IMAGE_CAPTURED, image);

			frameCounter++;
			locCounter++;
			currentFrameTime = getCurrentTime();

			// sets the image to every class
			gvg.setImage((IMAGETYPE*) image);

			objectExtractor.setImage((IMAGETYPE*) image);
			fieldExtractor.setImage((IMAGETYPE*) image);

			activeFieldColorExtractor = robot.getConfig().getIntValue("vision.fieldcolorextractor", 1);

			if(activeFieldColorExtractor == 1) {
				FieldColorExtractor::getInstance().setImage((IMAGETYPE*) image);
			}

			// at first run, init  everything
			if (firstImage) {
				gvg.init();
				gvg.setColorMgr(colorMgr);

				fieldExtractor.setColorManager(colorMgr);
				objectExtractor.setColorManager(colorMgr);

				firstImage = false;
			}

			objectExtractor.setFeetSpace(Feet::getInstance().getFeetSpace());

			// recalc every frame
			if(activeFieldColorExtractor == 1) {
				// extract the field color
				tmpTime = getCurrentTime();
				FieldColorExtractor::getInstance().extract();
				fieldColorExtractionTime = getCurrentTime() - tmpTime;
			}

			// extract the field contour
			tmpTime = getCurrentTime();
			bool fieldExtractionSuccess = fieldExtractor.extract();
			fieldExtractionTime = getCurrentTime() - tmpTime;

			if(true == fieldExtractionSuccess) {
				// now do the work! Get edges and extract the objects from them

				tmpTime = getCurrentTime();
				gvg.run();
				gridTime = getCurrentTime() - tmpTime;

				tmpTime = getCurrentTime();
				objectExtractor.extractObjects(gvg.getEdges());
				objectExtractionTime = getCurrentTime() - tmpTime;

				DEBUG_TABLE("vision.runtimes", "Field color extraction time", fieldColorExtractionTime);
				DEBUG_TABLE("vision.runtimes", "Field contour extraction time", fieldExtractionTime);
				DEBUG_TABLE("vision.runtimes", "Grid time", gridTime);
				DEBUG_TABLE("vision.runtimes", "Object extraction time overall", objectExtractionTime);
				// update world model
				pushDataIntoWorldmodel();
			}
			else {
				fieldExtractor.clear();
				objectExtractor.clear();
			}

			//for now, we must throw the localization_updated event even when its a non-localization-frame
			bool localization = false;

			// find line points
			if (locCounter >= 3 || getCurrentTime() - lastLocalizationTime > 500) {

				if( (int) TheCameraModel::getInstance().getTimeDiff() > 20) {
					//WARNING("gyro values too old: %d", (int) TheCameraModel::getInstance().getTimeDiff());
				}
				else if (objectExtractor.hasFieldLineCluster()) {

					locCounter = 0;
					lastLocalizationTime = getCurrentTime();

					if(robot.getConfig().getIntValue("vision.savelocalizationdata", 0) != 0) {
						objectExtractor.saveDataForLocalization();
					}

					// trigger localization about updated fieldlines
					localization = Events::getInstance().trigger(EVT_DATA_FOR_LOCALIZATION_UPDATED);
				}

			}

			//in case of no localization in this frame, throw the event, though
			if (!localization)
				Events::getInstance().trigger(EVT_LOCALIZATION_UPDATED);

			// save image if required
			handleImageSaving();

			// notification that image is fully processed
			Events::getInstance().trigger(EVT_IMAGE_PROCESSED, image);

			DEBUG_TABLE("vision.runtimes", "Overall frame time", currentFrameTime - lastFrameTime);

			lastFrameTime = currentFrameTime;
		} else if (lastFrameTime + 3000 < getCurrentTime()) {
			ERROR("Did not receive image data for several seconds!");
			// TODO: trigger watchdog in competition mode
		}

		// update statistics (output every 5 seconds)
		if (getCurrentTime() - start >= VISION_STATISTICS_INTERVAL*1000) {
			// framerate
			double fps = frameCounter * 1000.0 / (double)(getCurrentTime() - start);
			avg_fps += fps;
			++fpscounter;
			INFO("Vision framerate: %.1f fps", fps);
			wm.setVisionFPS(fps);
			if (fps < 10) {
				WARNING("Framerates below 10 fps cause problems in the BehaviorLayer.");
			}
			INFO("Average fps since start: %.1f fps", avg_fps / fpscounter);

			// log edge and gradient infos
#if VISION_DEBUG
			double ppf = 0.0;
			double gppf = 0.0;
			double gpaxxxpf = 0.0;

			if (frameCounter != 0) {
				ppf = (double) (CameraImage::pixelCounterGradient
						+ CameraImage::pixelCountergetPixelAsXXX)
						/ frameCounter;
				gppf = (double) CameraImage::pixelCounterGradient
						/ frameCounter;
				gpaxxxpf = (double) CameraImage::pixelCountergetPixelAsXXX
						/ frameCounter;
			}
			INFO(
					"Pixel per frame:                 %.1f\n"
					" -used for gradient calculation: %.1f\n"
					" -get pixel as XXX per frame:    %.1f\n",
					ppf, gppf, gpaxxxpf);

#endif

			frameCounter = 0;
			CameraImage::pixelCounterGradient = 0;
			CameraImage::pixelCountergetPixelAsXXX = 0;

			start = getCurrentTime();

			// save localization data if activated
			if(robot.getConfig().getIntValue("vision.savelocalizationdata", 0) != 0) {
				std::fstream output("localizationInput.pb", std::ios::out | std::ios::trunc | std::ios::binary);
				if (!dataForLocalization.SerializeToOstream(&output)) {
					WARNING("Failed to write to localizationInput.pb");
				}
				else {
					INFO("writing %d bytes into localizatinInput.pb", dataForLocalization.ByteSize());
				}
			}

		}
	}
}


/*------------------------------------------------------------------------------------------------*/


/**
 ** Pushs the object data into the WorldModel
 */
void Vision::pushDataIntoWorldmodel() {

	RectangleObject *ball = &objectExtractor.getBall();
	RectangleObject *yellowGoalL = &objectExtractor.getYellowGoalLeft();
	RectangleObject *yellowGoalR = &objectExtractor.getYellowGoalRight();
	RectangleObject *blueGoalL = &objectExtractor.getBlueGoalLeft();
	RectangleObject *blueGoalR = &objectExtractor.getBlueGoalRight();
	RectangleObject *ybyPole = &objectExtractor.getYBYPole();
	RectangleObject *bybPole = &objectExtractor.getBYBPole();

	// send debug object positions
	DEBUG_TABLE("vision.objects", "ball.basepoint.x", ball->basePoint.x);
	DEBUG_TABLE("vision.objects", "ball.basepoint.y", ball->basePoint.y);
	DEBUG_TABLE("vision.objects", "yellowGoalL.basepoint.x", yellowGoalL->basePoint.x);
	DEBUG_TABLE("vision.objects", "yellowGoalL.basepoint.y", yellowGoalL->basePoint.y);
	DEBUG_TABLE("vision.objects", "yellowGoalR.basepoint.x", yellowGoalR->basePoint.x);
	DEBUG_TABLE("vision.objects", "yellowGoalR.basepoint.y", yellowGoalR->basePoint.y);
	DEBUG_TABLE("vision.objects", "blueGoalL.basepoint.x", blueGoalL->basePoint.x);
	DEBUG_TABLE("vision.objects", "blueGoalL.basepoint.y", blueGoalL->basePoint.y);
	DEBUG_TABLE("vision.objects", "blueGoalR.basepoint.x", blueGoalR->basePoint.x);
	DEBUG_TABLE("vision.objects", "blueGoalR.basepoint.y", blueGoalR->basePoint.y);
	DEBUG_TABLE("vision.objects", "ybyPole.basepoint.x", ybyPole->basePoint.x);
	DEBUG_TABLE("vision.objects", "ybyPole.basepoint.y", ybyPole->basePoint.y);
	DEBUG_TABLE("vision.objects", "bybPole.basepoint.x", bybPole->basePoint.x);
	DEBUG_TABLE("vision.objects", "bybPole.basepoint.y", bybPole->basePoint.y);

	PositionRelative invalidPosition;
	invalidPosition.setInvalid();

	// push ball if present
	if(ball->rectangle.x != -1) {
		// we want the center position on the bottom, because the translation works just for points on the ground
		PositionImage posImg((ball->rectangle.x + ball->rectangle.x + ball->rectangle.width) / 2, ball->rectangle.y + ball->rectangle.height);
		wm.updateVision(VISION_BALL, posImg.translateToRelative());
	}

	// push goal poles
	if (yellowGoalR->rectangle.x != -1 && yellowGoalL->rectangle.x != -1) { // whole goal
		wm.updateVision(VISION_YELLOWGOAL_LEFT,
				PositionImage(yellowGoalL->basePoint.x, yellowGoalL->basePoint.y).translateToRelative());
		wm.updateVision(VISION_YELLOWGOAL_RIGHT,
				PositionImage(yellowGoalR->basePoint.x, yellowGoalR->basePoint.y).translateToRelative());
	}
	else if (yellowGoalL->rectangle.x != -1 && yellowGoalL->type == GOAL_POLE_UNKNOWN_OBJECT) { // unknown pole
		wm.updateVision(VISION_YELLOWGOAL_UNKNOWN,
				PositionImage(yellowGoalL->basePoint.x, yellowGoalL->basePoint.y).translateToRelative());
	}
	else if (yellowGoalL->rectangle.x != -1) { // just left pole
		wm.updateVision(VISION_YELLOWGOAL_LEFT,
				PositionImage(yellowGoalL->basePoint.x, yellowGoalL->basePoint.y).translateToRelative());
	}
	else if (yellowGoalR->rectangle.x != -1) { // just right pole
		wm.updateVision(VISION_YELLOWGOAL_RIGHT,
				PositionImage(yellowGoalR->basePoint.x, yellowGoalR->basePoint.y).translateToRelative());
	}

	if(blueGoalR->rectangle.x != -1 && blueGoalL->rectangle.x != -1) { // whole goal
		wm.updateVision(VISION_BLUEGOAL_LEFT,
				PositionImage(blueGoalL->basePoint.x, blueGoalL->basePoint.y).translateToRelative());
		wm.updateVision(VISION_BLUEGOAL_RIGHT,
				PositionImage(blueGoalR->basePoint.x, blueGoalR->basePoint.y).translateToRelative());
	}
	else if(blueGoalL->rectangle.x != -1 && blueGoalL->type == GOAL_POLE_UNKNOWN_OBJECT) { // unknown pole
		wm.updateVision(VISION_BLUEGOAL_UNKNOWN,
				PositionImage(blueGoalL->basePoint.x, blueGoalL->basePoint.y).translateToRelative());
	}
	else if (blueGoalL->rectangle.x != -1) { // just left pole
		wm.updateVision(VISION_BLUEGOAL_LEFT,
				PositionImage(blueGoalL->basePoint.x, blueGoalL->basePoint.y).translateToRelative());
	}
	else if (blueGoalR->rectangle.x != -1) { // just right pole
		wm.updateVision(VISION_BLUEGOAL_RIGHT,
				PositionImage(blueGoalR->basePoint.x, blueGoalR->basePoint.y).translateToRelative());
	}

	// push poles
	if(bybPole->rectangle.x != -1) {
		// we want the center position on the bottom, because the translation works just for points on the ground
		PositionImage posImg((bybPole->rectangle.x + bybPole->rectangle.x + bybPole->rectangle.width) / 2, bybPole->rectangle.y + bybPole->rectangle.height);
		wm.updateVision(VISION_BYB_POLE, posImg.translateToRelative());
	}

	if(ybyPole->rectangle.x != -1) {
		// we want the center position on the bottom, because the translation works just for points on the ground
		PositionImage posImg((ybyPole->rectangle.x + ybyPole->rectangle.x + ybyPole->rectangle.width) / 2, ybyPole->rectangle.y + ybyPole->rectangle.height);
		wm.updateVision(VISION_YBY_POLE, posImg.translateToRelative());
	}

	// push teammates, opponents and obstacles

	// convert bounding boxes of obstacles to old-school obstacle struct ...
	ObstacleExtractor &obstacleExtractor = objectExtractor.getObstacleExtractor();
	std::vector<ObstacleStruct> cyan, magenta, black;
	for(std::list<BoundingBox>::const_iterator iter = obstacleExtractor.getBlackObstacleBoxes().begin(); iter != obstacleExtractor.getBlackObstacleBoxes().end(); ++iter) {
		const BoundingBox &box = *iter;
		ObstacleStruct obs(PositionImage(box.basePoint.x, box.basePoint.y), 15, BLACK);
		black.push_back(obs);
	}
	for(std::list<BoundingBox>::const_iterator iter = obstacleExtractor.getCyanTeamBoxes().begin(); iter != obstacleExtractor.getCyanTeamBoxes().end(); ++iter) {
		const BoundingBox &box = *iter;
		ObstacleStruct obs(PositionImage(box.basePoint.x, box.basePoint.y), 15, CYAN);
		cyan.push_back(obs);
	}
	for(std::list<BoundingBox>::const_iterator iter = obstacleExtractor.getMagentaTeamBoxes().begin(); iter != obstacleExtractor.getMagentaTeamBoxes().end(); ++iter) {
		const BoundingBox &box = *iter;
		ObstacleStruct obs(PositionImage(box.basePoint.x, box.basePoint.y), 15, MAGENTA);
		magenta.push_back(obs);
	}

	if(Cyan == wm.getCurrentWorldModel()->teamColor) {
		wm.updateVision(VISION_TEAMMATE, cyan);
		wm.updateVision(VISION_OPPONENT, magenta);
	}
	else {
		wm.updateVision(VISION_OPPONENT, cyan);
		wm.updateVision(VISION_TEAMMATE, magenta);
	}
	wm.updateVision(VISION_UNKNOWN, black);

}


/*------------------------------------------------------------------------------------------------*/

/**
 ** Save the vision configuration
 */
void Vision::saveConfiguration() {
	robot.getConfig().save();
	calibration.save("calibration.dat");
}


/*------------------------------------------------------------------------------------------------*/

/**
 * Adds image data to a PBI-object
 * @param pbImageData PBI-object to set data for
 * @param format
 * @param data
 * @param dataLength
 * @param imageWidth
 * @param imageHeight
 */
void addImageData(de::fumanoids::message::ImageData *pbImageData, de::fumanoids::message::ImageFormat format, void* data, int32_t dataLength, int32_t imageWidth, int32_t imageHeight, bool original, bool enhanced) {
	pbImageData->set_data(data, dataLength);
	pbImageData->set_compressed(false);
	pbImageData->set_width(imageWidth);
	pbImageData->set_height(imageHeight);
	pbImageData->set_format(format);
	pbImageData->set_original(original);
	pbImageData->set_enhanced(enhanced);
}

/**
 * Sends image data in form of an protobuf image to the given transport.
 * @param transport
 * @param compress
 * @param leadingSize
 */
void Vision::sendImageData(Transport *transport, bool compress, bool leadingSize) {

	if (transport == 0) {
		WARNING("Can not send image as transport is not established");
		return;
	}

	de::fumanoids::message::Image pbImage;
	de::fumanoids::message::ImageData *pbFullImage = pbImage.add_imagedata();
	de::fumanoids::message::ImageData *pbRawImage  = pbImage.add_imagedata();

	{
		// Critical Section so the image is not changed while we prepare it
		CriticalSectionLock lock(cs);
		IplImage *rgbImage = image->getImageAsRGB(1.0, NULL, false);

#if defined IMAGEFORMAT_YUV422
		addImageData(pbRawImage,  de::fumanoids::message::YUV422_IMAGE, image->currentData,  image->currentDataLength, image->getImageWidth(), image->getImageHeight(), true,  false);
		addImageData(pbFullImage, de::fumanoids::message::RGB_IMAGE,    rgbImage->imageData, rgbImage->imageSize,      rgbImage->width,        rgbImage->height,        false, false);
#elif defined IMAGEFORMAT_BAYER
		addImageData(pbRawImage,  de::fumanoids::message::BAYER_IMAGE,  image->currentData,  image->currentDataLength, image->getImageWidth(), image->getImageHeight(), true,  false);
		addImageData(pbFullImage, de::fumanoids::message::RGB_IMAGE,    rgbImage->imageData, rgbImage->imageSize,      rgbImage->width,        rgbImage->height,        false, true);
#else
		addImageData(pbFullImage, de::fumanoids::message::RGB_IMAGE,    image->currentData,  image->currentDataLength, image->getImageWidth(), image->getImageHeight(), true,  false);
		addImageData(pbFullImage, de::fumanoids::message::RGB_IMAGE,   rgbImage->imageData,  rgbImage->imageSize,      rgbImage->width,        rgbImage->height,        false, true);
#endif

		pbImage.mutable_center()->set_x( TheCameraModel::getInstance().centerX() );
		pbImage.mutable_center()->set_y( TheCameraModel::getInstance().centerY() );
		pbImage.mutable_center()->set_r( TheCameraModel::getInstance().focalLengthX() );
		pbImage.set_robotid(robot.getID());
		pbImage.set_pitch( image->getImagePositionPitch() );
		pbImage.set_roll( image->getImagePositionRoll() );
		pbImage.set_eyeheight( image->getImagePositionHeight() );
		pbImage.set_headangle( image->getImagePositionHeadAngle() );
		pbImage.set_horizon( TheCameraModel::getInstance().horizon(900) );

		if (calibration.calibration.has_camerasettings())
			*pbImage.mutable_camsettings() = calibration.calibration.camerasettings();

		if (cam) {
			pbImage.set_camname( cam->getCameraName() );
		}
	}

	std::string data;
	pbImage.SerializeToString(&data);

	void     *compressedData = 0;
	uint32_t  dataSize       = data.size();

	if (compress) {
//		printf("compressing\n");
		uLongf uncompressedSize = data.size();
		uLongf compressedSize   = compressBound(uncompressedSize);

		// reserve memory for compressed image
		compressedData = malloc(compressedSize);

		if (compressedData != 0) {
			int res = ::compress2((Bytef*)compressedData, &compressedSize, (Bytef*)data.c_str(), uncompressedSize, Z_BEST_COMPRESSION);

			if (res == Z_OK && compressedSize < uncompressedSize) {
				INFO("Image compression finished (compressed to %ld (%d%%) bytes)", compressedSize, compressedSize*100/uncompressedSize);
				dataSize = compressedSize;
			} else {
				printf("compressed size >= uncompressed size\n");
				free(compressedData);
				compressedData = 0;
			}
		}
//		else printf("no memory\n");
	}

	robottime_t startTime    = getCurrentTime();
	int         bytesWritten = 0;

	INFO("Sending image (%scompressed, %d bytes)", compressedData ? "" : "un", dataSize);

	if (leadingSize) {
		if (!compressedData)
			transport->write((uint8_t*)&dataSize, 4);
		else {
			uint32_t uncompressedDataSize = 0;
			transport->write((uint8_t*)&uncompressedDataSize, 4);   // dummy zero as marker (UGLY HACK)
			transport->write((uint8_t*)&dataSize, 4);               // compressed size
			uncompressedDataSize = data.size();
			transport->write((uint8_t*)&uncompressedDataSize, 4);   // uncompressed size
		}
	}

	bytesWritten = transport->write(compressedData ? (const uint8_t*)compressedData : (const uint8_t*)data.c_str(), dataSize);
	if (bytesWritten < 0)
		ERROR("Sending of image failed");
	else {
		double duration = (getCurrentTime() - startTime) / 1000;
		if (duration > 1)
			printf("TCP: sent %d bytes in %.1f seconds (%.1f kbps)\n", bytesWritten, duration, bytesWritten/duration/1024);

		if (bytesWritten < (int)dataSize)
			ERROR("Image sending incomplete, only sent %d bytes (%d missing)", bytesWritten, dataSize - bytesWritten);
	}

	if (compressedData)
		free(compressedData);
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

bool Vision::operationCallback(
	OPERATION operation,
	uint8_t   flags,
	uint8_t  *data,
	uint16_t  dataLen,
	struct sockaddr_in *remoteAddress)
{
	if (operation == OP_SAVEVISIONSETTINGS) {
		saveConfiguration();

	} else if (operation == OP_IMAGECAPTURE	&& !(flags & FLAG_IS_ANSWER)) {
		uint16_t count = data[1];
		uint16_t interval;
		memcpy(&interval, data + 2, 2);
		interval = ntohs(interval);

		if (data[0] == 0) {
			if (imageStreamer != 0) {
				imageStreamer->stopStreaming();
				delete imageStreamer;
				imageStreamer = 0;
			}
			INFO("Stopping capture");
		} else if (data[0] == 1) {
			INFO("Starting capture (%d pictures in %d ms interval)", count, interval);
			imageStreamer = new ImagePresenter(2, IMAGEOVERLAY_OBJECTS);
			imageStreamer->startStreaming(count, interval);
		} else
			ERROR("OP_IMAGECAPTURE received unknown operation type %d", data[0]);
	} else
		return false;

	return true;
}


/*------------------------------------------------------------------------------------------------*/

/**
 **
 */

// TODO: move elsewhere that listens to events!

void Vision::handleImageSaving() {
	if (! robot.getConfig().getIntValue("vision.save.enabled"))
		return;

	static robottime_t lastSave = getCurrentTime();
	if (lastSave + robot.getConfig().getIntValue("vision.save.interval") > getCurrentTime())
		return;

	lastSave = getCurrentTime();

	std::string fileExtension = "pbi"; // robot.getConfig().getStrValue("vision.save.format");
	std::string path          = robot.getConfig().getStrValue("vision.save.path");
	std::stringstream fileName;

	int count = 0;
	do {
		count++;
		fileName.seekp(0, std::ios::beg);
		fileName << path << "/image-" << robot.getName() << "-";

		time_t now = time(NULL);
		struct tm dt;
		localtime_r(&now, &dt);

		fileName << std::setfill('0')
		   << std::setw(4) << dt.tm_year + 1900
		   << std::setw(2) << dt.tm_mon + 1
		   << std::setw(2) << dt.tm_mday
		   << "-"
		   << std::setw(2) << dt.tm_hour
		   << "_"
		   << std::setw(2) << dt.tm_min
		   << "_"
		   << std::setw(2) << dt.tm_sec;

		fileName << "-" << std::setfill('0') << std::setw(2) << count;

		// add extension
		fileName << "." << fileExtension;
	}  while (fileExists(fileName.str().c_str()));

//	INFO("Saving image %s", fileName.str().c_str());

	if (fileExtension == "pbi") {
		TransportFile *transport = new TransportFile(fileName.str().c_str());
		if (transport->open()) {
			sendImageData(transport, false, false);
			transport->close();
		}
		delete transport;
	} else {
//		cvSaveImage(fileName.str().c_str(), image);
	}
}
