/**
 * @{
 * @ingroup vision
 **
 */

#include "imagePresenter.h"

#include "commandLine.h"
#include "feet.h"
#include "image.h"
#include "robot.h"
#include "vision.h"
#include "objectExtractor.h"
#include "objectExtractor_field.h"
#include "objectExtractor_fieldlinefeature.h"
#include "boundingBox.h"
#include "localization/localization.h"
#include "localization/fieldOfPlay.h"
#include "localization/fieldOfPlayLut.h"
#include "motorbus/motorbus.h"
#include "position.h"
#include "transport/transport_file.h"
#include "vision/camera/camera_offline.h"
#include "objectExtractor_ball_lutfree.h"

#include <opencv/highgui.h>

#include <sstream>
#include <iomanip>
#include <functional>
#include <vector>

#include <errno.h>
#include <stdio.h>
#include <dirent.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <time.h>

#include <zlib.h>

#include "paintable.h"

/*------------------------------------------------------------------------------------------------*/

//static CvScalar blue    = cvScalar(255,   0,   0);
//static CvScalar red     = cvScalar(  0,   0, 255);
//static CvScalar yellow  = cvScalar(  0, 255, 255);
//static CvScalar orange  = cvScalar(  0, 163, 255);
//static CvScalar black   = cvScalar(  0,   0,   0);
//static CvScalar magenta = cvScalar(255,   0, 255);
//static CvScalar cyan    = cvScalar(255, 255,   0);
//static CvScalar white   = cvScalar(255, 255, 255);
//static CvScalar green   = cvScalar(  0, 255,   0);
//static CvScalar gray    = cvScalar(128, 128, 128);


/*------------------------------------------------------------------------------------------------*/

class ImagePresenterCmdLineCallback : public CommandLineInterface {
public:
	virtual bool commandLineCallback(CommandLine &cmdLine) {
		std::string cmd = cmdLine.getCommand(0);

		if (cmd == "display") {
			motors.enableTorque(MOTOR_HEAD_TURN, false);

//			printf("\n\n");
//			printf("Command display supports the following options:\n");
//			printf("  e - inverse projection\n");
//			printf("  m - world map\n");
//			printf("  w - self localization\n");
//			printf("\n\n");

			ImagePresenter *imagePresenter = 0;
			std::string mode = cmdLine.getCommand(1);
			if (mode == "m") {
				imagePresenter = new ImagePresenter(1);
				imagePresenter->startWorldMapDisplay();
			} else if (mode == "w") {
				imagePresenter = new ImagePresenter(2);
				imagePresenter->startLocalizationDisplay();
			} else if (mode == "e") {
				imagePresenter = new ImagePresenter(2);
				imagePresenter->startProjectedDisplay();
			} else if (mode == "r") {
				int timeout = atoi(cmdLine.getCommand(2).c_str());
				if (timeout == 0)
					timeout = 10;
				imagePresenter = new ImagePresenter(4, IMAGEOVERLAY_OBJECTS);
				imagePresenter->startRotation("FUmanoid Camera View", timeout);
			} else {
				imagePresenter = new ImagePresenter(4, IMAGEOVERLAY_OBJECTS);
				imagePresenter->startDisplay("FUmanoid");
			}

			imagePresenter->waitForWindowClose();
		}
		else if (cmd == "capture") {
			if (cmdLine.getCommandCount() < 4) {
				ERROR("Missing arguments. To run capture, use ./FUmanoids capture <basename> <extension> <interval_ms> [<count>]");
				return false;
			}
			const char* basename  = cmdLine.getCommand(1).c_str();
			const char* extension = cmdLine.getCommand(2).c_str();
			uint16_t intervalInMs = atoi(cmdLine.getCommand(3).c_str());
			int32_t  count        = atoi(cmdLine.getCommand(4).c_str());

			if (count == 0)
				count = -1;

			motors.enableTorque(MOTOR_HEAD_TURN, false);
			ImagePresenter *imagePresenter = new ImagePresenter(1, IMAGEOVERLAY_NORMAL);
			imagePresenter->saveImage(basename, extension, count, intervalInMs, false);
			imagePresenter->waitForSavingDone();
		}
		else if (cmd == "vision") {
			robot.waitForTermination();
		}
		return true;
	}
};

static ImagePresenterCmdLineCallback imagePresenterCmdLineCallback;
REGISTER_COMMAND("capture", "Capture/Save images",  false, false, false, &imagePresenterCmdLineCallback);
REGISTER_COMMAND("display", "Display image/status", false, false, false, &imagePresenterCmdLineCallback);
REGISTER_COMMAND("vision",  "For vision testing",   false, false, false, &imagePresenterCmdLineCallback);


/*------------------------------------------------------------------------------------------------*/

/**
 **
 */

ImagePresenter::ImagePresenter(int _scale, ImageOverlayModus _overlay, ImagePresentationMode _mode)
	: image(0)
	, scale(_scale)
	, overlay(_overlay)
	, mode(_mode)
	, action(IP_ACTION_IDLE)
	, displayTime(false)
	, displayFileName(false)
	, rotateMode(false)
	, rotateModeTimeout(1*1000)
	, lastRotation(0)
	, fileBaseName("capture")
	, fileExtension("png")
	, count(0)
	, fileMaxCount(-1)
	, fileWithTimeStamp(false)
	, transport(0)
	, windowTitle("Camera")
	, closingTimeout(-1)
	, lastImageTime(0)
	, timeout(200)
{
	cs.setName("ImagePresenter");
	cvInitFont( &font, CV_FONT_HERSHEY_SIMPLEX, 0.35, 0.35, 0.0, 1, CV_AA );
}


/*------------------------------------------------------------------------------------------------*/

/**
 **
 */

ImagePresenter::~ImagePresenter() {
	Events::getInstance().unregisterForEvent(EVT_IMAGE_PROCESSED, this);

	cancel(true);

	if (image != 0) {
		cvReleaseImage(&image);
		image = 0;
	}
}


/*------------------------------------------------------------------------------------------------*/

/** Starts displaying a window with the localization data.
 */

void ImagePresenter::startLocalizationDisplay() {
	if (action != IP_ACTION_IDLE) {
		ERROR("Can not start new image presentation modus while old one is still active");
		return;
	}

	displayTime    = true;
	windowTitle    = "Warped Line Points";

	action = IP_ACTION_DISPLAY;
	mode = IP_MODE_LOCALIZATION;
	run();
}


/*------------------------------------------------------------------------------------------------*/

/** Starts displaying a window with the world map.
 */

void ImagePresenter::startWorldMapDisplay() {
	if (action != IP_ACTION_IDLE) {
		ERROR("Can not start new image presentation modus while old one is still active");
		return;
	}

	displayTime    = true;
	windowTitle    = "Worldmap";

	action = IP_ACTION_DISPLAY;
	mode = IP_MODE_WORLDMAP;
	run();
}


/*------------------------------------------------------------------------------------------------*/

/** Starts displaying the inverse projection of the camera view.
 */

void ImagePresenter::startProjectedDisplay() {
	if (action != IP_ACTION_IDLE) {
		ERROR("Can not start new image presentation modus while old one is still active");
		return;
	}

	displayTime    = true;
	windowTitle    = "Projection";

	action = IP_ACTION_DISPLAY;
	mode = IP_MODE_PROJECTED;
	run();
}


/*------------------------------------------------------------------------------------------------*/

/** Starts displaying a window with the image.
 **
 ** @param windowTitleToSet            Window title to set
 ** @param minIntervalInMilliseconds   Minimum interval between refreshes
 ** @param closingTimeoutInSeconds     After how many seconds to close the window (-1 = never)
 */

void ImagePresenter::startDisplay(const char* windowTitleToSet, uint16_t minIntervalInMilliseconds, int32_t closingTimeoutInSeconds) {
	if (action != IP_ACTION_IDLE) {
		ERROR("Can not start new image presentation modus while old one is still active");
		return;
	}

	displayTime    = true;
	windowTitle    = windowTitleToSet;
	closingTimeout = closingTimeoutInSeconds;
	timeout        = minIntervalInMilliseconds;

	action = IP_ACTION_DISPLAY;
	run();
}


/*------------------------------------------------------------------------------------------------*/

/** Starts displaying a window with the image and rotate the view
 **
 ** @param windowTitleToSet            Window title to set
 ** @param minIntervalInMilliseconds   Minimum interval between refreshes
 ** @param closingTimeoutInSeconds     After how many seconds to close the window (-1 = never)
 */

void ImagePresenter::startRotation(const char* windowTitleToSet, uint16_t rotationTimeInSeconds, uint16_t minIntervalInMilliseconds, int32_t closingTimeoutInSeconds) {
	rotateMode = true;
	rotateModeTimeout = rotationTimeInSeconds * 1000;
	startDisplay(windowTitleToSet, minIntervalInMilliseconds, closingTimeoutInSeconds);
}


/*------------------------------------------------------------------------------------------------*/

/** Close the image window.
 **
 */

void ImagePresenter::stopDisplay() {
	if (action == IP_ACTION_DISPLAY) {
		cancel();
		action = IP_ACTION_IDLE;
	}
}


/*------------------------------------------------------------------------------------------------*/

/** Wait until the image window is closed.
 **
 */

void ImagePresenter::waitForWindowClose() {
	if (action == IP_ACTION_DISPLAY)
		wait();
}


/*------------------------------------------------------------------------------------------------*/

/** Save image(s)
 **
 ** @param basename     Filename (beginning, no extension, no path!)
 ** @param extension    File extension (valid values are .jpg, .png, .bmp)
 ** @param count        How many pictures to take (-1 = unlimited)
 ** @param minInterval  Minimum time in milliseconds between two saves
 ** @param timestamp    Whether to save the image with a timestamp
 */

void ImagePresenter::saveImage(
		const char* basename,
		const char* extension,
		int32_t     _count,
		uint16_t    minInterval,
		bool        timestamp)
{
	if (action != IP_ACTION_IDLE) {
		ERROR("Can not start new image presentation modus while old one is still active");
		return;
	}

	savingFinished.reset();

	// if we want to save more than one file, we do so in subdirectories
	if (_count > 1) {
		DIR *dp;
		struct dirent *dirp;
		std::vector<std::string> files;
		if ((dp = opendir(".")) != NULL) {
			while ((dirp = readdir(dp)) != NULL) {
				files.push_back(std::string(dirp->d_name));
			}
			closedir(dp);
		}

		// look for first unused directory of name "basenameXXX"
		uint32_t directoryID = 0;
		for (uint32_t i = 0; i < files.size(); i++) {
			if (strncmp(files[i].c_str(), basename, strlen(basename)) == 0 && files[i].size() == strlen(basename) + 3) {
				uint32_t number = atoi(files[i].c_str() + strlen(basename));
				if (number > directoryID)
					directoryID = number;
			}
		}

		directoryID++;
		std::stringstream directoryName;
		directoryName << basename << std::setfill('0') << std::setw(3) << directoryID << "/";
		INFO("Images will be stored in directory '%s'", directoryName.str().c_str());
		mkdir( directoryName.str().c_str(), 0755);

		fileBaseName = directoryName.str() + "/" + basename;

	} else {
		fileBaseName = basename;
	}

	fileExtension       = extension;
	count               = 0;
	fileMaxCount        = _count;
	timeout             = minInterval;
	fileWithTimeStamp   = timestamp;
	action              = IP_ACTION_SAVE;

	Events::getInstance().registerForEvent(EVT_IMAGE_PROCESSED, this);
}


/*------------------------------------------------------------------------------------------------*/

/** Wait until all requested images have been saved.
 **
 */

void ImagePresenter::stopSaving() {
	if (action == IP_ACTION_SAVE) {
		Events::getInstance().unregisterForEvent(EVT_IMAGE_PROCESSED, this);
		action = IP_ACTION_IDLE;
		savingFinished.trigger();
	}
}


/*------------------------------------------------------------------------------------------------*/

/** Wait until all requested images have been saved.
 **
 */

void ImagePresenter::waitForSavingDone() {
	if (action == IP_ACTION_SAVE) {
		savingFinished.wait();
	}
}


/*------------------------------------------------------------------------------------------------*/

/** Start sending the image over the network
 **
 */

void ImagePresenter::startStreaming(int32_t _count, uint16_t minIntervalInMilliseconds, Transport* _transport) {
	if (action != IP_ACTION_IDLE) {
		ERROR("Can not start new image presentation modus while old one is still active");
		return;
	}

	count   = (_count <= 0 ? -1 : _count);
	timeout = minIntervalInMilliseconds;
	action = IP_ACTION_STREAM;
	transport = _transport;

	streamingFinished.reset();

	Events::getInstance().registerForEvent(EVT_IMAGE_PROCESSED, this);
}


/*------------------------------------------------------------------------------------------------*/

/** Stop sending images over the network
 **
 */
void ImagePresenter::stopStreaming() {
	if (action == IP_ACTION_STREAM) {
		Events::getInstance().unregisterForEvent(EVT_IMAGE_PROCESSED, this);
		action = IP_ACTION_IDLE;
		streamingFinished.trigger();
	}
}


/*------------------------------------------------------------------------------------------------*/

/** Wait until streaming is done
 **
 */

void ImagePresenter::waitForStreamingDone() {
	if (action == IP_ACTION_STREAM) {
		streamingFinished.wait();
	}
}


/*------------------------------------------------------------------------------------------------*/

/** Event callback.
 **
 ** @param evtType       Type of event triggered
 ** @param imageVoidPtr  Pointer to the image taken
 */

void ImagePresenter::eventCallback(EventType evtType, void* imageVoidPtr) {
	CriticalSectionLock lock(cs);

	// we only process images once every 'timeout' milliseconds
	if (lastImageTime + timeout > getCurrentTime())
		return;

	cameraImage = (CameraImage*)imageVoidPtr;

	// remember current time
	lastImageTime = getCurrentTime();

	// release old image
	if (image != 0)
		cvReleaseImage(&image);

	if (rotateMode && getCurrentTime() > lastRotation + rotateModeTimeout) {
		lastRotation = getCurrentTime();

		if (mode == IP_MODE_NORMAL) {
			if (overlay == IMAGEOVERLAY_MAX)
				mode = IP_MODE_LOCALIZATION;
			else
				overlay = (ImageOverlayModus)((int)overlay+1);
		} else if (mode == IP_MODE_WORLDMAP) {
			mode = IP_MODE_LOCALIZATION;
		} else if (mode == IP_MODE_LOCALIZATION) {
			mode = IP_MODE_UNDISTORTED;
		} else if (mode == IP_MODE_UNDISTORTED) {
			mode = IP_MODE_PROJECTED;
		} else if (mode == IP_MODE_PROJECTED) {
			mode = IP_MODE_NORMAL;
			overlay = IMAGEOVERLAY_NORMAL;
		}
	}

	if (mode == IP_MODE_WORLDMAP) {
		image = createWorldMapImage();
	} else if (mode == IP_MODE_LOCALIZATION) {
		image = createLocalizationImage();
	} else if (mode == IP_MODE_DISTANCE_LUT) {
		image = createDistanceLutImage();
	} else if (mode == IP_MODE_PROJECTED) {
		image = createProjectedImage();
	} else if (mode == IP_MODE_UNDISTORTED) {
		image = createUndistortedImage();
	} else if (mode == IP_MODE_NORMAL) {
		// get new image in proper scale
		if (IMAGEOVERLAY_PROJECTION == overlay)
			image = cameraImage->getImageAsRGB(1.0/scale, Vision::getInstance().getColorMgr());
		else if (IMAGEOVERLAY_GRADIENT == overlay)
			image = cameraImage->getGradientImage(1.0/scale);
		else
			image = cameraImage->getImageAsRGB(1.0/scale);

		// process image
		switch (overlay) {
			case IMAGEOVERLAY_OBJECTS:
				overlayObjects();
				break;
			case IMAGEOVERLAY_EDGES:
				overlayEdges();
				break;
			case IMAGEOVERLAY_LINEPOINTS:
				overlayFieldLines();
				break;
			case IMAGEOVERLAY_FIELDCONTOUR:
				overlayFieldContour();
				break;
			case IMAGEOVERLAY_GRID:
				overlayGrid();
				break;
			case IMAGEOVERLAY_BALL:
				overlayBall();
				break;
			case IMAGEOVERLAY_TEST:
				overlayTest();
				break;
			case IMAGEOVERLAY_NORMAL:
			default:
				break;
		}
	} else {
		ERROR("Unknown mode");
	}

	if (displayTime) {
		time_t now = time(NULL);
		struct tm dt;
		localtime_r(&now, &dt);

		char timeText[9];
		strftime(timeText, 9, "%H:%M:%S", &dt);
		cvPutText(image, timeText, cvPoint(5, 15), &font, red);
	}
	if (displayFileName) {
		if(Vision::getInstance().getCameraType() == "offline") {
			CameraOffline* cam = dynamic_cast<CameraOffline*>(Vision::getInstance().getCamera());
			std::string fileName = cam->getCurrentImageName();
			cvPutText(image, fileName.c_str(), cvPoint(5, image->height-16), &font, red);
		}
	}

	// present image as required
	switch (action) {
	case IP_ACTION_DISPLAY:
		cvResizeWindow(windowTitle.c_str(), image->width, image->height);
		cvShowImage(windowTitle.c_str(), image);
		break;
	case IP_ACTION_SAVE:
		saveImage();
		if (count >= fileMaxCount && fileMaxCount >= 0)
			stopSaving();
		break;
	case IP_ACTION_STREAM:
		if (transport != 0)
			streamImageToNetwork();
		else
			streamImageToFile();

		if (count > 0 && --count == 0)
			stopStreaming();
		break;
	default:
		break;
	}
}


/*------------------------------------------------------------------------------------------------*/

/** Prints the keyboard shortcuts that are available.
 **
 */

void ImagePresenter::printKeyboardHelp() {
	// output keyboard help
	printf("\n\n---- Vision display keyboard help ----\n");
	printf("\n");
	printf("  n  view image with no overlays\n");
	printf("  c  view image with projected colors\n");
	printf("  o  view image with objects\n");
	printf("  p  view image with edges \n");
	printf("  l  view image with line points \n");
	printf("  u  view image with grid \n");
	printf("  g  view gradient image\n");
	printf("  d  view distance lookup table\n");
	printf("  f  view field contour\n");
	printf("  z  view image with test overlay\n");
	printf("  k  view undistorted image\n");
	printf("\n");
	printf("  e  inverse projection\n");
	printf("  m  view world map\n");
	printf("  w  view warped line points and localization stuff\n");
	printf("\n");
	printf("  1  image in original size\n");
	printf("  2  image scaled 1/2\n");
	printf("  4  image scaled 1/4\n");
	printf("\n");
	printf("  9  faster updates\n");
	printf("  0  fewer updates\n");
	printf("\n");
	printf("  s  save image to file\n");
	printf("\n");
	printf("  r  rotating display mode\n");
	printf("  t  toggle display of time\n");
	printf("  h  display this help\n");
	printf("  q  quit display\n");
	printf("\nif offline camera is used with input directory \n");
	printf("  x  next image\n");
	printf("  y  previous image\n");
	printf("--------------------------------------\n\n");
}


/*------------------------------------------------------------------------------------------------*/

/** Function executed in the thread. This is only used to handle keypresses for
 ** the window. The actual update of the window's contents is done in the callback.
 **
 */

void ImagePresenter::threadMain() {
	if (action != IP_ACTION_DISPLAY)
		return;

	// open window
	cvNamedWindow(windowTitle.c_str(), CV_WINDOW_AUTOSIZE);
	cvStartWindowThread();

	// register for event
	Events::getInstance().registerForEvent(EVT_IMAGE_PROCESSED, this);

	printKeyboardHelp();

	// loop until we are quit, i.e. thread is cancelled or window is closed
	robottime_t end = getCurrentTime() + closingTimeout*1000;
	while (isRunning() && cvGetWindowHandle(windowTitle.c_str()) != 0) {
		char c = cvWaitKey(250);

		CriticalSectionLock lock(cs);

		switch (c) {
			case 'n':
			case 'N': mode = IP_MODE_NORMAL; overlay = IMAGEOVERLAY_NORMAL;     break;
			case 'c':
			case 'C': mode = IP_MODE_NORMAL; overlay = IMAGEOVERLAY_PROJECTION; break;
			case 'o':
			case 'O': mode = IP_MODE_NORMAL; overlay = IMAGEOVERLAY_OBJECTS;    break;
			case 'p':
			case 'P': mode = IP_MODE_NORMAL; overlay = IMAGEOVERLAY_EDGES;  break;
			case 'l':
			case 'L': mode = IP_MODE_NORMAL; overlay = IMAGEOVERLAY_LINEPOINTS; break;
			case 'g':
			case 'G': mode = IP_MODE_NORMAL; overlay = IMAGEOVERLAY_GRADIENT;   break;
			case 'u':
			case 'U': mode = IP_MODE_NORMAL; overlay = IMAGEOVERLAY_GRID;       break;
			case 'a':
			case 'A': mode = IP_MODE_NORMAL; overlay = IMAGEOVERLAY_BALL;       break;
			case 'z':
			case 'Z': mode = IP_MODE_NORMAL; overlay = IMAGEOVERLAY_TEST;       break;
			case 'd':
			case 'D': mode = IP_MODE_DISTANCE_LUT; break;
			case 'm':
			case 'M': mode = IP_MODE_WORLDMAP; break;
			case 'w':
			case 'W': mode = IP_MODE_LOCALIZATION; break;
			case 'e':
			case 'E': mode = IP_MODE_PROJECTED; break;
			case 'k':
			case 'K': mode = IP_MODE_UNDISTORTED; break;
			case 'f':
			case 'F': mode = IP_MODE_NORMAL; overlay = IMAGEOVERLAY_FIELDCONTOUR; break;
			case '1': scale = 1; break;
			case '2': scale = 2; break;
			case '4': scale = 4; break;
			case '9': timeout -= 100; printf("timeout: %dms\n", timeout); break;
			case '0': timeout += 100; printf("timeout: %dms\n", timeout); break;
			case 'r':
			case 'R': rotateMode = !rotateMode; break;
			case 's':
			case 'S': saveImage(); break;
			case 't':
			case 'T': displayTime = !displayTime; break;
			case 'h':
			case 'H': printKeyboardHelp(); break;
			case 'x':
			case 'X': CameraOffline::incrImageIdx(); /*localization.increaseVisionDataIndex();*/ break;
			case 'y':
			case 'Y': CameraOffline::decrImageIdx(); /*localization.decreaseVisionDataIndex(); */break;
			case ':':
			case '.': displayFileName = !displayFileName; break;
			case 27: //escape
			case 'q': cvDestroyWindow(windowTitle.c_str()); break;
			default : break;
		}

		if (closingTimeout > 0 && end <= getCurrentTime()) {
			break;
		}
	}

	// unregister for event
	Events::getInstance().unregisterForEvent(EVT_IMAGE_PROCESSED, this);
}


/*------------------------------------------------------------------------------------------------*/

/** Actually save an image.
 **
 */

void ImagePresenter::saveImage() {
	CriticalSectionLock lock(cs);

	std::stringstream fileName;

	do {
		count++;
		fileName.seekp(0, std::ios::beg);
		fileName << fileBaseName;

		if (fileWithTimeStamp) {
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
		}

		if (count > 1 || fileMaxCount > 1)
			fileName << "-" << std::setfill('0') << std::setw(4) << count;

		// add extension
		fileName << "." << fileExtension;
	}  while (fileExists(fileName.str().c_str()));

	INFO("Saving image %s (%d/%d)", fileName.str().c_str(), count, fileMaxCount);

	if (fileExtension == "pbi") {
		TransportFile *transport = new TransportFile(fileName.str().c_str());
		if (transport->open()) {
			Vision::getInstance().sendImageData(transport, false, false);
			transport->close();
		}
		delete transport;
	} else
		cvSaveImage(fileName.str().c_str(), image);

	INFO("Image saved");
}


/*------------------------------------------------------------------------------------------------*/

/**
 **
 */

void ImagePresenter::streamImageToFile() {
	CriticalSectionLock lock(cs);

	const char *tmpFileName = "/tmp/fumanoid-tmp-image.jpg";
	cvSaveImage(tmpFileName, image);

	uint8_t *jpgData = 0;
	int fd = open(tmpFileName, O_RDONLY);
	if (fd >= 0) {
		struct stat fileStats;
		fstat(fd, &fileStats);
		if (fileStats.st_size > 0) {
			jpgData = (uint8_t*) malloc(fileStats.st_size);
			if (fileStats.st_size != ::read(fd, jpgData, fileStats.st_size)) {
				ERROR("Could not read temporary image file");
			}
			comm.sendMessage(OP_IMAGECAPTURE, FLAG_IS_ANSWER, jpgData, fileStats.st_size, 0);
		} else
			ERROR("Empty JPG image (%s)", tmpFileName);

		close(fd);

		if (fd >= 0)
			unlink(tmpFileName);
	} else
		ERROR("JPG image not found");

	if (jpgData)
		free(jpgData);
}


/*------------------------------------------------------------------------------------------------*/

/**
 **
 */

void ImagePresenter::streamImageToNetwork() {
	/*
	** Response format:
	**   uint16_t (BE)  imageWidth
	**   uint16_t (BE)  imageHeight
	**   uint32_t (BE)  uncompressedImageSize
	**   uint32_t (BE)  compressedImageSize
	**   uint8_t[]      imageData
	*/

	// send the width/height right away as this can be used as a rough
	// indicator for timestamps on the recipient side
	uint16_t widthBE  = htons((uint16_t)image->width);
	uint16_t heightBE = htons((uint16_t)image->height);
	transport->write((uint8_t*)&widthBE,  2);
	transport->write((uint8_t*)&heightBE, 2);

	// prepare compression
	bool compress = true;
	uLongf uncompressedSize = image->imageSize;
	uLongf compressedSize   = compressBound(uncompressedSize);

	// send uncompressed size
	uint32_t value32 = htonl(uncompressedSize);
	transport->write((uint8_t*)&value32, 4);

	if (compress) {
		// reserve memory for compressed image
		void* uncompressedImage = image->imageData;
		void* compressedImage   = malloc(compressedSize);

		if (compressedImage != 0) {
			// compress
			int res = ::compress2((Bytef*)compressedImage, &compressedSize, (Bytef*)uncompressedImage, uncompressedSize, Z_BEST_COMPRESSION);

			if (res == Z_OK && compressedSize < uncompressedSize) {
				INFO("Image compression finished (compressed to %ld (%d%%) bytes)", compressedSize, compressedSize*100/uncompressedSize);

				value32 = htonl((uint32_t)compressedSize);
				transport->write((uint8_t*)&value32, 4);

				uint32_t bytesWritten = transport->write((uint8_t*)compressedImage, (int)compressedSize);
				if (bytesWritten != compressedSize)
					ERROR("Compressed image transmission failed, wrote only %d out of %d bytes", bytesWritten, (int32_t)compressedSize);
				else
					INFO("Streamed compressed image - %d out of %d bytes", bytesWritten, compressedSize);
			} else
				compress = false;

			free(compressedImage);
		} else
			compress = false;
	} else
		compress = false;

	if (!compress) {
		value32 = htonl(uncompressedSize);
		transport->write((uint8_t*)&value32, 4);
		int32_t bytesWritten = transport->write((uint8_t*)image->imageData, uncompressedSize);
		INFO("Streamed image - %d out of %d bytes", bytesWritten, uncompressedSize);
	}
}


/*------------------------------------------------------------------------------------------------*/

/** Overlay the recognized objects onto the image
 **
 */

void ImagePresenter::overlayObjects() {
	CriticalSectionLock lock(cs);

	overlayCircle(image);
	overlayHorizon(image);

	ObjectExtractor &objectExtractor = Vision::getInstance().getObjectExtractor();

	RectangleObject ball = objectExtractor.getBall();
	ball.paint(image, scale);
	RectangleObject yellowGoalL = objectExtractor.getYellowGoalLeft();
	yellowGoalL.paint(image, scale);
	RectangleObject yellowGoalR = objectExtractor.getYellowGoalRight();
	yellowGoalR.paint(image, scale);
	RectangleObject blueGoalL = objectExtractor.getBlueGoalLeft();
	blueGoalL.paint(image, scale);
	RectangleObject blueGoalR = objectExtractor.getBlueGoalRight();
	blueGoalR.paint(image, scale);
	RectangleObject yby = objectExtractor.getYBYPole();
	yby.paint(image, scale);
	RectangleObject byb = objectExtractor.getBYBPole();
	byb.paint(image, scale);

	ClusterObject fieldLines = objectExtractor.getFieldLineCluster();
	fieldLines.paint(image, scale);

	overlayFeetAndObstacles();


	// draw extracted field line features
	std::vector<FieldLineFeature> features = objectExtractor.getFieldLineFeatures();
	for(std::vector<FieldLineFeature>::iterator iter = features.begin(); iter != features.end(); ++iter) {
		const FieldLineFeature &f = *iter;

		std::string text;
		if(f.type == LCrossingFieldLine)
			text = "L";
		else if(f.type == TCrossingFieldLine)
			text = "T";
		else if(f.type == XCrossingFieldLine)
			text = "X";
		cvPutText(image, text.c_str(), cvPoint(f.getX()  / scale, f.getY() / scale), &font, red);
	}

}

void ImagePresenter::overlayFieldContour() {
	CriticalSectionLock lock(cs);

	// draw horizon
	overlayHorizon(image);

	FieldExtractor &fieldExtractor = Vision::getInstance().getFieldExtractor();

	// draw original field contour points
	std::vector<CvPoint> fieldEdge = fieldExtractor.getOriginalFieldContour();
	for(uint16_t i = 0; i < fieldEdge.size(); ++i) {
		const CvPoint &p = fieldEdge[i];
		cvCircle(image, cvPoint(p.x / scale, p.y / scale), 5, white, 1, CV_AA, 0);
	}

	// draw filtered field contour points
	std::vector<CvPoint> filteredFieldContour = fieldExtractor.getFilteredFieldContour();
	for(uint16_t i = 0; i < filteredFieldContour.size(); ++i) {
		const CvPoint &p = filteredFieldContour[i];
		cvCircle(image, cvPoint(p.x / scale, p.y / scale), 5, yellow, 1, CV_AA, 0);
	}

	// draw field contour
	for(uint16_t x = 0; x < Vision::getInstance().getImageWidth() - 1; ++x) {
		cvLine(image, cvPoint(x / scale, fieldExtractor.getHeighestFieldCoordinate(x) / scale), cvPoint((x+1) / scale, fieldExtractor.getHeighestFieldCoordinate(x + 1) / scale), red, 1, CV_AA, 0);
	}


	// draw goal extraction histograms
	ObjectExtractor &objectExtractor = Vision::getInstance().getObjectExtractor();
	int16_t* histogramBlueContour = objectExtractor.goalExtractor.histogramBlueContour;
	int16_t* histogramYellowContour = objectExtractor.goalExtractor.histogramYellowContour;

	if (histogramBlueContour != 0 && histogramYellowContour != 0) {
		for(int16_t x = 0; x < 640 - 1; ++x) {
			cvLine(image, cvPoint(x / scale, (Vision::getInstance().getImageHeight() / 2 + 50 + 10 * histogramBlueContour[x]) / scale),
					cvPoint((x+1) / scale, (Vision::getInstance().getImageHeight() / 2 + 50 + 10 * histogramBlueContour[x+1]) / scale),
					blue, 1, CV_AA, 0);
			cvLine(image, cvPoint(x / scale, (Vision::getInstance().getImageHeight() / 2 - 10 * histogramYellowContour[x]) /scale),
						cvPoint((x+1) / scale, (Vision::getInstance().getImageHeight() / 2 - 10 * histogramYellowContour[x+1]) / scale),
						yellow, 1, CV_AA, 0);
		}
	}
}

/*------------------------------------------------------------------------------------------------*/

/**
 * Shows the grid with the cell vectors
 */
void ImagePresenter::overlayGrid() {
	CriticalSectionLock lock(cs);

	GradientVectorGriding &gvg = Vision::getInstance().getGVG();

	for(uint8_t y = 0; y < Vision::getInstance().getImageHeight() / GRID_CELL_SIZE; ++y) {
		for(uint8_t x = 0; x < Vision::getInstance().getImageWidth() / GRID_CELL_SIZE; ++x) {

			GridCell *cell =  gvg.getGridCell(x, y);

			int16_t magn = 0;
			int16_t absX = 0, absY = 0;
			CvPoint pp;
			if(cell->er1.usedPixelCounter != 0) {
				magn = cell->er1.magnitude;
				absX = gvg.getAbsolutePosition(cell->er1, x,y).x;
				absY = gvg.getAbsolutePosition(cell->er1, x,y).y;

				pp = cvPoint(absX / scale, absY / scale);

				CvPoint dirP = cvPoint((absX + cell->er1.dx * 10 / magn) / scale, (absY + cell->er1.dy * 10 / magn) / scale);
				cvLine(image, pp, dirP, yellow, 1, CV_AA, 0);
				cvCircle(image, pp, 2, magenta, 1, CV_AA, 0);
			}
			if(cell->er2.usedPixelCounter != 0) {
				magn = cell->er2.magnitude;
				absX = gvg.getAbsolutePosition(cell->er2, x,y).x;
				absY = gvg.getAbsolutePosition(cell->er2, x,y).y;

				pp = cvPoint(absX / scale, absY / scale);

				CvPoint dirP = cvPoint((absX + cell->er2.dx * 10 / magn) / scale, (absY + cell->er2.dy * 10 / magn) / scale);
				cvLine(image, pp, dirP, blue, 1, CV_AA, 0);
				cvCircle(image, pp, 2, cyan, 1, CV_AA, 0);
			}

		}
	}

	// paint grid
	for(int y = GRID_CELL_SIZE; y < Vision::getInstance().getImageHeight() -1; y += GRID_CELL_SIZE) {
		cvLine(image, cvPoint(0,y / scale), cvPoint(Vision::getInstance().getImageWidth() / scale, y / scale), black, 1, CV_AA, 0);
	}

	for(int x = GRID_CELL_SIZE; x < Vision::getInstance().getImageWidth() - 1; x += GRID_CELL_SIZE) {
		cvLine(image, cvPoint(x / scale, 0), cvPoint(x /scale, Vision::getInstance().getImageHeight() / scale), black, 1, CV_AA,0);
	}
}

/*------------------------------------------------------------------------------------------------*/

/** Given an image, overlay the edges
 **
 */
void ImagePresenter::overlayEdges() {
	CriticalSectionLock lock(cs);

	// draw edges (without field line edges)
	EdgeVector edges = Vision::getInstance().getEdges();
	EdgeVector::iterator iter;
	for (iter = edges.begin(); iter != edges.end(); ++iter) {
		(*iter)->paint(image, scale);
	}

	// draw field line edges
	EdgeVector lines = Vision::getInstance().getGVG().getFieldLineEdges();
	for (iter = lines.begin(); iter != lines.end(); ++iter) {
		(*iter)->paint(image, scale);
	}


	// draw grid
	for(int y = GRID_CELL_SIZE; y < Vision::getInstance().getImageHeight() -1; y += GRID_CELL_SIZE) {
		cvLine(image, cvPoint(0,y / scale), cvPoint(Vision::getInstance().getImageWidth() / scale, y / scale), black, 1, CV_AA, 0);
	}

	for(int x = GRID_CELL_SIZE; x < Vision::getInstance().getImageWidth() - 1; x += GRID_CELL_SIZE) {
		cvLine(image, cvPoint(x / scale, 0), cvPoint(x /scale, Vision::getInstance().getImageHeight() / scale), black, 1, CV_AA,0);
	}
}

/*------------------------------------------------------------------------------------------------*/

/** Show the line points
 **
 */

void ImagePresenter::overlayFieldLines() {
	CriticalSectionLock lock(cs);

	// draw raw field line features
	FieldLineFeatureExtractor &fle = Vision::getInstance().getObjectExtractor().getFieldLineFeatureExtractor();
	std::vector<FieldLineFeature> rawFeatures = fle.getRawLineFeatures();
	for(std::vector<FieldLineFeature>::iterator iter = rawFeatures.begin(); iter != rawFeatures.end(); ++iter) {
		const FieldLineFeature &f = *iter;

		std::string text;
		if(f.type == LCrossingFieldLine)
			text = "L";
		else if(f.type == TCrossingFieldLine)
			text = "T";
		else if(f.type == XCrossingFieldLine)
			text = "X";
		cvPutText(image, text.c_str(), cvPoint(f.getX()  / scale, f.getY() / scale), &font, yellow);
	}

	// draw extracted field line features
	std::vector<FieldLineFeature> features = fle.getLineFeatures();
	for(std::vector<FieldLineFeature>::iterator iter = features.begin(); iter != features.end(); ++iter) {
		const FieldLineFeature &f = *iter;

		std::string text;
		if(f.type == LCrossingFieldLine)
			text = "L";
		else if(f.type == TCrossingFieldLine)
			text = "T";
		else if(f.type == XCrossingFieldLine)
			text = "X";
		cvPutText(image, text.c_str(), cvPoint(f.getX()  / scale, f.getY() / scale), &font, red);
	}

	// draw debug stuff for field line feature extraction
	std::list<BoundingBox> groupedFeatures = fle.getGroupedFeatures();
	for(std::list<BoundingBox>::iterator iter = groupedFeatures.begin(); iter != groupedFeatures.end(); ++iter) {
		BoundingBox &box = *iter;
		box.paint(image, scale);

		cvCircle(image, cvPoint((box.rectangle.x + box.rectangle.x + box.rectangle.width) / 2 / scale, (box.rectangle.y + box.rectangle.y + box.rectangle.height) / 2 / scale), 1, red, 1, CV_AA, 0);

		// calculate center of bounding box
		int centerX = (box.rectangle.x + box.rectangle.x + box.rectangle.width) / 2;
		int centerY = (box.rectangle.y + box.rectangle.y + box.rectangle.height) / 2;

		for(int angle = -180; angle < 180; angle += 5) {

			int x = centerX + toInt(fmul(fcos(toFixed(angle)), toFixed(30)));
			int y = centerY + toInt(fmul(fsin(toFixed(angle)), toFixed(30)));

			// check if outside the field
			if(Vision::getInstance().getFieldExtractor().getHeighestFieldCoordinate(x) > y) {
				continue;
			}

			cvCircle(image, cvPoint(x / scale, y / scale), 1, red, 1, CV_AA, 0);

			Color color = Vision::getInstance().getColorMgr()->getPixelColor(*(IMAGETYPE*)Vision::getInstance().getCameraImage(), x, y);
			if(color == White) {
				cvCircle(image, cvPoint(x / scale, y / scale), 2, yellow, 1, CV_AA, 0);
			}
		}

	}

	// draw field line edges
	EdgeVector edges = fle.getFieldLineEdges();
	EdgeVector::iterator iter;

	for (iter = edges.begin(); iter != edges.end(); ++iter) {
		const Edge *edge = *iter;

		// draw edge (magenta)
		for (std::vector<EdgePoint>::const_iterator piter = edge->linePoints.begin(); piter != edge->linePoints.end()-1; ++piter) {
			cvLine(image, cvPoint(piter->getX() / scale, piter->getY() / scale), cvPoint((piter+1)->getX() / scale, (piter+1)->getY() / scale), magenta, 2);
		}

		// draw begin and end of edge in cyan
		CvPoint s = cvPoint(edge->getFirstPoint().getX(), edge->getFirstPoint().getY());
		CvPoint e = cvPoint(edge->getLastPoint().getX(), edge->getLastPoint().getY());
		cvLine(image, cvPoint(s.x/scale, s.y/scale),
					cvPoint(s.x/scale, s.y/scale),
					cyan, 4);
		cvLine(image, cvPoint(e.x/scale, e.y/scale),
					cvPoint(e.x/scale, e.y/scale),
					cyan, 4);
	}

}

/*------------------------------------------------------------------------------------------------*/

/**
 * Overlay for test purpose
 */
void ImagePresenter::overlayTest() {
	CriticalSectionLock lock(cs);

	// please don't commit any changes you made here ...


}

/*------------------------------------------------------------------------------------------------*/

/** Given an image, overlay the feet and obstacles
 */

void ImagePresenter::overlayFeetAndObstacles(bool showFeet, bool showObstacles) {
	if (showFeet) {
		// mark feetspace
		int16_t left, top, right, bottom;
		Feet::getInstance().getFeetSpace(left, top, right, bottom);
		cvRectangle(image, cvPoint(left/scale, top/scale), cvPoint(right/scale, bottom/scale), white);
	}

	if(showObstacles) {
		ObjectExtractor &objectExtractor = Vision::getInstance().getObjectExtractor();

		std::list<BoundingBox> blackObs = objectExtractor.getBlackObstacleBoxes();
		for(std::list<BoundingBox>::const_iterator iter = blackObs.begin(); iter != blackObs.end(); ++iter) {
			const BoundingBox &box = *iter;

			CvPoint scaledP1 = { box.rectangle.x / scale, box.rectangle.y / scale };
			CvPoint scaledP2 = { (box.rectangle.x + box.rectangle.width) / scale,  (box.rectangle.y + box.rectangle.height) / scale };
			CvPoint scaledBasePoint = { box.basePoint.x / scale, box.basePoint.y / scale };
			cvRectangle(image, scaledP1, scaledP2, black, 1, CV_AA, 0);
			cvCircle(image, scaledBasePoint, 2, black, 1, CV_AA, 0);
		}
		std::list<BoundingBox> cyanObs = objectExtractor.getCyanTeamBoxes();
		for(std::list<BoundingBox>::iterator iter = cyanObs.begin(); iter != cyanObs.end(); ++iter) {
			const BoundingBox &box = *iter;

			CvPoint scaledP1 = { box.rectangle.x / scale, box.rectangle.y / scale };
			CvPoint scaledP2 = { (box.rectangle.x + box.rectangle.width) / scale,  (box.rectangle.y + box.rectangle.height) / scale };
			CvPoint scaledBasePoint = { box.basePoint.x / scale, box.basePoint.y / scale };
			cvRectangle(image, scaledP1, scaledP2, cyan, 1, CV_AA, 0);
			cvCircle(image, scaledBasePoint, 2, black, 1, CV_AA, 0);
		}
		std::list<BoundingBox> magentaObs = objectExtractor.getMagentaTeamBoxes();
		for(std::list<BoundingBox>::iterator iter = magentaObs.begin(); iter != magentaObs.end(); ++iter) {
			const BoundingBox &box = *iter;

			CvPoint scaledP1 = { box.rectangle.x / scale, box.rectangle.y / scale };
			CvPoint scaledP2 = { (box.rectangle.x + box.rectangle.width) / scale,  (box.rectangle.y + box.rectangle.height) / scale };
			CvPoint scaledBasePoint = { box.basePoint.x / scale, box.basePoint.y / scale };
			cvRectangle(image, scaledP1, scaledP2, magenta, 1, CV_AA, 0);
			cvCircle(image, scaledBasePoint, 2, black, 1, CV_AA, 0);
		}
	}
}

/**
 * Overlays the horizon in the given image
 * @param img
 */
void ImagePresenter::overlayHorizon(IplImage *img) {
	int16_t horizonY = TheCameraModel::getInstance().horizon(900);
	cvLine(img,cvPoint(0,horizonY/scale), cvPoint(Vision::getInstance().getImageWidth()/scale, horizonY/scale), white, 1, CV_AA, 0);
}

/**
 * Overlays the calibrated circle and the center in the given image
 * @param img
 */
void ImagePresenter::overlayCircle(IplImage *img) {
	int16_t centerX = TheCameraModel::getInstance().centerX();
	int16_t centerY = TheCameraModel::getInstance().centerY();

	int16_t f = TheCameraModel::getInstance().focalLengthX();

	// center
	cvCircle(img, cvPoint(centerX / scale, centerY / scale), 2, white, CV_FILLED, CV_AA, 0);
	// circle
	cvCircle(img, cvPoint(centerX / scale, centerY / scale), f / scale, white, 1, CV_AA, 0);
}

/*------------------------------------------------------------------------------------------------*/


/** Create an image of the playing field.
 **
 */

IplImage* ImagePresenter::createFieldImage() {
	CriticalSectionLock lock(cs);

	// Konstanten aus der Spielfeldskizze
	int A = FieldOfPlay::A / scale;
	int B = FieldOfPlay::B / scale;
	int C = FieldOfPlay::C / scale;
	int D = FieldOfPlay::D / scale;
	int E = FieldOfPlay::E / scale;
	int G = FieldOfPlay::G / scale;
	int I = FieldOfPlay::I / scale;
	int J = FieldOfPlay::J / scale;

	int MARKER_LENGTH = 10 / scale;
	int LINE_WIDTH    =  5 / scale;

	PositionAbsolute blueGoal = FieldOfPlay::blueGoalCenter();
	PositionAbsolute yellowGoal = FieldOfPlay::yellowGoalCenter();
	PositionAbsolute bybPole = FieldOfPlay::bybPole();
	PositionAbsolute ybyPole = FieldOfPlay::ybyPole();

	IplImage* field = cvCreateImage(cvSize(A+2*J, B+2*J), IPL_DEPTH_8U, 3);
	cvSetZero(field);

	cvLine(field, cvPoint(J, J),         cvPoint(A + J, J),         white, LINE_WIDTH);
	cvLine(field, cvPoint(A + J, J),     cvPoint(A + J, B + J),     white, LINE_WIDTH);
	cvLine(field, cvPoint(A + J, B + J), cvPoint(J, B + J),         white, LINE_WIDTH);
	cvLine(field, cvPoint(J, B + J),     cvPoint(J, J),             white, LINE_WIDTH);
	cvLine(field, cvPoint(A / 2 + J, J), cvPoint(A / 2 + J, B + J), white, LINE_WIDTH);
	cvCircle(field, cvPoint(A / 2 + J, B / 2 + J), I/2 , white, LINE_WIDTH);
	cvLine(field, cvPoint(A / 2 + J - MARKER_LENGTH / 2, B/2 + J),
			cvPoint(A / 2 + J + MARKER_LENGTH / 2, B / 2 + J), white, LINE_WIDTH);

	cvLine(field, cvPoint(G + J - MARKER_LENGTH / 2, B / 2 + J),
			cvPoint(G + J + MARKER_LENGTH / 2, B / 2 + J), white, LINE_WIDTH);
	cvLine(field, cvPoint(G + J, B / 2 + J - MARKER_LENGTH / 2),
			cvPoint(G + J, B / 2 + J + MARKER_LENGTH / 2), white, LINE_WIDTH);

	cvLine(field, cvPoint(A - G + J - MARKER_LENGTH / 2, B /2 + J),
			cvPoint(A - G + J + MARKER_LENGTH / 2, B / 2 + J), white, LINE_WIDTH);
	cvLine(field, cvPoint(A - G + J, B / 2 + J - MARKER_LENGTH / 2),
			cvPoint(A - G + J, B / 2 + J + MARKER_LENGTH / 2), white, LINE_WIDTH);

	cvLine(field, cvPoint(J,J+C),      cvPoint(J+E, J+C),      white, LINE_WIDTH);
	cvLine(field, cvPoint(J,J+B-C),    cvPoint(J+E, J+B-C),    white, LINE_WIDTH);
	cvLine(field, cvPoint(J+A,J+C),    cvPoint(J+A-E, J+C),    white, LINE_WIDTH);
	cvLine(field, cvPoint(J+A,J+B-C),  cvPoint(J+A-E, J+B-C),  white, LINE_WIDTH);

	cvLine(field, cvPoint(J+E, J+C),   cvPoint(J+E, J+B-C),    white, LINE_WIDTH);
	cvLine(field, cvPoint(J+A-E, J+C), cvPoint(J+A-E, J+B-C) , white, LINE_WIDTH);

	cvLine(field, cvPoint(blueGoal.getX()/scale+A/2+J, blueGoal.getY()/scale+B/2+J-D/2), cvPoint(blueGoal.getX()/scale+A/2+J, blueGoal.getY()/scale+B/2+J+D/2), blue, 8/scale);
	cvLine(field, cvPoint(yellowGoal.getX()/scale+A/2+J, yellowGoal.getY()/scale+B/2+J-D/2), cvPoint(yellowGoal.getX()/scale+A/2+J, yellowGoal.getY()/scale+B/2+J+D/2), yellow, 8/scale);
	cvCircle(field, cvPoint(ybyPole.getX()/scale+A/2+J, -ybyPole.getY()/scale+B/2+J), 8/scale, blue, -1);
	cvCircle(field, cvPoint(ybyPole.getX()/scale+A/2+J, -ybyPole.getY()/scale+B/2+J), 8/scale, yellow, 3/scale);
	cvCircle(field, cvPoint(bybPole.getX()/scale+A/2+J, -bybPole.getY()/scale+B/2+J), 8/scale, yellow, -1);
	cvCircle(field, cvPoint(bybPole.getX()/scale+A/2+J, -bybPole.getY()/scale+B/2+J), 8/scale, blue, 3/scale);

	return field;
}


/*------------------------------------------------------------------------------------------------*/

/** Show the world map
 **
 */

IplImage* ImagePresenter::createWorldMapImage() {
	CriticalSectionLock lock(cs);

	IplImage *field = createFieldImage();

	worldModelOutput cWM = wm.getCurrentWorldModel();

	std::vector<ObstacleStruct>* obstacles = &cWM->obstacles;

	const int obstacleWidth=20/scale;
	PositionRobot myPos = cWM->robotPos;

	// draw obstacles
	for (std::vector<ObstacleStruct>::const_iterator iter = obstacles->begin(); iter != obstacles->end(); iter++) {

		const ObstacleStruct &obs = *iter;

		PositionAbsolute absPos = obs.posRel.translateToAbsolute();

		if (abs(absPos.getX() > 320) || abs(absPos.getY()) > 220) // TODO:
			continue;

		// draw a square of with center at absPos and a side length of obstacleWidth
		cvRectangle(field, cvPoint( ((absPos.getX()/scale + field->width/2) - obstacleWidth/2),
				(-absPos.getY()/scale + field->height/2 - obstacleWidth/2)), cvPoint( ((absPos.getX()/scale + field->width/2) + obstacleWidth/2),
						(-absPos.getY()/scale + field->height/2 + obstacleWidth/2)), gray);
	}
	// draw myself
	cvRectangle(field,
				cvPoint((myPos.getX()/scale + field->width/2 - obstacleWidth / 2),
					(-myPos.getY()/scale + field->height/2 - obstacleWidth/2)),
				cvPoint((myPos.getX()/scale + field->width/2 + obstacleWidth / 2),
					(-myPos.getY()/scale + field->height/2 + obstacleWidth/2)),
				red);

	// draw ball
	PositionAbsolute absPos;
	if (getCurrentTime()-cWM->ballLastSeen<500) {
		absPos = cWM->ballAbs;
		cvCircle(field, cvPoint((absPos.getX()/scale+field->width/2), (-absPos.getY()/scale+field->height/2)), 1, cvScalar(0,128,255), 8/scale);
	}

	return field;
}


/*------------------------------------------------------------------------------------------------*/

/** Show data of the localization with particles, warped lines, landmarks etc.
 **
 */


IplImage* ImagePresenter::createLocalizationImage() {
	using namespace FixedPointMath;
	CriticalSectionLock lock(cs);

	worldModelOutput cWM = wm.getCurrentWorldModel();

	IplImage *field = createFieldImage();

	// print pitch/ roll/ headangle
	char angleText[60];
	sprintf(angleText, "pitch: %f roll: %f headAngle: %f", toFloat(localization.localizationData.pitch), toFloat(localization.localizationData.roll), toFloat(localization.localizationData.headAngle));
	cvPutText(field, angleText, cvPoint(5, 30), &font, red);

	// draw angle histogram (green)
	std::vector<int> histogram = localization.getAngleHistogram();
	int max = *std::max_element(histogram.begin(), histogram.end());
	double s = 1.0;
	if (max > field->height/2)
		s = field->height/2.0 / max;
	for (int i=0; i<90; i++) {
		cvLine(field, cvPoint(10+i, 0), cvPoint(10+i, (int)(s*histogram[i])), green);
	}

	Edges warpedEdges = localization.getSplittedEdges();
	PositionParticle robotPos = localization.getBestPosition();

//	// draw circle histogram (green circles)
	if(localization.edgeEvaluator.getCircleCenter().isValid()) {
		std::vector<int> acc = localization.getCircleHistogram();
		max = *std::max_element(acc.begin(), acc.end());
		s = 255.0/max;
		for (int i=0; i<61; ++i) {
			for (int j=0; j<61; ++j) {
				PositionRelative r((j-30)*10, (i-30)*10);
				PositionAbsolute a = r.translateToAbsolute(robotPos);
				if (acc[i*61+j] > 0) {
					cvCircle(field, cvPoint(a.getX()/scale + field->width/2 , -a.getY()/scale + field->height/2), 1, CV_RGB(0, s*acc[i*61+j], 0), 2);
				}
				if (acc[i*61+j] == max) {
					cvCircle(field, cvPoint(a.getX()/scale + field->width/2 , -a.getY()/scale + field->height/2), 1, CV_RGB(0, s*acc[i*61+j], 0), 8);
				}
			}
		}
	}

	// draw edges colored by type
	// draw circle edges (magenta) and straight edges (cyan)
	CvScalar edgeColor;
	for (Edges::iterator i = warpedEdges.begin(); i != warpedEdges.end(); ++i) {
		if (i->size()<2)
			continue;

		for (Edgels::iterator j = i->begin(); j != i->end()-1; ++j) {
			PositionAbsolute a1 = j->translateToAbsolute(robotPos);
			PositionAbsolute a2 = (j+1)->translateToAbsolute(robotPos);

			if ( (*i).edgeType == CircleEdge ) {
				edgeColor = magenta;
			} else if ( (*i).edgeType == StraightEdge ) {
				edgeColor = cyan;
			}		
			else {
				edgeColor = gray;
			}
			cvLine(field, cvPoint(a1.getX()/scale + field->width/2 , -a1.getY()/scale + field->height/2),
					cvPoint(a2.getX()/scale + field->width/2 , -a2.getY()/scale + field->height/2),
					edgeColor, 2);

			if(j == i->begin()) {
				cvCircle(field, cvPoint(a1.getX() / scale + field->width/2, -a1.getY() / scale + field->height / 2),2, red, 1, CV_AA, 0 );
			}
		}
	}

	// draw field line features
	std::vector<FieldLineFeatureRelative> fieldLineFeatures = localization.localizationData.fieldLineFeatures;
	for(std::vector<FieldLineFeatureRelative>::const_iterator iter = fieldLineFeatures.begin(); iter != fieldLineFeatures.end(); ++iter) {
		const FieldLineFeatureRelative &f = *iter;

		PositionAbsolute posAbs = f.translateToAbsolute(robotPos);

		std::string text;
		if(f.type == LCrossingFieldLine)
			text = "L";
		else if(f.type == TCrossingFieldLine)
			text = "T";
		else if(f.type == XCrossingFieldLine)
			text = "X";
		cvPutText(field, text.c_str(), cvPoint(posAbs.getX() / scale + field->width/2, - posAbs.getY() / scale + field->height / 2), &font, blue);

	}

	// draw seen landmarks
	if(localization.localizationData.myGoalLeft.isValid()) {
		CvScalar color;
		PositionAbsolute posAbs = localization.localizationData.myGoalLeft.translateToAbsolute(robotPos);
		if(cWM->ourGoalColor == BlueGoal) {
			color = blue;
		}
		else {
			color = yellow;
		}
		cvCircle(field, cvPoint(posAbs.getX() / scale + field->width/2, -posAbs.getY() / scale + field->height / 2), 2, color, 3, CV_AA, 0 );
	}
	if(localization.localizationData.myGoalRight.isValid()) {
		CvScalar color;
		PositionAbsolute posAbs = localization.localizationData.myGoalRight.translateToAbsolute(robotPos);
		if(cWM->ourGoalColor == BlueGoal) {
			color = blue;
		}
		else {
			color = yellow;
		}
		cvCircle(field, cvPoint(posAbs.getX() / scale + field->width/2, -posAbs.getY() / scale + field->height / 2), 2, color, 3, CV_AA, 0 );
	}
	if(localization.localizationData.myGoalUnknown.isValid()) {
		CvScalar color;
		PositionAbsolute posAbs = localization.localizationData.myGoalUnknown.translateToAbsolute(robotPos);
		if(cWM->ourGoalColor == BlueGoal) {
			color = blue;
		}
		else {
			color = yellow;
		}
		cvCircle(field, cvPoint(posAbs.getX() / scale + field->width/2, -posAbs.getY() / scale + field->height / 2), 2, color, 3, CV_AA, 0 );
	}
	if(localization.localizationData.oppGoalLeft.isValid()) {
		CvScalar color;
		PositionAbsolute posAbs = localization.localizationData.oppGoalLeft.translateToAbsolute(robotPos);
		if(cWM->ourGoalColor == BlueGoal) {
			color = yellow;
		}
		else {
			color = blue;
		}
		cvCircle(field, cvPoint(posAbs.getX() / scale + field->width/2, -posAbs.getY() / scale + field->height / 2), 2, color, 3, CV_AA, 0 );
	}
	if(localization.localizationData.oppGoalRight.isValid()) {
		CvScalar color;
		PositionAbsolute posAbs = localization.localizationData.oppGoalRight.translateToAbsolute(robotPos);
		if(cWM->ourGoalColor == BlueGoal) {
			color = yellow;
		}
		else {
			color = blue;
		}
		cvCircle(field, cvPoint(posAbs.getX() / scale + field->width/2, -posAbs.getY() / scale + field->height / 2), 2, color, 3, CV_AA, 0 );
	}
	if(localization.localizationData.oppGoalUnknown.isValid()) {
		CvScalar color;
		PositionAbsolute posAbs = localization.localizationData.oppGoalUnknown.translateToAbsolute(robotPos);
		if(cWM->ourGoalColor == BlueGoal) {
			color = yellow;
		}
		else {
			color = blue;
		}
		cvCircle(field, cvPoint(posAbs.getX() / scale + field->width/2, -posAbs.getY() / scale + field->height / 2), 2, color, 3, CV_AA, 0 );
	}
	if(localization.localizationData.bybPole.isValid()) {
		PositionAbsolute posAbs = localization.localizationData.bybPole.translateToAbsolute(robotPos);
		cvCircle(field, cvPoint(posAbs.getX() / scale + field->width/2, -posAbs.getY() / scale + field->height / 2), 2, yellow, -1, CV_AA, 0 );
		cvCircle(field, cvPoint(posAbs.getX() / scale + field->width/2, -posAbs.getY() / scale + field->height / 2), 4, blue, 2, CV_AA, 0 );
	}
	if(localization.localizationData.ybyPole.isValid()) {
		PositionAbsolute posAbs = localization.localizationData.ybyPole.translateToAbsolute(robotPos);
		cvCircle(field, cvPoint(posAbs.getX() / scale + field->width/2, -posAbs.getY() / scale + field->height / 2), 2, blue, -1, CV_AA, 0 );
		cvCircle(field, cvPoint(posAbs.getX() / scale + field->width/2, -posAbs.getY() / scale + field->height / 2), 4, yellow, 2, CV_AA, 0 );
	}


#if 0
std::vector<std::pair<PositionRelative, PositionRelative> > lineSegments = localization.lineSegments();

for (size_t i=0; i < lineSegments.size(); ++i ) {
	PositionAbsolute a1 = lineSegments[i].first.translateToAbsolute(robotPos);
	PositionAbsolute a2 = lineSegments[i].second.translateToAbsolute(robotPos);
	cvLine(field, cvPoint(a1.getX()/scale + field->width/2 , -a1.getY()/scale + field->height/2),
						cvPoint(a2.getX()/scale + field->width/2 , -a2.getY()/scale + field->height/2),
						red, 2);
}

#endif

#if 1
	// draw particles (orange) and best particle (current estimated position) in yellow
	std::vector<PositionParticle> particles = localization.particleFilter.getParticlesBeforeResampling();
	if (particles.size() > 0) {
		PositionParticle worst = *std::min_element(particles.begin(), particles.end());

		s = 50.0 / (robotPos.getBelief()-worst.getBelief());
		int offset = 0;//16 / scale;
		for (std::vector<PositionParticle>::iterator i = particles.begin(); i!= particles.end(); ++i) {
			PositionRelative eye((int)(s*(i->getBelief() - worst.getBelief()) + offset), 0);
			PositionAbsolute eyeAbs = eye.translateToAbsolute(*i);
			cvCircle(field, cvPoint(i->getX()/scale + field->width/2 , -i->getY()/scale + field->height/2), 1, orange, 4/scale);
			cvLine(field, cvPoint(i->getX()/scale + field->width/2 , -i->getY()/scale + field->height/2),
					cvPoint(eyeAbs.getX()/scale + field->width/2 , -eyeAbs.getY()/scale + field->height/2), cvScalar(0, 100, 255), 1);

			// print belief on particles
//			char getBelief[10];
//			sprintf(getBelief, "%f", i->getBelief());
//			cvPutText(field, getBelief, cvPoint(i->getX()/scale + field->width/2 + 5, -i->getY()/ scale + field->height/2), &font, red);
		}

		// draw robot's position
		PositionRelative eye((int)(s* (robotPos.getBelief() - worst.getBelief() ) + offset), 0);
		PositionAbsolute eyeAbs = eye.translateToAbsolute(robotPos);
		cvCircle(field, cvPoint(robotPos.getX()/scale + field->width/2 , -robotPos.getY()/scale + field->height/2), 1, red, 8/scale+1);
		cvLine(field, cvPoint(robotPos.getX()/scale + field->width/2 , -robotPos.getY()/scale + field->height/2),
				cvPoint(eyeAbs.getX()/scale + field->width/2 , -eyeAbs.getY()/scale + field->height/2), red, 1);
	}
#endif
	return field;
}


/*------------------------------------------------------------------------------------------------*/

/** Project image in relative coordinates (bird's perspective)
 **
 */

IplImage* ImagePresenter::createProjectedImage() {
	IMAGETYPE* rawImage = (IMAGETYPE*)cameraImage;

	IplImage *projectedImage = cvCreateImage(cvSize(cameraImage->getImageWidth(), cameraImage->getImageHeight()), IPL_DEPTH_8U, 3);
	cvSetZero(projectedImage);

	int lastXX = 0;
	for (int x=0; x < cameraImage->getImageWidth(); x+=2) {
		lastXX = -1;
		for (int y=0; y < cameraImage->getImageHeight(); y+=2) {
			PositionImage imagePos(x, y);
			PositionRelative relPos = TheCameraModel::getInstance().inversePerspectiveProjection(imagePos,
					toFixed(cameraImage->getImagePositionPitch()), toFixed(cameraImage->getImagePositionRoll()), toFixed(cameraImage->getImagePositionHeadAngle()));

			uint8_t r, g, b;
			rawImage->getPixelAsRGB(x, y, &r, &g, &b);


			int xx = relPos.getX();
			int yy = (-relPos.getY() + 240);

			if (0 <= xx && xx < cameraImage->getImageWidth() && 0 <= yy && yy < cameraImage->getImageHeight()-2) {
				if (lastXX == -1)
					lastXX = xx;
				cvRectangle(projectedImage, cvPoint(lastXX, yy), cvPoint(xx, yy+2), cvScalar(b, g, r), CV_FILLED);
				lastXX = xx;
			}
		}
	}

	return projectedImage;
}


/**
 * Show ball found with lut free method
 */
void ImagePresenter::overlayBall() {
	CriticalSectionLock lock(cs);

	int16_t horizonY = TheCameraModel::getInstance().horizon(900);
	horizonY = ::max(0,horizonY - (horizonY&(GRID_CELL_SIZE-1)));
	BallExtractorLutFree& ballExtractor = Vision::getInstance().
					getObjectExtractor().getBallExtractorLutFree();
	horizonY -= horizonY%16;

	uint8_t* rowDataStart = (uint8_t*)image->imageData;
	for (uint16_t y=0; y < image->height; y++, rowDataStart += image->widthStep) {
		uint8_t* imageData = rowDataStart;

		for(uint16_t x=0; x < image->width; x++, imageData += 3) {
			//image might be smaller than cameraImage, so use scale
			if (!ballExtractor.isBallPixelFast(x*scale,y*scale)){
				// set every non-ball-pixel to black
				*imageData = 0;
				*(imageData+1) = 0;
				*(imageData+2) = 0;
			}
		}
	}

	uint16_t topInImg = ballExtractor.getTopInPixels();
	uint16_t middleInImg = ballExtractor.getMiddleInPixels();

	int16_t centerX = Vision::getInstance().getImageCenterX();
	int16_t centerY = Vision::getInstance().getImageCenterY();
	uint16_t radius = Vision::getInstance().getImageF();

	// paint grid lines
	for(int y = horizonY; y < topInImg; y += GRID_CELL_SIZE) {
		uint16_t x_start, x_end;
		Vision::getInstance().getLineInCenterCircle(y,&x_start,&x_end);

		// horizontal line
		cvLine(image, cvPoint(x_start / scale, y / scale), cvPoint(x_end / scale, y / scale), gray, 1, CV_AA, 0);
	}

	for(int x = GRID_CELL_SIZE; x < Vision::getInstance().getImageWidth() - 1; x += GRID_CELL_SIZE) {
		int32_t distYtoCenter = radius*radius-(centerX-x)*(centerX-x);
		int16_t sqrt = (int16_t) FixedPointMath::isqrt(distYtoCenter);
		int16_t y_start = max(centerY - sqrt, horizonY);
		int16_t y_end   = min(centerY + sqrt, topInImg);

		if(y_start > y_end) continue;

		// vertical line
		cvLine(image, cvPoint(x / scale,  y_start / scale), cvPoint(x /scale, y_end / scale), gray, 1, CV_AA,0);
	}

	for(int y = topInImg; y < Vision::getInstance().getImageHeight() -1; y += GRID_CELL_SIZE) {
		uint16_t x_start, x_end;
		Vision::getInstance().getLineInCenterCircle(y,&x_start,&x_end);

		// horizontal line
		cvLine(image, cvPoint(x_start / scale, y / scale), cvPoint(x_end / scale, y / scale), gray, 1, CV_AA, 0);
	}

	for(int x = GRID_CELL_SIZE; x < Vision::getInstance().getImageWidth() - 1; x += GRID_CELL_SIZE) {
		int32_t distYtoCenter = radius*radius-(x-centerX)*(x-centerX);
		int16_t sqrt = (int16_t) FixedPointMath::isqrt(distYtoCenter);
		int16_t y_start = max(centerY - sqrt, topInImg);
		int16_t y_end = min(centerY + sqrt, Vision::getInstance().getImageHeight() - 1);

		if(y_start > y_end) continue;

		// vertical line
		cvLine(image, cvPoint(x / scale, y_start / scale), cvPoint(x / scale, y_end / scale), gray, 1, CV_AA,0);
	}

	// draw search spaces
	cvLine(image, cvPoint(0, topInImg / scale), cvPoint(Vision::getInstance().getImageWidth() /scale, topInImg / scale), yellow, 1, CV_AA,0);
	cvLine(image, cvPoint(0, middleInImg / scale), cvPoint(Vision::getInstance().getImageWidth() /scale, middleInImg / scale), yellow, 1, CV_AA,0);

	// draw center circle
	overlayCircle(image);

	// draw field contour
	FieldExtractor &fieldExtractor = Vision::getInstance().getFieldExtractor();
	for(uint16_t x = 0; x < Vision::getInstance().getImageWidth() - 1; ++x) {
		cvLine(image, cvPoint(x / scale, fieldExtractor.getHeighestFieldCoordinate(x) / scale),
				cvPoint((x+1) / scale, fieldExtractor.getHeighestFieldCoordinate(x + 1) / scale),
				green, 1, CV_AA, 0);
	}

	// paint non-fieldline edges
	GradientVectorGriding &grid = Vision::getInstance().getGVG();
	for(uint8_t y = 0; y < Vision::getInstance().getImageHeight() / GRID_CELL_SIZE; ++y) {
		for(uint8_t x = 0; x < Vision::getInstance().getImageWidth() / GRID_CELL_SIZE; ++x) {
			GridCell* cell = grid.getGridCell(x,y);

			// cell might be zero if we have got an image with wrong colors
			if(cell == 0) continue;

			Edge* edge = ballExtractor.getPossibleBallEdge(cell);
			if(edge != NULL) edge->paint(image,scale);
		}
	}

	std::map<CvPoint, uint8_t>::iterator mapIter;

	// draw cells which have been processed
	for(mapIter = ballExtractor.cells.begin(); mapIter != ballExtractor.cells.end(); mapIter++) {
	  cvRectangle(image,cvPoint(mapIter->first.x/scale,mapIter->first.y/scale),
			  cvPoint((mapIter->first.x+mapIter->second)/scale,
			  (mapIter->first.y+mapIter->second)/scale), magenta, 1, CV_AA, 0);
	}

	std::vector<PossibleBall> balls = ballExtractor.getPossibleBalls();
	std::vector<PossibleBall>::iterator iter = balls.begin();
	bool first = true;

	// paint all possible balls
	for(; iter != balls.end(); ++iter) {
		CvScalar color = cyan;

		if (first) {
			color = red;
			first = false;
		}

		PossibleBall ball = *iter;
		INFO("Ball (%d %d) %dx%d, hue %d, dist %d, error %d", ball.upperLeftX,
				ball.upperLeftY, ball.width, ball.height, ball.meanHue,
				ball.getRelativeDistance(), ball.error);

		cvRectangle(image, cvPoint(ball.upperLeftX/scale, ball.upperLeftY/scale),
				cvPoint((ball.upperLeftX+ball.width)/scale, (ball.upperLeftY+ball.height)/scale), color, 2, CV_AA, 0);

	}
}


/*------------------------------------------------------------------------------------------------*/

/**
 **
 **
 */

IplImage* ImagePresenter::createDistanceLutImage() {
	CriticalSectionLock lock(cs);

	IplImage *img = cvCreateImage(cvSize( (FieldOfPlay::A + 2*FieldOfPlay::J) / scale, (FieldOfPlay::B + 2*FieldOfPlay::J) / scale), IPL_DEPTH_8U, 3);

		for (int y = 0; y < img->height; ++y){
		for (int x=0; x < img->width; ++x){
			PositionAbsolute a(x - (FieldOfPlay::A/2 + FieldOfPlay::J)/scale, y - (FieldOfPlay::B/2 + FieldOfPlay::J)/scale);
			FieldOfPlay::LutEntry e = FieldOfPlay::lookUp(a);
			int dx = e.distanceToHorizontalLine - (img->width/2 - x);
			int dy = e.distanceToCircle - (img->height/2 - y);
			int distance = (int)round(sqrt(dx * dx + dy * dy));
			((uchar*)img->imageData + y * img->widthStep)[3*x+0] = distance;
			((uchar*)img->imageData + y * img->widthStep)[3*x+1] = distance;
			((uchar*)img->imageData + y * img->widthStep)[3*x+2] = distance;
		}
	}
	return img;
}

IplImage * ImagePresenter::createUndistortedImage() {
	CriticalSectionLock lock(cs);

	INFO("undistorted image");

	IplImage *undistortedImg = cvCreateImage(cvSize(1500, 1000), IPL_DEPTH_8U, 3);
	cvSetZero(undistortedImg);


	uint8_t r, g, b;
	for(int16_t y = 0; y < Vision::getInstance().getImageHeight(); ++y) {
		for(int16_t x = 0; x < Vision::getInstance().getImageWidth(); ++x) {

			((IMAGETYPE*) Vision::getInstance().getCameraImage())->getPixelAsRGB(x, y, &r, &g, &b);

			PositionImage undistPos = TheCameraModel::getInstance().undistortImagePoint(PositionImage(x, y));

			cvCircle(undistortedImg, cvPoint(undistPos.getX() - TheCameraModel::getInstance().centerX() + 750,
					undistPos.getY() - TheCameraModel::getInstance().centerY() + 500), 1, CV_RGB(r, g, b), 1, CV_AA, 0);
		}
	}

	return undistortedImg;
}

/**
 * @}
 */
