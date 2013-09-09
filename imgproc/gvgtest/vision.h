#ifndef __VISION_H__
#define __VISION_H__

#include "camera/camera.h"
#include "camera/camera_image.h"
#include "calibrationFile.h"
#include "gvg.h"
#include "comm.h"
#include "thread.h"
#include "object.h"
#include "objectExtractor_field.h"
#include "objectExtractor.h"
#include "fieldColorExtractor.h"

#include "singleton.h"

#include "camera/theCameraModel.h"

#include "comm/protobuf/msg_vision.pb.h"

#include <vector>

/** @defgroup vision The Vision
 *
 * The Vision is responsible for managing the whole vision subsystem.
 * In this module, the low-level image processing as well as the object extraction is done.
 *
 * The vision module has a hierarchy-structure with three layers:
 *  - Low-level image processing: Analyze gradient image -> see GradientVectorGriding and CameraImage classes
 *  - %Edge Extraction: Combine the image gradients to long edges -> see GradientVectorGriding class
 *  - Object extraction: Extract the object information out of the found edges -> see ObjectExtractor and Extractor classes
 *
 * @ingroup core
 * @author Naja von Schmude, Lisa Dohrmann
 *
 * @{
 */

#define VISION_DEBUG false // be sure to switch this to "false" before a game!

/*------------------------------------------------------------------------------------------------*/

class ColorMgr;


/*------------------------------------------------------------------------------------------------*/

class ImagePresenter;

/**
 * Main vision class is responsible for receiving events of new images,
 * starting all vision sub-modules and informing the worldmodel.
 *
 * @ingroup vision
 */
class Vision : public Singleton<Vision>, public Thread, public OperationCallback {
public:
	virtual ~Vision();

	bool init();

	virtual const char* getName() {
		return "Vision";
	}

	inline IplImage* getImage(double scale=1.0) {
		return image->getImageAsRGB(scale);
	}

	inline CameraImage* getCameraImage() {
		return image;
	}

	inline std::string getCameraType() {
		return cameraType;
	}

	inline Camera* getCamera() {
		return cam;
	}

	inline ColorManager* getColorMgr() { return colorMgr; }

	inline uint16_t getImageCenterX() const { return TheCameraModel::getInstance().centerX();       }
	inline uint16_t getImageCenterY() const { return TheCameraModel::getInstance().centerY();       }
	inline uint16_t getImageF()       const { return TheCameraModel::getInstance().focalLengthX();  }

	inline uint16_t getImageWidth() const {
		if (cam == 0)
			return 640;
		return cam->getImageWidth();
	}
	inline uint16_t getImageHeight() const {
		if (cam == 0)
			return 480;
		return cam->getImageHeight();
	}

	virtual bool operationCallback(
				OPERATION operation,
				uint8_t   flags,
				uint8_t  *data,
				uint16_t  dataLen,
				struct sockaddr_in *remoteAddress);

	void getCalibration(std::string *data);
	void saveConfiguration();

	inline GradientVectorGriding& getGVG() {
		return gvg;
	}
	inline FieldExtractor& getFieldExtractor() {
		return fieldExtractor;
	}
	inline ObjectExtractor& getObjectExtractor() {
		return objectExtractor;
	}

	/**
	 * Returns the start and end points of a line at the height of y
	 * starting and ending on the center circle.
	 * @param y - height of line in the image
	 * @param lineStart - will be set to the x coordinate that lies on the center circle
	 * @param lineEnd   - will be set to the x coordinate that lies on the center circle
	 */
	inline void getLineInCenterCircle(uint16_t y, uint16_t* lineStart, uint16_t* lineEnd) {
		uint16_t centerX = getImageCenterX();
		uint16_t centerY = getImageCenterY();
		uint16_t radius = getImageF();

		int32_t distXtoCenter = radius * radius - (y-centerY) * (y-centerY);
		int16_t sqrt = (int16_t) FixedPointMath::isqrt(distXtoCenter);
		*lineStart = max(centerX - sqrt, 0);
		*lineEnd   = min(centerX + sqrt, getImageWidth()-1);
	}

	ContinousEvent initialized;

	CalibrationFile calibration;

	int activeFieldColorExtractor;

	de::fumanoids::message::LocalizationInput dataForLocalization;

protected:
	CriticalSection cs;

	virtual void threadMain();
	void threadInit();
	bool initCamera();

	void handleImageSaving();

	friend class TestVision;

private:
	Camera         *cam;           //!< Camera object
	CameraImage    *image;         //!< current image
	ColorManager   *colorMgr;      //!< color manager holding the calibration
	uint32_t		frameCounter;  //!< counts the number of frames beeing processed since the beginning
	std::string 	cameraType;    //!< holds the type of the camera e.g. quickcam, offline etc.

	/// the class responsible to stream images
	ImagePresenter *imageStreamer;

	/// GVG algorithm to find the edges
	GradientVectorGriding gvg;

	/// Main object extractor. Calls all the specific object extractors like ball extractor etc.
	ObjectExtractor objectExtractor;

	/// %Extractor of the field contour
	FieldExtractor fieldExtractor;

	Vision();
	void pushDataIntoWorldmodel();
	friend class Singleton<Vision>;

	friend class ImagePresenter;

	inline EdgeVector& getEdges() {
		return gvg.getEdges();
	}

	friend class CommTCP;
	void sendImageData(Transport *transport, bool compress, bool leadingSize=true);
};

/**
 * @}
 */
#endif /* __VISION_H__ */
