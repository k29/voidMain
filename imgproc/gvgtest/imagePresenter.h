/**
 ** @{
 ** @ingroup vision
 **
 **
 */

#ifndef __IMAGEPRESENTER_H_
#define __IMAGEPRESENTER_H_

#include "camera/camera_image.h"
#include "transport/transport.h"
#include "events.h"
#include "thread.h"
#include "timer.h"
#include "position.h"

#include <string>


/*------------------------------------------------------------------------------------------------*/

typedef enum {
	IMAGEOVERLAY_NORMAL,       //!< no overlays
	IMAGEOVERLAY_OBJECTS,      //!< overlay found objects
	IMAGEOVERLAY_EDGES,        //!< overlay edges
	IMAGEOVERLAY_PROJECTION,   //!< overlay projected colors
	IMAGEOVERLAY_LINEPOINTS,   //!< overlay line points
	IMAGEOVERLAY_GRADIENT,     //!< gradient image
	IMAGEOVERLAY_FIELDCONTOUR, //!< field contour
	IMAGEOVERLAY_GRID,         //!< grid
	IMAGEOVERLAY_BALL,         //!< overlay ball (lut free)
	IMAGEOVERLAY_TEST          //!< overlay for different test purpose
	, IMAGEOVERLAY_MAX         // keep this the last entry as it is used to loop over the enum!!!
} ImageOverlayModus;

typedef enum {
	IP_MODE_NORMAL,
	IP_MODE_WORLDMAP,
	IP_MODE_LOCALIZATION,
	IP_MODE_PROJECTED,
	IP_MODE_DISTANCE_LUT,
	IP_MODE_UNDISTORTED
} ImagePresentationMode;

typedef enum {
	IP_ACTION_IDLE,
	IP_ACTION_DISPLAY,
	IP_ACTION_SAVE,
	IP_ACTION_STREAM
} ImagePresentationAction;


/*------------------------------------------------------------------------------------------------*/

/**
 * The ImagePresenter class is responsible for showing or saving images.
 * @ingroup vision
 */
class ImagePresenter : public EventCallback, protected Thread {
public:
	ImagePresenter(int _scale=1, ImageOverlayModus _overlay=IMAGEOVERLAY_NORMAL, ImagePresentationMode _mode=IP_MODE_NORMAL);
	virtual ~ImagePresenter();
    const virtual char *getName()
    {
        return "ImagePresenter";
    }

    // functions for display mode
    void startWorldMapDisplay();
    void startProjectedDisplay();
    void startLocalizationDisplay();
    void startDisplay(const char *windowName, uint16_t minIntervalInMilliseconds = 200, int32_t closingTimeout = -1);
    void startRotation(const char *windowName, uint16_t rotationTimeInSeconds = 10, uint16_t minIntervalInMilliseconds = 200, int32_t closingTimeout = -1);
    void waitForWindowClose();
    void stopDisplay();
    // functions for save mode
    void saveImage(const char *basename, const char *extension = "png", int32_t count = 1, uint16_t minIntervalInMillisconds = 0, bool timestamp = false);
    void stopSaving();
    void waitForSavingDone();
    // functions for sending images elsewhere
    void startStreaming(int32_t count = -1, uint16_t minIntervalInMilliseconds = 250, Transport *transport = 0);
    void waitForStreamingDone();
    void stopStreaming();
    // general functions to influence image processing
    void changeScale(int _scale)
    {
        scale = _scale;
    }

    void changeModus(ImageOverlayModus _overlay)
    {
        overlay = _overlay;
    }

    // event callback function
    virtual void eventCallback(EventType evtType, void *imageVoidPtr);
protected:
    CriticalSection cs;
    CameraImage *cameraImage;
    IplImage *image;
    int scale;
    ImageOverlayModus overlay;
    ImagePresentationMode mode;
    ImagePresentationAction action;
    CvFont font;
    bool displayTime;
    bool displayFileName;

    // support for rotating through the different display modes
    bool rotateMode;
    int32_t rotateModeTimeout;
    robottime_t lastRotation;

    std::string fileBaseName;          //!< base file name
    std::string fileExtension;         //!< file type / file name extension
    int32_t count;                     //!< number of files to save
    int32_t fileMaxCount;              //!< how many files to save at most
    bool fileWithTimeStamp;            //!< whether to put a timestamp in the filename
    ContinousEvent savingFinished;
    ContinousEvent streamingFinished;
    Transport *transport;
    std::string windowTitle;
    int32_t closingTimeout;
    robottime_t lastImageTime;
    int32_t timeout;
    virtual void threadMain();
    void saveImage();
    virtual void streamImageToFile();
    virtual void streamImageToNetwork();

    void overlayRegions();
    void overlayObjects();
    void overlayFieldContour();
    void overlayFieldLines();
    void overlayEdges();
    void overlayGrid();
    void overlayBall();
    void overlayTest();

    void overlayFeetAndObstacles(bool feet = true, bool obstacles = true);
    void overlayHorizon(IplImage *img);
    void overlayCircle(IplImage *img);

    void printKeyboardHelp();

    IplImage *createFieldImage();
    IplImage *createWorldMapImage();
    IplImage *createLocalizationImage();
    IplImage *createProjectedImage();
    IplImage *createDistanceLutImage();
    IplImage *createUndistortedImage();
};

/**
 * @}
 */

#endif
