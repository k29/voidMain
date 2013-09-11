/** @file
 **
 ** Base class for generic v4l2 camera support. Can be used
 ** standalone or refined/subclassed for individual cameras.
 **
 ** Based on Torsten's CameraConnection / GumCamera classes.
 */

#include "vision/image.h"
#include "camera_v4l2.h"
#include "comm.h"
#include "robot.h"

#include "timer.h"

#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <malloc.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <arpa/inet.h>


/*------------------------------------------------------------------------------------------------*/

// Number of video buffers. This is the number of buffers allocated, but usually one is not queued
// in v4l2 (currently processed by the vision) so a value of 1 is really not a good idea ;-)
#define REQUESTED_VIDEO_BUFFERS  3


/*------------------------------------------------------------------------------------------------*/

/**
 **
 **
 **/

CameraV4L2::CameraV4L2()
	: fd(-1)
	, buffers(NULL)
	, n_buffers(0)
	, bufferDequeued(false)
{
}

CameraV4L2::~CameraV4L2() {
	if (isOpen())
		closeCamera();
}


/*------------------------------------------------------------------------------------------------*/

/** Opens the connection to the camera.
 **
 ** @param deviceName            Name of device to use
 ** @param requestedImageWidth   Preferred width of image
 ** @param requestedImageHeight  Preferred height of image
 **
 ** @return true iff camera could be connected successfully
 **/

bool CameraV4L2::openCamera(const char* deviceName, uint16_t requestedImageWidth, uint16_t requestedImageHeight) {
//	if (strcmp(IMAGETYPENAME, "CameraImageYUV422") != 0) {
//		ERROR("Can not use a V4L2 camera with non-YUV422 image class! Adjust image.h!");
//		return false;
//	}

	if (false == openDevice(deviceName))
		return false;

//	determineControls();

	if (false == initDevice(requestedImageWidth, requestedImageHeight)) {
		uninitDevice();
		closeDevice();
		return false;
	}

	if (false == startCapturing()) {
		stopCapturing();
		uninitDevice();
		closeDevice();
		return false;
	}

	// give camera some time to settle before applying configuration settings
	delay(500);
	defaultConfiguration();

	return true;
}


/*------------------------------------------------------------------------------------------------*/

/** Closes the camera.
 **
 **/

void CameraV4L2::closeCamera() {
	stopCapturing();
	uninitDevice();
	closeDevice();
}


/*------------------------------------------------------------------------------------------------*/

/** Opens a v4l2 device.
 **
 ** @param deviceName   Name of device to open
 **
 ** @return true iff device could be opened
 **/

bool CameraV4L2::openDevice(const char* deviceName) {
	struct stat st;
	if ( -1 == stat(deviceName, &st)) {
		ERROR("Could not open camera device '%s' : %d, %s", deviceName, errno, strerror(errno));
		return false;
	}

	if (!S_ISCHR(st.st_mode)) {
		printf("%s is not a device\n", deviceName);
		return false;
	}

	fd = open(deviceName, O_RDWR | O_NONBLOCK, 0);
	if (-1 == fd) {
		printf("Cannot open %s: %d, %s\n", deviceName, errno, strerror(errno));
		return false;
	}

	return true;
}


/*------------------------------------------------------------------------------------------------*/

/** Closes the connection to the device.
 **
 **/

void CameraV4L2::closeDevice() {
	if (isOpen()) {
		if (-1 == close (this->fd)) {
			printf("Error %d closing camera\n", errno);
		}
		this->fd = -1;
	}
}


/*------------------------------------------------------------------------------------------------*/

/** Get pixel format.
 **
 ** @return pixel format to use
 */

uint32_t CameraV4L2::getPixelFormat() {
		return V4L2_PIX_FMT_YUYV;
}


/*------------------------------------------------------------------------------------------------*/

/** Create a camera image object of the appropriate type (rf. getPixelFormat())
 **
 ** @return camera image object of the appropriate type
 */

CameraImage* CameraV4L2::createImage() {
	return new IMAGETYPE(imageWidth, imageHeight);
}


/*------------------------------------------------------------------------------------------------*/

/** Initialize camera device
 **
 ** @return true iff camera could be initialized successfully
 */

bool CameraV4L2::initDevice(uint16_t requestedImageWidth, uint16_t requestedImageHeight) {
	struct v4l2_capability cap;
	struct v4l2_cropcap    cropcap;
	struct v4l2_crop       crop;

	// query capabilities
	if (-1 == xioctl(VIDIOC_QUERYCAP, &cap)) {
		if (errno == EINVAL)
			printf("not a v4l2 device\n");
		else
			printf("error %d on VIDIOC_QUERYCAP\n", errno);

		return false;
	}

	if ( !(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE) ) {
		printf("not a video capture device\n");
		return false;
	}

//	if ( !(cap.capabilities & V4L2_CAP_S) ) {
//		printf("does not support streaming i/o\n");
//		return false;
//	}

	// query the video cropping and scaling abilities
	CLEAR(cropcap);
	cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	if (0 == xioctl(VIDIOC_CROPCAP, &cropcap)) {

		// reset cropping rectangle to default
		crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		crop.c = cropcap.defrect; /* reset to default */
		if (-1 == xioctl(VIDIOC_S_CROP, &crop)) {
			switch (errno) {
			case EINVAL:
				/* Cropping not supported. */
				break;
			default:
				/* Errors ignored. */
				break;
			}
		}
	} else {
		/* Errors ignored. */
	}

	// set dimensions of captured images
	struct v4l2_format fmt;
	CLEAR (fmt);
	fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	fmt.fmt.pix.width       = requestedImageWidth;
	fmt.fmt.pix.height      = requestedImageHeight;
	fmt.fmt.pix.pixelformat = getPixelFormat();
	fmt.fmt.pix.field       = V4L2_FIELD_ANY;

	if (-1 == xioctl (VIDIOC_S_FMT, &fmt)) {
		ERROR("Can't set image size (%d, %d), format 0x%x, vision disabled.", requestedImageWidth, requestedImageHeight, getPixelFormat());
		perror("VIDIOC_S_FMT");
		return false;
	}

	// the camera may not support the requested dimensions, so we
	// query what it is really going to use
	struct v4l2_format fmtCheck;
	CLEAR(fmtCheck);
	fmtCheck.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	if (-1 == xioctl (VIDIOC_G_FMT, &fmtCheck)) {
		ERROR("Can't query image size, vision disabled.");
		perror("VIDIOC_G_FMT");
		return false;
	} else {
		imageWidth  = fmtCheck.fmt.pix.width;
		imageHeight = fmtCheck.fmt.pix.height;
	}

	if (fmtCheck.fmt.pix.pixelformat != getPixelFormat())
		WARNING("Wrong pixel format (got %d but expected %d)", fmtCheck.fmt.pix.pixelformat, getPixelFormat());

	image = createImage();

	return init_mmap();
}


/*------------------------------------------------------------------------------------------------*/

/** Memory Map initialization
 **
 */

bool CameraV4L2::init_mmap() {
	struct v4l2_requestbuffers req;
	CLEAR (req);
	req.count = REQUESTED_VIDEO_BUFFERS;
	req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	req.memory = V4L2_MEMORY_MMAP;

	if (-1 == xioctl(VIDIOC_REQBUFS, &req)) {
		if (EINVAL == errno)
			printf("does not support memory mapping\n");
		else
			printf("error %d in VIDIOC_REQBUFS\n", errno);

		return false;
	}

	buffers = (Buffer*)calloc(req.count, sizeof(*buffers));
	if (NULL == buffers) {
		printf("Out of memory (allocating %d bytes)\n", (uint32_t)(req.count * sizeof(*buffers)));
		return false;
	}

	for (n_buffers = 0; n_buffers < req.count; ++n_buffers) {
		struct v4l2_buffer buf;

		CLEAR (buf);
		buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buf.memory = V4L2_MEMORY_MMAP;
		buf.index = n_buffers;

		if (-1 == xioctl( VIDIOC_QUERYBUF, &buf)) {
			printf("error %d in VIDIOC_QUERYBUF\n", errno);
			return false;
		}

		buffers[n_buffers].length = buf.length;
		buffers[n_buffers].start  = mmap ( NULL                    // start anywhere
		                                 , buf.length
		                                 , PROT_READ | PROT_WRITE  // required
		                                 , MAP_SHARED              // recommended
		                                 , fd
		                                 , buf.m.offset);

		if (MAP_FAILED == buffers[n_buffers].start) {
			printf("error memory mapping buffer %d\n", n_buffers);
			return false;
		}
	}

	return true;
}


/*------------------------------------------------------------------------------------------------*/

/**
 **
 **
 **/

void CameraV4L2::uninitDevice() {
	for (unsigned int i = 0; i < n_buffers; ++i) {
		if (-1 == munmap(buffers[i].start, buffers[i].length)) {
			printf("error unmapping buffer %d\n", i);
		}
	}

	free(buffers);
	buffers   = 0;
	n_buffers = 0;
}


/*------------------------------------------------------------------------------------------------*/

/** Checks whether connection to camera is established.
 **
 ** @return true iff camera is connected/on/open
 **/

bool CameraV4L2::isOpen() {
	return (fd != -1);
}


/*------------------------------------------------------------------------------------------------*/

/** Queries a device parameter until a response or error is returned.
 **
 ** @param request  Parameter to query
 ** @param arg      Value to set / get
 **
 ** @return return value of query call
 **/

int CameraV4L2::xioctl(unsigned long int request, void* arg) {
	int r = 0;

	do {
		r = ioctl(fd, request, arg);
	} while ( r == -1 && EINTR == errno );

	return r;
}


/*------------------------------------------------------------------------------------------------*/

/**
 ** Preparing the image capture functions
 */

bool CameraV4L2::startCapturing() {
	enum v4l2_buf_type type;

	for (uint8_t i = 0; i < n_buffers; i++) {
		struct v4l2_buffer buf;
		CLEAR (buf);
		buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buf.memory = V4L2_MEMORY_MMAP;
		buf.index  = i;

		if (-1 == xioctl(VIDIOC_QBUF, &buf)) {
			printf("error %d in VIDIOC_QBUF\n", errno);
			return false;
		}
	}

	type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	if (-1 == xioctl (VIDIOC_STREAMON, &type)) {
		printf("error %d in VIDIOC_STREAMON\n", errno);
		return false;
	}

	return true;
}


/*------------------------------------------------------------------------------------------------*/

/**
 **
 */

void CameraV4L2::stopCapturing() {
	enum v4l2_buf_type type;
	type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	if (-1 == xioctl(VIDIOC_STREAMOFF, &type))
		printf("error %d in VIDIOC_STREAMOFF\n", errno);
}


/*------------------------------------------------------------------------------------------------*/

/** Start frame reading. This function must be called from external classes to do frame update.
 **
 */

bool CameraV4L2::capture() {
//	robottime_t start = getCurrentTime();
	while (isOpen()) {
		fd_set fds;
		struct timeval tv;

		FD_ZERO (&fds);
		FD_SET (fd, &fds);

		tv.tv_sec  = 1;
		tv.tv_usec = 0;

		int r = select(fd + 1, &fds, NULL, NULL, &tv);
		if (-1 == r) {
			if (EINTR == errno)
				continue;

			printf("Error %d in select\n", errno);
			return false;
		}

		if (0 == r) {
			printf("select timeout, frame lost\n");
			return false;
		}

		if (readFrame()) {
			//	printf("waited %d ms for frame\n", (int)(getCurrentTime()-start));
			totalFrames++;
			return true;
		}
	}

	return false;
}


/*------------------------------------------------------------------------------------------------*/

/**
 **
 */

bool CameraV4L2::readFrame() {
	if (bufferDequeued) {
		if (-1 == xioctl (VIDIOC_QBUF, &dequeuedBuffer))
			ERROR("Error %d (%s) queueing buffer (VIDIOC_QBUF)", errno, strerror(errno));

		bufferDequeued = false;
	}

	CLEAR (dequeuedBuffer);

	// dequeue buffer
	dequeuedBuffer.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	dequeuedBuffer.memory = V4L2_MEMORY_MMAP;
	if (-1 == xioctl(VIDIOC_DQBUF, &dequeuedBuffer)) {
		switch (errno) {
			case EAGAIN:
				return false;

			case ENODEV:
				ERROR("Camera device became unplugged.");
				closeCamera();
				return false;

			case EIO:
				/* Could ignore EIO, see spe
				 *
				 *fall through */

			default:
				printf("Error %d reading frame / VIDIOC_DQBUF\n", errno);
				return false;
		}
	}

	bufferDequeued = true;
	assert (dequeuedBuffer.index < n_buffers);
	image->setImage(buffers[dequeuedBuffer.index].start, buffers[dequeuedBuffer.index].length);
//	printf("buffer %d\n", 	dequeuedBuffer.index);
	return true;
}


/*------------------------------------------------------------------------------------------------*/

/** Sets a camera setting.
 **
 ** @param setting  Setting to set
 ** @param value    Value to set
 */

void CameraV4L2::setSetting(CAMERA_SETTING setting, int32_t value) {
	int16_t index = getSettingIndex(setting);
	if (index == -1)
		return; // not supported

	struct v4l2_queryctrl queryctrl;
	struct v4l2_control   control;

	memset (&queryctrl, 0, sizeof (queryctrl));
	queryctrl.id = supportedSettings[index].id;

	if (-1 == xioctl(VIDIOC_QUERYCTRL, &queryctrl)) {
		if (errno != EINVAL)
			printf("error %d (%s) in VIDIOC_QUERYCTRL for %s\n", errno, strerror(errno), supportedSettings[index].name);
		else
			printf("%s is not supported\n", supportedSettings[index].name);
	} else if (queryctrl.flags & V4L2_CTRL_FLAG_DISABLED) {
		printf("%s is disabled\n", supportedSettings[index].name);
	} else {
		memset(&control, 0, sizeof(control));
		control.id    = supportedSettings[index].id;
		control.value = value;
		if (-1 == xioctl(VIDIOC_S_CTRL, &control)) {
			printf("error %d (%s) setting %s\n", errno, strerror(errno), supportedSettings[index].name);
		} else {
			supportedSettings[index].currentValue = control.value; // TODO: read value back from camera
			printf("%s set to %d (0x%x)\n", supportedSettings[index].name, value, value);
		}
	}
}


/*------------------------------------------------------------------------------------------------*/

/** Gets a camera setting. Should only be called for settings where supports() returned true.
 **
 ** @param setting  Setting to get
 ** @return value of setting, -1 on error
 */
/*
int32_t CameraV4L2::getSetting(CAMERA_SETTING setting) {
	int16_t index = getSettingIndex(setting);
	if (index == -1) {
		return -1; // not supported
	}

	struct v4l2_queryctrl queryctrl;
	struct v4l2_control   control;

	memset(&queryctrl, 0, sizeof (queryctrl));
	queryctrl.id = supportedSettings[index].id;

	if (-1 == xioctl (VIDIOC_QUERYCTRL, &queryctrl)) {
		if (errno != EINVAL)
			printf("error %d (%s) in VIDIOC_QUERYCTRL for %s\n", errno, strerror(errno), supportedSettings[index].name);
		else
			printf("%s is not supported\n", supportedSettings[index].name);
	} else if (queryctrl.flags & V4L2_CTRL_FLAG_DISABLED) {
		printf("%s is disabled\n", supportedSettings[index].name);
	} else {
		// printf("min: %d, max: %d, step %d\n", queryctrl.minimum, queryctrl.maximum, queryctrl.step);

		memset (&control, 0, sizeof (control));
		control.value = 15;
		control.id = supportedSettings[index].id;
		if (-1 == xioctl(VIDIOC_G_CTRL, &control)) {
			printf("error %d (%s) getting %s\n", errno, strerror(errno), supportedSettings[index].name);
		} else {
			printf("%s (%x) read as %d\n", supportedSettings[index].name, supportedSettings[index].id, control.value);
			return control.value;
		}
	}

	return -1;
}
*/


/*------------------------------------------------------------------------------------------------*/

/** Enumerate all available controls
 **
 ** @param setting  Setting to get
 ** @return value of setting, -1 on error
 */

void CameraV4L2::determineControls(uint32_t minID, uint32_t maxID) {
	struct v4l2_queryctrl queryctrl = {0};
	struct v4l2_querymenu querymenu = {0};

	for (queryctrl.id = minID; queryctrl.id < maxID; queryctrl.id++) {
		if (0 == ioctl(fd, VIDIOC_QUERYCTRL, &queryctrl)) {
			if (queryctrl.flags & V4L2_CTRL_FLAG_DISABLED)
				continue;

			printf(" Control %s\t(code: %x, val: %d, min: %d, max: %d)\n", queryctrl.name, queryctrl.id, queryctrl.default_value, queryctrl.minimum, queryctrl.maximum);

			if (queryctrl.type == V4L2_CTRL_TYPE_MENU) {
				printf("  Menu items:\n");

				memset(&querymenu, 0, sizeof(querymenu));
				querymenu.id = queryctrl.id;

				for (querymenu.index = (__u32)queryctrl.minimum; querymenu.index <= (__u32)queryctrl.maximum; querymenu.index++) {
					if (0 == ioctl(fd, VIDIOC_QUERYMENU, &querymenu)) {
						printf("  %s\n", querymenu.name);
					} else {
						perror("VIDIOC_QUERYMENU");
						exit(EXIT_FAILURE);
					}
				}
			}
		} else {
			if (errno == EINVAL)
				continue;

			perror("VIDIOC_QUERYCTRL");
			exit(EXIT_FAILURE);
		}
	}
}

void CameraV4L2::determineControls() {
	printf("\n==== Parameters supported by camera ====\n");
	determineControls(V4L2_CID_BASE, V4L2_CID_LASTP1);
	determineControls(V4L2_CID_PRIVATE_BASE, V4L2_CID_PRIVATE_BASE+100);
	printf("========================================\n\n");
}
