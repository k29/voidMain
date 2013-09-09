#include "macroMagic.h"

#ifndef IMAGE_H_
#define IMAGE_H_

#ifndef IMAGETYPE
	#if defined IMAGEFORMAT_YUV422
		#define IMAGETYPE      CameraImageYUV422
		#define IMAGETYPEFULL  CameraImageYUV422 // TODO: should be YUV
		class CameraImageYUV422;
		#include "camera/camera_imageYUV422.h"
	#elif defined IMAGEFORMAT_BAYER
		#define IMAGETYPE     CameraImageBayer
		#define IMAGETYPEFULL CameraImageRGB
		class CameraImageBayer;
		class CameraImageRGB;
		#include "camera/camera_imageBayer.h"
		#include "camera/camera_imageRGB.h"
	#elif defined IMAGEFORMAT_RGB
		#define IMAGETYPE     CameraImageRGB
		#define IMAGETYPEFULL CameraImageRGB
		class CameraImageRGB;
		#include "camera/camera_imageRGB.h"
	#endif

	#define IMAGETYPENAME STRINGIFY(IMAGETYPE)
#endif

#endif
