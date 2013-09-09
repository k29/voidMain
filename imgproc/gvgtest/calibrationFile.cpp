//#define RUN_IN_CAMERA_CALIBRATION

#include "calibrationFile.h"

#ifdef RUN_IN_CAMERA_CALIBRATION
#include "calibrationApp.h"
#endif

#include <zlib.h>


/*------------------------------------------------------------------------------------------------*/

/**
 **
 **
**/

CalibrationFile::CalibrationFile() {
}


/*------------------------------------------------------------------------------------------------*/

/**
 **
 **
**/

CalibrationFile::~CalibrationFile() {
}


/*------------------------------------------------------------------------------------------------*/

/**
**/

void CalibrationFile::init() {
#ifdef RUN_IN_CAMERA_CALIBRATION
	if (app->gui3d)
		app->gui3d->loadMarkerPoints(calibration);

	app->colorMgr.load(calibration);
#endif
}


/*------------------------------------------------------------------------------------------------*/

/**
**/

void CalibrationFile::printInfo() {
	printf("Calibration Info\n");
	if (calibration.has_filename())
		printf("\tOriginal file name: %s\n", calibration.filename().c_str());
	if (calibration.has_lastmodification())
		printf("\t Last modification: %s\n", calibration.lastmodstring().c_str());
	if (calibration.has_timeofdayinfo())
		printf("\t       Time of Day: %s\n", calibration.timeofdayinfo().c_str());
	if (calibration.has_location())
		printf("\t          Location: %s\n", calibration.location().c_str());
	printf("\t            Format: ");

	for (int i=0; i < calibration.lut_size(); i++) {
		if (calibration.lut(i).lutformat() == de::fumanoids::message::YUV)
			printf("YUV");
		else if (calibration.lut(i).lutformat() == de::fumanoids::message::RGB)
			printf("RGB");
		else
			printf("???");

		printf(" (%d bits)   ", calibration.lut(i).bpc());
	}
	printf("\n");

	if (calibration.has_notes() && calibration.notes().size() > 0) {
		printf("\n");
		printf("General notes:\n\t%s\n", calibration.notes().c_str());
	}
}


/*------------------------------------------------------------------------------------------------*/

/**
**/

void CalibrationFile::setModified(bool _modified) {
	modified = _modified;

#ifdef RUN_IN_CAMERA_CALIBRATION
	std::stringstream ss;

	if (modified)
		ss << "CameraCalibration - *" << filename;
	else
		ss << "CameraCalibration - " << filename;

	app->setTitle(ss.str());
#endif
}


/*------------------------------------------------------------------------------------------------*/

/**
**/

void CalibrationFile::getData(std::string *data) {
	calibration.SerializeToString(data);
}


/*------------------------------------------------------------------------------------------------*/

/** Load lookup table from file.
 **
 ** @param _filename  Name of file containing the binary lookup table data
**/

void CalibrationFile::load(const char* _filename) {
	printf("load from %s\n", _filename);
	filename = _filename;
	std::ifstream file(_filename, std::ios::in | std::ios::binary);
	if (false == file.fail()) {
		load(file);
	}
}

void CalibrationFile::load(void* data, uint32_t dataLength) {
	calibration.ParseFromArray(data, dataLength);
	setModified(false);
	finalizeLoad();
}

void CalibrationFile::load(std::istream &in) {
	calibration.ParseFromIstream(&in);
	filename = calibration.filename();
	printf("%d bytes loaded\n", calibration.ByteSize());
	setModified(false);
	finalizeLoad();
}

void CalibrationFile::finalizeLoad() {
#ifdef RUN_IN_CAMERA_CALIBRATION
	app->gui3d->loadMarkerPoints(calibration);
	app->colorMgr.load(calibration);
#endif
}


/*------------------------------------------------------------------------------------------------*/

/** Save calibration to file.
 **
 ** @param _filename  Name of file
**/

bool CalibrationFile::save(const char* _filename) {
	std::ofstream file(_filename, std::ios::out | std::ios::binary);
	return save(file);
}

bool CalibrationFile::save(std::ostream &out) {
	printf("saving calibration\n");
	if (false == out.fail()) {
#ifdef RUN_IN_CAMERA_CALIBRATION
		// add marker points to message
		app->gui3d->saveMarkerPoints(calibration);
#endif

		printf("saving %d bytes\n", calibration.ByteSize());
		printf("initialized: %s\n", calibration.IsInitialized() ? "yes" : "no");
		if (calibration.SerializeToOstream(&out)) {
			setModified(false);
			return true;
		}

		printf("Error saving calibration info to stream. Some field not set?");
	} else
		printf("Output stream for saving calibration has failed.");

	return false;
}


/*------------------------------------------------------------------------------------------------*/

/** Get size of calibration data
 **
 ** @return size of calibration data
**/

int32_t CalibrationFile::size() {
	return calibration.ByteSize();
}


/*------------------------------------------------------------------------------------------------*/

/**
 ** @return size of compressed data
**/

int32_t CalibrationFile::compress(void** compressedLUTPtr) {
	std::string data;
	calibration.SerializeToString(&data);

	uLongf uncompressedSize = size();
	uLongf compressedSize   = compressBound(uncompressedSize);

	*compressedLUTPtr = malloc(compressedSize);
	if (*compressedLUTPtr == 0)
		return -1;

	int res = ::compress2((Bytef*)*compressedLUTPtr, &compressedSize, (Bytef*)data.c_str(), uncompressedSize, Z_BEST_COMPRESSION);
	if (res != Z_OK) {
		free(*compressedLUTPtr);
		*compressedLUTPtr = 0;
		compressedSize = -1;
	}

	return compressedSize;
}


/*------------------------------------------------------------------------------------------------*/

/**
**/

bool CalibrationFile::uncompress(void* compressedLUT, uint32_t compressedLUTsize, uint32_t _uncompressedSize) {
	uLongf uncompressedSize = _uncompressedSize;
	Bytef* data = (Bytef*)malloc(uncompressedSize);
	if (data == 0)
		return false;

	int res = ::uncompress(data, &uncompressedSize, (Bytef*)compressedLUT, compressedLUTsize);
	load(data, uncompressedSize);
	return res == Z_OK;
}


/*------------------------------------------------------------------------------------------------*/

/**
 * Returns the magnitude threshold.
 * @return the magnitude threshold or -1 on error
**/

int32_t CalibrationFile::getMagnitudeThreshold() const {
	if (calibration.has_magnitudethreshold() && calibration.magnitudethreshold() > 0)
		return calibration.magnitudethreshold();
	else
		return -1;
}
