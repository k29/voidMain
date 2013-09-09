/**
 **
 */

#ifndef _CALIBRATION_FILE_H_
#define _CALIBRATION_FILE_H_


#include "comm/protobuf/msg_calibration.pb.h"
#include <string>
#include <fstream>

class CalibrationFile {
public:
	CalibrationFile();
	virtual ~CalibrationFile();

	void init();
	void printInfo();

	void load(const char *filename);
	void load(void* data, uint32_t dataLength);
	void load(std::istream &in);
	bool save(const char* filename);
	bool save(std::ostream &out);
	void empty();

	int32_t size();
	void getData(std::string *data);

	int32_t compress(void** compressedLUTPtr);
	bool uncompress(void* compressedLUT, uint32_t compressedLUTsize, uint32_t _uncompressedSize);

	void setModified(bool modified=true);
	bool isModified() const { return modified; }
	std::string getName() const { return filename; }

	int32_t getMagnitudeThreshold() const;

	// TODO: make protected
	de::fumanoids::message::Calibration calibration;

protected:
	void finalizeLoad();

	std::string filename;
	bool modified;

};

#endif
