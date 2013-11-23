#ifndef __IMU_H
#define __IMU_H
#include "cmt3.h"
<<<<<<< HEAD
#include "cmtdef.h"
#include "xsens_time.h"
#include "xsens_list.h"
#include <pthread.h>
#include "cmtscan.h"
=======
>>>>>>> 69f8829d20fbb6f84639f7d7262273b65cc18225
class Imu
{
private:
	bool initialized;
	xsens::Cmt2s serial;
	xsens::Message msg, reply;
	unsigned long msgCount;
public:
	Imu()
	{
		initialized = false;
		roll = 0.;
		pitch = 0.;
		yaw = 0.;
		msgCount = 0;
	};
	bool init();
	void update();
	double return_yaw();
	double roll, pitch, yaw;
};
#endif