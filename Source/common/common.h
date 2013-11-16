#ifndef COMMONN_H
#define COMMONN_H

#include <pthread.h>
#include "../walk/commondefswalk.h"
#include "../xsens/imu.h" 
#include "../gameController/gamecontrollerfunc.h"
#include "../gameController/gamecontrollerobj.h"
#include "opencv2/opencv.hpp"
#include "modes.h"
#define MYPORT "3838"

int getImuAngle();

void registerXABSL();
void start();
//extern Imu imu;

/* Threads */
extern pthread_t thread_id_walk;
extern pthread_t thread_id_gc;

/* Thread locks */
extern pthread_mutex_t mutex_walkflag;
//pthread_mutex_t mutex_encircleflag=PTHREAD_MUTEX_INITIALIZER;
extern pthread_mutex_t mutex_walkstr;
extern pthread_mutex_t mutex_bodycommand;
extern pthread_mutex_t mutex_changewalkflag;
extern pthread_mutex_t mutex_GCData;
extern pthread_mutex_t mutex_pathpacket;

extern WalkStructure walkstr;
extern WalkStructure prevwalkstr;

extern RoboCupGameControlData GCData;

/* Direct interface between path and walk. Updating this variable is controlled by behavior */ 
extern PathPacket pathpackvar;

extern Imu imu; 

extern int IMU_INITIAL_ANGLE;

#endif
