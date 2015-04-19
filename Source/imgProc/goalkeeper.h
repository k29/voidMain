#ifndef GOALKEEPER_H
#define GOALKEEPER_H

#include "../common/commondefs.h"
#include "../common/defines.h"
#include "imgproc.h"
#include "featuredetection.h"
#include "camcapture.h"
#include "headmotor.h"
#include "camcontrol.h"
#include <cstdio>
#include <cmath>
#include <cstdlib>

class GoalKeeper
{
public:
	static const double ACTIVATION_RADIUS = 120.0;
	static const double GOAL_WIDTH = 100.0;
	static const double CRITICAL_RADIUS = 40.0;
	static const double ACTIVATION_THRESHOLD = 20.0;
	GoalKeeper();
	// ~GoalKeeper();
	GoalKeeperAction keeperUpdate(CamCapture &cam, HeadMotor &hm, CamControl* camcont, FeatureDetection* fd, MotionModel &mm);
};

#endif