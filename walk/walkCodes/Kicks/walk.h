#ifndef _WALK_H_
#define _WALK_H_

#include "AcYut.h"
#include "xsens/imu.h"

enum {SSP,DSP};

class Walk
{
	private:
	AcYut* bot;
	LEG leg;

	// Y Data //
	double legYin;
	double supLegYin;
	double veloY;
	
	// Z Data //
	double legZin;
	double supLegZin;
	double veloZ;
	double zMax;
	
	// Turn Data //
	double legRotin;
	double legRotfi;
	double supLegRotin;
	double supLegRotfi;
	
	// X Data //
	double lift;
	
	public:
	Walk(AcYut* bot);
	int kick();
	int dribble(float veloYin, float veloYfi, float veloZin, float veloZfi);
	LEG changeLeg();
	double getVeloY();
	double getVeloZ();
};




#endif
