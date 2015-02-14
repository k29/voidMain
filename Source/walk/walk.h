#ifndef _WALK_H_
#define _WALK_H_

#include "AcYut.h"
#include "../xsens/imu.h"
#include <stdlib.h>
#include <cmath>


enum {SSP,DSP};

class Walk
{
	private:
	AcYut* bot;
	LEG leg;
	int hi ;
	int stepCount;
	double sspTimeVar;
	// Y Data //
	double legYin;
	double supLegYin;
	double veloYin;
	double veloYfi;
	double veloYfi_d;
	// Z Data //
	double legZin;
	double supLegZin;
	double veloZin;
	double veloZfi;
	double zMax;
	double dz;
	// double strafe;
	// double sspZAmp;
	
	// Turn Data //
	double legRotin;
	double legRotfi;
	double supLegRotin;
	double supLegRotfi;
	
	// X Data //
	double lift;
	
	//PID Constants
	double integ_const_z;
	double deriv_const_z;
	double prop_const_z;
	double integ_max_z;	
	double prev_mean_z;
	double integ_term_z;

	double integ_const_y;
	double deriv_const_y;
	double prop_const_y;
	double integ_max_y;
	double mean_y;
	double prev_mean_y;
	double prev_imu_yaw;
	double integ_term_y;

	double prev_y;
	public:
	Walk(AcYut* bot);
	int kick();
	int dribble();
	int dribble_new(double dy, double dx,double t1,double t2);
	int dribble(double dy, double dx,double t1,double t2);
	int pathdribble(double vel_y, double dx, double t1, double t2);
	int start();
	int start2();
	float accelerate();
	float velocity();
	float velocity2();
	float decelerate();
	float turnleft(float theta);
	float turnright(float theta);
	// int setStrafe(double l_strafe);
};




#endif
