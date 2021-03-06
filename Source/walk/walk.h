#ifndef _WALK_H_
#define _WALK_H_

#include "AcYut.h"
#include "../xsens/imu.h"
#include <stdlib.h>
#include <cmath>
#include <fstream>
#include <ctime>
#include <deque>
#include <eigen3/Eigen/Dense>

using namespace std;
using namespace Eigen;
enum {SSP,DSP};

class Walk
{
	private:
	AcYut* bot;
	LEG leg;
//Physical Quantities
	double height;
	double feetSeparation;
	double hipLength;
	double Tc;
	double C;

	int hi ;
	int stepCount;
	double sspTimeVar;
	double dspTime;
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
	double correction_factor;
	// X Data //
	double lift;
	double com_offset[100];
	double com_offsety[100];
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
	deque<double> pos_q;
	// deque<double> imubuf_loc;
	double integ_term_y;
	double veldef;

	double prev_y;

	public:
	Walk(AcYut* bot);
	int kick();
	int dribble(int flag = 0);
	// int dribble_new(double dy, double dx,double t1,double t2);
	// int dribble(double dy, double dx,double t1,double t2);
	int pathdribble(double vel_y, double vel_z, double t1, double t2);
	int start();
	int captureStepZ(double &c1_z, double &c2_z, double zMax, double dsp1Time, double dsp2Time, double cap_pos_actual, double cap_vel_actual, double walkTime, double &sspTime, double &z_a_free, double &z_b_free, double &z_c_free, double &legZfi, double &supLegZfi);
	int captureStepY(double &c1_y, double &c2_y, double dsp1Time, double dsp2Time, double yr, double vely, double y,  double &supLegYfi, double &veloYfi_d, double cap_pos_actual, double cap_vel_actual, double walkTime, double &sspTime, double &y_a_free, double &y_b_free, double &y_c_free, double &y_d_free);
	int handMotion(double handSwing);
	int stopMotion();
	int sideMotion(double distance);
	int backMotion(double distance);
	int getLeg();
	// int start2();
	float accelerate();
	float velocity();
	float velocity2();
	float decelerate();
	float turnleft(float theta);
	float turnright(float theta);
	// int setStrafe(double l_strafe);
};




#endif
