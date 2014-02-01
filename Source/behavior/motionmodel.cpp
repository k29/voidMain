#include "motionmodel.h"

MotionModel::MotionModel()
{
	position.x=0;
	position.y=0;
	confidence=0;
	// updated=0;
	
}

void MotionModel::refresh(double x,double y,double c)
{
	position.x=x;
	position.y=y;
	confidence=c;
}


AbsCoords MotionModel::read()
{
	return position;
}

void MotionModel::update(float r,float theta)
{
	decayMultiplier=exp(-r/decayConstant);
	double thetaWorld=double(theta+getImuAngle()); //imu.yaw is in degrress change it if needed

	double displacementX= r*cos(thetaWorld);
	double displacementY= r*sin(thetaWorld);

	position.x+=displacementX;
	position.y+=displacementY;
	confidence*=decayMultiplier;
}

void MotionModel::decay()
{
	confidence*=decayMultiplier;	
}
