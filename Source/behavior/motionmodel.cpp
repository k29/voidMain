#include "motionmodel.h"

MotionModel::MotionModel()
{
	position.x=0;
	position.y=0;
	confidence=0;
	// updated=0;
	decayMultiplier=exp(-1.0/decayConstant);
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
	
	double thetaWorld=theta+imu.yaw;

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
