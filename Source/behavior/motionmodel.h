/* MotionModel for segway-lite*/

#ifndef MOTION_MODEL_
#define MOTION_MODEL_

#include <cmath>
#include "../common/commondefs.h"
#include "../common/common.h"


/* C=C0 exp(-n/N) where n: number of times update has been called , N: decay constant */
#define decayConstant 50   

      
class MotionModel
{
	private:

		AbsCoords position;
		
		double decayMultiplier; 
	
	public:
		double confidence;

		MotionModel();
		
		void refresh(double x,double y,double c);


		AbsCoords read();

		void update(double r,double theta);

		void decay();

		/* temporary variables to accomodate localize.cpp */
		// int updated;
		// double r;
		// double theta;
		// double theta2;

};
extern MotionModel motionModel;
#endif