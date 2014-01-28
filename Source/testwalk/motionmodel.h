/* MotionModel for segway-lite*/


#include <cmath>
#include "../common/commondefs.h"


/* C=C0 exp(-n/N) where n: number of times update has been called , N: decay constant */
#define decayConstant 50   


class MotionModel
{
	private:

		AbsCoords position;
		double confidence;
		double decayMultiplier; 
	
	public:

		MotionModel();
		void refresh(double x,double y,double c);

		AbsCoords read();

		void update(float r,float theta);



};