#include <stdio.h>
#include "leg.h"
#include "communication.h"
#include <math.h>
#include "AcYut.h"

float scurve(float in,float fi,float t, float tot)
{
	float frac=(float)t/(float)tot;
	if(frac>1)
	frac=1;
	float retfrac=frac*frac*(3-2*frac);
	float mval=in + (fi-in)*retfrac;
	return mval;
}

float linear(float in,float fi,float t, float tot)
{
	if(t>=0&&t<=tot)
	return in*(1-t/tot)+fi*(t/tot);
	else if(t<0)
	return in;
	else
	return fi;
}


void moveTo(Leg* leg, Communication &comm, float fx, float fy, float fz, float ftheta)
{
	float totalTime = 2;
	float fps = 60;
	float x = 390, y = 0, z = 0, theta = 0;
	for(float t = 0.0; t < totalTime; t += 1/fps)
	{
		x = scurve(390, fx, t, totalTime);
		y = scurve(0, fy, t, totalTime);
		z = scurve(0, fz, t, totalTime);
		theta = scurve(0, ftheta, t, totalTime);
		printf("Going to %f %f %f %f\n", x, y, z, theta);
		leg->move(x, y, z, theta);
		comm.syncFlush();
		usleep(1000000.0/fps);
	}
}

float xleft, yleft, zleft;
float xright, yright, zright;

void moveBoth(AcYut &bot, Communication &comm, float fx1, float fy1, float fz1, float fx2, float fy2, float fz2, float totalTime = 1)
{
	float fps = 60;
	float x1 = xleft, y1 = yleft, z1 = zleft;
	float x2 = xright, y2 = yright, z2 = zright;
	for(float t = 0.0; t < totalTime; t += 1/fps)
	{
		x1 = scurve(xleft, fx1, t, totalTime);
		y1 = scurve(yleft, fy1, t, totalTime);
		z1 = scurve(zleft, fz1, t, totalTime);
		printf("Left: Going to %f %f %f\n", x1, y1, z1);
		bot.left_leg->move(x1, y1, z1, 0);

		x2 = scurve(xright, fx2, t, totalTime);
		y2 = scurve(yright, fy2, t, totalTime);
		z2 = scurve(zright, fz2, t, totalTime);
		printf("Right: Going to %f %f %f\n", x2, y2, z2);
		bot.right_leg->move(x2, y2, z2, 0);
		comm.syncFlush();
		usleep(1000000.0/fps);
	}
	xleft = x1;
	xright = x2;
	yleft = y1;
	yright = y2;
	zleft = z1;
	zright = z2;
}


int main()
{
	Communication comm;
	AcYut bot(&comm);
	xleft = xright = 390;
	yleft = yright = zleft = zright = 0;
	// moveTo(bot.right_leg, comm, 390, 0, 20, 10);
	// moveTo(bot.left_leg, comm, 390, 0, 20, 10);
	// return 0;
	// bot.right_leg->move(390,10,0,0);
	// bot.left_leg->move(390,0,0,0);
	// comm.syncFlush();
	// moveTo(bot.left_leg, comm, 390, 0, -80);
	// moveTo(bot.right_leg, comm, 350, 40, 80);
	//zright + yright should always be greater than some number (50? 60?)
	moveBoth(bot, comm, xleft, yleft, 5, xright, yright, 5);

	double z = 25;
	double t = 0;
	double fps = 60;
	double x;
	double MUL = 6.1;
	double PI = 3.1415926535;
	double TP = 2*PI/MUL;
	// double xstart = TP*0.5*0.3;
	// double xend = TP*0.5*0.7;
		double xstart = TP*0.5*0.3333;
	double xend = TP*0.5*0.6666;

	double ystart = TP*0.5*0.4;
	double yend = TP*0.5*0.6;
	double y;

	double zamp = 25;
	while(1)
	{

		t = 0;
		while(t < TP)
		{
			z = zamp*sin(t*MUL);
			double thalf;
			double signMult;
			if(t < TP/2)
			{
				thalf = t;
				signMult = 1.0;
			}
			else
			{
				thalf = t - TP/2;
				signMult = -1.0;
			}

			if(thalf < xstart || thalf > xend)
				x = 0;
			else
			{
				x = 40*sin((thalf - xstart)*PI/((xend-xstart)))*signMult;
				// x = (20*sin(2*(thalf - xstart)*PI/((xend-xstart)) -PI/2) + 20)*signMult;
			}


			y = 15*sin((t+PI/2)*MUL);

		

			printf("t = %lf x = %lf\n", t, x);
			
			
			double x1, x2;
			if(x > 0)
			{
				x1 = 390 - x;
				x2 = 390;
			}
			else
			{
				x1 = 390;
				x2 = 390 +x;
			}
			bot.right_leg->move(x1, -y, z+5,0);
			bot.left_leg->move(x2, y, -z+5,0);
			// bot.right_leg->move(x1, -0, z+25,y);
			// bot.left_leg->move(x2, 0, -z+25,0);
			comm.syncFlush();
			usleep(1000000/fps);
			t = t + 1/fps;	
		}
		t = t - TP;
	}
	return 0;

	
}