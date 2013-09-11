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

int main()
{
	Communication comm;
	AcYut bot(&comm);
	
	float t=5;
	int fps=60;
	
	float x,xi,xf;
	xi=390;
	xf=69.5;
	
	for(int i=0;i<t*fps;i++)
	{
		x=scurve(xi,xf,i,t*fps);
		
		bot.right_leg->move(x,0,0,0);
		bot.left_leg->move(x,0,0,0);
		comm.syncFlush();
		
		usleep(16666);
	}
	
	sleep(3);
	
	xi=69.5;
	xf=390;
	
	for(int i=0;i<t*fps;i++)
	{
		x=scurve(xi,xf,i,t*fps);
		
		bot.right_leg->move(x,0,0,0);
		bot.left_leg->move(x,0,0,0);
		comm.syncFlush();
		
		usleep(16666);
	}
}
