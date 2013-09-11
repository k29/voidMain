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
	Communication *comm=new Communication;
	AcYut bot(comm);
	
	float t=0.5;
	int fps=60;
	int sleep=16666;
	
	float x,xr,y,yr,z,zr,deltaz;
	
	for(int i=0;i<=t*fps;i++)
	{
		float c=i/(t*fps);
		
		x=390;
		xr=390;
		
		y=0;
		yr=0;
		
		deltaz=90*c*c*(3-2*c);
		
		z=deltaz;
		zr=-deltaz;
		
		printf("\n%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t",x,xr,y,yr,z,zr);
		
		bot.right_leg->move(x,y,z,0);
		bot.left_leg->move(xr,yr,zr,0);
		comm->syncFlush();
		
		usleep(sleep);
		
	}
	for(int i=0;i<=t*fps;i++)
	{
		float c=i/(t*fps);
		float m=(c-0.25)*2;
		
		x=390-20*(1+sin(3.14159*c-3.14159/2));
		xr=390;
		
		y=0;
		yr=0;
		
		
		if(c<0.25)
		{
			z=90;
			zr=-90;
		}
		else if(c>0.25 && c<0.75)
		{
			deltaz=30*m*m*(3-2*m);
		
			z=90+deltaz;
			zr=-90;//-deltaz/12;
		}
		else if(c>0.75)
		{
			z=120;
			zr=-90;
		}
		printf("\n%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t",x,xr,y,yr,z,zr);
		
		bot.right_leg->move(x,y,z,0);
		bot.left_leg->move(xr,yr,zr,0);
		comm->syncFlush();
		
		usleep(sleep);
		
	}
/*	for(int i=0;i<=t*fps;i++)
	{
		float c=i/(t*fps);
		
		x=390;
		xr=390;
		
		y=0;
		yr=0;
		
		deltaz=80*c*c*(3-2*c);
		
		z=115-deltaz;
		zr=-50+deltaz;
		
		printf("\n%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t",x,xr,y,yr,z,zr);
		
		bot.right_leg->move(x,y,z,0);
		bot.left_leg->move(xr,yr,zr,0);
		comm->syncFlush();
		
		usleep(sleep);
		
	}*/
}
