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
	
	float t=0.5;
	int fps=60;
	float x,y,z,xr,yr,zr;
	float c;
		
	for(int i=0;i<t*fps;i++)
	{
		c=(float)((float)i/((float)t*fps));	
		printf("%f",c);
			
		x=390-30*2*c*(2-2*c);
		xr=390;
		
		y=scurve(0,30,i,t*fps);
		yr=-y;
		
		if(c<0.25)
		{
			z=70*(4*c)*(4*c)*(3-8*c);
			zr=-z;
			z=1.6*z;
		}
		else if(c>0.75)
		{
			z=70*(1-(4*(c-0.75))*(4*(c-0.75))*(3-8*(c-0.75)));
			zr=-z;
			z=1.6*z;
		}
		else
		{
			z=70;
			zr=-70;
			z=1.6*z;
		}
		
		printf("\n%lf\t%lf\t%lf\t%lf\t%lf\t%lf",x,xr,y,yr,z,zr);
		
		bot.right_leg->move(x,y,z,0);
		bot.left_leg->move(xr,yr,zr,0);
		comm.syncFlush();
		
		usleep(33333);
	}
	for(int i=0;i<t*fps;i++)
	{
		c=(float)((float)i/((float)t*fps));	
		printf("%f",c);
			
		x=390-30*2*c*(2-2*c);
		xr=390;
		
		y=scurve(-30,30,i,t*fps);
		yr=-y;
		
		if(c<0.25)
		{
			z=70*(4*c)*(4*c)*(3-8*c);
			zr=-z;
			z=1.6*z;
		}
		else if(c>0.75)
		{
			z=70*(1-(4*(c-0.75))*(4*(c-0.75))*(3-8*(c-0.75)));
			zr=-z;
			z=1.6*z;
		}
		else
		{
			z=70;
			zr=-70;
			z=1.6*z;
		}
		
		printf("\n%lf\t%lf\t%lf\t%lf\t%lf\t%lf",x,xr,y,yr,z,zr);
		
		bot.left_leg->move(x,y,z,0);
		bot.right_leg->move(xr,yr,zr,0);
		comm.syncFlush();
		
		usleep(33333);
	}
	for(int i=0;i<t*fps;i++)
	{
		c=(float)((float)i/((float)t*fps));	
		printf("%f",c);
			
		x=390-30*2*c*(2-2*c);
		xr=390;
		
		y=scurve(-30,30,i,t*fps);
		yr=-y;
		
		if(c<0.25)
		{
			z=70*(4*c)*(4*c)*(3-8*c);
			zr=-z;
			z=1.6*z;
		}
		else if(c>0.75)
		{
			z=70*(1-(4*(c-0.75))*(4*(c-0.75))*(3-8*(c-0.75)));
			zr=-z;
			z=1.6*z;
		}
		else
		{
			z=70;
			zr=-70;
			z=1.6*z;
		}
		
		printf("\n%lf\t%lf\t%lf\t%lf\t%lf\t%lf",x,xr,y,yr,z,zr);
		
		bot.right_leg->move(x,y,z,0);
		bot.left_leg->move(xr,yr,zr,0);
		comm.syncFlush();
		
		usleep(33333);
	}
	for(int i=0;i<t*fps;i++)
	{
		c=(float)((float)i/((float)t*fps));	
		printf("%f",c);
			
		x=390-30*2*c*(2-2*c);
		xr=390;
		
		y=scurve(-30,0,i,t*fps);
		yr=-y;
		
		if(c<0.25)
		{
			z=70*(4*c)*(4*c)*(3-8*c);
			zr=-z;
			z=1.6*z;
		}
		else if(c>0.75)
		{
			z=70*(1-(4*(c-0.75))*(4*(c-0.75))*(3-8*(c-0.75)));
			zr=-z;
			z=1.6*z;
		}
		else
		{
			z=70;
			zr=-70;
			z=1.6*z;
		}
		
		printf("\n%lf\t%lf\t%lf\t%lf\t%lf\t%lf",x,xr,y,yr,z,zr);
		
		bot.left_leg->move(x,y,z,0);
		bot.right_leg->move(xr,yr,zr,0);
		comm.syncFlush();
		
		usleep(33333);
	}
}
