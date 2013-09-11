#include <stdio.h>
#include "leg.h"
#include "communication.h"
#include <math.h>
#include "AcYut.h"

  
void raiseby(float xi,float yi,float zi,float xri,float yri,float zri,float xfi, float time, AcYut *bot)
{
	//printf("\n%lf\t%lf\t%lf\t%lf\t%lf\t%lf\n",xi,yi,zi,xri,yri,zri);
	float t=time;
	int fps=60;
	int sleep=16666;
	float x,y,z,xr,yr,zr;
	for(int i =0;i<t*fps;i++)
	{
		float c= (float)i/(float)(t*fps);
		x=xi+(xfi-xi)*c;
		xr=xri;
		
		y=yi;
		yr=yri;
		
		z=zi;
		zr=zri;
		
		//send(x,y,z,xr,yr,zr);
		bot->left_leg->move(x,y,z,0);
		bot->right_leg->move(xr,yr,zr,0);
		bot->updateBot();
		
		bot->left_leg->getMotorLoad(FOOT_ROLL);
		bot->right_leg->getMotorLoad(FOOT_ROLL);
		
		usleep(sleep);
	}
}

void wtshift(float xi,float yi,float zi,float xri,float yri,float zri,float zn,float zrn, AcYut *bot)
{
	float t=0.05;
	int fps=60;
	int sleep=16666;
	float x,y,z,xr,yr,zr;
	for(int i =0;i<t*fps;i++)
	{
		float c= (float)i/(float)(t*fps);
		x=xi;
		xr=xri;
		
		y=yi;
		yr=yri;
		
		z=zi+(zn-zi)*c;
		zr=zri+(zrn-zri)*c;;
		
		//send(x,y,z,xr,yr,zr);
		bot->left_leg->move(x,y,z,0);
		bot->right_leg->move(xr,yr,zr,0);
		bot->updateBot();
		
		bot->left_leg->getMotorLoad(FOOT_ROLL);
		bot->right_leg->getMotorLoad(FOOT_ROLL);
		
		usleep(sleep);
	}
}


int main()
{
	
	Communication comm;
	AcYut bot(&comm);
	float t=1;
	int fps=60;
	int h=0;
	float val[5];
	float *fval;
	int value;
	float zRight= 0;
	float zLeft = 0;
	float yRight= 0;
	float yLeft = 0;
	//	float initial_pitch=imu.pitch;
	float now_pitch;
	float x,y,z,xr,yr,zr,zn,zrn;
	for(int i=0;i<t*fps;i++)
	{
		float c=(float)i/(float)(t*fps);
		//printf("\n%lf\n",c);
		x=390;
		xr=390;
		
		y=0;
		yr=0;
		
		z=25*c*c*(3-2*c);
		zr=-z;
		
		//send(x,y+yLeft,z+zLeft,xr,yr+yRight,zr+zRight);
		bot.left_leg->move(x,y,z,0);
		bot.right_leg->move(xr,yr,zr,0);
		comm.syncFlush();
		bot.left_leg->getMotorLoad(FOOT_ROLL);
		bot.right_leg->getMotorLoad(FOOT_ROLL);
		
		usleep(16666);
	}
	x=390;
	z=25;
	zr=-25;
	while(x>=340)
	{	
		raiseby(x,y,z,xr,yr,zr,x-1,0.05,&bot);
		x = x-1;
		//imu.update();
		if(1)//fabs(now_pitch-initial_pitch)>0.5)
		{
			bot.left_leg->getMotorLoad(FOOT_ROLL);
			bot.right_leg->getMotorLoad(FOOT_ROLL);
			//usleep(50000);
			raiseby(x,y,z,xr,yr,zr,x+1,0.05,&bot);
			x = x+1;
			wtshift(x,y,z,xr,yr,zr,z+1,zr-1,&bot);
			z = z + 1;
			zr = zr - 1;
			//printf("Here in loop\n");
			if(z > 60)
			break;

			usleep(50000);
		}
	}
	while(1)
	{
		bot.left_leg->getMotorLoad(FOOT_ROLL);
		bot.right_leg->getMotorLoad(FOOT_ROLL);
		usleep(50000);
			
	}
	//raiseby(x,y,z,xr,yr,zr,x-30,5,&bot);
	
	//bot.left_leg->getLoad();
	//bot.right_leg->getLoad();




	
}
