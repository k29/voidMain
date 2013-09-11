#include "AcYut.h"
#include "communication.h"
#include "xsens/imu.h"

float scurve(float in,float fi,float t, float tot)
{
	float frac=(float)t/(float)tot;
	if(frac>1)
	frac=1;
	float retfrac=frac*frac*(3-2*frac);
	float mval=in + (fi-in)*retfrac;
	return mval;
}


float scurve2(float in,float fi,float t, float tot)
{
	float frac=t/tot;
	float ret_frac=6*pow(frac,5)-15*pow(frac,4)+10*pow(frac,3);
	return in+(fi-in)*ret_frac;
}


int kick(Leg *leg, Leg *revLeg, Communication *comm)
{
	float kickTime = 1.0;
	int fps = 80;
	float frameTime	= (float)1.0/(float)fps;
	int sleep = frameTime * 1000000;
	printf("FrameTime\t%lf\n",frameTime);
	float kickHeight = 40;
	float botHeight  = 390;
	float wtShift    = 50;
	float wtShiftTime= 0.5;
	float liftTime   = 0.5;
	float counterBalance = 15;
	
	
	float yStart=0, yrStart=0, zStart=0, zrStart=0;
	float x,xr,y,yr,z,zr;
	for(float time=0;time<wtShiftTime;time+=frameTime)
	{
		x = botHeight;
		xr= botHeight;
		y = yStart;
		yr= yrStart;
		z = scurve2(zStart, wtShift, time, wtShiftTime);
		zr= scurve2(zrStart, -wtShift, time, wtShiftTime);
		printf("Time\t%lf\tX\t%lf\tY\t%lf\tZ\t%lf\tXR\t%lf\tYR\t%lf\tZR\t%lf\n",time,x,y,z,xr,yr,zr);
	
		leg->runIK(x,y,z,0);
		revLeg->runIK(xr,yr,zr,0);
		comm->syncFlush();
		
		usleep(sleep);
	}
	
	for(float time=0;time<liftTime;time+=frameTime)
	{
		x = scurve2(botHeight, botHeight - kickHeight, time, liftTime);
		xr= botHeight;
		y = yStart;
		yr= yrStart;
		z = scurve2(wtShift,wtShift+counterBalance,time,liftTime);
		zr= scurve2(-wtShift,-wtShift - counterBalance, time, liftTime);
	
		printf("Time\t%lf\tX\t%lf\tY\t%lf\tZ\t%lf\tXR\t%lf\tYR\t%lf\tZR\t%lf\n",time,x,y,z,xr,yr,zr);
		leg->runIK(x,y,z,0);
		revLeg->runIK(xr,yr,zr,0);
		comm->syncFlush();
		usleep(sleep);
		
	} 		
	
};


int main()
{
	Imu imu;
	imu.init();
	
	Communication comm;
	AcYut bot(&comm,&imu);
	int quit=1;
	while(quit)
	{
		usleep(16670);
		printf("Roll\t%3.3lf\tPitch\t%3.3lf\tYaw\t%3.3lf\n",imu.roll,imu.pitch,imu.yaw);
		const double (&var1)[AXES] = bot.getCOM();
		printf("mainX\t%3.3lf\tY\t%3.3lf\tZ\t%3.3lf\n",var1[0],var1[1],var1[2]);
		bot.printCOM();
		bot.getWorldFrameCoods((double*)var1);
		
	}
	
	//printf("******************************************\n\n\n\n");
	//bot.left_leg->runIK(390,0,10,0);
	//bot.right_leg->runIK(390,0,-10,0);
	//bot.getCOM();
	//const float (&joints)[LEG_MOTOR_COUNT][AXES] = bot.left_leg->runFK();
	
	//printf("\n\n\nIN MAIN\n");
	//for(int i = 0; i <LEG_MOTOR_COUNT;i++)
	//printf("X%lf\tY%lf\tZ%lf\n",joints[i][0],joints[i][1],joints[i][2]);
	
	//bot.right_leg->runIK(390,50,50,0);
	//bot.right_leg->runFK();
	
	//kick(bot.right_leg,bot.left_leg,&comm);

};

