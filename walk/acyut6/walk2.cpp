#include <stdio.h>
#include "leg.h"
#include "communication.h"
#include <math.h>
#include "AcYut.h"
#define NOM 28


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



enum SupportPhase{DSP,SSP};
float lYin  = -25;
float slYin = 20;
float vYin  = 100;
float vYfi  = 100;
float vTTar = 100;
float lZin = 20 ;
float slZin = 20;
float vZin  = -140;
float vZfi  = 140;

enum state{Y,YR,YV,Z,ZR,ZV,R,RR};

float endState[8];

float* dribble(float legYin, float supLegYin, float veloYin, float veloYfi, float legZin, float supLegZin, float veloZin, float veloZfi, float legRotin, float legRotfi, float supLegRotin, float supLegRotfi, Leg *leg, Leg *revLeg, Communication *comm)
{
	printf("VYin\t%lf\tVyfi\t%lf\n",veloYin,veloYfi);
	int fps = 60;
	int sleep = 1000000.0/(float)fps;
	float timeInc =1.0/(float)fps;
	float feetSeperation = 10;
	float hipLength = 130;
	
	float Tc = sqrt(576.00/9810.0);
	legZin += hipLength/2;
	supLegZin += hipLength/2;
	
	///// desired Values 
	
	float D_dsp1Time = 0.05;
	float D_dsp2Time = 0.05;
	
	printf("Tc\t\t%lf\n",Tc);
	printf("legZin\t\t%lf\n",legZin);
	printf("supLegZin\t%lf\n",supLegZin);
	
	//// Z Control 
	
	float zMax = 60;
	
	float sspZAmp   = zMax; 
	float sspZPhs   = asinh(veloZin*Tc/sspZAmp);
	float sspZTime  = Tc * (asinh(veloZfi*Tc/sspZAmp) - sspZPhs);
	float sspZSupin = sspZAmp * cosh(sspZPhs);
	float sspZSupfi = sspZAmp * cosh(sspZTime/Tc + sspZPhs);
	
	float sspTime   = sspZTime;
	float dsp1Time  = (supLegZin - sspZSupin)/(-veloZin);		
	float dsp2Time  = D_dsp2Time;
	float stepTime  = sspTime + dsp1Time + dsp2Time;
	
	float sspZin    = legZin + (-veloZin)*dsp1Time;
	float P_sspZAmp = zMax;
	float P_sspZPhs = asinh(-veloZfi*Tc/P_sspZAmp);
	float P_sspZin  = P_sspZAmp * cosh(P_sspZPhs);
	float P_legZin  = P_sspZin - (-veloZfi)* D_dsp2Time;
	float P_veloZfi = veloZfi + (veloZfi + veloZin)/2;
	float P_sspZTime= Tc * (asinh(P_veloZfi*Tc/P_sspZAmp) - P_sspZPhs);
	
	
	float sspZfi    = P_legZin + veloZfi*D_dsp1Time; 
	
	float legZfi    = sspZfi - veloZfi*dsp2Time;
	float supLegZfi = sspZSupfi + veloZfi*dsp2Time;
	
	//TODO add case in which dsp1Time becomes negative 
	//TODO add case where dsp2Time needs to be modified (if required)
	//TODO calculation of sspZfi for next step based on predicted change in velocity for the next step 
	
	printf("zMax\t\t%lf\n",zMax);
	printf("sspZPhs\t\t%lf\n",sspZPhs);
	printf("sspZTime\t%lf\n",sspZTime);
	printf("sspZin\t\t%lf\n",sspZin);
	printf("sspZSupin\t%lf\n",sspZSupin);
	printf("sspZfi\t\t%lf\n",sspZfi);
	printf("sspZSupfi\t%lf\n",sspZSupfi);
	printf("dsp1Time\t%lf\n",dsp1Time);
	printf("stepTime\t%lf\n",stepTime);
	printf("LegZfi\t\t%lf\n",legZfi);
	printf("supLegZfi\t%lf\n",supLegZfi);
	
	// Y Control 
	
	legYin    = -legYin;
	supLegYin = -supLegYin;
	
	float sspYin    = legYin + veloYin * dsp1Time;
	float sspYSupin = supLegYin + veloYin * dsp1Time;
	float sspYAmp   = veloYin * sqrt(pow(Tc,2) - pow(sspYSupin/veloYin,2));
	float sspYPhs   = asinh((sspYSupin/veloYin)/sqrt(pow(Tc,2) - pow(sspYSupin/veloYin,2)));
	float veloYfi_d   = sspYAmp/Tc * cosh (sspTime/Tc + sspYPhs);
	//printf("**************%lf******************\n",sspYAmp/Tc * cosh (sspTime/Tc + sspYPhs));
	float P_sspYAmp = sgn(veloYin) * Tc * sqrt((2*cosh(P_sspZTime/Tc)*veloYfi_d*veloYfi - pow(veloYfi_d,2) - pow(veloYfi,2))/(pow(sinh(P_sspZTime/Tc),2)));
	float P_sspYPhs = -acosh(veloYfi_d*Tc/P_sspYAmp);
	float P_sspYin  = P_sspYAmp * sinh(P_sspYPhs);
	float P_legYin  = P_sspYin - veloYfi_d*D_dsp1Time;

	float sspYfi    = P_legYin - veloYfi_d*dsp2Time;
	float sspYSupfi = sspYAmp * sinh(sspTime/Tc + sspYPhs);
	
	float legYfi    = sspYfi + veloYfi_d*dsp2Time;
	float supLegYfi = sspYSupfi + veloYfi_d*dsp2Time;
	
	printf("\n\n\n");
	printf("legYin\t\t%lf\n",legYin);
	printf("supLegYin\t%lf\n",supLegYin);
	printf("sspYin\t\t%lf\n",sspYin);
	printf("sspYSupin\t%lf\n",sspYSupin);
	printf("sspYAmp\t\t%lf\n",sspYAmp);
	printf("sspYPhs\t\t%lf\n",sspYPhs);
	printf("sspYfi\t\t%lf\n",sspYfi);
	printf("sspYSupfi\t%lf\n",sspYSupfi);
	printf("legYfi\t\t%lf\n",legYfi);
	printf("suplegYfi\t%lf\n",supLegYfi);
	printf("veloYfi_d\t%lf\n",veloYfi_d);
	
	printf("P_sspYAmp\t%lf\n",P_sspYAmp);
	printf("P_sspYPhs\t%lf\n",P_sspYPhs);
	printf("P_sspZTime\t%lf\n",P_sspZTime);
	printf("P_sspYin\t\t%lf\n",P_sspYin);
	printf("P_legYin\t\t%lf\n",P_legYin);
	
	
	float height = 390;
	float lift   = 20;
	float xfreq  = 2*3.14;
	float displacement = 1;
	float startX = dsp1Time;
	float stopX  = stepTime-dsp2Time;
	float xPhase =-3.14/2;
	
	
	float x,xr,y,yr,z,zr,phi,phiR;
	float yReal,yrReal,zReal,zrReal;
	float walkTime =0.0;
	int walk_i = 0;
	int target_i = 0;
	float *fval;
	//compliance(leg,10,10);
	
	
	int state = DSP;
	int sspMotVal = 1703;
	int sspRMotVal = 1703;

	printf("*************************************** STEP **********************************************\n");	
	for(walkTime = 0.0/fps; walkTime<=stepTime; walkTime +=timeInc)
	{
		if(walkTime < dsp1Time)
		{
			x  = height;
			xr = height;
			y  = -legYin - veloYin*walkTime;
			yr = -supLegYin - veloYin*walkTime;
			z  = legZin - veloZin*walkTime - hipLength/2;
			zr = supLegZin + veloZin*walkTime - hipLength/2;
			phi= legRotin; 
			phiR= supLegRotin;
			printf("DSP1\t");
			state = DSP;
		}
		else if (walkTime >= dsp1Time && walkTime <= dsp1Time + sspTime)
		{
			x  = height - lift * (sin(xfreq*((walkTime-startX)/(stopX-startX))+xPhase) + displacement)/(1+displacement);
			xr = height;
			y  = linear(-sspYin,-sspYfi, walkTime-dsp1Time,sspTime);
			yr = -sspYAmp * sinh((walkTime-dsp1Time)/Tc +sspYPhs);
			z  = scurve(sspZin,sspZfi, walkTime-dsp1Time,sspTime) - hipLength/2;
			zr =  sspZAmp * cosh((walkTime-dsp1Time)/Tc + sspZPhs) -hipLength/2;
			phi= linear(legRotin,legRotfi,walkTime-dsp1Time,sspTime);
			phiR=linear(supLegRotin,supLegRotfi,walkTime-dsp1Time,sspTime);
			state = SSP;
			printf("SSP0\t");
		}
		else if (walkTime > dsp1Time+sspTime )//&& walkTime<=stepTime
		{
			state = DSP;
			x  = height;
			xr = height;
			y  = -sspYfi - veloYfi_d*(walkTime-dsp1Time-sspTime);
			yr = -sspYSupfi - veloYfi_d*(walkTime-dsp1Time-sspTime);
			z  = sspZfi - veloZfi*(walkTime-dsp1Time-sspTime) -hipLength/2;
			zr = sspZSupfi + veloZfi*(walkTime-dsp1Time-sspTime) - hipLength/2;
			phi= legRotfi;
			phiR= supLegRotfi;
			printf("DSP2\t");
		}
		else 
		{
			printf("************* SOMETHING WRONG*******************\n");
		}
		//printf("X\t%3.1lf\tXR\t%3.1lf\tY\t%lf\tYR\t%lf\tZ\t%lf\tZR\t%lfP\t%lf\tPR\t%lf\n",x,xr,y,yr,z,zr,phi,phiR);
		//printf("Y\t%lf\tYR\t%lf\tZ\t%lf\tZR\t%lf\tP\t%lf\tPR\t%lf\n",y,yr,z,zr,phi,phiR);
		printf("Z\t%lf\tZR\t%lf\n",z,zr);
		///printf("X\t%lf\tXR\t%lf\tZ\t%lf\tZR\t%lf\tY\t%lf\tYR\t%lf\n",x,xr,z,zr,y,yr);
		
		
		/*if(leg==0)
		{
			zReal=z*cos(-phi*3.14/180)+y*sin(-phi*3.14/180);
			yReal=-z*sin(-phi*3.14/180)+y*cos(-phi*3.14/180);
			
			zrReal=zr*cos(phiR*3.14/180)-yr*sin(phiR*3.14/180);
			yrReal=zr*sin(phiR*3.14/180)+yr*cos(phiR*3.14/180);
		}
		else if(leg==1)
		{
			zReal=z*cos(phi*3.14/180)-y*sin(phi*3.14/180);
			yReal=z*sin(phi*3.14/180)+y*cos(phi*3.14/180);
			
			zrReal=zr*cos(-phiR*3.14/180)+yr*sin(phiR*3.14/180);
			yrReal=-zr*sin(-phiR*3.14/180)+yr*cos(phiR*3.14/180);
			
		}
		*/
		leg->move(x,y,z+feetSeperation,0);
		revLeg->move(xr,yr,zr+feetSeperation,0);
		comm -> syncFlush();
		leg->getMotorLoad(FOOT_ROLL);
		revLeg->getMotorLoad(FOOT_ROLL);

		usleep(sleep);
		
	}
	endState[Y]  = -legYfi;
	endState[YR] = -supLegYfi;
	endState[YV] = veloYfi_d;
	endState[Z]  = supLegZfi - hipLength/2;
	endState[ZR] = legZfi - hipLength/2;
	endState[ZV] = veloZfi;
	endState[R]  = legRotfi;
	endState[RR] = supLegRotfi;
	
	return endState;
}


float scurve2(float in,float fi,float t, float tot)
{
	float frac=t/tot;
	float ret_frac=6*pow(frac,5)-15*pow(frac,4)+10*pow(frac,3);
	return in+(fi-in)*ret_frac;
}

int walk3(float yin,float yfi,float yrin,float yrfi, float zin, float zfi,float zrin, float zrfi, float thetain, float thetafi,float thetarin, float thetarfi,Leg *leg, Leg *revLeg, Communication *comm)
{
	FILE *f;
		
	float time=1;
	float fps=60;
	
	/////// X-Lift Control /////
	float x_start=0.2; // increase 
	float x_end=0.8;   // decrease 
	float lift=40;
	float height=390;
	float freq=2*3.14;
	float displacement=1;	
	float phase=-1.57;
	
	/////// X-Land Control /////
	//float dec_time=0.75;	
	float window=1-x_end;    
	float load_dec=0;//10       ///////// 0
	float freq2=2*3.14;
	float displacement2=1;
	float phase2=-1.57;
	
	/////// Z-Shift Control /////
	
	float peak_start=0.0;
	float peak_max=0.5;//0.5
	float peak_stay=0.65;//+0.1*leg;//0.75
	float peak_end=1;
	float zmax=25+7;//*leg;
	float zfreq=1.57;
	float zphase=0;
	float zdisplacement=0;
//	float peaktime=0.75;
	
	/////// Y Control /////
	
	float y_start=0.25;
	float y_end=0.75;
	float yr_start=0.25;
	float yr_end=0.75;

	/////// Z Control /////

	float z_start=0.25;
	float z_0=0.417;
	float z_move=0.5833;
	float z_end=0.5;
        
    ///////Separation control////
    	float seperation=5;
	
	//////Turn Control////
	float turn_start;
	float turn_end;

	/////// Live Variables /////
	int frame=0;
	float t_frac=0,t=0;
	float x,xr,y,z,yr,zr,z_shift,phi,phir;
	float yreal,yrreal,zreal,zrreal;
	float com_y,com_z;
	
 	/////COMPLIANCE//
	float cptime=1231321312;//x_end;

	/////// COM Trajectory //////
	//Z Movement
	float z_com_start=0;
	float z_com_peak=0.5;
	float z_com_stay=0.75;
	float z_com_end=1;
	float z_com_max=32;
	
	//Y Movement

	float y_com_start=0;
	float y_com_end=1;

	
	/*/////// Packet Generation Variables /////
	int adchksum=7+NOM*3,i=0;
	float *fval;
	float val[5],valr[5];
	int mot=0*(1-leg)+20*leg;
	int revmot=20*(1-leg)+0*leg;
	int value=0;
      */  
	//compliance(leg,10,10);	
	for(frame=0;frame<time*fps;frame++,t+=1.0/fps)
	{
		t_frac=t/time;
		x=height;
		xr=height;
		
		////X LIFT////		
		if(t_frac>x_start&&t_frac<x_end)
		x-=lift*(sin(freq*(t_frac-x_start)/(x_end-x_start)+phase)+displacement)/(1+displacement);
		
		//if(t_frac>x_end-window&&t_frac<x_end+window)
		//x-=load_dec*(sin(freq2*(t_frac-x_end+window)/(2*window)+phase2)+displacement2)/(1+displacement2);
		//if(t_frac<increase_end)
		//xr-=load_dec*(sin(freq2*(t_frac)/(increase_end)+phase2)+displacement2)/(1+displacement2);
		z_shift = 0;
		/////Z SHIFT//////
		if(t_frac<peak_start)
		z_shift=0;
		else if(t_frac>peak_start&&t_frac<peak_max)
		z_shift=zmax*(sin(zfreq*(t_frac-peak_start)/(peak_max-peak_start)+zphase)+zdisplacement)/(1+zdisplacement);
		else if(t_frac>peak_stay&&t_frac<peak_end)
		z_shift=zmax-zmax*(sin(zfreq*(t_frac-peak_stay)/(peak_end-peak_stay)+zphase)+zdisplacement)/(1+zdisplacement);
		else if(t_frac>peak_end)
		z_shift=0;
		//else
		//z_shift=z_shift;
	
	
		
		////Y MOVEMENT////
		if(t_frac<y_start)
		y=yin;  
        else if(t_frac>=y_start&&t_frac<y_end)
		y=scurve2(yin,yfi,t_frac-y_start,y_end-y_start);
		else
		y=yfi;


		/////YR MOVT/////

	    if(t_frac<yr_start)
		yr=yrin;
		else if(t_frac>=yr_start&&t_frac<yr_end)
		yr=scurve2(yrin,yrfi,t_frac-yr_start,yr_end-yr_start);
		else 
		yr=yrfi;
		
		
		//////////////// Z MOVEMENT///
		if(t_frac<z_start)
		{
			z=zin;
			zr=zrin;
		}
		else if(t_frac>z_start&&t_frac<z_0)	
	    {
			z=scurve(zin,zin+zrin,t_frac-z_start,z_0-z_start);
			zr=scurve(zrin,0,t_frac-z_start,z_0-z_start);
		}
		else if(t_frac>z_0&&t_frac<z_move)
		{

			z=scurve(zin+zrin,zfi+zrfi,t_frac-z_0,z_move-z_0);
			zr=scurve(0,0,t_frac-z_0,z_move-z_0);
		}
		else if(t_frac>z_move&&t_frac<z_end)
		{
			z=scurve(zrfi+zfi,zfi,t_frac-z_move,z_end-z_move);
			zr=scurve(0,zrfi,t_frac-z_move,z_end-z_move);
		}
		////TURN////
		if(t_frac<turn_start)
		{
			phi=thetain;
			phir=thetarin;
		}
		else if(t_frac>turn_start&&t_frac<turn_end)
		{
			phi=scurve(thetain,thetafi,t_frac-turn_start,turn_end-turn_start);
			phir=scurve(thetarin,thetarfi,t_frac-turn_start,turn_end-turn_start);
		}
		else
		{
			phi=thetafi;
			phir=thetarfi;
		}
		
		printf("z%lf zshift %lf r%lf\n",z,z_shift,zr);
		/////// COM Trajectory /////
		///////// Z Axis ////////
		if(t_frac<z_com_start)
		com_z=0;
		else if(t_frac>z_com_start&&t_frac<z_com_peak)
		com_z=z_com_max*sin(1.57*(t_frac-z_com_start)/(z_com_peak-z_com_start));
		else if(t_frac>=z_com_peak&&t_frac<z_com_stay)
		com_z=z_com_max;
		else if(t_frac>=z_com_stay&&t_frac<z_com_end)
		com_z=z_com_max*sin(1.57*(t_frac-z_com_stay)/(z_com_end-z_com_stay)+1.57);
		else
		com_z=0;
		
		/*if(frame!=0)
		com_z=(com[1]-left_joints[6][1]*leg-right_joints[6][1]*(1-leg)+(-55)*leg+(55)*(1-leg));
		else
		com_z=0;
		*/
		///////// Y Axis ////////
		if(t_frac<=y_com_start)
		com_y=0;
		else if(t_frac>y_com_start&&t_frac<=y_com_end)
		com_y=(yrin-yrfi)*(t_frac-y_com_start)/(y_com_end-y_com_start);
		else
		com_y=(yrin-yrfi);
		
		/////Compliance////
		//if(compare_float(cptime,t_frac)==1)
		//compliance(leg,10,32);
		
		
		//if(leg==0)
		//{
			zreal=z;//z*cos(-phi*3.14/180)+y*sin(-phi*3.14/180);
			yreal=y;//-z*sin(-phi*3.14/180)+y*cos(-phi*3.14/180);
			
			zrreal=zr;//zr*cos(phir*3.14/180)-yr*sin(phir*3.14/180);
			yrreal=yr;//zr*sin(phir*3.14/180)+yr*cos(phir*3.14/180);
		//}
		//else if(leg==1)
		//{
		//	zreal=z*cos(phi*3.14/180)-y*sin(phi*3.14/180);
		//	yreal=z*sin(phi*3.14/180)+y*cos(phi*3.14/180);
		//	
		//	zrreal=zr*cos(-phir*3.14/180)+yr*sin(phir*3.14/180);
		//	yrreal=-zr*sin(-phir*3.14/180)+yr*cos(phir*3.14/180);
		//
		//}
			
		zreal+=z_shift;
		zrreal-=z_shift;

		printf("X\t%lf\tXR\t%lf\tY\t%lf\tYR\t%lf\tZ\t%lf\tZR\t%lf\n",x,xr,yreal,yrreal,zreal,zrreal);

		leg->move(x,yreal,zreal+10,0);
		revLeg->move(xr,yrreal,zrreal+10,0);
		comm -> syncFlush();
		//leg->getMotorLoad(FOOT_ROLL);
		//revLeg->getMotorLoad(FOOT_ROLL);

		usleep(1000000.0/fps);
	}	
	printf("******************************************************************************************\n");
	//COUNTER+=STEP*2;
	//COM_INC+=(yrin-yrfi);
	return EXIT_SUCCESS;
		
	
}

int main()
{
	Communication comm;
	AcYut bot(&comm);

	float *state;
	state = endState;
	while(1)
	{
		walk3(-30,30,30,-30,0,0,0,0,0,0,0,0,bot.right_leg,bot.left_leg,&comm);
		printf("\n");
		walk3(-30,30,30,-30,0,0,0,0,0,0,0,0,bot.left_leg,bot.right_leg,&comm);
		printf("\n");
	}
	
	
}
