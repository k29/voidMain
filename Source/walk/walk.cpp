#include "walk.h"
int k=1;
double pi = acos(-1);
Walk::Walk(AcYut* bot)
{
	this->bot = bot;
	legYin=-1.773347;
	supLegYin=-4.571982;
	veloYin=-0.611621;
	veloYfi=10;
	legZin=25.155276;
	supLegZin=25.155276;
	veloZin=-170;
	veloZfi=170;
	zMax=55;
	dz = 0;
	lift=40;
	legRotin=0;
	legRotfi=0;
	supLegRotin=0;
	supLegRotfi=0;
	sspTimeVar = 0.300999;
	// strafe=0;
	// sspZAmp=zMax;
	stepCount = 0;

	integ_const_z = 0.05;//0.05;//2;
	deriv_const_z = 0.01;//0.01;//0.005;
	prop_const_z = 0.6;//0.6;//1;
	integ_max_z = 2;
	prev_mean_z = 0;

	integ_const_y = 1;//1;
	deriv_const_y = 0.005;//0.005;
	prop_const_y = 1;//1;
	integ_max_y = 2;
	prev_mean_y = 0;
	prev_imu_yaw = 0;
	start2();
	printf("START completed\n\n\n");
};

int Walk::start2()
{
//	leg=(LEG)(1-(int)leg);
	printf("%d\t",leg);
	// printf("VYin\t%lf\tVyfi\t%lf\n",veloYin,veloYfi);
	int fps = 120;
	int sleep = 1000000.0/(double)fps;
	double timeInc =1.0/(double)fps;
	double feetSeperation = 30;
	double hipLength = 130;
	
	double Tc = sqrt(600.00/9810.0);
	legZin += hipLength/2;
	supLegZin += hipLength/2;
	
	///// desired Values 
	
	double D_dsp1Time = 0.03;
	double D_dsp2Time = 0.03;
	
	printf("Tc\t\t%lf\n",Tc);
	// ////printf("legZin\t\t%lf\n",legZin);
	// ////printf("supLegZin\t%lf\n",supLegZin);
	
	//// Z Control 
	//Changed to reflect variable stepping time.
	double sspZAmp   = zMax;
	// double sspZTime  = Tc * (asinh(veloZfi*Tc/sspZAmp) - sspZPhs);
	double sspZTime = sspTimeVar;
	veloZin = -sspZAmp*sinh(sspZTime/Tc)/(Tc*sqrt(2*(1 + cosh(sspZTime/Tc))));
	// sspZAmp = (veloZfi*Tc)/(sinh(sspZTime/Tc + sspZPhs));
	// veloZfi = sspZAmp*sinh(sspZTime/Tc + sspZPhs)/Tc;
	veloZfi = - veloZin; 
	double sspZPhs   = asinh(veloZin*Tc/sspZAmp);
	
	//End of change

	double sspZSupin = sspZAmp * cosh(sspZPhs);
	double sspZSupfi = sspZAmp * cosh(sspZTime/Tc + sspZPhs);
	
	double sspTime   = sspZTime;
	double dsp1Time  = (supLegZin - sspZSupin)/(-veloZin);		
	double dsp2Time  = D_dsp2Time;
	double stepTime  = sspTime + dsp1Time + dsp2Time;
	
	double sspZin    = legZin + (-veloZin)*dsp1Time;
	double P_sspZAmp = zMax;
	double P_sspZPhs = asinh(-veloZfi*Tc/P_sspZAmp);
	double P_sspZin  = P_sspZAmp * cosh(P_sspZPhs);
	double P_legZin  = P_sspZin - (-veloZfi)* D_dsp2Time;
	double P_veloZfi = veloZfi + (veloZfi + veloZin)/2;
	double P_sspZTime= Tc * (asinh(P_veloZfi*Tc/P_sspZAmp) - P_sspZPhs);
	
	
	double sspZfi    = P_legZin + veloZfi*D_dsp1Time; 
	double legZfi    = sspZfi - veloZfi*dsp2Time;
	double supLegZfi = sspZSupfi + veloZfi*dsp2Time;
	
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
	
	double sspYin    = legYin + veloYin * dsp1Time;
	double sspYSupin = supLegYin + veloYin * dsp1Time;
	double veloYfi_d = veloYfi;
	double Cy = cosh(sspTime/(2*Tc))/sinh(sspTime/(2*Tc));
	double By = veloYfi_d*Tc/(cosh(sspTime/Tc)-Cy*sinh(sspTime/Tc));
	double Ay = -By*Cy;
	printf("Ay\t\t%lf\n",Ay);
	printf("By\t\t%lf\n",By);
	printf("Cy\t\t%lf\n",Cy);
	// double sspYAmp   = veloYin * sqrt(pow(Tc,2) - pow(sspYSupin/veloYin,2));
	// double sspYPhs   = asinh((sspYSupin/veloYin)/sqrt(pow(Tc,2) - pow(sspYSupin/veloYin,2)));
	// double veloYfi_d   = sspYAmp/Tc * cosh (sspTime/Tc + sspYPhs);
	
	//////printf("**************%lf******************\n",sspYAmp/Tc * cosh (sspTime/Tc + sspYPhs));
	double P_sspYAmp = sgn(veloYfi_d) * Tc * sqrt((2*cosh(P_sspZTime/Tc)*veloYfi_d*veloYfi - pow(veloYfi_d,2) - pow(veloYfi,2))/(pow(sinh(P_sspZTime/Tc),2)));
	double P_sspYPhs = -acosh(veloYfi_d*Tc/P_sspYAmp);
	double P_sspYin  = P_sspYAmp * sinh(P_sspYPhs);
	double P_legYin  = P_sspYin - veloYfi_d*D_dsp1Time;

	double sspYfi    = P_legYin - veloYfi_d*dsp2Time;
	// double sspYSupfi = sspYAmp * sinh(sspTime/Tc + sspYPhs);
	double sspYSupfi = -(Ay*cosh(sspTime/(Tc)) + By*sinh(sspTime/(Tc)));
	double legYfi    = sspYfi + veloYfi_d*dsp2Time;
	double supLegYfi = sspYSupfi + veloYfi_d*dsp2Time;
	
	printf("\n\n\n");
	printf("legYin\t\t%lf\n",legYin);
	printf("supLegYin\t%lf\n",supLegYin);
	printf("sspYin\t\t%lf\n",sspYin);
	printf("sspYSupin\t%lf\n",sspYSupin);
	// printf("sspYAmp\t\t%lf\n",sspYAmp);
	// printf("sspYPhs\t\t%lf\n",sspYPhs);
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
	// Y (cubic)
	
	double a = ((-veloYfi-veloYin)-2*(-sspYfi+sspYin)/sspTime)/pow(sspTime,2);
	double b = ((-sspYfi+sspYin)/sspTime+veloYin-a*pow(sspTime,2))/sspTime;
	double c = -veloYin;
	double d = -sspYin;
	
	double height = 400;
	//double lift   = 30;
	double xfreq  = 2*pi;
	double displacement = 1;
	double startX = dsp1Time-1.0/60.0;
	double stopX  = stepTime-dsp2Time + 1.0/60.0;
	double xPhase =-pi/2;
	
	double dampLift = 0;
	double dampFreq = 2*pi;
	double dampStart = dsp1Time + sspTime - 4.0/60.0;
	double dampEnd   = stepTime;
	double dampPhase = -pi/2;
	double dampDisplacement = 1;
	
	double fraction;
	
	double x,xr,y,yr,z,zr,phi,phiR,s,sr;
	double yReal,yrReal,zReal,zrReal;
	double walkTime =0.0;
	int walk_i = 0;
	int target_i = 0;
	double *fval;
	//compliance(leg,10,10);
	
	double y_half_time=a*pow(sspTime/2.0,3)+b*pow(sspTime/2.0,2)+c*(sspTime/2.0)+d;
	// double yr_half_time=-sspYAmp * sinh((sspTime/2.0)/Tc +sspYPhs);
	double yr_half_time = Ay*cosh(sspTime/(2*Tc)) + By*sinh(sspTime/(2*Tc));
	double z_half_time=scurve(sspZin,sspZfi,sspTime/2.0,sspTime) - hipLength/2;
	double zr_half_time=sspZAmp * cosh((sspTime/2.0)/Tc + sspZPhs) -hipLength/2;
	
	printf("Z %lf ZR %lf Y %lf YR %lf\n",z_half_time,zr_half_time,y_half_time,yr_half_time);
	//bot->reachSlow(400,y_half_time,z_half_time,400,yr_half_time,zr_half_time);
	bot->reachSlow(400,y_half_time,z_half_time,400,yr_half_time,zr_half_time);
	int state = DSP;
//	////printf("*************************************** STEP **********************************************\n");	
	for(walkTime = dsp1Time+sspTime/2.0; walkTime<=stepTime; walkTime +=timeInc)
	{
		if (walkTime >= dsp1Time && walkTime <= dsp1Time + sspTime)
		{
			fraction=2*(walkTime-startX)/(stopX-startX);
			//x  = height - lift * (sin(xfreq*((walkTime-startX)/(stopX-startX))+xPhase) + displacement)/(1+displacement);
			x = height;//- lift*fraction*(2-fraction);
			xr = height;
			y=a*pow(walkTime-dsp1Time,3)+b*pow(walkTime-dsp1Time,2)+c*(walkTime-dsp1Time)+d;
			//y  = linear(-sspYin,-sspYfi, ((walkTime-dsp1Time)/sspTime);
			// yr = -sspYAmp * sinh((walkTime-dsp1Time)/Tc +sspYPhs);
			yr = Ay*cosh((walkTime-dsp1Time)/(Tc)) + By*sinh((walkTime-dsp1Time)/(Tc));
			z  = scurve(sspZin,sspZfi, walkTime-dsp1Time,sspTime) - hipLength/2;
			zr =  sspZAmp * cosh((walkTime-dsp1Time)/Tc + sspZPhs) -hipLength/2;
			phi= scurve(legRotin,legRotfi,walkTime-dsp1Time,sspTime);
			phiR=scurve(supLegRotin,supLegRotfi,walkTime-dsp1Time,sspTime);
			state = SSP;
	//		////printf("SSP0\t");
		}
		else if (walkTime > dsp1Time+sspTime )//&& walkTime<=stepTime
		{
			state = DSP;
			x  = height;// - dampLift* (sin(dampFreq*((walkTime-dampStart)/(dampEnd-dampStart))+dampPhase)+dampDisplacement)/(1+dampDisplacement);
			xr = height;
			y  = -sspYfi - veloYfi_d*(walkTime-dsp1Time-sspTime);
			yr = -sspYSupfi - veloYfi_d*(walkTime-dsp1Time-sspTime);
			z  = sspZfi - veloZfi*(walkTime-dsp1Time-sspTime) -hipLength/2;
			zr = sspZSupfi + veloZfi*(walkTime-dsp1Time-sspTime) - hipLength/2;
			phi= legRotfi;
			phiR= supLegRotfi;
	//		////printf("DSP2\t");
		}
		else 
		{
	//		////printf("************* SOMETHING WRONG*******************\n");
		}
		printf("X\t%3.1lf\tXR\t%3.1lf\tY\t%lf\tYR\t%lf\tZ\t%lf\tZR\t%lf\n",x,xr,y,yr,z,zr);
		////printf("Y\t%lf\tYR\t%lf\tZ\t%lf\tZR\t%lf\tP\t%lf\tPR\t%lf\n",y,yr,z+s,zr+sr,phi,phiR);
		///////printf("Z\t%lf\tZR\t%lf\n",z,zr);
		//printf("S phi\t%lf\tphiR\t%lf\tZ\t%lf\tZR\t%lf\tY\t%lf\tYR\t%lf\n",phi,phiR,z,zr,y,yr);
		
		bot->leg[leg]->runIK(x,y,z+feetSeperation,phi);
		bot->leg[1-leg]->runIK(xr,yr,zr+feetSeperation,phiR);
		//printf("%d %d\n\n\n",leg,1-leg);
		bot->updateBot();
		//printf("Sent Values\n");
		//leg->getMotorLoad(FOOT_ROLL);
		//revLeg->getMotorLoad(FOOT_ROLL);

		usleep(sleep);
		//getchar();
		//printf("Z = %f Zr = %f Y = %f Yr = %f\n", z, zr, y, yr);
		
	}
	supLegYin  = -legYfi;
	legYin = -supLegYfi;
	veloYin=veloYfi_d;
	supLegZin = supLegZfi - hipLength/2;
	legZin = legZfi - hipLength/2;
	veloZin = -veloZfi;
	veloZfi= veloZfi;	
	supLegRotin  = legRotfi;
	legRotfi=0;
	legRotin = supLegRotfi;
	supLegRotfi=0;
	return EXIT_SUCCESS;//endState55	
	
}

int Walk::kick()
{
	


}

int Walk::start()
{
	lift=5;
	dribble();
	lift=10;
	dribble();
	lift=20;
	dribble();
	lift=30;
}

float Walk::accelerate()
{
	veloYfi=veloYfi*1.72;
	legRotfi = 0;
	supLegRotfi =0;
}

float Walk::decelerate()
{
	veloYfi=veloYfi*0.35;
	legRotfi=0;
	supLegRotfi=0;
}

float Walk::velocity()
{
	return veloYfi;
}

float Walk::velocity2()
{
	return veloYfi_d;
}

float Walk::turnright(float theta)
{
	if(theta==0)
	{
		legRotfi=0;
		supLegRotfi=0;
		dribble();
		return 0;
	}
	if(theta>15)
		legRotfi=(1-(int)leg)*15;
	else
		legRotfi=(1-(int)leg)*theta;
	supLegRotfi=0;
	dribble();	
	theta=theta-legRotfi;
	turnright(theta);	
}

float Walk::turnleft(float theta)
{
	if(theta==0)
	{
		legRotfi=0;
		supLegRotfi=0;
		dribble();
		return 0;
	}
	if(theta>15)
		legRotfi=((int)leg)*15;
	else
		legRotfi=((int)leg)*theta;
	supLegRotfi=0;
	dribble();	
	theta=theta-legRotfi;
	turnleft(theta);}
/*
int Walk::setStrafe(double l_strafe)
{
	strafe = l_strafe;
	dribble();
	strafe = -l_strafe;
	dribble();
	strafe = 0;
	return 0;
}*/
int Walk::dribble()
{
	stepCount++;
	leg=(LEG)(1-(int)leg);	
//	printf("%d\t",leg);
	// if (leg==1)
		// legRotfi-=1;
	int fps = 120;
	int sleep = 1000000.0/(double)fps;
	double timeInc =1.0/(double)fps;
	double feetSeperation = 30;
	double hipLength = 130;
	
	double y_offset = 0;

	double Tc = sqrt(600.00/9810.0);
	legZin += hipLength/2;
	supLegZin += hipLength/2;
	
	///// desired Values 
	
	double D_dsp1Time = 0.05 + dz/(veloZfi);		//changed from 0.03
	double D_dsp2Time = 0.05 + dz/(veloZfi);		//""
	double sspZTime = sspTimeVar;
	// ////printf("Tc\t\t%lf\n",Tc);
	// ////printf("legZin\t\t%lf\n",legZin);
	// ////printf("supLegZin\t%lf\n",supLegZin);
	
	//// Z Control 

	/*double c1 = cosh(sspZTime/Tc) - 1;
	double s1 = sinh(sspZTime/Tc);
	double strafeFrac = strafe/sspZAmp; 
*/	
	// double sspZPhs = acosh((c1*strafeFrac + s1*sqrt(pow(strafeFrac,2) + 2*c1))/(2*c1));
	// double sspZTime  = Tc * (asinh(veloZfi*Tc/sspZAmp) - sspZPhs);
	double sspZAmp = zMax;
	double sspZPhs   = asinh(veloZin*Tc/sspZAmp);
	double sspZSupin = sspZAmp * cosh(sspZPhs);
	double sspZSupfi = sspZAmp * cosh(sspZTime/Tc + sspZPhs);
	double veloZfi_d = sspZAmp*sinh(sspZTime/Tc + sspZPhs)/Tc;
	
	// printf("VYin\t%lf\tVyfi_d\t%lf\n",veloYin,veloYfi);
	double sspTime   = sspZTime;
	double dsp1Time  = (supLegZin - sspZSupin)/(-veloZin);		
	double dsp2Time  = D_dsp2Time;
	double stepTime  = sspTime + dsp1Time + dsp2Time;
	
	double sspZin    = legZin + (-veloZin)*dsp1Time;
	double P_sspZAmp = zMax;
	double P_sspZTime = sspTimeVar;
	// double P_c1 = cosh(P_sspZTime/Tc) - 1;
	// double P_s1 = sinh(P_sspZTime/Tc);
	// double P_sspZAmp = sqrt(pow(strafe/P_c1,2) + 2*pow(veloZfi_d*Tc,2)/P_c1 + 2*strafe*P_s1*veloZfi_d*Tc/pow(P_c1,2));
	// printf("\n\nStrafe = %lf\n\n", strafe);
	double P_sspZPhs = asinh(-veloZfi_d*Tc/P_sspZAmp);	
	// double P_veloZfi

	// double P_sspZPhs = asinh(-veloZfi*Tc/P_sspZAmp);
	double P_sspZin  = P_sspZAmp*cosh(P_sspZPhs);
	double P_legZin  = P_sspZin - (-veloZfi_d)* D_dsp2Time;
	double P_veloZfi = P_sspZAmp*sinh(P_sspZTime/Tc + P_sspZPhs)/Tc;
	// double P_sspZTime= Tc * (asinh(P_veloZfi*Tc/P_sspZAmp) - P_sspZPhs);
	
	
	double sspZfi    = P_legZin + veloZfi_d*D_dsp1Time; 
	double legZfi    = sspZfi - veloZfi_d*dsp2Time;
	double supLegZfi = sspZSupfi + veloZfi_d*dsp2Time;
	
	//TODO add case in which dsp1Time becomes negative 
	//TODO add case where dsp2Time needs to be modified (if required)
	//TODO calculation of sspZfi for next step based on predicted change in velocity for the next step 
	/*printf("legZin\t\t%lf\n",legZin);
	printf("supLegZin\t\t%lf\n",supLegZin);
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
	printf("VeloZin\t%lf\n", veloZin);
	printf("VeloZfi\t%lf\n", P_veloZfi);
	printf("VeloZfi_d\t%lf\n", veloZfi_d);

	printf("P_sspZPhs\t\t%lf\n",P_sspZPhs);
	printf("P_sspZAmp\t\t%lf\n",P_sspZAmp);
	printf("P_sspZin\t\t%lf\n", P_sspZin);
*/	// Y Control 
	
	legYin    = -legYin;
	supLegYin = -supLegYin;
	
	double sspYin    = legYin + veloYin * dsp1Time;
	double sspYSupin = supLegYin + veloYin * dsp1Time;

	//Commented equations pertain to non-general (old) y motion
	
	double Ay = sspYSupin;
	double By = veloYin*Tc;

	double veloYfi_d = Ay*sinh(sspTime/Tc)/Tc + By*cosh(sspTime/Tc)/Tc;

	double P_By = veloYfi_d*Tc;
	double P_Ay = (veloYfi - veloYfi_d*cosh(sspTime/Tc))*Tc/sinh(sspTime/Tc);

	double P_sspYin = P_Ay;
	// double sspYAmp   = veloYin * sqrt(pow(Tc,2) - pow(sspYSupin/veloYin,2));
	// double sspYPhs   = asinh((sspYSupin/veloYin)/sqrt(pow(Tc,2) - pow(sspYSupin/veloYin,2)));
	// double veloYfi_d   = sspYAmp/Tc * cosh (sspTime/Tc + sspYPhs);
	//////printf("**************%lf******************\n",sspYAmp/Tc * cosh (sspTime/Tc + sspYPhs));
	// double P_sspYAmp = sgn(veloYin) * Tc * sqrt((2*cosh(P_sspZTime/Tc)*veloYfi_d*veloYfi - pow(veloYfi_d,2) - pow(veloYfi,2))/(pow(sinh(P_sspZTime/Tc),2)));
	// double P_sspYPhs = -acosh(veloYfi_d*Tc/P_sspYAmp);
	// double P_sspYin  = P_sspYAmp * sinh(P_sspYPhs);
	double P_legYin  = P_sspYin - veloYfi_d*D_dsp1Time;

	double sspYfi    = P_legYin - veloYfi_d*dsp2Time;
	double sspYSupfi = Ay*cosh(sspTime/Tc) + By*sinh(sspTime/Tc);
	// double sspYSupfi = sspYAmp * sinh(sspTime/Tc + sspYPhs);
	
	double legYfi    = sspYfi + veloYfi_d*dsp2Time;
	double supLegYfi = sspYSupfi + veloYfi_d*dsp2Time;
	
/*	// printf("Yeh raha%f\n", sspZTime);
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
*/	// Y (cubic)
	
	double a = ((-veloYfi-veloYin)-2*(-sspYfi+sspYin)/sspTime)/pow(sspTime,2);
	double b = ((-sspYfi+sspYin)/sspTime+veloYin-a*pow(sspTime,2))/sspTime;
	double c = -veloYin;
	double d = -sspYin;
	
	double height = 400;
	//double lift   = 30;
	double xfreq  = 2*pi;
	double displacement = 1;
	double startX = dsp1Time-1.0/60.0;
	double stopX  = stepTime-dsp2Time + 1.0/60.0;
	double xPhase =-pi/2;
	
	double dampLift = 3;
	double dampFreq = 2*pi;
	double dampStart = dsp1Time + sspTime - 4.0/60.0;
	double dampEnd   = stepTime;
	double dampPhase = -pi/2;
	double dampDisplacement = 1;
	
	double fraction;
	
	double x,xr,y,yr,z,zr,phi,phiR,s,sr;
	double yReal,yrReal,zReal,zrReal;
	double walkTime =0.0;
	int walk_i = 0;
	int target_i = 0;
	double *fval;
	//compliance(leg,10,10);
	double integ_term_z = 0, deriv_term_z = 0, prop_term_z = 0, err_z = 0, correction_z = 0;
	double integ_term_y = 0, deriv_term_y = 0, prop_term_y = 0, err_y = 0, correction_y = 0; 
	
	int state = DSP;

	//Experimental
/*	double curr_imu_yaw = bot->getImuYaw();
	//Imu yaw negative change if turned to the 'right'
	printf("Imu change = %f\t\n", curr_imu_yaw - prev_imu_yaw);
	if (fabs(curr_imu_yaw - prev_imu_yaw) < 5)
	{
		if (leg == RIGHT)
			legRotfi = (curr_imu_yaw - prev_imu_yaw);
		else
			legRotfi = -(curr_imu_yaw - prev_imu_yaw);
	}
	else
	{
		legRotfi = 0;
	}
	prev_imu_yaw = curr_imu_yaw;
*/
	double mean_y, mean_z;
	if (prev_mean_y != 0)
	{
		prev_mean_y /= (double)stepCount;
		// printf("Mean values = %f ",prev_mean_y);	
		mean_y = prev_mean_y;
		prev_mean_y *= (double)stepCount;
	}
	else
	{
		mean_y = 12;
	}
	if (prev_mean_z != 0)
	{
		prev_mean_z /= (double)stepCount;
		// printf("%f \n",prev_mean_z);
		mean_z = prev_mean_z;
		prev_mean_z *= (double)stepCount;
	}
	else
		mean_z = 0;
//	////printf("*************************************** STEP **********************************************\n");	
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
	//		////printf("DSP1\t");
			state = DSP;
		}
		else if (walkTime >= dsp1Time && walkTime <= dsp1Time + sspTime)
		{
			fraction=2*(walkTime-startX)/(stopX-startX);
			x  = height - lift * (sin(xfreq*((walkTime-startX)/(stopX-startX))+xPhase) + displacement)/(1+displacement);
			// x = height- lift*fraction*(2-fraction);
			xr = height ;//- 10*sin(pi*(walkTime - dsp1Time)/sspTime);
			y=a*pow(walkTime-dsp1Time,3)+b*pow(walkTime-dsp1Time,2)+c*(walkTime-dsp1Time)+d;
			//y  = linear(-sspYin,-sspYfi, ((walkTime-dsp1Time)/sspTime);
			yr = -(Ay*cosh((walkTime-dsp1Time)/Tc) + By*sinh((walkTime-dsp1Time)/Tc));
			// yr = -sspYAmp * sinh((walkTime-dsp1Time)/Tc +sspYPhs);
			z  = scurve(sspZin,sspZfi, walkTime-dsp1Time,sspTime) - hipLength/2;
			zr =  sspZAmp * cosh((walkTime-dsp1Time)/Tc + sspZPhs) -hipLength/2;
			phi= scurve(legRotin,legRotfi,walkTime-dsp1Time,sspTime);
			phiR=scurve(supLegRotin,supLegRotfi,walkTime-dsp1Time,sspTime);
			state = SSP;
	//		////printf("SSP0\t");
		}
		else if (walkTime > dsp1Time+sspTime )//&& walkTime<=stepTime
		{
			state = DSP;
			x  = height - dampLift* (sin(dampFreq*((walkTime-dampStart)/(dampEnd-dampStart))+dampPhase)+dampDisplacement)/(1+dampDisplacement);
			xr = height;
/*			const double (&COM)[AXES] = bot->getRotCOM();
			if (walkTime == (stepTime) && fabs(COM[2] - mean_z) > 0.5 )
				walkTime--;*/
			y  = -sspYfi - veloYfi_d*(walkTime-dsp1Time-sspTime);
			yr = -sspYSupfi - veloYfi_d*(walkTime-dsp1Time-sspTime);
			z  = sspZfi - veloZfi*(walkTime-dsp1Time-sspTime) -hipLength/2;
			zr = sspZSupfi + veloZfi*(walkTime-dsp1Time-sspTime) - hipLength/2;
			phi= legRotfi;
			phiR= supLegRotfi;
	//		////printf("DSP2\t");
		}
		else 
		{
	//		////printf("************* SOMETHING WRONG*******************\n");
		}
		// printf("X\t%3.1lf\tXR\t%3.1lf\tY\t%lf\tYR\t%lf\tZ\t%lf\tZR\t%lf\n",x,xr,y,yr,z,zr);
		////printf("Y\t%lf\tYR\t%lf\tZ\t%lf\tZR\t%lf\tP\t%lf\tPR\t%lf\n",y,yr,z+s,zr+sr,phi,phiR);
		///////printf("Z\t%lf\tZR\t%lf\n",z,zr);
//		printf("W phi\t%lf\tphiR\t%lf\tZ\t%lf\tZR\t%lf\tY\t%lf\tYR\t%lf",phi,phiR,z,zr,y,yr);
		const double (&COM)[AXES] = bot->getRotCOM();
		// bot->printRotCOM();
		// bot->printCOM();
		prev_mean_y += COM[1]/(stepTime*(double)fps);
		prev_mean_z += COM[2]/(stepTime*(double)fps);

		double err_new_z = COM[2] - mean_z;
		deriv_term_z = deriv_const_z*(err_new_z - err_z)/timeInc;
		prop_term_z = prop_const_z*err_new_z;

		// printf("Velocity_Z = %f\n", (err_new_z - err_z)/timeInc);

		double integ_new_z = integ_term_z + integ_const_z*(err_z + err_new_z)*timeInc/2;
		
		// printf(" P = %f I = %f D = %f\n",err_new_z,integ_new_z,(err_new_z - err_z)/timeInc);

		if (!(integ_new_z > integ_max_z || integ_new_z < -integ_max_z))
			integ_term_z = integ_new_z;
		
		correction_z = deriv_term_z + integ_term_z + prop_term_z;
		err_z = err_new_z;
		// printf("Mean Values %f %f \n", mean_y, mean_z);
		double err_new_y = COM[1] - mean_y;

		// bot->printRotCOM();

		deriv_term_y = deriv_const_y*(err_new_y - err_y)/timeInc;
		prop_term_y = prop_const_y*err_new_y;
		double integ_new_y = integ_term_y + integ_const_y*(err_y + err_new_y)*timeInc/2;
		if (!(integ_new_y > integ_max_y || integ_new_y < -integ_max_y))
			integ_term_y = integ_new_y;
		
		correction_y = deriv_term_y + integ_term_y + prop_term_y;
		// correction_y = prop_term_y;
		err_y = err_new_y;
		// correction = 0;

		//PROPER 
		bot->leg[leg]->runIK(x,y - correction_y,z+feetSeperation + correction_z,phi);
		bot->leg[1-leg]->runIK(xr,yr - correction_y,zr+feetSeperation - correction_z,phiR);
		
		printf("%f\n",COM[2]);
		//NON IMU			
		// bot->leg[leg]->runIK(x,y,z+feetSeperation ,phi);
		// bot->leg[1-leg]->runIK(xr,yr ,zr+feetSeperation,phiR);
		//Y IMU
		// bot->leg[leg]->runIK(x,y - correction_y,z+feetSeperation ,phi);
		// bot->leg[1-leg]->runIK(xr,yr - correction_y,zr+feetSeperation,phiR);
		//On spot
		// bot->leg[leg]->runIK(x,0,z+feetSeperation - correction_z,phi);
		// bot->leg[1-leg]->runIK(xr,0,zr+feetSeperation + correction_z,phiR);
				
		// printf("Z : Integral= %f\tDerivative = %f\tProportion = %f\tCorrection = %f\n",integ_term_z, deriv_term_z, prop_term_z, correction_z);
		// printf("Y : Integral= %f\tDerivative = %f\tProportion = %f\tCorrection = %f\n\n",integ_term_y, deriv_term_y, prop_term_y, correction_y);

		// bot->leg[leg]->runIK(x,y- (COM[1] - 11),z+feetSeperation-0*COM[2],phi);
		// bot->leg[1-leg]->runIK(xr,yr - (COM[1] - 11),zr+feetSeperation+0*COM[2],phiR);
			

		// bot->printRotCOM(); 
		// bot->leg[leg]->runIK(x,y+y_offset-(COM[1] - 12),z+feetSeperation ,phi);
		// bot->leg[1-leg]->runIK(xr,yr +y_offset- (COM[1] - 12),zr+feetSeperation,phiR);
		// printf("x = %f y = %f z = %f \n", COM[0], COM[1], COM[2]);
		// bot->leg[leg]->runIK(x,y,z+feetSeperation ,phi);
		// bot->leg[1-leg]->runIK(xr,yr,zr+feetSeperation ,phiR);
		// bot->leg[leg]->runIK(x,0,z+feetSeperation +COM[2],phi);
		// bot->leg[1-leg]->runIK(xr,0,zr+feetSeperation-COM[2] ,phiR);
		// bot->leg[leg]->runIK(x,COM[1] - 15,z+feetSeperation-2*COM[2],phi);
		// bot->leg[1-leg]->runIK(xr,
		// //printf("%d %d\n\n\n",leg,1-leg);
		// printf("%f %f %f\n",COM[1] - 11,COM[1],COM[2] );
		//in use
		// bot->leg[leg]->runIK(x,y - (COM[1] - 11),z+feetSeperation-0*COM[2],phi);
		// bot->leg[1-leg]->runIK(xr,yr - (COM[1] - 11),zr+feetSeperation+0*COM[2],phiR);
		//endinuse
		
		// bot->leg[leg]->runIK(x,y- (COM[1] - 11),z+feetSeperation-0*COM[2],phi);
		// bot->leg[1-leg]->runIK(xryr0 - (COM[1] - 11),zr+feetSeperation+0*COM[2],phiR);
		

		// //printf("%d %d\n\n\n",leg,1-leg);
		// printf("%f %f %f\n",COM[1] - 15,COM[1],COM[2] );
		bot->updateBot();
		//printf("Sent Values\n");
		//leg->getMotorLoad(FOOT_ROLL);
		//revLeg->getMotorLoad(FOOT_ROLL);

		usleep(sleep);
		//getchar();
		
	}
	// printf("%f\n",-sspYfi + sspYin );
	supLegYin  = -legYfi;
	legYin = -supLegYfi;
	veloYin=veloYfi_d;
	supLegZin = supLegZfi - hipLength/2;
	legZin = legZfi - hipLength/2;
	veloZin = -veloZfi_d;
	veloZfi= P_veloZfi;  
	supLegRotin  = legRotfi;
	legRotin = supLegRotfi;
	// sspZAmp = P_sspZAmp;
	return EXIT_SUCCESS;//endState;
	
}

int Walk::pathdribble(double vel_y, double dx, double t1, double t2)
{
	veloYfi = vel_y;
	dz = dx;
	legRotfi = t1;
	supLegRotfi = t2;
	int x = dribble();
	//Resetting z and theta to zero for safety
	dz = 0;
	legRotfi = 0;
	supLegRotfi = 0;
	return x;
}

int Walk::dribble(double dy, double dx, double t1, double t2)
{
	leg=(LEG)(1-(int)leg);	
	printf("%d\t",leg);
	
	printf("X:%lf Y:%lf T1:%lf T2:  %lf\n\n",dx,dy,t1,t2);
	
	printf("VYin\t%lf\tVyfi_d\t%lf\n",veloYin,veloYfi_d);
	int fps = 120;
	int sleep = 1000000.0/(double)fps;
	double timeInc =1.0/(double)fps;
	double feetSeperation = 0;
	double hipLength = 130;
	
	double Tc = sqrt(600.00/9810.0);
	legZin += hipLength/2;
	supLegZin += hipLength/2;
	
	
	legRotfi=t1;
	supLegRotfi=t2;
	///// desired Values 
	
	double D_dsp1Time = 0.05+dx/(veloZfi);
	double D_dsp2Time = 0.05+dx/(veloZfi);
	
	// ////printf("Tc\t\t%lf\n",Tc);
	// ////printf("legZin\t\t%lf\n",legZin);
	// ////printf("supLegZin\t%lf\n",supLegZin);
	
	//// Z Control 
	
	double sspZAmp   = zMax; 
	double sspZPhs   = asinh(veloZin*Tc/sspZAmp);
	double sspZTime  = Tc * (asinh(veloZfi*Tc/sspZAmp) - sspZPhs);
	double sspZSupin = sspZAmp * cosh(sspZPhs);
	double sspZSupfi = sspZAmp * cosh(sspZTime/Tc + sspZPhs);
	
	double sspTime   = sspZTime;
	double dsp1Time  = (supLegZin - sspZSupin)/(-veloZin);		
	double dsp2Time  = D_dsp2Time;
	double stepTime  = sspTime + dsp1Time + dsp2Time;
	
	double sspZin    = legZin + (-veloZin)*dsp1Time;
	double P_sspZAmp = zMax;
	double P_sspZPhs = asinh(-veloZfi*Tc/P_sspZAmp);
	double P_sspZin  = P_sspZAmp * cosh(P_sspZPhs);
	double P_legZin  = P_sspZin - (-veloZfi)* D_dsp2Time;
	double P_veloZfi = veloZfi + (veloZfi + veloZin)/2;
	double P_sspZTime= Tc * (asinh(P_veloZfi*Tc/P_sspZAmp) - P_sspZPhs);
	
	
	double sspZfi    = P_legZin + veloZfi*D_dsp1Time; 
	double legZfi    = sspZfi - veloZfi*dsp2Time;
	double supLegZfi = sspZSupfi + veloZfi*dsp2Time;
	
	//TODO add case in which dsp1Time becomes negative 
	//TODO add case where dsp2Time needs to be modified (if required)
	//TODO calculation of sspZfi for next step based on predicted change in velocity for the next step 
	
	////printf("zMax\t\t%lf\n",zMax);
	////printf("sspZPhs\t\t%lf\n",sspZPhs);
	////printf("sspZTime\t%lf\n",sspZTime);
	////printf("sspZin\t\t%lf\n",sspZin);
	////printf("sspZSupin\t%lf\n",sspZSupin);
	////printf("sspZfi\t\t%lf\n",sspZfi);
	////printf("sspZSupfi\t%lf\n",sspZSupfi);
	////printf("dsp1Time\t%lf\n",dsp1Time);
	////printf("stepTime\t%lf\n",stepTime);
	////printf("LegZfi\t\t%lf\n",legZfi);
	////printf("supLegZfi\t%lf\n",supLegZfi);
	
	// Y Control 
	
	legYin    = -legYin;
	supLegYin = -supLegYin;
	
	double sspYin    = legYin + veloYin * dsp1Time;
	double sspYSupin = supLegYin + veloYin * dsp1Time;
	double sspYAmp   = veloYin * sqrt(pow(Tc,2) - pow(sspYSupin/veloYin,2));
	double sspYPhs   = asinh((sspYSupin/veloYin)/sqrt(pow(Tc,2) - pow(sspYSupin/veloYin,2)));
	double veloYfi_d   = sspYAmp/Tc * cosh (sspTime/Tc + sspYPhs);
	printf("VYD-%lf\n",veloYfi_d);
	//////printf("**************%lf******************\n",sspYAmp/Tc * cosh (sspTime/Tc + sspYPhs));
	//double P_sspYAmp = sgn(veloYin) * Tc * sqrt((2*cosh(P_sspZTime/Tc)*veloYfi_d*veloYfi - pow(veloYfi_d,2) - pow(veloYfi,2))/(pow(sinh(P_sspZTime/Tc),2)));
	//double P_sspYPhs = -acosh(veloYfi_d*Tc/P_sspYAmp);
	//double P_sspYin  = P_sspYAmp * sinh(P_sspYPhs);
	double P_sspYin  = (-sinh(P_sspZTime/Tc)*veloYfi_d*Tc + dy)/(cosh(P_sspZTime/Tc)-1);	
	double P_sspYAmp = sgn(veloYfi_d)*sqrt(-pow(P_sspYin,2)+pow(veloYfi_d*Tc,2));
	double P_sspYPhs = asinh(P_sspYin/P_sspYAmp);
	double P_legYin  = P_sspYin - veloYfi_d*D_dsp1Time;
	double P_veloYfi= (P_sspYAmp/Tc)*cosh((P_sspZTime/Tc)+P_sspYPhs);
	double P_sspYfi = P_sspYAmp*sinh((P_sspZTime/Tc)+P_sspYPhs);

	double sspYfi    = P_legYin - veloYfi_d*dsp2Time;
	double sspYSupfi = sspYAmp * sinh(sspTime/Tc + sspYPhs);
	
	double legYfi    = sspYfi + veloYfi_d*dsp2Time;
	double supLegYfi = sspYSupfi + veloYfi_d*dsp2Time;
	/*
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
	printf("P_veloYfi\t\t%lf\n",P_veloYfi);
	printf("P_sspYfi\t\t%lf\n",P_sspYfi);
	// Y (cubic)
	*/ //Uncommented for behavior testing
	double a = ((-veloYfi_d-veloYin)-2*(-sspYfi+sspYin)/sspTime)/pow(sspTime,2);
	double b = ((-sspYfi+sspYin)/sspTime+veloYin-a*pow(sspTime,2))/sspTime;
	double c = -veloYin;
	double d = -sspYin;
	// printf("abcd = %f\t%f\t%f\t%f\n " , a , b , c , d);
	// printf("sspTime = %f\n" , sspTime);
	double height = 400;
	//double lift   = 30;
	double xfreq  = 2*pi;
	double displacement = 1;
	double startX = dsp1Time-1.0/60.0;
	double stopX  = stepTime-dsp2Time + 1.0/60.0;
	double xPhase =-pi/2;
	
	double dampLift = 0;
	double dampFreq = 2*pi;
	double dampStart = dsp1Time + sspTime - 4.0/60.0;
	double dampEnd   = stepTime;
	double dampPhase = -pi/2;
	double dampDisplacement = 1;
	
	double fraction;
	
	double x,xr,y,yr,z,zr,phi,phiR,s,sr;
	double yReal,yrReal,zReal,zrReal;
	double walkTime =0.0;
	int walk_i = 0;
	int target_i = 0;
	double *fval;
	//compliance(leg,10,10);
	
	
	int state = DSP;
//	////printf("*************************************** STEP **********************************************\n");	
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
	//		////printf("DSP1\t");
			state = DSP;
		}
		else if (walkTime >= dsp1Time && walkTime <= dsp1Time + sspTime)
		{
			fraction=2*(walkTime-startX)/(stopX-startX);
			//x  = height - lift * (sin(xfreq*((walkTime-startX)/(stopX-startX))+xPhase) + displacement)/(1+displacement);
			x = height- lift*fraction*(2-fraction);
			xr = height;
			y=a*pow(walkTime-dsp1Time,3)+b*pow(walkTime-dsp1Time,2)+c*(walkTime-dsp1Time)+d;
			//y  = linear(-sspYin,-sspYfi, ((walkTime-dsp1Time)/sspTime);
			yr = -sspYAmp * sinh((walkTime-dsp1Time)/Tc +sspYPhs);
			z  = scurve(sspZin,sspZfi, walkTime-dsp1Time,sspTime) - hipLength/2;
			zr =  sspZAmp * cosh((walkTime-dsp1Time)/Tc + sspZPhs) -hipLength/2;
			phi= scurve(legRotin,legRotfi,walkTime-dsp1Time,sspTime);
			phiR=scurve(supLegRotin,supLegRotfi,walkTime-dsp1Time,sspTime);
			state = SSP;
	//		////printf("SSP0\t");



		}
		else if (walkTime > dsp1Time+sspTime )//&& walkTime<=stepTime
		{
			state = DSP;
			x  = height;// - dampLift* (sin(dampFreq*((walkTime-dampStart)/(dampEnd-dampStart))+dampPhase)+dampDisplacement)/(1+dampDisplacement);
			xr = height;
			y  = -sspYfi - veloYfi_d*(walkTime-dsp1Time-sspTime);
			yr = -sspYSupfi - veloYfi_d*(walkTime-dsp1Time-sspTime);
			z  = sspZfi - veloZfi*(walkTime-dsp1Time-sspTime) -hipLength/2;
			zr = sspZSupfi + veloZfi*(walkTime-dsp1Time-sspTime) - hipLength/2;
			phi= legRotfi;
			phiR= supLegRotfi;
	//		////printf("DSP2\t");
		}
		else 
		{
	//		////printf("************* SOMETHING WRONG*******************\n");
		}
		///////printf("X\t%3.1lf\tXR\t%3.1lf\tY\t%lf\tYR\t%lf\tZ\t%lf\tZR\t%lfP\t%lf\tPR\t%lf\n",x,xr,y,yr,z,zr,phi,phiR);
		////printf("Y\t%lf\tYR\t%lf\tZ\t%lf\tZR\t%lf\tP\t%lf\tPR\t%lf\n",y,yr,z+s,zr+sr,phi,phiR);
		///////printf("Z\t%lf\tZR\t%lf\n",z,zr);
		// printf("phi\t%lf\tphiR\t%lf\tZ\t%lf\tZR\t%lf\tY\t%lf\tYR\t%lf\n",phi,phiR,z,zr,y,yr); Uncommented for behavior testing
		
		bot->leg[leg]->runIK(x,y,z+feetSeperation,phi);
		bot->leg[1-leg]->runIK(xr,yr,zr+feetSeperation,phiR);
		//printf("%d %d\n\n\n",leg,1-leg);
		bot->updateBot();
		//printf("Sent Values\n");
		//leg->getMotorLoad(FOOT_ROLL);
		//revLeg->getMotorLoad(FOOT_ROLL);

		usleep(sleep);
		//getchar();
		
	}
	supLegYin  = -legYfi;
	legYin = -supLegYfi;
	veloYin=veloYfi_d;
	supLegZin = supLegZfi - hipLength/2;
	legZin = legZfi - hipLength/2;
	veloZin = -veloZfi;
	veloZfi= veloZfi;	
	supLegRotin  = legRotfi;
	legRotfi=0;
	legRotin = supLegRotfi;
	supLegRotfi=0;
	this->veloYfi_d = veloYfi_d;
	return EXIT_SUCCESS;//endState;
}
