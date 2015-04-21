#include "walk.h"
int k=1;
double pi = acos(-1);
Walk::Walk(AcYut* bot)
{
	this->bot = bot;

	Tc = sqrt(600.00/9810.0);
	C = 1/Tc;
	feetSeparation = 50;
	hipLength = 130;
	height = 400;

	legYin=-1.773347;
	supLegYin=-4.571982;
	veloYin=-0.611621;
	veloYfi=10;
	zMax=40;
	dz = 0;
	dspTime = 0.07;
	lift=70;
	legRotin=0;
	legRotfi=0;
	supLegRotin=0;
	supLegRotfi=0;
	sspTimeVar = 0.290999;
	veldef = C*zMax*(exp(-C*sspTimeVar/2) - exp(C*sspTimeVar/2))/2;
	veloZin = veldef;
	veloZfi = -veldef;
	supLegZin = zMax*(exp(-C*sspTimeVar/2) + exp(C*sspTimeVar/2))/2 - dspTime*veloZin - hipLength/2;//25.155276;
	legZin = supLegZin;//25.155276;
	correction_factor = 0;
	// strafe=0;
	// sspZAmp=zMax;
	stepCount = 0;

	integ_const_z =0.5;//0.05;//0.05;//2;
	deriv_const_z =0.8/120;//1.2/120.0;//0.01;//0.005;
	prop_const_z = 1;//0.6;//1;
	integ_max_z = 10;
	prev_mean_z = 0;
	integ_term_z = 0;

	integ_const_y = 0;//1;//1;//1;
	deriv_const_y = 0;//0.005;//0.005;
	prop_const_y = 0;//1;//1;
	integ_max_y = 2;
	integ_term_y;
	prev_mean_y = 0;
	prev_imu_yaw = 0;
	
	// ofstream fis;
	// fis.open("Source/comoffset.txt");
	// fis.close();
/*	ifstream fil;
	
	fil.open("Source/walk/comoffset.txt");
	if (fil.is_open())
	{
		int q = 0;
		while (!fil.eof()) 
		{
	    	fil>>com_offset[q++];
	    	// cout<<com_offset[q-1]<<endl;
		}
	}
	else
		cout<<"File not found\n";
	fil.close();
	
	ifstream fil1;
	
	fil1.open("Source/walk/comoffsety.txt");
	if (fil1.is_open())
	{
		int q = 0;
		while (!fil1.eof()) 
		{
	    	fil1>>com_offsety[q++];
	    	// cout<<com_offset[q-1]<<endl;
		}
	}
	else
		cout<<"File not found\n";
	fil1.close();*/
	
	start();
	printf("START completed\n\n\n");

};
int Walk::start()
{
	dribble(2);
	dribble(1);
}

int Walk::kick()
{
	


}

int Walk::pathdribble(double vel_y, double dz, double t1, double t2)
{
	if (vel_y != -1)
		veloYfi = vel_y;
	//dz is negative for left, positive for right
	//We want the strafing to occur by stepping outward followed by inward
	//which is handled by this conditional construct
	if (dz>0)
	{
		this->dz = dz;
		// if (leg == LEFT)
		// {
		// 	// dribble();	
		// 	// this->dz = dz;
		// }
		// else
		// 	this->dz = dz;
	}
	else if (dz<0)
	{
		if (leg==LEFT)
			this->dz = -dz;
		// else
		// {
		// 	dribble();
		// 	this->dz = -dz;
		// }
	}
	// this->dz = dz;
	// printf("%f %f\n", t1, t2);
	legRotfi = t1;
	supLegRotfi = t2;
	int x = dribble();
	//Resetting z and theta to zero for safety
	dz = 0;
	legRotfi = 0;
	supLegRotfi = 0;
	return x;
}

int Walk::sideMotion(double distance)
{
	stopMotion();
	if ((leg == LEFT && distance > 0 )|| (leg == RIGHT && distance < 0))
	{
		dribble();
	}
	// pathdribble(-20, 0, 0, 0);
	pathdribble(-30, 0, 0, 0);
	int j = 1;
	while((10.0*j) < fabs(distance))
	{
		if (j%2)
		{
			veloZfi = -veldef + 80;
		}
		else
		{
			veloZfi = -veldef;
		}
		j++;
		// cout<<"VeloZfi "<<veloZfi<<" supLegZin "<<legZin<<endl;
		dribble();
	}

	veloZfi = -veldef;


	pathdribble(0, 0, 0, 0);
	pathdribble(10, 0, 0, 0);
}


int Walk::stopMotion()
{
	if (veloYfi == 0)
		return 1;
	while (fabs(veloYfi) > 20)
	{		
		pathdribble(veloYfi/3, 0, 0, 0);
	}

	pathdribble(0, 0, 0, 0);
	return 1;
}

int Walk::backMotion(double distance)
{
	stopMotion();

	pathdribble(-10, 0, 0, 0);

	while (fabs(veloYfi) < 90)
	{
		accelerate();
		dribble();
		distance -= (-veloYfi*(sspTimeVar + 2*dspTime));
	}

	while (distance > 0)
	{
		dribble();
		distance -= (-veloYfi*(sspTimeVar + 2*dspTime));
	}
	stopMotion();
	pathdribble(10, 0, 0, 0);
	return leg;
}

int Walk::captureStepZ(double &c1_z, double &c2_z, double zMax, double dsp1Time, double dsp2Time, double cap_pos_actual, double cap_vel_actual, double walkTime, double &sspTime, double &z_a_free, double &z_b_free, double &z_c_free, double &legZfi, double &supLegZfi)
{

	double cap_sup_defpos = c1_z*exp(C*sspTime) + c2_z*exp(-C*sspTime); 
	double cap_sup_defvel = C*(c1_z*exp(C*sspTime) - c2_z*exp(-C*sspTime));
	// double cap_pos_actual = zMax - 1*(leg==1?1:-1)*COM[2];
	double cap_pos_ideal = c1_z*exp(C*(walkTime-dspTime)) + c2_z*exp(-C*(walkTime-dspTime));
	 cout<<"COM position should be "<<cap_pos_ideal<<endl;
	 cout<<"Actual COM position is "<<cap_pos_actual<<endl;
	 cout<<"Actual velocity is "<<cap_vel_actual<<endl;

	// cap_pos_actual = zMax;
	// cap_vel_actual = 0; 
	double cap_ideal_energy = (-pow(C,2)*pow(cap_pos_ideal,2))/2;
	 cout<<"Energy should be "<<cap_ideal_energy<<endl;
	double cap_actual_energy = (pow(cap_vel_actual,2)-pow(C,2)*pow(cap_pos_actual,2))/2; 
	 cout<<"Energy is "<<cap_actual_energy<<endl;

	if (fabs(cap_ideal_energy - cap_actual_energy) < 1000)
		return -1;
	double time_left = sspTime - (walkTime - dsp1Time);

	double cap_sup_vel =  C*cap_pos_actual*sinh(C*time_left) + cap_vel_actual*cosh(C*time_left) ;
	 cout<<"Velocity at support exchange should be "<<cap_sup_vel<<endl;
	double cap_sup_pos = sqrt(pow(zMax,2) + pow(cap_sup_vel,2)/pow(C,2));
	 cout<<"Default position at support exchange "<<cap_sup_defpos<<endl;
	 cout<<"Position of next step at support exchange should be "<<cap_sup_pos <<endl;

	 int flag = 0;
/*	if (fabs(cap_sup_pos - cap_sup_defpos) < 2)
	{
		cap_sup_pos = cap_sup_defpos;
		flag++;
	}
	if (fabs(cap_sup_vel - cap_sup_defvel) < 5)
	{
	 	cap_sup_vel = cap_sup_defvel;
	 	flag++;
	}
	if (flag == 2)
	{
		veloZfi = -veldef;
		return -1;
	}	*/
 	double veloZfi_d = cap_sup_vel;

	cout<<"Plotting new trajectories "<<endl;	
	c1_z = (cap_pos_actual + cap_vel_actual/C)*exp(-C*(walkTime-dsp1Time))/2;
	c2_z = (cap_pos_actual - cap_vel_actual/C)*exp(C*(walkTime-dsp1Time))/2;

	double sspZSupfi = c1_z*exp(C*sspTime) + c2_z*exp(-C*sspTime);
	cout<<"Original supLegZfi "<<supLegZfi<<endl;
	supLegZfi = sspZSupfi + veloZfi_d*dsp2Time;
	cout<<"Final supLegZfi "<<supLegZfi<<endl;

	double P_veloZin = -veloZfi_d;
	double P_sspZSupin = cap_sup_pos;
	double P_supLegZin  = P_sspZSupin - (P_veloZin)*dsp2Time;

	double P_c1_z = (P_sspZSupin + P_veloZin/C)/2;
	double P_c2_z = (P_sspZSupin - P_veloZin/C)/2;
	veloZfi = C*(P_c1_z*exp(C*sspTime) - P_c2_z*exp(-C*sspTime));
	// cout<<veloZfi<<" Look "<<endl;
	cout<<"Original legZfi "<<legZfi<<endl;
	legZfi    = P_supLegZin;
	cout<<"New legZfi "<<legZfi<<endl;

	z_c_free = legZfi;
	z_b_free = 4*(zMax - legZfi)/(sspTime+dsp1Time+dsp2Time);
	z_a_free = 4*(legZfi- zMax)/pow(sspTime+dsp1Time+dsp2Time,2);	
		// if (P_sspZSupin+feetSeparation + cap_sup_vel*dsp2Time> 130)
			// z_next_initial = 130-feetSeparation - cap_sup_vel*dsp2Time;
	return 1;
	// }
	// else
		// return -1;
}

int Walk::captureStepY(double &c1_y, double &c2_y, double dsp1Time, double dsp2Time, double yr, double vely, double y, double &supLegYfi, double &veloYfi_d, double cap_pos_actual, double cap_vel_actual, double walkTime,  double &sspTime, double &y_a_free, double &y_b_free, double &y_c_free, double &y_d_free )
{
	cout<<"Actual COM position is "<<cap_pos_actual<<endl;
	cout<<"Position should be "<<-yr<<endl;
	// cap_vel_actual = -vely;
	cout<<"Actual COM velocity is "<<cap_vel_actual<<endl;
	cout<<"Velocity should be "<<-vely<<endl;
	double cap_ideal_energy =  (pow(vely,2.0) - pow(C,2.0)*pow(yr,2.0))/2.0;
	double time_left = sspTime - (walkTime - dsp1Time);
	cout<<"Energy should be "<<cap_ideal_energy<<endl;
	double cap_actual_energy =  (pow(cap_vel_actual,2.0) - pow(C,2.0)*pow(cap_pos_actual,2.0))/2.0;
	cout<<"Energy is "<<cap_actual_energy<<endl;
	double cap_sup_vel =  C*cap_pos_actual*sinh(C*time_left) + cap_vel_actual*cosh(C*time_left) ;
	cout<<"Original velocity at support exchange "<<veloYfi_d<<endl;
	cout<<"Velocity required at support exchange "<<cap_sup_vel<<endl;
	double cap_sup_pos = cap_sup_vel*tanh(C*time_left)/C + cap_pos_actual;
	double cap_sup_defpos = c1_y*exp(C*sspTime) + c2_y*exp(-C*sspTime);
	cout<<"Position of next step at support exchange should be "<<cap_sup_pos <<endl;
	cout<<"Original position at support exchange "<<cap_sup_defpos<<endl;

	if (cap_sup_pos > 45)
		cap_sup_pos = 45;

	veloYfi_d = cap_sup_vel;
	if (veloYfi_d > 260)
		veloYfi_d = 260;
	c1_y = (cap_sup_pos + veloYfi_d/C)*exp(-C*sspTime)/2;
	c2_y = (cap_sup_pos - veloYfi_d/C)*exp(C*sspTime)/2;

	supLegYfi = cap_sup_pos + veloYfi_d*dsp2Time;



	// double y_free_fi = cap_sup_pos - veloYfi_d*(dsp1Time + dsp2Time);
	// if (y_free_fi > 90)
	// 	y_free_fi = 90;
	// cout<<"New free leg target "<<y_free_fi<<endl;
	// Matrix<double, 4, 4> A;
	// Matrix<double, 4, 1> B, X;

	// A << pow(sspTime/2,3),pow(sspTime/2,2),pow(sspTime/2,1),1,
	// 	 pow(sspTime,3),pow(sspTime,2),pow(sspTime,1),1,
	// 	 3*pow(sspTime/2,2),2*pow(sspTime/2,1),1,0,
	// 	 3*pow(sspTime,2),2*pow(sspTime,1),1,0,
	// B << y,y_free_fi, cap_vel_actual, -veloYfi_d;
	// X = A.colPivHouseholderQr().solve(B);

	// y_a_free = X(0);
	// y_b_free = X(1);
	// y_c_free = X(2);
	// y_d_free = X(3);

	// legYfi = y_free_fi + veloYfi_d*dsp2Time;
	// cout<<"legYfi c"<<legYfi<<endl;
}

int Walk::handMotion(double handSwing)
{
	int arr_l[4], arr_r[4];
	// cout<<"Hand Swing = "<<handSwing<<endl;
	bot->left_hand->getGoalPositionSoft(arr_l);
	bot->right_hand->getGoalPositionSoft(arr_r);
	if (leg==RIGHT)
	{
		arr_l[0] = 4096 - handSwing;
		// arr_r[0] = arr_l[0] + 50;
	}
	else
	{
		arr_r[0] = handSwing;
		// arr_l[0] = arr_r[0] - 50;
	}
	bot->left_hand->setGoalPositionSoft(arr_l);
	bot->right_hand->setGoalPositionSoft(arr_r);

}
float Walk::accelerate()
{
	if (veloYfi == 0)
		veloYfi = 10;
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
int Walk::getLeg()
{
	return leg;
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
	turnleft(theta);
}
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
int Walk::dribble(int flag)
{

	stepCount++;
	leg=(LEG)(1-(int)leg);	
//	printf("%d\t",leg);
	// if (leg==1)
		// legRotfi-=1;
	int fps = 120;
	int sleep = 1000000.0/(double)fps;
	double timeInc =1.0/(double)fps;
	// double feetSeparation = 30;
	
	double y_offset = 0;

	legZin += hipLength/2;
	supLegZin += hipLength/2;
	///// desired Values 
	
	double D_dsp1Time = dspTime;	// + dz/(veloZfi);		//changed from 0.03
	double D_dsp2Time = dspTime;	//+ dz/(veloZfi);		//""
	double sspZTime = sspTimeVar;
	// ////printf("Tc\t\t%lf\n",Tc);
	// printf("legZin\t\t%lf\n",legZin);
	// printf("supLegZin\t%lf\n",supLegZin);
	
	//// Z Control 

	/*double c1_z = cosh(sspZTime/Tc) - 1;
	double s1 = sinh(sspZTime/Tc);
	double strafeFrac = strafe/sspZAmp; 
*/	
	// double sspZPhs = acosh((c1_z*strafeFrac + s1*sqrt(pow(strafeFrac,2) + 2*c1_z))/(2*c1_z));
	// double sspZTime  = Tc * (asinh(veloZfi*Tc/sspZAmp) - sspZPhs);
	// double sspZAmp = zMax;
	// double sspZPhs   = asinh(veloZin*Tc/sspZAmp);
	// double sspZSupin = sspZAmp * cosh(sspZPhs);
	// double sspZSupfi = sspZAmp * cosh(sspZTime/Tc + sspZPhs);
	// double veloZfi_d = sspZAmp*sinh(sspZTime/Tc + sspZPhs)/Tc;

	//exponential equations
	// double C = 1.0/Tc;
	double sspZSupin  = supLegZin + (veloZin)* D_dsp2Time;
	double sspZin = legZin - veloZin*D_dsp2Time;
	double c1_z = (sspZSupin + veloZin/C)/2;
	double c2_z = (sspZSupin - veloZin/C)/2;

	// double c1_z = zMax*exp(-C*sspZTime/2.0)/2.0;
	// double c2_z = zMax*exp(C*sspZTime/2.0)/2.0;
	// double sspZSupin = c1_z + c2_z;
	double sspZSupfi = c1_z*exp(C*sspZTime) + c2_z*exp(-C*sspZTime);
	double veloZfi_d = C*(c1_z*exp(C*sspZTime) - c2_z*exp(-C*sspZTime));

	double sspTime   = sspZTime;
	double dsp1Time  = D_dsp1Time;		
	double dsp2Time  = D_dsp2Time;
	double stepTime  = sspTime + dsp1Time + dsp2Time;
	
	// double P_sspZAmp = zMax;
	double P_sspZTime = sspTimeVar;
	double P_veloZin = -veloZfi_d;
	// veloZfi = -veldef;	//Returning to default trajectory
	double P_c1_z = (veloZfi - P_veloZin*exp(-C*P_sspZTime))/(C*(exp(C*P_sspZTime) - exp(-C*P_sspZTime)));
	double P_c2_z = P_c1_z - P_veloZin/C;
	// cout<<"P_velozin "<<P_veloZin<<endl;
	// cout<<"veldef "<<veldef<<endl;
	veloZfi = -veldef;	//Returning to default trajectory
	// P_c1_z = zMax*exp(-C*sspZTime/2.0)/2.0;
	// P_c2_z = zMax*exp(C*sspZTime/2.0)/2.0;
	
	// double P_c1_z = cosh(P_sspZTime/Tc) - 1;
	// double P_s1 = sinh(P_sspZTime/Tc);
	// double P_sspZAmp = sqrt(pow(strafe/P_c1_z,2) + 2*pow(veloZfi_d*Tc,2)/P_c1_z + 2*strafe*P_s1*veloZfi_d*Tc/pow(P_c1_z,2));
	// printf("\n\nStrafe = %lf\n\n", strafe);
	// double P_sspZPhs = asinh(-veloZfi_d*Tc/P_sspZAmp);	
	// double P_sspZPhs = asinh(-veloZfi*Tc/P_sspZAmp);

	double P_sspZSupin  = P_c1_z + P_c2_z;
	double P_supLegZin  = P_sspZSupin - (P_veloZin)* D_dsp2Time;
	// double P_veloZfi = C*(P_c1_z - P_c2_z);
	// double P_sspZTime= Tc * (asinh(P_veloZfi*Tc/P_sspZAmp) - P_sspZPhs);
	
	// double sspZfi    = P_supLegZin + veloZfi_d*dsp1Time + dz; //Make this an input for strafe
	double sspZfi    = P_supLegZin - P_veloZin*dsp1Time;
	double legZfi    = sspZfi + P_veloZin*dsp2Time;
	double supLegZfi = sspZSupfi - P_veloZin*dsp2Time;
	
	// double c1_z_swing = (sspZin - sspZfi*exp(-C*sspTimeVar))/(exp(C*sspTimeVar) - exp(-C*sspTimeVar));
	// double c2_z_swing = sspZin - c1_z_swing;

/*	double z_c_free = legZin;
	double z_a_free = (2*(legZfi + legZin) - 4*(zMax))/pow(sspTime+dsp1Time+dsp2Time,2);
	double z_b_free = 2*((legZfi- legZin)/2 - z_a_free*pow(sspTime+dsp1Time+dsp2Time,2)/2)/(sspTime+dsp1Time+dsp2Time);

*/
	Matrix<double, 5, 5> A;
	Matrix<double, 5, 1> B, X;

	A << 0,0,0,0,1,
		 0,0,0,1,0,
		 pow(sspTime,4),pow(sspTime,3),pow(sspTime,2),pow(sspTime,1),1,
		 4*pow(sspTime,3),3*pow(sspTime,2),2*pow(sspTime,1),1,0,
		 pow(sspTime/2,4),pow(sspTime/2,3),pow(sspTime/2,2),pow(sspTime/2,1),1,
 		 
	B << sspZin, -veloZin,sspZfi,-veloZfi_d, zMax;
	X = A.colPivHouseholderQr().solve(B);

	double z_a_free = X(0);
	double z_b_free = X(1);
	double z_c_free = X(2);
	double z_d_free = X(3);
	double z_e_free = X(4);

/*	Matrix<double, 7, 7> A;
	Matrix<double, 7, 1> B, X;

	A << 0,0,0,0,0,0,1,
 		 0,0,0,0,0,1,0,
 		 0,0,0,0,2,0,0,
		 pow(sspTime,6),pow(sspTime,5),pow(sspTime,4),pow(sspTime,3),pow(sspTime,2),pow(sspTime,1),1,
		 6*pow(sspTime,5),5*pow(sspTime,4),4*pow(sspTime,3),3*pow(sspTime,2),2*pow(sspTime,1),1,0,
		 pow(sspTime/2,6),pow(sspTime/2,5),pow(sspTime/2,4),pow(sspTime/2,3),pow(sspTime/2,2),pow(sspTime/2,1),1,
		 30*pow(sspTime,4),20*pow(sspTime,3),12*pow(sspTime,2),6*pow(sspTime,1),2,0,0,
	B << sspZin, -veloZin,0, sspZfi,-veloZfi_d, zMax,0;
	X = A.colPivHouseholderQr().solve(B);

	double z_a_free = X(0);
	double z_b_free = X(1);
	double z_c_free = X(2);
	double z_d_free = X(3);
	double z_e_free = X(4);
	double z_f_free = X(5);
	double z_g_free = X(6);*/
	// cout<<z_a_free*pow(sspTime/2,2) + z_b_free*sspTime/2 + z_c_free<<endl;

	// printf("%f %f\n",c1_z_swing,c2_z_swing);
	// printf("%f %f %f %f \n", sspZin, c1_z_swing + c2_z_swing, sspZfi, c1_z_swing*exp(C*sspTimeVar) + c2_z_swing*exp(-C*sspTimeVar));
	//TODO add case in which dsp1Time becomes negative 
	
	//TODO add case where dsp2Time needs to be modified (if required)
	//TODO calculation of sspZfi for next step based on predicted change in velocity for the next step 
	// printf("legZin\t\t%lf\n",legZin);
	// printf("supLegZin\t\t%lf\n",supLegZin);
	// printf("zMax\t\t%lf\n",zMax);
	// printf("sspZPhs\t\t%lf\n",sspZPhs);
	// printf("sspZTime\t%lf\n",sspZTime);
	// printf("sspZin\t\t%lf\n",sspZin);
	// printf("sspZSupin\t%lf\n",sspZSupin);
	// printf("sspZfi\t\t%lf\n",sspZfi);
	// printf("sspZSupfi\t%lf\n",sspZSupfi);
	// printf("dsp1Time\t%lf\n",dsp1Time);
	// printf("stepTime\t%lf\n",stepTime);
	// printf("LegZfi\t\t%lf\n",legZfi);
	// printf("supLegZfi\t%lf\n",supLegZfi);
	// printf("VeloZin\t%lf\n", veloZin);
	// printf("VeloZfi\t%lf\n", P_veloZfi);
	// printf("VeloZfi_d\t%lf\n", veloZfi_d);

	// printf("P_sspZPhs\t\t%lf\n",P_sspZPhs);
	// printf("P_sspZAmp\t\t%lf\n",P_sspZAmp);
	// printf("P_sspZin\t\t%lf\n", P_sspZin);
	// Y Control 
	
	legYin    = -legYin;
	supLegYin = -supLegYin;
	
	double sspYin    = legYin + veloYin * dsp1Time;
	double sspYSupin = supLegYin + veloYin * dsp1Time;

	//Commented equations pertain to non-general (old) y motion
	
	// double Ay = sspYSupin;
	// double By = veloYin*Tc;
	double c1_y = (sspYSupin + veloYin/C)/2;
	double c2_y = (sspYSupin - veloYin/C)/2;

	// double veloYfi_d = Ay*sinh(sspTime/Tc)/Tc + By*cosh(sspTime/Tc)/Tc;
	double veloYfi_d = C*(c1_y*exp(C*sspTime) - c2_y*exp(-C*sspTime));

	// double P_By = veloYfi_d*Tc;
	// double P_Ay = (veloYfi - veloYfi_d*cosh(sspTime/Tc))*Tc/sinh(sspTime/Tc);
	double P_c1_y = (veloYfi - veloYfi_d*exp(-C*sspTime))/(C*(exp(C*sspTime) - exp(-C*sspTime)));
	double P_c2_y = P_c1_y - veloYfi_d/C;

	// double P_sspYin = P_Ay;
	double P_sspYin = P_c1_y + P_c2_y;
	// double sspYAmp   = veloYin * sqrt(pow(Tc,2) - pow(sspYSupin/veloYin,2));
	// double sspYPhs   = asinh((sspYSupin/veloYin)/sqrt(pow(Tc,2) - pow(sspYSupin/veloYin,2)));
	// double veloYfi_d   = sspYAmp/Tc * cosh (sspTime/Tc + sspYPhs);
	//////printf("**************%lf******************\n",sspYAmp/Tc * cosh (sspTime/Tc + sspYPhs));
	// double P_sspYAmp = sgn(veloYin) * Tc * sqrt((2*cosh(P_sspZTime/Tc)*veloYfi_d*veloYfi - pow(veloYfi_d,2) - pow(veloYfi,2))/(pow(sinh(P_sspZTime/Tc),2)));
	// double P_sspYPhs = -acosh(veloYfi_d*Tc/P_sspYAmp);
	// double P_sspYin  = P_sspYAmp * sinh(P_sspYPhs);
	double P_legYin  = P_sspYin - veloYfi_d*D_dsp1Time;

	double sspYfi    = P_legYin - veloYfi_d*dsp2Time;
	double sspYSupfi = c1_y*exp(C*sspTime) + c2_y*exp(-C*sspTime);
	// double sspYSupfi = Ay*cosh(sspTime/Tc) + By*sinh(sspTime/Tc);
	// double sspYSupfi = sspYAmp * sinh(sspTime/Tc + sspYPhs);
	// cout<<" see "<<	 Ay*cosh(sspTime/Tc) + By*sinh(sspTime/Tc) <<" "<<sspYSupfi<<endl;
	double legYfi    = sspYfi + veloYfi_d*dsp2Time;
	double supLegYfi = sspYSupfi + veloYfi_d*dsp2Time;
	// cout<<"legYfi "<<legYfi<<endl;
	// cout<<"sspYfi "<<sspYSupfi<<endl;
	// Y (cubic)
	
	double y_a_free = ((-veloYfi_d-veloYin)-2*(-sspYfi+sspYin)/sspTime)/pow(sspTime,2);
	double y_b_free = ((-sspYfi+sspYin)/sspTime+veloYin-y_a_free*pow(sspTime,2))/sspTime;
	double y_c_free = -veloYin;
	double y_d_free = -sspYin;

/*	Matrix<double, 4, 4> A;
	Matrix<double, 4, 1> B, X;

	A << 0,0,0,1,
		 pow(stepTime,3),pow(stepTime,2),pow(stepTime,1),1,
		 pow(0.1*stepTime,3),pow(0.1*stepTime,2),pow(0.1*stepTime,1),1,
		 pow(0.4*stepTime,3),pow(0.4*stepTime,2),pow(0.4*stepTime,1),1,
 		 
	B << -legYin,-legYfi,-10,-20;
	X = A.colPivHouseholderQr().solve(B);*/

	// cout<<5*X(0)*pow(stepTime,4) + 4*X(1)*pow(stepTime,3) + 3*X(2)*pow(stepTime,2) + 2*X(3)*pow(stepTime,1) + X(4)<<" "<<-veloYfi_d<<endl; 
	
	// double height = 400;
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
	double deriv_term_z = 0, prop_term_z = 0, err_z = 0, correction_z = 0;
	double deriv_term_y = 0, prop_term_y = 0, err_y = 0, correction_y = 0; 
	
	int state = DSP;

	//Experimental
/*	double curr_imu_yaw = bot->getImuYaw();
	//Imu yaw negative change if turned to the 'right'
	printf("Imu change = %f\t\n", curr_imu_yaw-prev_imu_yaw );
	if (fabs(curr_imu_yaw - prev_imu_yaw) < 5)
	{
		if (leg == RIGHT)
			supLegRotfi = (curr_imu_yaw - prev_imu_yaw);
		else
			supLegRotfi = -(curr_imu_yaw - prev_imu_yaw);
	}
	else
	{
		legRotfi = 0;
	}
	prev_imu_yaw = curr_imu_yaw;*/
/*
	double mean_y, mean_z;
	if (prev_mean_y != 0)
	{
		mean_y = prev_mean_y/(double)(stepCount);
		mean_y = (int)mean_y;
		// mean_y = prev_mean_y/(double)stepCount;
		// mean_y = (int)mean_y;
		// printf("Mean values = %f ",mean_y);	
		// mean_y = prev_mean_y;
	}
	else
	{
		mean_y = 12;
	}
	if (prev_mean_z != 0)
	{
		mean_z = prev_mean_z/(double)(stepCount);
		mean_z = (int)mean_z;
		// printf("%f \n",mean_z);
		// mean_z = prev_mean_z;
		// prev_mean_z *= (double)stepCount;
	}
	else
		mean_z = 0;*/

	double avg = 0;
	double rms = 0;
	// printf("\nSTEP BEGINS\n");
//	////printf("*************************************** STEP **********************************************\n");	
	double z_free_traj;
	double prev_com_z = 0;
	double handSwing = 0;
	double pos_z_actual, vel_z_actual;
	double pos_y_actual, vel_y_actual;
	double vely = 0;

		// cout<<endl;
	for(walkTime = 0.0/fps; walkTime<=stepTime; walkTime +=timeInc)
	{
		timespec timer;
		clock_gettime(CLOCK_REALTIME, &timer);
		double time1 = timer.tv_nsec;

		if (flag == 1)
		{
			walkTime = stepTime/2;
		}

		if (walkTime<dsp1Time + sspZTime/2)
			handSwing = scurve(2048,2198,walkTime,(sspZTime+dsp1Time + dsp2Time)/2);
		else
			handSwing = scurve(2198,2048,walkTime-(dsp1Time+dsp2Time +sspZTime)/2,(sspZTime+dsp1Time + dsp2Time)/2);
		
		if(walkTime < dsp1Time)
		{
			x  = height;
			xr = height ;
			y  = -legYin - veloYin*walkTime;
			yr = -supLegYin - veloYin*walkTime;
			z  = legZin - veloZin*walkTime - hipLength/2;
			// z = z_a_free*pow(walkTime,2) + z_b_free*(walkTime) + z_c_free -hipLength/2;
			zr = supLegZin + veloZin*walkTime - hipLength/2;
			phi= legRotin; 
			phiR= supLegRotin;
	//		////printf("DSP1\t");
			state = DSP;
		}
		else if (walkTime >= dsp1Time && walkTime <= dsp1Time + sspTime)
		{
			fraction=2*(walkTime-startX)/(stopX-startX);
			if (walkTime<= dsp1Time + sspZTime)
			{
				x  = height - lift * (sin(xfreq*((walkTime-startX)/(stopX-startX))+xPhase) + displacement)/(1+displacement) - scurve(0, 5, walkTime-dsp1Time, sspTime + dsp1Time);
				// x = height- lift*fraction*(2-fraction);
				xr = height - scurve(0, 5, walkTime-dsp1Time, sspTime + dsp1Time);//- 10*sin(pi*(walkTime - dsp1Time)/sspTime);
				y=y_a_free*pow(walkTime-dsp1Time,3)+y_b_free*pow(walkTime-dsp1Time,2)+y_c_free*(walkTime-dsp1Time)+y_d_free;
				//y  = linear(-sspYin,-sspYfi, ((walkTime-dsp1Time)/sspTime);
				// yr = -(Ay*cosh((walkTime-dsp1Time)/Tc) + By*sinh((walkTime-dsp1Time)/Tc));
				yr = -(c1_y*exp(C*(walkTime-dsp1Time)) + c2_y*exp(-C*(walkTime-dsp1Time)));
				vely = -C*(c1_y*exp(C*(walkTime-dsp1Time)) - c2_y*exp(-C*(walkTime-dsp1Time)));
				// cout<<"yr = "<<yr<<" vely = "<<vely<<" y_energy = "<<y_energy<<endl;
				// z  = scurve(sspZin,sspZfi, walkTime-dsp1Time,sspTime) - hipLength/2;
				// z = z_a_free*pow(walkTime-dsp1Time,2) + z_b_free*(walkTime-dsp1Time) + z_c_free -hipLength/2;
				z = z_a_free*pow(walkTime-dsp1Time,4) + z_b_free*pow(walkTime-dsp1Time,3) + z_c_free*pow(walkTime-dsp1Time,2) + z_d_free*(walkTime-dsp1Time) + z_e_free -hipLength/2;
				// z = z_a_free*pow(walkTime-dsp1Time,6) + z_b_free*pow(walkTime-dsp1Time,5) + z_c_free*pow(walkTime-dsp1Time,4) + z_d_free*pow(walkTime-dsp1Time,3) + z_e_free*pow(walkTime-dsp1Time, 2) + z_f_free*pow(walkTime-dsp1Time,1) + z_g_free - hipLength/2;
				zr = c1_z*exp(C*(walkTime-dsp1Time)) + c2_z*exp(-C*(walkTime-dsp1Time)) -hipLength/2;
				phi= scurve(legRotin,legRotfi,walkTime-dsp1Time,sspTime);
				phiR=scurve(supLegRotin,supLegRotfi,walkTime-dsp1Time,sspTime);
				double t_remain = stepTime - walkTime - dsp2Time;
				const double (&COM)[AXES] = bot->getRotCOM();
				pos_z_actual = zr+hipLength/2 + 1*(leg==1?-1:1)*COM[2];
				double velz =  C*(c1_z*exp(C*(walkTime-dsp1Time)) - c2_z*exp(-C*(walkTime-dsp1Time)));
		// cout<<z<<" "<<zr<<endl;

				pos_y_actual = yr - COM[1];
				// cout<<yr<<endl;
				const double (&COMVel)[AXES] = bot->getRotCOMVel((leg?-1:1)*velz, -vely);
				vel_z_actual = (leg?-1:1)*COMVel[2];
				vel_y_actual = COMVel[1];
				// cout<<COMVel[2]<<" \t"<<velz<<endl;
				// cout<<leg<<" "<<COMVel[2]<<" \t"<</velz<<" \t"<<COM[2]<<" \t"<<COM0[2]<<endl;
				// double zmp_offset_z = (sspZSupfi*2*C*exp(C*t_remain) - (pos_z_actual)*C*(1+exp(2*C*t_remain)) + COMVel[2]*(1-exp(2*C*t_remain)))/(C*(exp(2*C*t_remain) - 2*exp(C*t_remain) + 1));
				// double zmp_offset_z2 = (pow(C,2)*(pow(sspZSupfi,2) - pow(zr+hipLength/2, 2)) + pow(velz,2) - pow(veloZfi,2))/(2*pow(C,2)*(sspZSupfi - (zr+hipLength/2) ));
				// double time_1 = log((sspZSupfi - zmp_offset_z)/(pos_z_actual + COMVel[2]/C - zmp_offset_z) + sqrt(pow(sspZSupfi - zmp_offset_z,2)/pow((pos_z_actual+ COMVel[2]/C - zmp_offset_z),2) - (pos_z_actual - COMVel[2]/C - zmp_offset_z)/(pos_z_actual+ COMVel[2]/C - zmp_offset_z)))/C; 
				// zmp_offset_y = (-sspYSupfi - veloYfi_d/C -  exp(C*sspTime)*(yr+vely/C))/(1-exp(C*sspTime));
				// cout<<pos_z_actual<<" "<<zr+hipLength/2<<" "<<velz<<" "<<vel_z_actual<<" "<<zmp_offset_z<<" "<<t_remain<<" "<<time_1<<endl;
				// cout<<" Z_off_z "<<zmp_offset_z<<" z_off_y "<<zmp_offset_y<<" t_rem "<<t_remain<<" zr "<<zr + hipLength/2<<" velz "<<velz<<endl;
				// cout<<leg<<" "<<velz<<endl;
			}

			// double z_traj = c1_z*exp(C*(walkTime-dsp1Time)) + c2_z*exp(-C*(walkTime-dsp1Time)) -hipLength/2;
			else
			{
				z = z_a_free*pow(walkTime,2) + z_b_free*(walkTime) + z_c_free -hipLength/2;
				if (walkTime-dsp1Time < sspTime/2)
					xr = height - scurve(0,30,walkTime-dsp1Time-sspZTime,sspTime/2);
				else
					xr = height - scurve(30,0 ,walkTime-dsp1Time-sspZTime - sspTime/2,sspTime/2);
			}
			// printf("%f %f\n",z_free_traj ,zr);
			// double energy =  (pow(velz,2.0) - pow(C,2.0)*pow(zr+hipLength/2,2.0))/2.0;
			// double alpha = sqrt(-2*energy/pow(C,2.0));
			// z = z_free_traj;
			// printf("Energy = %f supleg = %f velz = %f alpha = %f leg = %d\n", energy, zr+hipLength/2, velz, alpha, leg);
			// printf("leg = %d zr = %f ",leg,sspZAmp * cosh((walkTime-dsp1Time)/Tc + sspZPhs));
			// printf("zrr = %f", c1_z*exp(C*(walkTime-dsp1Time)) +c2_z*exp(-C*(walkTime-dsp1Time)) );	
			// printf(" velz = %f velzz = %f\n",sspZAmp*sinh((walkTime-dsp1Time)/Tc + sspZPhs)/Tc,C*(c1_z*exp(C*(walkTime-dsp1Time)) - c2_z*exp(-C*(walkTime-dsp1Time))));
			state = SSP;
	//		////printf("SSP0\t");
		}
		else if (walkTime > dsp1Time+sspTime )//&& walkTime<=stepTime
		{
			state = DSP;
			x  = height - dampLift* (sin(dampFreq*((walkTime-dampStart)/(dampEnd-dampStart))+dampPhase)+dampDisplacement)/(1+dampDisplacement)- scurve(5, 0, walkTime-dsp1Time-sspTime, stepTime);
			xr = height - scurve(5, 0, walkTime-dsp1Time-sspTime, stepTime);
			y  = -sspYfi - veloYfi_d*(walkTime-dsp1Time-sspTime);
			yr = -sspYSupfi - veloYfi_d*(walkTime-dsp1Time-sspTime);
			z  = sspZfi - veloZfi_d*(walkTime-dsp1Time-sspTime) -hipLength/2;
			// z = z_a_free*pow(walkTime,2) + z_b_free*(walkTime) + z_c_free -hipLength/2;
			zr = sspZSupfi + veloZfi_d*(walkTime-dsp1Time-sspTime) - hipLength/2;
			phi= legRotfi;
			phiR= supLegRotfi;
	//		////printf("DSP2\t");
		}
		else 
		{
	//		////printf("************* SOMETHING WRONG*******************\n");
		}
		// cout<<z<<" "<<zr<<endl;
		if (flag == 1)
		{
			bot->reachSlow(height,y,z+feetSeparation,height,yr,zr+feetSeparation);
			flag = 3;
		}

		// double ynew = -41710*pow(walkTime,4) + 34010*pow(walkTime,3) -8262*pow(walkTime,2) + 698*pow(walkTime,1) -33.2;
		// handMotion(handSwing); //Turn this on for rhythmic swinging of hands with walk.
		// printf("X\t%3.1lf\tXR\t%3.1lf\tY\t%lf\tYR\t%lf\tZ\t%lf\tZR\t%lf\n",x,xr,y,yr,z,zr);
		////printf("Y\t%lf\tYR\t%lf\tZ\t%lf\tZR\t%lf\tP\t%lf\tPR\t%lf\n",y,yr,z+s,zr+sr,phi,phiR);
		///////printf("Z\t%lf\tZR\t%lf\n",z,zr);
//		printf("W phi\t%lf\tphiR\t%lf\tZ\t%lf\tZR\t%lf\tY\t%lf\tYR\t%lf",phi,phiR,z,zr,y,yr);
		// const double (&COM1)[AXES] = bot->getCOM();
		// bot->printRotCOM();
		// bot->printCOM();
		// int a = walkTime*fps;
		// cout<<a<<endl;
/*		if (a%5 == 0)
		{
			cout<<"Reading\n";
			const double (&COM)[AXES] = bot->getCOM();
		}	
*/			// cout<<walkTime<<"\t"<<y<<"\t"<<y - 6.85*(COM[1]-17.5)<<"\t"<<ynew<<"\t"<<endl;
		// cout<<y<<"\t"<<y - 6.85*(COM[1]-17.5)<<"\t"<<ynew<<"\t"<<endl;
		int fcount = walkTime*fps;
		if (flag != 3)
		{
			bot->leg[leg]->runIK(x,y ,z+feetSeparation ,phi);
			bot->leg[1-leg]->runIK(xr,yr ,zr+feetSeparation,phiR);
			// bot->getCOM();
			// cout<<z<<" "<<z- 6.65*(leg==1?1:-1)*COM[2]<<endl; 
			// cout<<y<<" "<<y - 6.85*(COM[1]-17.5)<<" "<<yr<<endl;
			// cout<<walkTime<<" "<<y - 6.85*(COM[1]-17.5)<<endl;
			// bot->leg[leg]->runIK(x,y- 6.85*(COM[1]-17.5),z+feetSeparation ,phi);
			// bot->leg[1-leg]->runIK(xr,yr ,zr+feetSeparation,phiR);		
			// bot->getCOM();
			// cout<<COM[1]<<endl;
			// cout<<z<<" "<<zr<<endl;
		}
		else
		{
			bot->leg[leg]->runIK(height,y,z+feetSeparation ,phi);
			bot->leg[1-leg]->runIK(height,yr ,zr+feetSeparation,phiR);		
		}
		// cout<<z<<" "<<zr<<endl;
		// printf("%f %f\n",COM[2],(COM[2]-prev_com_z)/timeInc);
		// prev_com_z = COM[2];
				// cout<<"WalkTime"<<walkTime<<endl;
		if (walkTime >= dsp1Time +  sspZTime/2 -timeInc && walkTime <= dsp1Time + sspZTime/2 && flag == 0)
		{
			const double (&COMVelHalfTime)[AXES] = bot->getRotCOMVel(0, -vely);
			vel_z_actual = (leg?-1:1)*COMVelHalfTime[2];
			vel_y_actual = COMVelHalfTime[1];
			// cout<<"Look "<<legZfi<<endl;
			// captureStepZ(  c1_z, c2_z, zMax, dsp1Time, dsp2Time, pos_z_actual, vel_z_actual,walkTime, sspTime, z_a_free, z_b_free, z_c_free, legZfi, supLegZfi);
			// cout<<"Look "<<legZfi<<endl;
			// captureStepY(c1_y, c2_y, dsp1Time, dsp2Time, yr, vely, y, supLegYfi, veloYfi_d, -pos_y_actual, vel_y_actual, walkTime, sspTime, y_a_free, y_b_free, y_c_free, y_d_free);
		}
		// cout<<leg<<" ";
		// bot->getRotCOM();
		//BEST SO FAR
		// bot->leg[leg]->runIK(x,y,z+feetSeparation - 2.05*com_offset[fcount],phi);
		// bot->leg[1-leg]->runIK(xr,yr ,zr+feetSeparation ,phiR);

		// rms += pow(COM[2],2);
		// cout<<state<<" "<<zr<<" "<<sspZSupfi<<" "<<sspZfi<<endl;
		// rms += pow(COM[1]-17.5,2);
		// avg += COM[1]; 

		if (flag != 2)
			bot->updateBot();
		// cout<< timer.tv_nsec<<endl;
		// else
			// cout<<"Value setting step";
		//printf("Sent Values\n");
		//leg->getMotorLoad(FOOT_ROLL);
		//revLeg->getMotorLoad(FOOT_ROLL);
		// cout<<"End Frame\n";
		clock_gettime(CLOCK_REALTIME, &timer);
		double time2 = timer.tv_nsec;
		// cout<<"Time diff total: "<<(time2 - time1)*pow(10.0,-3)<<endl;
		double sub_usleep = (sleep-(time2-time1)*pow(10.0,-3));
		// cout<<"Sleep default: "<<sleep<<endl;
		// cout<<"Removed "<<sub_usleep<<endl;
		if (sub_usleep>0 && (time2 - time1) > 0)
			usleep(sub_usleep);
		//getchar();
		
	}
	rms = sqrt(rms/(stepTime*fps));
	avg /= (stepTime*fps);
	// printf("Avg = %f\n",avg);
	// printf("leg = %d rms = %f multiplier = %f\n",leg,rms,stepCount*0.05);
	// printf("rms = %f\n",rms);
	// printf("%f\n",-sspYfi + sspYin );
	supLegYin  = -legYfi;
	legYin = -supLegYfi;
	veloYin=veloYfi_d;

	supLegZin = legZfi - hipLength/2; 
	legZin = supLegZfi - hipLength/2;
//@Above two lines, This is what should happen, but this implementation will be changed because
	//we want the robot to strafe, and to do that we need to have it put the free foot out in each step,
	//and bring the next free foot back inside on the next step
	//for this we need the deviation to be reflected in calculations of initial state of next step
// @Below two lines are coded for this.
	// supLegZin = supLegZfi - hipLength/2; 
	// legZin = legZfi - hipLength/2;

	veloZin = -veloZfi_d;
	// veloZfi= P_veloZfi;  
	supLegRotin  = legRotfi;
	legRotin = supLegRotfi;
	// sspZAmp = P_sspZAmp;

	if (pos_z_actual<0)
	{
		// cout<<"Look "<<pos_z_actual<<endl;
		leg=(LEG)(1-(int)leg);	
	}
	return EXIT_SUCCESS;//endState;
	
}