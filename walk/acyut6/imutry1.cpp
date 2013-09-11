#include <stdio.h>
#include "leg.h"
#include "communication.h"
#include <math.h>
#include <sys/time.h>
#include "AcYut.h"
#include "xsens/imu.h"
#include <signal.h>
#include <stdlib.h>

//COM caluculations

float left_com[3]={0},right_com[3]={0};
float com[3]={0},rot_com[3]={0},rot_hip[3]={0},rot_rleg[3],hip[3]={0};

using namespace std;

#define MotMass 153.0
#define BracMass 44.0
#define PI (3.141592653589793)
#define degtorad(x) x*PI/180.0
#define legMass 1713.5
#define bodyMass 3954.0
#define botMass 8409.0	//lesMass*2 + bodyMass + batteryMass
#define hipLength 120.0

static const double l0=50.0;
static const double l1=170.0;
static const double l2=69.5;
static const double l3=170.0;

void generate_legcom(int leg, float xf, float yf, float zf, float angle[7])
{
	float brac1[3], brac2[3];
	float xk, yk, zk, t3, t4, t5;	//xk,yk and zk are knee coordinates, t2, t3, t4 and t5 are angles
	
	//angle is given as motor vaue
	t3 = degtorad(angle[4]*360/4096.0);
	t4 = degtorad(angle[6]*360/4096.0);
	t5 = degtorad(angle[5]*360/4096.0);
		
	//knee coords
	xk = l3*sin(t3)*cos(t5) + l2/2.0;
	yk = l3*cos(t3)*cos(t4) - l3*sin(t3)*sin(t5)*sin(t4);
	zk = l3*cos(t3)*sin(t4) + l3*sin(t3)*sin(t5)*cos(t4);
	
	//com of brackets between motors
	brac1[0] = xk/2.0;
	brac1[1] = yk/2.0;
	brac1[2] = zk/2.0;
	
	brac2[0] = (xk + xf)/2.0;
	brac2[1] = (yk + yf)/2.0;
	brac2[3] = (zk + zf)/2.0;
	
	//com of leg
	if(leg==0)
	{
		left_com[0] = (((2*MotMass + 114.376955)*0) + (4*BracMass*brac1[0]) + ((2*MotMass + 94.30511)*xk) + (4*BracMass*brac2[0])+ ((2*MotMass + 225.049260)*xf))/legMass;
		left_com[1] = (((2*MotMass + 114.376955)*0) + (4*BracMass*brac1[1]) + ((2*MotMass + 94.30511)*yk) + (4*BracMass*brac2[1])+ ((2*MotMass + 225.049260)*yf))/legMass;
		left_com[2] = (((2*MotMass + 114.376955)*0) + (4*BracMass*brac1[2]) + ((2*MotMass + 94.30511)*zk) + (4*BracMass*brac2[2])+ ((2*MotMass + 225.049260)*zf))/legMass;
	}
	else 
	{
		right_com[0] = (((2*MotMass + 114.376955)*0) + (4*BracMass*brac1[0]) + ((2*MotMass + 94.30511)*xk) + (4*BracMass*brac2[0])+ ((2*MotMass + 225.049260)*xf))/legMass;
		right_com[1] = (((2*MotMass + 114.376955)*0) + (4*BracMass*brac1[1]) + ((2*MotMass + 94.30511)*yk) + (4*BracMass*brac2[1])+ ((2*MotMass + 225.049260)*yf))/legMass;
		right_com[2]= (((2*MotMass + 114.376955)*0) + (4*BracMass*brac1[2]) + ((2*MotMass + 94.30511)*zk) + (4*BracMass*brac2[2])+ ((2*MotMass + 225.049260)*zf))/legMass;
	}
	//constants are bracket mass  
}

float* generate_com(float left_x, float left_y, float left_z, float angle_left[7], float right_x, float right_y, float right_z, float angle_right[7], int coordSys )
{
	//left leg
	generate_legcom(0,left_x, left_y, left_z, angle_left);
	
	//right leg
	generate_legcom(1,right_x, right_y, right_z, angle_right);
	
	//final com
	
	com[0] = (legMass*(left_com[0]) + legMass*(right_com[0]) + bodyMass*(-203.741) )/botMass;
	com[1] = (legMass*(left_com[1]) + legMass*(right_com[1]) + bodyMass*0)/botMass;
	
	if(coordSys == 0)													//z axis according to left leg
		com[2] = (legMass*(left_com[2]) + legMass*(-right_com[2] - hipLength) + bodyMass*(-hipLength/2.0))/botMass;
	else															//z axis according to right leg
		com[2] = (legMass*(-left_com[2] - hipLength) + legMass*(right_com[2]) + bodyMass*(-hipLength/2.0))/botMass;
		
	return com;
}

int main()
{
	return 0;
}

