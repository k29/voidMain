#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <math.h>

using namespace std;

#define MotMass 153.0
#define BracMass 44.0
#define PI (3.141592653589793)
#define deg2rad(x) (x)*PI/180.0
#define legMass 1713.5
#define bodyMass 3954.0
#define botMass 8409.0	//lesMass*2 + bodyMass + batteryMass
#define hipLength2 120.0 //distance between legs (hip motors)

static const double l0=50.0;
static const double l1=170.0;
static const double l2=69.5;
static const double l3=170.0;

float* generate_legcom(float xf, float yf, float zf, float angle[7])
{
	float comLeg[3], brac1[3], brac2[3];
	float xk, yk, zk, t3, t4, t5;	//xk,yk and zk are knee coordinates, t2, t3, t4 and t5 are angles
	
	//angle is given as motor vaue
	t3 = deg2rad(angle[4]*360/4096.0);
	t4 = deg2rad(angle[6]*360/4096.0);
	t5 = deg2rad(angle[5]*360/4096.0);
		
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
	comLeg[0] = (((2*MotMass + 114.376955)*0) + (4*BracMass*brac1[0]) + ((2*MotMass + 94.30511)*xk) + (4*BracMass*brac2[0])+ ((2*MotMass + 225.049260)*xf))/legMass;
	comLeg[1] = (((2*MotMass + 114.376955)*0) + (4*BracMass*brac1[1]) + ((2*MotMass + 94.30511)*yk) + (4*BracMass*brac2[1])+ ((2*MotMass + 225.049260)*yf))/legMass;
	comLeg[2] = (((2*MotMass + 114.376955)*0) + (4*BracMass*brac1[2]) + ((2*MotMass + 94.30511)*zk) + (4*BracMass*brac2[2])+ ((2*MotMass + 225.049260)*zf))/legMass;
	//constants are bracket mass  
	return comLeg;
}

float* generate_com(float left_x, float left_y, float left_z, float angle_left[7], float right_x, float right_y, float right_z, float angle_right[7] )
{
	//left leg
	float* com_left = generate_legcom(left_x, left_y, left_z, angle_left);
	
	//right leg
	float* com_right = generate_legcom(right_x, right_y, right_z, angle_right);
	
	//final com - right leg coordinate system
	float com[3];
	com[0] = (legMass*(com_left[0]) + legMass*(com_right[0]) + bodyMass*(-203.741) )/botMass;
	com[1] = (legMass*(com_left[1]) + legMass*( com_right[1]) + bodyMass*0)/botMass;
	com[2] = (-legMass*(com_left[2] - hipLength2) + legMass*com_right[2] + bodyMass*(-hipLength2/2.0))/botMass;
		
	return com;
}

int main()
{
	float *cog;
	cout<<"Enter xf, yf, zf\n";
	float x, y, z;
	cin>>x>>y>>z;
	cout<<"Enter motorvalues corrsponding to t3, t4 and t5\n";
	float ang[7];
	cin>>ang[4]>>ang[6]>>ang[5];
	cog=generate_legcom(x,y,z,ang);
	for(int i=0; i<3; i++)
		cout<<cog[i]<<"\t";
} 
