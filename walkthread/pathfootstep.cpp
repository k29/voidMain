#include "AcYut.h"
#include "communication.h"
#include "walk.h"
#include "xsens/imu.h"
#include <signal.h>
#include <fstream>
#include <ncurses.h>
#include <stdio.h>
#include <math.h>
#include "commondefs.h"
#include <time.h>
#include <cstring>
#include <set>
#include <unistd.h>
#include <sys/ioctl.h>	
#include <sys/types.h>
#include <inttypes.h>
#include <stdlib.h>
#include <sys/time.h>
#include <termios.h>



#define distance(x1,y1,x2,y2)	sqrt(pow(y2-y1,2) + pow(x2-x1,2))

const double pi = acos(-1);
//	c1, c2... are constants for equations which are outlined below 	
//	vd = Current velocity, vf = Final velocity, delta_y = Step length of the foot when it moves from back to front
													//	vd is taken as velocity at the beginning of the step and vf as vlocity at the end of the step

const double c1=1.721463;							//	vf_max = c1*vd			Maximum possible velocity at the end of the step
const double c2=(c1 - pow(pow(c1,2)-1,0.5));		//	vf_min = c2*vd			Minimum possible velocity at the beginning of the step
const double c5=7.853317745/2;						// 	vf + vd = c5*delta_y 	Generalised equation which connects the above equations
const double c3=2.721463/c5;				//	delta_y_max = c3*vd		Maximum step length possible, this corresponds to equation 1
const double c4=1.320237/c5;				//	delta_y_min	= c4*vd		Minimum step length possible, this corresponds to equation 2
// const double v_initial = 10;				 //	Velocity when bot begins to move
// const double initial_delta_y= 2*v_initial/c5;//	Initial step length is calculated by using equation 4 and given initial velocity which is assumed constant, i.e, vf=vd=vi
const double max_velocity = 40;//	Maximum velocity corresponding to above delta y assuming step length is constant for the motion	
const double max_delta_y  = 2*max_velocity/c5;				 //	Maximum length of a step (read delta y), from back to front. This is a limitation of stability; if bot moves more farther than this, it will be unstable
const double foot_width  = 100;				 //	Width of the foot, might not be used
const double foot_separation = 130;			 //	Standard Length between centers of the feet during linear motion
const double max_delta_x = 30;	
const double max_delta_theta = 15;	
const double min_delta_y = 6;				 //	This is an arbitrary value chosen to put a lower limit on the bot's step length such that the step length oscillates between
											 //	these maximum and minimum values of delta x. This will be changed for fine-tuning the motion.

const double OBSTACLE_RADIUS = 200;


using namespace std;

bool quit = false;


int kbhit2()
{
	int	 count = 0;
	int	 error;
	static struct termios	Otty, Ntty;

	tcgetattr(STDIN_FILENO, &Otty);
	Ntty = Otty;

	Ntty.c_lflag &= ~ICANON; /* raw mode */
	Ntty.c_cc[VMIN] = CMIN; /* minimum chars to wait for */
	Ntty.c_cc[VTIME] = CTIME; /* minimum wait time	 */

	if (0 == (error = tcsetattr(STDIN_FILENO, TCSANOW, &Ntty)))
	{
		struct timeval	tv;
		error += ioctl(STDIN_FILENO, FIONREAD, &count);
		error += tcsetattr(STDIN_FILENO, TCSANOW, &Otty);
/* minimal delay gives up cpu time slice, and
* allows use in a tight loop */
		tv.tv_sec = 0;
		tv.tv_usec = 10;
		select(1, NULL, NULL, NULL, &tv);
	}
	return (error == 0 ? count : -1 );
}

bool getInput(char *c)
{
	if (kbhit2())
	{
		*c = getch();
		return true;
	}

	return false;

}


// FOOTSTEP PLANNING FUNCTIONS BEGIN HERE
double calc_dist_covered(int count, int count_ref, foot step[])	/*	Self-explanatory. This requires a function to calculate the distance because the total distance covered will be the sum of the
													distance covered by the current foot, which may be left or right. For precision the foot will have to be identified each time 
													and the function called again each time. This needs a better method.
												*/
{
	double dist_covered = 0;
	
	for (int i = count%2 + count_ref; i <= count ; i+=2)
	{
		dist_covered += step[i].delta_y ;
	}

	return dist_covered;
}


double calc_delta_x(double inner_radius , foot step)
{
	double delta_x = inner_radius - pow ((pow(inner_radius,2) - pow(step.delta_y,2)), 0.5);
	return delta_x;
}

double calc_delta_theta(double inner_radius , foot step)	
{
	double delta_theta = fabs(acos((pow ((pow(inner_radius,2) - pow(step.delta_y,2)), 0.5))/inner_radius))*180/pi;

	return delta_theta;
}

int constraint_check(foot step , double vf , int loop_num)
{
	if (loop_num == 1 && step.delta_y>max_delta_y)
		return 2;
	else if (loop_num ==2 && step.delta_y<min_delta_y)
		return 3;
	else if (vf>max_velocity)
		return 4;
	else if (step.delta_x>max_delta_x)
		return 5;
	else if (step.delta_theta>max_delta_theta)
		return 6;		
	else
		return 1;
}

void convert_values (PathPacket pathpackvar	,	double dist_circstart[],	double theta_arc[],	double radius[],	int footlr[])
{
		
		// dist[0]	=	distance(pathpackvar.finalpath[0].x,pathpackvar.finalpath[0].y,pathpackvar.finalpath[k-1].x,pathpackvar.finalpath[k-1].y); IS THIS NEEDED?
		for (int i = 0;	i<=pathpackvar.no_of_points/2; i++)
		{
			dist_circstart[i]	=	10*distance(pathpackvar.finalpath[pathpackvar.no_of_points-2*i+1].x,pathpackvar.finalpath[pathpackvar.no_of_points-2*i+1].y,pathpackvar.finalpath[pathpackvar.no_of_points-2*i].x,pathpackvar.finalpath[pathpackvar.no_of_points-2*i].y);
			radius[i]	=	OBSTACLE_RADIUS;
			theta_arc[i]	=	180*2*acos(sqrt(1- pow((10*distance(pathpackvar.finalpath[pathpackvar.no_of_points-2*i].x,pathpackvar.finalpath[pathpackvar.no_of_points-2*i].y,pathpackvar.finalpath[pathpackvar.no_of_points-2*i-1].x,pathpackvar.finalpath[pathpackvar.no_of_points-2*i-1].y)/(2*radius[i])),2)))/pi;
			double vector_product	=	pathpackvar.finalpath[pathpackvar.no_of_points-2*i-1].x*(pathpackvar.finalpath[pathpackvar.no_of_points-2*i].y - pathpackvar.finalpath[pathpackvar.no_of_points-2*i+1].y) + pathpackvar.finalpath[pathpackvar.no_of_points-2*i].x*(pathpackvar.finalpath[pathpackvar.no_of_points-2*i+1].y - pathpackvar.finalpath[pathpackvar.no_of_points-2*i-1].y) + pathpackvar.finalpath[pathpackvar.no_of_points-2*i+1].x*(pathpackvar.finalpath[pathpackvar.no_of_points-2*i-1].y - pathpackvar.finalpath[pathpackvar.no_of_points-2*i].y);
			//Vector product of the two vectors made by the three points will give the direction of turning.
			footlr[i]	=	(vector_product > 0) ? 0: 1;
			printf ("X[i] = %f\tY[i]=%f\n",pathpackvar.finalpath[pathpackvar.no_of_points - 2*i].x,pathpackvar.finalpath[pathpackvar.no_of_points - 2*i].y);
			printf ("X[i+1] = %f\tY[i+1]=%f\n", pathpackvar.finalpath[pathpackvar.no_of_points - 2*i -1].x,pathpackvar.finalpath[pathpackvar.no_of_points - 2*i -1].y);
			// printf ("X[i+2] = %f\tY[i+2]=%f\n",pathpackvar.finalpath[pathpackvar.no_of_points - 2*i-2].x,pathpackvar.finalpath[pathpackvar.no_of_points - 2*i-2].y);
			printf("i = %d\tDist_circstart = %f\tRadius = %f\tTheta_arc	= %f\tfootlr = %d\n",i,dist_circstart[i],radius[i],theta_arc[i],footlr[i]);
		}
}



int footstepmain(double v_initial , double initial_delta_y , int lr , int pathnumber , foot & step[])
{
	int count =	0,check = 0;
	int count_ref = 0;
	foot step[3000];				 			//	This is the array which will contain the required data
	
	//Linear motion is straightforward(haha) : The bot accelerates to its maximum step-length possible, and stays there until it arrives at a circle.
	//Outlined below is the code for linear motion. Delta x and delta theta remain zero throughout the motion and the code is written thusly.
	double vd = v_initial;						//	vd is initialized with initial velocity
	double vf;
	step[0].delta_y = initial_delta_y;			//	delta_length is initialized with initial step length	
	
	// const int no_circles = ;
	double theta_arc[30];					//	This will later be changed to an input.
	double radius[30];					//	This will later be changed to an input.
														//	This measures the angle travelled along the circular arc, in order to determine when it has to end circular motion.
	int footlr[30]; 
	double dist_circstart[30];				//	This is the distance to the beginning of the circle from the beginning of motion.
	PathPacket pathpackvar;
	// int location = sizeof(pathpackvar)*(pathnumber - 1);
	// fil1.seekg( location , ios::beg);
	fstream fil1;
	int pathnumber = 2;
	if (pathnumber == 1)
	{
		fil1.open("paths.dat", ios::in|ios::binary);
	}
	else
	{
		fil1.open("paths1.dat", ios::in|ios::binary);
	}

	fil1.read((char*)&pathpackvar,sizeof(pathpackvar));
	convert_values (pathpackvar,	dist_circstart,	theta_arc,	radius,	footlr);
	fil1.close();	

	count =	0;									//	Counts number of footsteps
	int i=0;	
	for (int i=0;i<2;i++)				//	no_circles will be replaced by an expression with k in it.
	{
		// printf("Count = %d\n",count);
		// printf ("Theta_arc = %f\tradius =%f\tdist_circstart = %f\t\n",theta_arc[i], radius[i], dist_circstart[i]);
		double dist_covered = 0	;					//	Corresponds to cal_dist_covered function.
		count_ref = count;
		while (dist_covered<dist_circstart[i])
		{
			step[count+1].delta_y = c3 * vd;
			if (step[count+1].delta_y>max_delta_y)
			{
				step[count+1].delta_y = max_delta_y;
			}

			vf = c5*step[count+1].delta_y - vd;
			vd = vf;								//	It seems unnecessary to involve vf here, but this is done to establish a standard for the code used to develop motion along
													//  the circle, where vf will be necessary. It will also be useful for later modifications.
			step[count].delta_x = 0;
			step[count].delta_theta = 0;
			count++;
			dist_covered = calc_dist_covered(count,count_ref,step);
			// printf(" I was here %d\n",count);
		}
						
		int constraint_value;
		double inner_radius = fabs(radius[i]); 
		double theta_current = 0;	
		check = 0;
		if (theta_arc[i]>=365 || theta_arc[i]<=-5 )
			check = 2;
		while(check!=2)
		{
			check=0;											
			while (check==0)
			{
				step[count+1].delta_y = c3 * vd;
				if ((count+1)%2 == footlr[i])				//This assumes that the entering foot is the inner one. This will be changed later in order to generalize.
				{
					step[count].delta_x = calc_delta_x(inner_radius , step[count]);
					step[count].delta_theta = calc_delta_theta(inner_radius , step[count]);
				}
				else
				{
					step[count].delta_x = 0;
					step[count].delta_theta = -1*step[count-1].delta_theta;
				}
				vf = c5*step[count+1].delta_y - vd;
				constraint_value = constraint_check(step[count+1] , vf , 1);	//	1 indicates loop 1
				if (constraint_value == 1)
				{
					if ((count+1)%2 == footlr[i])
						theta_current += step[count].delta_theta;
					if (theta_current<theta_arc[i])
						{
							count++;
							vd=vf;

						}
					else
						check=2;
					// printf("\nvf = %f\tDelta_theta[%d] = %f\tTheta_Current = %f\n",vf,count,step[count].delta_theta,theta_current);	
					
				}
				else
				{
					check = 1;
					// printf("\nConstraint Violation Loop 1 = %d\n",constraint_value);
					// printf("\nInterrupt vf = %f\tStep_length[%d] = %f\t%f\n",vf,count+1,step[count+1].delta_y,step[count+1].delta_theta);	
				}
			}

			if (check!=2)
				check=0;
			while (check==0)
			{
				step[count+1].delta_y = c4 * vd;
				if ((count+1)%2 == footlr[i])				//This assumes that the entering foot is the inner one. This will be changed later in order to generalize.
				{
					step[count].delta_x = calc_delta_x(inner_radius , step[count]);
					step[count].delta_theta = calc_delta_theta(inner_radius , step[count]);
				}
				else
				{
					step[count].delta_x = 0;
					step[count].delta_theta = -1*step[count-1].delta_theta;
				}
				vf = c5*step[count+1].delta_y - vd;
				constraint_value = constraint_check(step[count+1] , vf , 2);		//2 indicates loop 2
				if (constraint_value == 1)
				{
					if ((count+1)%2 == footlr[i])
						theta_current += step[count].delta_theta;
					if (theta_current<theta_arc[i])
					{
						count++;
						vd = vf;
					}
					else
						check=2;
					// printf("\nvf = %f\tDelta_theta[%d] = %f\tTheta_Current = %f\n",vf,count,step[count].delta_theta,theta_current);	
					
				}
				else
				{
					check=1;
					// printf("\nConstraint Violation Loop 2 = %d\n",constraint_value);
					// printf("\nInterrupt vf = %f\tStep_length[%d] = %f\t%f\n",vf,count+1,step[count+1].delta_y,step[count+1].delta_theta);	
				}



				
			}

		}
	}
	for (int i=0;i<count-1;i++)
	{
		printf ("Delta_y = %f\tDelta_x = %f\tDelta_theta = %f\n",step[i].delta_y,step[i].delta_x,step[i].delta_theta);
	}
	printf("COUNT = %d",count);
	return 0;

}

// FOOTSTEP PLANNING FUNCTIONS END HERE


void doquit(int para)
{
	quit = true;
};

struct foot
{
	double delta_x,delta_y,delta_theta;				//	An array of this structure will store the final output of the following program, namely, delta_x, delta_y, delta_theta
};


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

int balanceStatic(AcYut& bot, int phase, int leg = DSP)
{
	sleep(1);
	const double (&COM)[AXES] = bot.getRotCOM(); 
	//printf("COM X\t%3.3lf\tY\t%3.3lf\tZ\t%3.3lf\n",COM[0],COM[1],COM[2]);
	printf("R\t%3.3lf\tP\t%3.3lf\n",bot.imu->roll,bot.imu->pitch);
	const supportPolygon poly = bot.calcSupportPolygon();
	const Point center = getPolyCentroid(bot.calcSupportPolygon());
	printf("\n\n");
	float error[AXES] = {0};
	float corr[AXES] = {0};
	error[Y] = COM[Y] - center.y;
	error[Z] = COM[Z] - center.x;
	corr[Y] = 0.2 * error[Y];
	corr[Z] = 0.2 * error[Z];
	
	const float (&leftLeg)[AXES+1] = bot.left_leg->getLegCoods();
	const float (&rightLeg)[AXES+1] = bot.right_leg->getLegCoods();
	printf("Left Leg\t%lf\t%lf\t%lf\n",leftLeg[X],leftLeg[Y],leftLeg[Z],leftLeg[4]);
	printf("Right Leg\t%lf\t%lf\t%lf\n",rightLeg[X],rightLeg[Y],rightLeg[Z],rightLeg[4]);
	 
	
	bot.right_leg->runIK(rightLeg[X],rightLeg[Y] + corr[Y],rightLeg[Z],rightLeg[4]);
	bot.updateBot();
	printf("COMY\t%3.3lf\tZ\t%3.3lf\n",COM[1],COM[2]);
	printf("CenY\t%3.3lf\tX\t%3.3lf\n",center.y,center.x);
	printf("CorY\t%3.3lf\tZ\t%3.3lf\n",corr[Y],corr[Z]);
	for(int i=0;i<100;i++);
	printf("\n");
	return 1;
	
}

int kick(AcYut *bot)
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
	
		bot->right_leg->runIK(x,y,z,0);
		bot->left_leg->runIK(xr,yr,zr,0);
		bot->updateBot();
		
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
		bot->right_leg->runIK(x,y,z,0);
		bot->left_leg->runIK(xr,yr,zr,0);
		bot->updateBot();
		
		usleep(sleep);
		
	} 		
	
};

void test(AcYut *bot)
{

	int xmax = 390;
	int xmin = 360;
	int ymax = 30;
	int ymin = -30;
	float time = 0.5;
	float frameTime = 1.0/60;
	
	float t = 0;
	float x=xmax, y= ymin;
	for(t=0;t<time;t+=frameTime)
	{
		x = linear(xmax,xmin,t,time);
		y = linear(ymin,ymax,t,time);
		bot->right_leg->runIK(x,y,0,0);
		bot->updateBot();
		usleep(16670);
		printf("test x %lf y %lf\n",x,y);
		getchar();
	}
	
	for(t=0;t<time;t+=frameTime)
	{
		x = linear(xmin,xmax,t,time);
		y = linear(ymax,ymin,t,time);
		bot->right_leg->runIK(x,y,0,0);
		bot->updateBot();
		usleep(16670);
		printf("test x %lf y %lf\n",x,y);
		getchar();
	}
	
	
	
	
	

}



int main()
{
	Imu imu;
	printf("GENORAI");
	//imu.init();
	foot foot1[1000];
	//Initial position of demarcated code.
	//(void) signal(SIGINT,doquit);	
	Communication comm;
	AcYut bot(&comm,NULL);
	Walk walk(&bot);
	//int i=0;

	
	//test(&bot);
	//walk.dribble();
	//walk.dribble()//;
	// while(walk.vel//ocity()*1.72 <=90)
	//{//
			// walk.dribble();
	//	walk.accelerate();
	// }
	// while(1)
	// walk.dribble();
	char key = ' ';   
	do
	{
		// Call walkthread here instead of reading file.
		footstepmain(Walk::velocity() , foot1[j].y , i%2 , pathnumber , foot1);
		int j=0 , i=0;
		while(j<i-1 && key != 'c' && key != 'q')
		{//	bot.leg[0]->runIK(390,0,0,0);
		//	bot.leg[1]->runIK(390,0,0,0);
		//	bot.updateBot();
			printf("\n\n\nSTEP %d\n\n",j);
			walk.dribble(foot1[j].delta_y/2,foot1[j].delta_x,foot1[j].delta_theta,0);// + pow(-1,i)*4.544584329 ,0);
		 	// walk.dribble();
			if (!getInput(&key))
				j++;
		//	if(walk.velocity()*1.72 <=155)
		//		walk.accelerate();
		}
	} while (key != 'q');
	//walk.turnright(35.;
	//while(1)
	//	balanceStatic(bot,1,DSP);
	return 0;
	
	
};



