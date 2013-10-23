

/*	The program aims to plan footstep positions along a path composed of lines and circles
	Along a line it will accelerate at maximum velocity until it reaches the circle
	In the circle it will repeatedly accelerate and decelerate to obey the velocity constraints which are outlined below
	The circle cannot have uniform motion because the inner step length will be smaller than the outer one which will eventually give negative velocity when applied to the 
	equations below.
*/



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
const double v_initial = 10;				 //	Velocity when bot begins to move
const double initial_delta_y= 2*v_initial/c5;//	Initial step length is calculated by using equation 4 and given initial velocity which is assumed constant, i.e, vf=vd=vi
const double max_velocity = 40;//	Maximum velocity corresponding to above delta y assuming step length is constant for the motion	
const double max_delta_y  = 2*max_velocity/c5;				 //	Maximum length of a step (read delta y), from back to front. This is a limitation of stability; if bot moves more farther than this, it will be unstable
const double foot_width  = 100;				 //	Width of the foot, might not be used
const double foot_separation = 130;			 //	Standard Length between centers of the feet during linear motion
const double max_delta_x = 30;	
const double max_delta_theta = 15;	
const double min_delta_y = 6;				 //	This is an arbitrary value chosen to put a lower limit on the bot's step length such that the step length oscillates between
											 //	these maximum and minimum values of delta x. This will be changed for fine-tuning the motion.

const double OBSTACLE_RADIUS = 200;

struct foot
{
	double delta_x,delta_y,delta_theta;				//	An array of this structure will store the final output of the following program, namely, delta_x, delta_y, delta_theta
};



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






/*	delta_x is a function of inner radius and delta y because the inner footstep must be along a circle of radius (R - foot_separation/2)
	For outer foot the delta_x will always be zero because it is aligned at the ideal foot separation with the inner foot
	Hence a formula is derived with the above constraint which is used in the function below.
*/
double calc_delta_x(double inner_radius , foot step)
{
	double delta_x = inner_radius - pow ((pow(inner_radius,2) - pow(step.delta_y,2)), 0.5);
	return delta_x;
}


//	This is similar to the above functions in its dependencies. However, delta_theta will not be zero for the outer foot, rather it will be negative of the value of the inner foot
//	because of the goal of aligning with the inner foot.
double calc_delta_theta(double inner_radius , foot step)	
{
	double delta_theta = fabs(acos((pow ((pow(inner_radius,2) - pow(step.delta_y,2)), 0.5))/inner_radius))*180/pi;

	return delta_theta;
}


/*	This function checks that the motion along the circle follows the constraints of the bot's motion. It will return a value if the constraints are violated.
	This value will indicate which constraint has been violated, for debugging use. If it returns 1, there is no constraint violation.
	The variable loop_num refers to the loop within which constraints are being checked because the two loops defined below have slightly different constraints.
*/
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

using namespace std; 
// int main(double vd , double initial_delta_y , int lr , int pathnumber , foot & step[] )
int main()
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
	/*PathStructure ps;
	srand(time(NULL));
	ps.n_obstacles = 2;
	ps.absObstacles[0].x=20; 
	ps.absObstacles[0].y=31;
	ps.absObstacles[1].x=60 ;
	ps.absObstacles[1].y=100;
	ps.absObstacles[2].x=-40;
	ps.absObstacles[2].y=-80;

	ps.ball.x =110;
	ps.ball.y =110;

	ps.goal.x=190;
	ps.goal.y=120;
	
	int r=200;
	int theta=100;
	
	Path p;
	p.path_return(ps);
	p.updatePathPacket(pathpackvar);*/
	
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
	fstream fil;
	fil.open("Error3.dat", ios::out|ios::binary);
	for (int i=0;i<count-1;i++)
	{
		printf ("Delta_y = %f\tDelta_x = %f\tDelta_theta = %f\n",step[i].delta_y,step[i].delta_x,step[i].delta_theta);
		fil.write((char*)&step[i],sizeof(step[i]));
	}
	fil.close();
	// printf("\n\nmax vel=%f\n\n",max_velocity);
	printf("COUNT = %d",count);
	return 0;





}