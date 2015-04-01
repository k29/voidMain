#include "walk_thread.h"

#define distance(x1,y1,x2,y2)	sqrt(pow(y2-y1,2) + pow(x2-x1,2))
#define convdegrad(p)	p*acos(-1)/180
 
const double pi = acos(-1);


//	c1, c2... are constants for equations which are outlined below 	
//	vd = Current velocity, vf = Final velocity, delta_y = Step length of the foot when it moves from back to front
													//	vd is taken as velocity at the beginning of the step and vf as vlocity at the end of the step

											//	vf_max = c1*vd			Maximum possible velocity at the end of the step
const double c1=1.836726;							// For zmax = 65 ,  1.8367262
const double c2=(c1 - pow(pow(c1,2)-1,0.5));		//	vf_min = c2*vd			Minimum possible velocity at the beginning of the step
// const double c5=7.853317745/2;						// 	vf + vd = c5*delta_y 	Generalised equation which connects the above equations
const double c5=3.7225963;						//	For zMax = 65 , 7.445192041/2
const double c3=(c1+1)/c5;				//	delta_y_max = c3*vd		Maximum step length possible, this corresponds to equation 1
const double c4=(c2+1)/c5;				//	delta_y_min	= c4*vd		Minimum step length possible, this corresponds to equation 2
const double v_initial = 10;				 //	Velocity when bot begins to move
const double first_delta_y= 2*v_initial/c5;//	Initial step length is calculated by using equation 4 and given initial velocity which is assumed constant, i.e, vf=vd=vi
const double max_velocity = 150;//	Maximum velocity corresponding to above delta y assuming step length is constant for the motion	
const double max_delta_y  = 2*max_velocity/c5;				 //	Maximum length of a step (read delta y), from back to front. This is a limitation of stability; if bot moves more farther than this, it will be unstable
const double foot_width  = 100;				 //	Width of the foot, might not be used
const double foot_separation = 130;			 //	Standard Length between centers of the feet during linear motion
const double max_delta_x = 30;	
const double max_delta_theta = 15;	
const double min_delta_y = 15;				 //	This is an arbitrary value chosen to put a lower limit on the bot's step length such that the step length oscillates between
											 //	these maximum and minimum values of delta x. This will be changed for fine-tuning the motion.

const double OBSTACLE_RADIUS = 200;


using namespace std;

bool quit = false;





// FOOTSTEP PLANNING FUNCTIONS BEGIN HERE

struct foot
{
	double delta_x,delta_y,delta_theta,vel_y;				//	An array of this structure will store the final output of the following program, namely, delta_x, delta_y, delta_theta
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


double calc_delta_x(double inner_radius , foot step)
{
	double delta_x = inner_radius - pow ((pow(inner_radius,2) - pow(step.delta_y,2)), 0.5);
	return delta_x;
}

double calc_delta_theta(double inner_radius , foot step)	
{
	// double delta_theta = fabs(acos((pow ((pow(inner_radius,2) - pow(step.delta_y,2)), 0.5))/inner_radius))*180/pi;
	double delta_theta = fabs(asin(step.delta_y/inner_radius))*180/pi;
	return delta_theta;
}

int constraint_check(foot step , double vf , int loop_num)
{
	if (loop_num == 1 && step.delta_y>2*max_delta_y/3)
		return 2;
	else if (loop_num ==2 && step.delta_y<min_delta_y)
		return 3;
	else if (vf>max_velocity/2)
		return 4;
	else if (step.delta_x>max_delta_x)
		return 5;
	else if (step.delta_theta>max_delta_theta)
		return 6;		
	else
		return 1;
}

void convert_values (PathPacket pathpackvarlocal, double dist_circstart[], double theta_arc[], double radius[],	int footlr[])
{
		// dist[0]	=	distance(pathpackvar.finalpath[0].x,pathpackvar.finalpath[0].y,pathpackvar.finalpath[k-1].x,pathpackvar.finalpath[k-1].y); IS THIS NEEDED?
		double ox = 0;
		double oy = 0;	
		double x1, y1, x2, y2;

		double dirx=1, diry=0;
		for (int i = 0;	i<=pathpackvarlocal.no_of_points-2; i+=2)
		{
			dirx = x2-x1;
			diry = y2-y1;

			x1 = pathpackvarlocal.finalpath[i].x;
			y1 = pathpackvarlocal.finalpath[i].y;
			x2 = pathpackvarlocal.finalpath[i+1].x;
			y2 = pathpackvarlocal.finalpath[i+1].y;

			double vector_product = (x1-ox)*(y2-y1) - (y1-oy)*(x2-x1);

			footlr[i/2] = (vector_product>0)?1:0;
			radius[i]	=	OBSTACLE_RADIUS;

			double theta = 180*2*acos(sqrt(1 - pow((10*distance(ox,oy,x1,y1)/(2*radius[i])),2)))/pi;

			int turn = ((dirx*(y2-y1) - diry*(x2-x1))>0)?1:0;
			if (turn)
				theta_arc[i/2] = (footlr[i/2])?(360-theta):theta;				
			else if (!turn)
				theta_arc[i/2] = (!footlr[i/2])?(360-theta):theta;

			dist_circstart[i/2] = 10*distance(x1,y1,x2,y2);
			// printf ("X[i] = %f\tY[i]=%f\n",pathpackvar.finalpath[pathpackvar.no_of_points - 2*i].x,pathpackvar.finalpath[pathpackvar.no_of_points - 2*i].y);
			// printf ("X[i+1] = %f\tY[i+1]=%f\n", pathpackvar.finalpath[pathpackvar.no_of_points - 2*i -1].x,pathpackvar.finalpath[pathpackvar.no_of_points - 2*i -1].y);
			// printf ("X[i+2] = %f\tY[i+2]=%f\n",pathpackvar.finalpath[pathpackvar.no_of_points - 2*i-2].x,pathpackvar.finalpath[pathpackvar.no_of_points - 2*i-2].y);
			printf("i = %d\tDist_circstart = %f\tRadius = %f\tTheta_arc	= %f\tfootlr = %d\n",i/2,dist_circstart[i/2],radius[i/2],theta_arc[i/2],footlr[i/2]);

			ox = x2;
			oy = y2;

		}
}


void imagepaint(foot step[] , int count, int footlr[])
{
	double yl = 200 - foot_separation/20 , xl = 200 , thetal = 0;
	double yr = 200 + foot_separation/20 , xr = 200, thetar = 0;
	IplImage* footimage=cvCreateImage(cvSize(752,480),8,3);
	cvSet(footimage, CV_RGB(255,255,255));
	cvLine(footimage,cvPoint(xl,(yl+yr)/2),cvPoint(xl + 100 , (yl+yr)/2),cvScalar(255,0,0)); 
	cvCircle(footimage, cvPoint(300,200+40),40, cvScalar(255,0,0));
	
	float y1 = 240 - 40*cos(deg2rad(70));
	float x1 = 300 + 40*sin(deg2rad(70));

	float y2 = y1 + 100*sin(deg2rad(70));
	float x2 = x1 + 100*cos(deg2rad(70));
	
	cvLine(footimage,cvPoint(x1, y1), cvPoint(x2, y2), cvScalar(255,0,0) );
	
	float x3 = x2 + 40*sin(deg2rad(70));
	float y3 = y2 - 40*cos(deg2rad(70));
	cvCircle(footimage, cvPoint(x3, y3), 40, cvScalar(255,0,0));

	for (int i = 0 ; i<(count-1) ; i++)
	{
		if (i%2 == 0)		
		{
			if (step[i].delta_x == 0 && i!=0)	
			{
				xl += (step[i].delta_y*cos(deg2rad(thetal)) + (step[i-1].delta_x)*sin(deg2rad(thetal)))/10;
				yl += (step[i].delta_y*sin(deg2rad(thetal))	+ (step[i-1].delta_x)*cos(deg2rad(thetal)))/10;
			}
			else
			{
				xl += ((step[i].delta_y )*cos(deg2rad(thetal)) + (step[i].delta_x)*sin(deg2rad(thetal)))/10;
				yl += ((step[i].delta_y )*sin(deg2rad(thetal)) + (step[i].delta_x)*cos(deg2rad(thetal)))/10;
			}
			thetal += step[i].delta_theta;
			// cvLine(footimage,cvPoint(xl+7,yl),cvPoint(xl +7+ 0.1*cos(deg2rad(thetal)) , yl + 0.1*sin(deg2rad(thetal)) ),cvScalar(0,0,0)); 
			if ((i+1)%7 == 0)
				cvCircle(footimage, cvPoint(xl+11,yl-7), 10, cvScalar(0,255,0));
		}
		else
		{
			if (step[i].delta_x == 0 && i!=0)
			{
				xr += (step[i].delta_y*cos(deg2rad(thetar)) + step[i-1].delta_x*sin(deg2rad(thetar)))/10;
				yr += (step[i].delta_y*sin(deg2rad(thetar)) + step[i-1].delta_x*cos(deg2rad(thetar)))/10;
			}
			else
			{
				xr += ((step[i].delta_y)*cos(deg2rad(thetar)) + step[i].delta_x*sin(deg2rad(thetar)))/10;
				yr += ((step[i].delta_y)*sin(deg2rad(thetar)) + step[i].delta_x*cos(deg2rad(thetar)))/10;
			}
			thetar += -step[i].delta_theta;
			// cvLine(footimage,cvPoint(xr,yr),cvPoint(xr + 0.1*cos(deg2rad(thetar)) , yr + 0.1*sin(deg2rad(thetar)) ),cvScalar(0,0,0)); 
			if ((i%7) == 0)
				cvCircle(footimage, cvPoint(xr-6,yr+7), 10, cvScalar(0,255,0));

		}

	// cvLine(image,cvPoint(n1.x+200,n1.y+200),cvPoint(n2.x+200,n2.y+200),cvScalar(255,255,0)); //Ellipse needed
	// cvLine(image,cvPoint())
		cvShowImage("Field", footimage);
		// cvZero(footimage);
		cvWaitKey();
	}
	cvSaveImage("fsp.jpg" ,footimage);
	cvZero(footimage);

	cvWaitKey();
}

int footstepmain(double v_initial , double initial_delta_y , int lr ,  foot step[] , int & stepcount , PathPacket pathpackvarlocal)
{
	int count =	0,check = 0;
	int count_ref = 0;
	//Linear motion is straightforward(haha) : The bot accelerates to its maximum step-length possible, and stays there until it arrives at a circle.
	//Outlined below is the code for linear motion. Delta x and delta theta remain zero throughout the motion and the code is written thusly.
	double vd = v_initial;						//	vd is initialized with initial velocity
	step[0].vel_y = vd;
	double vf;
	if (initial_delta_y == 0)
	{
		step[0].delta_y = first_delta_y;
	}
	else
		step[0].delta_y = initial_delta_y;			//	delta_length is initialized with initial step length	
	if (step[0].delta_y >= max_delta_y - 0.01)
	{
		step[1].delta_y = (vd + c5*step[0].delta_y/2)/c5;
		step[1].delta_x = 0;
		step[1].delta_theta = 0;
		vd  = c5*step[0].delta_y/2;
		step[1].vel_y = vd;	
		count++;
	}

	//printf("\n\n\n v_initial = %f\n\n\n",v_initial);
	// const int no_circles = ;
	double theta_arc[30];					//	This will later be changed to an input.
	double radius[30];					//	This will later be changed to an input.
														//	This measures the angle travelled along the circular arc, in order to determine when it has to end circular motion.
	int footlr[30]; 
	double dist_circstart1[30];				//	This is the distance to the beginning of the circle from the beginning of motion.
	double dist_circstart[31];
		
	int i=0;	
	int no_obstacles;
	#ifdef IP_IS_ON
	convert_values (pathpackvarlocal,	dist_circstart1,	theta_arc,	radius,	footlr);
	// count =	0;									//	Counts number of footsteps
	no_obstacles = pathpackvarlocal.no_of_points/2;
	dist_circstart[0] = 0;
	for (int i = 0; i <= no_obstacles; ++i)
	{
		dist_circstart[i+1] = dist_circstart1[i];
	}
	#endif
	//Comment this portion for normal walk path - Currently this is conditioned on IP being off
	#ifndef IP_IS_ON
	no_obstacles = 1;
	theta_arc[0] = 70;
	radius[0] = 400;
	footlr[0] = 1;	
	dist_circstart[0] = 1000;

	theta_arc[1] = 80;
	radius[1] = 400;
	footlr[1] = 0;
	dist_circstart[1] = 1000;
	#endif
	//till here
	int fmscheck = 0;
	int first_circ_foot[4] , last_circ_foot[4]; //FSPMOD CODE
	for (int i=0;i<no_obstacles;i++)	
	{
	
		footlr[i] = pow((footlr[i] - lr),2); 
		double dist_covered = 0	;					//	Corresponds to cal_dist_covered function.
		count_ref = count;
		fmscheck = 0;
		while (dist_covered<dist_circstart[i])
		{
			step[count+1].delta_y = c3 * vd - 0.01;
			if (step[count+1].delta_y>=max_delta_y)
			{
				if (fmscheck == 0)
				{
					step[count+1].delta_y = max_delta_y;
					fmscheck++;
				}
				else if (fmscheck == 1)
				{
					step[count+1].delta_y = (vd + c5*step[count].delta_y/2)/c5;
					fmscheck++;
				}
				else if (fmscheck == 2)
				{
					step[count+1].delta_y = max_delta_y;
				}
			}

			vf = c5*step[count+1].delta_y - vd;
			vd = vf;								//	It seems unnecessary to involve vf here, but this is done to establish a standard for the code used to develop motion along
													//  the circle, where vf will be necessary. It will also be useful for later modifications.
			step[count + 1].delta_x = 0;
			step[count + 1].delta_theta = 0;
			step[count + 1].vel_y = vd;
			count++;
			dist_covered = calc_dist_covered(count,count_ref,step);
			// printf(" I was here %d\n",count);
		}
		first_circ_foot[i] = count + 1;	//FSPMOD CODE
						
		int constraint_value;
		double inner_radius = fabs(radius[i] - foot_separation/2); 
		double theta_current = 0;	
		check = 0;
		if (fmscheck == 2)
		{
			step[count+1].delta_y = c4 * vd + 0.01;
			step[count+1].delta_x = 0;
			step[count+1].delta_theta = 0;
			vf = c5*step[count+1].delta_y - vd;
			vd = vf;
			step[count + 1].vel_y = vd;
			count++;
			
			step[count+1].delta_y = c4 * vd + 0.01;
			step[count+1].delta_x = 0;
			step[count+1].delta_theta = 0;
			vf = c5*step[count+1].delta_y - vd;
			vd = vf;
			step[count + 1].vel_y = vd;
			count++;	
		}
		if (theta_arc[i]>=365 || theta_arc[i]<=-5 )
			check = 2;
		while(check!=2)
		{
			check=0;											
			while (check==0)
			{
				step[count+1].delta_y = c3 * vd - 0.01;
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
							vd=vf;
							step[count + 1].vel_y = vd;
							count++;
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
				step[count+1].delta_y = c4 * vd + 0.01;
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
						vd = vf;
						step[count + 1].vel_y = vd;
						count++;
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
		last_circ_foot[i] = count;		//FSPMOD CODE
	}
	for (int i=0;i<count-1;i++)
	{
		// printf ("Delta_y = %f\tDelta_x = %f\tDelta_theta = %f\tVel_y = %f\n",step[i].delta_y,step[i].delta_x,step[i].delta_theta, step[i].vel_y);
	}
/*	//FSPMOD CODE
	for (int i=0; i<no_obstacles; i++)
	{	
		double outer_foot_dist = 0, inner_foot_dist = 0;
		for(int j = first_circ_foot[i] ; j <= last_circ_foot[i] ; j++)
		{
			if ((j + first_circ_foot[i])%2)
			{
				if (step[first_circ_foot[i]].delta_theta)
				{
					outer_foot_dist += step[j].delta_y;
				}
				else
					inner_foot_dist += step[j].delta_y;
			}
			else
			{
				if (step[first_circ_foot[i]].delta_theta)
				{
					inner_foot_dist += step[j].delta_y;
				}
				else
					outer_foot_dist += step[j].delta_y;
			}
			// printf("\n\nOuter Foot Distance = %f\n" , outer_foot_dist);
			// printf("Inner Foot Distance = %f\n\n" , inner_foot_dist);
		}
	}
	//FSPMOD CODE*/
	// printf("COUNT = %d",count);
	stepcount = count ;
	// imagepaint(step , count, footlr);
	return 0;
}

// FOOTSTEP PLANNING FUNCTIONS END HERE


void doquit(int para)
{
	quit = true;
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

void* walk_thread(void*)
{
	Imu imu;
	// printf("GENORAI\n");
	imu.init();
	foot foot1[1000];

	Communication comm;
	AcYut bot(&comm,&imu);
	Walk walk(&bot);
	int j=0,i=0;
	PathPacket pathpackvarlocal;
	// Call walkthread here instead of reading file.
	pthread_mutex_lock(&mutex_pathpacket);
	pathpackvarlocal = pathpackvar;
	cout<<"no_of_points "<<pathpackvar.no_of_points<<endl;
	for (int i1 = 0; i1 < pathpackvar.no_of_points; ++i1)
	{
		cout<<"x: "<<pathpackvar.finalpath[i1].x<<"y; "<<pathpackvar.finalpath[i1].y<<endl;
	}

	pthread_mutex_unlock(&mutex_pathpacket);
	footstepmain(10 , foot1[j].delta_y , j%2 ,  foot1 , i , pathpackvarlocal);
	double r = 0, x = 0, y = 0, theta = 0;
	while (1)
	{
		// printf("in walk thread\n");
		j = 0;
		while (j<i-1 )
		{
			// walk.dribble(foot1[j].delta_y/2,foot1[j].delta_x,foot1[j].delta_theta,0);
			walk.pathdribble(foot1[j].vel_y, foot1[j].delta_x,foot1[j].delta_theta,0);
			if (j%2 == 0)		
				{
					if (foot1[j].delta_x == 0 && j!=0)	
					{
						x += foot1[j].delta_y*cos(deg2rad(theta)) + (foot1[j-1].delta_x)*sin(deg2rad(theta));
						y += foot1[j].delta_y*sin(deg2rad(theta))	+ (foot1[j-1].delta_x)*cos(deg2rad(theta));
					}
					else
					{
						x += (foot1[j].delta_y )*cos(deg2rad(theta)) + (foot1[j].delta_x)*sin(deg2rad(theta));
						y += (foot1[j].delta_y )*sin(deg2rad(theta)) + (foot1[j].delta_x)*cos(deg2rad(theta));
					}
					theta += foot1[j].delta_theta;
					r = sqrt(pow(x,2) + pow(y,2));
				}
			// printf("step = %d theta = %f delta_x = %f delta_y = %f vel_y = %f\n" ,j, foot1[j].delta_theta , foot1[j].delta_x, foot1[j].delta_y, foot1[j].vel_y);
			
			pthread_mutex_lock(&mutex_motionModel);
			motionModel.update(r,atan(y/x)*180/pi);
			pthread_mutex_unlock(&mutex_motionModel);
			j++;
		}
		j--;
		pthread_mutex_lock(&mutex_pathpacket);
		pathpackvarlocal = pathpackvar;
		r = 0;
		x = 0;
		y = 0;
		theta = 0;
		cout<<'no_of_points '<<pathpackvar.no_of_points<<endl;
		for (int i1 = 0; i1 < pathpackvar.no_of_points; ++i1)
		{
			cout<<"x: "<<pathpackvar.finalpath[i1].x<<"y; "<<pathpackvar.finalpath[i1].y<<endl;
		}
		pthread_mutex_unlock(&mutex_pathpacket);
		footstepmain(walk.velocity() , foot1[j].delta_y , j%2 ,  foot1 ,i , pathpackvarlocal);
	}



	//walk.turnright(35.;
	//while(1)
	//	balanceStatic(bot,1,DSP);
	return 0;
	
	
};



