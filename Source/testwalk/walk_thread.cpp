#include "walk_thread.h"

#define distance(x1,y1,x2,y2)	sqrt(pow(y2-y1,2) + pow(x2-x1,2))
#define convdegrad(p)	p*acos(-1)/180

const double pi = acos(-1);
//	c1, c2... are constants for equations which are outlined below 	
//	vd = Current velocity, vf = Final velocity, delta_y = Step length of the foot when it moves from back to front
													//	vd is taken as velocity at the beginning of the step and vf as vlocity at the end of the step

											//	vf_max = c1*vd			Maximum possible velocity at the end of the step
const double c1=1.981991;							// For zmax = 65 ,  1.8367262
const double c2=(c1 - pow(pow(c1,2)-1,0.5));		//	vf_min = c2*vd			Minimum possible velocity at the beginning of the step
// const double c5=7.853317745/2;						// 	vf + vd = c5*delta_y 	Generalised equation which connects the above equations
const double c5=3.523127;						//	For zMax = 65 , 7.445192041/2
const double c3=(c1+1)/c5;				//	delta_y_max = c3*vd		Maximum step length possible, this corresponds to equation 1
const double c4=(c2+1)/c5;				//	delta_y_min	= c4*vd		Minimum step length possible, this corresponds to equation 2
const double v_initial = 10;				 //	Velocity when bot begins to move
const double first_delta_y= 2*v_initial/c5;//	Initial step length is calculated by using equation 4 and given initial velocity which is assumed constant, i.e, vf=vd=vi
const double max_velocity = 90;//	Maximum velocity corresponding to above delta y assuming step length is constant for the motion	
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






WalkPacket convertPathPacket()
{
	WalkPacket w;
	w.id=pathpackvar.id;
	w.no_of_points=pathpackvar.no_of_points;

	if(pathpackvar.pathType==1) /* if it is in r-theta form : note that x,y represent r-theta here directly*/
		{

		for(int i=0;i<w.no_of_points;++i)
		{
			w.finalPath[i].r=pathpackvar.finalpath[i].x;

			w.finalPath[i].theta=pathpackvar.finalpath[i].y;	
		}	

		return w;
		}
	// for(int i=0;i<p.no_of_points;++i)
		// printf("Packet before conversion is %f %f\n",p.finalpath[i].x,p.finalpath[i].y);

	double x,y;
	double theta;
	for(int i=0;i<w.no_of_points;++i)
		{
			x=pathpackvar.finalpath[i].x;
			y=pathpackvar.finalpath[i].y;
			w.finalPath[i].r     = sqrt(x*x+y*y);

			if(fabs(x)<0.0001)
				{
					if(y>0)
						theta=pi/2;
					else 
						theta=-pi/2;
				}
			else
			{
			theta = atan(y/x);

				if(y<0 && x<0)
						theta-=pi;
				else if(x<0 && y>0)
						theta+=pi;
			}

			w.finalPath[i].theta=theta;	
		}
	return w;
}

void doquitWalk()
{
	// walk.move(0.00,0.00);
	// printf("Walk stopped");
	// exit(0);
};

void* walk_thread(void*)
{
	
	printf("in walkthread\n"); //----> DONT REMOVE THIS OR WALKTHREAD WONT WORK
	// Communication comm;
	// testBot bot();
	Walk walk;
	// usleep(500000);
	// double pi=acos(-1);
	// walk.move(100.0,0);	

	
	// // while(1)
	// // {
	// 	// printf("in WT\n");
	// 	printf("WT started\n");
	// 	walk.move(100,0);
	// // }
	// // (void) signal(SIGINT,doquitWalk);


	PathPacket pathpackvarlocal;
	WalkPacket walkpacket;
	Coords coords;
	walkpacket.no_of_points=30;
	int fps=30.0;
	vector<int> executed(30,1);
	while (1)
	{
		printf("%s\n","inside while1" );
			// printf("Size is %d\n",pathpackvar.no_of_points);
			pthread_mutex_lock(&mutex_pathpacket);
					if(pathpackvar.updated==1)
						{
							// printf("before conversion\n");
							walkpacket=convertPathPacket();
							// printf("after conversion\n");
							pathpackvarlocal=pathpackvar;
							pathpackvar.updated=0;
							executed.assign(30,0);
						}		
			pthread_mutex_unlock(&mutex_pathpacket);
			cout<<pathpackvarlocal.no_of_points<<endl;
		for(int i=0;i<pathpackvarlocal.no_of_points-2;++i)
		//int i=0;
		//while(1)
		{

			pthread_mutex_lock(&mutex_pathpacket);
					if(pathpackvar.updated==1)
						{
							walkpacket=convertPathPacket();
							i=0;
							pathpackvar.updated=0;
							executed.assign(30,0);
						}		
			pthread_mutex_unlock(&mutex_pathpacket);
			// #ifndef ALL_PRINTING_OFF
			// printf("1\n");
			// walk.move(10,30);
			// #endif
			// printf("%d\n",executed[i] );
			if(!executed[i])
				{
					if(walkpacket.finalPath[i].r>0.00005)
					{
						printf("Size is %d\n",walkpacket.no_of_points);
						printf("Path sent signal %f %f\n",walkpacket.finalPath[i].r,walkpacket.finalPath[i].theta);
					}


					// anantanurag
					assert(i<27);
					if(i%2==0)// Obstacle radius == 20 cm
					{
						int j=0,phi;
							printf("%s\n","hello1" );
						if((pathpackvarlocal.finalpath[i].x-1)*(pathpackvarlocal.finalpath[i+2].y-pathpackvarlocal.finalpath[i+1].y)-(pathpackvarlocal.finalpath[i].y)*(pathpackvarlocal.finalpath[i+2].x-pathpackvarlocal.finalpath[i+1].x)<0)
							{

								phi=2*asin(sqrt(pow(pathpackvarlocal.finalpath[i].x-pathpackvarlocal.finalpath[i+1].x,2)+pow(pathpackvarlocal.finalpath[i].x-pathpackvarlocal.finalpath[i+1].x,2)));
								printf("%s %d\n","to move theta",phi );
								if((pathpackvarlocal.finalpath[i].x-1)*(pathpackvarlocal.finalpath[i+1].y+pathpackvarlocal.finalpath[i+2].y)-(pathpackvarlocal.finalpath[i].y-pathpackvarlocal.finalpath[i-1].y)*(pathpackvarlocal.finalpath[i+1].x+pathpackvarlocal.finalpath[i+2].x) >0)
									walk.move(0,phi);
								else
									walk.move(0,-1*phi);
								printf("%s\n","hello2" );
							}
							
						else
						{
							phi=360-2*asin(sqrt(pow(pathpackvarlocal.finalpath[i].x-pathpackvarlocal.finalpath[i+1].x,2)+pow(pathpackvarlocal.finalpath[i].x-pathpackvarlocal.finalpath[i+1].x,2)));
							//if(pathpackvarlocal.finalpath.x[i]*(pathpackvarlocal.finalpath.y[i+2]-pathpackvarlocal.finalpath.y[i+1])-pathpackvarlocal.finalpath.y[i]*(pathpackvarlocal.finalpath.x[i+2]-pathpackvarlocal.finalpath.x[i+1]))
							printf("%s %d\n","to move theta",phi );
							if((pathpackvarlocal.finalpath[i].x-1)*(pathpackvarlocal.finalpath[i+1].y+pathpackvarlocal.finalpath[i+2].y)-(pathpackvarlocal.finalpath[i].y)*(pathpackvarlocal.finalpath[i+1].x+pathpackvarlocal.finalpath[i+2].x) >0)
									walk.move(0,phi);
								else
									walk.move(0,-1*phi);
						}
						printf("%s\n","before 1st sleep" );
						usleep(2*abs(phi*80/9)*1000);

					}
					else
					{
						printf("%s %d\n","to move radius",sqrt(pow(pathpackvarlocal.finalpath[i+1].x-pathpackvarlocal.finalpath[i].x,2)+pow(pathpackvarlocal.finalpath[i+1].y-pathpackvarlocal.finalpath[i].y,2)));
						walk.move(sqrt(pow(pathpackvarlocal.finalpath[i+1].x-pathpackvarlocal.finalpath[i].x,2)+pow(pathpackvarlocal.finalpath[i+1].y-pathpackvarlocal.finalpath[i].y,2)),0);
						usleep(2*1000*(sqrt(pow(pathpackvarlocal.finalpath[i+1].x-pathpackvarlocal.finalpath[i].x,2)+pow(pathpackvarlocal.finalpath[i+1].y-pathpackvarlocal.finalpath[i].y,2))+2.5)/(0.145*255));
					}
					printf("%s\n", "after second sleep");

					//usleep((1.5*abs(theta*80/9)+(sqrt(sq(pathpackvarlocal.finalPath[i+1].x-pathpackvarlocal.finalPath[i].x)+sq(pathpackvarlocal.finalPath[i+1].y-pathpackvarlocal.finalPath[i].y))+2.5)/(0.145*255))*1000);

					





					
					// double a=walkpacket.finalPath[i].r;
					// double b=walkpacket.finalPath[i].theta;
					
					// walk.move(a,b);
					// walk.move(0.005,0.08);
					// printf("1\n");
					//walk.move(walkpacket.finalPath[i].r,walkpacket.finalPath[i].theta); // Move this line to if else blocks
					pthread_mutex_lock(&mutex_motionModel);
					motionModel.update(walkpacket.finalPath[i].r,walkpacket.finalPath[i].theta);
					pthread_mutex_unlock(&mutex_motionModel);
					executed[i]=1;
					//i++;
					
				}
				pthread_mutex_lock(&mutex_pathpacket);
						if(pathpackvar.updated==1||i>28)
						{
							break;
							
							
							
						}
						else if(pathpackvar.updated==0||i>28)
						{
							printf("%s\n","path packet not updated and path traversed" );
							break;

						}
						else
						{

						}
					pthread_mutex_unlock(&mutex_pathpacket);
				printf("%s\n","inside while 2 when not broken" );
		}
		printf("%s\n","while 2 broken" );
	}



	
	return 0;
	
	
};



