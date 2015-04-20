# include "walk_thread.h"
// # include <unistd.h>
# include <pthread.h>

# define ARC_RADIUS 20  // in cm
int arc_angle(float x1, float x2, float y1, float y2)
{
	int phi;
	float perpendicular;

	perpendicular=sqrt(pow(x2-x1,2)+pow(y2-y1,2))/2;

	phi=180/3.1415*asin(perpendicular/ARC_RADIUS);



	return 2*phi;
}

int forward_move_length(float x1, float x2, float y1, float y2)
{
	int length;

	length=sqrt(pow(x1-x2,2)+pow(y1-y2,2));

	return length;
}

int vector_cross_reflex(float x1, float x2, float y1, float y2)   // whether to follow relfex or non reflex angle
{

	if(x1*y2-x2*y1<0)
		return 1;
	else
		return -1;

}

int vector_cross_clock(float x1, float x2, float y1, float y2)  // whether to turn clockwise or anticlockwise
{
	if (x1*y2-x2*y1<0)
	{
		return 1;
	}
	else
		return -1;
}


void* walk_thread(void*)//don't know why such prototype
{

	Walk walk;  // object to access move function

	PathPacket pathpackvarlocal;

	int phi, radius;
	int x_initial=1, y_initial=0, flag=0;
	int reflex;
	int direction;
	bool flagStop = 0;

		while(1)
		{
			// printf("%s\n","Entering while(1)" );
			pthread_mutex_lock(&mutex_pathpacket);
				// printf("walk_thread locked\n");

				if(pathpackvar.updated==1)
				{
					pathpackvarlocal=pathpackvar;
					pathpackvar.updated=0;
					// flagStop = 1;

				}
			pthread_mutex_unlock(&mutex_pathpacket);
				// printf("walk_thread unlocked\n");


			// printf("%d\n",pathpackvarlocal.no_of_points );
			for(int i=0;i<pathpackvarlocal.no_of_points;i++)
			{
				// printf("%f\n",pathpackvarlocal.finalpath[i].x );
				// printf("%f\n",pathpackvarlocal.finalpath[i].y );
			}



		
			for(int i=0;i<pathpackvarlocal.no_of_points;i++)
			{
				// printf("%s\n","Entering for loop" );

				/*
						First packet is arc, second 
						packet is straight line, and so on 
				*/


				if (pathpackvarlocal.no_of_points==1&&(pathpackvarlocal.BACK_WALK==1 || pathpackvarlocal.NEAR_OBSTACLE))
				{
					radius=forward_move_length(0, pathpackvarlocal.finalpath[i].x,0, pathpackvarlocal.finalpath[i].y);
					// printf("%f\n",radius );
 
					// fflush(stdout);
					walk.move(-1*radius,0);
					// fflush(stdout);
					usleep(1000000*(radius+2.5)/(0.145*255));
					usleep(1000000);

					continue;
				}
				// else if(pathpackvarlocal.no_of_points==1&&pathpackvarlocal.BACK_WALK==0)
				// {
				// 	walk.move(9999,9999);
				// 	usleep(1000000);
				// 	continue;
				// }
				// else
				// 	{;} // proceed
				

				if(i%2==0)
				{
					if(i==0)
						phi=arc_angle(0,pathpackvarlocal.finalpath[i].x,0,pathpackvarlocal.finalpath[i].y);
					else
						phi=arc_angle(pathpackvarlocal.finalpath[i-1].x,pathpackvarlocal.finalpath[i].x,pathpackvarlocal.finalpath[i-1].y,pathpackvarlocal.finalpath[i].y);

					
					if(i<2)
						reflex=vector_cross_reflex(x_initial,pathpackvarlocal.finalpath[i+1].x- pathpackvarlocal.finalpath[i].x ,y_initial,pathpackvarlocal.finalpath[i+1].y- pathpackvarlocal.finalpath[i].y);
					else if(i!=pathpackvarlocal.no_of_points-1)
						reflex=vector_cross_reflex(pathpackvarlocal.finalpath[i-1].x - pathpackvarlocal.finalpath[i-2].x , pathpackvarlocal.finalpath[i+1].x - pathpackvarlocal.finalpath[i].x ,pathpackvarlocal.finalpath[i-1].y - pathpackvarlocal.finalpath[i-2].y, pathpackvarlocal.finalpath[i+1].y - pathpackvarlocal.finalpath[i].y );
					else
						reflex=-1;
					


					if(i==0)
						direction=vector_cross_clock(x_initial,pathpackvarlocal.finalpath[i].x+pathpackvarlocal.finalpath[i+1].x,y_initial,pathpackvarlocal.finalpath[i].y+pathpackvarlocal.finalpath[i+1].y);
					else if(i!=pathpackvarlocal.no_of_points-1)
						direction=vector_cross_clock(pathpackvarlocal.finalpath[i].x- pathpackvarlocal.finalpath[i-1].x,pathpackvarlocal.finalpath[i].x+pathpackvarlocal.finalpath[i+1].x- pathpackvarlocal.finalpath[i-1].x ,pathpackvarlocal.finalpath[i].y- pathpackvarlocal.finalpath[i-1].y,pathpackvarlocal.finalpath[i].y+pathpackvarlocal.finalpath[i+1].y- pathpackvarlocal.finalpath[i-1].y);
					else
						direction=-1;

					// ostream::flush;

					if(reflex==-1&&direction==-1)
						walk.move(0,phi);
					else if(reflex==1&&direction==-1)
						walk.move(0,360-phi) ;
					else if(reflex==-1&&direction==1)
						walk.move(0,-1*(360-phi));
					else
						walk.move(0,-1*phi);
					// ostream::flush;
					// nanosleep(100000);
					// usleep(1000000);
					usleep(1.5*1.75 *abs(phi*80/9)*1000);
					usleep(500000);
					// ostream::flush;
					
					
										// arc traversal
				}
				else
				{

					radius=forward_move_length(pathpackvarlocal.finalpath[i-1].x, pathpackvarlocal.finalpath[i].x,pathpackvarlocal.finalpath[i-1].y, pathpackvarlocal.finalpath[i].y);


					// fflush(stdout);
					walk.move(radius,0);
					// fflush(stdout);
					usleep(1000000*(radius+2.5)/(0.145*255));
					usleep(500000);
					// fflush(stdout);
					
					// straight line traversal
				}
				if(i%2==1)
				{
					pthread_mutex_lock(&mutex_pathpacket);
						// printf("walk_thread locked\n");
						if(pathpackvar.updated==1)
						{
							// printf("%s\n","breaking out of for loop" );
							pthread_mutex_unlock(&mutex_pathpacket);
							break;
						}
					pthread_mutex_unlock(&mutex_pathpacket);
						// printf("walk_thread unlocked\n");
				}

			}	
			// printf("%s\n","while(1) ending" );
		}	
}