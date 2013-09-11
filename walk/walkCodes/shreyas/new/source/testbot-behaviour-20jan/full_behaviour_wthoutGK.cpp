#include <stdio.h>
#include <math.h>
#include <sys/time.h>
#include <unistd.h>
#include <stdlib.h>
#include <inttypes.h>
#include <string.h>  /* String function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include <time.h>
#include <ftdi.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <pthread.h>

#include <errno.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>

#include <pthread.h>
#include <string.h>
//path as it was on 21-9-2012(no initial orientation- only obs avoidance and final orientation) will only work on testbot

#define TESTINGBOT
//#define GAMECONTROLLERACTIVE
//#define SWITCHACTIVE
#define WIFITRANSMISSION
#include "commondefs.h"
#include "headmotor.h"
#include "camcapture.h"
#include "camcontrol.h"
#include "featuredetection.h"
#include "localize.h"
#include "defines.h"
#include "path.hpp"

#ifndef TESTINGBOT
#include "walk_runtime.c"//contains callWalk function
#endif

#ifdef SWITCHACTIVE
#include "acyut_gpio_func.cpp"
#endif

#ifdef GAMECONTROLLERACTIVE
#include "gamecontrollerfunc.h"
#include "gamecontrollerobj.h"
#endif

#ifdef TESTINGBOT
#include "testbot.c"
#endif




//TYPE DEFINITIONS
//Also see commondefs.h

enum WalkCommands {INIT,WALK,ENCIRCLE,GETUPFRONT,GETUPBACK,KICK};//behaviour will calculate r and theta from path and continue

typedef struct
{
	float r;
	float theta;
	float encircleAngle;
	MotionModel mm;
}walkstructure;


enum Goal{BLUEGOAL=0,YELLOWGOAL=1};

//OTHER DEFINITIONS
#define NO_OF_FRAMES_TO_WAIT_B4_KICKOFF 50
#define ENCIRCLE_THRESHOLD 20 //min dist from ball where goalfinding starts
#define KICKDIST 15 //dist in cm from ball where kicking is done
#define MINenAngle 15 //minimum angle for encircling
//distances in cm

#define LM_PREVFACT 0.5
#define LM_WT 0.72165
#define B_PREVFACT 0.5
#define B_WT 1.0
#define B_WT2 1
#define TH 0.6
#define TL 0.03

//VARIABLE DECLARATIONS
//State currentstate=INITIAL;
BallReturns b;
GoalReturns g;
WalkCommands bodycommand;
Flags globalflags;
Goal mygoal=YELLOWGOAL;//0=blue,1=yellow
//int flag_encircle;
int Flag_moving=0;//0 if not walking
int Flag_moving_local;//for walkthread
int Flag_moving_mainlocal;//for main

pthread_mutex_t mutex_walkflag=PTHREAD_MUTEX_INITIALIZER;
//pthread_mutex_t mutex_encircleflag=PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t mutex_walkstr=PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t mutex_bodycommand=PTHREAD_MUTEX_INITIALIZER;
walkstructure walkstr;
CamCapture capture;
FeatureDetection fd(capture);
HeadMotor hdmtr;
CamControl camcont(capture);
Localize loc;
float bwt,lmwt;
int ballnotinframe;
MotionModel localmm;
double conf;
PathStructure pathstr;
Path pathobj;
#ifdef GAMECONTROLLERACTIVE
GameControllerAcyut GC;
#endif

#ifdef WIFITRANSMISSION
#include "communication.cpp"
#endif

//walk function-walk to r,theta------>this function will keep on reading *Flag_moving* and *flag_encircle*, if true then it will use the updated r and theta or other data, make acYut walk or encircle and reset the flag.
//do* functions belong to Behaviour, they just copy values to the structure used by WALK
void doEncircle(float angle)
{
	//printf("\nEncircling.......................................\n");
	//compensation for encircling(if goal is towards right, encircle towards left and vice versa) is done in walkthread(ie callEncircle)!!!!,
	pthread_mutex_lock(&mutex_walkstr);
	walkstr.encircleAngle=angle;
	pthread_mutex_unlock(&mutex_walkstr);
}

void doWalk(float r1,float theta1)//CONTROLS WHAT VALUES ARE BEING USED BY WALK
{
	//a=dist of ball from AcYut in cm
	//b=angle in degrees(left=-ve,right=+ve)
	//printf("\nwalking...........................................\n");
	pthread_mutex_lock(&mutex_walkstr);
	walkstr.r=10*r1;
	walkstr.theta=theta1;//will be read by walk_thread and used to call walk
	pthread_mutex_unlock(&mutex_walkstr);
}
inline void stopWalk()
{
	pthread_mutex_lock(&mutex_walkflag);
	Flag_moving=0;
	pthread_mutex_unlock(&mutex_walkflag);

}
//int walk3(int leg,float yin,float yfi,float yrin,float yrfi, float zin, float zfi,float zrin, float zrfi, float thetain, float thetafi,float thetarin, float thetarfi) //walk charli
#ifdef TESTINGBOT
#define MINTHETA 15.0//degrees
#define STEPLEN 150.0//mm
#define WHEELDIAmm 100.0 //mm
#define WHEELDIST 240.0 //mm
#define tbFORWARD 0
#define tbBACKWARD 1
#define tbLEFT 3
#define tbRIGHT 2
#define tbSTOP 0,0
#define MSPEED 440.0
void callWalk(float walkr,float walktheta,int override=0)
{
	//walkr--distance to walk in mm
	//walktheta-- angle to move in degrees
	//rotate
	double delta;
	double arclen;
	//printf("[walkthread]   walk cmd received angle=%f",enAngle);
	if(override!=0)//overrides min values
	{
		if(walktheta<0)
		{
			walktheta*=-1;
			arclen=(walktheta/180.0)*2*3.1415*WHEELDIST;
			delta=(arclen/(2*3.1415*WHEELDIAmm*MSPEED/360))*1000000;
			testBotWalk1(tbLEFT,512);
			usleep((int)delta);
			testBotWalk1(tbSTOP);
			pthread_mutex_lock(&mutex_walkstr);
			walkstr.mm.theta=-walktheta;
			pthread_mutex_unlock(&mutex_walkstr);
		}
		else if(walktheta>0)
		{
			pthread_mutex_lock(&mutex_walkstr);
			walkstr.mm.theta=walktheta;
			pthread_mutex_unlock(&mutex_walkstr);
			arclen=(walktheta/180.0)*2*3.1415*WHEELDIST;
			delta=(arclen/(2*3.1415*WHEELDIAmm*MSPEED/360))*1000000;
			testBotWalk1(tbRIGHT,512);
			usleep((int)delta);
			testBotWalk1(tbSTOP);
		}
		delta=(walkr/(3.1415*WHEELDIAmm*MSPEED/360))*1000000;
		testBotWalk1(tbFORWARD,512);
		usleep((int)(delta/0.6));
		testBotWalk1(tbSTOP);
		pthread_mutex_lock(&mutex_walkstr);
		walkstr.mm.r=walkr;
		walkstr.mm.updated=1;
		pthread_mutex_unlock(&mutex_walkstr);
		return ;
	}
	if(walktheta<0)
	{//turn left for DELTA usecs
		//s=r.theta
		//so to move theta degrees, s=(theta/180)*2*3.1415*WHEELDIST
		/* though this formula is wrong(by some factor, maybe 2^(some n)), adjustments to achieve accurate turning have been made in MSPEED for this error,
		 any change in the formula will require recalibration of mspeed for angle rotation!!!*/
		walktheta*=-1;
		if(walktheta>MINTHETA)
		{
			arclen=(MINTHETA/180.0)*2*3.1415*WHEELDIST;
			delta=(arclen/(2*3.1415*WHEELDIAmm*MSPEED/360))*1000000;
			testBotWalk1(tbLEFT,512);
			usleep((int)delta);
			testBotWalk1(tbSTOP);
			pthread_mutex_lock(&mutex_walkstr);
			walkstr.mm.r=0;
			walkstr.mm.theta=-MINTHETA;
			pthread_mutex_unlock(&mutex_walkstr);

		}
		else
		{
			arclen=(walktheta/180.0)*2*3.1415*WHEELDIST;
			delta=(arclen/(2*3.1415*WHEELDIAmm*MSPEED/360))*1000000;
			testBotWalk1(tbLEFT,512);
			usleep((int)delta);
			testBotWalk1(tbSTOP);
			pthread_mutex_lock(&mutex_walkstr);
			walkstr.mm.r=0;
			walkstr.mm.theta=-walktheta;//IMP:Minus sign over here is because of multipication by -1 in the line above before "if"
			pthread_mutex_unlock(&mutex_walkstr);
		}

	}
	else if(walktheta>0)
	{//turn right for DELTA usecs
		if(walktheta>MINTHETA)
		{
			arclen=(MINTHETA/180.0)*2*3.1415*WHEELDIST;
			delta=(arclen/(2*3.1415*WHEELDIAmm*MSPEED/360))*1000000;
			testBotWalk1(tbRIGHT,512);
			usleep((int)(delta/0.6));
			testBotWalk1(tbSTOP);
			pthread_mutex_lock(&mutex_walkstr);
			walkstr.mm.r=0;
			walkstr.mm.theta=MINTHETA;
			pthread_mutex_unlock(&mutex_walkstr);
		}
		else
		{
			arclen=(walktheta/180.0)*2*3.1415*WHEELDIST;
			delta=(arclen/(2*3.1415*WHEELDIAmm*MSPEED/360))*1000000;
			testBotWalk1(tbRIGHT,512);
			usleep(delta/0.6);
			testBotWalk1(tbSTOP);
			pthread_mutex_lock(&mutex_walkstr);
			walkstr.mm.r=0;
			walkstr.mm.theta=walktheta;
			pthread_mutex_unlock(&mutex_walkstr);
		}

	}
	if((walktheta>=0&&walktheta<=5)||(walktheta<=0&&walktheta>=-5))
	{
		if(walkr>STEPLEN)
		{
			//if speed is 1023, angular speed is--476deg/sec for RX28!!!!!!!!!!
			//speed is 512 so angular speed is MSPEEDdeg/sec
			// check the above facts...
			//therefore (pi D)*(MSPEED/360) mm in one second ie speed=MSPEED*3.1415*WHEELDIAmm/360 mm per sec
			//therefore DELTA=(walkr/3.1415*WHEELDIAmm*MSPEED/360)
			delta=(STEPLEN/(3.1415*WHEELDIAmm*MSPEED/360))*1000000;
			testBotWalk1(tbFORWARD,512);
			usleep(delta/0.6);
			testBotWalk1(tbSTOP);
			pthread_mutex_lock(&mutex_walkstr);
			walkstr.mm.r=STEPLEN;
			//walkstr.mm.updated=1;
			pthread_mutex_unlock(&mutex_walkstr);
		}
		else
		{
			delta=(walkr/(3.1415*WHEELDIAmm*MSPEED/360))*1000000;
			testBotWalk1(tbFORWARD,512);
			usleep(delta/0.6);
			testBotWalk1(tbSTOP);
			pthread_mutex_lock(&mutex_walkstr);
			walkstr.mm.r=walkr;
			//walkstr.mm.updated=1;
			pthread_mutex_unlock(&mutex_walkstr);
		}
	}
	pthread_mutex_lock(&mutex_walkstr);
	walkstr.mm.updated=1;
	pthread_mutex_unlock(&mutex_walkstr);
}
#endif
#ifndef TESTINGBOT
//

//TODO: Resolve how to do Motion Model for Acyut Encircle

//
int callEncircle(float enAngle)//expects goalangle in this convention --> positive=right negative=left; so a positive argument will encircle to the left and vice versa
{
	printf("****************ENCIRCLE****************\n");
	// if(status!=0)
	// {
	// 	walk2(flag,YR_END,0,Y_END,0,0,0,0,0,0,0,0,0,1);
	// 	usleep(16670*delay*2);
	// 	status=0;
	// }
	int side;
	if(enAngle>0)//positive angle==encircle left side
	side=1;//1=rightside
	else
	side=0;
	if(enAngle>MINenAngle||enAngle<-MINenAngle)
	{
		printf("**********enAngle>MINenAngle||enAngle<-MINenAngle******************\n");
		walk3(side,0,20,0,-20,0,30,0,30,0,-MINenAngle,0,0);
		usleep(50000);
		walk3(1-side,-20,0,20,0,30,0,30,0,0,0,-MINenAngle,0);
		usleep(50000);
	}
}
#endif

#ifdef TESTINGBOT
#define ENCIRCLERADIUS 400 //mm
int callEncircle(float enAngle)
{
  	// Function to en"circle" for diff drive bot
	//THIS FUNCTION ASSUMES: Bot faces the center of the circle of radius ENCIRCLERADIUS
	//then it takes steps of 20 degrees to land on the next point of the circle
	//+right, -left
	printf("[walkthread]   encircling cmd received angle=%f\n",enAngle);
	float base=(180 - MINenAngle)/2;
	if(enAngle>MINenAngle||enAngle<-MINenAngle)
	{
		//assuming a triangle made by 2 radius vectors from center at MINenAngle deg with each other, base angles are (180-minenangle)/2 so first rotate "base" degrees then reverse 100 degrees
		if(enAngle>0)
			callWalk(0,-base,1);
		else if(enAngle<0)
			callWalk(0,base,1);
		callWalk(2*ENCIRCLERADIUS*cos(base*3.14159/180.0)/*cos 15 degrees*/ ,0,1);
		if(enAngle>0)
			callWalk(0,(180-base),1);
		else if(enAngle<0)
			callWalk(0,-(180-base),1);
	}
}
#endif

void* walk_thread(void*)
{
	WalkCommands bodycommandlocal;
	while(1)
	{
		walkstructure localwalkstr;
		pthread_mutex_lock(&mutex_walkflag);
		Flag_moving_local=Flag_moving;
		pthread_mutex_unlock(&mutex_walkflag);
		pthread_mutex_lock(&mutex_bodycommand);
		bodycommandlocal=bodycommand;
		pthread_mutex_unlock(&mutex_bodycommand);
		pthread_mutex_lock(&mutex_walkstr);
		localwalkstr=walkstr;
		pthread_mutex_unlock(&mutex_walkstr);
		if(Flag_moving_local==1)//if walkflag is set ie. if behav is telling walkthread to walk
		{
			if(bodycommandlocal==WALK)
			{
				printf("[walkthread]   walk cmd sent; r=%f theta=%f\n",localwalkstr.r,localwalkstr.theta);
				callWalk(localwalkstr.r,localwalkstr.theta);
				// pthread_mutex_lock(&mutex_walkflag);
				// Flag_moving=0;
				// pthread_mutex_unlock(&mutex_walkflag);
				stopWalk();
			}
			if(bodycommandlocal==ENCIRCLE)
			{
				printf("[walkthread]  encircling cmd sent; angle=%f\n",localwalkstr.encircleAngle);
				callEncircle(localwalkstr.encircleAngle);
				// pthread_mutex_lock(&mutex_walkflag);
				// Flag_moving=0;
				// pthread_mutex_unlock(&mutex_walkflag);
				stopWalk();
			}
		}
		else
		{
			usleep(20000);
			continue;
		}
	}
}

#ifdef SWITCHACTIVE
void check_switch()
{
	int localswitch1;
	pthread_mutex_lock(&mutex_switch);
	localswitch1=switch1;
	pthread_mutex_unlock(&mutex_switch);

	if(localswitch1!=1)
	{
		printf("[behaviour]  Switch is OFF\n");
		exit(0);
	}
	printf("[behaviour]  Switch is On\n");
}
#endif

inline void updateWalkFlag()
{
	pthread_mutex_lock(&mutex_walkflag);
	Flag_moving_mainlocal=Flag_moving;
	pthread_mutex_unlock(&mutex_walkflag);
}

void updatewt(double ballratio)
{
	lmwt=LM_WT*(1-conf);//lmwt should be greater
	bwt=B_WT*(ballratio);//*(1-ballnotinframe);
	//bwt2=ballnotinframe*B_WT2;
}

void moveAcYut(double r,double theta,double enc_angle,WalkCommands com)//This Function sends data to walk thread using mutexlocks
{
	pthread_mutex_lock(&mutex_walkflag);
	int localflagmoving=Flag_moving;
	pthread_mutex_unlock(&mutex_walkflag);

	if(localflagmoving==0)
	{
		if(com==WALK)
		{
			pthread_mutex_lock(&mutex_walkstr);
			walkstr.r=r*10;
			walkstr.theta=theta;
			walkstr.encircleAngle=0;
			pthread_mutex_unlock(&mutex_walkstr);
		}
		else if(com==ENCIRCLE)
		{
			pthread_mutex_lock(&mutex_walkstr);
			walkstr.r=0;
			walkstr.theta=0;
			walkstr.encircleAngle=enc_angle;
			pthread_mutex_unlock(&mutex_walkstr);
		}

		pthread_mutex_lock(&mutex_bodycommand);
		bodycommand=com;
		pthread_mutex_unlock(&mutex_bodycommand);

		pthread_mutex_lock(&mutex_walkflag);
		Flag_moving=1;
		pthread_mutex_unlock(&mutex_walkflag);
	}
}

void convertPathData()
{
	//Path uses a coordinate system which roates along with acyut.
	//in that coordinate system, acyut always faces positive X axis and **Yaxis lies to its right**
	pathstr.n_obstacles=fd.n_obstacles;
	for(int i=0;i<fd.n_obstacles;i++)//assuming right positive and left neagative for theta
	{
		pathstr.absObstacles[i].y=(fd.obstacles[i].r)*cos(deg2rad(fd.obstacles[i].theta));
		pathstr.absObstacles[i].x=(fd.obstacles[i].r)*sin(deg2rad(fd.obstacles[i].theta));
	}
	AbsCoords goalcoords=loc.getGoalCoords(mygoal);
	double tempx=goalcoords.x-loc.selfX;
	double tempy=goalcoords.y-loc.selfY;
	pathstr.goal.x= (tempx*cos(deg2rad(loc.selfAngle))) - (tempy* sin(deg2rad(loc.selfAngle)));//Rotating coordinate system.
	pathstr.goal.y= (tempx*sin(deg2rad(loc.selfAngle))) + (tempy* cos(deg2rad(loc.selfAngle)));
	printf("Passed:-->>>>goal coords x:%lf  y:%lf\n",pathstr.goal.x,pathstr.goal.y);

	//printf("goal coords y:%lf\n",pathstr.goal.x);
	pathstr.ball.x=fd.ball.r*cos(deg2rad(fd.ball.theta));
	pathstr.ball.y=fd.ball.r*sin(deg2rad(fd.ball.theta));
	printf("relative ball----> %f  %f\n",fd.ball.r,fd.ball.theta);
	printf("Passed:-->>>>ball coords x:%lf  y:%lf\n",pathstr.ball.x,pathstr.ball.y);
}

int main()
{
	//bootup
	//initialise
	int kickofftimecount=0;

	#ifndef TESTINGBOT
		bootup_files_walk();
		//initialize_acyut();
	#endif

	#ifdef TESTINGBOT
		testbot_bootup();
	#endif
	//create threads

	#ifdef GAMECONTROLLERACTIVE
	GameControllerAcyut GC;
	pthread_t thread_id_gc;
    pthread_create (&thread_id_gc, NULL, readGameController, NULL);//function defined in its appropriate header file
	RoboCupGameControlData localgcdata;
	#endif

	pthread_t thread_id_walkthread;
	pthread_t thread_id_wifi;
	#ifdef 	SWITCHACTIVE
    void *statusgpio;
    pthread_t thread_id_gpio;
    pthread_create(&thread_id_gpio,NULL,switchupdate,NULL);//function defined in its appropriate header file
	
	led(3,1);
	led(4,0);


	while(1)//Master while
	#endif
	{
		#ifdef 	SWITCHACTIVE
		check_switch();//if on, does nothing, if off, exits program
		#endif
		#ifdef WIFITRANSMISSION
		pthread_create(&thread_id_wifi,NULL,main_wifitransmission,NULL);
		#endif
		pthread_create (&thread_id_walkthread, NULL, walk_thread, NULL);
		printf("[behaviour]  WALKTHREAD CREATED\n");
		hdmtr.bootup_files();
		capture.init();
		globalflags.reset();
		int nolocframecount=0;
		//currentstate=INITIAL;

		//INITIALISE---------------------------------------

		// pthread_mutex_lock(&mutex_bodycommand);
		// bodycommand=INIT;
		// pthread_mutex_unlock(&mutex_bodycommand);
		// pthread_mutex_lock(&mutex_walkflag);
		// Flag_moving=1;
		// pthread_mutex_unlock(&mutex_walkflag);
		// sleep(2);
		//INITIALISE--------------------------------------

		while(1)//main Behaviour starts
		{

			loc.printPosition();
			hdmtr.update();
			capture.getImage();
			cvShowImage("Image",capture.rgbimg);
			cvWaitKey(5);
			fd.getLandmarks(capture, hdmtr);
			cvShowImage("Localization", loc.dispImage);

			#ifdef SWITCHACTIVE
			check_switch();
			#endif
			#ifdef GAMECONTROLLERACTIVE
			pthread_mutex_lock(&mutex_GCData);
			localgcdata=GCData;
			pthread_mutex_unlock(&mutex_GCData);

			GC.update(localgcdata);
			int teamindex;
			teamindex=GC.getTeamIndex();
			//camcont.setGoal(GC.mygoal(teamindex));

			mygoal=GC.updategoal(teamindex);
			switch(localgcdata.state)//checking gamecontroller
			{
				case STATE_READY:printf("[behaviour] currentGCstate=READY");

								continue;

				case STATE_PLAYING:break;

				default:printf("[behaviour]  Gamecontroller: No PLAY signal yet\n");
						//currentstate=BALL1;
						//stopwalk
						stopWalk();
						#ifndef TESTINGBOT
							//INITIALISE---------------------------------------

							pthread_mutex_lock(&mutex_bodycommand);
							bodycommand=INIT;
							pthread_metex_unlock(&mutex_bodycommand);
							pthread_mutex_lock(&mutex_walkflag);
							Flag_moving=1;
							pthread_mutex_unlock(&mutex_walkflag);
							sleep(2);
							//INITIALISE--------------------------------------
						#endif
						continue;
			}
			if(GC.isMyKickOff())
			{
				printf("its my kickoff!!! :D \n\n%d\n\n",localgcdata.teams[localgcdata.kickOffTeam].teamNumber);
				//continue normally if not then dont walk for some time
			}
			else
			{
				printf("not my kickoff :/ \n%d\n\n",localgcdata.teams[localgcdata.kickOffTeam].teamNumber);
				//wait
			}

			if(localgcdata.teams[localgcdata.kickOffTeam].teamNumber!=myteamnumber)
			{
				kickofftimecount++;
				if(kickofftimecount<NO_OF_FRAMES_TO_WAIT_B4_KICKOFF)
				{
					printf("[behaviour] WALK stopped because of opponents KICKOFF\n\n");
					stopWalk();
				}
			}
			#endif


			//update image and other details

			updateWalkFlag();
			//featuredetect()

			//will read mm values only if updated
			pthread_mutex_lock(&mutex_walkstr);
			localmm=walkstr.mm;
			pthread_mutex_unlock(&mutex_walkstr);

			loc.doLocalize(fd,localmm);
			conf = loc.confidence();
			globalflags.LOCALISED=conf;
			ballnotinframe=!(fd.ballFound());
			pthread_mutex_lock(&mutex_walkstr);
			walkstr.mm.updated=0;
			pthread_mutex_unlock(&mutex_walkstr);
			printf("[behaviour] confidence recieved = %lf\n",conf);
			//WEiGHTAGE FUNcTION ----------------------------------------------------------------------------------------------------------------
			updatewt(fd.ballRatio);
			//printf("[behaviour] ballratio=%lf\n",fd.ballRatio);
			if(globalflags.LocalizationState==0)
			{
				if(conf<TH)
				{
					camcont.search(hdmtr);
					printf("[behaviour] Looking for LANDMARKS***** State=0\n\n");
					nolocframecount++;
					if(nolocframecount>200)
					{
						moveAcYut(0,45,0,WALK);
						nolocframecount=0;
					}
				}
				else
					globalflags.LocalizationState=1;
			}
			if(globalflags.LocalizationState==1)
			{
				if(conf<TL)
				{
					globalflags.LocalizationState=0;
				}
				else
				{
					printf("[behaviour] BALL FINDING*****\n\n");
					//ballreturn=camcont.findBall(fd,hm);
					globalflags.p=DONTGENERATE;
					//printf("[behaviour] BWT=%lf  LWT=%lf\n",bwt,lmwt);
					if(bwt>lmwt)
					{
						//printf("[behaviour] called findBall\n\n");
						int ballreturn=camcont.findBall(fd,hdmtr);
						switch(ballreturn)
						{
							case BALLFOUND:
											ballnotinframe=0;
											globalflags.b=FOUND;
											// if(fd.ball.r<BALLDIST_THRESHOLD)
											// {
											// 	globalflags.b=FOUNDNEAR;
											// 	printf("FOUNDNEAR %lf\n",fd.ball.r);
											// }
											// else
											// {
											// 	globalflags.b=FOUNDFAR;
											// 	printf("FOUNDFAR %lf\n",fd.ball.r);
											// }
											break;
							case TURNRIGHT:
											ballnotinframe=1;
											globalflags.b=NOTFOUND;
											//SEND TURN COMMAND
											moveAcYut(0,30,0,WALK);
											break;
							case TURNLEFT:
											ballnotinframe=1;
											globalflags.b=NOTFOUND;
											//SEND TURN COMMAND
											moveAcYut(0,-30,0,WALK);
											break;
							case BALLFINDING:
											ballnotinframe=1;
											globalflags.b=NOTFOUND;
											continue;
							default:continue;

						}
					}
					else
					{
						if(conf<TH)
						{
							camcont.search(hdmtr);
							printf("[behaviour] Looking for LANDMARKS***** State=1\n\n");
						}
					}
				}
			}



			if(globalflags.b==FOUND)
			{
				globalflags.p=GENERATE;
			}


			if(Flag_moving_mainlocal==0)//not moving
			{

				printf("[behaviour] Not Moving\n");
				if(globalflags.p==GENERATE)
				{
					//callpath
					printf("[behaviour] CALLING PATH_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-\n\n");
					convertPathData();
					PathReturns pathreturnval=pathobj.path_return(pathstr);
					switch(pathreturnval)
					{
						case DOWALK:
									printf("[behaviour]-------Path Returns DOWALK--------\n");
									moveAcYut(pathobj.next.r,pathobj.next.theta,0,WALK);//REMEMBER Pathobj.next.theta has the opposite convention  hence the MINUS sign
									//moveAcYut(path.r,path.theta,0,WALK);
									// moveAcYut(fd.ball.r,fd.ball.theta,0,WALK);
									//cvWaitKey();
									break;
						case DOENCIRCLE:
									printf("[behaviour]-------Path Returns DOENCIRCLE--------\n");
									moveAcYut(0,0,pathobj.NextEncircleAngle,ENCIRCLE);
									//cvWaitKey();
									break;
						case DOKICK:
									printf("[behaviour]-------Path Returns DOKICK--------\n");
									cvWaitKey();
									//KICKING
									break;
						case NOPATH:
									printf("[behaviour]-------Path Returns NOPATH--------\n");
									moveAcYut(fd.ball.r,fd.ball.theta,0,WALK);//REMEMBER Pathobj.next.theta has the opposite convention  hence the MINUS sign
									//moveAcYut(path.r,path.theta,0,WALK);
									// moveAcYut(fd.ball.r,fd.ball.theta,0,WALK);
									//cvWaitKey();
									break;

					}
					//printf("Preparing to walk %d\n",pathreturnval);
				}
				else
				{
					//do something

					//continue;
				}
				//get r and theta from path
				//path generated

				//send data to walk
				//#ifdef WIFITRANSMISSION


				usleep(10000);
				#ifdef WIFITRANSMISSION
				convertTransmitData();
				//printf("Called conver wifidata");
				#endif
			}




			//
		}
	}

	//close threads
		void* status;
	#ifdef GAMECONTROLLERACTIVE
	void *status_gc;
	pthread_join (thread_id_gc, &status_gc);
	#endif
	#ifdef SWITCHACTIVE
	void *status_gpio;
	pthread_join (thread_id_gpio, &status_gpio);
	#endif
	#ifdef WIFITRANSMISSION
	void *status_wifi;
	pthread_join (thread_id_wifi, &status_wifi);
	#endif
	pthread_join(thread_id_walkthread,&status);

	return 0;

}
