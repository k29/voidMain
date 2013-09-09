#include <opencv2/opencv.hpp>
#include "defines.h"
#include "camcapture.h"
#include "headmotor.h"
#include "commondefs.h"
#include <math.h>
#include "localize.h"
#include "imgproc.h"
#include <time.h>
#include <stdlib.h>
#include "featuredetection.h"

using namespace LOCALIZE_INTERNALS;

//Feature detection functions are in featuredetection.h


Localize::Localize()
{
	//Initializing. Choose particles randomly, each particle gets same probability
	int i;
	srand(time(NULL));
	for(i=0; i<NO_OF_PARTICLES; i++)
	{
		p[i].x = rand()%(MAX_X);	//Assigns a random number between 0 and MAX_X INCLUSIVE
		p[i].y = rand()%(MAX_Y);	//Assigns a random number between 0 and MAX_Y INCLUSIVE
		p[i].angle = ((double)((rand()%(360))))*PI/180.0;	//Assigns a random angle with given resolution
	}
	nFrames = 0;
	dispImage = cvCreateImage(cvSize(MAX_X, MAX_Y), 8, 1);
	createLandmarkList();
}



//Origin is corner side of yellow goal along BYB line
//NOTE: Coordinates are negative and beyond range also! (because of BYB and YBY)
void Localize::createLandmarkList()
{

	landT[0].x=MAX_X/2;
	landT[0].y = MAX_Y;
	landT[1].x=MAX_X/2;
	landT[1].y = 0;
	
	landX[0].x = MAX_X/2;
	landX[0].y = MAX_Y/2 - MAX_Y/8;
	landX[1].x = MAX_X/2;
	landX[1].y = MAX_Y/2 + MAX_Y/8;
	
	landGPY[1].x=0;
	landGPY[1].y=MAX_Y/4;
	landGPY[0].x=0;
	landGPY[0].y=MAX_Y*3/4;

	landGPB[0].x=MAX_X;
	landGPB[0].y=MAX_Y/4;
	landGPB[1].x=MAX_X;
	landGPB[1].y=MAX_Y*3/4;

	landYBY.x=MAX_X/2;
	landYBY.y=MAX_Y + MAX_Y/15;

	landBYB.x=MAX_X/2;
	landBYB.y=MAX_X + MAX_Y/15;
	
}



void Localize::printPosition()
{
	int tempx, tempy;
	cvZero(dispImage);
	for (int i = 0; i < NO_OF_PARTICLES; ++i)
	{
		tempx = p[i].x;
		tempy = p[i].y;
		if(tempx <0)
			tempx = 0;
		if(tempy<0)
			tempy = 0;
		if(tempx >= MAX_X)
			tempx = MAX_X - 1;
		if(tempy >= MAX_Y)
			tempy = MAX_Y - 1;
		returnPixel1C(dispImage, tempx, MAX_Y - tempy) = 255;
	}
	printf("X: %lf Y:%lf Angle:%lf\n", selfX, selfY, selfAngle);

	return;
}



void Localize::walkersalias(
int n,   // number of classes
double *p, // relative weights of each class
int nans,  // sample size to return
int *ans  // sample as an array of class indices
){
    int *a = calloc(n,sizeof(int));
	double *q, rU;
	int i,j,k;
	int *HL,*H,*L;
	HL = calloc(n,sizeof(int));
	q = calloc(n,sizeof(double));
	H = HL - 1; L = HL + n;
	double sum = 0;
	for(i = 0; i < n; i++)
	{
		sum += p[i];
	}
//	printf("Sum=%lf\n", sum);
	for(i = 0; i < n; i++)
	{
		p[i] /= sum;
	}
	for(i = 0; i < n; i++)
	{
		q[i] = p[i] * n;
		if(q[i] < 1.) *++H = i; else *--L = i;
	}
	if(H >= HL && L < HL +n)
	{
		for(k = 0; k < n-1; k++)
		{
			i = HL[k];
			j = *L;
			a[i] = j;
			q[j] += q[i] - 1;
			if(q[j] < 1.) L++;
			if(L >= HL + n) break;
		}
	}
	for(i = 0; i < n; i++) q[i] += i;
	for(i = 0; i < nans; i++)
	{
		rU = (double) rand() / RAND_MAX * n;
		k = (int) rU;
		ans[i] = (rU < q[k]) ? k : a[k];
	}
	free(HL);
	free(q);
        free(a);
}



//TODO: when value of particle goes beyond min max limits due to randomness, assign that particle
//a random location (currently something weird is happening)
void Localize::doLocalize(FeatureDetection &fd, MotionModel &mm)
{
	

	if(mm.updated==1)
	{
		for(int k=0; k<NO_OF_PARTICLES; k++)
		{
			// if((rand()%(200)==0))
			// {
			// 	p[k].x = rand()%(MAX_X);	//Assigns a random number between 0 and MAX_X INCLUSIVE
			// 	p[k].y = rand()%(MAX_Y);	//Assigns a random number between 0 and MAX_Y INCLUSIVE
			// 	p[k].angle = ((double)((rand()%(360))))*PI/180.0;	//Assigns a random angle with given resolution
			// 	continue;
			// }
			p[k].angle = p[k].angle + ((float)(rand()%10 - 5 - mm.theta))*PI/180.0;
			while(p[k].angle < 0)
				p[k].angle += 2*PI;

			double addx = (mm.r*cos(p[k].angle))/10.0;
			double addy = (mm.r*sin(p[k].angle))/10.0;
			if(k==0)
			{
				printf("[localize] addx %lf, addy %lf\n", addx, addy);
			}
			p[k].x = p[k].x + addx + rand()%10 - 5;
			if(p[k].x >= MAX_X)
				p[k].x = p[k].x - MAX_X - 1;
			if(p[k].x < 0)
				p[k].x = MAX_X + p[k].x;

			p[k].y = p[k].y + addy + rand()%10 - 5;
			if(p[k].y >= MAX_Y)
				p[k].y = p[k].y - MAX_Y - 1;
			if(p[k].y < 0)
				p[k].y = MAX_Y + p[k].y;
		}
		updatePosition();
	}
	else
	{
		for(int k=0; k<NO_OF_PARTICLES; k++)
		{
			//Adding a bit of randomness, plus replacing 2% of total points by completely random points
			// if((rand()%(2000)==0))
			// {
			// 	p[k].x = rand()%(MAX_X);	//Assigns a random number between 0 and MAX_X INCLUSIVE
			// 	p[k].y = rand()%(MAX_Y);	//Assigns a random number between 0 and MAX_Y INCLUSIVE
			// 	p[k].angle = ((double)((rand()%(360))))*PI/180.0;	//Assigns a random angle with given resolution
			// 	continue;
			// }
		}
	}

	if(fd.l.size()<=1)
		return;
	//printf("NO OF LANDMARKS = %d !!!!!!!!!!!!!!!!!", fd.l.size());
	nFrames++;
	//If no motion model but at least 2 landmark spotted, randomize a bit
	if(mm.updated==0)
	{
		for(int k = 0; k < NO_OF_PARTICLES; k++)
		{
			p[k].angle = p[k].angle + ((float)(rand()%10 - 5))*PI/180.0;
			while(p[k].angle < 0)
				p[k].angle += 2*PI;
			p[k].x = p[k].x + rand()%10 - 5;
			if(p[k].x >= MAX_X)
				p[k].x = p[k].x - MAX_X - 1;
			if(p[k].x < 0)
				p[k].x = MAX_X + p[k].x;
			p[k].y = p[k].y + rand()%10 - 5;
			if(p[k].y >= MAX_Y)
				p[k].y = p[k].y - MAX_Y - 1;
			if(p[k].y < 0)
				p[k].y = MAX_Y + p[k].y;
		}
	}

	for(int i = 0; i < NO_OF_PARTICLES; i++)
		w[i] = 1.0;

	//For each landmark
	for(int i=0; i<fd.l.size(); i++)
	{
		int temp_x1, temp_y1;
		int temp_x2, temp_y2;
		int temp_x3, temp_y3;
		int temp_x4, temp_y4;
		int no_of_instance;
		double prob1, prob2, prob3, prob4;
		//For each particle, multiply weight according to prob of landmark
		switch(fd.l[i].type)
		{
			case LAND_GP:
			no_of_instance=4;
			temp_x1=landGPY[0].x;
			temp_y1=landGPY[0].y;
			temp_x2=landGPY[1].x;
			temp_y2=landGPY[1].y;	
			temp_x3=landGPB[0].x;
			temp_y3=landGPB[0].y;
			temp_x4=landGPB[1].x;
			temp_y4=landGPB[1].y;
			break;
			case LAND_X:
			no_of_instance = 2;
			temp_x1=landX[0].x;
			temp_y1=landX[0].y;
			temp_x2=landX[1].x;
			temp_y2=landX[1].y;
			break;
		}

		for(int j=0; j<NO_OF_PARTICLES; j++)
		{
			if(no_of_instance==1)
			{
				double temp_angle = rad2deg(atan2(temp_y1-p[j].y,temp_x1-p[j].x)-p[j].angle);
				temp_angle += fd.l[i].angle;	//+ because landmark angle conventions are bad
				while(temp_angle < 0)
					temp_angle += 360.0;
				if(temp_angle > 180.0)
					temp_angle = 360.0 - temp_angle;
				w[j]*=exp(-pow((sqrt(pow((temp_x1-p[j].x),2)+pow((temp_y1-p[j].y),2))-fd.l[i].distance),2)/((U_DIST + fd.l[i].distance*U_DIST_P)*(U_DIST + fd.l[i].distance*U_DIST_P))-pow((temp_angle),2)/(U_ANGLE*U_ANGLE));
			//	printf("%lf\n", pow((sqrt(pow((temp_x1-p[j].x),2)+pow((temp_y1-p[j].y),2))-fd.l[i].distance),2)-pow((rad2deg(atan2(temp_y1-p[j].y,temp_x1-p[j].x)-p[j].angle)- fd.l[i].angle),2));
			}
			else if(no_of_instance==2)
			{
				double temp_angle1 = rad2deg(atan2(temp_y1-p[j].y,temp_x1-p[j].x)-p[j].angle);
				temp_angle1 += fd.l[i].angle;
				while(temp_angle1 < 0)
					temp_angle1 += 360.0;
				if(temp_angle1 > 180.0)
					temp_angle1 = 360.0 - temp_angle1;
				double temp_angle2 = rad2deg(atan2(temp_y2-p[j].y,temp_x2-p[j].x)-p[j].angle);
				temp_angle2 += fd.l[i].angle;
				while(temp_angle2 < 0)
					temp_angle2 += 360.0;
				if(temp_angle2 > 180.0)
					temp_angle2 = 360.0 - temp_angle2;
				prob1=exp(-pow((sqrt(pow((temp_x1-p[j].x),2)+pow((temp_y1-p[j].y),2))-fd.l[i].distance),2)/((U_DIST + fd.l[i].distance*U_DIST_P)*(U_DIST + fd.l[i].distance*U_DIST_P))-pow((temp_angle1),2)/(U_ANGLE*U_ANGLE));
				prob2=exp(-pow((sqrt(pow((temp_x2-p[j].x),2)+pow((temp_y2-p[j].y),2))-fd.l[i].distance),2)/((U_DIST + fd.l[i].distance*U_DIST_P)*(U_DIST + fd.l[i].distance*U_DIST_P))-pow((temp_angle2),2)/(U_ANGLE*U_ANGLE));

				if(prob1>prob2)
					w[j]*=prob1;
				else
					w[j]*=prob2;
			}
			else if(no_of_instance==4)
			{
				double temp_angle1 = rad2deg(atan2(temp_y1-p[j].y,temp_x1-p[j].x)-p[j].angle);
				temp_angle1 += fd.l[i].angle;
				while(temp_angle1 < 0)
					temp_angle1 += 360.0;
				if(temp_angle1 > 180.0)
					temp_angle1 = 360.0 - temp_angle1;
				double temp_angle2 = rad2deg(atan2(temp_y2-p[j].y,temp_x2-p[j].x)-p[j].an