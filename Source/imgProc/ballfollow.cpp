// NOTE: If program crashes on start, it can be because camcapture object was made earlier with camcapture_debug off.
// So just rm camcapture.o or make clean and make again

// #include <opencv2/opencv.hpp>
// //#include <FlyCapture2.h>
// #include "camcapture.h"
// #include "headmotor.h"
// #include "camcontrol.h"
// #include "imgproc.h"
// #include <time.h>
// #include <cvblob.h>

// int main()
// {
// 	CamCapture cam(true);
// 	HeadMotor hm(false);
// 	CamControl cc(cam);
// 	GoalReturns gr;
// 	hm.bootup_files();
// 	int fps = 0;
// 	time_t seconds1, seconds2;
// 	if(cam.init()==CAM_FAILURE)
// 	{
// 		printf("\n Exiting after init error\n");
// 		return 0;
// 	}
// 	seconds1 = seconds2 = time (NULL);
// 	cc.setGoal(1);
// 	while(1)
// 	{
// 		cam.getImage();
// 		// hm.update();
// 		// gr = cc.findGoal(cam, hm);
// 		cc.check_foundGoal(cam);
// 		cvShowImage("Image", cam.rgbimg);
// #ifdef CAMCAPTURE_DEBUG
// 		cvShowImage("Seg", cam.showSeg);
// #endif
// 		 cvWaitKey(10);
		 
		 
		 
// 		 fps++;
// 		 seconds2 = time(NULL);
// 		 if(seconds1!=seconds2)
// 		 {
// 		 	seconds1 = seconds2;
// 		 	printf("%d frames\n", fps);
// 		 	fps = 0;
// 		 }
// 	}
// 	return 0;
// }


// #include <opencv2/opencv.hpp>
// //#include <FlyCapture2.h>
// #include "camcapture.h"
// #include "headmotor.h"
// #include "camcontrol.h"
// #include "localize.h"
// #include "imgproc.h"
// #include "featuredetection.h"
// #include <time.h>

// int main()
// {
// 	CamCapture cam(true, 100, 50);
// 	// Localize l;
// 	FeatureDetection fd(cam);
// 	HeadMotor hm(true);	//means initialize motors also
// 	// MotionModel mm;
// 	CamControl cc(cam);
// 	// mm.updated = 0;
// 	hm.bootup_files();
// 	int fps = 0;
// 	time_t seconds1, seconds2;
// 	if(cam.init()==CAM_FAILURE)
// 	{
// 		printf("\n Exiting after init error\n");
// 		return 0;
// 	}
// 	seconds1 = seconds2 = time (NULL);
// 	double prevr;
// 	prevr = 0;
// 	FILE *fp;
// 	fp=fopen("goalkeeper.txt","w+");
// 	while(1)
// 	{
// 		// cc.search(hm);
// 		// l.printPosition();
// 		cam.getImage();
// 		hm.update();
// 		fd.getLandmarks(cam, hm);

		

// 		BallReturns b;
// 		b=cc.findBall(fd, hm);

// 		if(b==BALLFOUND)
// 		{
// 			fprintf(fp,"\nDifference is : %f \nCurrent R=%f \n", prevr - fd.ball.r*cos(deg2rad(fd.ball.theta)),fd.ball.r*cos(deg2rad(fd.ball.theta)));
// 			prevr = fd.ball.r*cos(deg2rad(fd.ball.theta));
// 		}
// 		else
// 		{
// 			fprintf(fp,".");
// 			printf("~~~ Ball Not Found ~~~\n");
// 		}
// 		// l.doLocalize(fd, mm);
// 		// l.confidence();
// 		// cvShowImage("Localization", l.dispImage);

// 		 cvShowImage("Main", cam.rgbimg);
// #ifdef CAMCAPTURE_DEBUG
// 		 cvShowImage("Seg", cam.showSeg);
// #endif
// 		 cvWaitKey(5);
// 		 fflush(fp);
// 		 usleep(5000);
// 		 fps++;
// 		 seconds2 = time(NULL);
// 		 if(seconds1!=seconds2)
// 		 {
// 		 	seconds1 = seconds2;
// 		 	printf("%d frames\n", fps);
// 		 	fps = 0;
// 		 }
// 	}
// 	return 0;
// }

#include <opencv2/opencv.hpp>
//#include <FlyCapture2.h>
#include "camcapture.h"
#include "headmotor.h"
#include "camcontrol.h"
#include "localize.h"
#include "imgproc.h"
#include "featuredetection.h"
#include <time.h>
//#include "xsens/imu.h"
// #define CAMCAPTURE_DEBUG
using namespace std;

int main()
{
	CamCapture cam(true, 100, 50);
	Localize l;
	FeatureDetection fd(cam);
	// Imu imu;
	// imu.init();
	HeadMotor hm(true);	//means initialize motors also
	MotionModel mm;
	CamControl cc(cam);
	mm.updated = 0;
	hm.bootup_files();
	int fps = 0;
	time_t seconds1, seconds2;
	if(cam.init()==CAM_FAILURE)
	{
		printf("\n Exiting after init error\n");
		// return 0;
	}
	seconds1 = seconds2 = time (NULL);
	int imageCounter = 0;
	while(1)
	{
		// cc.findBall(fd, hm);
		// printf("~~~~~~~~~~~~~~~~~Angle:%lf\n", imu.yaw +180);
		 cvWaitKey(20);
		// continue;
		// l.printPosition();
		hm.update();
		//cout<<"update"<<endl;
		cam.getImage();
		//cout<<"getimg"<<endl;
		// continue;

		fd.getLandmarks(cam, hm, mm);
		//cout<<"landmark"<<endl;
		cc.findBall(fd, hm);
		//cc.search(hm);
		//cout<<"findball"<<endl;
		// // printf("Calling FD\n");
		
		//l.doLocalize(fd, mm, -9999);
		//l.confidence();
		//l.printPosition();
		//cvShowImage("Localization", l.dispImage);
		cvShowImage("Main", cam.rgbimg);
#ifdef CAMCAPTURE_DEBUG
		 cvShowImage("Seg", cam.showSeg);
#endif
		 if((cvWaitKey(5)&0xff)=='p')
		 {
		 	char temp[50];
		 	sprintf(temp, "image_field_%d.jpg",imageCounter);
		 	imageCounter++;
		 	cvSaveImage(temp, l.dispImage);
		 }
		 fps++;
		 seconds2 = time(NULL);
		 if(seconds1!=seconds2)
		 {
		 	seconds1 = seconds2;
		 	printf("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%d frames\n", fps);
		 	fps = 0;
		 }
	}
	return 0;
}
