#ifndef FEATUREDETECTION_H
#define FEATUREDETECTION_H

#include "camcapture.h"
#include "defines.h"
#include "headmotor.h"
#include "imgproc.h"
#include "localizeinternals.h"
#include "commondefs.h"
#include <math.h>
#include <cvblob.h>
#include "tbb/parallel_for.h"
#include "tbb/blocked_range2d.h"

/*
HOW TO USE THIS CLASS:
** Note that constructor requires camCapture object, but it need not be initialized **
Call getLandmarks() for *each* image to get current list of landmarks as well as ball.
	-> Current list means not just current frame, but last few frames (read implementation below)
List of landmarks includes all identifiable field landmarks, which are stored in vector l.
ballX(), ballY() and ballFound() return ball pixel values and boolean of whether ball is in current image or not.
obstacles[] should store list of obstacle values (size should be in n_obstacles)
ball stores real distance and angle of ball
*/
//new technique to get features:
//0. for each feature in main array, increase framecounter. if framecounter above a threshold, delete it.
//1. get features in current image. store in a temporary array
//2. for each new feature, check main array
//3. if same feature with similar location exists, refresh its values and set its framecounter to 0
//4. if feature does not exist, add it to main vector and set its counter to 0

class FeatureDetection
{
private:
	const int IMAGE_HEIGHT;
	const int IMAGE_WIDTH;
	static const float pix2cm = 0.28;	//WARNING: need to change this whenever changing size of image
	static const float s_height=105.0;//93.0
	static const float neck_len=7.0; //constanty=4.5,rangey=10.0,constantx=8.0,rangex=16.0,rminx=70.0;
	static const int LANDMARK_PERSISTENCE = 20;
	static const double forwardTiltCorrection = -5.3;	//correction in degrees
	cvb::CvBlobs blobs_red;
	cvb::CvBlobs blobs_yellow;
	cvb::CvBlobs blobs_black;
	IplImage* seg_red;
	IplImage* seg_yellow;
	IplImage* labelImg;
	IplImage* labelImg_small;
	IplImage* seg_white;
	IplImage* seg_black;
	IplImage* seg_green;
	IplImage* seg_white_count;
	void findReal(int x,int y, float &objdis, float &objangdeg, HeadMotor &hm);
	void getGoals(HeadMotor &hm);
	void getBlobs(CamCapture &cam);
	void getCorners(CamCapture &cam, HeadMotor &hm);
	void getBall(CamCapture &cam, HeadMotor &hm);
	bool checkSkeleton(int x, int y, int i, int j);
	bool ballFound_var;
	int ballX_var;
	int ballY_var;
	inline double getDistance(CvPoint2D64f a, CvPoint2D64f b)
	{
		return sqrt((a.x-b.x)*(a.x-b.x) + (a.y-b.y)*(a.y-b.y));
	}
	inline bool isOnImageEdge(int x, int y)
	{
		//We only need to check if too far left, right or up. Too far down is fine
		if(x > IMAGE_WIDTH - 20 || x < 20 || y < 15)
			return true;
		else
			return false;
	}
	inline bool isSameLandmark(LOCALIZE_INTERNALS::Landmark &l1, LOCALIZE_INTERNALS::Landmark &l2)
	{
		if(l1.type!=l2.type)
			return false;
		if(fabs(l1.distance-l2.distance) < 20.0 && fabs(l1.angle-l2.angle) < 10.0)
			return true;
		else
			return false;
	}
	void getInGreen(CamCapture &cam);
	void getInGreenKartik(CamCapture &cam);

public:
	LOCALIZE_INTERNALS::Landmark templ[20]; //Max landmarks you might detect in an image
	int tempnLand;
	std::vector<LOCALIZE_INTERNALS::Landmark> l;
	// int nLand;
	Coords ball;
	FeatureDetection(CamCapture &cam);
	void getLandmarks(CamCapture &cam, HeadMotor &hm);
	inline bool ballFound(){return ballFound_var;};
	inline int ballX(){return ballX_var;};
	inline int ballY(){return ballY_var;};
	void updatePacket(FeaturesPacket &fp);
	double ballRatio;
	Coords obstacles[10];	//Max obstacles you might detect in an image
	int n_obstacles;
};


#endif
