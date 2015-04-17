#include "goalkeeper.h"

GoalKeeper::GoalKeeper()
{
	printf("Goal Keeper created.\n");
}

GoalKeeperAction GoalKeeper::keeperUpdate(CamCapture &cam, HeadMotor &hm, CamControl* camcont, FeatureDetection* fd, MotionModel &mm)
{
	cam.getImage();
	fd->getLandmarks(cam, hm, mm);
	if(camcont->findBall(*fd,hm) != BALLFOUND)
		return STAY;
	double x1 = fd->ball.r*cos(deg2rad(fd->ball.theta));
	double y1 = fd->ball.r*sin(deg2rad(fd->ball.theta));
	printf("Ball Position >>>>>>>>>>>>>>>> x: %lf \t y: %lf\n", x1, y1);
	if(fd->ball.r > ACTIVATION_RADIUS)
		return STAY;
	if(fd->ball.r < CRITICAL_RADIUS)
	{
		printf("Reached CRITICAL_RADIUS\n");
		if(fd->ball.theta > 0)
			return FALLRIGHT;
		return FALLLEFT;
	}
	cvWaitKey(50);
	cam.getImage();
	fd->getLandmarks(cam, hm, mm);
	if(fd->ball.r > ACTIVATION_RADIUS)
		return STAY;
	double x2 = fd->ball.r*cos(deg2rad(fd->ball.theta));
	double y2 = fd->ball.r*sin(deg2rad(fd->ball.theta));
	printf("difference: %lf\n", x2 - x1);
	if(x2 < x1)
		return STAY;
	if(fabs(x2 - x1) < ACTIVATION_THRESHOLD)
		return STAY;
	double alpha = (x2*(y1 - y2))/(x1 - x2);
	double estGoalImpact = y2 - alpha;
	printf("Estimated Point of Impact: %lf\n", estGoalImpact);
	if(fabs(estGoalImpact) > GOAL_WIDTH/2.0)
		return STAY;
	if(estGoalImpact < 0)
		return FALLLEFT;
	return FALLRIGHT;
}