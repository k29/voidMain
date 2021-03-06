#include "WorldState.h"

playerstate* playerstate::theInstance =NULL;

playerstate::playerstate()
	{
	/* XABSL */	
	theInstance=this;
	

	/* Image */
	fd= new FeatureDetection(capture);
	camcont= new CamControl(capture);
	timesteps = 0;
	GOAL_KEEPER_FLAG = false;

	}

double playerstate::getTheta()
	{
		return (double)theInstance->fd->ball.theta;
	}


double playerstate::getConfidence()
	{
		return (double)theInstance->confidence;
	}

double playerstate::getReset()
	{
		return (double)theInstance->resetflag;
	}
double playerstate::getRoboCupState()
	{
		return (double)theInstance->GCData.state;
	}
double playerstate::getDistance()
	{
		return (double)theInstance->fd->ball.r;
	}

double playerstate::getGoalKeepFlag()
	{
		return (double)theInstance->GOAL_KEEPER_FLAG;
	}