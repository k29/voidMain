#ifndef HANDH
#define HANDH
#include "motor.h"
#include "communication.h"
#include "commondefswalk.h"

#include <stdio.h>
#include <math.h>
#define HAND_MOTORS_COUNT 4

class Hand
{
private:
	Communication *comm;
	Motor* m[HAND_MOTORS_COUNT];
	double COM[AXES];
	int hand;
public:
	Hand(int hand, Communication* comm);
	const double (&getHandCOM(Motor* mot[])) [3];
	const double (&getHandCOM()) [3];
	void setIDList(int* idList, int n);
	int setSpeed(int speed);
	int getLoad();
	int* pingHand();
	int getMotorLoad(int motorID);
	void init(int* pos);
	void setGoalPositionSoft(int* pos);
	void setGoalPositionSync(int* pos);
	int setGoalPositionSync();
	void getGoalPositionSoft(int* pos);
};

#endif
