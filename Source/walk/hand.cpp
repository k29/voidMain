#include "hand.h"

Hand::Hand(int hand, Communication* comm)
{
	this->comm = comm;
	for (int i = 0; i < HAND_MOTORS_COUNT; ++i)
	{
		if (hand == LEFT)
		{
			if (i==2||i==3)
				m[i] = new Motor(MX106, hand*20+7+i, this->comm);
			if (i==1||i==0)
				m[i] = new Motor(MX64, hand*20+7+i, this->comm);
		}
		if (hand == RIGHT)
		{
			if (i==2)
				m[i] = new Motor(MX106, hand*20+7+i, this->comm);
			if (i==1||i==0||i==3)
				m[i] = new Motor(MX64, hand*20+7+i, this->comm);
		}

	}
	this->hand = hand;
}

void Hand::setIDList(int* idList, int n)
{
	for (int i = 0; i < n && i < HAND_MOTORS_COUNT; ++i)
	{
		m[i]->setIDSoft(idList[i]);
	}
}

int Hand::setSpeed(int speed)
{
	for(int i=0; i< HAND_MOTORS_COUNT; i++)
	{
		m[i]->setMovingSpeedSync(speed);
	}

}

int Hand::getLoad()
{
	for (int i = 0; i < HAND_MOTORS_COUNT; ++i)
	{
		m[i]->getPresentLoad();
	}
}

int* Hand::pingHand()
{
	for (int i = 0; i < HAND_MOTORS_COUNT; ++i)
	{
		m[i]->ping();
	}
}

int Hand::getMotorLoad(int motor)
{
	m[motor]->getPresentLoad();
}

void Hand::init()
{
	double pos[HAND_MOTORS_COUNT];
	if(hand==LEFT)
	{
		pos[0]=2048;
		pos[1]=4096 - 1100;
		pos[2]=2048;
		pos[3]=4096 - 3000;
	}
	else
	{
		pos[0]=2048;
		pos[1]=1100;
		pos[2]=2048;
		pos[3]=3000;
	}
	
	

	for (int i = 0; i < HAND_MOTORS_COUNT; ++i)
	{
		m[i]->setGoalPositionSync(pos[i]);
	}
}

void Hand::setGoalPositionSync(int* pos)
{

	// pos[0]=2048;
	// 	pos[1]=4096-3071;
	// 	pos[2]=4096-1792;
	// 	pos[3]=2048;
	for (int i = 0; i < HAND_MOTORS_COUNT; ++i)
	{
		m[i]->setGoalPositionSync(pos[i]);
	}
}
