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
	// for (int i = 0; i < HAND_MOTORS_COUNT; ++i)
	// {
	// 	m[i]->ping();
	// }
}

int Hand::getMotorLoad(int motor)
{
	m[motor]->getPresentLoad();
}

void Hand::init(int* pos)
{
	// int pos[HAND_MOTORS_COUNT];
/*	if(hand==LEFT)
	{
		pos[0]=//4096-2068;
		pos[1]=4096-1100;
		pos[2]=4096-2148;
		pos[3]=4096-3570;//4096 - 1800;
	}
	else
	{
		pos[0]=2048;
		pos[1]=1100;
		pos[2]=2048;
		pos[3]=3470;//1700;
	}*/
	
	setGoalPositionSoft(pos);	
	setGoalPositionSync(pos);
}


int Hand::setGoalPositionSync()
{
	for(int i=0;i<HAND_MOTOR_COUNT;i++)
	{
		if(m[i]->getGoalPositionSync() == false && m[i]->isSlave() == false)
		{
			if(m[i]->setGoalPositionSync()==EXIT_FAILURE)
			return EXIT_FAILURE;			
		}
	}
	
	return EXIT_SUCCESS;
}

void Hand::setGoalPositionSoft(int* pos)
{
	for (int i = 0; i < HAND_MOTORS_COUNT; ++i)
	{
		m[i]->setGoalPositionSoft(pos[i]);
	}
}
void Hand::setGoalPositionSync(int* pos)
{

	for (int i = 0; i < HAND_MOTORS_COUNT; ++i)
	{
		m[i]->setGoalPositionSync(pos[i]);
	}
}

void Hand::getGoalPositionSoft(int* arr)
{
	for (int i = 0; i < HAND_MOTORS_COUNT; ++i)
	{
		arr[i] = this->m[i]->getGoalPositionSoft();
	}

}

//x = -139.524 (downward negative) z = 49.183 y = 36.017
//1 hand = 0.609 kg
//x = 183.8 mm (upward positive), ,z = 0, y = 22
const double (&Hand::getHandCOM(Motor* mot[])) [AXES]
{
	//Relative to horn of shoulder motor 7/27
	//All calculations are made on the assumption that the hand is moving only by the first shoulder motor (roll motion)
	double def_y_handCOM = 36.017;
	double def_x_handCOM = 139.524; //Downward positive
	double def_z_handCOM = 49.183;
	double r = sqrt(pow(def_y_handCOM,2) + pow(def_x_handCOM,2));
	double base_theta = atan((def_y_handCOM/def_x_handCOM))*180/acos(-1);
	// printf("%f\n",((double)(mot[0]->getGoalPositionSoft() )/(double)mot[0]->getMaxPos() * 360));
	double delta_theta = (hand==LEFT?1:-1)*((double)(mot[0]->getGoalPositionSoft() - mot[0]->getZeroPos())/(double)mot[0]->getMaxPos() * 360);
	double current_theta = base_theta + delta_theta;

	// printf("GoalPositionSoft = %d\n",mot[0]->getGoalPositionSoft());
	COM[X] = r*cos(deg2rad(current_theta));
	COM[Y] = r*sin(deg2rad(current_theta));
	COM[Z] = def_z_handCOM;

	// printf("r = %f base_theta = %f delta_theta = %f current_theta = %f Cx = %f Cy = %f Cz = %f \n",r, base_theta, delta_theta, current_theta, COM[X], COM[Y], COM[Z]);

	return COM;
}
const double (&Hand::getHandCOM()) [AXES]
{
	return getHandCOM(this->m);
}