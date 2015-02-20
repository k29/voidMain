#include "AcYut.h"
#include <iostream>
#include <fstream>
#include <iomanip>
using namespace std;

// float initval[14] = {4096-2048, 4096-1100, 2048, 4096-3000, 2048, 1100, 2048, 3000, 390, 0, 0, 390, 0, 0};

void AcYut::initialize()
{
	left_leg->setSpeed(50);
	right_leg->setSpeed(50);
	left_hand->setSpeed(50);
	right_hand->setSpeed(50);
	comm->syncFlush();
	init_val[0] =	4096-2068;
	init_val[1] =	4096-1100;
	init_val[2] =	4096-2148;
	init_val[3] =	4096-3570;
	init_val[4] =	2048;
	init_val[5] =	1100;
	init_val[6] =	2048;
	init_val[7] =	3470;//1700;
	init_val[8] =	legHeight;
	init_val[9] =	0;
	init_val[10] =	0;
	init_val[11] = 0;
	init_val[12] =	legHeight;
	init_val[13] =	0;
	init_val[14] =	0;
	init_val[15] = 0;

	int arr_l[]={init_val[0], init_val[1], init_val[2],init_val[3]};
	int arr_r[]={init_val[4], init_val[5], init_val[6],init_val[7]};
	
	left_leg->runIK(legHeight,0,20,0);
	left_leg->setGoalPositionSync();
	right_leg->runIK(legHeight,0,20,0);
	right_leg->setGoalPositionSync();
	left_hand->init(arr_l);
	right_hand->init(arr_r);
	comm->syncFlush();
	sleep(4);
	
	left_leg->setSpeed(0);
	right_leg->setSpeed(0);
	left_hand->setSpeed(0);
	right_hand->setSpeed(0);
	comm->syncFlush();
	
	//left_leg->getLoad();
	//right_leg->getLoad();
	
	printf("Initialized Bot\n");
}

int AcYut::reachSlow(double left_x,double left_y,double left_z,double right_x,double right_y,double right_z)
{
	left_leg->setSpeed(50);
	right_leg->setSpeed(50);
	left_hand->setSpeed(50);
	right_hand->setSpeed(50);
	comm->syncFlush();
	
	left_leg->runIK(left_x,left_y,left_z,0);
	left_leg->setGoalPositionSync();
	right_leg->runIK(right_x,right_y,right_z,0);
	right_leg->setGoalPositionSync();

	int arr_l[]={init_val[0], init_val[1], init_val[2],init_val[3]};
	int arr_r[]={init_val[4], init_val[5], init_val[6],init_val[7]};

	left_hand->init(arr_l);
	right_hand->init(arr_r);
	comm->syncFlush();
	
	sleep(3);
	
	left_leg->setSpeed(0);
	right_leg->setSpeed(0);
	left_hand->setSpeed(0);
	right_hand->setSpeed(0);
	comm->syncFlush();
}

AcYut::AcYut(Communication* comm, Imu* imu)
{
	polyPoints=0;
	this->comm = comm;
	this->imu = imu;	

	offsets[0] = 65;
	offsets[1] = 20;
	offsets[2] = -20;
	offsets[3] = -60;
	offsets[4] = 60;
	offsets[5] = -105;
	offsets[6] = 280;

	offsets[20] = 40;//40
	offsets[21] = 0;
	offsets[22] = -0 + 20;
	offsets[23] = -70 - 150;
	offsets[24] = 70;
	offsets[25] = -270;
	offsets[26] = -256;

	//int ids[], int offsets[], int driveMode[], int zeros[]);
	printf("Initializing motors ...\n");
	this->left_leg = new Leg(LEFT,comm,(int*)legMotorIDs[0],&(offsets[0]),(int*)legDriveMode[0],(int*)legMotorZeros[0]);
	this->right_leg = new Leg(RIGHT,comm,(int*)legMotorIDs[1],&(offsets[20]),(int*)legDriveMode[1],(int*)legMotorZeros[1]);
	this->left_hand=new Hand(LEFT,comm);
	this->right_hand=new Hand(RIGHT,comm);
	leg[0] = left_leg;
	leg[1] = right_leg;
	this->pingBot();
	initialize();
	
}

AcYut::~AcYut()
{
	//clearPoS();
	delete left_leg;
	delete right_leg;
	delete left_hand;
	delete right_hand;

}

int AcYut::getFeetCoods()
{
	return getFeetCoods(LEFT) || getFeetCoods(RIGHT);
}

int AcYut::getFeetCoods(int leg)
{
	if(leg!=RIGHT && leg!=LEFT)
	return EXIT_FAILURE;
	
	double feetCoods[FEET_CORNERS][AXES];
	const double (&pointer)[FEET_CORNERS][AXES] = (this->leg)[leg]->getFeetCoods();
	
	for(int i=0;i<FEET_CORNERS;i++)
	{
		feetCoods[i][X] = pointer[i][X];
		feetCoods[i][Y] = pointer[i][Y];
		feetCoods[i][Z]	= (-pointer[i][Z]-hipLength/2)*(leg==LEFT?1:-1);
		// printf("Feet Coods%lf\t%lf\t%lf\n",feetCoods[i][X],feetCoods[i][Y],feetCoods[i][Z]);	
	}
	double feet[AXES];
	for(int i=0;i<FEET_CORNERS;i++)
	{
		getWorldFrameCoods(feetCoods[i],feet);
		for(int j=0;j<AXES;j++)
		{
			this->feetCoods[leg][i][j] = feet[j];
			printf("Feet[%d] %lf\t",j,feet[j]);
		}
		printf("\n");
	}
	return EXIT_SUCCESS;
}

const supportPolygon AcYut::calcSupportPolygon()
{
	
	getFeetCoods();
	double feet[FEET_CORNERS*2][AXES];
	
	for(int i=0;i<2;i++)
		for(int j=0;j<FEET_CORNERS;j++)
			for(int k=0;k<AXES;k++)
			feet[i*4+j][k] = feetCoods[i][j][k];
	
	int maxX=0;
	for(int i=0;i<FEET_CORNERS*2;i++)
		if(feet[i][X] > maxX)
		maxX= feet[i][X];
				
	printf("maxX %5d\n",maxX);
	int length=0;
	Point points[100];
	for(int i=0;i<FEET_CORNERS*2;i++)
	{	if(round(feet[i][X])>=round(maxX-10))
		{
			length++;
			points[i].y = feet[i][Y];
			points[i].x = feet[i][Z];
			//printf("Feet X\t%3.3lf\tY\t%3.3lf\tZ\t%3.3lf\n",feet[i][X],feet[i][Y],feet[i][Z]);
		
		}
		printf("Feet \tX\t%5.3lf\tY\t%5.3lf\tZ\t%5.3lf\n",feet[i][X],feet[i][Y],feet[i][Z]);
		//printf("Feet X\t%3.3lf\n",feet[i][X]);
		//printf("Length %d\n",length);
	}
	
	poly.length = calcConvexHull(points, length);
	
	
	for(int i=0;i<poly.length;i++)
	{
		poly.point[i] = points[i];
		printf("%d\tPolyPoints\tY\t%5.3lf\tZ\t%5.3lf\n",i,poly.point[i].y,poly.point[i].x);
	}

	return poly;
	
}
/*
int AcYut::clearPoS()
{
	posPoint *temp;
	while(supportPolygon !=NULL)
	{
		temp = supportPolygon;
		supportPolygon = supportPolygon->nxtPoint;
		delete temp;
	}
	
	return EXIT_SUCCESS;
	
}
*/
int AcYut::updateBot()
{
	left_leg->setGoalPositionSync();
	right_leg->setGoalPositionSync();
	left_hand->setGoalPositionSync();
	right_hand->setGoalPositionSync();
	comm->syncFlush();
}

int AcYut::pingBot()
{
	left_leg->pingLeg();
	right_leg->pingLeg();
	left_hand->pingHand();
	right_hand->pingHand();   
	return EXIT_SUCCESS;  //TODO return the ping status of each motor
}

int AcYut::writeSensedCurrent(FILE *&fp)
{
	double *rightleg,*leftleg;
	rightleg=right_leg->getSensedCurrentAmps();
	leftleg=left_leg->getSensedCurrentAmps();
	
	char time_buf[21];
	time_t now;
	time(&now);
	strftime(time_buf, 21, "%Y-%m-%dT%H:%S:%MZ", gmtime(&now));
	
	for(int i=0;i<LEG_MOTOR_COUNT;i++)
	{
		fprintf(fp,"\nLeft %d %lf\t",i,leftleg[i]);
	}
	for(int i=0;i<LEG_MOTOR_COUNT;i++)
	{
		fprintf(fp,"\nRight %d %lf\t",i,rightleg[i]);
	}
	fprintf(fp,"\n");
	delete rightleg;
	delete leftleg;
}
void AcYut::getArmTorsoCOM(double armTorsoCOM[AXES])
{
	//All calculations are made on the assumption that the hand is moving only by the first shoulder motor (roll motion)
	const double (&lhandCOM)[AXES] = left_hand->getHandCOM();
	const double (&rhandCOM)[AXES] = right_hand->getHandCOM();

	const double torsoNoArm[] = {-183.8,22,0 };
	const double torsoHeight = 270.5;
	const double shoulderSpan = 198;

	armTorsoCOM[X] = (torsoNoArm[X]*1.743 + (lhandCOM[X]-torsoHeight)*0.609 + (rhandCOM[X]-torsoHeight)*0.609)/(1.743 + 0.609 + 0.609);
	armTorsoCOM[Y] = (torsoNoArm[Y]*1.743 + (lhandCOM[Y])*0.609 + (rhandCOM[Y])*0.609)/(1.743 + 0.609 + 0.609);
	armTorsoCOM[Z] = (torsoNoArm[Z]*1.743 + (-lhandCOM[Z]-shoulderSpan/2)*0.609 + (rhandCOM[Z]+shoulderSpan/2)*0.609)/(1.743 + 0.609 + 0.609);
	// cout<<"X: "<<armTorsoCOM[X]<<" Y: "<<armTorsoCOM[Y]<<" Z: "<<armTorsoCOM[Z]<<endl;

}

//0.616 kg - Hand - 0.087 kg thigharm, 0.04 + 0.05 + 0.013 forearm
const double (&(AcYut::getCOM())) [AXES]
{
	const double (&leftLegCOM)[AXES] = left_leg->getLegCOM();
	const double (&rightLegCOM)[AXES] = right_leg->getLegCOM();
	
	#ifdef DEBUG
	for(int i=0;i<AXES;i++)
	printf("Left COM[%d]\t%4.2lf\n",i,leftLegCOM[i]);
	for(int i=0;i<AXES;i++)
	printf("Right COM[%d]\t%4.2lf\n",i,rightLegCOM[i]);
	#endif
	
	float leftLegMass=2.101, rightLegMass = 2.101;
	float bodyMass = 2.961;
	float torsoCOM[AXES];
	torsoCOM[X] = -151.96 -61.5;
	torsoCOM[Y] = torsoCOM[Z] = 0;
	double armTorsoCOM[AXES];
	getArmTorsoCOM(armTorsoCOM);

	// torsoCOM[Y] = armTorsoCOM[Y];

	COM[X] = leftLegCOM[X]*leftLegMass + rightLegCOM[X]*rightLegMass + torsoCOM[X]*bodyMass; 
	COM[Y] = leftLegCOM[Y]*leftLegMass + rightLegCOM[Y]*rightLegMass + torsoCOM[Y]*bodyMass; 
	COM[Z] = (-leftLegCOM[Z] -hipLength/2)*leftLegMass + (rightLegCOM[Z]+hipLength/2)*rightLegMass + torsoCOM[Z]*bodyMass; 
	
	for(int i=0;i<AXES;i++)
	{
		COM[i]/=(leftLegMass+rightLegMass+bodyMass);
		#ifdef DEBUG
		printf("COM[%d]\t%4.2lf\n",i,COM[i]);
		#endif
	}
	// cout<<COM[1]<<endl;	
	return COM;
}

const double (&(AcYut::getRotCOM()))[AXES]
{
	getCOM();
	getWorldFrameCoods(COM,rotCOM);
	return rotCOM;
}

int AcYut::printCOM()
{
	for(int i=0;i<AXES;i++)
		printf("COM[%d]\t%4.2lf\t",i,COM[i]);
	printf("\n");
	return EXIT_SUCCESS;
}

int AcYut::printRotCOM()
{
	for(int i=0;i<AXES;i++)
		printf("COM[%d]\t%4.2lf\t",i,rotCOM[i]);
	printf("\n");
	return EXIT_SUCCESS;
}

double* AcYut::getWorldFrameCoods(double coods[], double ans[])
{
	double roll = -deg2rad(imu->roll);
	double pitch = -deg2rad(imu->pitch);
	double floorcoods_x = 100.0;
	double floorcoods_z = 0;
	// cout<<"IMU pitch = "<<-(imu->pitch)<<endl;
	ans[X] = cos(roll)*cos(pitch)*floorcoods_x + sin(roll)*coods[Y] - cos(roll)*sin(pitch)*coods[Z];
	ans[Y] = -sin(roll)*cos(pitch)*floorcoods_x + cos(roll)*coods[Y] + sin(roll)*sin(pitch)*coods[Z];
	ans[Z] = sin(pitch)*floorcoods_x + cos(pitch)*floorcoods_z;

/*	ans[X] = cos(roll)*cos(pitch)*coods[X] + sin(roll)*coods[Y] - cos(roll)*sin(pitch)*coods[Z];
	ans[Y] = -sin(roll)*cos(pitch)*coods[X] + cos(roll)*coods[Y] + sin(roll)*sin(pitch)*coods[Z];
	ans[Z] = sin(pitch)*coods[X] + cos(pitch)*coods[Z];*/
	// #ifdef DEBUG
	// printf("X\t%3.3lf\tY\t%3.3lf\tZ\t%3.3lf\n",coods[X],coods[Y],coods[Z]);
	// printf("ROT X\t%3.3lf\tY\t%3.3lf\tZ\t%3.3lf\n",ans[X],ans[Y],ans[Z]);
	// printf("IMU ROLL = %f PITCH = %f YAW = %f\n", imu->roll,imu->pitch,imu->yaw);
	// #endif
	
	return ans;
}

int AcYut::storeCOM2(int n)
{
	float i,j,k;
	// getRotCOM();
	ofstream filout;
	filout.open("X1.dat",ios::out|ios::app);
	filout<<n;
	filout<<"\t"<<rotCOM[0]<<endl;
	//cout<<rotCOM[0]<<"   ";
	filout.close();
	filout.open("Y1.dat",ios::out|ios::app);
	filout<<n;
	filout<<"\t"<<rotCOM[1]<<endl;
	//cout<<rotCOM[1]<<"   ";
	filout.close();
	filout.open("Z1.dat",ios::out|ios::app);
	filout<<n;
	filout<<"\t"<<rotCOM[2]<<endl;
	//cout<<rotCOM[2]<<"   ";
	filout.close();
	cout<<endl; 
	return(0);
}

int AcYut::storevalues(int n)
{
	std::cout<<n<<endl;
	left_leg->getLoad();
	right_leg->getLoad();
	
	return(0);
}

double AcYut::getImuYaw()
{
	return imu->yaw;
}