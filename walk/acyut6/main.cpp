#include <iostream>
#include <stdio.h>
#include <strings.h>
#include "AcYut.h"

using namespace std;

int main()
{
	Communication comm;
	AcYut bot(&comm);

	//Leg l1(LEFT,&comm), l2(RIGHT,&comm);
	//int idList1[] = {0,1,2,3,4,5,6};
	//int idList2[] = {20,21,22,23,24,25,26};
	
	//l1.setIDList(idList1,7);
	//l2.setIDList(idList2,7);
//	l1.setSpeed(50);
//	l2.setSpeed(50);
//	comm.syncFlush();	
//	l1.move(250,0,0,0);
//	l2.move(250,0,0,0);	
//	comm.syncFlush();
	
	// Motor m1(MX106, 18, &comm);
	// Motor m2(MX106, 17, &comm);
	// m1.ping();
	// m2.ping();
	// return 0;
	// // m1.setGoalPositionSync(100);
	// // m2.setGoalPositionSync(200);
	// // printf("Waiting 1 second.\n");
	// // sleep(1);
	// // comm.syncWrite();

	// int pos1 = m1.getPresentPosition();
	// int pos2 = m2.getPresentPosition();
	// m1.setMovingSpeed(0);
	// sleep(2);
	// m1.setGoalPositionSync(pos1);
	// m2.setGoalPositionSync(pos2);

	// printf("Waiting 1 second.\n");
	// sleep(1);
	// comm.syncFlush();

	// sleep(1);
	// m1.torqueDisable();
	// sleep(1);
	// m2.torqueDisable();
}
