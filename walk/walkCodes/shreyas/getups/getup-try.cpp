#include "AcYut.h"
#include "communication.h"
#include "xsens/imu.h"
#include <fstream>

using namespace std;

int main()
{
	Imu imu;
	imu.init();
	
	Communication comm;
	AcYut bot(&comm,NULL);

	int ch=1;
	
	// while(ch!=4)
	// {
	// 	printf("\n1. Torque OFF\n2. Torque ON \n3. Read MOTOR values\n 4.EXIT\n");
	// 	scanf("%d",&ch);
		
	// 	switch(ch)
	// 	{
	// 		case 1: bot.left_hand->torqueOFF();
	// 			bot.right_hand->torqueOFF();
	// 			break;
	// 		case 2: bot.left_hand->torqueON();
	// 			bot.right_hand->torqueON();
	// 			break;
	// 		case 3: 

	// 			printf("\nLeft Hand ");
	// 			// bot.left_hand->readMotorValues();
	// 			bot.right_hand->readMotorValues();
	// 			break;
	// 		case 4: break;
	// 	}
	// }


	/*	for (int i = 0; i < 3; ++i)
	{
		bot.right_hand->torqueOFF();
		usleep(100000);
	}*/

	//for (int i = 0; i < 2; ++i)
	//{
	//	bot.right_hand->readMotorValues();
	//	usleep(100000);
	//}

	int arr[4];

	fstream f1;
	f1.open("salute.move", ios::out);

	for (int i = 0; i < 102; ++i)
	{
		bot.right_hand->readMotorValues(arr);
	 	if(i <=2)
	 		continue;
	 	cout<<arr[0]<<" "<<arr[1]<<" "<<arr[2]<<"\n";	
	 	f1<<arr[0]<<" "<<arr[1]<<" "<<arr[2]<<"\n";
	 	usleep(100000);
	}

	f1.close();



/*	fstream f1;
	f1.open("~/Dropbox/WalkCodes/shreyas/getups/wave.move", ios::in);

	bot.right_hand->setSpeed(100);
	comm.syncFlush();

	for (int i = 0; i < 100; ++i)
	{
		f1>>arr[0]>>arr[1]>>arr[3];
		arr[2] = 0;
		bot.right_hand->setGoalPositionSync(arr);
		comm.syncFlush();
		usleep(100000);
	}

	f1.close();
*/	bot.initialize();
	
	return 0;
}
