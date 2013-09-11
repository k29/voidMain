#include "AcYut.h"
#include "communication.h"
#include "xsens/imu.h"
#include <signal.h>

int main()
{
	Imu imu;
	imu.init();
	
	Communication comm;
	AcYut bot(&comm,NULL);

	int ch=1;
	
	while(ch!=4)
	{
		printf("\n1. Torque OFF\n2. Torque ON \n3. Read MOTOR values\n 4.EXIT\n");
		scanf("%d",&ch);
		
		switch(ch)
		{
			case 1: bot.left_hand->torqueOFF();
				bot.right_hand->torqueOFF();
				
				break;
			case 2: bot.left_hand->torqueON();
				bot.right_hand->torqueON();
					
				break;
			case 3: printf("Left Hand ");
				bot.left_hand->readMotorValues();
				printf("\nRight Hand ");
				bot.right_hand->readMotorValues();
				
				break;
			case 4: break;
		}
	}
	
	return 0;
}
