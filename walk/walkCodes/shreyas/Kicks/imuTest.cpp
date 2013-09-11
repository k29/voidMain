#include "xsens/imu.h"
#include <signal.h>
#include <sys/time.h>
#include <unistd.h>

int quit=1;

void doquit(int para)
{
	quit = 0;
}

int main()
{
	Imu imu;
	imu.init();
	(void) signal(SIGINT,doquit);
	
	
	while(quit)
	{
		usleep(10);
		printf("Roll\t%3.3lf\tPitch\t%3.3lf\tYaw\t%3.3lf\n",imu.roll,imu.pitch,imu.yaw);	
	}
};

