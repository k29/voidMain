#include "xsens/imu.h"

int main()
{
	Imu imu;
	imu.init();
	while(1)
	{
	printf("%lf\t%lf\t%lf\n",imu.roll,imu.pitch,imu.yaw);
	}
	return 0;
}
