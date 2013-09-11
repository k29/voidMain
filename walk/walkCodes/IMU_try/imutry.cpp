#include "xsens/imu.h"

int main()
{
	Imu imu;
	imu.init();
	while(1)
	{
		imu.update();
		printf("Roll: %lf Pitch: %lf Yaw: %lf\n", imu.roll, imu.pitch, imu.yaw);
	}
	return 0;
}