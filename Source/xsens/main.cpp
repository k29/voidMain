#include "imu.h"
#include "unistd.h"
#include <fstream>
#include <iostream>
#include <iomanip>
using namespace std;

// int main()
// {

// 	Imu i;
// 	i.init();
	
// 	i.__flush();
// 	i.__update();
// 	usleep(500000);
// 	// while(1)
// 	// {
// 	// 	i.__update();
		
// 	// 	int value = i.yaw;// - offset;
// 	// 	fstream f1;
// 	// 	f1.open("imuyaw.angle", std::ios::out);
// 	// 	f1<<value;
// 	// 	f1.close();
// 	// 	printf("Value %lf\n", value);
// 	// 	// printf("%lf %lf %lf\n", i.roll, i.pitch, i.yaw);
// 	// 	// count++;
// 	// 	// if(count > 9)
// 	// 	// {
// 	// 	// 	count = 0;
// 	// 	// 	offset++;
// 	// 	// }
// 	// 	// usleep(500000);
// 	// }
// }
int main()
{
	Imu imu;
	if(!imu.init())
		return 0;
	int i=100000;
	double value;
	fstream f1;
	f1.open("Source/xsens/imuyaw.angle",ios::out);
	if(f1.is_open())
		printf("FILE OPENED SUCCESSFULLY\n");
	usleep(2000000);
	while(i--)
	{		
		value=imu.yaw;
		printf("%lf\n",value);		
	}
	f1<<imu.yaw;
	f1.close();
	return 0;
}
