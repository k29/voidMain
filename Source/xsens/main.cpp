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
	imu.init();
<<<<<<< HEAD
	int i=500000;
	double value;
	while(i--)
	{
		
=======
	int i=500;
	double value;
	while(i--)
	{
		imu.update();
>>>>>>> 69f8829d20fbb6f84639f7d7262273b65cc18225
		value=imu.yaw;
		printf("%lf\n",value);
		
	}
<<<<<<< HEAD
=======
	fstream f1;
	f1.open("Source/xsens/imuyaw.angle",std::ios::out);
	f1<<value;
	f1.close();
	printf("Value written to file is %lf",value);
>>>>>>> 69f8829d20fbb6f84639f7d7262273b65cc18225
	return 0;
}
