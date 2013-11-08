#include "imu.h"
#include "unistd.h"
#include <fstream>

using namespace std;

int main()
{
	Imu i;
	i.init();
	int offset;
	
	i.__flush();
	i.__update();
	offset = (int)i.yaw;
	int count = 0;
	usleep(500000);
	while(1)
	{
		i.__flush();
		i.__update();
		int value = ((int)i.yaw) - offset;
		fstream f1;
		f1.open("imuyaw.angle", std::ios::out);
		f1<<value;
		f1.close();
		// printf("Value %d\n", value);
		// printf("%lf %lf %lf\n", i.roll, i.pitch, i.yaw);
		count++;
		if(count > 9)
		{
			count = 0;
			offset++;
		}
		usleep(500000);
	}
}