#include "imu.h"
#include "unistd.h"
#include <fstream>

using namespace std;

int main()
{
	while(1)
	{
		fstream f1;
		f1.open("imuyaw.angle", ios::in);
		int value;
		f1>>value;
		printf("Value is %d\n", value);
		f1.close();
		usleep(1000000);
	}
}