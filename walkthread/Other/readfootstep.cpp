#include <fstream>

struct foot
{
	double delta_x,delta_y,delta_theta;				
};

using namespace std;

int main()
{
	fstream fil;
	foot step[3000];
	fil.open("Errorright.dat", ios::in|ios::binary);
	int i = 0;
	while (!fil.eof())
	{
		fil.read((char*)&step[i],sizeof(step[i]));
		printf ("Delta_y = %f\tDelta_x = %f\tDelta_theta = %f\n",step[i].delta_y,step[i].delta_x,step[i].delta_theta);
	}
	fil.close();
	return 0;
}