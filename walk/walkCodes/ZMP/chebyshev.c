#include<stdio.h>
#include<math.h>

void main()
{
	int n=30;
	int a=0;
	int b=-120;
	int i=0;
	for(i=1;i<=n;i++)
	{
		printf("%lf\n",ceil(0.5*(a+b)+0.5*(b-a)*cos((2.0*i-1)/(2.0*n)*3.14)));
	}
}
