# include "walk.h"



# include <cstdio>
 


int main(void)
{
	printf("hello\n");
	int inp=0;
	double radius=0, theta=0;
	printf("hello\n");
	Walk a;

	printf("hello\n");

	do
	{	
		printf("hello\n");
		printf("Enter radius\n");
		scanf("%lf",&radius);
		printf("Enter angle\n");
		scanf("%lf",&theta);

		a.move(radius,theta);

		printf("Enter -1 to terminate \n");
		scanf("%d",&inp);

	}while(inp!=-1);
	


}