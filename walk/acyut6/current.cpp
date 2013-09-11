#include <stdio.h>
#include <cstdio>
#include "communication.h"
#include "motor.h"
#include <iostream>

using namespace std;

int main()
{
	Communication comm;
	int id;
	printf("Enter ID\n");
	scanf("%d",&id);
	Motor m(MX106,id,&comm);
	FILE *fp;
	fp = fopen("current.dat", "w");
	double curr;
	while(1)
	{
		curr=m.getSensedCurrent();
		fprintf(fp,"%lf\n",curr);
		cout<<"\n"<<curr;
	}

	return 0;
}


