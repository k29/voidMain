#include<stdio.h>
void main()
{
float c;
int xmin,xmax,zmin,zmax,fps,t1,t,m,yi,yf;
int zin,zrin,xin,xrin,yin,yrin;
float z,y,x;
printf("enter time-");
scanf("%d",&t);
printf("enter initial y-");
scanf("%d",&yi);
printf("enter final y-");
scanf("%d",&yf);
xmin=350;
xmax=390;
zmin=0;
zmax=50;
fps=60;
t1=t*fps;
for(m=0;m<t1;m++)
{
	c=(float)m/(float)t1;
	if(c<0.5)
	{
	z=zmin+(zmax-zmin)*(c*2)*(c*2)*(3-(4*c));
	x=xmax-(xmax-xmin)*(c*2)*(c*2)*(3-(4*c));
	}
	else
	{
	z=zmin+(zmax-zmin)*(2*c-2)*(2*c-2)*((4*c)-1);
	x=xmax-(xmax-xmin)*(2*c-2)*(2*c-2)*((4*c)-1);
	}
	y=yi+(yf-yi)*c*c*(3-(2*c));
	zin=(int)z;
	zrin=-zin;
	xin=(int)x;
	xrin=xmax;
	yin=(int)y;
	yrin=-yin;
	printf("\nl1: x:%d y:%d z:%d \t l2: x:%d y:%d z:%d",xin,yin,zin,xrin,yrin,zrin);
}
}
