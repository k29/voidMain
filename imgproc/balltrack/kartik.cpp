#include "imgproc.h"
#include "camcapture.h"
#include <opencv2/opencv.hpp>
#include <FlyCapture2.h>
#include <stdio.h>
#include <ftdi.h>
#include <unistd.h>
#include <termios.h>
#include "headmotor.h"
#include "defines.h"

int main()
{
int h,w,x,y;
x=0;
y=0;
int n=0;
float avgx;
float avgy;
float addx=0.0;
float addy=0.0;
int rgb[3];
float rb=0.0;
float rg=0.0;
float angle1,angle2;
 	
CamCapture cam;
HeadMotor hm;
hm.bootup_files();
cam.init();

while( 1 ) 
	{
	cam.getImage();
        hm.update();
        //if( !cam.rgbimg ) break;
        w=cam.rgbimg->width;
	h=cam.rgbimg->height;
	IplImage* image2=cvCreateImage(cvSize(w,h),8,1);
	for(int i=0;i<w;i++)
		{
		for(int j=0;j<h;j++)
			{
				if(cam.isRed(i,j)==true)
				{
				x=x+i;
				y=y+j;
				n++;
				
			//	}
			//pixelColor3C(cam.rgbimg, i, j, rgb);
			//rb=(float)rgb[2]/(float)rgb[0];
			//rg=(float)rgb[2]/(float)rgb[1];
			//if(rg>=0.472&&rb>=0.4720)
			//	{
				returnPixel1C(image2, i, j)=255;
				}
			else
				{
				returnPixel1C(image2, i, j)=0;
				}	
			}
		}
	avgx=(float)x/(float)n;
	avgy=(float)y/(float)n;
	printf("x%f",avgx);
	printf(" y%f",avgy);
	printf("of no orange pixels%d\n",n); 
	x=0;
	y=0;
	//cvShowImage("input",cam.rgbimg);
	//cvShowImage("result",image2);
	hm.read_motor(angle1,angle2);
	if(avgx<w/2+50.0&&avgx>w/2-50.0&&avgy>h/2-50.0&&avgy<h/2+50.0)
	{
	}
	else
	{
	if(avgx>w/2)
	{
	addx=2.0;
	}
	if(avgx<w/2)
	{
	addx=-2.0;
	}
	if(avgy>h/2)
	{
	addy=-2.0;
	}
	if(avgy<h/2)
	{
	addy=2.0;
	}
	hm.write_motor(angle1+addy,angle2+addx);
	}
	cvShowImage("input",cam.rgbimg);
	cvShowImage("result",image2);
	n=0;
	addx=0.0;
	addy=0.0;
	cvWaitKey(1);
	}
return 0;	
}	
		
				
			
