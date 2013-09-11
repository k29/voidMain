#include<math.h>
#include<inttypes.h>
#include<ftdi.h>
#include<stdio.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <termios.h>
void st();
struct ftdi_context ftdic1;
#define serialusb2d "A7003N1d"
struct ftdi_context ftdic2;
#define serialusb3d "FTSBY5CA"//"A7003N1d"

int bootup_files1()
{
int ret;
    if (ftdi_init(&ftdic2) < 0)
    {
        fprintf(stderr, "ftdi_init failed\n");
        return 0;
    }
    if(ftdi_set_interface(&ftdic2, 2) < 0)
	printf("INTERFACE NOT SET");
    if ((ret = ftdi_usb_open_desc (&ftdic2, 0x0403, 0x6011, NULL, serialusb3d)) < 0)
    {
        fprintf(stderr, "unable to open ftdi device: %d (%s)\n", ret, ftdi_get_error_string(&ftdic2));
        printf("unable to open USB2D");
        return 0;
    }
    else
    {
	ftdi_set_baudrate(&ftdic2,57600);
	printf("USB2D ftdi successfully open\n");
    }
 
}
int kbhit2()
{
	int	 count = 0;
	int	 error;
	static struct termios	Otty, Ntty;

	tcgetattr(STDIN_FILENO, &Otty);
	Ntty = Otty;

	Ntty.c_lflag &= ~ICANON; /* raw mode */
	Ntty.c_cc[VMIN] = CMIN; /* minimum chars to wait for */
	Ntty.c_cc[VTIME] = CTIME; /* minimum wait time	 */

	if (0 == (error = tcsetattr(STDIN_FILENO, TCSANOW, &Ntty)))
	{
		struct timeval	tv;
		error += ioctl(STDIN_FILENO, FIONREAD, &count);
		error += tcsetattr(STDIN_FILENO, TCSANOW, &Otty);
/* minimal delay gives up cpu time slice, and
* allows use in a tight loop */
		tv.tv_sec = 0;
		tv.tv_usec = 10;
		select(1, NULL, NULL, NULL, &tv);
	}
	return (error == 0 ? count : -1 );
}



int gd();
int compare_float(float f1, float f2)
{
  float precision = 1.0/60;
  if (((f1 - precision) < f2) && 
      ((f1 + precision) > f2))
   {
    return 1;
   }
  else
   {
    return 0;
   }
}

int bootup_files()
{
int ret;
    if (ftdi_init(&ftdic1) < 0)
    {
        fprintf(stderr, "ftdi_init failed\n");
        return 0;
    }
    
    if ((ret = ftdi_usb_open_desc (&ftdic1, 0x0403, 0x6001, NULL, serialusb2d)) < 0)
    {
        fprintf(stderr, "unable to open ftdi device: %d (%s)\n", ret, ftdi_get_error_string(&ftdic1));
        printf("unable to open USB2D");
        return 0;
    }
    else
    {
	ftdi_set_baudrate(&ftdic1,1000000);
	printf("USB2D ftdi successfully open\n");
    }
 
}
int offset[]={5,295,265,190,24,-55,306,0,0,-100,0,0,254,0,0,0,0,0,0,0,4,274,298,403,-2,-8,-306,0,0,0,0,0,273};
typedef unsigned char byte;
float* generateik(float x, float y, float z,int leg);
byte pack[]={0xff,0xff,0xfe,0x2e,0x83,0x1e,0x02,0x00,0x00,0x00,0x01,0x00,0x00,0x02,0x00,0x00,0x03,0x00,0x00,0x04,0x00,0x00,0x05,0x00,0x00,0x14,0x00,0x00,0x15,0x00,0x00,0x16,0x00,0x00,0x17,0x00,0x00,0x18,0x00,0x00,0x19,0x00,0x00,0x0c,0x00,0x00,0x20,0x00,0x00,0x00};
int cs=0;
int t=1;
int xmin,xmax,zmin,ymax,ymin,zmax,fps;
byte temp,aa[20],packfinal[22];
float mval[5];
int count;
void main()
{
int ym[12];
float p;
char b;
bootup_files();
bootup_files1();
int a;
xmin=350;
xmax=390;
zmin=0;
zmax=28;
fps=60;
ymax=30;
ymin=0;
int count;
int yi,yf;
ym[0]=0;
for(count=0;count<=10;count++)
{
	p=(float)count/(float)10;	
	if(p<0.5)
		ym[count+1]=ymin+(ymax-ymin)*2*p*2*p*(3-4*p);
	else
		ym[count+1]=ymin+(ymax-ymin)*(2*p-2)*(2*p-2)*(4*p-1);	
	printf("%d\n",count+1);
	if(count<=5)		
	{
		if(count%2==0)
			st(-ym[count],ym[count+1],1);
		else
			st(-ym[count],ym[count+1],2);
	}
	else
	{
		if(count%2==0)
			st(-ym[count],ym[count+1],2);
		else
			st(-ym[count],ym[count+1],1);
	}
	if(count==5)
	{
		for(a=0;a<9;a++)		
		{
			if(a%2==0)
				st(-ymax,ymax,1);
			else
				st(-ymax,ymax,2);
			//scanf("%c",&b);		
		}
	}
	//scanf("%c",&b);
}//end of for
}
float* generateik(float x, float y, float z,int leg)
{
	//printf("%lf %lf %lf\t",x,y,z);
	int i=0;
	float cons[10];
	float ang[6];
	float theta[4];
	float phi[5];
	float b_len=180;   // Length of bracket 
	float l22=49;		// Distance between bracket ends
	float r2d=180/3.14159265358979323846264338327950288419716939937510;
	float leg_l=pow(x*x+y*y+z*z,0.5);
	
	cons[0]=1675;
	cons[1]=2952+255;
	cons[2]=2008+973;
	cons[3]=1459+255;
	cons[4]=1703;
	
	// CALCULATING LEG LENGTH
	float val=(leg_l-l22)/(2*b_len);
	theta[3]=acos(val); // angle for motor 3 from XZ Acyut plane 
	theta[2]=asin(val);	// angle for motor 2 12 from YZ Acyut plane 
	theta[1]=theta[3];	// angle for motor 1 from XZ Acyut plane 
	
	// CALCULATING LEG POSITION
	
	phi[3]=atan(y/x);
	phi[5]=atan(z/x);
	phi[1]=phi[3];
	phi[0]=phi[5];
	
	// GETTING ANGLES
	//printf("%lf asdjhkdshdk\n",avg_angle[1]);
	ang[0]=phi[0];//+(avg_angle[1]*pow(-1,leg))/r2d;
	ang[1]=-theta[1]+phi[1];//-(avg_angle[0]/r2d);
	ang[2]=-theta[2];
	ang[3]=-theta[3]-phi[3];
	ang[4]=phi[5];//+(avg_angle[1]/r2d)*pow(-1,leg);;
	//printf("%f\t%f\n",phi[0],ang[0]);
	mval[0]=(ang[0]*r2d*4096/300)+cons[0];//+z*1.1;
	mval[1]=(ang[1]*r2d*4096/300)+cons[1];
	mval[2]=(ang[2]*r2d*4096/300)+cons[2];
	mval[3]=(ang[3]*r2d*4096/300)+cons[3];
	mval[4]=(ang[4]*r2d*4096/300)+cons[4];//-z*1.1;

	// GEN PRINTING FOR DEBUGGING
	
	//for(i=0;i<5;i++)
	//{
		//printf("ANG[%d] : %f\t",i,ang[i]*r2d);
	//}
	//printf("\n");
	//for(i=0;i<5;i++)
	//printf("mval[%d] : %f\t",i,mval[i]);
	//printf("\n");
	//printf("THETA 3 : %f\tTHETA 2 : %f\tTHETA 1 : %f\t\n",theta[3]*r2d,theta[2]*r2d,theta[1]*r2d);
	//printf("PHI 3 : %f\tPHI 5 : %f\tPHI 1 : %f\tPHI 0 : %f\t\n",phi[3]*r2d,phi[5]*r2d,phi[1]*r2d,phi[0]*r2d);

	return mval;
}
void st(int yi,int yf, int wl)
{
int t1,m,ii;
t1=t*fps;
float z,y,x,c;
int zin,zrin,xin,xrin,yin,yrin;
for(m=0;m<t1;m++)
		{	
			cs=0;
			int var=kbhit2();
			c=(float)m/(float)t1;
			//printf("TIME %f\n",c); 
			if(c<0.5)
			{
				if(c<0.25)
					x=xmax;
				//printf("C < 0.5\n");
				//z=zmin+(zmax-zmin)*(c*2)*(c*2)*(3-(4*c));
				else
					x=xmax-(xmax-xmin)*((c-0.25)*4)*((c-0.25)*4)*(3-((c-0.25)*8));
			}
			else
			{
				if(c>0.75)
					x=xmax;
				//printf("C > 0.5\n");
				//z=zmin+(zmax-zmin)*(2*c-2)*(2*c-2)*((4*c)-1);
				else
				x=xmax-(xmax-xmin)*(4*(c-0.25)-2)*(4*(c-0.25)-2)*((8*(c-0.25))-1);
			}
			if(c<0.25)
				z=zmin+(zmax-zmin)*(2-4*c)*(4*c);
			else if(c>0.75)
				z=zmin+(zmax-zmin)*(2-4*(c-0.5))*(4*(c-0.5));
			else
				z=zmax;
			y=yi+(yf-yi)*c*c*(3-(2*c));
			zin=(int)z;
			zrin=-zin;
			xin=(int)x;
			xrin=xmax;
			yin=(int)y;
			yrin=-yin;
			if(wl==1)
				generateik(xin,yin,zin,1);
			else
				generateik(xrin,yrin,zrin,1);
			gd();
			pack[8]=(int)(mval[0]+offset[0])%256;
			pack[9]=(mval[0]+offset[0])/256;
			pack[11]=(int)(mval[1]+offset[1])%256;
			pack[12]=(mval[1]+offset[1])/256;
			pack[14]=(int)(mval[2]+offset[2])%256;
			pack[15]=(mval[2]+offset[2])/256;
			pack[17]=(int)(mval[3]+offset[3])%256;
			pack[18]=(mval[3]+offset[3])/256;
			pack[20]=(742+offset[4])%256;
			pack[21]=(742+offset[4])/256;
			pack[23]=(int)(mval[4]+offset[5])%256;
			pack[24]=(mval[4]+offset[5])/256;
			pack[44]=(int)(mval[2]+offset[12])%256;
			pack[45]=(mval[2]+offset[12])/256;
			//for(ii=0;ii<5;ii++)
			//		printf(" %f\n",mval[ii]);
			//printf("\n");
			if(wl==1)		
				generateik(xrin,yrin,zrin,2);
			else
				generateik(xin,yin,zin,2);				
			gd();			
			pack[26]=(int)(mval[0]+offset[20])%256;
			pack[27]=(mval[0]+offset[20])/256;
			pack[29]=(int)(mval[1]+offset[21])%256;
			pack[30]=(mval[1]+offset[21])/256;
			pack[32]=(int)(mval[2]+offset[22])%256;
			pack[33]=(mval[2]+offset[22])/256;
			pack[35]=(int)(mval[3]+offset[23])%256;
			pack[36]=(mval[3]+offset[23])/256;
			pack[38]=(274+offset[24])%256;
			pack[39]=(274+offset[24])/256;
			pack[41]=(int)(mval[4]+offset[25])%256;
			pack[42]=(mval[4]+offset[25])/256;
			pack[47]=(int)(mval[2]+offset[32])%256;
			pack[48]=(mval[2]+offset[32])/256;
			//printf("%x\t%x\t",pack[0],pack[1]);	
			
		for(ii=2;ii<49;ii++)
			{
			//	printf("%x\t",pack[ii]);		
				cs=cs+(int)pack[ii];
			//	printf("%x\n",cs);
			}
			pack[49]=0xff-(byte)(cs);		
			//printf("%x\n",pack[49]);	
			if(ftdi_write_data(&ftdic1,pack,50)<0)
				printf("PACKET SENDING ERROR\n");
			usleep(1000000/fps);
			//while(!(kbhit2()>var));
			//printf("\n");
			///printf("l1: x:%d y:%d z:%d l2: x:%d y:%d z:%d \n",xin,yin,zin,xrin,yrin,zrin);
			//	/printf("\n");		
			//for(ii=0;ii<5;ii++)
			//		printf(" %f ",mval[ii]);	
		}
	offset[3]=offset[3]+5;
	offset[23]=offset[23]+5;
}
int gd()
{
	int count=0;
	int theta_1,theta_2,theta_3,a1,a2,a3,v1,v2,v3;
	int i;
	
	while(1)
	{
		
		usleep(10);
		ftdi_read_data(&ftdic2,&temp,1);
//		printf("%d\n",temp);
		//continue;
		if(count==0)
		{
			if(temp==0xff)
			{
				packfinal[0]=0xff;
				count++;
				printf("GOT FIRST 0xff\n");
			}
			else
			{
				count=0;
			}
		}
		else if(count==1)
		{
			if(temp==0xff)
			{
				packfinal[1]=0xff;
				count++;
				printf("GOT SECOND 0xff\n");
	
			}
			else
			{
				count=0;
			}
		}
		if(count==2)
		{
//			printf("HERE\n");
			ftdi_read_data(&ftdic2,aa,20);
			if((aa[18]==0xfe)&&(aa[19]==0xfe))
			{
//				printf("READ 20 Correct data\n");
				for(i=0;i<20;i++)
				{
					packfinal[i+2]=aa[i];
					
				}
			theta_1=-180+(int)(packfinal[2]+256*((int)packfinal[3]))/2;
			theta_2=-180+(int)(packfinal[4]+256*((int)packfinal[5]))/2;
			theta_3=-180+(int)(packfinal[6]+256*((int)packfinal[7]))/2;
			a1=-300+(int)(packfinal[8]+256*((int)packfinal[9]));
			a2=-300+(int)(packfinal[10]+256*((int)packfinal[11]));
			a3=-300+(int)(packfinal[12]+256*((int)packfinal[13]));
			v1=-300+(int)((packfinal[14]+256*((int)packfinal[15]))/100);
			v2=-300+(int)((packfinal[16]+256*((int)packfinal[17]))/100);
			v3=-300+(int)((packfinal[18]+256*((int)packfinal[19]))/100);
			count==0;
			printf("\n %d %d %d %d %d %d %d %d %d\t",theta_1,theta_2,theta_3,a1,a2,a3,v1,v2,v3);
				count++;
				return 1;
			}
			else
				count=0;
		}
		/*if(count==3)
		{
			printf("DATA FOUND\n");	
			//	for(i=0;i<22;i++)
			//	printf("%x ",packfinal[i]);
			theta_1=-180+(int)(packfinal[2]+256*((int)packfinal[3]))/2;
			theta_2=-180+(int)(packfinal[4]+256*((int)packfinal[5]))/2;
			theta_3=-180+(int)(packfinal[6]+256*((int)packfinal[7]))/2;
			a1=-300+(int)(packfinal[8]+256*((int)packfinal[9]));
			a2=-300+(int)(packfinal[10]+256*((int)packfinal[11]));
			a3=-300+(int)(packfinal[12]+256*((int)packfinal[13]));
			v1=-300+(int)((packfinal[14]+256*((int)packfinal[15]))/100);
			v2=-300+(int)((packfinal[16]+256*((int)packfinal[17]))/100);
			v3=-300+(int)((packfinal[18]+256*((int)packfinal[19]))/100);
			count==0;
			printf("\n %d %d %d %d %d %d %d %d %d\t",theta_1,theta_2,theta_3,a1,a2,a3,v1,v2,v3);
		}*/
	}	
}
