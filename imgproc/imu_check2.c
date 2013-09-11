#include <math.h>
#include <stdio.h>
#include <sys/time.h>
#include <unistd.h>
#include <stdlib.h>
#include <inttypes.h>
#include <string.h>  /* String function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include <time.h>
#include <ftdi.h>
#include <sys/ioctl.h>
#include <sys/types.h>


#define MOTOR_DATA 0
#define IMU_DATA 0
#define FT4232 "FTSBY5CA"//"FTSC152E"//"FTSC152E"
#define NOM 28                                  //Current number of Motors
#define LEN_SYNC_WRITE ((NOM*3)+4) 		//Total Lenght Of Packet
#define INST_SYNC_WRITE     0x83
#define SMOOTHSTEP(x) ((x) * (x)*(3-2*(x)))
#define max( a, b ) ( ((a) > (b)) ? (a) : (b) )
#define min( a, b ) ( ((a) < (b)) ? (a) : (b) )

#define acyut 4  //acyut using...........................................change here..............
#define E 2.7183
#define D2R (3.14/180)
#define MAX_BOUNDARIES 160
#define TRUE 1
#define serialimu "A700eSSZ"//"A4007rXR"//"A8004Yt8" //"A8008c3o" 
long time_start, time_end;
struct timeval tv;
 typedef uint8_t byte;
	
///////////IMU DECLARATIONS//////////////////////////
/// GEN DECLARATIONS
int imu_init=0;

/// BASE VALUES 
float base_angle[3]={0};
float base_acc[3]={0};
float base_gyro[3]={0};
float gravity_magnitude=0;
float gravity[3]={0};

/// SENSOR DATA
float imu_angle[3]={0};  
float imu_acc[3]={0};
float imu_gyro[3]={0};


/// INFERRED
float current_angle[3]={0};
float current_acc[3]={0};
float held_angle[3]={0};
float gyro_angle[3]={0};

struct ftdi_context imu;
int imu_t;
float scurve(float in,float fi,float t, float tot);
void imucal();
int kbhit()
{
	struct timeval tv;
	fd_set read_fd;
	tv.tv_sec=0;
	tv.tv_usec=0;
	FD_ZERO(&read_fd);
	FD_SET(0,&read_fd);
	if(select(1, &read_fd,NULL, /*No writes*/NULL, /*No exceptions*/&tv) == -1)
		return 0; /* An error occured */
	if(FD_ISSET(0,&read_fd))
		return 1;
	return 0;
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

void close_files()
{
	int ret_imu;

	if(IMU_DATA==0)
	{
		if((ret_imu=ftdi_usb_close(&imu)) < 0)
		{
			fprintf(stderr, "unable to close usb2d ftdi device: %d (%s)\n", ret_imu, ftdi_get_error_string(&imu));
    		}
		ftdi_deinit(&imu);
	}
	else if(IMU_DATA==1)
	close(imu_t);

}


int bootup_files()
{
	int ret_imu;	
	if(IMU_DATA==0)
	{
		if (ftdi_init(&imu) < 0)
    		{
	        	fprintf(stderr, "ftdi_init failed\n");
        		ret_imu=-99;
        		//return EXIT_FAILURE;
    		}	
   		//ftdi_set_interface(&imu,2);
  		if ((ret_imu= ftdi_usb_open_desc (&imu, 0x0403, 0x6001, NULL, serialimu)) < 0)
    		{
        		fprintf(stderr, "unable to open ftdi device: %d (%s)\n", ret_imu, ftdi_get_error_string(&imu));
        		printf("unable to open IMU\n");
       			 //return EXIT_FAILURE;
    		}	
    		else
    		{
    			printf("IMU ftdi successfully open\n");
	   		ftdi_set_baudrate(&imu,1000000);
    		}
	}   
   	else if (IMU_DATA==1)
	{
		struct termios imu_options;
		if((imu_t=open("/dev/ttyUSB2", O_RDWR | O_NOCTTY | O_NDELAY))==-1)
		{
			ret_imu=-1;
			printf("IMU Port Unavialable\n");
			close_files();
			exit(0);
		}
		tcgetattr(imu_t, &imu_options);
		cfsetispeed(&imu_options, B57600);
		cfsetospeed(&imu_options, B57600);
		imu_options.c_cflag |= (CLOCAL | CREAD);
  		imu_options.c_cflag &= ~PARENB;
  		imu_options.c_cflag &= ~CSTOPB;
   		imu_options.c_cflag &= ~CSIZE;
   		imu_options.c_cflag |= CS8;
  		imu_options.c_oflag |= OPOST;   
   		imu_options.c_oflag &= ~ONOCR;
  		imu_options.c_oflag |= FF0;
  		imu_options.c_oflag &= ~FF1;
   		imu_options.c_oflag |= ONLCR;
		imu_options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);    
   		imu_options.c_cflag &= ~CRTSCTS;  
//		tcflush(fd, TCIFLUSH);
  		tcsetattr(imu_t, TCSANOW, &imu_options);
  		fcntl(imu_t, F_SETFL,0);
		
	}
	else
	printf("UNKNOWN IMU CONNECTION\n");
	
	if(ret_imu>=0)
	return EXIT_SUCCESS;	
	else
	return EXIT_FAILURE;

}

void imucal_init()
{
	//printf("entered imu cal init\n");
	float ang[3]={0};
	float acc[3]={0};
	float gyro[3]={0};
	byte im[20];
	byte i;
	byte j;
	byte k=0;
	byte a=3;
	byte flag=0;
	byte imu_no_of_times=0;
	byte imu_noat=0;
	byte flag2=0;
	byte start=0;
	base_angle[0]=base_angle[1]=base_angle[2]=0;
	//ftdi_usb_purge_buffers(&imu);
	//tcflush(imu,TCIOFLUSH);
	while(imu_no_of_times<10)
	{
		imu_noat=0;
		flag2=0;
		while(flag2==0&&imu_noat<10)
		{   
				
			//if(ftdi_read_data(&imu,&im[0],1)>0)
			if(iread(IMU_DATA,&imu_t,&imu,&im[0],1)>0)
			{	
				//printf("%d\t",im[0]);
							
				if(im[0]==0xff)
				{
					//printf("first ff detected....\n");
					start++;
				}
				else
				start=0;
				
				if(start==2)
				{	
					start=0;
					i=0;
					printf("HERE\n");
					//if(ftdi_read_data(&imu,&im[i],8)>0 && im[6]==254 && im[7]==254)
					if(iread(IMU_DATA,&imu_t,&imu,&im[i],8)>0 && im[6]==254 && im[7]==254)
					{	
						flag++;
						flag2=1;
						for(j=0,k=0;j<6;j+=2,k+=1)
						{
							ang[k]=im[j]+im[j+1]*256;
							ang[k]=(ang[k] - 0) * (180 + 180) / (720 - 0) + -180;
							base_angle[k]+=ang[k];
						}
/*						for(j=0,k=0;j<6;j+=2,k+=1)*/
/*						{*/
/*							acc[k]=im[j+6]+im[j+7]*256;*/
/*							acc[k]=(acc[k] - 0) * (300 + 300) / (600 - 0) + -300;*/
/*							base_acc[k]+=acc[k];	*/
/*						}*/
/*						for(j=0,k=0;j<6;j+=2,k+=1)*/
/*						{*/
/*							gyro[k]=im[j+12]+im[j+13]*256;*/
/*							gyro[k]=(gyro[k] - 0) * (300 + 300) / (60000 - 0) + -300;*/
/*							base_gyro[k]+=gyro[k];	*/
/*						}*/
						//printf("FLAG : %d BASE_ANG 0: %f BASE_ANG 1: %f BASE_ANG 2: %f\n",flag,base_angle[0],base_angle[1],base_angle[2]);				
						printf("FLAG : %d BASE_ANG 0: %f BASE_ANG 1: %f BASE_ANG 2: %f BASE_ACC 0: %f BASE_ACC 1: %f BASE_ACC 2: %f BASE_GYRO 0: %f BASE_GYRO 1: %f BASE_GYRO 2: %f\n",flag,ang[0],ang[1],ang[2],acc[0],acc[1],acc[2],gyro[0],gyro[1],gyro[2]);	
						
					}
				 }
			}
			imu_noat++;
		}
		imu_no_of_times++;
	}
	if(flag>=1)
	{
		for(k=0; k<3; k++)
		{
			base_angle[k]=base_angle[k]/flag;
			base_acc[k]=base_acc[k]/flag;
			base_gyro[k]=base_gyro[k]/flag;
			printf("BASE ANGLE %d : %lf     ",k,base_angle[k]);
			printf("BASE ACC %d : %lf     ",k,base_acc[k]);
			printf("BASE GYRO %d : %lf     ",k,base_gyro[k]);
		}
		printf("IMU INITIALIZED\n");
		imu_init=1;
	}
	else
	{
		printf("IMU NOT RESPONDING\n");
		imu_init=0;
	}
	
	/*for(k=0;k<3;k++)		
	{
		printf("BASE ANGLE : %d is %lf\n",k,base_angle[k]);
	}*/
	sleep(1);
}

void imu_initialize()
{
	imucal_init();
	if(imu_init==1)
	{
		gravity_magnitude=sqrt(pow(base_acc[0],2)+pow(base_acc[1],2)+pow(base_acc[2]+43,2));
		
		gravity[0]=base_acc[0];
		gravity[1]=base_acc[1];
		gravity[2]=base_acc[2]+43;
				
	}
}	

inline void imucal()
{
	byte im[20],i,j,k=0,a=3,fallen=0;
	float ang[3]={0};
	float acc[3]={0};
	float gyro[3]={0};
	byte flag=0,imu_no_of_times=0;
	imu_angle[0]=0;
	imu_angle[1]=0;
	imu_angle[2]=0;
	imu_acc[0]=0;
	imu_acc[1]=0;
	imu_acc[2]=0;
	imu_gyro[0]=0;
	imu_gyro[1]=0;
	imu_gyro[2]=0;
	int start=0,count=0;
	gettimeofday(&tv,NULL); /*gets the current system time into variable tv */ 
	time_start  = (tv.tv_sec *1e+6) + tv.tv_usec;
			ipurge(MOTOR_DATA,&imu_t,&imu);
	//printf("BASE_ANG 0: %f BASE_ANG 1: %f BASE_ANG 2: %f\n",base_angle[0],base_angle[1],base_angle[2]);	
	//if(imu_init!=1)
	//printf("INITIALIZE THE IMU\n");
	//tcflush(imu,TCIFLUSH);
	//tcflow(imu,TCIOFLUSH);
	while(imu_no_of_times<1&& imu_init==1)
	{		
		flag=0;
		while(flag==0)
		{   
			//if(ftdi_read_data(&imu,&im[0],1)>0)
			if(iread(IMU_DATA,&imu_t,&imu,&im[0],1)>0)
			{
				if(im[0]==0xff)
				start++;
				else
				start=0;
				
				if(start==2)
				{
					start=0;
					//printf("PACKET RECIEVED\n");
					i=0;
					//if(ftdi_read_data(&imu,&im[i],8)>0 && im[6]==254 && im[7]==254)
					if(iread(IMU_DATA,&imu_t,&imu,&im[i],8)>0 && im[6]==254 && im[7]==254)
					{	
						flag=1;	
					}
					
					if(flag==1)
					{	
						count++;
						//printf("count : %d\n",count);
						for(j=0,k=0;j<6;j+=2,k+=1)
						{
							ang[k]=im[j]+im[j+1]*256;
							ang[k]=(ang[k] - 0) * (180 + 180) / (720 - 0) + -180-base_angle[k];
							imu_angle[k]=imu_angle[k]+ang[k];
						}
/*						for(j=0,k=0;j<6;j+=2,k+=1)*/
/*						{*/
/*							acc[k]=im[j+6]+im[j+7]*256;*/
/*							acc[k]=(acc[k] - 0) * (300 + 300) / (600 - 0) + -300;*/
/*							imu_acc[k]=imu_acc[k]+acc[k];//-base_acc[k];*/
/*						}*/
/*						for(j=0,k=0;j<6;j+=2,k+=1)*/
/*						{*/
/*							gyro[k]=im[j+12]+im[j+13]*256;*/
/*							gyro[k]=(gyro[k] - 0) * (300 + 300) / (60000 - 0) + -300;*/
/*							imu_gyro[k]=imu_gyro[k]+(gyro[k]-base_gyro[k]);*/
/*						}*/
						//printf("FLAG : %d BASE_ANG 0: %f BASE_ANG 1: %f BASE_ANG 2: %f BASE_ACC 0: %f BASE_ACC 1: %f BASE_ACC 2: %f BASE_GYRO 0: %f BASE_GYRO 1: %f BASE_GYRO 2: %f\n",flag,ang[0],ang[1],ang[2],acc[0],acc[1],acc[2],gyro[0],gyro[1],gyro[2]);
						
					}	
				}
			}
		}
		imu_no_of_times++;
	}
	
	if(count!=0)
	{
		//angle_moved+=abs(imu_gyro[0])<20000?imu_gyro[0]:0;
		for(k=0;k<3;k++)		
		{
			imu_angle[k]=imu_angle[k]/count;
			imu_acc[k]=imu_acc[k]/count;
			imu_gyro[k]=imu_gyro[k]/count;
			printf("ANGLE %d : %lf\t",k,imu_angle[k]);
//			printf("ANGLE %d : %lf\t",k,ang[k]);
			printf("ACC %d : %lf\t",k,imu_acc[k]);
			printf("GYRO %d : %lf\n",k,imu_gyro[k]);
		}
	}
	else
	{
		printf("IMU READING FAILED\n");
		//usleep(16670*0.6);
	}

	gettimeofday(&tv,NULL); /*gets the current system time into variable tv */ 
	time_end  = (tv.tv_sec *1e+6) + tv.tv_usec;
	//ftdi_usb_purge_buffers(&imu);
	//tcflush(imu,TCIFLUSH);
	//tcflow(imu,TCIOFLUSH);
}

inline void get_imu_data()
{
	int i=0;
	float gyro_threshold=0.008;
	float gyro_scale=1.5;
	float acc_magnitude;
	if(imu_init==0)
	{
		printf("IMU NOT INITIALIZED\n");
		return ;
	}
	imucal();
	acc_magnitude=sqrt(pow(imu_acc[0],2)+pow(imu_acc[1],2)+pow(imu_acc[2]+43,2));
	printf("GRAVITY %f\t ACC %f\n",gravity_magnitude,acc_magnitude);
	printf("GYRO 0 :%f GYRO 1 : %fn",imu_gyro[0],imu_gyro[1]);
	if(fabs(imu_gyro[0])<=gyro_threshold&&fabs(imu_gyro[1])<=gyro_threshold)//&&abs(imu_gyro[2])<=gyro_threshold)
	{
		if(fabs(acc_magnitude-gravity_magnitude)<1)
		{	
			//// STATIC CONDITION - > RESETTING HELD ANGLE 
			for(i=0;i<2;i++)
			{
				printf("STATIC COND \n");
				held_angle[i]=imu_angle[i];
				gyro_angle[i]=0;
				//current_angle[i]=held_angle[i]+gyro_angle[i];
			}
		}
		else
		{
			//// LINEAR ACC CONDITION 
			for(i=0;i<2;i++)
			{
				printf("PURE ACC \n");
				held_angle[i]=held_angle[i];
				//gyro_angle[i]+=imu_gyro[i]*gyro_scale;	
				//current_angle[i]=held_angle[i]+gyro_angle[i];
			}
		}
	}
	else
	{
		for(i=0;i<2;i++)
		{
			printf("ROTATING\n");
			gyro_angle[i]+=imu_gyro[i]*gyro_scale;	
			printf("**********************************GYRO ANG %d : %f**********************\n",i,gyro_angle[i]);
		}
	}
	
	for(i=0;i<2;i++)
	{
		current_angle[i]=held_angle[i]+gyro_angle[i];	
		printf("ANGLE[%d] = %f\n",i,current_angle[i]);
	}
	
	//float temp=current_angle[1];
	current_angle[1]*=-1;//current_angle[0];
	current_angle[0]*=-1;//temp;
	imu_acc[2]+=43;
	current_acc[0]=cos(current_angle[0]*D2R)*imu_acc[0]-sin(current_angle[1]*D2R)*imu_acc[2];
	current_acc[1]=sin(current_angle[0]*D2R)*sin(current_angle[1]*D2R)*imu_acc[0]+cos(current_angle[0]*D2R)*imu_acc[1]+sin(current_angle[0]*D2R)*cos(current_angle[1]*D2R)*imu_acc[2];
	current_acc[2]=cos(current_angle[0]*D2R)*sin(current_angle[1]*D2R)*imu_acc[0]-sin(current_angle[0]*D2R)*imu_acc[1]+cos(current_angle[0]*D2R)*cos(current_angle[1]*D2R)*imu_acc[2];
	
	current_acc[0]-=gravity[0];
	current_acc[1]-=gravity[1];
	current_acc[2]-=gravity[2];
	/*current_acc[0]=cos(current_angle[1]*D2R)*imu_acc[0]+sin(current_angle[1]*D2R)*sin(current_angle[0]*D2R)*imu_acc[1]-sin(current_angle[1]*D2R)*cos(current_angle[0]*D2R)*imu_acc[2];
	current_acc[1]=cos(current_angle[0]*D2R)*imu_acc[1]+sin(current_angle[0]*D2R)*imu_acc[2];
	current_acc[2]=sin(current_angle[1]*D2R)*imu_acc[0]-sin(current_angle[0]*D2R)*cos(current_angle[1]*D2R)*imu_acc[1]+cos(current_angle[0]*D2R)*cos(current_angle[1]*D2R)*imu_acc[2];
	*/
	for(i=0;i<=2;i++)
	{
		printf("ACC[%d] = %f\t GRAVITY[%d] %f\n",i,current_acc[i],i,gravity[i]);
	}
	
	///// ROTATE ACC VECTOR
	///// SUBTRACT GRAVITY
}		
/*
inline void imucal()    //// OPTIMISED IMUCAL
{
	byte rcv_pkt[30];
	int try=0;
	int i=0,exit=0;
	//ftdi_usb_purge_buffers(&imu);
	//usleep(100);	
	for(try=0;try<5&&exit==0;try++)
	{
		printf("TRYING\n");
		if(ftdi_read_data(&imu,rcv_pkt,30)>0)
		{0
			printf("DATA READ\n");
			for(i=0;i<30;i++)
			printf("%d\t",rcv_pkt[i]);
			printf("\n");
			
			for(i=0;i<20&&exit==0;i++)
			{
				if(rcv_pkt[i]==0xff&&rcv_pkt[i+1]==0xff&&rcv_pkt[i+8]==0xfe&&rcv_pkt[i+9]==0xfe)
				{
					ftdi_usb_purge_buffers(&imu);
					current_angle[0]=(rcv_pkt[i+2]+rcv_pkt[i+3]*256)*(180 - (-180))/(720-0)+-180-base_angle[0];
					current_angle[1]=(rcv_pkt[i+4]+rcv_pkt[i+5]*256)*(180 - (-180))/(720-0)+-180-base_angle[1];
					current_angle[2]=(rcv_pkt[i+6]+rcv_pkt[i+7]*256)*(180 - (-180))/(720-0)+-180-base_angle[2];
					exit=1;
					printf("%f %f %f \n",current_angle[0],current_angle[1],current_angle[2]);
				}
			}
	
		}
		
		try++;
	}
	ftdi_usb_purge_buffers(&imu);
	if(exit!=1)				
	printf("+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++DATA NOT RECIEVED\n");
}
*/
//-----------------------------------------------------------imu functions end----------------------------------------------------------------



int iwrite(int choose, int *termios_pointer, struct ftdi_context *ftdi_pointer, byte *packet, int length)
{
	/*int i=0;
	
	f=fopen("check","a");
	if(f!=NULL)
	{
		for(i=0;i<length;i++)
		{
			printf("%x\t",packet[i]);
			printf("\n");
			fprintf(f,"%02x",packet[i]);
			//fprintf(f,"\n");
		}
		fclose(f);
	}
	*/
	switch(choose)
	{
		case 0: return ftdi_write_data(ftdi_pointer,packet,length);
			break;
		case 1 :return write(*termios_pointer,packet,length);
			break;
		default : return -99999;
	}
}

int iread(int choose, int *termios_pointer, struct ftdi_context *ftdi_pointer, byte *packet, int length)
{
	switch(choose)
	{
		case 0: return ftdi_read_data(ftdi_pointer,packet,length);
			break;
		case 1 :return read(*termios_pointer,packet,length);
			break;
		default : return -99999;
	}
}

int ipurge(int choose,int *termios_pointer, struct ftdi_context *ftdi_pointer)
{
	switch(choose)
	{
		case 0 : return ftdi_usb_purge_buffers(ftdi_pointer);
			break;
		case 1 : return tcflush(*termios_pointer,TCIOFLUSH);
			break;
		default : return -99999;
	}
}


float scurve2(float in,float fi,float t, float tot)
{
	float frac=t/tot;
	float ret_frac=6*pow(frac,5)-15*pow(frac,4)+10*pow(frac,3);
	return in+(fi-in)*ret_frac;
}
float scurve(float in,float fi,float t, float tot)
{
	float frac=(float)t/(float)tot;
	float retfrac=frac*frac*(3-2*frac);
	float mval=in + (fi-in)*retfrac;
	return mval;
}

float linear(float in,float fi,float t, float tot)
{
	return in*(1-t/tot)+fi*(t/tot);
}


int main()
{
	bootup_files();
	byte data;
	while(1)	
	{
		if(iread(IMU_DATA,&imu_t,&imu,&data,1)>0)
		printf("%d\n",data);
		// else
		// printf("NO DATA\n");
	}
	//imu_initialize();
	//while(!kbhit())
	//get_imu_data();
	
	close_files();
}
