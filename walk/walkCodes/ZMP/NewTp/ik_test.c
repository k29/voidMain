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

#define MASS (5.733)
#define PLOT 1

#define MOTOR_DATA 0
#define IMU_DATA 0
#define FT4232 "FTSBY5CA"//"FTSC152E"//"FTSC152E"
#define NOM 28                                  //Current number of Motors
#define LEN_SYNC_WRITE ((NOM*3)+4) 		//Total Lenght Of Packet
#define INST_SYNC_WRITE     0x83
#define SMOOTHSTEP(x) ((x) * (x)*(3-2*(x)))
#define max( a, b ) ( ((a) > (b)) ? (a) : (b) )
#define min( a, b ) ( ((a) < (b)) ? (a) : (b) )
#define D2R (3.14/180)
#define acyut 4  //acyut using...........................................change here..............
#define E 2.7183
#define MAX_BOUNDARIES 160
#define TRUE 1
#define serialbluetooth "A8008c3o"    //"A6007WeE"  
#define serialusb2d "A700eSSZ" //"A900fDpz"//"A900fDhp" //"A600cBa7" //"A600cURy"  //"A900fC5U"//nd USB2D: A7003N1d  A7003NDp	 A7005LC8 //A700eST2
#define serialimu "A8004Yt8" //"A8008c3o" 
long time_start, time_end,walk_prev_frame,walk_curr_frame;
struct timeval tv;
 
typedef uint8_t byte;
byte ignore_ids[]={10,15,16,17,18,30};  // enter ids to ignore :)
float q[5];
float mval[5];
	
float output[20];

struct ftdi_context ftdic1,imu;
int fd,imu_t;

byte tx_packet[128]={0};
byte last_tx_packet[128]={0};

	

typedef struct
{
	float mat[4][4];
}matrix;

typedef struct
{
	matrix rotmat;
	float a;
	float d;
	float theta;
	float alpha;
}fk_frame;

	
////////////////////////////////////////////////////////////////////
///////////////////// FORWARD KINEMATICS, INVERSE KINEMATICS , COM GENERATION , ZMP CALCULATION DEFINITIONS
float left_joints[7][3],right_joints[7][3];
float left_com[3]={0},right_com[3]={0};
float com[3]={0},rot_com[3]={0},rot_hip[3]={0},rot_rleg[3],hip[3]={0};
int offset[33]={0};
int motor_val[33]={0};
int target_left[6],target_right[6];
float STEP=10;
	
/////////////////////////////////////////////////////////////////////
float* generateik(float x,float y,float z,int i );
float scurve(float in,float fi,float t, float tot);
void initialize_acyut();

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



int get_lb( int word )
{
  unsigned short temp;
  temp = word & 0xff;
  return (int)temp;
}

int get_hb( int word )
{
  unsigned short temp;
  temp = word & 0xff00;
  temp = temp >> 8;
  return (int)temp;
}
void print_matrix(matrix a)
{	
	int i=0,j=0;
	for(i=0;i<4;i++)
	{
		for(j=0;j<4;j++)
		{
			printf("%lf\t",a.mat[i][j]);
		}
		printf("\n");
	}
}
	
matrix multiply(matrix a, matrix b)
{
	matrix ans;
	int i=0,j=0;
	
	for(i=0;i<4;i++)
	{
		for(j=0;j<4;j++)
		{
			ans.mat[i][j]=a.mat[i][0]*b.mat[0][j]+a.mat[i][1]*b.mat[1][j]+a.mat[i][2]*b.mat[2][j]+a.mat[i][3]*b.mat[3][j];
		}
	}
	
	return ans;
}

void identity_mat(matrix *a)
{
	int i,j;
	for(i=0;i<4;i++)
	{
		for(j=0;j<4;j++)
		{
			if(i==j)
			a->mat[i][j]=1;
			else
			a->mat[i][j]=0;
		}
	}
}
	
void form_matrix(fk_frame *a)
{
	a->rotmat.mat[0][0]=cos(a->theta*3.14/180);
	a->rotmat.mat[0][1]=-sin(a->theta*3.14/180)*cos(a->alpha*3.14/180);
	a->rotmat.mat[0][2]=sin(a->theta*3.14/180)*sin(a->alpha*3.14/180);
	a->rotmat.mat[0][3]=a->a*cos(a->theta*3.14/180);
	
	a->rotmat.mat[1][0]=sin(a->theta*3.14/180);
	a->rotmat.mat[1][1]=cos(a->theta*3.14/180)*cos(a->alpha*3.14/180);
	a->rotmat.mat[1][2]=-cos(a->theta*3.14/180)*sin(a->alpha*3.14/180);
	a->rotmat.mat[1][3]=a->a*sin(a->theta*3.14/180);
	
	a->rotmat.mat[2][0]=0;
	a->rotmat.mat[2][1]=sin(a->alpha*3.14/180);
	a->rotmat.mat[2][2]=cos(a->alpha*3.14/180);
	a->rotmat.mat[2][3]=a->d;

	a->rotmat.mat[3][0]=0;
	a->rotmat.mat[3][1]=0;
	a->rotmat.mat[3][2]=0;
	a->rotmat.mat[3][3]=1;
	
}

inline int read_position(int mot_no)
{
	//printf("READING MOTOR NO. %d\n",mot_no);
	
	//gettimeofday(&tv,NULL); gets the current system time into variable tv  */
	//time_start  = (tv.tv_sec *1e+6) + tv.tv_usec;

	int try=0;
	int noat=0;
	int start=0;
	int exit=0;
	int data_not_rcv=1;
	int i=0;
	byte read1;
	byte snd_packet[8];
	byte rcv_packet[6];
	byte chksum=0xff;

	snd_packet[0]=0xff;
	snd_packet[1]=0xff;
	snd_packet[2]=mot_no;
	snd_packet[3]=0x04;
	snd_packet[4]=0x02;
	snd_packet[5]=0x24;
	snd_packet[6]=0x02;
	snd_packet[7]=0xff;

	for(i=2;i<7;i++)
	snd_packet[7]-=snd_packet[i];
	
	for(try=0;try<10;try++)
	{
		start=0;
		noat=0;
		//if(ftdi_write_data(&ftdic1,snd_packet,8)==8)
		if(iwrite(MOTOR_DATA,&fd,&ftdic1,snd_packet,8)>=0)
		{
			//gettimeofday (&tv, NULL);
			//time_end = (tv.tv_sec *1e+6) + tv.tv_usec;
			data_not_rcv=1;
			//printf("SEND TIME : %ld\n",-time_start+time_end);
			//if(ftdi_setrts(&ftdic1,1)==0)
			//printf("RTS SET 1\n");
						
			while(noat<(MOTOR_DATA==0?400:4000)&&data_not_rcv==1)
			{	
				//if(ftdi_read_data(&ftdic1,&read,1)==1&&read==0xff)
				if(iread(MOTOR_DATA,&fd,&ftdic1,&read1,1)>=0&&read1==0xff)
				start++;
				else
				{
					start=0;
					noat++;
				}
				
				if(start==2)
				{
					start=0;
					data_not_rcv==0;
					//if(ftdi_read_data(&ftdic1,rcv_packet,6)>0)
					if(iread(MOTOR_DATA,&fd,&ftdic1,rcv_packet,6)>0)
					{
						chksum=0xff;						
						for(i=0;i<5;i++)
						{
							printf("%d\t",rcv_packet[i]);
							chksum-=rcv_packet[i];
						}			
						printf("CHKSUM %d C %d\n",rcv_packet[i],chksum);			
						if(chksum==rcv_packet[5]&&rcv_packet[0]==mot_no)
						{	
							//gettimeofday (&tv, NULL);
							//time_end = (tv.tv_sec *1e+6) + tv.tv_usec;
							//if(ftdi_setrts(&ftdic1,0)==0)
							//printf("RTS SET 0\n");
							//printf("MOTOR READ %d\n TIME:%ld\n",noat,time_end-time_start);
							return rcv_packet[3]+rcv_packet[4]*256;
						}
						else
						noat++;
					}
				}
			}		
		}
	}
	//if(ftdi_setrts(&ftdic1,0)==0)
	//printf("RTS SET 0\n");
	//gettimeofday (&tv, NULL);
	//time_end = (tv.tv_sec *1e+6) + tv.tv_usec;
																			
	//printf("NOT READ\t %ld\n",time_end-time_start);
	return 9999;					
					
}

int forward_kinematics(int leg,int mval[6])   
{
	
	float hip_length=100;
	int i =0;
	float cnst[6];
	float coods[7][3]={0};
	cnst[0]=1675;	
	cnst[1]=3207;
	cnst[2]=1752;
	cnst[3]=1714;
	if(leg==0)
	cnst[4]=742;
	else if(leg==1)
	cnst[4]=274;
	
	cnst[5]=1703;

	for(i=0;i<6;i++)
	{
		//printf("%d\t%d\t",i,mval[i]);
		if(i==4)
		mval[i]=(mval[i]-cnst[i])*300/1024;   /// MOTOR 4 is RX64 ..... 
		else
		mval[i]=(mval[i]-cnst[i])*300/4096;
		//printf("%f\n",mval[i]);
	}
	fk_frame frames[7];
	matrix base_frame;
	identity_mat(&base_frame);
	
	frames[0].a=0;
	frames[0].d=0;
	frames[0].alpha=90;
	frames[0].theta=-90+pow(-1,leg)*mval[5];
	
	frames[1].a=0;
	frames[1].d=50;
	frames[1].alpha=90;
	frames[1].theta=-90-mval[4];
	
	frames[2].a=180;
	frames[2].d=0;
	frames[2].alpha=0;
	frames[2].theta=90-mval[3];
	
	frames[3].a=50;
	frames[3].d=0;
	frames[3].alpha=0;
	frames[3].theta=-mval[2];
	
	frames[4].a=180;
	frames[4].d=0;
	frames[4].alpha=0;
	frames[4].theta=-mval[2];
	
	frames[5].a=25;
	frames[5].d=0;
	frames[5].alpha=90;
	frames[5].theta=-mval[1];
	
	for(i=0;i<6;i++)
	{
		form_matrix(&frames[i]);
		//print_matrix(frames[i].rotmat);
		//printf("\n\n\n\n");
		base_frame=multiply(base_frame,frames[i].rotmat);
		coods[i+1][0]=base_frame.mat[0][3];
		coods[i+1][1]=base_frame.mat[1][3];
		coods[i+1][2]=base_frame.mat[2][3];
		//printf("%f\t%f\t%f\n",coods[i][0],coods[i][1],coods[i][2]);
		//print_matrix(base_frame);
	
	}
	
	if(leg==0)
	{
		for(i=0;i<7;i++)
		{
			left_joints[i][0]=coods[i][0];
			left_joints[i][1]=coods[i][1]-hip_length/2;   
			left_joints[i][2]=coods[i][2];
			printf("LEFT  %f\t%f\t%f\n",left_joints[i][0],left_joints[i][1],left_joints[i][2]);
		}	
		//i=6;
		//printf("LEFT  %f\t%f\t%f\n",left_joints[i][0],left_joints[i][1],left_joints[i][2]);
	
	}	
	else if(leg==1)
	{
		for(i=0;i<7;i++)
		{
			right_joints[i][0]=coods[i][0];
			right_joints[i][1]=coods[i][1]+hip_length/2;
			right_joints[i][2]=coods[i][2];
			printf("RIGHT %f\t%f\t%f\n",right_joints[i][0],right_joints[i][1],right_joints[i][2]);
		}
		//i=6;	
		//printf("RIGHT %f\t%f\t%f\n",right_joints[i][0],right_joints[i][1],right_joints[i][2]);
	
	}
	//print_matrix(base_frame);
	return EXIT_SUCCESS;
}

float* generate_leg_com(int leg)
{	
	int i=0,j=0;
	
	float coods[7][3];
	float leg_com[3];
	float motor_mass=150;
	float tot_mass=0;	

	float bracket_mass[6];
	bracket_mass[0]=0;  /// 5-4 bracket
	bracket_mass[1]=0;  /// 4-3 bracket
	bracket_mass[2]=112;   /// 3-12 bracket
	bracket_mass[3]=0;   /// 12-2 bracket
	bracket_mass[4]=112;   /// 2-1 bracket
	bracket_mass[5]=0;   /// 1-0 bracket
	
	if(leg==0)
	{
		for(i=0;i<7;i++)
		{
			coods[i][0]=left_joints[i][0];
			coods[i][1]=left_joints[i][1];
			coods[i][2]=left_joints[i][2];
		}	
	}	
	else if(leg==1)
	{
		for(i=0;i<7;i++)
		{
			coods[i][0]=right_joints[i][0];
			coods[i][1]=right_joints[i][1];
			coods[i][2]=right_joints[i][2];
		}	
	}		
		
	///////////// MOTORS 
	for(i=0;i<7;i++)
	{
		for(j=0;j<3;j++)
		leg_com[j]+=coods[i][j]*motor_mass;
		
		tot_mass+=motor_mass;
	}
	
	//////////// BRACKETS
	for(i=0;i<6;i++)
	{
		for(j=0;j<3;j++)
		leg_com[j]+=(coods[i][j]+coods[i+1][j])/2*bracket_mass[i];
		
		tot_mass+=bracket_mass[i];
	}
	
	//////// FEET TP HERE ////////////
		

	for(j=0;j<3;j++)
	leg_com[j]=leg_com[j]/tot_mass;
	//leg==1?printf("RIGHT "):printf("LEFT  ");
	//printf("COM X:%lf\tY:%lf\tZ:%lf\n",leg_com[0],leg_com[1],leg_com[2]);	
	if(leg==0)
	{
		left_com[0]=leg_com[0];
		left_com[1]=leg_com[1];
		left_com[2]=leg_com[2];
	}	
	else if(leg==1)
	{
		right_com[0]=leg_com[0];
		right_com[1]=leg_com[1];
		right_com[2]=leg_com[2];
	}	
	return EXIT_SUCCESS;
}

int generate_com()
{
	generate_leg_com(0);
	generate_leg_com(1);

	// LEFT LEG CONTIBUTION
	
	float l_wt=1499;
	com[0]=(left_com[0]+0)*l_wt;
	com[1]=(left_com[1]+0)*l_wt;
	com[2]=(left_com[2]+0)*l_wt;
	
	// RIGHT LEG CONTRIBUTION	
	
	float r_wt=1499;
	com[0]+=(right_com[0]+0)*r_wt;
	com[1]+=(right_com[1]+0)*r_wt;
	com[2]+=(right_com[2]+0)*r_wt;
	
	// TORSO CONTRIBUTION
	float torso_wt=2734.9+300;
	com[0]+=121.358*torso_wt;
	com[1]+=0*torso_wt;
	com[2]+=0*torso_wt;

	com[0]=com[0]/(l_wt+r_wt+torso_wt);
	com[1]=com[1]/(l_wt+r_wt+torso_wt);
	com[2]=com[2]/(l_wt+r_wt+torso_wt);
	//printf(" COM X:%lf\tY:%lf\tZ:%lf\t",com[0],com[1],com[2]);
	return EXIT_SUCCESS;
}

int  projection_system(int leg)
{
	gettimeofday (&tv, NULL);
	time_start = (tv.tv_sec *1e+6) + tv.tv_usec;

	int val[6];
	int i=0;
	int mot_no[]={0,1,2,3,4,5};
	int flag=0;	
	for(i=0;i<6;i++)
	{
		printf("READING MOTOR NO. %d\n",mot_no[i]);
		val[i]=read_position(mot_no[i]);
		if(val[i]==9999)
		{
			printf("USING TARGET VALUE %d\n",mot_no[i]);
			val[i]=target_left[i];
		}
		else
		{
			val[i]-=offset[mot_no[i]];
			printf("MOT : %d TARGET : %d ACTUAL %d\n",mot_no[i],target_left[i],val[i]);
		}
		
		
	}
	
	forward_kinematics(0,val);
	for(i=0;i<6;i++)
	mot_no[i]+=20;
	
	for(i=0;i<6;i++)
	{
		printf("READING MOTOR NO. %d\t",mot_no[i]);
		val[i]=read_position(mot_no[i]);
		if(val[i]==9999)
		{
			val[i]=target_right[i];
			printf("USING TARGET VALUE %d\n",mot_no[i]);
		}
		else
		{
			val[i]-=offset[mot_no[i]];
			printf("MOT : %d TARGET : %d ACTUAL %d\n",mot_no[i],target_right[i],val[i]);
		}
	}
	forward_kinematics(1,val);
}
	
void send_sync()
{ 
	//ftdi_write_data(&ftdic1,tx_packet, tx_packet[3]+4);
	iwrite(MOTOR_DATA,&fd,&ftdic1,tx_packet,tx_packet[3]+4);
}



void close_files()
{
	int ret_motor,ret_imu;
	if(MOTOR_DATA==0)
	{
		if((ret_motor=ftdi_usb_close(&ftdic1)) < 0)
		{
			fprintf(stderr, "unable to close usb2d ftdi device: %d (%s)\n", ret_motor, ftdi_get_error_string(&ftdic1));
    		}
		ftdi_deinit(&ftdic1);
//	free(&ftdic1);
	}
	else if (MOTOR_DATA==1)
	{
		close(fd);
	}

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

uint8_t bid[30];

void initialize_ids(void)  
{
	uint8_t initialize_ids_i;
	for(initialize_ids_i=0;initialize_ids_i<((NOM-4)/2)-1;initialize_ids_i++)
	{
		bid[initialize_ids_i] = initialize_ids_i;
	}
	bid[((NOM-4)/2)-1] = 15;
	bid[((NOM-4)/2)] = 16;
	bid[((NOM-4)/2)+1] = 17;
	bid[((NOM-4)/2)+2] = 18;
	

	for(initialize_ids_i=((NOM+4)/2)-1;initialize_ids_i<NOM;initialize_ids_i++)   
	{
		bid[initialize_ids_i] = initialize_ids_i+5;
	}
	bid[26]=12;
	bid[27]=32;
	//for(initialize_ids_i=0;initialize_ids_i<30;initialize_ids_i++)
	//printf("%d\n",bid[initialize_ids_i]);
}
	
void set_offset()
{
	if(acyut==4)
	{
		offset[0]=15;
		offset[1]=350-25; 
		offset[2]=275; 
		offset[3]=232-20-25-25-10;    // 142s  
		offset[4]=0; 
		offset[5]=-15;//+7; 
		offset[6]=346-40; 
		offset[7]=0; 
		offset[8]=0; 
		offset[9]=0-100;//-(1023-865);//-150; 
		offset[10]=0; 	 
		offset[11]=0; 
		offset[12]=254; 
		offset[13]=0; 
		offset[14]=0; 
		offset[15]=-119; 
		offset[16]=42; 
		offset[17]=0; 
		offset[18]=0; 
		offset[19]=0; 
		offset[20]=-6;
		offset[21]=334+15-25; 
		offset[22]=288; 
		offset[23]=248-25-25-10;//-20; 
		offset[24]=-10; 
		offset[25]=-15;//-7; 
		offset[26]=-306; 
		offset[27]= 0; 
		offset[28]=0;// 90*1023/300; 
		offset[29]=80;//s1023-865; 
		offset[30]=0;
		offset[31]=0;
		offset[32]=258;
	}
	else
	{
		offset[0]=-4;
		offset[1]=120;
		offset[2]=304;
		offset[3]=341;
		offset[4]=0;
		offset[5]=0;
		offset[6]=300;
		offset[7]=0;
		offset[8]=0;
		offset[9]=0;
		offset[10]=0;
		offset[11]=0;
    		offset[12]=254;
		offset[13]=0;
		offset[14]=0;
		offset[15]=-119;
		offset[16]=0;
		offset[17]=0;
		offset[18]=0;
		offset[19]=0;
		offset[20]=-19;
		offset[21]=321;
		offset[22]=304;
		offset[23]=341;
		offset[24]=0;
		offset[25]=0;
		offset[26]=-279;
		offset[27]= 0;
		offset[28]= 0;
		offset[29]=0;
		offset[30]=0;
		offset[31]=0;
		offset[32]=254;
	}
}	

int bootup_files()
{
	initialize_ids();
	int chk_i_ids;
	int ret_motor,ret_imu;
	/*for (chk_i_ids = 0; chk_i_ids < 24; chk_i_ids += 1)
	{
		printf("bid[%d]= %d\n",chk_i_ids,bid[chk_i_ids]);
	}*/
	
	set_offset();
	if(MOTOR_DATA==0)
	{
		if (ftdi_init(&ftdic1) < 0)
    		{	
		        fprintf(stderr, "ftdi_init failed\n");
			ret_motor=-99;
        		//return EXIT_FAILURE;
    		}		
 		//ftdi_set_interface(&ftdic1,0);   
   		if ((ret_motor = ftdi_usb_open_desc (&ftdic1, 0x0403, 0x6001, NULL, serialusb2d)) < 0)
    		{
        		fprintf(stderr, "unable to open ftdi device: %d (%s)\n", ret_motor, ftdi_get_error_string(&ftdic1));
        		printf("unable to open USB2D\n");
        		//return EXIT_FAILURE;
    		}
    		else
    		{
    			printf("USB2D ftdi successfully open\n");
			ftdi_set_baudrate(&ftdic1,1000000);
    		}	
	}
	else if(MOTOR_DATA==1)
	{
		struct termios options;
		struct timeval start, end;
		long mtime, seconds, useconds;
		if((fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY))==-1)
		{	
			ret_motor=-1;
			printf("Serial Port unavailable\n");
			close_files();
			exit(0);
		}
		else
		printf("Motor Line Opened By TERMIOS\n");
 		tcgetattr(fd, &options);
  		cfsetispeed(&options, B1000000);
  		cfsetospeed(&options, B1000000);
  		options.c_cflag |= (CLOCAL | CREAD);
  		options.c_cflag &= ~PARENB;
  		options.c_cflag &= ~CSTOPB;
     		options.c_cflag &= ~CSIZE;
     		options.c_cflag |= CS8;
     		options.c_oflag &= ~OPOST;
     		options.c_oflag &= ~ONOCR;
     		options.c_oflag |= FF0;
     		options.c_oflag &= ~FF1;
     		options.c_oflag &= ~ONLCR;
	
		options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);    
    		options.c_cflag &= ~CRTSCTS;
//		tcflush(fd, TCIFLUSH);
  		tcsetattr(fd, TCSANOW, &options);
  		fcntl(fd, F_SETFL, FNDELAY);
	}
	else
	printf("UNKNOWN MOTOR CONNECTION\n");
	
		
	if(IMU_DATA==0)
	{
		if (ftdi_init(&imu) < 0)
    		{
	        	fprintf(stderr, "ftdi_init failed\n");
        		ret_imu=-99;
        		//return EXIT_FAILURE;
    		}	
   		ftdi_set_interface(&imu,2);
  		if ((ret_imu= ftdi_usb_open_desc (&imu, 0x0403, 0x6011, NULL, FT4232)) < 0)
    		{
        		fprintf(stderr, "unable to open ftdi device: %d (%s)\n", ret_imu, ftdi_get_error_string(&imu));
        		printf("unable to open IMU\n");
       			 //return EXIT_FAILURE;
    		}	
    		else
    		{
    			printf("IMU ftdi successfully open\n");
	   		ftdi_set_baudrate(&imu,57600);
    		}
	}   
    	else if (IMU_DATA==1)
	{
		struct termios imu_options;
		if((imu_t=open("/dev/ttyUSB3", O_RDWR | O_NOCTTY | O_NDELAY))==-1)
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
     		imu_options.c_oflag |= ONOCR;
     		imu_options.c_oflag |= FF0;
     		imu_options.c_oflag &= ~FF1;
     		imu_options.c_oflag &= ~ONLCR;
	
		imu_options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);    
    		imu_options.c_cflag &= ~CRTSCTS;
//		tcflush(fd, TCIFLUSH);
  		tcsetattr(imu_t, TCSANOW, &imu_options);
  		fcntl(imu_t, F_SETFL,0);
		
	}
	else
	printf("UNKNOWN IMU CONNECTION\n");
	
	if(ret_motor>=0&&ret_imu>=0)
	return EXIT_SUCCESS;	
	else
	return EXIT_FAILURE;

}

int checksum(void)
{
  int bchecksum=0x00;
  int checksum_i=0;
  for(checksum_i = 2;checksum_i<=(tx_packet[3]+2);checksum_i++) //except 0xff,checksum
  {
    bchecksum = bchecksum + tx_packet[checksum_i];
	
  }
  bchecksum=bchecksum%256;
  //  bchecksum = bchecksum & 0x11;	
  //  bchecksum = ~bchecksum;
  //  bchecksum=bchecksum & 0x11;
  bchecksum= 255 - bchecksum;

  return bchecksum;

}


int checksum_last(void)
{
  int bchecksum=0x00;
  int checksum_i=0;
  for(checksum_i = 2;checksum_i<=(last_tx_packet[3]+2);checksum_i++) //except 0xff,checksum
  {
    bchecksum = bchecksum + last_tx_packet[checksum_i];
	
  }
  bchecksum=bchecksum%256;
  //  bchecksum = bchecksum & 0x11;	
  //  bchecksum = ~bchecksum;
  //  bchecksum=bchecksum & 0x11;
  bchecksum= 255 - bchecksum;

  return bchecksum;

}


void speed(int a)
{
	byte speed_packet[128];
	speed_packet[0]=0xff;
	speed_packet[1]=0xff;
	speed_packet[2]=0xfe;
	speed_packet[3]=NOM*3+4;
	speed_packet[4]=0x83;
	speed_packet[5]=0x20;
	speed_packet[6]=0x02;
	speed_packet[NOM*3+7]=0xff;
	int sync_write_i,i;
	switch(a)
	{
		case 0:
			for(sync_write_i=0;sync_write_i<NOM;sync_write_i++)
		    {
		      speed_packet[7+(sync_write_i*3)]= bid[sync_write_i];
		      speed_packet[8+(sync_write_i*3)]=0x20;
		      speed_packet[9+(sync_write_i*3)]=0x00;
		    }
					
		    for(i=2;i<NOM*3+7;i++)
			speed_packet[3*NOM+7]-=speed_packet[i];

		    //if(ftdi_write_data(&ftdic1,speed_packet,speed_packet[3]+4)<0)
		    if(iwrite(MOTOR_DATA,&fd,&ftdic1,speed_packet,speed_packet[3]+4)<0)
		    printf("Speed Not Written\n");
			break;
		case 1:
			for(sync_write_i=0;sync_write_i<NOM;sync_write_i++)
		    {	
		      speed_packet[7+(sync_write_i*3)]= bid[sync_write_i];
		      speed_packet[8+(sync_write_i*3)]=0x00;
		      speed_packet[9+(sync_write_i*3)]=0x00;
		    }
					
		    for(i=2;i<NOM*3+7;i++)
			speed_packet[3*NOM+7]-=speed_packet[i];

		    if(iwrite(MOTOR_DATA,&fd,&ftdic1,speed_packet,speed_packet[3]+4)<0)
		    printf("Speed Not Written\n");
			break;
	}
	//tx_packet[128]={0xff,0xff,0xfe,LEN_SYNC_WRITE,INST_SYNC_WRITE,0x1e,0x02};
	//tx_packet[2]=0xfe;
	//tx_packet[3]=LEN_SYNC_WRITE;
	//tx_packet[4]=INST_SYNC_WRITE;
	//tx_packet[5]=0x1e;
	//tx_packet[6]=0x02;
}


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

int high_punch=150;
int low_punch=0;

int compliance(int leg,int cp,int cpr)
{
	int mot,revmot,h=0;
	if(leg==0)
	{
		mot=0;
		revmot=20;
	}
	else if(leg==1)
	{
		mot=20;
		revmot=0;
	}
	else
	{
		printf("TAKE LITE\n");
		return EXIT_FAILURE;
	}
	
	byte compliance_packet[]={0xff,0xff,0xfe,0x28,0x83,0x1c,0x02,mot,cp,cp,mot+1,cp,cp,mot+2,cp,cp,mot+12,cp,cp,mot+3,cp,cp,mot+5,cp,cp,revmot,cpr,cpr,revmot+1,cpr,cpr,revmot+2,cpr,cpr,revmot+12,cpr,cpr,revmot+3,cpr,cpr,revmot+5,cpr,cpr,0xff};
	for(h=2;h<43;h++)
	compliance_packet[43]-=compliance_packet[h];

	//if(ftdi_write_data(&ftdic1,compliance_packet,compliance_packet[3]+4)>0)
	if(iwrite(MOTOR_DATA,&fd,&ftdic1,compliance_packet,compliance_packet[3]+4)<0)
	//printf("Compliance Written\n");
	//else
	{
		//printf("ERROR : Compliance\n");
	}
}




float* generateik(float x, float y, float z,int leg)
{
	//printf("%lf %lf %lf\n",x,y,z);
	int i=0;
	float cons[10];
	float ang[6];
	float theta[4];
	float phi[5];
	float b_len=180;   // Length of bracket 
	float l22=49;		// Distance between bracket ends
	float r2d=180/3.14159265358979323846264338327950288419716939937510;
	float leg_l=pow(x*x+y*y+z*z,0.5);
	
	if(leg_l>2*b_len+l22)
	{	
		return mval;
		printf("IK EXCEED\n");
	}
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

float scurve2(float in,float fi,float t, float tot)
{
	float frac=t/tot;
	float ret_frac=6*pow(frac,5)-15*pow(frac,4)+10*pow(frac,3);
	return in+(fi-in)*ret_frac;
}
float scurve(float in,float fi,float t, float tot)
{
	float frac=(float)t/(float)tot;
	if(frac>1)
	frac=1;
	float retfrac=frac*frac*(3-2*frac);
	float mval=in + (fi-in)*retfrac;
	return mval;
}

float linear(float in,float fi,float t, float tot)
{
	return in*(1-t/tot)+fi*(t/tot);
}

float rotation(int mot,float phi)
{
	 float motval;
	 if(mot==4)
	 motval=742-phi*1023/300;   /// 742 , 274 are initialize values of motors 4 , 24
	 else if(mot==24)
	 motval=274+phi*1023/300;
	 else
	 motval=sqrt(-1);
	 
	 return motval;
}




int generatePacket()
{
	int packet_i=0;
	int adchksum=NOM*3+7;
	tx_packet[0]=0xff;
	tx_packet[1]=0xff;
	tx_packet[2]=0xfe;
	tx_packet[3]=NOM*3+4;
	tx_packet[4]=INST_SYNC_WRITE;
	tx_packet[5]=0x1e;
	tx_packet[6]=0x02;
	tx_packet[adchksum]=0xff;
	for(packet_i=0;packet_i<NOM;packet_i++)
	{
		tx_packet[3*packet_i+7]=bid[packet_i];
		tx_packet[3*packet_i+8]=get_lb(motor_val[bid[packet_i]]);
		tx_packet[3*packet_i+9]=get_hb(motor_val[bid[packet_i]]);
		//printf(" %d %d\n",bid[packet_i],motor_val[bid[packet_i]]);
	}
	
	for(packet_i=2;packet_i<adchksum;packet_i++)
	tx_packet[adchksum]-=tx_packet[packet_i];
	
	return 1;
	
}

int pingtest()
{
	int noat=0;
	//printf("pinging....\n");
	int failure[NOM][2];
	byte txp[15]={0xff,0Xff,0,0X02,0X01,0},rxp[4],chkff,count=2,exit=0,chksum,noat2=0;
	int er=0,ping_test_k=0,i,j,z,igm=0;
	//ftdi_usb_purge_buffers(&ftdic1);
	ipurge(MOTOR_DATA,&fd,&ftdic1);
	for (i = 0; i < NOM; i++)
	{
		count=2;
		//printf("in loop\n");
		//printf("i =  %d\n",i);
		//printf("\n");
		txp[2]=bid[i];
		failure[i][0]=bid[i];
		failure[i][1]=0;
		//printf("bid = %d , i =%d\n",bid[i],i);
		igm=0;
		exit=0;
		chksum=0x03+txp[2];
		chksum=~chksum;
		txp[5]=chksum;
		printf("Checking Motor ID : %d\n",txp[2]);
		for (ping_test_k = 0; ping_test_k < sizeof(ignore_ids)/sizeof(byte); ping_test_k += 1)
		{
			if(txp[2]==ignore_ids[ping_test_k])
			{
				igm=1;
				printf("*****Motor Pinging Ineffectual %d*****\n",ignore_ids[ping_test_k]);
				er+=ignore_ids[ping_test_k];
				break;
			}
		}
		for(noat=0;noat<(MOTOR_DATA==0?4:4000);noat++)
		{
			//printf("NOAT %d \n",noat);
			//ftdi_write_data(&ftdic1,txp,txp[3]+4);
			iwrite(MOTOR_DATA,&fd,&ftdic1,txp,txp[3]+4);
			noat2=0;
			while(noat2<5)
			{
				noat2++;
				//printf("COUNT:%d \n",noat2);
				//if((ftdi_read_data(&ftdic1,&chkff,1)>0)&&chkff==0xff)
				if((iread(MOTOR_DATA,&fd,&ftdic1,&chkff,1)>0)&&chkff==0xff)
				count--;
				else
				count=2;
				//printf("pack: %d\n",chkff);
				
				if(count==0)
				{
					//ftdi_read_data(&ftdic1,rxp,4);
					iread(MOTOR_DATA,&fd,&ftdic1,rxp,4);
					count=2;
					printf("DATA READ : %d %d %d %d\n",rxp[0],rxp[1],rxp[2],rxp[3]);
					chksum=0;
					for(z=0;z<3;z++)
					chksum+=rxp[z];
					chksum=~chksum;
					if(chksum!=rxp[3])
					{
						printf("ERROR: Packet checksum error\n");
						if(igm==1)
						{
							//er+=txp[2];
							exit=1;
						}
					}
					else if(rxp[0]!=txp[2])
					{
						printf("ERROR: Error Pinging Motor\n");
						if(igm==1)
						{
							//er+=txp[2];
							exit=1;
						}
					}		
					else if(rxp[2]!=0)
					{
						printf("ERROR: Motor Error :%d\n",rxp[2]);
						if(igm==1)
						{
							//er+=txp[2];
							exit=1;
						}
					}
					else
					{
						printf("SUCCESS PINGING MOTOR : %d\n",rxp[0]);
						if(igm==0)
						er+=rxp[0];
						exit=1;
						failure[i][1]=1;
						
					}
					//printf("Er:%d\n",er);
					break;
				}
				
			}
			//printf("ER %d\n",er);
			if(exit==1)
			break;
		}
		printf("\n\n");
	}
	printf("\n\nMOTORS NOT RESPONDING\n");
	for(i=0;i<NOM;i++)
	{
		//printf("i=%d",i);
		if(failure[i][1]==0)
		{
			printf("%d",failure[i][0]);
			for (ping_test_k = 0; ping_test_k < sizeof(ignore_ids)/sizeof(byte); ping_test_k += 1)
			{
				if(failure[i][0]==ignore_ids[ping_test_k])
				printf(" <- IGNORED");
			}
			printf("\n");
		}
	}
	return ((er==440)?1:0);
	

}

void initialize_acyut()
{
	int h=0;
	float val[5];
	float *fval;
	int value;

	fval=generateik(390,0,0,0);
	for(h=0;h<5;h++)
	val[h]=fval[h];

	motor_val[0]=val[0]+offset[0];
	motor_val[1]=val[1]+offset[1];
	motor_val[2]=val[2]+offset[2];
	motor_val[3]=val[3]+offset[3];
	motor_val[4]=rotation(4,0)+offset[4];
	motor_val[5]=val[4]+offset[5];
	motor_val[6]=512+offset[6];
	motor_val[7]=698+offset[7];
	motor_val[8]=512+offset[8];
	motor_val[9]=150+offset[9];
	motor_val[10]=823+offset[10];
	motor_val[12]=val[2]+offset[12];
	
	motor_val[15]=1863+offset[15];
	motor_val[16]=2048+offset[16];
	motor_val[17]=546+offset[17];
	motor_val[18]=588+offset[18];
	
	fval=generateik(390,0,0,0);
	for(h=0;h<5;h++)
	val[h]=fval[h];

	motor_val[20]=val[0]+offset[20];
	motor_val[21]=val[1]+offset[21];
	motor_val[22]=val[2]+offset[22];
	motor_val[23]=val[3]+offset[23];
	motor_val[24]=rotation(24,0)+offset[24];
	motor_val[25]=val[4]+offset[25];
	motor_val[26]=512+offset[26];
	motor_val[27]=325+offset[27];
	motor_val[28]=512+offset[28];
	motor_val[29]=865+offset[29];
	motor_val[30]=200+offset[30];
	motor_val[32]=val