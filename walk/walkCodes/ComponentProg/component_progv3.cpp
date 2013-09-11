//TODO IMPLEMENT CENTRAL CONTROL
//TODO IMPLEMENT PHYSICS ENGINE
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
#define BOT_HEIGHT 390
#define PLOT 0
#define PI 3.1415926535
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
#define serialbluetooth "A8008c3o"    //"A6007WeE"  
#define serialusb2d "A900fDpz"//"A900fDhp" //"A600cBa7" //"A600cURy"  //"A900fC5U"//nd USB2D: A7003N1d  A7003NDp	 A7005LC8 //A700eST2
#define serialimu "A8004Yt8" //"A8008c3o" 

long int COUNTER=0;
long int COM_INC=0;
int COUNT;
int walk_count=0;
char comfile[50]="\0";
char leftfile[50]="\0";
char rightfile[50]="\0";
char expect_com[50]="\0";
char hipfile[50]="\0";
char normalrctn[50]="\0";
long time_start, time_end,walk_prev_frame,walk_curr_frame;
int NOFWALK=0;
struct timeval tv;
 
typedef uint8_t byte;
byte ignore_ids[]={10,15,16,17,18,30};  // enter ids to ignore :)
float q[5];
	
float output[20];

	
struct plot
{
	float x;
	float y;
}point;

struct acc
{
	float x,y,z;
	float acc;
}acc[5];

struct gyro
{
  float x,y,z;
  float gyro;
}gyro[5];

struct estimate
{
  float x,y,z;
 float est;
}est[5];

struct angle
{	
	int xy,yz,zx;
}ang[5]; 
 
struct boundary_array
{
	int nof;
	float x;	
 	float y;
} bounds[MAX_BOUNDARIES];

struct ftdi_context imu;

int flag_imu=1,temp_x;
 
FILE *f;
FILE *f1;
int fd,fb,imu_t;
struct ftdi_context ftdic1,ftdic2;  //ftdic1=usb2dyn   ftdic2=bluetooth
byte last_tx_packet[128]={0};
typedef struct 
{
	byte ID;
	byte LB;
	byte HB;
}motor_data;

typedef struct 
{
	byte num_motors;
	byte address;
	motor_data motorvalues[NOM];
	byte checksum;
}frame;

typedef struct
{
	byte header[2];
	int noframes;
	frame frames[720];
//	byte endbit[2];
}move;

//////Dhairya & Apoorv's GUI Comm defs//////
unsigned int length;
typedef struct
{
	double code1;
	byte code2;
	char remote_movename[30];
}remote_content;

typedef struct
{
	char file_name[30];
	int dlay;
}combine;

typedef struct
{
	int no_of_files;
	combine cmb_moves[256]; 
}combine_f;

typedef struct
{
	byte Id;
	byte sign;
	byte hB;
	byte lB;
}offsets;
offsets master[NOM];

typedef struct
{
	char rem_move_name[30];
	int init_h;
	int final_h;
	byte init_status;
	byte final_status;
}height_status;

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

typedef struct 
{
	float x;
	float y;
	float z;
}coordinates;

typedef struct
{
	long int nof;
	coordinates coods[6000];
	coordinates left[6000];
	coordinates right[6000];
}zmp_file;

typedef struct
{
	float magnitude;
	float frequency;
	float phase;
	float displacement;
	float start;
	float peak_start;
	float peak_end;
	float end;
	//float time;
}
curve;

typedef struct
{
	int no_curves;
	curve curves[10000];
}fourier;

zmp_file zmp;

///////////////////////////
//////////////////// WALKING DEFINITIONS///////////////////////////

typedef struct
{
	/////// X-Lift Control /////
	float x_start;
	float x_end;
	float lift;
	float height;
	float freq;
	float displacement;	
	float phase;
}x_lift;

typedef struct
{
	/////// X-Land Control /////
	//float dec_time=0.75;	
	float x_land_start;//1-x_end    
	float x_land_increase_end;
	float x_land_decrease_start;
	float x_land_end;
	float x_land_freq;
	float x_land_displacement;
	float x_land_phase;
	float x_land_dec;
	float x_land_base;
}x_land;

typedef struct 
{
	/////// X-Load Control /////
	float x_load_start;
	float x_load_increase_end;
	float x_load_decrease_start;
	float x_load_end;
	float x_load_displacement;//x_land_displacement;
	float x_load_phase;//-1.57;
	float x_load_freq;//3.14;
	float x_load_dec;
	//float x_load_window=0.05;
}x_load;

typedef struct
{
	x_lift lift;
	x_land land;
	x_load load;
	
}walk_x;

/*typedef struct
{
	/////// Z-Shift Control /////
	float peak_start;
	float peak_max;//0.5
	float peak_stay;//0.75
	float peak_end;
	float zmax;
	float zfreq;
	float zphase;
	float zdisplacement;

}walk_zshift;
*/

typedef struct
{
	/////// Z-Shift Control /////
	float zmax;
	float startTime;
	float startPhase;
	float maxTime;
	float maxPhase;
	float stayTime;//0.75
	float stayPhase;
	float endTime;
	float endPhase;
	float displacement;
	
	float decTime;
	float decDisplacement;
	float decPhase;
	float decFreq;
	float decAmp;

}walk_zshift;
typedef struct
{
	/////// Y Control /////
	float y_start;
	float y_end;
	float y_0;
	float y_move;
}walk_y;

typedef struct
{
	/////// Z Control /////
	float z_start;
	float z_0;
	float z_move;
	float z_end;
}walk_z;

typedef struct
{
	 ///////Separation control////
    	float seperation;
 	/////COMPLIANCE//
	float cptime;
}walk_misc;

typedef struct
{
	//////Turn Control////
	float turn_start;
	float turn_end;
}walk_turn;

typedef struct
{
	/////// COM Trajectory //////
	//Z Movement
	float zstartTime;
	float zstartPhase;
	float zpeakTime;
	float zpeakPhase;
	float zstayTime;
	float zstayPhase;
	float zendTime;
	float zendPhase;
	float zmax;
	
	//Y Movement
	float y_com_start;
	float y_com_end;
}walk_com;

typedef struct
{
	walk_x x;
	walk_zshift z_shift;
	walk_y y;
	walk_z z;
	walk_turn turn;
	walk_misc misc;
	walk_com com_trajectory;
}leg_config;
	
typedef struct
{
	float time;
	float exp_time;
	int fps;
	leg_config left;
	leg_config right;
}walk_config; 

walk_config walk;

typedef struct
{
	int leg;
	float hip_velo[3];
	float hip_coods[3];
	float Omega[3];
	float cor_z;
	float cor_zr;
	float set_point[2][3];
	struct timeval endTime;
	
}walk_end;	

walk_end prevWalk;
	
////////////////////////////////////////////////////////////////////
///////////////////// FORWARD KINEMATICS, INVERSE KINEMATICS , COM GENERATION , ZMP CALCULATION DEFINITIONS
float STEP=0;
	
/////////////////////////////////////////////////////////////////////
void reset();
void check();
void loads();
float* generatespline(int gn,float gx[],float gy[]);
float vectorMagnitude(float *vec);
float* vectorScalarMultiply(float *vec,float scalar,float *ans);
float* vectorCrossProduct(float *vec1,float *vec2,float *ans);
int signum(float num);
float scurve(float in,float fi,float t, float tot);
int inRange(float var, float limit_low, float limit_high);
int iwrite(int choose, int *termios_pointer, struct ftdi_context *ftdi_pointer, byte *packet, int length);
int iread(int choose, int *termios_pointer, struct ftdi_context *ftdi_pointer, byte *packet, int length);
int ipurge(int choose,int *termios_pointer, struct ftdi_context *ftdi_pointer);


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
	if (((f1 - precision) < f2) && ((f1 + precision) > f2))
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

void changemode(int dir)   // do not display characters in terminal
{
	  static struct termios oldt, newt;

	  if ( dir == 1 )
	  {
		tcgetattr( STDIN_FILENO, &oldt);
		newt = oldt;
		newt.c_lflag &= ~( ICANON | ECHO );
		tcsetattr( STDIN_FILENO, TCSANOW, &newt);
	  }
	  else
	  tcsetattr( STDIN_FILENO, TCSANOW, &oldt);
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

	int read_try=0;
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
	
	for(read_try=0;read_try<1;read_try++)
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
						
			while(noat<(MOTOR_DATA==0?4:4000)&&data_not_rcv==1)
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
							//printf("%d\t",rcv_packet[i]);
							chksum-=rcv_packet[i];
						}			
						//printf("CHKSUM %d C %d\n",rcv_packet[i],chksum);			
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
	

int bootup_files()
{
	initialize_ids();
	int chk_i_ids;
	int ret_motor,ret_imu;
	/*for (chk_i_ids = 0; chk_i_ids < 24; chk_i_ids += 1)
	{
		printf("bid[%d]= %d\n",chk_i_ids,bid[chk_i_ids]);
	}*/
	
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
   		ftdi_set_interface(&imu,(ftdi_interface)2);
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

//...........................................................imu calibrating functions........................................	

//----------------------------------------imu functions-------------------------------------------------------------------------------


int iwrite(int choose, int *termios_pointer, struct ftdi_context *ftdi_pointer, byte *packet, int length)
{
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

/*
void punch(int a)
{
	int sync_write_i,i;
	switch(a)
	{
		case 0:
			tx_packet[5]=0x30;
		    for(sync_write_i=0;sync_write_i<NOM;sync_write_i++)
		    {
				tx_packet[7+(sync_write_i*3)]= bid[sync_write_i];
	    		tx_packet[8+(sync_write_i*3)]=get_lb(low_punch);
	      		tx_packet[9+(sync_write_i*3)]=get_hb(low_punch);
	    	}
	
	    	tx_packet[3*NOM+7]=checksum();
			//	printf("presendsync\n");
			for(i=0;i<tx_packet[3]+4;i++)
			printf("%x\t",tx_packet[i]);
	    	send_sync();		
	    	printf("\n%d punch written\n",low_punch);
		break;
	case 1:
		tx_packet[5]=0x30;
	    for(sync_write_i=0;sync_write_i<NOM;sync_write_i++)
	    {	
	    	tx_packet[7+(sync_write_i*3)]= bid[sync_write_i];
	      	if(tx_packet[7+(sync_write_i*3)]==0||tx_packet[7+(sync_write_i*3)]==1||tx_packet[7+(sync_write_i*3)]==2||tx_packet[7+(sync_write_i*3)]==3||tx_packet[7+(sync_write_i*3)]==12||tx_packet[7+(sync_write_i*3)]==20||tx_packet[7+(sync_write_i*3)]==21||tx_packet[7+(sync_write_i*3)]==22||tx_packet[7+(sync_write_i*3)]==23||tx_packet[7+(sync_write_i*3)]==32)
	      	{
	      		tx_packet[8+(sync_write_i*3)]=get_lb(high_punch);
	      		tx_packet[9+(sync_write_i*3)]=get_hb(high_punch);
	      	}
	      	else
	      	{
	      		tx_packet[8+(sync_write_i*3)]=get_lb(low_punch);
	      		tx_packet[9+(sync_write_i*3)]=get_hb(low_punch);
	      	}
	    }
	
	   	tx_packet[3*NOM+7]=checksum();
		for(i=0;i<tx_packet[3]+4;i++)
		printf("%x\t",tx_packet[i]);
	    send_sync();		
	    printf("\n%d punch written",high_punch);
	break;
	}
	//tx_packet[128]={0xff,0xff,0xfe,LEN_SYNC_WRITE,INST_SYNC_WRITE,0x1e,0x02};
	tx_packet[2]=0xfe;
	tx_packet[3]=LEN_SYNC_WRITE;
	tx_packet[4]=INST_SYNC_WRITE;
	tx_packet[5]=0x1e;
	tx_packet[6]=0x02;
}
*/



/*----------------------------including imu change--------------------------------------------------*/
float* generatespline(int gn,float gx[],float gy[])
{
	
	  float ga[gn+1];
	  float gb[gn];
	  float gd[gn];
	
	  float gh[gn];
	  float gal[gn];
	  float gc[gn+1],gl[gn+1],gmu[gn+1],gz[gn+1];
	
	  //1..............................
	 // ga[]=new float[gn+1];
		int gi; 
	  for(gi=0;gi<gn+1;gi++)
	    ga[gi]=gy[gi];
	  //2.................................
	  //gb[]=new float[gn];
	  //gd[]=new float[gn];
	  //3.................................
	  //gh[]=new float[gn];
	  for(gi=0;gi<gn;gi++)
	    gh[gi]=gx[gi+1]-gx[gi];
	  //4...................................
	  //gal[]=new float[gn];
	  for(gi=1;gi<gn;gi++)
	    gal[gi]=3*((((float)ga[gi+1]-(float)ga[gi])/(float)gh[gi])-(((float)ga[gi]-(float)ga[gi-1])/(float)gh[gi-1]));
	  //5......................................
	  //gc[]=new float[gn+1];
	  //gl[]=new float[gn+1];
	  //gmu[]=new float[gn+1];
	  //gz[]=new float[gn+1];
	  //6........................................
	  gl[0]=1;
	  gmu[0]=0;
	  gz[0]=0;
	  //7.........................................
	  for(gi=1;gi<gn;gi++)
	  {
	    gl[gi]=2*(gx[gi+1]-gx[gi-1])-(gh[gi-1]*gmu[gi-1]);
	    gmu[gi]=(float)gh[gi]/(float)gl[gi];
	    gz[gi]=((float)gal[gi]-(float)gh[gi-1]*gz[gi-1])/(float)gl[gi];
	  }
	  //8.........................................
	  gl[gn]=1;
	  gz[gn]=0;
	  gc[gn]=0;
	  //9.............................................
	  int gj;
	  for(gj=gn-1;gj>=0;gj--)
	  {
	    gc[gj]=gz[gj]-gmu[gj]*gc[gj+1];
	    gb[gj]=(((float)ga[gj+1]-(float)ga[gj])/(float)gh[gj])-(((float)gh[gj]*((float)gc[gj+1]+2*(float)gc[gj]))/(float)3);
	    gd[gj]=((float)gc[gj+1]-(float)gc[gj])/(float)(3*gh[gj]);
	  }
	  //10......................................................
	
	
	  int tp=0,gt;
	  for(gt=0;gt<gn;gt++)
	  {
	    printf("eqn");
	    output[tp]=ga[gt];
	    printf("%f    ",ga[gt]);
	    tp++;
	    output[tp]=gb[gt];
	    printf("%f    ",gb[gt]);
	    tp++;
	    output[tp]=gc[gt];
	    printf("%f    ",gc[gt]);
	    tp++;
	    output[tp]=gd[gt];
	    printf("%f    \n",gd[gt]);
	    tp++;
	  }
	  return output;
}

int read_load(int n)
{
	int load=0,dir=0,mov=0;
	int count=0,count1=0,no_of_try=0;	
	byte val,snd_packet[8],rcv_packet[10],chksum=0xff,noat=0;
	snd_packet[0]=0xff;snd_packet[1]=0xff;snd_packet[2]=n;snd_packet[3]=0x04;snd_packet[4]=0x02;snd_packet[5]=0x28;snd_packet[6]=0x06;snd_packet[7]=0xff;
	int load_try =0;
	for (no_of_try = 0; no_of_try < 5; no_of_try += 1)
	{
		snd_packet[7]-=snd_packet[no_of_try+2];
	}
	
	while(load_try++<3)
	{
		//if(ftdi_write_data(&ftdic1,snd_packet,snd_packet[3]+4)==8)
		if(iwrite(MOTOR_DATA,&fd,&ftdic1,snd_packet,snd_packet[3]+4)==8)
		{
			noat=0;
			chksum=0xff;
			while(noat<10)
			{
				//if(ftdi_read_data(&ftdic1,&val,1)==1)
				if(iread(MOTOR_DATA,&fd,&ftdic1,&val,1)==1)
				{
					if(val==0xff)
					count++;
					else
					{
						count=0;
						noat++;
					}
				}
				else
				noat++;
				if(count==2)
				{
					if(iread(MOTOR_DATA,&fd,&ftdic1,rcv_packet,10)==10)
					{
						int lol=0;
						for (lol = 0; lol < 9; lol += 1)
						{
							//printf("%d\t",rcv_packet[lol]);
							chksum-=rcv_packet[lol];
						}
						//printf("CHK %d",chksum);
						if(chksum==rcv_packet[9])
						{
							load_try=111;
							break;
						}
						else
						printf("Checksum Error\n");
									
					}
					else
					printf("Unable to read\n");
	
				}
			}
			if(noat>=10)
			printf("Noat Error\n");
	
		}
		else
		load_try++;
		if(load_try==111)
		{
			load=((rcv_packet[4]&0x03)*256)+rcv_packet[3];
			dir=(rcv_packet[4]&0x04)/4;
			mov=rcv_packet[8];
			//printf("VAL %d DIR %d\n",load,dir);
			//printf("ID %d LB %d HB %d\n",rcv_packet[0],rcv_packet[3],rcv_packet[4]);
		}
		else
		{
			//printf("RETRY\n");
			rcv_packet[0]=n;
			rcv_packet[3]=0x00;
			rcv_packet[4]=0xee;
			load=0;
			dir=9;
			mov=0;
				
		}
	}
	/*if(try==112)
	printf("DONE\n");
	else
	printf("NOT DONE\n");
	*/
	return ((load*100)+dir+(mov*10));
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




int torque(int enable)
{
	if(enable!=0)
	enable=1;
	
	int c=0;
	byte torque_packet[NOM*2+8];
	torque_packet[0]=0xff;
	torque_packet[1]=0xff;
	torque_packet[2]=0xfe;
	torque_packet[3]=NOM*2+4;
	torque_packet[4]=0x83;
	torque_packet[5]=0x18;
	torque_packet[6]=0x01;
	torque_packet[7+NOM*2]=0xff-torque_packet[2]-torque_packet[3]-torque_packet[4]-torque_packet[5]-torque_packet[6];
	
	for(c=0;c<NOM;c++)
	{
		torque_packet[7+c*2]=bid[c];
		torque_packet[8+c*2]=enable;
		torque_packet[7+NOM*2]-=(torque_packet[7+c*2]+torque_packet[8+c*2]);
	}
	
	//if(ftdi_write_data(&ftdic1,torque_packet,torque_packet[3]+4)>0)
	if(iwrite(MOTOR_DATA,&fd,&ftdic1,torque_packet,torque_packet[3]+4)>0)
	printf("TORQUE PACKET SENT\n");
	else
	printf("ERROR : TORQUE\n");
	
	
}


int inRange(float var, float limit_low, float limit_high)
{
	if(var>=limit_low&&var<limit_high)
	return 1;
	else 
	return 0;
}

float calcInc(float *arr, int length)
{
	int i=0;
	float val=0;
	for(i=0;i<length;i++)
	{
		//printf("VAL %10.5f\n",val);
		val+=arr[i]*sin(3.14*i/length);
	}
	
	return val;
}

int signum(float num)
{
	if(num<0)
	return -1;
	else if(num>0)
	return 1;
	else
	return 0;
}


float vectorMagnitude(float *vec)
{
	return sqrt(pow(vec[0],2)+pow(vec[1],2)+pow(vec[2],2));
}

float* vectorScalarMultiply(float *vec,float scalar,float *ans)
{
	ans[0]=vec[0]*scalar;
	ans[1]=vec[1]*scalar;
	ans[2]=vec[2]*scalar;
	

	return ans;
}

float* vectorCrossProduct(float *vec1,float *vec2,float *ans)
{
	ans[0]=vec1[1]*vec2[2]-vec1[2]*vec2[1];
	ans[1]=-(vec1[0]*vec2[2]-vec1[2]*vec2[0]);
	ans[2]=vec1[0]*vec2[1]-vec1[1]*vec2[0];
	
	return ans;
}


/* Variables to calibrate
	
	correction P gains for all sequences 
	velocity contribution for wt shift 



	
*/	


enum{	LEFT=0,
	RIGHT=1};

enum states{WSHIFT};

enum components{GAIT_GENERATOR,
		COMP_LENGTH
		};






class Acyut
{
	public:
	float left_x,left_y,left_z,left_rot;
	float right_x,right_y,right_z,right_rot;
	int grounded;
	int ik_val[33];
	int change[33];
	
	int imu_init;
	float angle[3];
	float accel[3];
	
	float left_joints[7][3];
	float rot_leftJoints[3];
	float right_joints[7][3];
	float rot_rightJoints[3];
	float left_com[3],right_com[3],com[3],rot_com[3];
	//float rot_hip[3],hip[3];
	//float rot_rleg[3];
	
	
		
	private:
	
	//// IMU STUFF
	
	float base_angle[3];
	float imu_angle[3];  
	float held_angle[3];
	float gyro_angle[3];
	float ang_acc[3];

	float base_gyro[3];
	float imu_gyro[3];
	float prev_gyro[3];

	float base_acc[3];
	float imu_acc[3];
	
	float gravity[3];
	float gravity_magnitude;
		

	//// MOTOR DATA 
	int offset[33];
	int motor_val[33];
	int read_val[33];
	byte tx_packet[128];
	
	public:

	float* rotateToIMU(float *quan,float *result)
	{
		float temp[3]={0};
		float temp2[3]={0};
		float ang[3];
		
		ang[0]=angle[0]*3.14/180;
		ang[1]=angle[1]*3.14/180;
		ang[2]=0;
		
		convertIKtoIMU(quan,temp);
		rotateIMU(temp,ang,temp2);
		convertIMUtoIK(temp2,result);
		
		return result;
		
	}

	float* rotateFromIMU(float *quan,float *result)
	{
		float temp[3]={0};
		float temp2[3]={0};
		float ang[3];
		
		ang[0]=angle[0]*3.14/180;
		ang[1]=angle[1]*3.14/180;
		ang[2]=0;
		
		convertIKtoIMU(quan,temp);
		revrotateIMU(temp,ang,temp2);
		convertIMUtoIK(temp2,result);
		
		return result;	
		left_y=0;
		left_z=0;
	
		
	}

	int setLegCoods(float left_x,float left_y,float left_z,float left_rot,float right_x,float right_y,float right_z,float right_rot)
	{
		this->left_x=left_x;
		this->left_y=left_y;
		this->left_z=left_z;
		this->left_rot=left_rot;
		this->right_x=right_x;
		this->right_y=right_y;
		this->right_z=right_z;
		this->right_rot=right_rot;
		return EXIT_SUCCESS;
	}

	int updateFrame()
	{
		generateik(LEFT);
		generateik(RIGHT);
		applyChange();
	}
	
	
	int sendFrame()
	{
		set_offset();
		generatePacket();
		iwrite(MOTOR_DATA,&fd,&ftdic1,tx_packet,tx_packet[3]+4);
	}
	
	void setIMUBase()
	{
		//printf("SETTING IMU BASE\n");
		if(get_imu_data(10)==EXIT_SUCCESS)
		{
			imu_init=1;
			printf("IMU INITIALIZED\n");
			for(int i=0;i<3;i++)
			{
				base_angle[i]=imu_angle[i];
				base_acc[i]=imu_acc[i];
				base_gyro[i]=imu_gyro[i];
				prev_gyro[i]=imu_gyro[i];
			}
			gravity_magnitude=sqrt(pow(base_acc[0],2)+pow(base_acc[1],2)+pow(base_acc[2],2));
			
			gravity[0]=0;//base_acc[0];
			gravity[1]=0;//base_acc[1];
			gravity[2]=base_acc[2];
			//printf("GRAVITY : %f\t%f\t%f\n",gravity[0],gravity[1],gravity[2]);		
		}
		else 
		imu_init=0;
		
	}	

	inline void getIMUAngle()
	{
		int i=0;
		float gyro_threshold=0.01;
		float gyro_scale=1.5;
		float acc_magnitude;
		if(imu_init==0||get_imu_data(1)==EXIT_FAILURE)
		{
			//printf("IMU ANGLE NOT AVAILABLE\n");
			return ;
		}
		for(i=0;i<3;i++)
		{
			imu_angle[i]-=base_angle[i];
			imu_gyro[i]-=base_gyro[i];
			ang_acc[i]=imu_gyro[i]-prev_gyro[i];
			prev_gyro[i]=imu_gyro[i];
		}
		acc_magnitude=sqrt(pow(imu_acc[0],2)+pow(imu_acc[1],2)+pow(imu_acc[2],2));
		//printf("GRAVITY %f\t ACC %f\n",gravity_magnitude,acc_magnitude);
		//printf("GYRO 0 :%f GYRO 1 : %f\n",imu_gyro[0],imu_gyro[1]);
		if(fabs(imu_gyro[0])<=gyro_threshold&&fabs(imu_gyro[1])<=gyro_threshold)//&&abs(imu_gyro[2])<=gyro_threshold)
		{
			if(fabs(acc_magnitude-gravity_magnitude)<1)
			{	
				//// STATIC CONDITION - > RESETTING HELD ANGLE 
				for(i=0;i<2;i++)
				{
					//printf("STATIC COND \n");
					held_angle[i]=imu_angle[i];
					gyro_angle[i]=0;
				}
			}
			else
			{
				//// LINEAR ACC CONDITION 
				for(i=0;i<2;i++)
				{
					//printf("PURE ACC \n");
					held_angle[i]=held_angle[i];
					//gyro_angle[i]+=imu_gyro[i]*gyro_scale;	
				}
			}
		}
		else
		{
			for(i=0;i<2;i++)
			{
				//printf("ROTATING\n");
				gyro_angle[i]+=imu_gyro[i]*gyro_scale;
				//printf("GYRO ANGLE %d : %f\n",i,gyro_angle[i]);	
			}
		}
		
		for(i=0;i<2;i++)
		{
			angle[i]=held_angle[i]+gyro_angle[i];	
			//angle[i]=imu_angle[i];
		//	printf("ANGLE[%d] = %f\t",i,angle[i]);
		}
		//printf("\n");
	}		
	
	
	int  projection_system()
	{
		readLegMotors();
		forward_kinematics(LEFT);
		forward_kinematics(RIGHT);
		generate_leg_com(LEFT);
		generate_leg_com(RIGHT);
		generate_com();
		rotate_com();
		
//		for (int i = 0; i < 7; i += 1)
//		{
//			printf("L %f %f %f\n",left_joints[i][0],left_joints[i][1],left_joints[i][2]);
//		}
//		for (int i = 0; i < 7; i += 1)
//		{
//			printf("R %f %f %f\n",right_joints[i][0],right_joints[i][1],right_joints[i][2]);
//		}
//		printf("L COM X:%lf\tY:%lf\tZ:%lf\n",left_com[0],left_com[1],left_com[2]);
//		printf("R COM X:%lf\tY:%lf\tZ:%lf\n",right_com[0],right_com[1],right_com[2]);
//		printf("THIS COM X:%lf\tY:%lf\tZ:%lf\n",com[0],com[1],com[2]);

		//generate_com(left_joints,right_joints,left_com,right_com);
		//rotate_com(1-leg,com,left_joints[6],right_joints[6],rot_com,rot_leftJoints,rot_rightJoints);
		
		//printf("LEFT COM X:%10.5f Y:%10.5f Z:%10.5f\n",left_com[0],left_com[1],left_com[2]);
		//printf("RIGHT COM X:%10.5f Y:%10.5f Z:%10.5f\n",right_com[0],right_com[1],right_com[2]);
		//printf("COM X:%10.5f Y:%10.5f Z:%10.5f\n",com[0],com[1],com[2]);
		
		/*if(leg==1)
		printf("LEFT  LEG X : %f Y : %f Z : %f\t",left_joints[6][0],left_joints[6][1],left_joints[6][2]);  
		else 
		printf("RIGHT LEG X : %f Y : %f Z : %f\t",right_joints[6][0],right_joints[6][1],right_joints[6][2]);  
		*/
			
		//printf("LEFT X:%10.5f Y:%10.5f Z:%10.5f\n",rot_leftJoints[0],rot_leftJoints[1],rot_leftJoints[2]);
		//printf("RIGHT X:%10.5f Y:%10.5f Z:%10.5f\n",rot_rightJoints[0],rot_rightJoints[1],rot_rightJoints[2]);
		//printf("COM X:%10.5f Y:%10.5f Z:%10.5f\n",rot_com[0],rot_com[1],rot_com[2]);
		
	}
	
	private:
	
	
	float* convertIKtoIMU(float *in, float *out)
	{
		out[0]=-in[1];  // Origin remains same 
		out[1]=in[2];
		out[2]=-in[0];
		
		return out;
	}	

	float* convertIMUtoIK(float* in, float *out)
	{
		out[0]=-in[2]; // Origin remains same 
		out[1]=-in[0];
		out[2]=in[1];
		
		return out;
	}

	float* rotateIMU(float *quan,float *ang, float *result)
	{
		result[0]=cos(ang[1])*quan[0]+sin(ang[1])*quan[2];
		result[1]=sin(ang[0])*sin(ang[1])*quan[0]+cos(ang[0])*quan[1]-sin(ang[0])*cos(ang[1])*quan[2];
		result[2]=-cos(ang[0])*sin(ang[1])*quan[0]+sin(ang[0])*quan[1]+cos(ang[0])*cos(ang[1])*quan[2];
		
		return result;
	}	
	
	float* revrotateIMU(float *quan,float *ang, float *result)
	{	
		result[0]=cos(ang[1])*quan[0]+sin(ang[1])*sin(ang[0])*quan[1]-sin(ang[1])*cos(ang[0])*quan[0];
		result[1]=cos(ang[0])*quan[1]+sin(ang[0])*quan[2];
		result[2]=sin(ang[1])*quan[0]-cos(ang[1])*sin(ang[0])*quan[1]+cos(ang[1])*cos(ang[0])*quan[2];
		
		return result;
	}	
	
	int readLegMotors()
	{
		int read_val;
		int mot_no[]={0,1,2,12,3,4,5,20,21,22,32,23,24,25};
		int flag=0;	
		for(int i=0;i<14;i++)
		{
			//printf("READING MOTOR NO. %d\n",mot_no[i]);
			//if(mot_no[i]==0)//||mot_no[i]==1||mot_no[i]==3)
			{
				this->read_val[mot_no[i]]=ik_val[mot_no[i]];
				continue;
			}
			read_val=read_position(mot_no[i]);
			if(read_val==9999)
			{
				printf("USING TARGET VALUE %d\n",mot_no[i]);
				this->read_val[mot_no[i]]=read_val;
			}
			else
			{
				this->read_val[mot_no[i]]=read_val-offset[mot_no[i]];
				printf("MOT : %d TARGET : %d ACTUAL %d\n",mot_no[i],ik_val[mot_no[i]],this->read_val[mot_no[i]]);
			}
		}
	}
	
	int  forward_kinematics(int leg)   
	{
		float hip_length=100;
		int i =0;
		
		float (*coods)[3];
		if(leg==LEFT)
		coods=left_joints;
		else 
		coods=right_joints;
		
		//float coods[7][3]={0};
		float mval[6]={0};
		
		if(leg==LEFT)
		{
			mval[0]=((float)read_val[0]-1675)*300/4096;
			mval[1]=((float)read_val[1]-3207)*300/4096;
			mval[2]=((float)read_val[2]-1752)*300/4096;
			mval[3]=((float)read_val[3]-1714)*300/4096;
			mval[4]=((float)read_val[4]-742)*300/1023;
			mval[5]=((float)read_val[5]-1703)*300/4096;
		}
		else if(leg==RIGHT)
		{
			mval[0]=((float)read_val[20]-1675)*300/4096;
			mval[1]=((float)read_val[21]-3207)*300/4096;
			mval[2]=((float)read_val[22]-1752)*300/4096;
			mval[3]=((float)read_val[23]-1714)*300/4096;
			mval[4]=((float)read_val[24]-274)*300/1023;
			mval[5]=((float)read_val[25]-1703)*300/4096;
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
	
		coods[0][0]=0;
		coods[1][0]=0;
		coods[2][0]=0;
		
		for(i=0;i<6;i++)
		{
			form_matrix(&frames[i]);
			//print_matrix(frames[i].rotmat);
			//printf("\n\n\n\n");
			base_frame=multiply(base_frame,frames[i].rotmat);
			coods[i+1][0]=base_frame.mat[0][3];
			coods[i+1][1]=base_frame.mat[1][3]+pow(-1,leg+1)*hip_length/2;
			coods[i+1][2]=base_frame.mat[2][3];
			//printf("%f\t%f\t%f\nleft_joints[i][0],",coods[i+1][0],coods[i+1][1],coods[i+1][2]);
			//print_matrix(base_frame);
	
		}
//		if(leg==0)
//		{
//			for(i=0;i<7;i++)
//			{
//				left_joints[i][0]=coods[i][0];
//				left_joints[i][1]=coods[i][1];   
//				left_joints[i][2]=coods[i][2];
//				//printf("LEFT  %f\t%f\t%f\n",left_joints[i][0],left_joints[i][1],left_joints[i][2]);
//			}	
//			//printf("LEFT  %f\t%f\t%f\n",left_joints[6][0],left_joints[6][1],left_joints[6][2]);
//		
//		}		
//		else if(leg==1)
//		{	
//			for(i=0;i<7;i++)
//			{
//				right_joints[i][0]=coods[i][0];
//				right_joints[i][1]=coods[i][1];
//				right_joints[i][2]=coods[i][2];
//				//printf("RIGHT %f\t%f\t%f\n",right_joints[i][0],right_joints[i][1],right_joints[i][2]);
//			}
//			//printf("RIGHT %f\t%f\t%f\n",right_joints[6][0],right_joints[6][1],right_joints[6][2]);
//	
//		}
		return EXIT_SUCCESS;
	}
	
	int generate_leg_com(int leg)
	{		
		int i=0,j=0;
		float (*coods)[3];
		float *leg_com;
		if(leg==LEFT)
		{
			coods=left_joints;
			leg_com=left_com;
		}
		else if(leg==RIGHT)
		{
			coods=right_joints;
			leg_com=right_com;
		}
		
		float motor_mass=150;
		float tot_mass=0;	
	
		float bracket_mass[6];		
		
		bracket_mass[0]=0;  /// 5-4 bracket
		bracket_mass[1]=0;  /// 4-3 bracket
		bracket_mass[2]=112;   /// 3-12 bracket
		bracket_mass[3]=0;   /// 12-2 bracket
		bracket_mass[4]=112;   /// 2-1 bracket
		bracket_mass[5]=0;   /// 1-0 bracket
		
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
		
		//printf("leg COM X:%lf\tY:%lf\tZ:%lf\n",leg_com[0],leg_com[1],leg_com[2]);
		return EXIT_SUCCESS;
	}
	
	int generate_com()
	{
		// LEFT LEG CONTIBUTION
		
		float l_wt=1499;
		com[0]=(left_com[0]+0)*l_wt;
		com[1]=(left_com[1]+0)*l_wt;
		com[2]=(left_com[2]+0)*l_wt;
		
		//printf(" COM X:%lf\tY:%lf\tZ:%lf\n",com[0],com[1],com[2]);
		
		// RIGHT LEG CONTRIBUTION	
		
		float r_wt=1499;
		com[0]+=(right_com[0]+0)*r_wt;
		com[1]+=(right_com[1]+0)*r_wt;
		com[2]+=(right_com[2]+0)*r_wt;
		
		//printf(" COM X:%lf\tY:%lf\tZ:%lf\n",com[0],com[1],com[2]);
		
		// TORSO CONTRIBUTION
		float torso_wt=2734.9+300;
		com[0]+=121.358*torso_wt;
		com[1]+=0*torso_wt;
		com[2]+=0*torso_wt;
		
		//printf(" COM X:%lf\tY:%lf\tZ:%lf\n",com[0],com[1],com[2]);
			
		com[0]=com[0]/(l_wt+r_wt+torso_wt);
		com[1]=com[1]/(l_wt+r_wt+torso_wt);
		com[2]=com[2]/(l_wt+r_wt+torso_wt);
		
		
		return EXIT_SUCCESS;
	}
	
	int rotate_com()
	{	
		FILE *f;
	
		int i=0,j=0;	

		//// GETTING ANGLES FROM IMU
		getIMUAngle();
		//roll=current_angle[0]*3.14/180;
		//pitch=-current_angle[1]*3.14/180;
		//yaw=0;//current_angle[2];
	
	//	printf("ROLL %f PITCH %f \n\n\n\n",angle[0],angle[1]);
	
	//	float roll_matrix[3][3], pitch_matrix[3][3];
	//	roll_matrix.mat[0][0]=1;	roll_matrix.mat[0][1]=0;	roll_matrix.mat[0][2]=0;
	//	roll_matrix.mat[1][0]=0;	roll_matrix.mat[1][1]=cos(roll);roll_matrix.mat[1][2]=-sin(roll);
	//	roll_matrix.mat[2][0]=0;	roll_matrix.mat[2][1]=sin(roll);roll_matrix.mat[2][2]=cos(roll);
	
	//	pitch_matrix.mat[0][0]=cos(pitch);pitch_matrix.mat[0][1]=0;	pitch_matrix.mat[0][2]=sin(pitch);
	//	pitch_matrix.mat[1][0]=0;	pitch_matrix.mat[1][1]=1;	pitch_matrix.mat[1][2]=0;
	//	pitch_matrix.mat[2][0]=-sin(pitch);pitch_matrix.mat[2][1]=0;	pitch_matrix.mat[2][2]=cos(pitch);
	
	//	float coods[]={-com_leg_frame[1],com_leg_frame[0],-com_leg_frame[0]}
	
	
		//printf("%10.5f %10.5f %10.5f\n",com[0],com[1],com[2]);
	
		//printf("%10.5f %10.5f %10.5f\n",com_leg_frame[0],com_leg_frame[1],com_leg_frame[2]);
	
		rotateToIMU(com,rot_com);
		//rot_com[1]+=(-50)*(1-leg)+50*leg;

		rotateToIMU(right_joints[6],rot_rightJoints);
		//rot_hip[1]+=(-50)*(1-leg)+50*leg;

		rotateToIMU(left_joints[6],rot_leftJoints);
		//rot_rleg[1]+=(-50)*(1-leg)+50*leg;
	
	
		if(inRange(rot_rightJoints[0]-rot_leftJoints[0],-3,3)==1)
		{
			grounded=9;
	/*		if(leg==0)*/
	/*		{*/
	/*			rot_hip[0]=-rot_leftJoints[0];*/
	/*			rot_hip[1]=-rot_leftJoints[1]-50;*/
	/*			rot_hip[2]=-rot_leftJoints[2];		*/
	/*			rot_com[0]=rot_com[0]-rot_leftJoints[0];*/
	/*			rot_com[1]=rot_com[1]-rot_leftJoints[1]-50;*/
	/*			rot_com[2]=rot_com[2]-rot_leftJoints[2];*/
	/*		}*/
	/*		else if(leg==1)*/
	/*		{*/
	/*			rot_hip[0]=-rot_rightJoints[0];*/
	/*			rot_hip[1]=-rot_rightJoints[1]+50;*/
	/*			rot_hip[2]=-rot_rightJoints[2];*/
	/*			rot_com[0]=rot_com[0]-rot_rightJoints[0];*/
	/*			rot_com[1]=rot_com[1]-rot_rightJoints[1]+50;*/
	/*			rot_com[2]=rot_com[2]-rot_rightJoints[2];*/
	/*		}*/
		}
		else if(rot_rightJoints[0]>rot_leftJoints[0])
		{
			grounded=0;
	/*		rot_hip[0]=-rot_leftJoints[0];*/
	/*		rot_hip[1]=-rot_leftJoints[1]-50;*/
	/*		rot_hip[2]=-rot_leftJoints[2];*/
	/*		rot_com[0]=rot_com[0]-rot_leftJoints[0];*/
	/*		rot_com[1]=rot_com[1]-rot_leftJoints[1]-50;*/
	/*		rot_com[2]=rot_com[2]-rot_leftJoints[2];*/
		}
		else
		{
			grounded=1;
	/*		rot_hip[0]=-rot_rightJoints[0];*/
	/*		rot_hip[1]=-rot_rightJoints[1]+50;*/
	/*		rot_hip[2]=-rot_rightJoints[2];*/
	/*		rot_com[0]=rot_com[0]-rot_rightJoints[0];*/
	/*		rot_com[1]=rot_com[1]-rot_rightJoints[1]+50;*/
	/*		rot_com[2]=rot_com[2]-rot_rightJoints[2];*/
		}
	
//		if(leg==0)
//		{
//			rot_hip[0]=-rot_leftJoints[0];
//			rot_hip[1]=-rot_leftJoints[1]-50;
//			rot_hip[2]=-rot_leftJoints[2];		
//			rot_com[0]=rot_com[0]-rot_leftJoints[0];
//			rot_com[1]=rot_com[1]-rot_leftJoints[1]-50;
//			rot_com[2]=rot_com[2]-rot_leftJoints[2];
//			
//			rot_rleg[0]=rot_rightJoints[0]-rot_leftJoints[0];
//			rot_rleg[1]=rot_rightJoints[1]-rot_leftJoints[1];
//			rot_rleg[2]=rot_rightJoints[2]-rot_leftJoints[2];
//		}
//		else if(leg==1)
//		{
//			rot_hip[0]=-rot_rightJoints[0];
//			rot_hip[1]=-rot_rightJoints[1]+50;
//			rot_hip[2]=-rot_rightJoints[2];
//			rot_com[0]=rot_com[0]-rot_rightJoints[0];
//			rot_com[1]=rot_com[1]-rot_rightJoints[1]+50;
//			rot_com[2]=rot_com[2]-rot_rightJoints[2];

//			rot_rleg[0]=rot_leftJoints[0]-rot_rightJoints[0];
//			rot_rleg[1]=rot_leftJoints[1]-rot_rightJoints[1];
//			rot_rleg[2]=rot_leftJoints[2]-rot_rightJoints[2];
//		
//		}	
	
		//printf("GROUNDED %d\n",grounded);
		//printf("RLEG %f %f %f\n",rot_rleg[0],rot_rleg[1],rot_rleg[2]);
		#if PLOT==1
	
		zmp.coods[zmp.nof].x=rot_com[0];
		zmp.coods[zmp.nof].y=rot_com[1];
		zmp.coods[zmp.nof].z=rot_com[2]+COUNTER;
	
		zmp.left[zmp.nof].x=left_joints[6][0]-left_joints[6][0]*(1-leg)-right_joints[6][0]*leg;
		zmp.left[zmp.nof].y=left_joints[6][1]-(50+left_joints[6][1])*(1-leg)-(-50+right_joints[6][1])*leg;
		zmp.left[zmp.nof].z=left_joints[6][2]-left_joints[6][2]*(1-leg)-right_joints[6][2]*leg+COUNTER;
	
		zmp.right[zmp.nof].x=right_joints[6][0]-left_joints[6][0]*(1-leg)-right_joints[6][0]*leg;
		zmp.right[zmp.nof].y=right_joints[6][1]-(50+left_joints[6][1])*(1-leg)-(-50+right_joints[6][1])*leg;
		zmp.right[zmp.nof].z=right_joints[6][2]-left_joints[6][2]*(1-leg)-right_joints[6][2]*leg+COUNTER;
	
		//printf("ROT X:%lf\tY:%lf\tZ:%lf\n",rot_com[0],rot_com[1],rot_com[2]);
		f=fopen(comfile,"a");
		if(f!=NULL)
		{
			fprintf(f,"%lf\t%lf\t%lf\n",zmp.coods[zmp.nof].x,zmp.coods[zmp.nof].y,zmp.coods[zmp.nof].z);
			fclose(f);
		}	
		f=fopen(leftfile,"a");
			if(f!=NULL)
		{
			fprintf(f,"%lf\t%lf\t%lf\n",zmp.left[zmp.nof].x,zmp.left[zmp.nof].y,zmp.left[zmp.nof].z);
			fclose(f);
		}
		f=fopen(rightfile,"a");
		if(f!=NULL)
		{
			fprintf(f,"%lf\t%lf\t%lf\n",zmp.right[zmp.nof].x,zmp.right[zmp.nof].y,zmp.right[zmp.nof].z);
			fclose(f);
		}
		zmp.nof++;
		f=fopen(hipfile,"a");
		if(f!=NULL)
		{
			fprintf(f,"%lf\t%lf\t%lf\n",rot_hip[0],rot_hip[1],rot_hip[2]+COUNTER);
			fclose(f);
		}
		#endif
		return EXIT_SUCCESS;
		
	}
	
	inline int get_imu_data(int get_data)
	{
		byte im[20],i,j,k=0;
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
		int start=0;
		int count=0;
		int imu_noat=0;
		float temp=0;
			
		//printf("BASE_ANG 0: %f BASE_ANG 1: %f BASE_ANG 2: %f\n",base_angle[0],base_angle[1],base_angle[2]);	
		//if(imu_init!=1)
		//printf("INITIALIZE THE IMU\n");
		//tcflush(imu,TCIFLUSH);
		//tcflow(imu,TCIOFLUSH);
		for(;imu_no_of_times<get_data;imu_no_of_times++)
		{		
			imu_noat=0;
			flag=0;
			while(flag==0&&imu_noat<10)
			{   
				//if(ftdi_read_data(&imu,&im[0],1)>0)
				if(iread(IMU_DATA,&imu_t,&imu,&im[0],1)>0)
				{
					//printf("%x\n",im[0]);
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
						if(iread(IMU_DATA,&imu_t,&imu,&im[i],20)>0 && im[18]==254 && im[19]==254)
						{	
							flag=1;	
							//printf("CONFIRMED IMU PACKET\n");
						}
						
						if(flag==1)
						{	
							count++;
							//printf("count : %d\n",count);
							for(j=0,k=0;j<6;j+=2,k+=1)
							{
								temp=im[j]+im[j+1]*256;
								temp=(temp - 0) * (180 + 180) / (720 - 0) + -180;
								imu_angle[k]=imu_angle[k]+temp;
							}
							for(j=0,k=0;j<6;j+=2,k+=1)
							{
								temp=im[j+6]+im[j+7]*256;
								temp=(temp - 0) * (300 + 300) / (600 - 0) + -300;
								imu_acc[k]=imu_acc[k]+(temp*9.8/260);
							}
							for(j=0,k=0;j<6;j+=2,k+=1)
							{
								temp=im[j+12]+im[j+13]*256;
								temp=((temp - 0) * (300 + 300) / (60000 - 0) + -300);
								imu_gyro[k]=imu_gyro[k]+temp;
							}
							
						}	
					}
				}
				imu_noat++;
			}
			
		}
		
		if(count!=0)
		{
			//angle_moved+=abs(imu_gyro[0])<20000?imu_gyro[0]:0;
			for(k=0;k<3;k++)		
			{
				imu_angle[k]=imu_angle[k]/count;
				imu_acc[k]=(imu_acc[k]/count);
				imu_gyro[k]=imu_gyro[k]/count;
				//printf("ANGLE %d : %lf\t",k,imu_angle[k]);
				//printf("ACC %d : %lf\t",k,imu_acc[k]);
				//printf("GYRO %d : %lf\n",k,imu_gyro[k]);
				//printf("ang acc %f\n",ang_acc[k]);
			}
		}
		else
		{
			//printf("IMU READING FAILED\n");
			//usleep(16670*0.6);
		}
		ipurge(IMU_DATA,&imu_t,&imu);
		//ftdi_usb_purge_buffers(&imu);
		//tcflush(imu,TCIFLUSH);
		//tcflow(imu,TCIOFLUSH);
	}	
	
	
	//////  MOTOR CONTROL 
	int generateik(int leg)
	{
		int i=0;
		float cons[10];
		float ang[6];
		float theta[4];
		float phi[5];
		float b_len=180;   // Length of bracket 
		float l22=49;		// Distance between bracket ends
		float r2d=180/3.14159265358979323846264338327950288419716939937510;
		float x,y,z;

		if(leg==0)
		{
			x=left_x;
			z=left_z*cos(-left_rot*3.14/180)+left_y*sin(-left_rot*3.14/180);
			y=-left_z*sin(-left_rot*3.14/180)+left_y*cos(-left_rot*3.14/180);
			ik_val[4]=742-left_rot*1023/300;   /// 742 , 274 are initialize values of motors 4 , 24
		}
		else if(leg==1)
		{
			x=right_x;
			z=right_z*cos(right_rot*3.14/180)-right_y*sin(right_rot*3.14/180);
			y=right_z*sin(right_rot*3.14/180)+right_y*cos(right_rot*3.14/180);
			ik_val[24]=274+right_rot*1023/300;
		}
		else 
		return -1;
		
		float leg_l=pow(x*x+y*y+z*z,0.5);
		
		if(leg_l>2*b_len+l22)
		{
			printf("IK LIMITS EXCEEDED!!!!\n");
			return -2;
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
		ang[0]=phi[0];//+(current_angle[1]*pow(-1,leg))*0.2/r2d;
		ang[1]=-theta[1]+phi[1];//-(avg_angle[0]/r2d);
		ang[2]=-theta[2];
		ang[3]=-theta[3]-phi[3];
		ang[4]=phi[5];//+(avg_angle[1]/r2d)*pow(-1,leg);;
		//printf("%f\t%f\n",phi[0],ang[0]);
		ik_val[20*leg+0]=(ang[0]*r2d*4096/300)+cons[0];//+z*1.1;
		ik_val[20*leg+1]=(ang[1]*r2d*4096/300)+cons[1];
		ik_val[20*leg+2]=(ang[2]*r2d*4096/300)+cons[2];
		ik_val[20*leg+12]=(ang[2]*r2d*4096/300)+cons[2];
		ik_val[20*leg+3]=(ang[3]*r2d*4096/300)+cons[3];
		ik_val[20*leg+5]=(ang[4]*r2d*4096/300)+cons[4];//-z*1.1;
	
		// GEN PRINTING FOR DEBUGGING
		
		//for(i=0;i<5;i++)
		//{
			//printf("ANG[%d] : %f\t",i,ang[i]*r2d);
		//}
		//printf("\n");
		//for(i=0;i<5;i++)
		//printf("motor_val[%d] : %f\t",i,mval[i]);
		//printf("\n");
		//printf("THETA 3 : %f\tTHETA 2 : %f\tTHETA 1 : %f\t\n",theta[3]*r2d,theta[2]*r2d,theta[1]*r2d);
		//printf("PHI 3 : %f\tPHI 5 : %f\tPHI 1 : %f\tPHI 0 : %f\t\n",phi[3]*r2d,phi[5]*r2d,phi[1]*r2d,phi[0]*r2d);
		return EXIT_SUCCESS;
		
	}
	
	void applyChange()
	{
		for(int i=0;i<33;i++)
		{
			ik_val[i]+=change[i];
		}	
	
	}
	
	void set_offset()
	{
		for(int i=0;i<33;i++)
		motor_val[i]=ik_val[i]+offset[i];
	}	
	
	int speed(int a)
	{
		if(!(a>=0&&a<1024))
		return EXIT_FAILURE;
		
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
		for(sync_write_i=0;sync_write_i<NOM;sync_write_i++)
		{
			speed_packet[7+(sync_write_i*3)]= bid[sync_write_i];
			speed_packet[8+(sync_write_i*3)]=get_lb(a);
			speed_packet[9+(sync_write_i*3)]=get_hb(a);
		}
		for(i=2;i<NOM*3+7;i++)
		speed_packet[3*NOM+7]-=speed_packet[i];
		if(iwrite(MOTOR_DATA,&fd,&ftdic1,speed_packet,speed_packet[3]+4)<0)
		printf("Speed Not Written\n");
		
		return EXIT_SUCCESS;
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
		tx_packet[adchksum]=0xff-tx_packet[2]-tx_packet[3]-tx_packet[4]-tx_packet[5]-tx_packet[6];
		for(packet_i=0;packet_i<NOM;packet_i++)
		{
			tx_packet[3*packet_i+7]=bid[packet_i];
			tx_packet[3*packet_i+8]=get_lb(motor_val[bid[packet_i]]);
			tx_packet[3*packet_i+9]=get_hb(motor_val[bid[packet_i]]);
			tx_packet[adchksum]-=(tx_packet[3*packet_i+7]+tx_packet[3*packet_i+8]+tx_packet[3*packet_i+9]);
			//printf(" %d %d\n",bid[packet_i],motor_val[bid[packet_i]]);
		}
		
		return 1;
	
	}
	
	
	public:
	Acyut()
	{
		angle[0]=0;
		angle[1]=0;
		angle[2]=0;
		
		left_x=BOT_HEIGHT;
		left_y=0;
		left_z=0;
		right_x=BOT_HEIGHT;
		right_y=0;
		right_z=0;
		
		memset(change,0,31);
		memset(right_joints,0,21);
		memset(left_joints,0,21);
		memset(left_com,0,3);
		memset(right_com,0,3);
		memset(com,0,3);
		
		
		generateik(0);
		generateik(1);

		ik_val[6]=512;
		ik_val[7]=698;
		ik_val[8]=512;
		ik_val[9]=150;
		ik_val[10]=823;
		
		ik_val[15]=1863;
		ik_val[16]=2048;
		ik_val[17]=546;
		ik_val[18]=588;
	
		ik_val[26]=512;
		ik_val[27]=325;
		ik_val[28]=512;
		ik_val[29]=865;
		ik_val[30]=200;
		
		offset[0]=15+25;
		offset[1]=350-25-20-30; 
		offset[2]=275; 
		offset[3]=232+25;  
		offset[4]=25; 
		offset[5]=-15; 
		offset[6]=346-40;
		offset[7]=0; 
		offset[8]=0; 
		offset[9]=0-100; 
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
		offset[21]=334+15-25-20-30; 
		offset[22]=288; 
		offset[23]=248+25; 
		offset[24]=-10; 
		offset[25]=-15+50-55; 
		offset[26]=-306; 
		offset[27]= 0; 
		offset[28]=0; 
		offset[29]=80; 
		offset[30]=0;
		offset[31]=0;
		offset[32]=258;
		
		set_offset();
		generatePacket();
		
		speed(32);
		iwrite(MOTOR_DATA,&fd,&ftdic1,tx_packet,tx_packet[3]+4);
		sleep(4);
		speed(0);
		setIMUBase();
	}
	
};


				


//		BASEX,
//		PREVWTSHIFT,
//		WTSHIFT,
//		XLIFT,
//		REVXLIFT,
//		KEEP_WTSHIFT,
//		KEEP_XLIFT,
//		REVWTSHIFT,
//		XLAND,
//		REVXLAND,
//		MOVEY,
//		COMP_LENGTH,
//		};
		//CAPSTEP,
		//POSTCAP,
		
		
char* printComp(int comp)
{
	switch(comp)
	{
	}
}


typedef struct
{
	
	//unsigned start 	:2;
	//unsigned running	:2;
}WalkComponents;


class ComponentList
{
	public:
	bool value;
	float args[10];

};

class ComponentResponse
{
	public:
	
	ComponentList start[COMP_LENGTH];
	ComponentList stop[COMP_LENGTH];
	void *data;
	
	
	ComponentResponse()
	{
		data=NULL;
		for (int i = 0; i < COMP_LENGTH; i += 1)
		{
			start[i].value=false;
			stop[i].value=false;
			for (int j = 0; j < 10; j += 1)
			{
				start[i].args[j]=0;
				stop[i].args[j]=0;	
			}
			
			
		}
	}
	
};

class Component
{	

	// TODO  extend component to not need direct references to pointer
	public:
	void *pointer;
	int priority;
	ComponentResponse response;
	
	Component()
	{
		pointer=NULL;
		priority=-1;
	
	}
	
	

};



//union componentObjects{ char c;
//			int c;
//			};


//class wtShift;     /// FORWARD DECLARATIONS
//class revWtShift;
//class xLift;

enum{MOT0,MOT1,MOT2,MOT12,MOT3,MOT4,MOT5,REVMOT0,REVMOT1,REVMOT2,REVMOT12,REVMOT3,REVMOT4,REVMOT5,LEGMOTORS	
	};



class Walk
{
	public:
	
	// CONTROL VARIALBLES
	Acyut *bot;
	float leg_x;
	float leg_y;
	float leg_z;
	float leg_rot;
	float revleg_x;
	float revleg_y;
	float revleg_z;
	float revleg_rot;
	float time;
	int fps;
	int leg;
	int state;
	int change[LEGMOTORS];
	
	float velocityY;
	float velocityZ;
	float Omega;
	
	
	// PHYSICAL QUANTITIES
	float rotCom[3];
	float rotHip[3];
	float rotRevLeg[3];
	float hipVelo[3];
	float roll;
	float pitch;
	
	bool walking;
	

	
	Walk(Acyut *bot,int fps,int leg)
	{
		this->bot=bot;
		this->fps=fps;
		this->leg=LEFT;
		time=0;
		for (int i = 0; i < COMP_LENGTH; i += 1)
		{
			start[i].value=false;
			stop[i].value=false;
			runningComp[i]=false;
		}
		
		leg_x=BOT_HEIGHT;
		leg_y=0;
		leg_z=0;
		leg_rot=0;
		revleg_x=BOT_HEIGHT;
		revleg_y=0;
		revleg_z=0;
		revleg_rot=0;
		velocityY=0;
		velocityZ=0;
		Omega=0;
		walking=false;
		
		bot->projection_system();
		
		if(leg==RIGHT)
		{
			roll=bot->angle[1];
			pitch=bot->angle[0];
		
			rotHip[0]=-(bot->rot_leftJoints[0]);
			rotCom[0]=bot->rot_com[0]-bot->rot_leftJoints[0];
			rotRevLeg[0]=bot->rot_rightJoints[0]-bot->rot_leftJoints[0];
			rotHip[1]=-(bot->rot_leftJoints[1]);
			rotCom[1]=bot->rot_com[1]-bot->rot_leftJoints[1];
			rotRevLeg[1]=bot->rot_rightJoints[1]-bot->rot_leftJoints[1];
			rotHip[2]=-(bot->rot_leftJoints[2]);
			rotCom[2]=bot->rot_com[2]-bot->rot_leftJoints[2];
			rotRevLeg[2]=bot->rot_rightJoints[2]-bot->rot_leftJoints[2];
			prevHip[0]=rotHip[0];
			prevHip[1]=rotHip[1];
			prevHip[2]=rotHip[2];
			
		}
		else if(leg==LEFT)
		{
			roll=-(bot->angle[1]);
			pitch=bot->angle[0];
			
			rotHip[0]=-(bot->rot_rightJoints[0]);
			rotCom[0]=(bot->rot_com[0]-bot->rot_rightJoints[0]);   /// CONVERTING FROM FK COODS TO IK COODS 
			rotRevLeg[0]=bot->rot_leftJoints[0]-bot->rot_rightJoints[0];
			rotHip[1]=-(bot->rot_rightJoints[1])*-1;
			rotCom[1]=(bot->rot_com[1]-bot->rot_rightJoints[1])*-1;  /// CONVERTING FROM FK COODS TO IK COODS 
			rotRevLeg[1]=(bot->rot_leftJoints[1]-bot->rot_rightJoints[1])*-1;
			rotHip[2]=-(bot->rot_rightJoints[2]);
			rotCom[2]=(bot->rot_com[2]-bot->rot_rightJoints[2]);   /// CONVERTING FROM FK COODS TO IK COODS 
			rotRevLeg[2]=bot->rot_leftJoints[2]-bot->rot_rightJoints[2];
			prevHip[0]=rotHip[0];
			prevHip[1]=rotHip[1];
			prevHip[2]=rotHip[2];
			
		}
		
//		if(leg==LEFT)
//		{
//			for (int i = 0; i < 3; i += 1)
//			{
//				rotHip[i]=-(bot->rot_rightJoints[i]);
//				prevHip[i]=-(bot->rot_rightJoints[i]);
//				
//				rotCom[i]=bot->rot_com[i]-bot->rot_rightJoints[i];
//				rotRevLeg[i]=bot->rot_leftJoints[i]-bot->rot_rightJoints[i];
//				hipVelo[i]=0;
//			}
//		}
//		else if(leg==RIGHT)
//		{
//			for (int i = 0; i < 3; i += 1)
//			{
//				rotHip[i]=-(bot->rot_leftJoints[i]);
//				prevHip[i]=-(bot->rot_leftJoints[i]);
//				rotCom[i]=bot->rot_com[i]-bot->rot_rightJoints[i]*-1;
//				rotRevLeg[i]=bot->rot_leftJoints[i]-bot->rot_rightJoints[i];
//				hipVelo[i]=0;
//			}
//		}
		
		
	
		
		
		
	}
	
	int walkControl()
	{
		//TODO Component calling based on velocity 
		// Dribbling code only
		// Weight shifting components - WTSHIFT REVWTSHIFT
		if(velocityY!=0||velocityZ!=0||Omega!=0)
		walking=true;
		
		while(walking==true)
		{
			
			
		}
		
		
		
	}
	
	int getPhase(float hip_z)
	{
		int phase_z=0;
		if(hip_z<20&&hip_z>-5)
		phase_z=0;
		else if (hip_z>=20&&hip_z<45)
		phase_z=1;
		else if(hip_z>=45)
		phase_z=2;
		else if(hip_z<=-5&&hip_z>-57)
		phase_z=-1;
		else if(hip_z<=-57)
		phase_z=-2;
		//printf("%10.5f %2d\n",hip_z,phase_z);
		//printf("%2d\n",phase_z);
		return phase_z;	
	}
		
	
	void debug()
	{
		//printf("%s\t TIME %f\n",leg==LEFT?"LEFT":"RIGHT",time);
		//printf("TIME %f \n",time);
		
		//return;
		for (int i = 0; i < COMP_LENGTH; i += 1)
		{
			printf("%10s\t %d\t %d\t %d\t %d\n",printComp(i),start[i].value,stop[i].value,runningComp[i],component[i].priority);
		}
		printf("------X----X----X--------\n");
//		sleep(1);
		
	}
	
	void incrementTime()
	{
		usleep(1000000.0/fps);
		time+=(float)1.0/(float)fps;
	}
	
	void setLeg(int leg)
	{
		if(this->leg!=leg)
		{
			float temp_x=leg_x;
			float temp_y=leg_y;
			float temp_z=leg_z;
			
			leg_x=revleg_x;
			leg_y=revleg_y;
			leg_z=revleg_z;
			revleg_x=temp_x;
			revleg_y=temp_y;
			revleg_z=temp_z;
			
			hipVelo[1]*=-1;
			if(leg==LEFT)
			{
				prevHip[0]=-(bot->rot_rightJoints[0]);
				prevHip[1]=-(bot->rot_rightJoints[1]);
				prevHip[2]=-(bot->rot_rightJoints[2]);
			}	
			else 
			{
				prevHip[0]=-(bot->rot_leftJoints[0]);
				prevHip[1]=-(bot->rot_leftJoints[1])*-1;
				prevHip[2]=-(bot->rot_leftJoints[2]);
			}	
	
		}
		this->leg=leg;
	}
	
	
	void syncBot()
	{
		printf("LEG\tX %7.2f\tY %7.2f\tZ %7.2f\t",leg_x,leg_y,leg_z);
		printf("RLEG\tX %7.2f\tY %7.2f\tZ %7.2f\n",revleg_x,revleg_y,revleg_z);
		if(leg==LEFT)
		{
			bot->setLegCoods(leg_x,leg_y,leg_z,leg_rot,revleg_x,revleg_y,revleg_z,revleg_rot);
			bot->change[0]=change[MOT0];
			bot->change[1]=change[MOT1];
			bot->change[2]=change[MOT2];
			bot->change[12]=change[MOT12];
			bot->change[3]=change[MOT3];
			bot->change[4]=change[MOT4];
			bot->change[5]=change[MOT5];
			
			bot->change[20]=change[REVMOT0];
			bot->change[21]=change[REVMOT1];
			bot->change[22]=change[REVMOT2];
			bot->change[32]=change[REVMOT12];
			bot->change[23]=change[REVMOT3];
			bot->change[24]=change[REVMOT4];
			bot->change[25]=change[REVMOT5];
			
		}
		else if(leg==RIGHT)
		{
			bot->setLegCoods(revleg_x,revleg_y,revleg_z,revleg_rot,leg_x,leg_y,leg_z,leg_rot);
			bot->change[20]=change[MOT0];
			bot->change[21]=change[MOT1];
			bot->change[22]=change[MOT2];
			bot->change[32]=change[MOT12];
			bot->change[23]=change[MOT3];
			bot->change[24]=change[MOT4];
			bot->change[25]=change[MOT5];
			
			bot->change[0]=change[REVMOT0];
			bot->change[1]=change[REVMOT1];
			bot->change[2]=change[REVMOT2];
			bot->change[12]=change[REVMOT12];
			bot->change[3]=change[REVMOT3];
			bot->change[4]=change[REVMOT4];
			bot->change[5]=change[REVMOT5];
			
		}
		bot->updateFrame();
		bot->sendFrame();
	}
	
	///// COMPONENT HANDLING FUNCTIONS /////
	
	void runComponents()
	{
		//usleep(500000);
		updateRunList();
		getWalkFrame();
		//printf("RAN COMPONENTS\n");
		debug();
		updateResponse();
		//printf("RESPONSE COMPONENTS\n");
		//debug();
		doProjection();
		//printf("DONE PROJECTION\n");
		//debug();
		//updateStartList();
		//printf("UPDATED START\n");
		//debug();
		//updateStopList();
		//printf("UPDATED STOP\n");
		//debug();
		//printf("UPDATED RUN\n");
		//for (int i = 0; i < COMP_LENGTH; i += 1)
		//{
			//if(runningComp[i]==true)
			//evalComponent(i);
		//}
		//debug();
		//debug();
		//printf("RESPONSE\n");
		//debug();
		//printf("%f\t %f\t %f\t %f\t %f\t %f\n",leg_x,leg_y,leg_z,revleg_x,revleg_y,revleg_z);
		
	}
	
	int walkRunning()
	{
		for (int i = 0; i < COMP_LENGTH; i += 1)
		{
			if(runningComp[i]==true||start[i].value==true)
			return 1;
		}
		return 0;
	}
	

	bool isRunning(int componentID)
	{
		if((runningComp[componentID]==true&&stop[componentID].value==false)||(start[componentID].value==true))
		return true;
		else 
		return false;
		//return runningComp[componentID];
	
	}
	
	
	
	private:
	
	Component component[COMP_LENGTH];
	int priorityOrder[2][COMP_LENGTH];
	ComponentList start[COMP_LENGTH];
	ComponentList stop[COMP_LENGTH];
	bool runningComp[COMP_LENGTH];

	int createComponent(int component);
	int evalComponent(int component);
	int deleteComponent(int component);
	int getResponse(int component);
	float prevHip[3];
	
	void updateRunList()   
	{
		for (int i = 0; i < COMP_LENGTH; i += 1)
		{
//			printf("%s \n",printComp(i));
			
			if(start[i].value==true)
			{
//				printf("START %s \n",printComp(i));
				createComponent(i);
				runningComp[i]=true;
				start[i].value=false;
				for (int j = 0; j < 10; j += 1)
				{
					start[i].args[j]=0;
				}
			}
			else if(stop[i].value==true)
			{
				deleteComponent(i);
				runningComp[i]=false;
				stop[i].value=false;
			}
		}
		
	}
	
	void updateStartList()
	{
		for (int i = 0; i < COMP_LENGTH; i += 1)
		{
			updateStartComponent(i);
		}
	}
	
	void updateStartComponent(int component)  /// expand to include conditions specific to start components
	{
	
		for(int i=0; i<COMP_LENGTH;i++)
		{
			if(this->component[i].response.start[component].value==true)
			{
				//printf("COMPONENT %s %s\n",printComp(component),printComp(i));
				start[component].value=true;
				for (int j = 0; j < 10; j += 1)
				{
					start[component].args[j]=this->component[i].response.start[component].args[j];
				}	
				this->component[i].response.start[component].value=false;
			}
		}
	}
	
	void updateStopList()
	{
		for (int i = 0; i < COMP_LENGTH; i += 1)
		{
			updateStopComponent(i);
		}
	}
	
	void updateStopComponent(int component)  /// expand to include conditions specific to stop components
	{
		for(int i=0;i<COMP_LENGTH;i++)
		{
			if(this->component[i].response.stop[component].value==true)
			{
				stop[component].value=true;
				this->component[i].response.stop[component].value=false;
			}
		}
	}
	
	void updateResponse()
	{
		for (int i = 0; i < COMP_LENGTH; i += 1)
		{
			if(runningComp[i]==true)
			{
				getResponse(i);
//				printf("GOT RESPONSE\n");
				
				updateStartList();
//				printf("UPDATED START\n");

				updateStopList();
//				printf("UPDATED STOP\n");
			}
		}
//		printf("UPDATED ALL RESPONSES\n");
	}
	
	void getWalkFrame()
	{
		int length=getPriorityTable();
		for(int i=0;i<length;i++)
		{		
			evalComponent(priorityOrder[1][i]);
		}		
	}
	
	int getPriorityTable()
	{
		int length=0;
		for (int id = 0; id < COMP_LENGTH; id += 1)    // GETTING ACTIVE COMPONENTS
		{
			if(runningComp[id]==true)
			{
				priorityOrder[0][length]=component[id].priority;
				priorityOrder[1][length++]=id;
			}
			
		}
		for(int i=0;i<length;i++)
		{
			for(int j=i;j<length-1;j++)
			{
				if(priorityOrder[0][j+1]<priorityOrder[0][j])
				{
					priorityOrder[0][j+1]=priorityOrder[0][j+1]+priorityOrder[0][j];
					priorityOrder[0][j]=priorityOrder[0][j+1]-priorityOrder[0][j];
					priorityOrder[0][j+1]=priorityOrder[0][j+1]-priorityOrder[0][j];
					
					priorityOrder[1][j+1]=priorityOrder[1][j+1]+priorityOrder[1][j];
					priorityOrder[1][j]=priorityOrder[1][j+1]-priorityOrder[1][j];
					priorityOrder[1][j+1]=priorityOrder[1][j+1]-priorityOrder[1][j];
					
				}
			
			}
		}
		return length;
	}
	
	int givePriority(int id)
	{
		int maxPriority=-1;
		for (int i = 0; i < id; i += 1)
		{
			if(runningComp[i]==true&&maxPriority<component[i].priority)
			maxPriority=component[i].priority;
		}
		maxPriority++;
		
		return maxPriority;
	
	}

	int doProjection()
	{
		bot->projection_system();
		if(leg==RIGHT)
		{
			roll=bot->angle[1];
			pitch=bot->angle[0];
			rotHip[0]=-(bot->rot_leftJoints[0]);
			rotCom[0]=bot->rot_com[0]-bot->rot_leftJoints[0];
			rotRevLeg[0]=bot->rot_rightJoints[0]-bot->rot_leftJoints[0];
			rotHip[1]=-(bot->rot_leftJoints[1]);
			rotCom[1]=(bot->rot_com[1]-bot->rot_leftJoints[1]);
			rotRevLeg[1]=(bot->rot_rightJoints[1]-bot->rot_leftJoints[1]);
			rotHip[2]=-(bot->rot_leftJoints[2]);
			rotCom[2]=bot->rot_com[2]-bot->rot_leftJoints[2];
			rotRevLeg[2]=bot->rot_rightJoints[2]-bot->rot_leftJoints[2];
		}
		else if(leg==LEFT)
		{
			roll=-(bot->angle[1]);
			pitch=bot->angle[0];
			rotHip[0]=-(bot->rot_rightJoints[0]);
			rotCom[0]=(bot->rot_com[0]-bot->rot_rightJoints[0]);   /// CONVERTING FROM FK COODS TO IK COODS 
			rotRevLeg[0]=bot->rot_leftJoints[0]-bot->rot_rightJoints[0];
			rotHip[1]=-(bot->rot_rightJoints[1])*-1;
			rotCom[1]=(bot->rot_com[1]-bot->rot_rightJoints[1])*-1;  /// CONVERTING FROM FK COODS TO IK COODS 
			rotRevLeg[1]=(bot->rot_leftJoints[1]-bot->rot_rightJoints[1])*-1;
			rotHip[2]=-(bot->rot_rightJoints[2]);
			rotCom[2]=(bot->rot_com[2]-bot->rot_rightJoints[2]);   /// CONVERTING FROM FK COODS TO IK COODS 
			rotRevLeg[2]=bot->rot_leftJoints[2]-bot->rot_rightJoints[2];
		}
		
		
//		printf("ROTHIP\t");
		for (int i = 0; i < 3; i += 1)
		{
			hipVelo[i]=(rotHip[i]-prevHip[i])*fps;
			prevHip[i]=rotHip[i];
//			printf("%f\t",rotHip[i]);
		}
//		printf("\n");
//		printf("VELO\t");
//		for (int i = 0; i < 3; i += 1)
//		{
//			printf("%f \t",hipVelo[i]);
//		}	
//		printf("\n");
//		
		
	}
};

class LIMP
{
	private:
	Walk *walk_object;	
	float *time;
	float startTime;


	public:

	LIMP(Walk *walk_object, float *time)
	{
		this->walk_object=walk_object;
		this->time=time;
		this->startTime=*time;
	}
	
	int generateWalk()
	{
		walk_object->leg_x=0;
		walk_object->leg_y=0;
	}

};

/*class baseVals
{
	private:
	int bot_height;
	int seperation;
	Walk *walk_object;
	
	public:
	baseVals(Walk *walk_object)
	{
		this->walk_object=walk_object;
		this->bot_height=BOT_HEIGHT;
		this->seperation=0;
	}	

	int evaluate()
	{
		walk_object->leg_x=bot_height;
		walk_object->revleg_x=bot_height;
		walk_object->leg_x=0;
		walk_object->revleg_x=0;
		walk_object->leg_z=seperation;
		walk_object->revleg_z=seperation;
		
		for(int i =0 ;i<LEGMOTORS;i++)
		walk_object->change[i]=0;
		return EXIT_SUCCESS;
	}
	
	ComponentResponse* response(ComponentResponse *response)
	{
	
	}
};

class wtShift
{
	public:
	Walk *walk_object;
	int motor0change;
	float zshift;
	float runTime;
	float *time;
	
	float phase1;
	float phase2;
	float displacement;
	float startTime;
	wtShift(Walk *walk_object, float *time)
	{
		this->walk_object=walk_object;
		this->zshift=15;
		motor0change=-100;
		this->runTime=0.2;
		this->time=time;
		phase1=0;
		phase2=PI/2;
		displacement=0;
	}
	
	int evaluate()
	{
		float temp=zshift*(sin((phase2-phase1)*(*time-startTime)/runTime+phase1)+displacement)/(1+displacement);
		walk_object->leg_z+=temp;
		walk_object->revleg_z-=temp;
		walk_object->change[MOT0]-=motor0change*(sin((phase2-phase1)*(*time-startTime)/runTime+phase1)+displacement)/(1+displacement);
		
	
		return EXIT_SUCCESS;
	}
	
	ComponentResponse* response(ComponentResponse *response)
	{
		

	}
};
*/

int Walk::createComponent(int component)
{
	//printf("CREATE COMP\n");
	//printf("%s\n",printComp(component));
	if(runningComp[component]==true)
	{
		//printf("CREATING EXISTING COMPONENT\n");
		return EXIT_FAILURE;
	}
	else
	switch(component)
	{
		//case BASE : this->component[component].pointer = new baseVals(this);
		//		this->component[component].priority=givePriority(BASE);
		//break;
		//case WTSHIFT : this->component[component].pointer = new wtShift(this,&time);
		//		this->component[component].priority=givePriority(WTSHIFT);
		//break;
		default : printf("ERROR: COMPONENT DOES NOT EXIST\n");
		break;
	}
	getPriorityTable();
	return EXIT_SUCCESS;

}

int Walk::evalComponent(int component)
{
	///printf("EVAL COMP\n");
	if(runningComp[component]==true)
	switch(component)
	{
		//case BASE : ((baseVals*)(this->component[component].pointer))->evaluate();
		//break;
		//case WTSHIFT : ((wtShift*)(this->component[component].pointer))->evaluate();
		//break;
		default : 
		break;
		
	}
	
	return EXIT_SUCCESS;
		//return pointer;	
}
	
int Walk::deleteComponent(int component)
{
			
	//printf("DELETE COMP\n");
	if(runningComp[component]==true)
	switch(component)
	{
		//case BASE : delete (baseVals*)(this->component[component].pointer);
		//		this->component[component].priority=-1;
		//break;
		//case WTSHIFT : delete (wtShift*)(this->component[component].pointer);
		//		this->component[component].priority=-1;
		//break;
		default :
		break;
	}
	getPriorityTable();
	return EXIT_SUCCESS;
}

int Walk::getResponse(int component)
{
	//printf("RESPOSNSE COMP\n");
	if(runningComp[component]==true)
	switch(component)
	{
		//case BASE : 
		//break;
		//case WTSHIFT :
		//break;
		default :
		break;
	}

	return EXIT_SUCCESS;
}

int pingtest()
{
	//printf("pinging....\n");
	int failure[NOM][2];
	byte txp[15]={0xff,0Xff,0,0X02,0X01,0},rxp[4],chkff,count=2,exit=0,chksum,noat2=0;
	int er=0,ping_test_k=0,i,j,z,igm=0,noat=0;
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




int main()
{
	
	bootup_files();
	char ch;
	if(!pingtest())
	{
		printf("Bypass ping? (y/n)\n");
		scanf("%c",&ch);
		if(ch=='n')
		return EXIT_FAILURE;
	}

	Acyut bot;
	Walk *walk1 = new Walk(&bot,50,LEFT);
	int iter=0;
	compliance(0,10,10);
	while(walk1->walkRunning()==1)
	{
		//walk1->debug();
		//printf("Running Components\n");
		walk1->runComponents();
		//printf("Time INC\n");
		walk1->incrementTime();
		walk1->syncBot();
	}

	delete walk1;
//	#if PLOT==1
//	FILE *f;
//	f=fopen("Graphs/COUNT","r");
//	if(f!=NULL)
//	{
//		fscanf(f,"%d",&COUNT);
//		printf("COUNT : %d\n",COUNT);		
//		fclose(f);
//	}
//	char STR[4];
//	//itoa(COUNT,STR,10);
//	sprintf(STR,"%d",COUNT);
//	printf("%d\n",COUNT);
//	COUNT++;
//	printf("%d\n",COUNT);
//	strcat(comfile,"Graphs/Com/COM");
//	strcat(comfile,STR);
//	strcat(leftfile,"Graphs/LeftLeg/LEFT");
//	strcat(leftfile,STR);
//	strcat(rightfile,"Graphs/RightLeg/RIGHT");
//	strcat(rightfile,STR);
//	expect_com[0]='\0';
//	strcat(expect_com,"Graphs/ExpectedCOM/Expected");
//	strcat(expect_com,STR);
//	strcat(hipfile,"Graphs/Hip/Hip");
//	strcat(hipfile,STR);
//	strcat(normalrctn,"Graphs/NormalF/Normal");
//	strcat(normalrctn,STR);
//	
//	printf("%s\n%s\n%s\n%s\n%s\n%s\n",comfile,leftfile,rightfile,hipfile,expect_com,normalrctn);
//	
//	f=fopen(comfile,"w");
//	if(f!=NULL)
//	{
//		fprintf(f,"#X\tY\tZ\n");
//		fclose(f);
//	}
//	f=fopen(leftfile,"w");
//	if(f!=NULL)
//	{
//		fprintf(f,"#X\tY\tZ\n");
//		fclose(f);
//	}
//	f=fopen(rightfile,"w");
//	if(f!=NULL)
//	{
//		fprintf(f,"#X\tY\tZ\n");
//		fclose(f);
//	}
//	f=fopen(expect_com,"w");
//	if(f!=NULL)
//	{
//		fprintf(f,"#X\tY\tZ\n");
//		fclose(f);
//	}
//	f=fopen(hipfile,"w");
//	if(f!=NULL)
//	{
//		fprintf(f,"#X\tY\tZ\n");
//		fclose(f);
//	}
//	f=fopen(normalrctn,"w");
//	if(f!=NULL)
//	{
//		fprintf(f,"#X\tY\n");
//		fclose(f);
//	}
//	f=fopen("Graphs/COUNT","w");
//	if(f!=NULL)
//	{
//		fprintf(f,"%d",COUNT);
//		fclose(f);
//	}
//	#endif
//	byte read;
//	//torque(0);
//	//pingtest();
//	
//	int flag=0;
//	COUNTER=0;
//	//ftdi_set_latency_timer(&imu,1);
//	//ftdi_read_data_set_chunksize(&imu,4096);
//	initialize_acyut();
//	imu_initialize();
//	int ip=0;
//	walk5(1-flag,0,STEP,0,-STEP,0,0,0,0,0,0,0,0);
//	//usleep(16670*5);
//	while(ip++<7)
//	{
//		//printf("%d\n",read_position(5));
//		walk5(flag,-STEP,STEP,STEP,-STEP,0,0,0,0,0,0,0,0);
//		//sleep(3);//flag=1-flag;
//		//projection_system(flag);
//		usleep(16670);
//		flag=1-flag;
//		//walk5(0,0,15,0,-15,0,0,0,0,0,7,0,0);
//		//walk5(1,-15,0,15,0,0,0,0,0,0,0,7,0);
//		
//	}
//	walk5(flag,-STEP,0,STEP,0,0,0,0,0,0,0,0,0);
//	compliance(0,32,32);
//	//usleep(16670);
//	#if PLOT==1
//	f=fopen("WALK","w");
//	if(f!=NULL)
//	{
//		fwrite(&zmp,sizeof(zmp),1,f);
//		fclose(f);
//	}
//	close_files();
//	f=fopen("script","w");
//	if(f!=NULL)
//	{	
//		fprintf(f,"splot \"%s\" using 2:3:1 with lines, \"%s\" using 2:3:1 with lines, \"%s\" using 2:3:1  with lines, \"%s\" using 2:3:1 with lines, \"%s\" using 2:3:1 with lines\n",comfile,leftfile,rightfile,expect_com,hipfile);
//		//fprintf(f,"splot \"%s\" using 2:3:1 with lines, \"%s\" using 2:3:1 with lines\n",comfile,expect_com);
//		fclose(f);
//		//system("gnuplot < \"script\" -persist");
//	}
//	f=fopen("script2","w");
//	if(f!=NULL)
//	{
//		fprintf(f,"plot \"%s\" using 2,\"%s\" using 2, \"%s\" using 2\n",comfile,hipfile,expect_com);
//		fclose(f);
//		system("gnuplot < \"script2\" -persist");
//	}
//	f=fopen("script3","w");
//	if(f!=NULL)
//	{
//		fprintf(f,"plot \"%s\" using 3 with lines,\"%s\" using 3 with lines, \"%s\" using 3 with lines\n",comfile,hipfile,expect_com);
//		fclose(f);
//		system("gnuplot < \"script3\" -persist");
//	}
//	f=fopen("script4","w");
//	if(f!=NULL)
//	{
//		fprintf(f,"plot \"%s\" using 1 with lines\n",normalrctn);
//		fclose(f);
//		//system("gnuplot < \"script3\" -persist");
//	}
//	#endif		
////	sleep(2);
//	/*bootup_files();
//	initialize_acyut();
//	imu_initialize();
//	
//	int target_i=0;
//	for(target_i=0;target_i<10;target_i++)
//	{
//		get_imu_data();
//		printf("%d\n",target_i+1);
//		sleep(1);
//	}
//	
//	for(target_i=0;target_i<6;target_i++)
//	{
//		target_left[target_i]=motor_val[target_i]-offset[target_i];
//		target_right[target_i]=motor_val[20+target_i]-offset[20+target_i];
//	}
//	 	
//	projection_system(1);
//	*/
//	//forward_kinematics(1,cnst);
//	
//	
//	
////	generateik(390,0,0,0);
////	int i=0;
////	for(i=0;i<5;i++)
////	printf("%f\n",mval[i]);	

}
