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

/*
int setupWalk(int leg)
{

	struct timeval currentTime;
	float hip_velo[3]={0};
	float Omega[3]={0};
	float rad[3]={0};
	float proposedZShift=47;
	float init_phase=0;
	float setPhase=0;
	
	int start_sequence=0;
	int i=0;
	
	//projection_system(leg);
	gettimeofday(&currentTime,NULL);
	
	for(i=0;i<3;i++)
	{
		hip_velo[i]=(rot_hip[i]-prevWalk.hip_coods[i])/(1000*((currentTime.tv_sec+currentTime.tv_usec*1e-6)-(prevWalk.endTime.tv_sec+prevWalk.endTime.tv_usec*1e-6)));
		rad[i]=rot_hip[i]/1000;
	}
	
	printf("X: %10.5f Y: %10.5f Z: %10.5f XR: %10.5f YR: %10.5f ZR: %10.5f\n",prevWalk.set_point[0][0],prevWalk.set_point[0][1],prevWalk.set_point[0][2],prevWalk.set_point[1][0],prevWalk.set_point[1][1],prevWalk.set_point[1][2]); 
	
	//printf("DIFF HIP %10.5f TIME %10.5f\n",rot_hip[1]-prevWalk.hip_coods[1],((currentTime.tv_sec+currentTime.tv_usec*1e-6)-(prevWalk.endTime.tv_sec+prevWalk.endTime.tv_usec*1e-6)));
	
	vectorCrossProduct(rad,hip_velo,Omega);
	vectorScalarMultiply(Omega,180/(PI*pow(vectorMagnitude(rad),2)),Omega);
		
	hip_velo[1]*=pow(-1,leg+1);
	proposedZShift+=pow(hip_velo[1]*1000,2)*signum(hip_velo[1]*pow(-1,leg+1))/(17000*fabs(cos(current_angle[1]*3.14/180)));
	
	printf("ACC %10.5f \n",(17000*fabs(cos(current_angle[1]*3.14/180))));
	
	walk.time=1;
	walk.exp_time=1;
	walk.fps=50;
	
	walk.left.x.lift.x_start=0.2;
	walk.left.x.lift.x_end=0.8;
	walk.left.x.lift.lift=40;
	walk.left.x.lift.height=390;
	walk.left.x.lift.freq=2*3.14;
	walk.left.x.lift.displacement=1;	
	walk.left.x.lift.phase=-1.57;
	
	walk.left.x.land.x_land_start=0.9;//1-x_end   
	walk.left.x.land.x_land_increase_end=1;
	walk.left.x.land.x_land_decrease_start=1.5;
	walk.left.x.land.x_land_end=2;

	walk.left.x.land.x_land_freq=3.14;
	walk.left.x.land.x_land_displacement=1;
	walk.left.x.land.x_land_phase=-1.57;
	walk.left.x.land.x_land_dec=0;
	walk.left.x.land.x_land_base=0;
	
	walk.left.x.load.x_load_start=0.1;
	walk.left.x.load.x_load_increase_end=0.1;
	walk.left.x.load.x_load_decrease_start=0.1;
	walk.left.x.load.x_load_end=0.15;
	walk.left.x.load.x_load_displacement=0;//x_land_displacement;
	walk.left.x.load.x_load_phase=0;//-1.57;
	walk.left.x.load.x_load_freq=1.57;//3.14;
	walk.left.x.load.x_load_dec=0;//lift/60;
	
	if(proposedZShift>50)
	walk.left.z_shift.zmax=50;
	else if(proposedZShift<0)
	walk.left.z_shift.zmax=0;
	else 
	walk.left.z_shift.zmax=proposedZShift;
	
	init_phase=asin(((prevWalk.set_point[0][2]+prevWalk.set_point[1][2])*pow(-1,leg+2)/2)*(1+walk.left.z_shift.displacement)/walk.left.z_shift.zmax-walk.left.z_shift.displacement)-setPhase;
	printf("VELO %10.5f Zmax %10.5f\n",hip_velo[1],walk.left.z_shift.zmax);
	
	printf("side %10.5f ",(prevWalk.set_point[0][2]+prevWalk.set_point[1][2])*pow(-1,leg+1)/2);
	printf("INIT PHASE %10.5f \n",init_phase);
	if(isnanf(init_phase)==1)
	{
		//printf("DID THIS SHIT\n");
		//start_sequence=1;
		init_phase=-1.57;//exit(0);
	}
	
	walk.left.z_shift.startTime=0.0;
	walk.left.z_shift.startPhase=0;//init_phase;
	walk.left.z_shift.maxPhase=PI/2;
	walk.left.z_shift.maxTime=0.5*walk.left.z_shift.zmax/45*(walk.left.z_shift.maxPhase-walk.left.z_shift.startPhase)/(PI/2);//0.5
	walk.left.z_shift.stayTime=walk.left.z_shift.maxTime;//0.5*walk.left.z_shift.zmax/45;//0.75
	walk.left.z_shift.stayPhase=PI/2;
	walk.left.z_shift.endTime=walk.left.z_shift.stayTime+0.5;
	walk.left.z_shift.endPhase=PI;
	
	walk.left.z_shift.decAmp=10;//-5;
	walk.left.z_shift.decTime=0.1;
	walk.left.z_shift.decFreq=PI;
	walk.left.z_shift.decPhase=-PI/2;
	walk.left.z_shift.decDisplacement=1;
	//if(firstwalk==1)
	
	walk.left.z_shift.displacement=0;//1;
	
	walk.left.y.y_start=0;
	walk.left.y.y_0=0.20;
	walk.left.y.y_move=0.80;
	walk.left.y.y_end=1.0;
	
	walk.left.z.z_start=0;
	walk.left.z.z_0=0.2;
	walk.left.z.z_move=0.8;
	walk.left.z.z_end=1;
	
    	walk.left.misc.seperation=15;
	walk.left.misc.cptime=walk.left.x.lift.x_end;

	walk.left.turn.turn_start=0.2;
	walk.left.turn.turn_end=0.8;

	walk.left.com_trajectory.zstartTime=0;
	walk.left.com_trajectory.zstartPhase=0;
	walk.left.com_trajectory.zpeakTime=0.5;
	walk.left.com_trajectory.zpeakPhase=PI/2;
	walk.left.com_trajectory.zstayTime=0.5;
	walk.left.com_trajectory.zstayPhase=PI/2;
	walk.left.com_trajectory.zendTime=1;
	walk.left.com_trajectory.zendPhase=PI;
	walk.left.com_trajectory.zmax=45;
	
	walk.left.com_trajectory.y_com_start=0;
	walk.left.com_trajectory.y_com_end=1;

	walk.right=walk.left;
	
	
	
	return start_sequence;
}
*/
float x_val[2];


float setWalkLift(float t_frac,x_lift lift)
{
	float x=lift.height;
	if(t_frac>lift.x_start&&t_frac<=lift.x_end)
	x-=lift.lift*(sin(lift.freq*(t_frac-lift.x_start)/(lift.x_end-lift.x_start)+lift.phase)+lift.displacement)/(1+lift.displacement);
	
	return x;
}

float setWalkLand(float t_frac,x_land land)
{
	
	//printf("%10.5f %10.5f %10.5f %10.5f %10.5f \n",t_frac,land.x_land_start,land.x_land_increase_end,land.x_land_decrease_start,land.x_land_end);
	float x=land.x_land_base;
	///// Soften landing     
	if(t_frac>land.x_land_start&&t_frac<=land.x_land_increase_end)
	{
		x=land.x_land_base-land.x_land_dec*(sin(land.x_land_freq*(t_frac-land.x_land_start)/(land.x_land_increase_end-land.x_land_start)+land.x_land_phase)+land.x_land_displacement)/(1+land.x_land_displacement);
		//printf("land 1\n");
	}
	else if(t_frac>land.x_land_increase_end&&t_frac<=land.x_land_decrease_start)
	{
		x=land.x_land_base-land.x_land_dec;
		//printf("land 2\n");
	}
	else if(t_frac>land.x_land_decrease_start&&t_frac<=land.x_land_end)
	{
		x=land.x_land_base-land.x_land_dec*(sin(land.x_land_freq*(t_frac-land.x_land_decrease_start)/(land.x_land_end-land.x_land_decrease_start)+land.x_land_phase)+land.x_land_displacement)/(1+land.x_land_displacement);
		//printf("land 3\n");
	//x-=x_land_dec*(sin(x_land_freq*(t_frac-x_end+x_land_window)/(2*x_land_window)+x_land_phase)+x_land_displacement)/(1+x_land_displacement);
	}
	//printf("X LAND %f ", x);
	return x;
}

float setWalkLoad(float t_frac, x_load load)
{

	float xr=0;
	///// Assist weight shift
	if(t_frac>load.x_load_start&&t_frac<=load.x_load_increase_end)
	xr-=load.x_load_dec*(sin(load.x_load_freq*(t_frac-load.x_load_start)/(load.x_load_increase_end-load.x_load_start)+load.x_load_phase)+load.x_load_displacement)/(1+load.x_load_displacement);
	else if(t_frac>load.x_load_increase_end&&t_frac<=load.x_load_decrease_start)
	xr-=load.x_load_dec;
	else if(t_frac>load.x_load_decrease_start&&t_frac<=load.x_load_end)
	xr-=load.x_load_dec*(1-(sin(load.x_load_freq*(t_frac-load.x_load_decrease_start)/(load.x_load_end-load.x_load_decrease_start)+load.x_load_phase)+load.x_load_displacement)/(1+load.x_load_displacement));
		
	return xr;
}

float* setWalkX(float t_frac, walk_x x, int sequence,int lift_pause)
{
	if(sequence==0)
	{
		x_val[0]=x.lift.height;
		x_val[1]=x.lift.height;
	}
	else if(sequence==1)
	{
		if(lift_pause==0)
		x_val[0]=setWalkLift(t_frac,x.lift)+setWalkLand(t_frac,x.land);
		else 
		{
			x_val[0]=setWalkLand(t_frac,x.land);
			//printf("X %f ",x_val[0]);
		}
		x_val[1]=x.lift.height+setWalkLoad(t_frac,x.load);
	}
	else if(sequence==2)
	{
		x_val[0]=x.lift.height;
		x_val[1]=x.lift.height;
	}
	else 
	{
		x_val[0]=x.lift.height;
		x_val[1]=x.lift.height;
	}	

	return x_val;
		
}

float zshift_val=0;

float setWalkShift(float t_frac,walk_zshift zshift, int phase)
{
	/////Z SHIFT//////
	
	if(t_frac<=zshift.startTime && phase<2)
	zshift_val=0;
	else if(t_frac>zshift.startTime&&t_frac<=zshift.maxTime&&phase<2)
	zshift_val=zshift.zmax*(sin((zshift.maxPhase-zshift.startPhase)*(t_frac-zshift.startTime)/(zshift.maxTime-zshift.startTime)+zshift.startPhase)+zshift.displacement)/(1+zshift.displacement);
	//else if(t_frac>zshift.peak_max&&t_frac<=zshift.peak_stay)
	//zshift_val=zshift.zmax+4*(sin(3.14*((t_frac-zshift.peak_stay)/0.1)-1.57)+1)/2;
	else if(t_frac>zshift.stayTime&&t_frac<=zshift.endTime&&phase>=1)
	{
		printf("HER %10.5f\n",zshift.zmax);
		zshift_val=zshift.zmax*(sin((zshift.endPhase-zshift.stayPhase)*(t_frac-zshift.stayTime)/(zshift.endTime-zshift.stayTime)+zshift.stayPhase)+zshift.displacement)/(1+zshift.displacement)  +   zshift.decAmp*(sin(zshift.decFreq*((t_frac-zshift.stayTime)/zshift.decTime)+zshift.decPhase)+zshift.decDisplacement)/(1+zshift.decDisplacement);
	}
	else if(t_frac>zshift.endTime&&phase>1)
	{
		printf("here\n");
		zshift_val=zshift.decAmp*(sin(zshift.decFreq*((t_frac-zshift.stayTime)/zshift.decTime)+zshift.decPhase)+zshift.decDisplacement)/(1+zshift.decDisplacement);
	}
	return zshift_val;
}

float y_val[2];
	
float* setWalkY(float t_frac, walk_y y, float yin,float yfi, float yrin, float yrfi, int sequence)
{
	/*////Y MOVEMENT////
	if(t_frac<=y.y_start)
	y_val[0]=yin;  
        else if(t_frac>y.y_start&&t_frac<=y.y_end)
	y_val[0]=scurve2(yin,yfi,t_frac-y.y_start,y.y_end-y.y_start);
	else if(t_frac>y.y_end)
	y_val[0]=yfi;
	
	////Y MOVEMENT////
	if(t_frac<=y.yr_start)
	y_val[1]=yrin;  
        else if(t_frac>y.yr_start&&t_frac<=y.yr_end)
	y_val[1]=scurve2(yrin,yrfi,t_frac-y.yr_start,y.yr_end-y.yr_start);
	else if(t_frac>y.yr_end)
	y_val[1]=yrfi;
	*/
	if(sequence==0)
	{
		if(t_frac<=y.y_start)
		{
			y_val[0]=yin;
			y_val[1]=yrin;
		}
		else if(t_frac>y.y_start&&t_frac<=y.y_0)
		{
			y_val[0]=scurve2(yin,yin-yrin,t_frac-y.y_start,y.y_0-y.y_start);
			y_val[1]=scurve2(yrin,0,t_frac-y.y_start,y.y_0-y.y_start);
		}
		else if(t_frac>y.y_0)
		{
			y_val[0]=yin-yrin;
			y_val[1]=0;
		}
		else
		{
			printf("SEQ 1 Y\n");
		}
	}	
	else if(sequence==1)
	{
		if(t_frac<y.y_0)
		{
			y_val[0]=yin-yrin;
			y_val[1]=0;
		}
		else if(t_frac>y.y_0&&t_frac<=y.y_move)
		{
			y_val[0]=scurve2(yin-yrin,yfi-yrfi,t_frac-y.y_0,y.y_move-y.y_0);
			y_val[1]=0;
		}
		else if(t_frac>y.y_move)
		{
			y_val[0]=yfi-yrfi;
			y_val[1]=0;
		}
		else
		{
			printf("SEQ 2 Y\n");
		}
		
	}
	else if(sequence==2)
	{
		if(t_frac<=y.y_move)
		{
			y_val[0]=yfi-yrfi;
			y_val[1]=0;
		}
		else if(t_frac>y.y_move&&t_frac<=y.y_end)
		{
			y_val[0]=scurve2(yfi-yrfi,yfi,t_frac-y.y_move,y.y_end-y.y_move);
			y_val[1]=scurve2(0,yrfi,t_frac-y.y_move,y.y_end-y.y_move);
		}
		else if(t_frac>y.y_end)
		{
			y_val[0]=yfi;
			y_val[1]=yrfi;
		}
		else
		{
			printf("SEQ 2 Y\n");
		}
	}	
	return y_val;
}

float z_val[2];
	

float* setWalkZ(float t_frac,walk_z z, float zin, float zfi, float zrin,float zrfi, int sequence)
{
	if(sequence==0)
	{
		if(t_frac<=z.z_start)
		{	
			z_val[0]=zin;
			z_val[1]=zrin;
		//	printf("1-1\n");
		}	
		else if(t_frac>z.z_start&&t_frac<=z.z_0)	
		{
			z_val[0]=scurve(zin,zin+zrin,t_frac-z.z_start,z.z_0-z.z_start);
			z_val[1]=scurve(zrin,0,t_frac-z.z_start,z.z_0-z.z_start);
		//	printf("1-2\n");
		}
		else if(t_frac>z.z_0)
		{
			z_val[0]=zin+zrin;
			z_val[1]=0;
		//	printf("1-3\n");
		}
		
	}
	else if(sequence==1)
	{
		if(t_frac<z.z_0)
		{
			z_val[0]=zin+zrin;
			z_val[1]=0;
		//	printf("2-1\n");
		
		}
		else if(t_frac>z.z_0&&t_frac<=z.z_move)
		{
			z_val[0]=scurve(zin+zrin,zfi+zrfi,t_frac-z.z_0,z.z_move-z.z_0);
			z_val[1]=scurve(0,0,t_frac-z.z_0,z.z_move-z.z_0);
		//	printf("2-2\n");
		}
		else if(t_frac>z.z_move)
		{
			z_val[0]=zfi+zrfi;
			z_val[1]=0;
		//	printf("2-3\n");
		}
	}
	else if(sequence==2)
	{
		if(t_frac<z.z_move)
		{
			z_val[0]=zfi+zrfi;
			z_val[1]=0;
		//	printf("3-1\n");
		}
		else if(t_frac>z.z_move&&t_frac<=z.z_end)
		{
			z_val[0]=scurve(zrfi+zfi,zfi,t_frac-z.z_move,z.z_end-z.z_move);
			z_val[1]=scurve(0,zrfi,t_frac-z.z_move,z.z_end-z.z_move);
		//	printf("3-2\n");	
		}
		else if(t_frac>z.z_end)
		{
			z_val[0]=zfi;
			z_val[1]=zrfi;
		//	printf("3-3\n");
		}
	}
	return z_val;
	
}			
 	
float phi_val[2];

float* setTurnPhi(float t_frac,walk_turn turn, float thetain,float thetafi,float thetarin,float thetarfi, int sequence)
{
			
	if(sequence==0)
	{
		phi_val[0]=thetain;
		phi_val[1]=thetarin;
	}
	else if(sequence==1)
	{
		if(t_frac>turn.turn_start&&t_frac<=turn.turn_end)
		{
			phi_val[0]=scurve(thetain,thetafi,t_frac-turn.turn_start,turn.turn_end-turn.turn_start);
			phi_val[1]=scurve(thetarin,thetarfi,t_frac-turn.turn_start,turn.turn_end-turn.turn_start);
		}
	}
	else if(sequence>=2)
	{
		phi_val[0]=thetafi;
		phi_val[1]=thetarfi;
	}
		
	return phi_val;
}

float com_val[2];


	
float* setCOM(float t_frac, walk_com com, float yrin, float yrfi, int sequence)
{
	if(t_frac<=com.zstartTime&&sequence==0)
	com_val[0]=0;
	else if(t_frac>com.zstartTime&&t_frac<=com.zpeakTime&&sequence<=1)
	com_val[0]=com.zmax*(sin((com.zpeakPhase-com.zstartPhase)*(t_frac-com.zstartTime)/(com.zpeakTime-com.zstartTime)+com.zstartPhase));//0*(1-(t_frac-z_com_start)/(z_com_peak-z_com_start))+z_com_max*((t_frac-z_com_start)/(z_com_peak-z_com_start));
	else if(t_frac>com.zpeakTime&&t_frac<=com.zstayTime&&sequence==1)
	com_val[0]=com.zmax;//+z_static_inc*(sin(2*3.14*(t_frac-z_com_peak)/(z_com_stay-z_com_peak)-1.57)+1)/2;
	else if(t_frac>=com.zstayTime&&t_frac<=com.zendTime&&sequence>=2)
	com_val[0]=com.zmax*(sin((com.zendPhase-com.zstayPhase)*(t_frac-com.zstayTime)/(com.zendTime-com.zstayTime)+com.zstayPhase));
	else if(sequence==3)
	com_val[0]=0;
	
	
	if(t_frac<=com.y_com_start)
	com_val[1]=-yrin;
	else if(t_frac>com.y_com_start&&t_frac<=com.y_com_end)
	com_val[1]=-yrin-(-yrin+yrfi)*(t_frac-com.y_com_start)/(com.y_com_end-com.y_com_start);
	else
	com_val[1]=-yrfi;
	
	return com_val;
	
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

float calcMomentum(float *arr,int length)
{
	int i=0;
	int j=0;
	//const lenth=10;
	float diff[length][length];
	
	for(i=0;i<length;i++)
	diff[0][i]=arr[i];
	
	for(i=1;i<length;i++)
	{
		for(j=0;j<length-i;j++)
		{
			diff[i][j]=diff[i-1][j+1]-diff[i-1][j];
		}
	}
	
/*	for(i=1;i<2;i++)*/
/*	{*/
/*		for(j=0;j<1;j++)///length-i*/
/*		{*/
/*			printf("%10.5f",diff[i][j]);*/
/*			if(fabs(diff[i][j])>6)*/
/*			printf("***");*/
/*			else*/
/*			printf("   ");*/
/*			*/
/*		}*/
/*		//printf("\n");*/
/*	}*/
	return diff[1][0];
}


int identifyPhaseZ(float hip_z)
{
	int phase_z=-1;
	if(hip_z<5&&hip_z>-5)
	phase_z=0;
	else if (hip_z>=5&&hip_z<45)
	phase_z=1;
	else if(hip_z>=45)
	phase_z=2;
	else if(hip_z<-5&&hip_z>-57)
	phase_z=3;
	else if(hip_z<=-57)
	phase_z=4;
	else 
	phase_z=5;
	//printf("%10.5f %2d",hip_z,phase);
	//printf("%2d ",phase);
	return phase_z;
}

int identifyPhaseY(float hip_y)
{
	int phase_y=-1;
	if(hip_y<5&&hip_y>-5)
	phase_y=0;
	else if(hip_y>=5&&hip_y<100)
	phase_y=1;
	else if(hip_y>=100)
	phase_y=2;
	else if(hip_y<=-5&&hip_y>-85)
	phase_y=3;
	else if(hip_y<=-85)
	phase_y=4;
	
	return phase_y;
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


float cor_z[3]={0};
float cor_zr[3]={0};
float zcom[10]={0};
float ycom[10]={0};

/* Variables to calibrate
	
	correction P gains for all sequences 
	velocity contribution for wt shift 



	
*/	


enum{	LEFT=0,
	RIGHT=1};





class Acyut
{
	public:
	float left_x,left_y,left_z,left_rot;
	float right_x,right_y,right_z,right_rot;
	int grounded;
	int ik_val[33];
	
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
		
		return result;	left_x=390;
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
			printf("IMU READING FAILED\n");
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
		
		left_x=390;
		left_y=0;
		left_z=0;
		right_x=390;
		right_y=0;
		right_z=0;
		
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
		
		offset[0]=15;
		offset[1]=350-25-20-30; 
		offset[2]=275; 
		offset[3]=232+25;  
		offset[4]=0; 
		offset[5]=-15-30; 
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
		offset[25]=-15; 
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




enum components{BASEX,
		WTSHIFT,
		XLIFT,
		KEEP_WTSHIFT,
		KEEP_XLIFT,
		REVWTSHIFT,
		XLAND,
		COMP_LENGTH};

		
char* printComp(int comp)
{
	switch(comp)
	{
		case BASEX:return (char*)"BASEX";
		break;
		case WTSHIFT:return (char*)"WTSHIFT";
		break;
		case KEEP_WTSHIFT:return (char*)"KEEPWTSHIFT";
		break;
		case REVWTSHIFT:return (char*)"REVWTSHIFT";
		break;
		case XLIFT:return (char*)"XLIFT";
		break;
		case KEEP_XLIFT:return (char*)"KEEP_XLIFT";
		break;
		case XLAND:return (char*)"XLAND";
		break;
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


///,XABSORB,XRLIFT,

//union componentObjects{ char c;
//			int c;
//			};


//class wtShift;     /// FORWARD DECLARATIONS
//class revWtShift;
//class xLift;


class Walk
{
	public:
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
	
	float rotCom[3];
	float rotHip[3];
	float rotRevLeg[3];
	float hipVelo[3];
	
	
	
	Walk(Acyut *bot,int fps,int leg)
	{
		this->bot=bot;
		this->fps=fps;
		this->leg=leg;
		time=0;
		for (int i = 0; i < COMP_LENGTH; i += 1)
		{
			start[i].value=false;
			stop[i].value=false;
			runningComp[i]=false;
		}
		
		start[WTSHIFT].value=true;
		start[WTSHIFT].args[0]=45;
		start[WTSHIFT].args[1]=0;
		start[WTSHIFT].args[2]=PI/2;
		start[WTSHIFT].args[3]=0;
		start[WTSHIFT].args[4]=time;
		start[WTSHIFT].args[5]=0.5;
		start[BASEX].value=true;
		leg_x=390;
		leg_y=0;
		leg_z=0;
		leg_rot=0;
		revleg_x=390;
		revleg_y=0;
		revleg_z=0;
		revleg_rot=0;
		
		bot->projection_system();
		if(leg==LEFT)
		{
			for (int i = 0; i < 3; i += 1)
			{
				rotHip[i]=-(bot->rot_leftJoints[i]);
				prevHip[i]=-(bot->rot_leftJoints[i]);
				
				rotCom[i]=bot->rot_com[i]-bot->rot_leftJoints[i];
				rotRevLeg[i]=bot->rot_rightJoints[i]-bot->rot_leftJoints[i];
				hipVelo[i]=0;
			}
		}
		else if(leg==RIGHT)
		{
			for (int i = 0; i < 3; i += 1)
			{
				rotHip[i]=-(bot->rot_rightJoints[i]);
				prevHip[i]=-(bot->rot_rightJoints[i]);
				rotCom[i]=bot->rot_com[i]-bot->rot_rightJoints[i]*-1;
				rotRevLeg[i]=bot->rot_leftJoints[i]-bot->rot_rightJoints[i];
				hipVelo[i]=0;
			}
		}
		
		
	
		
		
		
	}
	
	void runComponents()
	{
		//usleep(500000);
		getWalkFrame();
		//printf("RAN COMPONENTS\n");
		//debug();
		updateResponse();
		//printf("RESPONSE COMPONENTS\n");
		//debug();
		doProjection();
		//printf("PROJECTION\n");
		//debug();
		
		//updateStartList();
		//printf("UPDATED START\n");
		//debug();
		//updateStopList();
		//printf("UPDATED STOP\n");
		//debug();
		updateRunList();
		//for (int i = 0; i < COMP_LENGTH; i += 1)
		//{
			//if(runningComp[i]==true)
			//evalComponent(i);
		//}
		debug();
		//debug();
		//printf("RESPONSE\n");
		//debug();
		//printf("%f\t %f\t %f\t %f\t %f\t %f\n",leg_x,leg_y,leg_z,revleg_x,revleg_y,revleg_z);
		
	}
	
	int walkRunning()
	{
		for (int i = 0; i < COMP_LENGTH; i += 1)
		{
			if(i==BASEX)
			continue;
			if(runningComp[i]==true||start[i].value==true)
			return 1;
		}
		return 0;
	}
	
	void debug()
	{
		printf("%s\t TIME %f\n",leg==LEFT?"LEFT":"RIGHT",time);
		for (int i = 0; i < COMP_LENGTH; i += 1)
		{
			printf("%10s\t %d\t %d\t %d\t %d\n",printComp(i),start[i].value,stop[i].value,runningComp[i],component[i].priority);
		}
//		sleep(1);
		
	}
	
	void incrementTime()
	{
		usleep(1000000.0/fps);
		time+=(float)1.0/(float)fps;
	}
	
	void setLeg(int leg)
	{
		this->leg=leg;
	}
	
	void syncBot()
	{
		printf("LEG %7.2f %7.2f %7.2f\n",leg_x,leg_y,leg_z);
		printf("RLEG   %7.2f %7.2f %7.2f\n",revleg_x,revleg_y,revleg_z);
		if(leg==LEFT)
		{
			bot->setLegCoods(leg_x,leg_y,leg_z,leg_rot,revleg_x,revleg_y,revleg_z,revleg_rot);
		}
		else if(leg==RIGHT)
		{
			bot->setLegCoods(revleg_x,revleg_y,revleg_z,revleg_rot,leg_x,leg_y,leg_z,leg_rot);
		}
		bot->updateFrame();
		bot->sendFrame();
	}

	bool isRunning(int componentID)
	{
		if(runningComp[componentID]==true&&stop[componentID].value==false)
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
			if(start[i].value==true)
			{
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
				printf("COMPONENT %s %s\n",printComp(component),printComp(i));
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
				updateStartList();
				updateStopList();
			}
		}
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
			rotHip[0]=-(bot->rot_leftJoints[0]);
			rotCom[0]=bot->rot_com[0]-bot->rot_leftJoints[0];
			rotRevLeg[0]=bot->rot_rightJoints[0]-bot->rot_leftJoints[0];
			rotHip[1]=-(bot->rot_leftJoints[1]);
			rotCom[1]=bot->rot_com[1]-bot->rot_leftJoints[1];
			rotRevLeg[1]=bot->rot_rightJoints[1]-bot->rot_leftJoints[1];
			rotHip[2]=-(bot->rot_leftJoints[2]);
			rotCom[2]=bot->rot_com[2]-bot->rot_leftJoints[2];
			rotRevLeg[2]=bot->rot_rightJoints[2]-bot->rot_leftJoints[2];
		}
		else if(leg==LEFT)
		{
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
		
		
		//printf("ROTHIP\t");
		for (int i = 0; i < 3; i += 1)
		{
			hipVelo[i]=(rotHip[i]-prevHip[i])*fps;
			prevHip[i]=rotHip[i];
		//	printf("%f\t",rotHip[i]);
		}
		//printf("\n");
		//printf("VELO\t");
//		for (int i = 0; i < 3; i += 1)
//		{
//		//	printf("%f \t",hipVelo[i]);
//		}	
//		printf("\n");
		
		
	}
	
	
	
};



class wtShift
{
	
	public:
	
	float zmax;
	float startTime;
	float startPhase;
	float maxTime;
	float maxPhase;
	float displacement;
	Walk *walk_object;	
	float *time;
	
	private :
	float predictedExpZ;
	float  prevExpZ;
	float finalZ;
	int posWtShift;
	int veloWtShift;
	
	public :
	wtShift(Walk *walk_object,float zmax,float startPhase,float maxPhase,float displacement,float startTime,float runTime,float *time)
	{
		this->zmax=zmax;
		this->startTime=startTime;
		this->startPhase=startPhase;
		this->maxTime=startTime+runTime;
		this->maxPhase=maxPhase;
		this->displacement=displacement;
		this->walk_object=walk_object;
		this->time=time;
		predictedExpZ=0;
		prevExpZ=0;
		finalZ=0;
	}
	
	int evaluate()
	{
		if(*time>startTime&&*time<maxTime)
		{
			walk_object->leg_z=zmax*(sin((maxPhase-startPhase)*(*time-startTime)/(maxTime-startTime)+startPhase)+displacement)/(1+displacement);
			walk_object->revleg_z=-zmax*(sin((maxPhase-startPhase)*(*time-startTime)/(maxTime-startTime)+startPhase)+displacement)/(1+displacement);
		}
		//printf("%f\n",walk_object->leg_z);
		//if(*time>maxTime)
		//exit(0);
	}
	
	ComponentResponse* response(ComponentResponse *response)
	{
//		if(walk_object->leg_z>35&&walk_object->isRunning(XLIFT)==false)
//		{
//			response->O_comps[XLIFT].start=true;
//		}
		veloWtShift=getPhase(HipZ());
		posWtShift=getPhase(walk_object->rotHip[1]);
		
		if(veloWtShift<=0)
		{
			response->start[XLIFT].value=true;
			response->start[XLIFT].args[0]=40;
			response->start[XLIFT].args[1]=-PI/2;    ///TODO convert this to function of wtshift to enable max step length time
			response->start[XLIFT].args[2]=PI/2;    ///TODO convert this to function of wtshift to enable max step length time
			response->start[XLIFT].args[3]=1;    ///TODO convert this to function of wtshift to enable max step length time
			response->start[XLIFT].args[4]=*time;    ///TODO convert this to function of wtshift to enable max step length time
			response->start[XLIFT].args[5]=0.5;    ///TODO convert this to function of wtshift to enable max step length time
			
			
			response->start[KEEP_WTSHIFT].value=true; // KEEPWTSHIFT to increase or maintain constant
			//if(posWtShift<=0)
			{
				
				//keepWtShift(Walk *walk_object,float option,float startPhase,float maxPhase,float displacement,float zmax,float runTime,float *time)
				response->start[KEEP_WTSHIFT].args[0]=1;
				response->start[KEEP_WTSHIFT].args[1]=(maxPhase-startPhase)*(*time-startTime)/(maxTime-startTime);
				response->start[KEEP_WTSHIFT].args[2]=maxPhase;
				response->start[KEEP_WTSHIFT].args[3]=displacement;
				response->start[KEEP_WTSHIFT].args[4]=zmax*(sin((maxPhase-startPhase)*(*time-startTime)/(maxTime-startTime)+startPhase)+displacement)/(1+displacement);
				response->start[KEEP_WTSHIFT].args[5]=maxTime-*time;
			}
			//else
			//{
			//	response->start[KEEP_WTSHIFT].args[0];
				
			//}
				
			response->stop[WTSHIFT].value=true;
//			response->start[KEEPWTSHIFT].args[0]=0;
//			response->start[KEEPWTSHIFT].args[1]=(maxPhase-startPhase)*(*time-startTime)/(maxTime-startTime);
//	keepWtShift(Walk *walk_object,float option,float startPhase,float maxPhase,float displacement,float zmax,float runTime,float *time)
		}

//		if(*time>maxTime)
//		{
//			response->start[REVWTSHIFT].value=true;
//			response->start[REVWTSHIFT].args[0]=45;
//			response->start[REVWTSHIFT].args[1]=PI/2;
//			response->start[REVWTSHIFT].args[2]=PI;
//			response->start[REVWTSHIFT].args[3]=0;
//			response->start[REVWTSHIFT].args[4]=*time;
//			response->start[REVWTSHIFT].args[5]=0.5;
//			response->stop[WTSHIFT].value=true;	
//		}
	}
	
	private:
	
	int getPhase(float hip_z)
	{
		int phase_z=0;
		if(hip_z<7&&hip_z>-7)
		phase_z=0;
		else if (hip_z>=7&&hip_z<45)
		phase_z=1;
		else if(hip_z>=45)
		phase_z=2;
		else if(hip_z<-7&&hip_z>-57)
		phase_z=-1;
		else if(hip_z<=-57)
		phase_z=-2;
		//printf("%10.5f %2d\n",hip_z,phase_z);
		//printf("%2d ",phase);
		return phase_z;	
	}
	
	
	float HipZ()
	{
		float expHip_z=walk_object->rotHip[1]+pow(walk_object->hipVelo[1],2)*signum(walk_object->hipVelo[1])/9800;
		prevExpZ=finalZ;
		finalZ=(0.5*predictedExpZ+0.5*expHip_z);
		predictedExpZ=finalZ+(finalZ-prevExpZ);
		
		//printf("EXPHIPZ %f %f %f\n",expHip_z,predictedExpZ,finalZ);
		
		return finalZ;
	
	}
};



class keepWtShift
{
	public:
	
	Walk *walk_object;
	int option;
	float startPhase;
	float maxPhase;
	float displacement;
	float zmax;
	float runTime;
	float *time;
	
	private :
	float currentPhase; 
	float startTime;
	public:
	keepWtShift(Walk *walk_object,float option,float startPhase,float maxPhase,float displacement,float zmax,float runTime,float *time)
	{
		this->walk_object=walk_object;
		this->option=(int)option;
		this->startPhase=startPhase;
		this->maxPhase=maxPhase;
		this->displacement=displacement;
		this->zmax=zmax;
		this->runTime=runTime;
		this->time=time;
		currentPhase=startPhase;
		startTime=*time;
	}

	int evaluate()
	{
		switch(option)
		{
				/// Case 0 is NOT complete	
			case 0 : if(currentPhase>maxPhase)
				{
					option=1;	
				}
				else
				{
					currentPhase=(maxPhase-startPhase)*(*time-startTime)/(runTime)+startPhase;
					walk_object->leg_z=zmax*(sin(currentPhase)+displacement)/(1+displacement);
					walk_object->revleg_z=-zmax*(sin(currentPhase)+displacement)/(1+displacement);
				}
			break;
		
			case 1 :
				walk_object->leg_z=zmax;//*(sin(maxPhase)+displacement)/(1+displacement);
				walk_object->revleg_z=-zmax;//*(sin(maxPhase)+displacement)/(1+displacement);
			break;
			
			default: printf("SOMEFINK WONG\n");
				break;
		
		}
	}
	
	ComponentResponse* response(ComponentResponse *response)
	{
		if(walk_object->isRunning(XLIFT)==false)
		{
			//sleep(2);
			//response->start[XLAND].value=true;
			//response->start[XLAND].args[4]=*time;
			response->start[REVWTSHIFT].value=true;
			response->start[REVWTSHIFT].args[0]=zmax;
			response->start[REVWTSHIFT].args[1]=currentPhase;
			response->start[REVWTSHIFT].args[2]=PI;
			response->start[REVWTSHIFT].args[3]=displacement;
			response->start[REVWTSHIFT].args[4]=*time;
			response->start[REVWTSHIFT].args[5]=0.5;
			response->stop[KEEP_WTSHIFT].value=true;
		}
		
	
	}

};



class revWtShift
{
	public :
	float zmax;
	float maxTime;
	float maxPhase;
	float endTime;
	float endPhase;
	float displacement;
	Walk *walk_object;
	float *time;
	
	revWtShift( Walk *walk_object,float zmax,float maxPhase,float endPhase,float displacement,float startTime,float runTime,float *time)
	{
		this->zmax=zmax;
		this->maxTime=startTime;
		this->maxPhase=maxPhase;
		this->endTime=startTime+runTime;
		this->endPhase=endPhase;
		this->displacement=displacement;
		this->walk_object=walk_object;
		this->time=time;
	}
	
	~revWtShift()
	{
		
	
	}
	
	
	int evaluate()
	{
		if(*time>maxTime)
		{
			walk_object->leg_z=zmax*(sin((endPhase-maxPhase)*(*time-maxTime)/(endTime-maxTime)+maxPhase)+displacement)/(1+displacement);
			walk_object->revleg_z=-zmax*(sin((endPhase-maxPhase)*(*time-maxTime)/(endTime-maxTime)+maxPhase)+displacement)/(1+displacement);
		}
	}
	
	ComponentResponse* response(ComponentResponse *response)
	{
		if(*time>endTime)
		{
			response->stop[REVWTSHIFT].value=true;
			
			response->start[WTSHIFT].value=true;
			response->start[WTSHIFT].args[0]=45;
			response->start[WTSHIFT].args[1]=0;
			response->start[WTSHIFT].args[2]=PI/2;
			response->start[WTSHIFT].args[3]=0;
			response->start[WTSHIFT].args[4]=*time;
			response->start[WTSHIFT].args[5]=0.5;
			
			if(walk_object->leg==LEFT)
			walk_object->setLeg(RIGHT);
			else if(walk_object->leg==RIGHT)
			walk_object->setLeg(LEFT);
			
		}
	}
	
};


class xLift
{
	public:
	
	float lift;
	float startPhase;
	float startTime;
	float peakPhase;
	float peakTime;
	float displacement;
	float *time;
	Walk *walk_object;
	
	xLift(Walk *walk_object,float lift,float startPhase,float peakPhase,float displacement,float startTime,float tot_time,float *time)
	{
		this->walk_object=walk_object;
		this->lift=lift;
		this->startPhase=startPhase;
		this->peakPhase=peakPhase;
		this->displacement=displacement;
		this->startTime=startTime;
		this->peakTime=startTime+tot_time;
		this->time=time;
	}

	int evaluate()
	{
		if(*time>startTime)
		walk_object->leg_x-=lift*(sin((peakPhase-startPhase)*(*time-startTime)/(peakTime-startTime)+startPhase)+displacement)/(1+displacement);
		//walk_object->leg_x=0;
		//if(*time>startTime)
		//walk_object->leg_x-=20;
	}
	
	ComponentResponse* response(ComponentResponse *response)
	{
		if(*time>peakTime)
		{
			//printf("XLIFT STOPPED SELF\n");
			response->stop[XLIFT].value=true;
			response->start[XLAND].value=true;
			response->start[XLAND].args[0]=40;
			response->start[XLAND].args[1]=PI/2;
			response->start[XLAND].args[2]=3*PI/2;
			response->start[XLAND].args[3]=1;//displacement;
			response->start[XLAND].args[4]=*time;
			response->start[XLAND].args[5]=0.5;
	
		}
	}
};


class xLand
{
	public:
	
	float lift;
	float peakPhase;
	float peakTime;
	float endPhase;
	float endTime;
	float displacement;
	float *time;
	Walk *walk_object;
	
	
	xLand(Walk *walk_object,float lift,float peakPhase,float endPhase,float displacement,float peakTime,float runTime,float *time)
	{
		this->lift=lift;
		this->peakPhase=peakPhase;
		this->peakTime=peakTime;
		this->endPhase=endPhase;
		this->endTime=peakTime+runTime;
		this->displacement=displacement;
		this->time=time;
		this->walk_object=walk_object;
		
		
		//printf("LIFT %f\n peakphase %f\n peakTime %f\n endPhase %f\n endTime %f\n displacement %f\n",this->lift,this->peakPhase,this->peakTime,this->endPhase,this->endTime,this->displacement);
		//printf("LIFT %f\n peakphase %f\n peakTime %f\n endPhase %f\n endTime %f\n displacement %f\n",lift,peakPhase,peakTime,endPhase,endTime,displacement);
	} 

	int evaluate()
	{
		if(*time>=peakTime)
		walk_object->leg_x-=lift*(sin((endPhase-peakPhase)*(*time-peakTime)/(endTime-peakTime)+peakPhase)+displacement)/(1+displacement);
	}
	
	ComponentResponse* response(ComponentResponse *response)
	{
		if(*time>endTime)
		{
			response->stop[XLAND].value=true;
		}
	}
};


class keepXLift
{

	public:
	
	


};




class baseX
{
	public: 
	
	float height;
	Walk *walk_object;
	float *time;
	bool fFrame;
	baseX(Walk *walk_object,float height,float *time)
	{
		this->walk_object=walk_object;
		this->time=time;
		this->height=height;
		fFrame=true;
	}
	
	int evaluate()
	{
		walk_object->leg_x=height;
		walk_object->revleg_x=height;
		if(fFrame==true)
		{
//			compliance(
			fFrame=false;
		}
		
		
	}	
	
	ComponentResponse* response(ComponentResponse *response)
	{
	
	}
};


int Walk::createComponent(int component)
{
	//printf("CREATE COMP\n");
	switch(component)
	{
		case BASEX:	this->component[BASEX].pointer = new baseX(this,390,&time);
				this->component[BASEX].priority=givePriority(BASEX);
				break;
		case WTSHIFT :	this->component[WTSHIFT].pointer = new wtShift(this,start[WTSHIFT].args[0],start[WTSHIFT].args[1],start[WTSHIFT].args[2],start[WTSHIFT].args[3],start[WTSHIFT].args[4],start[WTSHIFT].args[5],&time);
				this->component[WTSHIFT].priority=givePriority(WTSHIFT);
				break;
		case KEEP_WTSHIFT : this->component[KEEP_WTSHIFT].pointer = new keepWtShift(this,start[KEEP_WTSHIFT].args[0],start[KEEP_WTSHIFT].args[1],start[KEEP_WTSHIFT].args[2],start[KEEP_WTSHIFT].args[3],start[KEEP_WTSHIFT].args[4],start[KEEP_WTSHIFT].args[5],&time);
				this->component[KEEP_WTSHIFT].priority=givePriority(KEEP_WTSHIFT);
				break;
		case REVWTSHIFT : this->component[REVWTSHIFT].pointer = new revWtShift(this,start[REVWTSHIFT].args[0],start[REVWTSHIFT].args[1],start[REVWTSHIFT].args[2],start[REVWTSHIFT].args[3],start[REVWTSHIFT].args[4],start[REVWTSHIFT].args[5],&time);
				this->component[REVWTSHIFT].priority=givePriority(REVWTSHIFT);
				break;
		case XLIFT : 	this->component[XLIFT].pointer = new xLift(this,start[XLIFT].args[0],start[XLIFT].args[1],start[XLIFT].args[2],start[XLIFT].args[3],start[XLIFT].args[4],start[XLIFT].args[5],&time);
				this->component[XLIFT].priority=givePriority(XLIFT);
				break;
		case KEEP_XLIFT : this->component[KEEP_XLIFT].pointer = new keepXLift();
				this->component[KEEP_XLIFT].priority=givePriority(KEEP_XLIFT);
				break;
		case XLAND : 	this->component[XLAND].pointer = new xLand(this,start[XLAND].args[0],start[XLAND].args[1],start[XLAND].args[2],start[XLAND].args[3],start[XLAND].args[4],start[XLAND].args[5],&time);
				this->component[XLAND].priority=givePriority(XLAND);
				break;
		default : //printf("UNDER CONSTRUCTION!!!!!\n");
				break;
	}
	getPriorityTable();
	return EXIT_SUCCESS;

}

int Walk::evalComponent(int component)
{
	///printf("EVAL COMP\n");
	switch(component)
	{
		case BASEX:if(runningComp[component]==true)
				((baseX*)(this->component[BASEX].pointer))->evaluate();
			break;
		case WTSHIFT : if(runningComp[component]==true)
				((wtShift*)(this->component[WTSHIFT].pointer))->evaluate();
			break;
		case KEEP_WTSHIFT : if(runningComp[component]==true)
				((keepWtShift*)(this->component[KEEP_WTSHIFT].pointer))->evaluate();//printf("UNDER CONSTRUCTION!!!!!\n");
			break;
		case REVWTSHIFT : if(runningComp[component]==true)
				((revWtShift*)(this->component[REVWTSHIFT].pointer))->evaluate();
			break;
		case XLIFT : if(runningComp[component]==true)
				((xLift*)(this->component[XLIFT].pointer))->evaluate();
			break;
		case KEEP_XLIFT : //printf("UNDER CONSTRUCTION!!!!!\n");
			break;
		case XLAND : if(runningComp[component]==true)
				((xLand*)(this->component[XLAND].pointer))->evaluate();
			break;
		default : //printf("UNDER CONSTRUCTION!!!!!\n");
			break;
	}
	
	return EXIT_SUCCESS;
		//return pointer;	
}
	
int Walk::deleteComponent(int component)
{
	//printf("DELETE COMP\n");
	switch(component)
	{
		case BASEX : if(runningComp[component]==true)
			delete (baseX*)(this->component[component].pointer);
			this->component[component].priority=-1;

		case WTSHIFT : if(runningComp[component]==true)
				delete (wtShift*)(this->component[component].pointer);
				this->component[component].priority=-1;
			break;

		case KEEP_WTSHIFT : if(runningComp[component]==true)
				delete (keepWtShift*)(this->component[component].pointer);
				this->component[component].priority=-1;//printf("UNDER CONSTRUCTION!!!!!\n")
			break;

		case REVWTSHIFT : if(runningComp[component]==true)
				delete (revWtShift*)(this->component[component].pointer);
				this->component[component].priority=-1;
			break;

		case XLIFT : if(runningComp[component]==true)
			delete (xLift*)(this->component[component].pointer);
			this->component[component].priority=-1;
			break;

		case KEEP_XLIFT : //printf("UNDER CONSTRUCTION!!!!!\n");
			break;
		case XLAND : if(runningComp[component]==true)
			delete (xLand*)(this->component[component].pointer);
			this->component[component].priority=-1; 
			break;
		default : //printf("UNDER CONSTRUCTION!!!!!\n");
			break;
	}
	getPriorityTable();
	return EXIT_SUCCESS;
}

int Walk::getResponse(int component)
{
	//printf("RESPOSNSE COMP\n");
	switch(component)
	{
		case BASEX   : if(runningComp[component]==true)
				((baseX*)(this->component[component].pointer))->response(&(this->component[component].response));
				break;
				
		case WTSHIFT : 	//printf("POLLING WTSHIFT\n");
				if(runningComp[component]==true)
				((wtShift*)(this->component[component].pointer))->response(&(this->component[component].response));
			break;
		case KEEP_WTSHIFT : if(runningComp[component]==true)
				((keepWtShift*)(this->component[component].pointer))->response(&(this->component[component].response));
				break;
		case REVWTSHIFT :if(runningComp[component]==true)
				((revWtShift*)(this->component[component].pointer))->response(&(this->component[component].response));
			break;
		case XLIFT : if(runningComp[component]==true)
				((xLift*)(this->component[component].pointer))->response(&(this->component[component].response));
			break;
		case KEEP_XLIFT : //printf("UNDER CONSTRUCTION!!!!!\n");
			break;
		case XLAND : if(runningComp[component]==true)
				((xLand*)(this->component[component].pointer))->response(&(this->component[component].response));
			break;
		default : //printf("UNDER CONSTRUCTION!!!!!\n");
			break;
	}
	
	return EXIT_SUCCESS;
}

//inline int walk6(int leg,float veloY,float veloZ,float omega)
//{
//	Walk walk1=new Walk(&bot);
	
//}

/*
inline int walk5(int leg,float yin,float yfi,float yrin,float yrfi, float zin, float zfi,float zrin, float zrfi, float thetain, float thetafi,float thetarin, float thetarfi)
{
	printf("******* WALK LEG %2d **********\n",leg); 
	/////// Sequence Control /////
	int sequence=setupWalk(leg);
	int sequence_count=0;
	
	float time=walk.time;
	float exp_time=time;
	float fps=walk.fps;
	
	walk_x x_control;
	walk_zshift zshift;
	walk_y y_control;
	walk_z z_control;
	walk_turn turn;
	walk_misc misc;
	walk_com com;
	
	if(leg==0)
	{
		x_control= walk.left.x;
		zshift = walk.left.z_shift;
		y_control = walk.left.y;
		z_control = walk.left.z;
		turn = walk.left.turn;
		misc = walk.left.misc;
		com = walk.left.com_trajectory;
	}
	else if(leg==1)
	{
		x_control = walk.right.x;
		zshift = walk.right.z_shift;
		y_control = walk.right.y;
		z_control = walk.right.z;
		turn = walk.right.turn;
		misc = walk.right.misc;
		com = walk.right.com_trajectory;
	}
	
		
	/////// Live Variables /////
	int frame=0;
	float t_frac=0,t=0;
	float x,xr,y,z,yr,zr,z_shift,phi,phir;
	float yreal,yrreal,zreal,zrreal;		
	
	/////// Packet Generation Variables /////
	int adchksum=7+NOM*3,i=0;
	float *fval,*retval;
	float val[5],valr[5];
	int mot=0*(1-leg)+20*leg;
	int revmot=20*(1-leg)+0*leg;
	int value=0;
	
	//// Gen Variables
	int walk_i=0;
	int target_i=0;
	
	//// Correction variables
	int touchdown_count=0;
	int lift=0;
	float land_angle[2]={0};
	float feet_orient[2]={0};
	compliance(leg,7,7);	
	
	///// ADD XR /////
	float prev_shift=0;
	int seq2_count=0;
	
	int seq_change=0;
	float diff_shift=0;
	int phase_z=-1;
	int phase_y=-1;
	int use_imu=1;
	float speed=1.0;
	float gain_i=0.98;
	float seq_crit=0;
	float add_z=0;
	int step_down=0;
	float hipendcoods[3];
	float rot_hipendcoods[3];
	float temp[3],temp2[3];
	float save=0;
	
	int walk_end=0;
	int set_land=0;
	int com_i=0;
	
	float hip_velo[3]={0};
	float rad[3]={0};
	float prev_rot_hip[3]={0};
	float Omega[3]={0};
	float Alpha[3]={0};
	float prevOmega[3]={0};
	
	float set_point[3]={0};
	float resultIK[3]={0};
	for(frame=0;walk_end==0;frame++) 
 	{
		printf("S %1d  ",sequence);
		for(target_i=0;target_i<6;target_i++)
		{
			target_left[target_i]=motor_val[target_i]-offset[target_i];
			target_right[target_i]=motor_val[20+target_i]-offset[20+target_i];
		}
	 	projection_system(leg);
		
		for(com_i=9;com_i>0;com_i--)
		{
			zcom[com_i]=zcom[com_i-1];
			ycom[com_i]=ycom[com_i-1];
		}
		
		hip_velo[0]=(rot_hip[0]-prev_rot_hip[0])/(1000)*fps;
		prev_rot_hip[0]=rot_hip[0];
		rad[0]=rot_hip[0]/1000;
		
		hip_velo[1]=(rot_hip[1]*pow(-1,leg+1)-prev_rot_hip[1])/(1000)*fps;
		prev_rot_hip[1]=rot_hip[1]*pow(-1,leg+1);
		rad[1]=rot_hip[1]/1000*pow(-1,leg+1)+(-50/1000)*(1-leg)+50/1000*leg;
		
		hip_velo[2]=(rot_hip[2]-prev_rot_hip[2])/(1000)*fps;
		prev_rot_hip[2]=rot_hip[2];
		rad[2]=rot_hip[2]/1000;
		
		
		prevOmega[0]=Omega[0];
		prevOmega[1]=Omega[1];
		prevOmega[2]=Omega[2];
		
		
		vectorCrossProduct(rad,hip_velo,Omega);
		vectorScalarMultiply(Omega,180/(PI*pow(vectorMagnitude(rad),2)),Omega);
		//printf("Velo  %10.5f %10.5f %10.5f ",hip_velo[0],hip_velo[1],hip_velo[2]);
		//printf("Radius  %10.5f %10.5f %10.5f ",rad[0],rad[1],rad[2]);
		
		Alpha[0]=Omega[0]-prevOmega[0];
		Alpha[1]=Omega[1]-prevOmega[1];
		Alpha[2]=Omega[2]-prevOmega[2];
		
		vectorScalarMultiply(Alpha,fps,Alpha);
		
		//printf("Omega  %10.5f %10.5f %10.5f ",Omega[0],Omega[1],Omega[2]);
		//printf("\tMAG R %10.5f ",vectorMagnitude(rad));
		//printf("\tMAG OMEGA %10.5f ",vectorMagnitude(Omega));
		//printf("CRIT %10.5f ",pow(vectorMagnitude(rad),2)*vectorMagnitude(Omega));
		//printf("Omega Z %10.5f Alpha %10.5f  %10.5f\n",Omega[2],Alpha[2],z_shift);
		//printf("\n");
		//printf("Alpha %10.5f %10.5f %10.5f\n",Alpha[0],Alpha[1],Alpha[2]);
		zcom[0]=rot_hip[1]*pow(-1,leg+1);
		ycom[0]=rot_hip[2];
		
		
		phase_z=identifyPhaseZ(((rot_hip[1]+(-50)*(1-leg)+50*leg)*pow(-1,leg+1)));
		phase_y=identifyPhaseY(rot_hip[2]);
		//printf("PHASE %2d %2d",phase_z,phase_y);
		
		retval=setWalkX(t_frac,x_control,sequence,step_down);
		x=retval[0];
		xr=retval[1];//+add_xr;
		
		prev_shift=z_shift;
		z_shift=setWalkShift(t_frac,zshift,sequence);
		
		diff_shift=z_shift-prev_shift;
		//printf("Zshift %10.5f ",z_shift);
		
		retval=setWalkY(t_frac,y_control,yin,yfi,yrin,yrfi,sequence);
		y=retval[0];
		yr=retval[1];
		
		retval=setWalkZ(t_frac,z_control,zin,zfi,zrin,zrfi,sequence);
		z=retval[0];
		zr=retval[1];
		
		retval=setTurnPhi(t_frac,turn,thetain,thetafi,thetarin,thetarfi,sequence);
		phi=retval[0];
		phir=retval[1];
		
		retval=setCOM(t_frac,com,yrin,yrfi,sequence);
		com_z=com.zmax-retval[0];
		com_y=retval[1];
		
		
		
		if(sequence==0)
		{
			err_z=com_z-((rot_hip[1]+(-50)*(1-leg)+50*leg)*pow(-1,leg+1));
			err_y=com_y-rot_hip[2];
//			printf("ERR_Z %12.8f ",err_z);//printf("ERR_Y %3.7f\tERR_Z %3.7f\t%3.7f\t%3.7f\t",err_y,err_z,(rot_hip[1]+(-50)*(1-leg)+50*leg)*pow(-1,leg+1),com_z);
			//printf("ERR_Y %12.7f\t%12.7f\t%12.7f\t%12.7f\t",err_y,com_y,rot_hip[2],rot_com[2]-9.2733);
			//if(err_z>10||err_z<-5)
			cor_zr[sequence]=cor_zr[sequence]*gain_i+0.15*err_z;
			cor_z[sequence]=cor_z[sequence]*gain_i-0.15*err_z;
			if(err_y>5||err_y<-5)
			cor_y+=0.5*err_y;
			
			//feet_orient[0]=0.8*current_angle[0];
			//feet_orient[1]=0.8*current_angle[1]*pow(-1,leg);
			seq_crit=(rot_hip[1]*pow(-1,leg)-pow(hip_velo[1]*1000,2)*signum(hip_velo[1])/9000);
			//printf("CRIT %10.5f ",seq_crit);
			//printf("ROT HIP %10.5f ",rot_hip[1]*pow(-1,leg));//if(inRange(err_z,-5,10)==1&&(phase==0))//com_z<15//&&inRange(err_y,-5,5)==1)  //
			if(seq_crit>45)//inRange(err_z,-5,10)==1&&
			sequence_count++;
			else
			sequence_count=0;
			
			if(sequence_count==2||(use_imu==0&&x_control.lift.x_start<t_frac))//||(compare_float(t_frac,lift.x_start)))
			//if(t_frac>0.2)
			{
				cor_z[sequence+1]=cor_z[sequence];
				cor_zr[sequence+1]=cor_z[sequence];
				//sequence++;
				seq_change=1;
				//printf("************ SEQ ************\n");	
				x_control.lift.x_end=t_frac+(x_control.lift.x_end-x_control.lift.x_start);
				x_control.lift.x_start=t_frac;
				
				y_control.y_0=t_frac;
				y_control.y_move=x_control.lift.x_end;
				
				z_control.z_0=t_frac;
				z_control.z_move=x_control.lift.x_end;
				
				turn.turn_start=t_frac;
				turn.turn_end=x_control.lift.x_end;
				
				if(use_imu==1)
				{
				
					zshift.zmax=z_shift;
					zshift.maxTime=t_frac;
					zshift.endTime=x_control.lift.x_end+(zshift.endTime-zshift.stayTime);
					zshift.stayTime=x_control.lift.x_end;
					
				}
				//y_control.y_0=t_frac+(y_control.y_0-y_control.y_start);
				//y_control.y_move=t_frac+(y_control.y_move-y_control.y_start);
				//y_control.y_end=t_frac+(y_control.y_end-y_control.y_start);
				//y_control.y_start=t_frac;
				
				
				touchdown_count=0;
			}
			
		}
		else if(sequence==1)
		{
			//if(lift==0)
			err_z=com_z-(rot_hip[1]+(-50)*(1-leg)+50*leg)*pow(-1,leg+1);
			//else
			//err_z=com_z-(rot_hip[1]+(-50)*(1-leg)+50*leg)*pow(-1,leg+1);
			err_y=rot_hip[2];
			
			//if(touchdown==10&&lift==0)
			//touchdown_count++;
			//else if(lift==0)
			//touchdown=0;
			//else if(lift==1&&touchdown<=5)
			//touchdown_count++;


			//if(touchdown_count==5&&lift==0)
			//{
			//	lift=1;
			//	touchdown_count=0;
			//}
			//else if(touchdown_count==5&&lift==1)
			//{
			//	sequence++;
			//	
			//	
			//}
			
			
			if(abs(rot_rleg[0]-x_control.lift.lift/2)<=4&&lift==0)
			{
				lift=1;
				//zshift.peak_end=t_frac+(zshift.peak_end-zshift.peak_stay);
				//zshift.peak_stay=t_frac;
				//com.z_com_stay=zshift.peak_stay;
				//com.z_com_stay=zshift.peak_end;
				//printf("HREExsdcascacadscascascasasfadsajhskjanaslfaflkj\n\n\n");
			}
				
			if(lift==1&&touchdown<10)
			touchdown_count++;
			else
			touchdown_count=0;
			
			//printf("lift %d touch %d\n",lift,touchdown_count);
			
			if((touchdown_count>=5&&lift==1&&(step_down==0||x_control.land.x_land_end<t_frac))||(x_control.lift.x_end<t_frac&&use_imu==0))
			{
				cor_z[sequence+1]=cor_z[sequence];
				cor_zr[sequence+1]=cor_z[sequence];
				//printf("************ SEQ ************\n");	
				seq_change=1;
				//sequence++;
				
				if(use_imu==1)
				{
					zshift.endTime=t_frac+(zshift.endTime-zshift.stayTime);
					zshift.stayTime=t_frac;
					zshift.zmax=z_shift;
				}
				com.zstayTime=zshift.stayTime;
				com.zendTime=zshift.endTime;
				
				compliance(leg,40,32);
			
				x_control.land.x_land_start=t_frac;
				x_control.land.x_land_increase_end=t_frac+(zshift.endTime-zshift.stayTime)/2;
				x_control.land.x_land_decrease_start=t_frac+(zshift.endTime-zshift.stayTime)/2;
				x_control.land.x_land_end=t_frac+(zshift.endTime-zshift.stayTime);
				x_control.land.x_land_freq=PI;
				x_control.land.x_land_displacement=1;
				x_control.land.x_land_phase=-PI/2;
				x_control.land.x_land_dec=6;
				x_control.land.x_land_base=0;
					
				y_control.y_move=t_frac;
				y_control.y_end=zshift.endTime;
				
				z_control.z_move=t_frac;
				z_control.z_end=zshift.endTime;
				
				
				sequence_count=0;
				
				
				//cor_z=-cor_zr;
			}
			if(inRange(err_z,-50,70)==0)
			{
				cor_z[sequence]=cor_z[sequence]*gain_i-0.1*(err_z);
				cor_zr[sequence]=cor_zr[sequence]*gain_i+0.1*(err_z);
				//if(lift==0)
				
				//mag_xr=abs(0.5*(390-x-rot_rleg[0]));
				//printf("ERR_XX %10.5f\n",1.5*(390-x-rot_rleg[0]));
				//if(mag_xr>7)
				//mag_xr=7;
				//else if(mag_xr<-7)
				//mag_xr=-7;
			}
			cor_y=0;
			cor_yr=0;
			////// CAPTURE STEP ///////
			
			//if(x<385)        for testing 
			//phase_z=2;        for testing 
			//if(phase_z==2)
			//{
			//	step_down=1;
			//	printf("******** STEP DOWN ***********\n");
			//	seq_crit=(rot_hip[1]*pow(-1,leg)+pow(momn_z,2)*signum(momn_z));				
			//	//printf("PH %2d SEQ CRIT %10.5f ",phase_z,seq_crit);		
			//	//speed=1.5;
			//	hipendcoods[0]=hip[0];
			//	hipendcoods[1]=hip[1]+(-100)*(1-leg)+(100)*leg;
			//	hipendcoods[2]=hip[2];
			//	rotateToIMU(hipendcoods,rot_hipendcoods);
			//	if(touchdown>=10)
			//	add_z=1.1*abs(rot_hipendcoods[0]*sin(current_angle[1]*3.14/180));
			//	
			//	if(touchdown<=10&&set_land==0)   //&&lift==1&&
			//	{
			//		set_land=1;
			//		x_control.land.x_land_start=t_frac;
			//		x_control.land.x_land_increase_end=t_frac+0.07;
			//		x_control.land.x_land_decrease_start=t_frac+0.5;
			//		x_control.land.x_land_end=t_frac+1;
			//		x_control.land.x_land_freq=PI;
			//		x_control.land.x_land_displacement=1;
			//		x_control.land.x_land_phase=-PI/2;
			//		x_control.land.x_land_dec=6;
			//		x_control.land.x_land_base=x;
			//		
			//		printf(" SETTING LAND !!!!\n");
			//	}
			//	
			//	//save=add_z;
			//	//seq_change=1;
			//}
			
			land_angle[0]=0.8*current_angle[0];
			land_angle[1]=current_angle[1]*pow(-1,leg);
			feet_orient[0]=0.8*current_angle[0];
			feet_orient[1]=current_angle[1]*pow(-1,leg);
		}
		else if(sequence==2)
		{
			//diff_shift=-1;
			err_z=com_z-((rot_hip[1]+(-50)*(1-leg)+50*leg)*pow(-1,leg+1));
			cor_zr[sequence]=cor_zr[sequence]*gain_i+0.1*err_z;//+scurve(cor_zr[sequence-1],0,seq2_count,5);
			cor_z[sequence]=cor_z[sequence]*gain_i-0.1*err_z;//+scurve(cor_z[sequence-1],0,seq2_count++,5);
			//err_z=0;
			
			feet_orient[0]=scurve(land_angle[0],0,seq2_count,5);
			feet_orient[1]=scurve(land_angle[0],0,seq2_count,5);
			
			//printf("ROT %10.5f\t",rot_hip[1]);
			//if(inRange(err_z,-5,10)==1&&inRange(rot_hip[1],-10,10)==1&&inRange(z_shift,-1,1)==1)//&&inRange(err_y,-5,5)==1)
			if(rot_hip[1]*pow(-1,leg+1)<7&&inRange(z_shift,-1,1)==1)
			sequence_count++;
			else
			sequence_count=0;
			//printf("COUNT %2d ",sequence_count);
			//add_z=scurve(save,0,seq2_count,10);
			if(sequence_count==2||(use_imu==0&&t_frac>zshift.endTime))//||(compare_float(t_frac,lift.x_start)))
			{
				walk_end=1;
				seq_change=1;
				//sequence++;
				x_control.lift.x_end=t_frac+(x_control.lift.x_end-x_control.lift.x_start);
				x_control.lift.x_start=t_frac;
				//y_control.y_0=t_frac+(y_control.y_0-y_control.y_start);
				//y_control.y_move=t_frac+(y_control.y_move-y_control.y_start);
				//y_control.y_end=t_frac+(y_control.y_end-y_control.y_start);
				//y_control.y_start=t_frac;
				
				
				touchdown_count=0;
			}
		}
	//	printf("Mom %10.5f ",momn_z);
	//	printf("SigMC %2d ",signum(diff_shift*momn_z));
	//	printf("SigCZR %2d ",signum(-diff_shift*cor_zr[sequence]));
		
		//cor_zr[sequence]+=0.4*(-hip_velo[1]*1000-diff_shift);
		
		
		//else
			//printf("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!kljhjdfsh\n");
			//while(1)
			//sleep(1);
		//feet_orient[0]=0;
		//feet_orient[1]=0;
		//printf("T %10.5f SEQ %2d SHIFT %10.5f ",t_frac,sequence,z_shift);
		//printf("ERR-Z %10.5f CORZ %10.5f CORZr %10.5f\n",err_z,cor_z,cor_zr);
		//printf("COM_z %10.5f\t",com_z);
			
		
		//printf("X %10.5f XR %10.5f T_COUNT %2d LIFT %2d E_Z %10.5f COR %10.5f ADD_XR %10.5f ",x,xr,touchdown_count,lift,err_z,cor_zr,add_xr);
		//printf("ADD_XR %10.5f ERR_X %10.5f %10.5f %10.5f %10.5f\n",add_xr,(390-x-rot_rleg[0]),rot_rleg[0],rot_rleg[1],rot_rleg[2]);
		//printf("ZSFT %10.5f X %10.5f XR %10.5f\n",z_shift,x,xr);
		
		//printf("X %10.5f ",x);
		//printf("X %10.5f XR %10.5f Y %10.5f YR %10.5f Z %10.5f ZR %10.5f PHI %10.5f PHIR %10.5f ZREAL %10.5f ZRREAL %10.5f \n",x,xr,y,yr,z,zr,phi,phir,zreal,zrreal);
		//printf("ZREAL %10.5f ZRREAL %10.5f \n",zreal,zrreal);
		//printf("ANGLE %10.5f %10.5f\n",current_angle[0],current_angle[1]);	
		//printf("COM %10.5f  A_COM %10.5f  ERR_Z %10.5f  COR_Z %10.5f  COR_ZR %10.5f  Z_SHIFT %10.5f\n",com_z,((rot_hip[1]+(-50)*(1-leg)+50*leg)*pow(-1,leg+1)),err_z,cor_z,cor_zr,z_shift); 
		
		///printf("%10.5f %10.5f %10.5f ",hip[0],hip[1]+(-100)*(1-leg)+(100)*leg,hip[2]);
		//printf("ZREAL %10.5f ZRREAl %10.5f",zreal,zrreal);		
		//printf("P %10.5f PR %10.5f ",phi,phir);
		//printf("ERR_Z %12.7f\t",err_z);
		//	printf("CZ %10.5f CZR %10.5f ",cor_z[sequence],cor_zr[sequence]);
		//printf("ERRZ %10.5f ",err_z);
		//printf("X %10.5f XR %10.5f Y %10.5f YR %10.5f Z %10.5f ZR %10.5f ",x,xr,y,yr,zreal,zrreal);
		//printf("%10.5f %10.5f %10.5f ",rot_hipendcoods[0],rot_hipendcoods[1],rot_hipendcoods[2]);
		
		
		//printf("ADDZ %10.5f ",add_z);	
		//z+=add_z;
	
		if(leg==0)
		{
			zreal=z*cos(-phi*3.14/180)+y*sin(-phi*3.14/180);
			yreal=-z*sin(-phi*3.14/180)+y*cos(-phi*3.14/180);
			
			zrreal=zr*cos(phir*3.14/180)-yr*sin(phir*3.14/180);
			yrreal=zr*sin(phir*3.14/180)+yr*cos(phir*3.14/180);
		}
		else if(leg==1)
		{
			zreal=z*cos(phi*3.14/180)-y*sin(phi*3.14/180);
			yreal=z*sin(phi*3.14/180)+y*cos(phi*3.14/180);
			
			zrreal=zr*cos(-phir*3.14/180)+yr*sin(phir*3.14/180);
			yrreal=-zr*sin(-phir*3.14/180)+yr*cos(phir*3.14/180);
		}
			
		if(abs(cor_zr[sequence])<60&&abs(cor_z[sequence])<60)
		{
			if(use_imu==1)
			{
				//zrreal+=cor_zr[sequence];
				//zreal+=cor_z[sequence];
			}
		}
		//else
		//printf("TIME %10.5f CORRECTION OUT OF BOUNDS %d \n",(float)frame/walk.fps,sequence);
			
		if(seq_change==1)
		{
			//printf("************* SEQ CHANGE *********\n");
			sequence++;
			seq_change=0;
		}
		
		
		
		zreal+=z_shift;
		zrreal-=z_shift;
		
		set_point[0]=x;
		set_point[1]=zreal;
		set_point[2]=yreal;
		
		if(fabs(current_angle[0])<50&&fabs(current_angle[1])<50)
		rotateFromIMU(set_point,resultIK);
		
		
		
		printf("Z %10.5f ZR %10.5f \n",zreal,zrreal); 
		//fval=generateik(resultIK[0],resultIK[2],resultIK[1],leg);
		fval=generateik(x,yreal,zreal,leg);
		
		
		for(walk_i=0,target_i=0,target_i=0;walk_i<6;walk_i++)
		{
			if(walk_i==4)
			{
				motor_val[20*leg+walk_i]=rotation(20*leg+walk_i,phi)+offset[20*leg+walk_i];
				continue;
			}
			else if(walk_i==2)
			motor_val[20*leg+10+walk_i]=fval[target_i]+offset[20*leg+10+walk_i];
			motor_val[20*leg+walk_i]=fval[target_i]+offset[20*leg+walk_i];
			//printf("IK %f\t",fval[target_i]);
			target_i++;
			
		}
		
		fval=generateik(xr,yrreal,zrreal,1-leg);
		for(walk_i=0,target_i=0;walk_i<6;walk_i++)
		{
			if(walk_i==4)
			{
				motor_val[20*(1-leg)+walk_i]=rotation(20*(1-leg)+walk_i,phir)+offset[20*(1-leg)+walk_i];
				continue;
			}
			else if(walk_i==2)
			motor_val[20*(1-leg)+10+walk_i]=fval[target_i]+offset[20*(1-leg)+10+walk_i];
			motor_val[20*(1-leg)+walk_i]=fval[target_i]+offset[20*(1-leg)+walk_i];
			//printf("IK %f\t",fval[target_i]);
			target_i++;
		}
		
		//motor_val[20*leg+0]+=feet_orient[1];
		//printf("ORI %10.5f ",feet_orient[1]);
		
		
		if(generatePacket()==1)
		{	
			if(iwrite(MOTOR_DATA,&fd,&ftdic1,tx_packet,tx_packet[3]+4)<0)
			{
				printf("ERROR : SENDING PACKET\n");
			}
		}
		t_frac+=speed/50;
		usleep(20000);
		//printf("\n");
				
	}
	printf("\n\n************TOTAL MOVE TIME : %f************\n",(float)frame/walk.fps);//int cor_i=0;	
	COUNTER+=STEP*2;
	COM_INC+=(yrin-yrfi);
	
	
	
	/// saving all data in absolute coods for next walk 
	prevWalk.leg=leg;
	prevWalk.set_point[0][0]=x*(1-leg)+xr*leg;
	prevWalk.set_point[0][1]=yreal*(1-leg)+yrreal*leg;
	prevWalk.set_point[0][2]=rot_hip[1];//(-zreal)*(1-leg)+(-zrreal)*leg;
	prevWalk.set_point[1][0]=x*leg+xr*(1-leg);
	prevWalk.set_point[1][1]=yreal*leg+yrreal*(1-leg);
	prevWalk.set_point[1][2]=rot_hip[1];//zreal*leg+zrreal*(1-leg);
	
	
	//printf("X: %10.5f Y: %10.5f Z: %10.5f XR: %10.5f YR: %10.5f ZR: %10.5f\n",prevWalk.set_point[0][0],prevWalk.set_point[0][1],prevWalk.set_point[0][2],prevWalk.set_point[1][0],prevWalk.set_point[1][1],prevWalk.set_point[1][2]); 
	//printf("COR Z  : %10.5f ZR : %10.5f\n",cor_z[2],cor_zr[2]);
	
	prevWalk.Omega[0]=Omega[0];
	prevWalk.Omega[1]=Omega[1];
	prevWalk.Omega[2]=Omega[2]*pow(-1,leg+1);
	
	prevWalk.hip_velo[0]=hip_velo[0];
	prevWalk.hip_velo[1]=hip_velo[1]*pow(-1,leg+1);
	prevWalk.hip_velo[2]=hip_velo[2];
	
	prevWalk.hip_coods[0]=rot_hip[0];
	prevWalk.hip_coods[1]=rot_hip[1];
	prevWalk.hip_coods[2]=rot_hip[2];
	
	prevWalk.cor_z=cor_z[sequence];
	prevWalk.cor_zr=cor_zr[sequence];
	
	gettimeofday(&prevWalk.endTime,NULL);
	
	
	cor_zr[0]=0;//cor_z[2];
	cor_zr[1]=0;
	cor_zr[2]=0;
	
	cor_z[0]=0;//cor_zr[2];
	cor_z[1]=0;
	cor_z[2]=0;
	
	zcom[0]=0;//-zcom[0];
	zcom[1]=0;//-zcom[1];
	zcom[2]=0;//-zcom[2];
	zcom[3]=0;//-zcom[3];
	zcom[4]=0;//-zcom[4];
	zcom[5]=0;//-zcom[5];
	zcom[6]=0;//-zcom[6];
	zcom[7]=0;//-zcom[7];
	zcom[9]=0;//zcom[8];
	zcom[9]=0;//-zcom[9];
	
	ZR_END=zrreal;
	Z_END=zreal;
	

	return EXIT_SUCCESS;
	
}       	

*/

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
