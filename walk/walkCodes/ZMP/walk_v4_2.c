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
#define D2R (180/3.14)
#define acyut 4  //acyut using...........................................change here..............
#define E 2.7183
#define MAX_BOUNDARIES 160
#define TRUE 1
#define serialbluetooth "A8008c3o"    //"A6007WeE"  
#define serialusb2d "A700eSSZ" //"A900fDpA" //"A600cBa7" //"A600cURy"  //"A900fC5U"//nd USB2D: A7003N1d  A7003NDp	 A7005LC8	  A700eST2
#define serialimu "A8004Yt8" //"A8008c3o" 
long int COUNTER=0;
long int COM_INC=0;
int COUNT;
char comfile[30]="\0";
char leftfile[30]="\0";
char rightfile[30]="\0";
char expect_com[30]="\0";
char hipfile[30]="\0";
long time_start, time_end;
struct timeval tv;
 
typedef uint8_t byte;
byte ignore_ids[]={10,15,16,17,18};  // enter ids to ignore :)
float q[5];
float mval[5];
	
float output[20];

	
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

int boundaries_size=0, local_boundaries_size=0;
float boundaries[MAX_BOUNDARIES][3]={0};
float local_boundaries[MAX_BOUNDARIES][3]={0};

/////////////////////////////////////////////////
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
 
int gflag=0;
 
FILE *f;
FILE *f1;
int fd,fb,imu_t;
struct ftdi_context ftdic1,ftdic2;  //ftdic1=usb2dyn   ftdic2=bluetooth

byte tx_packet[128]={0xff,0xff,0xfe,LEN_SYNC_WRITE,INST_SYNC_WRITE,0x1e,0x02};
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
//////D&A's GUI Comm defs//////
byte* gui_packet;
int element,noat,offset_status=0;
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
	/////// X-Lift Control /////
	float x_start;
	float x_end;
	float lift;
	float height;
	float freq;
	float displacement;	
	float phase;
	
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
	
}walk_x;

typedef struct
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

typedef struct
{
	/////// Y Control /////
	float y_start;
	float y_end;
	float yr_start;
	float yr_end;
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
	float z_com_start;
	float z_com_peak;
	float z_com_stay;
	float z_com_end;
	float z_com_max;
	float z_static_inc;
	
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
	int fps;
	leg_config left;
	leg_config right;
}walk_config; 

walk_config walk;

zmp_file zmp;

//////////////////// WALKING DEFINITIONS///////////////////////////

float Y_END=0,YR_END=0,Z_END=0,ZR_END=0,THETA_END=0,THETAR_END=0;
////////////////////////////////////////////////////////////////////

///////////////////// FORWARD KINEMATICS, INVERSE KINEMATICS , COM GENERATION , ZMP CALCULATION DEFINITIONS
float left_joints[7][3],right_joints[7][3];
float left_com[3]={0},right_com[3]={0};
float com[3]={0},rot_com[3]={0};
int offset[33]={0};
int target_left[6],target_right[6];
float STEP=25;
	
/////////////////////////////////////////////////////////////////////
void reset();
void check();
void loads();
float* generateik(float x,float y,float z,int i );
float* generatespline(int gn,float gx[],float gy[]);
float scurve(float in,float fi,float t, float tot);
void initialize_acyut();
void imucal();
void get_imu_data();
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
	
	//gettimeofday(&tv,NULL); /*gets the current system time into variable tv */ 
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
	
	for(try=0;try<1;try++)
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

int forward_kinematics(int leg,float mval[6])   
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
		//printf("%d\t%f\t",i,mval[i]);
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
			//printf("LEFT  %f\t%f\t%f\n",left_joints[i][0],left_joints[i][1],left_joints[i][2]);
		}	
	}	
	else if(leg==1)
	{
		for(i=0;i<7;i++)
		{
			right_joints[i][0]=coods[i][0];
			right_joints[i][1]=coods[i][1]+hip_length/2;
			right_joints[i][2]=coods[i][2];
			//printf("RIGHT %f\t%f\t%f\n",right_joints[i][0],right_joints[i][1],right_joints[i][2]);
		}
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
	float torso_wt=2734.9;
	com[0]+=121.358*torso_wt;
	com[1]+=0*torso_wt;
	com[2]+=0*torso_wt;

	com[0]=com[0]/(l_wt+r_wt+torso_wt);
	com[1]=com[1]/(l_wt+r_wt+torso_wt);
	com[2]=com[2]/(l_wt+r_wt+torso_wt);
	//printf(" COM X:%lf\tY:%lf\tZ:%lf\t",com[0],com[1],com[2]);
	return EXIT_SUCCESS;
}



int rotate_com(int leg)
{	
	FILE *f;
	
	int i=0,j=0;	
	float com_leg_frame[3];
	float hip_coods[3];
	float roll=0,pitch=0,yaw=0;
	if(leg==0)
	{
		for(i=0;i<3;i++)
		{
			com_leg_frame[i]=com[i]-left_joints[6][i];
			hip_coods[i]=-left_joints[6][i];
		}
	}
	else if(leg==1)
	{
		for(i=0;i<3;i++)
		{
			com_leg_frame[i]=com[i]-right_joints[6][i];
			hip_coods[i]=-right_joints[6][i];
		}
	}
	
	//// GETTING ANGLES FROM IMU
	get_imu_data();
	roll=-current_angle[0]*3.14/180;
	pitch=-current_angle[1]*3.14/180;
	yaw=0;//current_angle[2];
	
	//printf("ROLL %f PITCH %f \t",current_angle[0],current_angle[1]);
	//// CONVERTING TO IMU FRAME
	float x=-com_leg_frame[1];
	float y=com_leg_frame[2];
	float z=-com_leg_frame[0];
	/*float roll_matrix[3][3], pitch_matrix[3][3];
	roll_matrix.mat[0][0]=1;	roll_matrix.mat[0][1]=0;	roll_matrix.mat[0][2]=0;
	roll_matrix.mat[1][0]=0;	roll_matrix.mat[1][1]=cos(roll);roll_matrix.mat[1][2]=-sin(roll);
	roll_matrix.mat[2][0]=0;	roll_matrix.mat[2][1]=sin(roll);roll_matrix.mat[2][2]=cos(roll);
	
	pitch_matrix.mat[0][0]=cos(pitch);pitch_matrix.mat[0][1]=0;	pitch_matrix.mat[0][2]=sin(pitch);
	pitch_matrix.mat[1][0]=0;	pitch_matrix.mat[1][1]=1;	pitch_matrix.mat[1][2]=0;
	pitch_matrix.mat[2][0]=-sin(pitch);pitch_matrix.mat[2][1]=0;	pitch_matrix.mat[2][2]=cos(pitch);
	
	float coods[]={-com_leg_frame[1],com_leg_frame[0],-com_leg_frame[0]}
	*/
	float rot_x=cos(pitch)*x+sin(pitch)*z;
	float rot_y=sin(roll)*sin(pitch)*x+cos(roll)*y+sin(roll)*cos(pitch)*z;
	float rot_z=-cos(roll)*sin(pitch)*x+sin(roll)*y+cos(roll)*cos(pitch)*z;
	
	rot_com[0]=-rot_z;
	rot_com[1]=-rot_x+(-55)*(1-leg)+55*leg;
	rot_com[2]= rot_y;
		
	float hip_x=-hip_coods[1];
	float hip_y=hip_coods[2];
	float hip_z=-hip_coods[0];
	
	float hip_rot_x=cos(pitch)*hip_x+sin(pitch)*hip_z;
	float hip_rot_y=sin(roll)*sin(pitch)*hip_x+cos(roll)*hip_y+sin(roll)*cos(pitch)*hip_z;
	float hip_rot_z=-cos(roll)*sin(pitch)*hip_x+sin(roll)*hip_y+cos(roll)*cos(pitch)*hip_z;
	
	
	//printf("REV LEG %d VAl %f \n",leg,left_joints[6][1]*(1-leg)+right_joints[6][1]);
	zmp.coods[zmp.nof].x=rot_com[0];
	zmp.coods[zmp.nof].y=rot_com[1];
	zmp.coods[zmp.nof].z=rot_com[2]+COUNTER;
	
	zmp.left[zmp.nof].x=left_joints[6][0]-left_joints[6][0]*(1-leg)-right_joints[6][0]*leg;
	zmp.left[zmp.nof].y=left_joints[6][1]-(55+left_joints[6][1])*(1-leg)-(-55+right_joints[6][1])*leg;
	zmp.left[zmp.nof].z=left_joints[6][2]-left_joints[6][2]*(1-leg)-right_joints[6][2]*leg+COUNTER;
	
	zmp.right[zmp.nof].x=right_joints[6][0]-left_joints[6][0]*(1-leg)-right_joints[6][0]*leg;
	zmp.right[zmp.nof].y=right_joints[6][1]-(55+left_joints[6][1])*(1-leg)-(-55+right_joints[6][1])*leg;
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
		fprintf(f,"%lf\t%lf\t%lf\n",-hip_rot_z,-hip_rot_x+(-55)*(1-leg)+(55)*leg,hip_rot_y+COUNTER);
		fclose(f);
	}
	return EXIT_SUCCESS;
	
}
	
int  projection_system(int leg)
{
	/*float cnst[6];
	cnst[0]=1675;	
	cnst[1]=3207;
	cnst[2]=1752;
	cnst[3]=1714;
	cnst[4]=274;
	cnst[5]=1703;
	*/
	gettimeofday (&tv, NULL);
	time_start = (tv.tv_sec *1e+6) + tv.tv_usec;

	float val[6];
	int i=0;
	int mot_no[]={0,1,2,3,4,5};
	int flag=0;	
	for(i=0;i<6;i++)
	{
		//printf("READING MOTOR NO. %d\n",mot_no[i]);
		//if(mot_no[i]==0||mot_no[i]==1||mot_no[i]==3)
		{
			val[i]=target_left[i];
			continue;
		}
		val[i]=read_position(mot_no[i]);
		if(val[i]==9999)
		{
			//printf("USING TARGET VALUE\n");
			val[i]=target_left[i];
		}
		else
		val[i]-=offset[mot_no[i]];
		
	}
	
	forward_kinematics(0,val);
	for(i=0;i<6;i++)
	mot_no[i]+=20;
	
	for(i=0;i<6;i++)
	{
		//printf("READING MOTOR NO. %d\t",mot_no[i]);
		//if(mot_no[i]==25)
		{
			val[i]=target_right[i];
			continue;
		}
		val[i]=read_position(mot_no[i]);
		if(val[i]==9999)
		{
			val[i]=target_right[i];
			//printf("USING TARGET VALUE\n");
		}
		else
		val[i]-=offset[mot_no[i]];
	}
	forward_kinematics(1,val);
	generate_com();
	/*if(leg==1)
	printf("LEFT  LEG X : %f Y : %f Z : %f\t",left_joints[6][0],left_joints[6][1],left_joints[6][2]);  
	else 
	printf("RIGHT LEG X : %f Y : %f Z : %f\t",right_joints[6][0],right_joints[6][1],right_joints[6][2]);  
	*/
	rotate_com(1-leg);
	gettimeofday (&tv, NULL);
	time_end = (tv.tv_sec *1e+6) + tv.tv_usec;
	//printf("TIME : %ld\n",time_end-time_start);
	/*if(time_end-time_start>25000)
	printf("||||||||||||||||||||||||||||||||||||||||||||||||||||||________________________|||||||||||||||||||||||||||||||||||||\n");
	if(time_end-time_start>30000)
	printf("____________________|||||||||||||||||||||||||||||||||||||\n");
	*/
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

}
	
void set_offset()
{
	if(acyut==4)
	{
		offset[0]=15;
		offset[1]=350-10; 
		offset[2]=275; 
		offset[3]=232-10;    // 142s  
		offset[4]=-1; 
		offset[5]=15+7; 
		offset[6]=346; 
		offset[7]=0; 
		offset[8]=0; 
		offset[9]=0; 
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
		offset[20]=-36;
		offset[21]=334-10; 
		offset[22]=288; 
		offset[23]=228-10; 
		offset[24]=5; 
		offset[25]=15-7; 
		offset[26]=-306; 
		offset[27]= 0; 
		offset[28]= 0; 
		offset[29]=0; 
		offset[30]=0;
		offset[31]=0;
		offset[32]=238;
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
   		ftdi_set_interface(&imu,3);
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

//...........................................................imu calibrating functions........................................	
inline void read_bounds(char* move)
{
	FILE *fr;
	char name[50]="\0";
	if(acyut==3)
	strcat(name,"acyut3/");
	else if(acyut==4)
	strcat(name,"acyut4/");
	
	strcat(name,"imu/calibration/");
	strcat(name,move);
	strcat(name,".bin");
	fr=fopen(name,"r");
	if(fr!=NULL)
	{
		fread(&bounds,sizeof(bounds),1,fr);
		printf("Reading from calibration file : %s \n",name);
		int i=0;
		for(;i<MAX_BOUNDARIES;i++)
		{
			boundaries[i][0]=bounds[i].x;
			boundaries[i][1]=bounds[i].y;
			printf("X\t%lf\tY\t%lf\n",boundaries[i][0],boundaries[i][1]);
		//	printf("%lf\t %lf\n",bounds[i].y,boundaries[i][1]);
		}	
	}
	else
		printf("ERROR: File not found");
	
	fclose(fr);
	//sleep(10);
}
	
inline void write_bounds(char* move)
{
	FILE *fw;
	char name[50]="\0";
	if(acyut==3)
	strcat(name,"acyut3/");
	else if(acyut==4)
	strcat(name,"acyut4/");
	
	strcat(name,"imu/calibration/");
	strcat(name,move);
	strcat(name,".bin");
	fw=fopen(name,"w");
	if(fw!=NULL)
	{
		int i=0;
		for(;i<MAX_BOUNDARIES;i++)
		{
			bounds[i].nof=(i+1);
			bounds[i].x=boundaries[i][0];
			bounds[i].y=boundaries[i][1];
			printf("X\t%lf\tY\t%lf\n",bounds[i].x,bounds[i].y);
		}
		
		fwrite(&bounds,sizeof(bounds),1,fw);	
		printf("New Calibration file made/updated : %s \n",move);
	}
	else
	printf("File Not Found\n");	
	
	fclose(fw);
}
	
void write_vals(char* move)
{
	FILE *fw;
	char name[50]="\0";
	if(acyut==3)
	strcat(name,"acyut3/");
	else if(acyut==4)
	strcat(name,"acyut4/");
	
	strcat(name,"imu/graphs/");
	strcat(name,move);
	strcat(name,".bin");
	fw=fopen(name,"w");
	int i=0;
	for(;i<MAX_BOUNDARIES;i++)
	{
		bounds[i].nof=(i+1);
		bounds[i].x=local_boundaries[i][0];
		bounds[i].y=local_boundaries[i][1];
	}
	
	fwrite(&bounds,sizeof(bounds),1,fw);	
	printf("New Graph file made/updated : %s \n",move);
		
}



//----------------------------------------imu functions-------------------------------------------------------------------------------


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
					//printf("HERE\n");
					//if(ftdi_read_data(&imu,&im[i],8)>0 && im[6]==254 && im[7]==254)
					if(iread(IMU_DATA,&imu_t,&imu,&im[i],20)>0 && im[18]==254 && im[19]==254)
					{	
						flag++;
						flag2=1;
						for(j=0,k=0;j<6;j+=2,k+=1)
						{
							ang[k]=im[j]+im[j+1]*256;
							ang[k]=(ang[k] - 0) * (180 + 180) / (720 - 0) + -180;
							base_angle[k]+=ang[k];
						}
						for(j=0,k=0;j<6;j+=2,k+=1)
						{
							acc[k]=im[j+6]+im[j+7]*256;
							acc[k]=(acc[k] - 0) * (300 + 300) / (600 - 0) + -300;
							base_acc[k]+=acc[k];	
						}
						for(j=0,k=0;j<6;j+=2,k+=1)
						{
							gyro[k]=im[j+12]+im[j+13]*256;
							gyro[k]=(gyro[k] - 0) * (300 + 300) / (60000 - 0) + -300;
							base_gyro[k]+=gyro[k];	
						}
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
					if(iread(IMU_DATA,&imu_t,&imu,&im[i],20)>0 && im[18]==254 && im[19]==254)
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
						for(j=0,k=0;j<6;j+=2,k+=1)
						{
							acc[k]=im[j+6]+im[j+7]*256;
							acc[k]=(acc[k] - 0) * (300 + 300) / (600 - 0) + -300;
							imu_acc[k]=imu_acc[k]+acc[k];//-base_acc[k];
						}
						for(j=0,k=0;j<6;j+=2,k+=1)
						{
							gyro[k]=im[j+12]+im[j+13]*256;
							gyro[k]=(gyro[k] - 0) * (300 + 300) / (60000 - 0) + -300;
							imu_gyro[k]=imu_gyro[k]+gyro[k]-base_gyro[k];
						}
						
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
			printf("ACC %d : %lf\t",k,imu_acc[k]);
			printf("GYRO %d : %lf\n",k,imu_gyro[k]);
		}
	}
	else
	{
		printf("IMU READING FAILED\n");
		//usleep(16670*0.6);
	}
	ipurge(MOTOR_DATA,&imu_t,&imu);
	gettimeofday(&tv,NULL); /*gets the current system time into variable tv */ 
	time_end  = (tv.tv_sec *1e+6) + tv.tv_usec;
	//ftdi_usb_purge_buffers(&imu);
	//tcflush(imu,TCIFLUSH);
	//tcflow(imu,TCIOFLUSH);
}

inline void get_imu_data()
{
	int i=0;
	float gyro_threshold=0.01;
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
	//printf("GYRO 0 :%f GYRO 1 : %f\n",imu_gyro[0],imu_gyro[1]);
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
			printf("GYRO ANGLE %d : %f\n",i,gyro_angle[i]);	
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

/*void imucal_init()
{
	//printf("entered imu cal init\n");
	//sleep(2);
	byte im[8],i,j,k=0,a=3;
//	byte buffer[59];
	float ang[3]={0};
	byte flag=0,imu_no_of_times=0,imu_noat=0,flag2=0;
	int start=0;
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
				printf("%d\t",im[0]);
							
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
					//printf("HERE\n");
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
						//printf("FLAG : %d BASE_ANG 0: %f BASE_ANG 1: %f BASE_ANG 2: %f\n",flag,base_angle[0],base_angle[1],base_angle[2]);	
						printf("FLAG : %d BASE_ANG 0: %f BASE_ANG 1: %f BASE_ANG 2: %f\n",flag,ang[0],ang[1],ang[2]);	
						
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
		base_angle[k]=base_angle[k]/flag;
		printf("IMU INITIALIZED\n");
		imu_init=1;
	}
	else
	{
		printf("IMU NOT RESPONDING\n");
		imu_init=0;
	}
	
	for(k=0;k<3;k++)		
	{
		printf("BASE ANGLE : %d is %lf\n",k,base_angle[k]);
	}
	sleep(1);
}



inline void imucal()
{
	byte im[10],i,j,k=0,a=3,fallen=0;
	float ang[3]={0};
	byte flag=0,imu_no_of_times=0;
	avg_angle[0]=0;
	avg_angle[1]=0;
	avg_angle[2]=0;
	int start=0,count=0;
	
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
							ang[k]=(ang[k] - 0) * (180 + 180) / (720 - 0) + -180 -base_angle[k];
							avg_angle[k]=avg_angle[k]+ang[k];
						}
						
					}	
				}
			}
		}
		imu_no_of_times++;
	}
	
	if(count!=0)
	{
		for(k=0;k<3;k++)		
		{
			avg_angle[k]=avg_angle[k]/count;
			printf("ANGLE %d : %lf     ",k,avg_angle[k]);
		}
		printf("\n");
	}
	else
	{
		printf("IMU READING FAILED\n");
		//usleep(16670*0.6);
	}
	ipurge(MOTOR_DATA,&imu_t,&imu);
	//ftdi_usb_purge_buffers(&imu);
	//tcflush(imu,TCIFLUSH);
	//tcflow(imu,TCIOFLUSH);
}
*/
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
		{
			printf("DATA READ\n");
			for(i=0;i<30;i++)
			printf("%d\t",rcv_pkt[i]);
			printf("\n");
			
			for(i=0;i<20&&exit==0;i++)
			{
				if(rcv_pkt[i]==0xff&&rcv_pkt[i+1]==0xff&&rcv_pkt[i+8]==0xfe&&rcv_pkt[i+9]==0xfe)
				{
					ftdi_usb_purge_buffers(&imu);
					avg_angle[0]=(rcv_pkt[i+2]+rcv_pkt[i+3]*256)*(180 - (-180))/(720-0)+-180-base_angle[0];
					avg_angle[1]=(rcv_pkt[i+4]+rcv_pkt[i+5]*256)*(180 - (-180))/(720-0)+-180-base_angle[1];
					avg_angle[2]=(rcv_pkt[i+6]+rcv_pkt[i+7]*256)*(180 - (-180))/(720-0)+-180-base_angle[2];
					exit=1;
					printf("%f %f %f \n",avg_angle[0],avg_angle[1],avg_angle[2]);
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



void readfiles(char* s)
{
	
	char s1[60];
	if(acyut==3)
	{
		strcpy(s1,"acyut3/moves/");
	}
	else
	{
		strcpy(s1,"acyut4/moves/");
	}
	strcat(s1,s);
	f=fopen(s1,"r");
	if(f==NULL)printf("Error opening file.....\n");
	int gpos;
	int readfiles_i,j;
	byte buf[2];
	move r;
	fread(&r,sizeof(move),1,f);
	byte checksum;
	frame temp;
//	fread(buf,1,2,f);
	




//------------------------------------------------------------------------------------imu declarations--------------------------------------------------------------------------------------
	byte im[8],imu_i,k=0,a=3;
	byte buffer[59];
	float ang[3]={0},v=0;
	float init_ang[3]={0};	
	int aftr_frames=4;
	float change[3]={0},actual_change[3]={0}, present_boundary[3]={0};
	int flag_imu_first=0,flag_imu_read=0;
	int checksum_change=0,checksum_new=0, change_enuf[3]={0};	
	int imu_gpos=0,imu_gpos1=0,imu_gpos2=0,imu_gpos3=0,imu_gpos4=0,imu_gpos5=0,imu_gpos6=0;;
	int imu_gpos_init=0,imu_gpos1_init=0,imu_gpos2_init=0,imu_gpos3_init=0,imu_gpos4_init=0;
//----------------------------------------------------------------------------------------imu declarations end------------------------------------------------------------------------------------

	

	
	int nof=0;
	//if(r.header[0]==0xff && r.header[1]==0xff)
	{	

		read_bounds(s);	
				
		//imucal_init();
			 
		int noframes;
//		fread(&noframes,sizeof(int),1,f);
		for(nof=0;nof<r.noframes;nof++)
		{
			usleep(16670);//16670
			checksum=0;
//			fread(&temp,sizeof(temp),1,f);
			checksum+=r.frames[nof].num_motors+r.frames[nof].address;
			for(j=0;j<r.frames[nof].num_motors;j++)
			{
				checksum+=r.frames[nof].motorvalues[j].ID;
				checksum+=r.frames[nof].motorvalues[j].LB;
				checksum+=r.frames[nof].motorvalues[j].HB;
			}
			checksum=255-checksum;

			if(checksum!=r.frames[nof].checksum)
			{
				printf("Checksum error for frame : %d\n.....",nof+1);return;
			}
			tx_packet[3]=(r.frames[nof].num_motors*3)+4;
			tx_packet[5]=r.frames[nof].address;
	


			change_enuf[0]=change_enuf[1]=change_enuf[2]=0;	

//			if(flag_imu_c==1 && nof%aftr_frames==1)
			/*if(nof%aftr_frames==0)
			{
				ftdi_usb_purge_buffers(&imu);
				flag_imu_read=0;
				checksum_change=0;
				change[0]=change[1]=change[2]=0;
				
				
				//-----------------------replacing imucal----------------------------------------------------------------------------
				byte im[10],i,j,k=0,a=3,fallen=0;
				//	byte buffer[59];
				float ang[3]={0};
				byte flag=0,imu_no_of_times=0;
				avg_angle[0]=avg_angle[1]=avg_angle[2]=0;
				int start=0;
		
				while(imu_no_of_times<1 && imu_init==1)
				{
					flag=0;
					
			        while(flag==0)
					{   
						if(ftdi_read_data(&imu,&im[0],1)>0);
						{
							if(im[0]==0xff && start==0)
							{
								start=1;
								printf("first ff detected....\n");
							}
							else
							{
								if(im[0]==0xff && start==1)
								{		
										i=0;
										if(ftdi_read_data(&imu,&im[i],8)>0 && im[6]==254 && im[7]==254)
										{	
											flag=1;	
										}
										
										if(flag==1)
										{	
											for(j=0,k=0;j<6;j+=2,k+=1)
											{
												ang[k]=im[j]+im[j+1]*256;
												ang[k]=(ang[k] - 0) * (180 + 180) / (720 - 0) + -180;
												ang[k]=base_angle[k]-ang[k];
								
			//									printf("ang %d is %lf and base ang is %lf\n",k, ang[k],base_angle[k]);
											}
									
										}	
								}
								else
								{
									start=0;
								}
							}
						}
		  			} 
		
				for(k=0;k<3;k++)		
				{
					avg_angle[k]=avg_angle[k]+ang[k];
					printf("avg angle %d is %lf     ",k,avg_angle[k]);
				}
				printf("\n");
				imu_no_of_times++;
				}	
					
				
				
							
				//imucal();
				
				for(k=0; k<3 ; k++)
				{
					avg_angle[k]=avg_angle[k]+ang[k];
					printf("avg angle %d is %lf     ",k,avg_angle[k]);
					change[k]=avg_angle[k];
					
				}

			
			}
			*/
			checksum_change=0;
			
			
			for(k=0; k<3; k++)
			{
				//Stick to boundary code
				//present_boundary[k] = boundaries[nof/aftr_frames][k] + ((( boundaries[(nof/aftr_frames)+1][k]-boundaries[nof/aftr_frames][k] ) / aftr_frames ) * (nof%aftr_frames) );
				//change[k]=avg_angle[k]-present_boundary[k];			
			
			
				// Follow differences in boundary code
				if(nof>=aftr_frames)
					present_boundary[k]=local_boundaries[(nof/aftr_frames)-1][k]+(boundaries[(nof/aftr_frames)][k]-boundaries[(nof/aftr_frames)-1][k])/aftr_frames*(nof%aftr_frames);
				else
					present_boundary[k]=current_angle[k];
				
				change[k]=current_angle[k]-present_boundary[k];
				local_boundaries[nof/aftr_frames][k]=current_angle[k];
				
				printf("\npresent boundary  %d is : %lf \t",k, present_boundary[k]);
				if(( change[k] > (1) || change[k]< (-1) ) && ( change[k] < (200) || change[k]> (-200)))				// threshold changes to apply....
				{
					change_enuf[k]=1;
					printf("Sufficient change in %d and change is: %lf and avg angle is %lf\n",k, change[k], current_angle[k]);
				}
					
				change[k]=change[k]*(4096/300);	
				
			}
			
			
			printf("\nchange enuf is : %d as change is :%lf\n",change_enuf[0], change[0]);		
			printf("change enuf is : %d as change is :%lf\n",change_enuf[1], change[1]);					


			change[0]=(change[0]*3)/4;//4/4			// to change x axis total change...............change here......................
			change[1]=(change[1]*1)/4;//2/4 //3/4			// to change y axis total change...............change here......................	
				
			for(j=0;j<r.frames[nof].num_motors;j++)
			{	
				tx_packet[j*3+7]=r.frames[nof].motorvalues[j].ID;				
				tx_packet[j*3+8]=r.frames[nof].motorvalues[j].LB;
				tx_packet[j*3+9]=r.frames[nof].motorvalues[j].HB;	
					
//					if(nof%aftr_frames==0)						// to continue applying change for the frames btween aftr_frames also
					{
						
						if(change_enuf[0]==1)					// threshold crossed for X axis
						{
							
							if(tx_packet[7+j*3]==3 || tx_packet[7+j*3]==23)
							{
								checksum_change-=(tx_packet[8+j*3]+tx_packet[9+j*3]);
								imu_gpos=tx_packet[8+j*3]+tx_packet[9+j*3]*256;
								//imu_gpos=( (imu_gpos_init-change[0]) * v ) + ( imu_gpos_init * (1-v) );
								printf("Id: %d\tOriginal value: %d\t",tx_packet[7+j*3],imu_gpos);
								imu_gpos-=(change[0]*4)/7;						//..............................change individual ratio distribution here....
								printf("New value %d\n", imu_gpos);
								tx_packet[8+j*3]=get_lb(imu_gpos);
								tx_packet[9+j*3]=get_hb(imu_gpos);
								checksum_change+=(tx_packet[8+j*3]+tx_packet[9+j*3]);
							}	
							if(tx_packet[7+j*3]==21)
							{
								checksum_change-=(tx_packet[8+j*3]+tx_packet[9+j*3]);
								imu_gpos1=tx_packet[8+j*3]+tx_packet[9+j*3]*256;
								//imu_gpos1=( (imu_gpos1_init-change[0]) * v ) + ( imu_gpos1_init * (1-v) );
								printf("Id: %d\tOriginal value: %d\t",tx_packet[7+j*3],imu_gpos1);
								imu_gpos1+=(change[0]*3)/7;						//..............................change individual ratio distribution here....
								printf("New value %d\n", imu_gpos1);
								tx_packet[8+j*3]=get_lb(imu_gpos1);
								tx_packet[9+j*3]=get_hb(imu_gpos1);
								checksum_change+=(tx_packet[8+j*3]+tx_packet[9+j*3]);
							}
			
							if(tx_packet[7+j*3]==1)
							{
								checksum_change-=(tx_packet[8+j*3]+tx_packet[9+j*3]);
								imu_gpos2=tx_packet[8+j*3]+tx_packet[9+j*3]*256;
								//imu_gpos2=( (imu_gpos2_init-change[0]) * v ) + ( imu_gpos2_init * (1-v) );
								printf("Id: %d\tOriginal value: %d\t",tx_packet[7+j*3],imu_gpos2);
								imu_gpos2+=(change[0]*3)/7;						//..............................change individual ratio distribution here....
								printf("New value %d\n", imu_gpos2);
								tx_packet[8+j*3]=get_lb(imu_gpos2);
								tx_packet[9+j*3]=get_hb(imu_gpos2);
								checksum_change+=(tx_packet[8+j*3]+tx_packet[9+j*3]);
							}	
						}
						if(change_enuf[1]==1)							// threshold crossed for Y axis
						{
							if(tx_packet[7+j*3]==0)
							{
								checksum_change-=(tx_packet[8+j*3]+tx_packet[9+j*3]);
								imu_gpos3=tx_packet[8+j*3]+tx_packet[9+j*3]*256;
								//imu_gpos3=( (imu_gpos3_init-change[1]) * v ) + ( imu_gpos3_init * (1-v) );
								printf("Id: %d\t Original value: %d\t",tx_packet[7+j*3],imu_gpos3);
								imu_gpos3-=(change[1]);							//..............................change individual ratio distribution here....
								printf("New value %d\n", imu_gpos3);
								tx_packet[8+j*3]=get_lb(imu_gpos3);
								tx_packet[9+j*3]=get_hb(imu_gpos3);
								checksum_change+=(tx_packet[8+j*3]+tx_packet[9+j*3]);
							}
							if(tx_packet[7+j*3]==20)
							{
								checksum_change-=(tx_packet[8+j*3]+tx_packet[9+j*3]);
								imu_gpos4=tx_packet[8+j*3]+tx_packet[9+j*3]*256;
								//imu_gpos4=( (imu_gpos4_init-change[1]) * v ) + ( imu_gpos4_init * (1-v) );
								printf("Id: %d\tOriginal value: %d\t",tx_packet[7+j*3],imu_gpos4);
								imu_gpos4+=(change[1]);							//..............................change individual ratio distribution here....
								printf("New value %d\n", imu_gpos4);
								tx_packet[8+j*3]=get_lb(imu_gpos4);
								tx_packet[9+j*3]=get_hb(imu_gpos4);
								checksum_change+=(tx_packet[8+j*3]+tx_packet[9+j*3]);
							}
							if(tx_packet[7+j*3]==5)
							{
								checksum_change-=(tx_packet[8+j*3]+tx_packet[9+j*3]);
								imu_gpos5=tx_packet[8+j*3]+tx_packet[9+j*3]*256;
								//imu_gpos3=( (imu_gpos3_init-change[1]) * v ) + ( imu_gpos3_init * (1-v) );
								printf("Id: %d\t Original value: %d\t",tx_packet[7+j*3],imu_gpos3);
								imu_gpos5-=(change[1]);							//..............................change individual ratio distribution here....
								printf("New value %d\n", imu_gpos3);
								tx_packet[8+j*3]=get_lb(imu_gpos5);
								tx_packet[9+j*3]=get_hb(imu_gpos5);
								checksum_change+=(tx_packet[8+j*3]+tx_packet[9+j*3]);
							}
							if(tx_packet[7+j*3]==25)
							{
								checksum_change-=(tx_packet[8+j*3]+tx_packet[9+j*3]);
								imu_gpos6=tx_packet[8+j*3]+tx_packet[9+j*3]*256;
								//imu_gpos4=( (imu_gpos4_init-change[1]) * v ) + ( imu_gpos4_init * (1-v) );
								printf("Id: %d\tOriginal value: %d\t",tx_packet[7+j*3],imu_gpos4);
								imu_gpos6+=(change[1]);							//..............................change individual ratio distribution here....
								printf("New value %d\n", imu_gpos4);
								tx_packet[8+j*3]=get_lb(imu_gpos6);
								tx_packet[9+j*3]=get_hb(imu_gpos6);
								checksum_change+=(tx_packet[8+j*3]+tx_packet[9+j*3]);
							}
					}
				}
			}
						
			checksum=255-checksum;
		
			checksum-=r.frames[nof].num_motors+r.frames[nof].address;
			checksum+=tx_packet[2]+tx_packet[3]+tx_packet[4]+tx_packet[5]+tx_packet[6];
			checksum_new=checksum;
			checksum_new+=checksum_change;
			printf("checksum change is %d and old checksum was %d new checksum is %d\n", checksum_change, checksum, checksum_new);
			tx_packet[7+NOM*3]=(255-checksum_new);

		
		local_boundaries_size=r.noframes;
		
			
		printf("\nFrame : %d\n",nof+1);
		
	/*	for(j=0;j<r.frames[nof].num_motors;j++)
		{
		if(r.frames[nof].address==0x1c||r.frames[nof].address==0x1a)
		printf("ID : %d		Value : %d\n",r.frames[nof].motorvalues[j].ID,r.frames[nof].motorvalues[j].LB);
		else
		printf("ID : %d		Value : %d\n",r.frames[nof].motorvalues[j].ID,r.frames[nof].motorvalues[j].LB+r.frames[nof].motorvalues[j].HB*256);
		}
	*/	
		//ftdi_write_data(&ftdic1,tx_packet, tx_packet[3]+4);
		iwrite(MOTOR_DATA,&fd,&ftdic1,tx_packet,tx_packet[3]+4);		
		}

	}
	if(f!=NULL)
	fclose(f);
	//graph_gnuplot(local_boundaries,local_boundaries_size,s,1);
	write_vals(s);
	
}


void initialize()
{
	speed(0);
	readfiles("init");
	int l=0;
	for(l=0;l<128;l++)
		last_tx_packet[l]=tx_packet[l];
		
	speed(1);
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

void drivemode()
{
	int id;
	byte baud[8]={0xff,0xff,0,0x04,0x03,0x0a,0x00,0};
	printf("\nEnter the id: ");
	scanf("%d",&id);
	baud[2]=id;
	int i=0;
	byte checksum=0;
	for(i=0;i<baud[3]+3;i++)
		checksum+=baud[i];
	checksum=255-checksum;
	baud[i]=checksum;
	for(i=0;i<baud[3]+4;i++)
		printf("%x\n",baud[i]);
	//ftdi_write_data(&ftdic1,baud, baud[3]+4);
	iwrite(MOTOR_DATA,&fd,&ftdic1,baud,baud[3]+4);
	printf("drive mode 0 written for %d id.",id);
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
	printf("ERROR : Compliance\n");
	
}



/*----------------------------including imu change--------------------------------------------------*/
int read_acyut()
{
	printf("Reading AcYut\n");
	int count=0,count1=0,no_of_try=0,nom;
	nom=gui_packet[element++];
	//printf("nom%d\n",nom);
	//printf("len%d\n",(nom*3)+4);
	byte* return_data;
	byte some_variable,snd_packet[8],rcv_packet[5],chksum=0;
	//snd_packet[0]={0xff,0xff,1,0x04,0x02,0x24,0x02,chksum};
	snd_packet[0]=0xff;snd_packet[1]=0xff;snd_packet[2]=0x00;snd_packet[3]=0x04;snd_packet[4]=0x02;snd_packet[5]=0x24;snd_packet[6]=0x02;snd_packet[7]=chksum;
	//printf("chal to rha hai\n");
	return_data=(byte *)malloc(sizeof(byte));
	free(return_data);
	return_data=(byte *)malloc(((nom*3)+4)*sizeof(byte));
	//printf("ab bhi chal raha hai\n");
	return_data[0]=0xff;
	return_data[1]=0xff;
	return_data[2]=1+nom*3;
	return_data[3+nom*3]=1+nom*3;    //checksum
	while(count++<nom)
	{
	    printf("IDS1 : %d \n",gui_packet[element]);
		snd_packet[2]=gui_packet[element++];
		
		snd_packet[7]=255-(snd_packet[2]+0x2c);
		if(ftdi_write_data(&ftdic1,snd_packet,8)!=8)
			return 21;
		count1=0,noat=0;
		while(noat<10)
		{
			if(ftdi_read_data(&ftdic1,&some_variable,1))
			{
				if(some_variable==0xff)
				{
					if(++count1==2)
					{	
						noat=0;
						chksum=0;
						count1=0;
						while(count1<6&&noat<10)
						{
							if(ftdi_read_data(&ftdic1,&some_variable,1)==1)
							{
								rcv_packet[count1++]=some_variable;
								chksum=chksum+some_variable;
								noat=0;
							//	printf("read bhi kar raha hai\n");
							}
							else
							{
								noat++;
								printf("attempting to read%d\n",noat);
							}
						}
						if(chksum==255&&noat<10)
						{
							no_of_try=0;
							break;
						}
						else
						{
							rcv_packet[0]=gui_packet[element-1];
							rcv_packet[3]=((noat>=10)?22:29);
							rcv_packet[4]=0xee;
						//	flag=0;
							noat>=10?printf("NOAT ERROR\nResending Packet\n"):printf("CHECKSUM ERROR\nResending Packet\n");
						}
						//}
					}
				}
				else count1=0;
				noat=0;
			}
			else
			{
				noat++;
				//printf("attempting to read yaha par %d\n",noat);
			}
		}
		if(noat>=10)
		{	
			if (++no_of_try<3)
			{
				element--;
				count--;
			}
			else
			{
				rcv_packet[0]=gui_packet[element-1];
				rcv_packet[3]=22;
				rcv_packet[4]=0xee;
				printf("SKIPPING MOTOR\n");
			}
		}
		if((noat<10||no_of_try>=3))
		{
			no_of_try=0;
			printf("count:%d\n",count);
			return_data[0+count*3]=rcv_packet[0];
			return_data[1+count*3]=rcv_packet[3];
			return_data[2+count*3]=rcv_packet[4];
			return_data[3+nom*3]=return_data[3+nom*3]+rcv_packet[0]+rcv_packet[3]+rcv_packet[4];
			printf("Id: %d Lb:%d Hb:%d Chksm:%d\n",return_data[0+count*3],return_data[1+count*3],return_data[2+count*3],return_data[3+nom*3]);
		}
	}
	int deepak=0;
	for(;deepak<4+nom*3;deepak++)
	{
		printf("pack [%d] = %d\n",deepak,return_data[deepak]);
	}
	if(!ftdi_write_data(&ftdic2,return_data,4+nom*3))
			return 21;
	return 1;
}
int dir_tran()
{
	byte checksum=0;
	int len=(element+3)+gui_packet[element+3];
	int k;
	byte packet[len];
	packet[0]=gui_packet[element++];
	packet[1]=gui_packet[element++];
	for(;element<len;element++)
	{	
		packet[element-1]=gui_packet[element];
		checksum+=packet[element-1];
		printf("DATA %d :%d\n",element-1,packet[element-1]);
	}
	/*printf("chksum for dir trans=%d\tchksum received from GUI=%d\n",(255-checksum),gui_packet[element]);
	int apoorv=0;
	for(;apoorv<len-1;apoorv++)
	{
		printf("packet[%d]=%d\n",apoorv,packet[apoorv]);
	}*/
	if((255-checksum)==gui_packet[element])
	{
		packet[element-1]=gui_packet[element];
		ftdi_write_data(&ftdic1,packet,len);
		if(packet[5]==0x1e)
		{
			for(k=0;k<130;++k)
			 last_tx_packet[k]=packet[k];
			 gflag++;
			 //flag_imu_first=0;
		}
		return 1;
	}
	else
	return 31;
}

int act_fpacket(char *movname)
{
	//printf("entering fpacket for playing file : %s \n",movname);
	int err=1;
	FILE *movr;
	move mov;
	movr=fopen(movname,"r");
	if(movr==NULL)
		return 243;
	fread(&mov,sizeof(move),1,movr);
	fclose(movr);
	
	int i,j,adchksum;
	
	for(i=0;i<mov.noframes;i++)
	{
		//byte yu=1;
		//ftdi_write_data(&ftdic1,0xff,1);
		//printf("\nftdi done\n");
		usleep(16670);
		tx_packet[3]=mov.frames[i].num_motors*3+4;
		tx_packet[5]=mov.frames[i].address;
		adchksum=7+mov.frames[i].num_motors*3;
		tx_packet[adchksum]=0;
		
		//int tp_gen=0;
		//printf("FRAME NO. %d\n",i);
		for(j=0;j<mov.frames[i].num_motors;j++)
		{
			tx_packet[7+j*3]=mov.frames[i].motorvalues[j].ID;
			tx_packet[8+j*3]=mov.frames[i].motorvalues[j].LB;
			tx_packet[9+j*3]=mov.frames[i].motorvalues[j].HB;
			//tx_packet[adchksum]+=tx_packet[7+j*3]+tx_packet[8+j*3]+tx_packet[9+j*3];
			tx_packet[adchksum]+=tx_packet[7+j*3]+tx_packet[8+j*3]+mov.frames[i].motorvalues[j].HB;
			//printf("%d\t%d\t%d\n",mov.frames[i].motorvalues[j].ID,mov.frames[i].motorvalues[j].LB,mov.frames[i].motorvalues[j].HB,tx_packet[adchksum]);
			//printf("ID%d\tfile [%d] lb =%d\t hb=%d\t chksum=%d\n",tx_packet[7+j*3],i,tx_packet[8+j*3],tx_packet[9+j*3],tx_packet[adchksum]);
		}
		/*for (tp_gen = 0; tp_gen < mov.frames[i].num_motors*3+8; tp_gen += 1)
		{
			printf("%d\n",tx_packet[tp_gen]);
		}*/
		byte framechksum=255-(mov.frames[i].num_motors+mov.frames[i].address+tx_packet[adchksum]);
		if(framechksum!=mov.frames[i].checksum)
		{
			err=244;
			//printf("Chksum=%d\tfile chksum[%d] =%d\n",framechksum,i,mov.frames[i].checksum);
			continue;
		}
		//printf("reached here\n\n");
		tx_packet[adchksum]+=tx_packet[2]+tx_packet[3]+tx_packet[4]+tx_packet[5]+tx_packet[6];
		tx_packet[adchksum]=255-tx_packet[adchksum];
		//printf("about to write in ftdi\n");
		//printf("Length %d\n",tx_packet[3]+4);
		
		////////////////////////////
		
		/*int dhairya=0;
		for(dhairya=0;dhairya<tx_packet[3]+4;dhairya++)
		{
			printf("DATA : %d\n",tx_packet[dhairya]);
			
		}*/
		//ftdi_write_data(&ftdic1,tx_packet,tx_packet[3]+4);
		iwrite(MOTOR_DATA,&fd,&ftdic1,tx_packet,tx_packet[3]+4);
	}
	for(i=0;i<128;i++)
	last_tx_packet[i]=tx_packet[i];
	return err;
	
}		

int act_ppacket(char* movname,int stopf,int startf)
{
	printf("entering ppacket\n");
	int err=1;
	FILE *movr;
	move mov;
	movr=fopen(movname,"r");
	if(movr==NULL)
	return 243;
	else
	fread(&mov,sizeof(move),1,movr);
	fclose(movr);
	
	int i,j,adchksum;
	printf("START FRAME : %d\n",startf);
	printf("STOP FRAME : %d\n",stopf);
	for(i=startf;i<stopf;i++)
	{
		printf("PLAYING FRAME : %d\n",i);
		usleep(16670);
		tx_packet[3]=mov.frames[i].num_motors*3+4;
		tx_packet[5]=mov.frames[i].address;
		adchksum=7+mov.frames[i].num_motors*3;
		tx_packet[adchksum]=0;
		for(j=0;j<mov.frames[i].num_motors;j++)
		{
			tx_packet[7+j*3]=mov.frames[i].motorvalues[j].ID;
			tx_packet[8+j*3]=mov.frames[i].motorvalues[j].LB;
			tx_packet[9+j*3]=mov.frames[i].motorvalues[j].HB;
			tx_packet[adchksum]+=tx_packet[7+j*3]+tx_packet[8+j*3]+tx_packet[9+j*3];
		}
		byte framechksum=255-(mov.frames[i].num_motors+mov.frames[i].address+tx_packet[adchksum]);
		if(framechksum!=mov.frames[i].checksum)
		{
			err=244;
			printf("Chksum=%d\tfile chksum[%d] =%d\n",framechksum,i,mov.frames[i].checksum);
			continue;
		}
		printf("reached here\n\n");
		tx_packet[adchksum]+=tx_packet[2]+tx_packet[3]+tx_packet[4]+tx_packet[5]+tx_packet[6];
		tx_packet[adchksum]=255-tx_packet[adchksum];
		//ftdi_write_data(&ftdic1,tx_packet,tx_packet[3]+4);
		iwrite(MOTOR_DATA,&fd,&ftdic1,tx_packet,tx_packet[3]+4);
	}
	for(i=0;i<130;i++)
		last_tx_packet[i]=tx_packet[i];
		
	return err;

}
/*
int moveplay(char* movn,int startf,intstopf)
{
	int imu_on=0;
	char movename[50];
	char calname[50];

	if(acyut==4)
	{
		strcat(movename,"acyut4/moves/");
		strcat(calname,"acyut4/imu/calibration/");
	}
	else if(acyut==3)
	{
		strcat(movename,"acyut3/moves/");
		strcat(calname,"acyut3/imu/calibration/");
	}
	else
	return -1;
	
	strcat(movename,movn);
	strcat(calname,movn);
	
	FILE* movf,calf;
	movr=fopen(movename,"r");
	if(movr=NULL)
	return -2;
	move mov;
	fread(&mov,sizeof(move),1,movf);
	fclose(movf);
	
	if(imu_init!=1)
	{
		printf("NON IMU PLAY : IMU NOT INITIALIZED\n");
	}	
	else
	{
		imu_on++;
	}
	calf=fopen(calname,"r");
	if(calf==NULL)
	{
		printf("NON IMU PLAY : IMU CALIBERATION NOT FOUND");	
	}
	else
	{
		imu_on++;
		fread(bounds,sizeof(bounds),1,calf);
	}	
	
	
	
}	
*/	

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

float* generateik(float x, float y, float z,int i)
{
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
	
	ang[0]=phi[0];
	ang[1]=-theta[1]+phi[1];
	ang[2]=-theta[2];
	ang[3]=-theta[3]-phi[3];
	ang[4]=phi[5];
	//printf("%f\n",phi[5]);
	mval[0]=(ang[0]*r2d*4096/300)+cons[0];//+z*1.1
	mval[1]=((ang[1]*r2d)*4096/300)+cons[1];
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



/*float* generateik(float x,float y,float z,float z_ang)
{

  float c1 = -123.39;       //'23.3789    '- 12.625
  float c2 = -70.54;     //'-115.664   '- 124.15
  float c3 = -33.39;     //'56.5332    '11.615
  float l1;
  float hip_x=0,hip_y=0,hip_z=0;
  float l12 = 179.410;//191.84;//179.410;
  float l23 = 150.500;//191.84;//150.500;
  float pi = 3.1415926;
  float l13=0;//, al1=0, q1=0, q2=0, q3=0,q_cor=0,q0=0;

  float al1=0,al2=0;
  float q1=0;
  float q2=0;
  float q3=0;
  float q4=0;
  float q_cor=0;
  float q0=0;
  float z_cor=52.5;
  float q4_init=1740;
  float q0_init=1715;
  float deltaq4=0;

  l1=sqrt(pow(hip_x-x,2)+pow(hip_y-y,2));
  l13 = l1;
  al1 = asin((l13 - 49) / (2 * l12));

  //al1 = asin((pow(l13-x,2)+pow(l23,2)-pow(l12,2))/(2*l23*(l13-x)));
  //al2 = acos((l23/l12)*cos(al1));

  q1 = ((al1 * 180) / pi);
  q2 = ((al1 * 180) / pi) ;
  q3 = ((al1 * 180) / pi) ;

  q_cor=((atan(y/x))*180)/pi;
  q3=q3-q_cor;
  q1=q1+q_cor;
  q1 = q1 - c1;
  q2 = q2 - c2;
  q3 = q3 - c3;
  q1 = ceil((((q1 * 4096) / 280) - 1));
  q2 = 4096 - ceil((((q2 * 4096) / 280) - 1));
    //  q3 = ceil((((q3 * 4096) / 280) - 1)) - 100;
  q3 = ceil((((q3 * 4096) / 280) - 1));

  if(z_ang!=0)
  {
    q0=ceil(z_ang);
    q4=q4_init;
  }
  else
  {
    deltaq4=asin(z/(l13+z_cor));
    deltaq4 =(deltaq4 * 180) / pi ;
    deltaq4=ceil((deltaq4*4096)/280);
    q0=q0_init+deltaq4;                                    //---------------change in value of id 20 or 0 for foot to be flat
    q4=q4_init+deltaq4;                                    //---------------change in value of id 25 or 5 for z movement
  }

  q[0]=q0;
  float change=0;
     change=150-((float)150/(420-150)*(x-150));
  q[1]=q1;
  q[2]=q2;
  q[3]=q3-change;
  q[4]=q4-z*1.1;

// convert to EX106+............................................
  int tp;
  for(tp=0;tp<5;tp++)
     q[tp]= (float) (  q[tp]*280  - 15*4096  )/250;

  return q;
}
*/
int read_load(int n)
{
	int load=0,dir=0,mov=0;
	int count=0,count1=0,no_of_try=0;	
	byte val,snd_packet[8],rcv_packet[10],chksum=0xff,noat=0;
	snd_packet[0]=0xff;snd_packet[1]=0xff;snd_packet[2]=n;snd_packet[3]=0x04;snd_packet[4]=0x02;snd_packet[5]=0x28;snd_packet[6]=0x06;snd_packet[7]=0xff;
	int try =0;
	for (no_of_try = 0; no_of_try < 5; no_of_try += 1)
	{
		snd_packet[7]-=snd_packet[no_of_try+2];
	}
	
	while(try++<3)
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
							try=111;
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
		try++;
		if(try==111)
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

int static_check()
{
	int i=0;
	int exit=0;
	int count[]={0,0,0};
	int change[3];
	int threshold=3;
	int value;
	int adchksum=NOM*3+7;
	while(exit==0)
	{
		imucal();
	
		for(i=0;i<2;i++)
		{
			if(abs(current_angle[i])<=threshold)
			{
				count[i]++;
				change[i]=0;
			}
			else if(abs(current_angle[i])<=20)
			{
				change[i]=0;
				change[i]=current_angle[i];
			}	
			else
			{
				count[i]=0;
				change[i]=0;
				
			}
		}

		for(i=0;i<NOM;i++)
		{
			if(tx_packet[7+i*3]==1||tx_packet[7+i*3]==21)
			{
				value=last_tx_packet[8+i*3]+last_tx_packet[9+i*3]*256;
				tx_packet[adchksum]-=tx_packet[8+i*3]+tx_packet[9+i*3];
				value+=change[0]*4096/300*3/24;
				if(value>=0 && value<=4096)
				{
					tx_packet[8+i*3]=value%256;
					tx_packet[9+i*3]=value/256;
				}
				tx_packet[adchksum]-=tx_packet[8+i*3]+tx_packet[9+i*3];
				printf("value %d \t %d\n",tx_packet[7+i*3],value);				
			}
			/*if(tx_packet[7+i*3]==3||tx_packet[7+i*3]==23)
			{
				value=tx_packet[8+i*3]+tx_packet[9+i*3]*256;
				tx_packet[adchksum]-=tx_packet[8+i*3]+tx_packet[9+i*3];
				value-=change[0]*4096/300*5/24;
				if(value>=0 && value<=4096)
				{
					tx_packet[8+i*3]=value%256;
					tx_packet[9+i*3]=value/256;
				}
				tx_packet[adchksum]-=tx_packet[8+i*3]+tx_packet[9+i*3];
				//printf("value %d \t %d\n",tx_packet[7+i*3],value);				
			}
			*/
			if(tx_packet[7+i*3]==0)
			{
				value=last_tx_packet[8+i*3]+last_tx_packet[9+i*3]*256;
				tx_packet[adchksum]+=tx_packet[8+i*3]+tx_packet[9+i*3];
				value-=change[1]*4096/300*7/24;
				if(value>=0 && value<=4096)
				{
					tx_packet[8+i*3]=value%256;
					tx_packet[9+i*3]=value/256;
				}
				tx_packet[adchksum]-=tx_packet[8+i*3]+tx_packet[9+i*3];
				printf("value %d \t %d\n",tx_packet[7+i*3],value);				
			}
			if(tx_packet[7+i*3]==20)
			{
				value=last_tx_packet[8+i*3]+last_tx_packet[9+i*3]*256;
				tx_packet[adchksum]+=tx_packet[8+i*3]+tx_packet[9+i*3];
				value+=change[1]*4096/300*7/24;
				if(value>=0 && value<=4096)
				{
					tx_packet[8+i*3]=value%256;
					tx_packet[9+i*3]=value/256;
				}
				tx_packet[adchksum]-=tx_packet[8+i*3]+tx_packet[9+i*3];
				printf("value %d \t %d\n",tx_packet[7+i*3],value);				
			}
			
			/*if(tx_packet[7+i*3]==5)
			{
				value=tx_packet[8+i*3]+tx_packet[9+i*3]*256;
				tx_packet[adchksum]+=tx_packet[8+i*3]+tx_packet[9+i*3];
				value+=change[1]*4096/300*12/24;
				if(value>=0 && value<=4096)
				{
					tx_packet[8+i*3]=value%256;
					tx_packet[9+i*3]=value/256;
				}
				tx_packet[adchksum]-=tx_packet[8+i*3]+tx_packet[9+i*3];
				//printf("value %d \t %d\n",tx_packet[7+i*3],value);				
			}
			if(tx_packet[7+i*3]==25)
			{
				value=tx_packet[8+i*3]+tx_packet[9+i*3]*256;
				tx_packet[adchksum]+=tx_packet[8+i*3]+tx_packet[9+i*3];
				value-=change[1]*4096/300*12/24;
				if(value>=0 && value<=4096)
				{
					tx_packet[8+i*3]=value%256;
					tx_packet[9+i*3]=value/256;
				}
				tx_packet[adchksum]-=tx_packet[8+i*3]+tx_packet[9+i*3];
				//printf("value %d \t %d\n",tx_packet[7+i*3],value);				
			}
			*/
			
		}
		
		//if (ftdi_write_data(&ftdic1,tx_packet,tx_packet[3]+4)<0)
		if (iwrite(MOTOR_DATA,&fd,&ftdic1,tx_packet,tx_packet[3]+4)<0)
		printf("ERROR SENDING PACKET\n");
		
		usleep(16670);
		if(count[0]>=5&&count[1]>=5)
		exit=1;
	}
	printf("ACYUT STATICALLY STABLE\n");
	//while(1)
	//sleep(8);
}

int level3()
{
	int side=0;
	int i=0;
	torque(0);
	sleep(1);
	speed(0);
	initialize_acyut();
	usleep(500000);
	speed(1);
	for(i=0;i<5;i++)
	{
		side+=current_angle[0];
	}
	if(side>0)
	act_fpacket("acyut4/moves/frontgetup");
	else if(side <0)
	act_fpacket("acyut4/moves/backgetup");

	return EXIT_SUCCESS;
}


void set_walk_offset()
{
	offset[0]+=0;
	offset[1]+=0;
	offset[2]+=0;
	offset[3]+=0;
	offset[4]+=0;
	offset[5]+=0;
	offset[6]+=0;
	offset[7]+=0;
	offset[8]+=0;
	offset[9]+=0;
	offset[10]+=0;
	offset[11]+=0;
	offset[12]+=0;
	offset[13]+=0;
	offset[14]+=0; 
	offset[15]+=0; 
	offset[16]+=0; 
	offset[17]+=0; 
	offset[18]+=0; 
	offset[19]+=0; 
	offset[20]+=0;
	offset[21]+=0;
	offset[22]+=0; 
	offset[23]+=0; 
	offset[24]+=0; 
	offset[25]+=0; 
	offset[26]+=0; 
	offset[27]+=0; 
	offset[28]+=0; 
	offset[29]+=0; 
	offset[30]+=0;
	offset[31]+=0; 
	offset[32]+=0;
}
// Correction 
float err_z=0;
float err_y=0;
float cor_z[120]={0};
float prev_err_z=0;
float future_err_z[40]={0};

float sum_err(float *err, float coeff_a)
{
	int sum=0;
	int i=0;	
	
	for(i=0;i<120;i++)
	sum+=pow(E,-coeff_a*i)*err[i];
	
	return sum;
}
long double fact(int n)
{
	long int i=0;
	long double fact=1;
	for(i=1;i<=n;i++)
	fact*=i;
	return fact;
}

/*int size=10;
int future_size=10;
float pid_z(int leg)
{
	int i=0,j=0,k=0;
	cor_z[0]=0;// LEFT LEG CONTIBUTION
	
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
	float torso_wt=2734.9;
	com[0]+=121.358*torso_wt;
	com[1]+=0*torso_wt;
	com[2]+=0*torso_wt;

	com[0]=com[0]/(l_wt+r_wt+torso_wt);
	com[1]=com[1]/(l_wt+r_wt+torso_wt);
	com[2]=com[2]/(l_wt+r_wt+torso_wt);
	//printf(" COM X:%lf\tY:%lf\tZ:%lf\t",com[0],com[1],com[2]);
	return EXIT_SUCCESS;
	for(j=size;j>0;j--)
	{
		cor_z[j]=cor_z[j-1];
		prev_err_z[0][j]=prev_err_z[0][j-1];
	}
	prev_err_z[0][0]=err_z;
	
	for(i=1;i<size;i++)
	{
		for(j=0;j<size-i;j++)
		prev_err_z[i][j]=(prev_err_z[i-1][j]-prev_err_z[i-1][j+1]);  
		/// to get twice the error use  prev_err_z[i][j]=(prev_err_z[i-1][j]-prev_err_z[i-1][j+1])/2;
	}
	long  double val=1;
	
	//for(i=0;i<size;i++)
	//{
		//for(j=0;j<size;j++)
		//printf("%lf\t",prev_err_z[i][0]);
		//printf("\n");
	//}
	//printf("\n");
	/////// FORWARD INTERPOLATION
	/*for(k=0;k<future_size;k++)	
	{
		future_err_z[k]=0;
		for(i=0;i<size;i++)
		{
			val=1;
			for(j=0;j<i;j++)
			{
				val*=(size-j+k);
				//printf("%lf\t",val);
			}
			future_err_z[k]+=prev_err_z[i][size-i-1]*val/fact(i);
			//printf("%LE\t%lf\t%Le\t%Le\n",future_err_z[k],prev_err_z[i][size-i-1],val,fact(i));
		}
		//printf("%LE\n",future_err_z[k-1]);	
	}//
	///////// BACKWARD INTERPOLATION
	int exit=0;
	for(k=0;k<future_size;k++)
	{
		exit=0;
		future_err_z[k]=0;
		for(i=0;i<size&&exit==0;i++)
		{
			val=1;
			for(j=0;j<i;j++)
			{
				//printf("here");
				val*=(j+1+k);
			}
			if((prev_err_z[i+1][0]<0?prev_err_z[i+1][0]*-1:prev_err_z[i+1][0]*1)>(prev_err_z[i][0]<0?prev_err_z[i][0]*-1:prev_err_z[i][0]*1))
			exit=1;
		
			future_err_z[k]+=prev_err_z[i][0]*val/fact(i);
		}	
	}
	for(i=0;i<future_size;i++)
	{
		printf("NEXT ERROR :%lf\n",future_err_z[i]);
		//future_err_z[k]=0;
	}
	printf("\n");
	//printf("\n*************************COR %lf*********************************************\n",future_err_z[0]);
	cor_z[0]=0;
}
*/

float pid_z(int leg)
{
	float pk=1;
	float pd=0;
	
	int i=0;
	for(i=119;i>0;i--)
	cor_z[i]=cor_z[i-1];

	cor_z[0]=pk*err_z+pd*(err_z-prev_err_z);
	if(abs(cor_z[0])>50)
	{
		printf("*******************************************************************\n");
		cor_z[0]=50*cor_z[0]/abs(cor_z[0]);

	}
	prev_err_z=err_z;

	printf("CORR : %lf\n",cor_z[0]);
	
	
}

int init_walk()
{
	walk.time=1;
	walk.fps=60;
	
	/////// X-Lift Control /////
	walk.left.x.x_start=0.15;
	walk.left.x.x_end=0.85;
	walk.left.x.lift=40;
	walk.left.x.height=390;
	walk.left.x.freq=2*3.14;
	walk.left.x.displacement=1;	
	walk.left.x.phase=-1.57;
	
	walk.right.x.x_start=0.15;
	walk.right.x.x_end=0.85;
	walk.right.x.lift=40;
	walk.right.x.height=390;
	walk.right.x.freq=2*3.14;
	walk.right.x.displacement=1;	
	walk.right.x.phase=-1.57;
	
	/////// X-Land Control /////
	//float dec_time=0.75;	
	walk.left.x.x_land_start=0.9;//1-x_end    
	walk.left.x.x_land_increase_end=1;
	walk.left.x.x_land_decrease_start=1.5;
	walk.left.x.x_land_end=2;
	walk.left.x.x_land_freq=3.14;
	walk.left.x.x_land_displacement=1;
	walk.left.x.x_land_phase=-1.57;
	walk.left.x.x_land_dec=0;
	
	walk.right.x.x_land_start=0.9;//1-x_end    
	walk.right.x.x_land_increase_end=1;
	walk.right.x.x_land_decrease_start=1.5;
	walk.right.x.x_land_end=2;
	walk.right.x.x_land_freq=3.14;
	walk.right.x.x_land_displacement=1;
	walk.right.x.x_land_phase=-1.57;
	walk.right.x.x_land_dec=0;
	
	/////// X-Load Control /////
	walk.left.x.x_load_start=0.1;
	walk.left.x.x_load_increase_end=0.1;
	walk.left.x.x_load_decrease_start=0.1;
	walk.left.x.x_load_end=0.15;
	walk.left.x.x_load_displacement=0;//x_land_displacement;
	walk.left.x.x_load_phase=0;//-1.57;
	walk.left.x.x_load_freq=1.57;//3.14;
	walk.left.x.x_load_dec=0;
	
	walk.right.x.x_load_start=0.1;
	walk.right.x.x_load_increase_end=0.1;
	walk.right.x.x_load_decrease_start=0.1;
	walk.right.x.x_load_end=0.15;
	walk.right.x.x_load_displacement=0;//x_land_displacement;
	walk.right.x.x_load_phase=0;//-1.57;
	walk.right.x.x_load_freq=1.57;//3.14;
	walk.right.x.x_load_dec=0;
	//float x_load_window=0.05;
	
	/////// Z-Shift Control /////
	walk.left.z_shift.peak_start=0.0;
	walk.left.z_shift.peak_max=0.5;//0.5
	walk.left.z_shift.peak_stay=0.5;//0.75
	walk.left.z_shift.peak_end=1;
	walk.left.z_shift.zmax=32;
	walk.left.z_shift.zfreq=1.57;
	walk.left.z_shift.zphase=0;
	walk.left.z_shift.zdisplacement=0;
	
	walk.right.z_shift.peak_start=0.0;
	walk.right.z_shift.peak_max=0.5;//0.5
	walk.right.z_shift.peak_stay=0.5;//0.75
	walk.right.z_shift.peak_end=1;
	walk.right.z_shift.zmax=32;
	walk.right.z_shift.zfreq=1.57;
	walk.right.z_shift.zphase=0;
	walk.right.z_shift.zdisplacement=0;
	
	/////// Y Control /////
	walk.left.y.y_start=0.25;
	walk.left.y.y_end=0.75;
	walk.left.y.yr_start=0.25;
	walk.left.y.yr_end=0.75;

	walk.right.y.y_start=0.25;
	walk.right.y.y_end=0.75;
	walk.right.y.yr_start=0.25;
	walk.right.y.yr_end=0.75;

	/////// Z Control /////
	walk.left.z.z_start=0.25;
	walk.left.z.z_0=0.417;
	walk.left.z.z_move=0.5833;
	walk.left.z.z_end=0.75;
        
	walk.right.z.z_start=0.25;
	walk.right.z.z_0=0.417;
	walk.right.z.z_move=0.5833;
	walk.right.z.z_end=0.75;
        
   	 ///////Separation control////
    	walk.left.misc.seperation=5;
    	walk.right.misc.seperation=5;
	
	//////Turn Control////
	walk.left.turn.turn_start;
	walk.left.turn.turn_end;

	walk.right.turn.turn_start;
	walk.right.turn.turn_end;

	/////COMPLIANCE//
	walk.left.misc.cptime=1156566765;

	walk.right.misc.cptime=1156566765;

	/////// COM Trajectory //////
	//Z Movement
	walk.left.com_trajectory.z_com_start=0;
	walk.left.com_trajectory.z_com_peak=0.25;
	walk.left.com_trajectory.z_com_stay=0.75;
	walk.left.com_trajectory.z_com_end=1;
	walk.left.com_trajectory.z_com_max=32;
	walk.left.com_trajectory.z_static_inc=4;
	
	walk.right.com_trajectory.z_com_start=0;
	walk.right.com_trajectory.z_com_peak=0.25;
	walk.right.com_trajectory.z_com_stay=0.75;
	walk.right.com_trajectory.z_com_end=1;
	walk.right.com_trajectory.z_com_max=32;
	walk.right.com_trajectory.z_static_inc=4;

	//Y Movement

	walk.left.com_trajectory.y_com_start=0;
	walk.left.com_trajectory.y_com_end=1;

	walk.right.com_trajectory.y_com_start=0;
	walk.right.com_trajectory.y_com_end=1;

}

float getWalkX(int leg,float t_frac)
{
	
	////X LIFT////		
	if(t_frac>x_start&&t_frac<x_end)
	x-=lift*(sin(freq*(t_frac-x_start)/(x_end-x_start)+phase)+displacement)/(1+displacement);
		
	///// Soften landing     
	if(t_frac>x_land_start&&t_frac<x_land_increase_end)
	x-=x_land_dec*(sin(x_land_freq*(t_frac-x_land_start)/(x_land_increase_end-x_land_start)+x_land_phase)+x_land_displacement)/(1+x_land_displacement);
	else if(t_frac>x_land_increase_end&&t_frac<x_land_decrease_start)
	x-=x_land_dec;
	else if(t_frac>x_land_decrease_start&&t_frac<x_land_end)
	x-=x_land_dec*(sin(x_land_freq*(t_frac-x_land_decrease_start)/(x_land_end-x_land_decrease_start)+x_land_phase)+x_land_displacement)/(1+x_land_displacement);
	//x-=x_land_dec*(sin(x_land_freq*(t_frac-x_end+x_land_window)/(2*x_land_window)+x_land_phase)+x_land_displacement)/(1+x_land_displacement);
		
	
}

float getWalkXR(float t_frac)
{
	///// Assist weight shift
	if(t_frac>x_load_start&&t_frac<=x_load_increase_end)
	xr-=x_load_dec*(sin(x_load_freq*(t_frac-x_load_start)/(x_load_increase_end-x_load_start)+x_load_phase)+x_load_displacement)/(1+x_load_displacement);
	else if(t_frac>x_load_increase_end&&t_frac<=x_load_decrease_start)
	xr-=x_load_dec;
	else if(t_frac>x_load_decrease_start&&t_frac<=x_load_end)
	xr-=x_load_dec*(1-(sin(x_load_freq*(t_frac-x_load_decrease_start)/(x_load_end-x_load_decrease_start)+x_load_phase)+x_load_displacement)/(1+x_load_displacement));

}

int walk3(int leg,float yin,float yfi,float yrin,float yrfi, float zin, float zfi,float zrin, float zrfi, float thetain, float thetafi,float thetarin, float thetarfi)
{
	FILE *f;

	float time=1;
	int fps=60;
	
	/////// X-Lift Control /////
	float x_start=0.15;
	float x_end=0.85;
	float lift=40;
	float height=390;
	float freq=2*3.14;
	float displacement=1;	
	float phase=-1.57;
	
	/////// X-Land Control /////
	//float dec_time=0.75;	
	float x_land_start=0.9;//1-x_end    
	float x_land_increase_end=1;
	float x_land_decrease_start=1.5;
	float x_land_end=2;
	float x_land_freq=3.14;
	float x_land_displacement=1;
	float x_land_phase=-1.57;
	float x_land_dec=0;
	
	/////// X-Load Control /////
	float x_load_start=0.1;
	float x_load_increase_end=0.1;
	float x_load_decrease_start=0.1;
	float x_load_end=0.15;
	float x_load_displacement=0;//x_land_displacement;
	float x_load_phase=0;//-1.57;
	float x_load_freq=1.57;//3.14;
	float x_load_dec=0;
	//float x_load_window=0.05;
	
	/////// Z-Shift Control /////
	float peak_start=0.0;
	float peak_max=0.5;//0.5
	float peak_stay=0.5;//0.75
	float peak_end=1;
	float zmax=32;
	float zfreq=1.57;
	float zphase=0;
	float zdisplacement=0;
	
	/////// Y Control /////
	float y_start=0.25;
	float y_end=0.75;
	float yr_start=0.25;
	float yr_end=0.75;

	/////// Z Control /////
	float z_start=0.25;
	float z_0=0.417;
	float z_move=0.5833;
	float z_end=0.75;
        
   	 ///////Separation control////
    	float seperation=5;
	
	//////Turn Control////
	float turn_start;
	float turn_end;

	/////COMPLIANCE//
	float cptime=1156566765;

	/////// COM Trajectory //////
	//Z Movement
	float z_com_start=0;
	float z_com_peak=0.25;
	float z_com_stay=0.75;
	float z_com_end=1;
	float z_com_max=32;
	float z_static_inc=4;
	//Y Movement

	float y_com_start=0;
	float y_com_end=1;
	
	/////// Live Variables /////
	int frame=0;
	float t_frac=0,t=0;
	float x,xr,y,z,yr,zr,z_shift,phi,phir;
	float yreal,yrreal,zreal,zrreal;
	float com_y,com_z;
	
	/////// Packet Generation Variables /////
	int adchksum=7+NOM*3,i=0;
	float *fval;
	float val[5],valr[5];
	int mot=0*(1-leg)+20*leg;
	int revmot=20*(1-leg)+0*leg;
	int value=0;
        
	compliance(leg,10,10);	
	for(frame=0;frame<time*fps;frame++,t+=1.0/fps)
	{
		t_frac=t/time;
		x=height;
		xr=height;
		
		
		/////// COM Trajectory /////
		///////// Z Axis ////////
		if(t_frac<=z_com_start)
		com_z=0;
		else if(t_frac>z_com_start&&t_frac<=z_com_peak)
		com_z=z_com_max*(sin(3.14*(t_frac-z_com_start)/(z_com_peak-z_com_start)-1.57)+1)/2;//0*(1-(t_frac-z_com_start)/(z_com_peak-z_com_start))+z_com_max*((t_frac-z_com_start)/(z_com_peak-z_com_start));
		else if(t_frac>z_com_peak&&t_frac<=z_com_stay)
		com_z=z_com_max;//+z_static_inc*(sin(2*3.14*(t_frac-z_com_peak)/(z_com_stay-z_com_peak)-1.57)+1)/2;
		else if(t_frac>=z_com_stay&&t_frac<=z_com_end)
		com_z=z_com_max-z_com_max*(sin(3.14*(t_frac-z_com_stay)/(z_com_end-z_com_stay)-1.57)+1)/2;
		else
		com_z=0;
		
		/*if(frame!=0)
		com_z=(com[1]-left_joints[6][1]*leg-right_joints[6][1]*(1-leg)+(-55)*leg+(55)*(1-leg));
		else
		com_z=0;
		*/
		
		/////Z SHIFT//////
		/*if(t_frac<peak_start)
		z_shift=0;
		else if(t_frac>peak_start&&t_frac<peak_max)
		z_shift=zmax*(sin(zfreq*(t_frac-peak_start)/(peak_max-peak_start)+zphase)+zdisplacement)/(1+zdisplacement);
		else if(t_frac>peak_stay&&t_frac<peak_end)
		z_shift=zmax-zmax*(sin(zfreq*(t_frac-peak_stay)/(peak_end-peak_stay)+zphase)+zdisplacement)/(1+zdisplacement);
		else if(t_frac>peak_end)
		z_shift=0;
		*/
		z_shift=com_z;
		
		////Y MOVEMENT////
		if(t_frac<y_start)
		y=yin;  
                else if(t_frac>=y_start&&t_frac<y_end)
		y=scurve2(yin,yfi,t_frac-y_start,y_end-y_start);
		else
		y=yfi;


		/////YR MOVT/////

                if(t_frac<yr_start)
		yr=yrin;
		else if(t_frac>=yr_start&&t_frac<yr_end)
		yr=scurve2(yrin,yrfi,t_frac-yr_start,yr_end-yr_start);
		else 
		yr=yrfi;
		
		
		//////////////// Z MOVEMENT///
		if(t_frac<z_start)
		{
			z=zin;
			zr=zrin;
		}
		else if(t_frac>z_start&&t_frac<z_0)	
	        {
			z=scurve(zin,zin+zrin,t_frac-z_start,z_0-z_start);
			zr=scurve(zrin,0,t_frac-z_start,z_0-z_start);
		}
		else if(t_frac>z_0&&t_frac<z_move)
		{

			z=scurve(zin+zrin,zfi+zrfi,t_frac-z_0,z_move-z_0);
			zr=scurve(0,0,t_frac-z_0,z_move-z_0);
		}
		else if(t_frac>z_move&&t_frac<z_end)
		{
			z=scurve(zrfi+zfi,zfi,t_frac-z_move,z_end-z_move);
			zr=scurve(0,zrfi,t_frac-z_move,z_end-z_move);
		}
		////TURN////
		if(t_frac<turn_start)
		{
			phi=thetain;
			phir=thetarin;
		}
		else if(t_frac>turn_start&&t_frac<turn_end)
		{
			phi=scurve(thetain,thetafi,t_frac-turn_start,turn_end-turn_start);
			phir=scurve(thetarin,thetarfi,t_frac-turn_start,turn_end-turn_start);
		}
		else
		{
			phi=thetafi;
			phir=thetarfi;
		}
		

		///////// Y Axis ////////
		if(t_frac<=y_com_start)
		com_y=0;
		else if(t_frac>y_com_start&&t_frac<=y_com_end)
		com_y=(yrin-yrfi)*(t_frac-y_com_start)/(y_com_end-y_com_start);
		else
		com_y=(yrin-yrfi);
		
		f=fopen(expect_com,"a");
		if(f!=NULL)
		{
			fprintf(f,"0\t%lf\t%lf\n",pow(-1,leg)*com_z,com_y+18+COM_INC);
			fclose(f);
		}
	
		/////Compliance////
		if(compare_float(cptime,t_frac)==1)
		compliance(leg,10,32);
		
		//zr+=err_z;
		//z-=err_z;
		
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
			
		zreal+=z_shift;
		zrreal-=z_shift;
		/*if(abs(err_z)<25)
		{
			zrreal+=err_z*0.2;
			zreal-=err_z*0.2;
		}*/
		zrreal+=0.5*sum_err(cor_z,1);
		zreal-=0.5*sum_err(cor_z,1);
		fval=generateik(x,yreal,zreal+seperation,0);
		for(i=0;i<5;i++)
		val[i]=fval[i];
		fval=generateik(xr,yrreal,zrreal+seperation,0);
		for(i=0;i<5;i++)
		valr[i]=fval[i];
		
		///// PROJECTION SYSTEM 
		target_left[0]=val[0]*(1-leg)+valr[0]*leg;
		target_left[1]=val[1]*(1-leg)+valr[1]*leg;	
		target_left[2]=val[2]*(1-leg)+valr[2]*leg;
		target_left[3]=val[3]*(1-leg)+valr[3]*leg;
		target_left[4]=rotation(4,phi*(1-leg)+phir*leg);
		target_left[5]=val[4]*(1-leg)+valr[4]*leg;
		
		target_right[0]=valr[0]*(1-leg)+val[0]*leg;
		target_right[1]=valr[1]*(1-leg)+val[1]*leg;
		target_right[2]=valr[2]*(1-leg)+val[2]*leg;
		target_right[3]=valr[3]*(1-leg)+val[3]*leg;
		target_right[4]=rotation(24,phir*(1-leg)+phi*leg);
		target_right[5]=valr[4]*(1-leg)+val[4]*leg;
		
		projection_system(leg);
		pid_z(leg);
		
		err_z=rot_com[1]*pow(-1,leg)-com_z;
		
		//err_z+=err_z>0?-3:3;
		//printf("COM : %lf\tE_COM : %lf\t ERROR : %lf\n",rot_com[1]*pow(-1,leg),com_z,err_z);

		printf("ERROR : %lf\n",err_z);

		for(i=0;i<NOM;i++)
		{
			if(tx_packet[7+i*3]==mot)
			{
				value=val[0]+offset[tx_packet[7+i*3]];
				if(value>=0 && value<=4096)
				{
					tx_packet[8+i*3]=value%256;
					tx_packet[9+i*3]=value/256;
				}
				//printf("value %d \t %d\n",tx_packet[7+i*3],value);				
			}
			else if(tx_packet[7+i*3]==mot+1)
			{
				value=val[1]+offset[tx_packet[7+i*3]];
				if(value>=0 && value<=4096)
				{
					tx_packet[8+i*3]=value%256;
					tx_packet[9+i*3]=value/256;
				}
				//printf("value %d \t %d\n",tx_packet[7+i*3],value);				
			}
			else if(tx_packet[7+i*3]==mot+2||tx_packet[7+i*3]==mot+12)
			{
				value=val[2]+offset[tx_packet[7+i*3]];
				if(value>=0 && value<=4096)
				{
					tx_packet[8+i*3]=value%256;
					tx_packet[9+i*3]=value/256;
				}
				//printf("value %d \t %d\n",tx_packet[7+i*3],value);				
			}
			else if(tx_packet[7+i*3]==mot+3)
			{
				value=val[3]+offset[tx_packet[7+i*3]];
				if(value>=0 && value<=4096)
				{
					tx_packet[8+i*3]=value%256;
					tx_packet[9+i*3]=value/256;
				}
				//printf("value %d \t %d\n",tx_packet[7+i*3],value);				
			}
			else if(tx_packet[7+i*3]==mot+4)
			{
				value=rotation(mot+4,phi)+offset[tx_packet[7+i*3]];
				if(value>=0 && value<=1024)
				{
					tx_packet[8+i*3]=value%256;
					tx_packet[9+i*3]=value/256;
				}
				//printf("value %d \t %d\n",tx_packet[7+i*3],value);				
			}
			else if(tx_packet[7+i*3]==mot+5)
			{
				value=val[4]+offset[tx_packet[7+i*3]];
				if(value>=0 && value<=4096)
				{
					tx_packet[8+i*3]=value%256;
					tx_packet[9+i*3]=value/256;
				}
				//printf("value %d \t %d\n",tx_packet[7+i*3],value);				
			}
			else if(tx_packet[7+i*3]==revmot)
			{
				value=valr[0]+offset[tx_packet[7+i*3]];
				if(value>=0 && value<=4096)
				{
					tx_packet[8+i*3]=value%256;
					tx_packet[9+i*3]=value/256;
				}
				//printf("value %d \t %d\n",tx_packet[7+i*3],value);				
			}
			else if(tx_packet[7+i*3]==revmot+1)
			{
				value=valr[1]+offset[tx_packet[7+i*3]];
				if(value>=0 && value<=4096)
				{
					tx_packet[8+i*3]=value%256;
					tx_packet[9+i*3]=value/256;
				}
				//printf("value %d \t %d\n",tx_packet[7+i*3],value);				
			}
			else if(tx_packet[7+i*3]==revmot+2||tx_packet[7+i*3]==revmot+12)
			{
				value=valr[2]+offset[tx_packet[7+i*3]];
				if(value>=0 && value<=4096)
				{
					tx_packet[8+i*3]=value%256;
					tx_packet[9+i*3]=value/256;
				}
				//printf("value %d \t %d\n",tx_packet[7+i*3],value);				
			}
			else if(tx_packet[7+i*3]==revmot+3)
			{
				value=valr[3]+offset[tx_packet[7+i*3]];
				if(value>=0 && value<=4096)
				{
					tx_packet[8+i*3]=value%256;
					tx_packet[9+i*3]=value/256;
				}
				//printf("value %d \t %d\n",tx_packet[7+i*3],value);				
			}
			else if(tx_packet[7+i*3]==revmot+4)
			{
				value=rotation(revmot+4,phir)+offset[tx_packet[7+i*3]];
				if(value>=0 && value<=1024)
				{
					tx_packet[8+i*3]=value%256;
					tx_packet[9+i*3]=value/256;
				}
				//printf("value %d \t %d\n",tx_packet[7+i*3],value);				
			}
			else if(tx_packet[7+i*3]==revmot+5)
			{
				value=valr[4]+offset[tx_packet[7+i*3]];
				if(value>=0 && value<=4096)
				{
					tx_packet[8+i*3]=value%256;
					tx_packet[9+i*3]=value/256;
				}
				//printf("value %d \t %d\n",tx_packet[7+i*3],value);				
			}	
		}
		tx_packet[adchksum]=255;

		for(i=2;i<adchksum;i++)
		tx_packet[adchksum]-=tx_packet[i];

		if(iwrite(MOTOR_DATA,&fd,&ftdic1,tx_packet,tx_packet[3]+4)<0)
		{
			//printf("ERROR : SENDING PACKET\n");
		}
		usleep(16670);
		//usleep(1000000.0/fps);
	}
	int cor_i=0;	
	for(;cor_i<120;cor_i++)
	cor_z[cor_i]*=-1;
	printf("******************************************************************************************\n");
	COUNTER+=STEP*2;
	COM_INC+=(yrin-yrfi);
	return EXIT_SUCCESS;
		
	
}
			
int walk2(int leg,float yin,float yfi,float yrin,float yrfi, float zin, float zfi,float zrin, float zrfi, float thetain, float thetafi,float thetarin, float thetarfi, float time)
{	
	// IMU DEFINITIONS
	
	int frames=1;
	float tolerance[2][3];
	tolerance[0][0]=5;
	tolerance[0][1]=15;
	tolerance[0][2]=50;
	tolerance[1][0]=8;
	tolerance[1][1]=15;
	tolerance[1][2]=50;
	
	float change[4];
	float lev2=0;
	float distance=sqrt(20*20+30*30);
	
	// GEN DEFINITIONS
	int i=0,h=0,j=0;
	
	//COOD DEFINITIONS
	float x,y,z,yr,zr,z_shift;
	float t=0;
	float t_frac;
	float l=0;
	float freq=2*3.14;    //controls how fast x varies with time
	
	// VARIABLES TO NOT MESS WITH 
	
			//X CONTROL
	float dis=0.5;        //controls how much sinx is displaced above x axis
	float phase=-1.57;	  //controls initial phase of sinx graph  
	float xmax=360;   //maximum x in walk
	float cpxval=380;
	float cptime=(freq-asin(((390-cpxval)/(390-xmax)*(1+dis))-(dis))+phase)/freq*time;

			//Z SHIFT CONTROL
	float xzval=390;//+5*(1-leg);   // x at which zmax is achieved   
	float zmax=53+0*(leg)+zfi/6-zin/6;///+2*(1-leg);
	float ztime=(asin(((390-xzval)/(390-xmax)*(1+dis))-(dis))-phase)/freq*time; // time of zmax ,xzval 
	float zfreq=zmax/sin(3.14*ztime/time);       //how quickly z increases to max value-- redundant

			//Y CONTROL
	float xyvali=380;
	float xyvalf=380;
	float xyrvali=380;//-10*leg+10*(1-leg);//+10*(1-leg)+10*leg;
	float xyrvalf=380;
	float yi_tfrac=(asin(((390-xyvali)/(390-xmax)*(1+dis))-(dis))-phase)/freq*time;
	float yf_tfrac=(freq-asin(((390-xyvalf)/(390-xmax)*(1+dis))-(dis))+phase)/freq*time;;
	float yri_tfrac=(asin(((390-xyrvali)/(390-xmax)*(1+dis))-(dis))-phase)/freq*time;
	float yrf_tfrac=(freq-asin(((390-xyrvalf)/(390-xmax)*(1+dis))-(dis))+phase)/freq*time;;
			//Z CONTROL
	float xzvali=370;
	float xzvalf=370;
	float zi_tfrac=(asin(((390-xzvali)/(390-xmax)*(1+dis))-(dis))-phase)/freq*time;
	float zf_tfrac=(freq-asin(((390-xzvalf)/(390-xmax)*(1+dis))-(dis))+phase)/freq*time;
	
			// TURN CONTROL
	float xtuval=380; 
	float turnt=(asin(((390-xtuval)/(390-xmax)*(1+dis))-(dis))-phase)/freq*time;
	//float turnt=0.2*time;
	//float ch=0;
	//float chr=0;
	
	// DIRTY DECLARATIONS
	float ch3=0,chr3=0;
	float change3=50;
	float changer3=25;
	float x3val=390;
	float turn3=(asin(((390-x3val)/(390-xmax)*(1+dis))-(dis))-phase)/freq*time;
	
	//ROTATION MATRIX
	float zreal,yreal,zrreal,yrreal;
	float phi,rphi;
	
	
	//PACKET DEFINITIONS
	set_walk_offset();
	int adchksum=7+NOM*3;
	float* fval;
	float val[5],valr[5]; 
	
	//MOTOR DEFINITIONS
	int mot,revmot;
	int value;
	
	//printf("LEG : %d\n",leg);
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
	//if compare
	compliance(leg,10,10);
	//usleep(16670);
	for(;t<time;t+=(float)1/60)
	{
		t_frac=t/time;
		
		////////////// Y MOVEMENT
		if((int)((t-yi_tfrac)*100)<0)
		{
			y=yin;
		}
		else if((int)((t-yf_tfrac)*100)<0)
		{
			y=scurve(yin,yfi,t-yi_tfrac,yf_tfrac-yi_tfrac);
		}
		else
		{
			y=yfi;
		}
		
		if((int)((t-yri_tfrac)*100)<0)
		{
			yr=yrin;
		}
		else if((int)((t-yrf_tfrac)*100)<0)
		{
			yr=scurve(yrin,yrfi,t-yi_tfrac,yf_tfrac-yi_tfrac);
		}
		else
		{
			yr=yrfi;
		}
		
		//////////////// Z MOVEMENT
		if((int)((t-zi_tfrac)*100)<0)
		{
			z=scurve(zin,zin+zrin,t-0,zi_tfrac-0);
			zr=scurve(zrin,0,t-0,zi_tfrac-0);
		}
		else if((int)((t-zf_tfrac)*100)<0)
		{
			z=scurve(zin+zrin,zfi+zrfi,t-zi_tfrac,zf_tfrac-zi_tfrac);
			zr=0;
		}
		else
		{
			z=scurve(zrfi+zfi,zfi,t-zf_tfrac,time-zf_tfrac);
			zr=scurve(0,zrfi,t-zf_tfrac,time-zf_tfrac);
		}
		
		///////////// TURN MOVEMENT
		if((int)((t-turnt)*100)<=0)
		{
			phi=thetain;
			rphi=thetarin;
		}
		else if((int)((t-(time-turnt))*100)<=0)
		{
			//ch=scurve(0,theta,t-turnt,time-2*turnt)-scurve(0,theta,t-turnt-0.016670,time-2*turnt);
			//chr=scurve(0,thetar,t-turnt,time-2*turnt)-scurve(0,thetar,t-turnt-0.016670,time-2*turnt);
			phi=scurve(thetain,thetafi,t-turnt,time-2*turnt);
			rphi=scurve(thetarin,thetarfi,t-turnt,time-2*turnt);
		}
		else
		{
			phi=thetafi;
			rphi=thetarfi;
		}
				
		/*///////////////////////////// MY CODE BEING MADE DIRTY ////////////////////////////
		if((int)((t-turn3)*100)<=0)
		{
			ch3=0;
			chr3=0;
			
		}
		else if((int)((t-(time-turn3))*100)<=0)
		{
			ch3=change3*sin(3.14*t-turn3/time-2*turn3);
			chr3=changer3*sin(3.14*t-turn3/time-2*turn3);
			
		}
		else
		{
			ch3=chr3=0;
		}
		//////////////////////////// DIRTYNESS ENDS //////////////////////////////////////	
		*/
		
		ch3=chr3=0;
		x=390-(390-xmax)*(max(0,sin((freq*t_frac)+phase)+dis)/(1+dis));
		z_shift=min(zmax,zfreq*(sin(2*3.14*t_frac-1.57)+1)/2);
		
		for(i=0;i<128;i++)
		{
			tx_packet[i]=last_tx_packet[i];
			//printf("%d\t",tx_packet[i]);
		}
		
		//printf("Time : %lf\n",t);
		
		if((int)((t-cptime)*100)==0)
		{
			compliance(leg,32,10);
		}
		///////////////////////////////////////// IMU ANGLE AND LEG REPOSITIONING .//////////////////////////////////////////
		//imucal();
		/*if((int)(t*60)%frames==0)
		{
			imucal();
			for(j=0;j<2;j++)
			{
				if(abs(avg_angle[j]-0)<=tolerance[j][0])
				{
					//printf("LEVEL 0\n");
					change[j]=0;
				}
				else if(abs(avg_angle[j]-0)>tolerance[j][0]&&abs(avg_angle[j]-0)<=tolerance[j][1])
				{
					printf("LEVEL 1\n");
					change[j]=avg_angle[j];
				}
				else if(abs(avg_angle[j]-0)>tolerance[j][1]&&abs(avg_angle[j]-0)<=tolerance[j][2])
				{
					lev2=1;
					//printf("******************************LEVEL 2*********************************\n");
					change[j]=0;
					
					float alpha=atan(tan(abs(avg_angle[1])*3.14/180)/tan(avg_angle[0]*3.14/180));
					float yfi_tar=distance*cos(alpha);
					float yrfi_tar=-distance*cos(alpha);
					float zfi_tar=distance*sin(alpha);
					float zrfi_tar=distance*sin(alpha);
					//printf("A : %lf\tY_TAR : %lf\tYR_TAR : %lf\tZ_TAR : %lf\tZr_TAR : %lf\n",alpha,yfi_tar,yrfi_tar,zfi_tar,zrfi_tar);		
		
					if((int)((t-yi_tfrac)*100)<0)
					{
						yfi=yfi_tar;
						yrfi=yrfi_tar;
						//y=yin;
						//yr=yrin;
					}
					else if((int)((t-yf_tfrac)*100)<0)
					{
						float y_frac=scurve(0,1,t-yi_tfrac,yf_tfrac-yi_tfrac);
						yfi=yfi_tar;
						yrfi=yrfi_tar;
						yin=(y-yfi*y_frac)/(1-y_frac);
						yrin=(yr-yrfi*y_frac)/(1-y_frac);
						//y=scurve(yin,yfi,t-yi_tfrac,yf_tfrac-yi_tfrac);
						//yr=scurve(yrin,yrfi,t-yi_tfrac,yf_tfrac-yi_tfrac);
					}
					else
					{
						//printf("Y Reached End\n");
						//y=yfi;
						//yr=yrfi;
					}
		
					if((int)((t-zi_tfrac)*100)<0)
					{
						zrfi=zrfi_tar;
						zfi=zfi_tar;
						//z=scurve(zin,zin+zrin,t-0,zi_tfrac-0);
						//zr=scurve(zrin,0,t-0,zi_tfrac-0);
					}
					else if((int)((t-zf_tfrac)*100)<0)
					{
						float z_frac=scurve(0,1,t-zi_tfrac,zf_tfrac-zi_tfrac);
						zrfi=zrfi_tar;
						zfi=zfi_tar;
						zin=(z-(zrfi+zfi)*z_frac)/(1-z_frac);
						zrin=0;
						//z=scurve(zin+zrin,zfi+zrfi,t-zi_tfrac,zf_tfrac-zi_tfrac);
						//zr=0;
					}
					else
					{	
						//printf("Z Reached End\n");
						//z=scurve(zrfi+zfi,zfi,t-zf_tfrac,time-zf_tfrac);
						//zr=scurve(0,zrfi,t-zf_tfrac,time-zf_tfrac);
					}
					
					if((int)((t-turnt)*100)<=0)
					{
						//phi=thetain;
						//rphi=thetarin;
						thetafi=0;
						thetarfi=0;
					}
					else if((int)((t-(time-turnt))*100)<=0)
					{
						float turn_frac=(0,1,t-turnt,time-2*turnt);
						thetafi=0;
						thetarfi=0;
						thetain=(phi-thetafi*turn_frac)/(1-turn_frac);
						thetarin=(rphi-thetarfi*turn_frac)/(1-turn_frac);
						//phi=scurve(thetain,thetafi,t-turnt,time-2*turnt);
						//rphi=scurve(thetarin,thetarfi,t-turnt,time-2*turnt);
					}
					else
					{
						//printf("Theta Reached End\n");
						//phi=thetafi;
						//rphi=thetarfi;
					}
		
				}
				else if(abs(avg_angle[j]-0)>tolerance[j][2])
				{
					//printf("LEVEL 3\n");
					change[j]=0;
					Y_END=0;
					YR_END=0;
					Z_END=0;
					ZR_END=0;
					THETA_END=0;
					THETAR_END=0;
					level3();
					return EXIT_FAILURE;
					// return level3_code
				}
			}
		}
		
		zr+=z_shift*pow(-1,leg+1)*sin(change[1]*3.14/180);
		z-=z_shift*pow(-1,leg+1)*sin(change[1]*3.14/180);
		
		//y+=y*0.2*sin(1.57*change[0]/tolerance[0][1]);
		//yr+=yr*0.2*sin(1.57*change[0]/tolerance[0][1]);
		y+=z_shift*sin(change[0]*3.14/180);
		yr+=z_shift*sin(change[0]*3.14/180);
		*/
		/////////////////////////////////////////////////////////////////////////////////////////
		/////////////////////////////////////// ZMP FOLLOW ///////////////////////////////////////
		
		float threshold[2][2];
		threshold[0][0]=10;
		threshold[0][1]=35;
		threshold[1][0]=10;
		threshold[1][1]=35;
		

		//if(abs(rot_com[1])>threshold[0][0] && abs(rot_com[1])<threshold[0][1])
		//	zr+=pow(-1,leg+1)*rot_com[1];
		//if(abs(rot_com[2])>threshold[1][0] && abs(rot_com[2])<threshold[1][1])
		//	yr+=rot_com[2];
		//printf("X : %lf\tY : %lf\tYR : %lf\tZ : %lf\tZr : %lf\n",x,y,yr,z,zr);		
		//// ROTATION MATRIX ///////////
		if(leg==0)
		{
			zreal=z*cos(-phi*3.14/180)+y*sin(-phi*3.14/180);
			yreal=-z*sin(-phi*3.14/180)+y*cos(-phi*3.14/180);
			
			zrreal=zr*cos(rphi*3.14/180)-yr*sin(rphi*3.14/180);
			yrreal=zr*sin(rphi*3.14/180)+yr*cos(rphi*3.14/180);
		}
		else if(leg==1)
		{
			zreal=z*cos(phi*3.14/180)-y*sin(phi*3.14/180);
			yreal=z*sin(phi*3.14/180)+y*cos(phi*3.14/180);
			
			zrreal=zr*cos(-rphi*3.14/180)+yr*sin(-rphi*3.14/180);
			yrreal=-zr*sin(-rphi*3.14/180)+yr*cos(-rphi*3.14/180);
			
		}
		
		//printf("Phi : %lf\tRPhi : %lf\tY : %lf\tYR : %lf\tZ : %lf\tZr : %lf\n",phi,rphi,yreal,yrreal,zreal,zrreal);		
				
		zreal+=z_shift;
		zrreal-=z_shift;
		
		fval=generateik(x,yreal,zreal,0);
		for(h=0;h<5;h++)
		val[h]=fval[h];
		fval=generateik(390,yrreal,zrreal,0);
		for(h=0;h<5;h++)
		valr[h]=fval[h];
		

		///// PROJECTION SYSTEM 
		target_left[0]=val[0]*(1-leg)+valr[0]*leg;
		target_left[1]=val[1]*(1-leg)+valr[1]*leg;
		target_left[2]=val[2]*(1-leg)+valr[2]*leg;
		target_left[3]=val[3]*(1-leg)+valr[3]*leg;
		target_left[4]=rotation(4,phi*(1-leg)+rphi*leg);
		target_left[5]=val[4]*(1-leg)+valr[4]*leg;
		
		target_right[0]=valr[0]*(1-leg)+val[0]*leg;
		target_right[1]=valr[1]*(1-leg)+val[1]*leg;
		target_right[2]=valr[2]*(1-leg)+val[2]*leg;
		target_right[3]=valr[3]*(1-leg)+val[3]*leg;
		target_right[4]=rotation(24,rphi*(1-leg)+phi*leg);
		target_right[5]=valr[4]*(1-leg)+val[4]*leg;
		
		projection_system(leg);
		
		for(i=0;i<NOM;i++)
		{
			if(tx_packet[7+i*3]==mot)
			{
		
				value=tx_packet[8+i*3]+tx_packet[9+i*3]*256;
				tx_packet[adchksum]+=tx_packet[8+i*3]+tx_packet[9+i*3];
				value=val[0]+offset[tx_packet[7+i*3]];
				if(value>=0 && value<=4096)
				{
					tx_packet[8+i*3]=value%256;
					tx_packet[9+i*3]=value/256;
				}
				tx_packet[adchksum]-=tx_packet[8+i*3]+tx_packet[9+i*3];
				//printf("value %d \t %d\n",tx_packet[7+i*3],value);				
			}
			if(tx_packet[7+i*3]==mot+1)
			{
				value=tx_packet[8+i*3]+tx_packet[9+i*3]*256;
				tx_packet[adchksum]+=tx_packet[8+i*3]+tx_packet[9+i*3];
				value=val[1]+offset[tx_packet[7+i*3]];
				if(value>=0 && value<=4096)
				{
					tx_packet[8+i*3]=value%256;
					tx_packet[9+i*3]=value/256;
				}
				tx_packet[adchksum]-=tx_packet[8+i*3]+tx_packet[9+i*3];
				//printf("value %d \t %d\n",tx_packet[7+i*3],value);				
			}
			if(tx_packet[7+i*3]==mot+2||tx_packet[7+i*3]==mot+12)
			{
				value=tx_packet[8+i*3]+tx_packet[9+i*3]*256;
				tx_packet[adchksum]+=tx_packet[8+i*3]+tx_packet[9+i*3];
				value=val[2]+offset[tx_packet[7+i*3]];
				if(value>=0 && value<=4096)
				{
					tx_packet[8+i*3]=value%256;
					tx_packet[9+i*3]=value/256;
				}
				tx_packet[adchksum]-=(tx_packet[8+i*3]+tx_packet[9+i*3]);
				//printf("value %d \t %d\n",tx_packet[7+i*3],value);
			}
			if(tx_packet[7+i*3]==mot+3)
			{
				value=tx_packet[8+i*3]+tx_packet[9+i*3]*256;
				tx_packet[adchksum]+=tx_packet[8+i*3]+tx_packet[9+i*3];
				value=val[3]+offset[tx_packet[7+i*3]]+ch3;
				if(value>=0 && value<=4096)
				{
					tx_packet[8+i*3]=value%256;
					tx_packet[9+i*3]=value/256;
				}
				tx_packet[adchksum]-=tx_packet[8+i*3]+tx_packet[9+i*3];
				//printf("value %d \t %d\n",tx_packet[7+i*3],value);				
			}printf("******************************************************************************************\n");
		
			if(tx_packet[7+i*3]==mot+4)
			{
				value=tx_packet[8+i*3]+tx_packet[9+i*3]*256;
				tx_packet[adchksum]+=tx_packet[8+i*3]+tx_packet[9+i*3];
				//value-=floor(pow(-1,leg)*ch*1023/300);
				value=rotation(mot+4,phi)+offset[tx_packet[7+i*3]];
				if(value>=0 && value<=1023)
				{
					tx_packet[8+i*3]=value%256;
					tx_packet[9+i*3]=value/256;
				}
				tx_packet[adchksum]-=tx_packet[8+i*3]+tx_packet[9+i*3];
				//printf("value %d \t %d\n",tx_packet[7+i*3],value);				
			}
			if(tx_packet[7+i*3]==mot+5)
			{
				value=tx_packet[8+i*3]+tx_packet[9+i*3]*256;
				tx_packet[adchksum]+=tx_packet[8+i*3]+tx_packet[9+i*3];
				value=val[4]+offset[tx_packet[7+i*3]];
				if(value>=0 && value<=4096)
				{
					tx_packet[8+i*3]=value%256;
					tx_packet[9+i*3]=value/256;
				}
				tx_packet[adchksum]-=tx_packet[8+i*3]+tx_packet[9+i*3];
				//printf("value %d \t %d\n",tx_packet[7+i*3],value);				
			}
			if(tx_packet[7+i*3]==revmot)
			{
				value=tx_packet[8+i*3]+tx_packet[9+i*3]*256;
				tx_packet[adchksum]+=tx_packet[8+i*3]+tx_packet[9+i*3];
				value=valr[0]+offset[tx_packet[7+i*3]];
				if(value>=0 && value<=4096)
				{
					tx_packet[8+i*3]=value%256;
					tx_packet[9+i*3]=value/256;
				}
				tx_packet[adchksum]-=tx_packet[8+i*3]+tx_packet[9+i*3];
				//printf("value %d \t %d\n",tx_packet[7+i*3],value);				
			}
			if(tx_packet[7+i*3]==revmot+1)
			{
				value=tx_packet[8+i*3]+tx_packet[9+i*3]*256;
				tx_packet[adchksum]+=tx_packet[8+i*3]+tx_packet[9+i*3];
				value=valr[1]+offset[tx_packet[7+i*3]];
				if(value>=0 && value<=4096)
				{
					tx_packet[8+i*3]=value%256;
					tx_packet[9+i*3]=value/256;
				}
				tx_packet[adchksum]-=tx_packet[8+i*3]+tx_packet[9+i*3];
				//printf("value %d \t %d\n",tx_packet[7+i*3],value);				
			}
			if(tx_packet[7+i*3]==revmot+2||tx_packet[7+i*3]==revmot+12)
			{
				value=tx_packet[8+i*3]+tx_packet[9+i*3]*256;
				tx_packet[adchksum]+=tx_packet[8+i*3]+tx_packet[9+i*3];
				value=valr[2]+offset[tx_packet[7+i*3]];
				if(value>=0 && value<=4096)
				{
					tx_packet[8+i*3]=value%256;
					tx_packet[9+i*3]=value/256;
				}
				tx_packet[adchksum]-=(tx_packet[8+i*3]+tx_packet[9+i*3]);
				//printf("value %d \t %d\n",tx_packet[7+i*3],value);
			}
			if(tx_packet[7+i*3]==revmot+3)
			{
				value=tx_packet[8+i*3]+tx_packet[9+i*3]*256;
				tx_packet[adchksum]+=tx_packet[8+i*3]+tx_packet[9+i*3];
				value=valr[3]+offset[tx_packet[7+i*3]]+chr3;
				if(value>=0 && value<=4096)
				{
					tx_packet[8+i*3]=value%256;
					tx_packet[9+i*3]=value/256;
				}
				tx_packet[adchksum]-=tx_packet[8+i*3]+tx_packet[9+i*3];
				//printf("value %d \t %d\n",tx_packet[7+i*3],value);				
			}
			if(tx_packet[7+i*3]==revmot+4)
			{
				value=tx_packet[8+i*3]+tx_packet[9+i*3]*256;
				tx_packet[adchksum]+=tx_packet[8+i*3]+tx_packet[9+i*3];
				//value+=pow(-1,leg)*chr*1023/300;
				value=rotation(revmot+4,rphi)+offset[tx_packet[7+i*3]];
				if(value>=0 && value<=1023)
				{
					tx_packet[8+i*3]=value%256;
					tx_packet[9+i*3]=value/256;
				}
				tx_packet[adchksum]-=tx_packet[8+i*3]+tx_packet[9+i*3];
				//printf("value %d \t %d\n",tx_packet[7+i*3],value);				
			}
			if(tx_packet[7+i*3]==revmot+5)
			{
				value=tx_packet[8+i*3]+tx_packet[9+i*3]*256;
				tx_packet[adchksum]+=tx_packet[8+i*3]+tx_packet[9+i*3];
				value=valr[4]+offset[tx_packet[7+i*3]];
				if(value>=0 && value<=4096)
				{
					tx_packet[8+i*3]=value%256;
					tx_packet[9+i*3]=value/256;
				}
				tx_packet[adchksum]-=tx_packet[8+i*3]+tx_packet[9+i*3];
				//printf("value %d \t %d\n",tx_packet[7+i*3],value);				
			}
			
		}
		//gettimeofday(&tv,NULL); /*gets the current system time into variable tv */ 
		//time_start  = (tv.tv_sec *1e+6) + tv.tv_usec;
		
		//if(ftdi_write_data(&ftdic1,tx_packet,tx_packet[3]+4)<0)
		//printf("ERROR SENDING PACKET\n");
		//if(write(fd,tx_packet,tx_packet[3]+4)<0)
		if(iwrite(MOTOR_DATA,&fd,&ftdic1,tx_packet,tx_packet[3]+4)<0)
		printf("ERROR SENDING PACKET\n");
		
		//gettimeofday (&tv, NULL);
		//time_end = (tv.tv_sec *1e+6) + tv.tv_usec;
		//usleep(16670);
		//printf("********************TIME TO SEND %ld \t ********************\n",time_end-time_start);
		
		for(i=0;i<128;i++)
		{	
			last_tx_packet[i]=tx_packet[i];
			//printf("%d\t",tx_packet[i]);
		}		
		
	}
	printf("********************************************************\n");
	//static_check(leg);
	set_offset();
	Y_END=y;
	YR_END=yr;
	Z_END=z;
	ZR_END=zr;
	THETA_END=phi;
	THETAR_END=rphi;
	COUNTER+=STEP*2;
	return EXIT_SUCCESS;
}		

int init[]={512,698,500,150,823,512,325,512,865,200};
int final[]={523,730,532,493,892,501,293,491,530,131};
int mot[]={6,7,8,9,10,26,27,28,29,30};

	
int pose_change(int time)
{
	int val=0;
	float t=0;
	int adchksum=7+NOM*3;
	
	int g=0;
		
	for(t=0;t<time;t+=((float)1/60))
	{
		for(g=0;g<128;g++)
		tx_packet[g]=last_tx_packet[g];
		
		int pose_c=0;
		for(pose_c=0;pose_c<10;pose_c++)
		{
			int j=0;
			for(j=0;j<28;j++)
			{
				if((int)tx_packet[7+j*3]-mot[pose_c]==0)
				{
					tx_packet[adchksum]+=tx_packet[8+j*3]+tx_packet[9+j*3];
					val=scurve(init[pose_c],final[pose_c],t,time)+offset[mot[pose_c]];
					tx_packet[8+j*3]=val%256;
					tx_packet[9+j*3]=val/256;
					tx_packet[adchksum]-=(tx_packet[8+j*3]+tx_packet[9+j*3]);
					printf("ID\t%d\tVAL\t%d\t%d\t%d\n",mot[pose_c],val,pose_c,j);
				}
			}
		}
		for(g=0;g<128;g++)
		{
			last_tx_packet[g]=tx_packet[g];
			printf("%d\t",tx_packet[g]);
		}
		printf("\n");
		
		//ftdi_write_data(&ftdic1,tx_packet,tx_packet[3]+4);
		iwrite(MOTOR_DATA,&fd,&ftdic1,tx_packet,tx_packet[3]+4);
		usleep(16670);
	}
	
}
void kick()
{	
	
	act_fpacket("acyut4/moves/kick2");
}
	
void imu_check()
{
	while(1)
	imucal();
}



int pingtest()
{

	//printf("pinging....\n");
	int failure[NOM][2];
	byte txp[15]={0xff,0Xff,0,0X02,0X01,0},rxp[4],chkff,count=2,exit=0,chksum,noat2=0;
	int er=0,ping_test_k=0,i,j,z,igm=0;
	ftdi_usb_purge_buffers(&ftdic1);
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
		for(noat=0;noat<5;noat++)
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
	tx_packet[0]=0xff;
	int adchksum=7+NOM*3;
	tx_packet[1]=0xff;
	tx_packet[2]=0xfe;
	tx_packet[3]=NOM*3+4;
	tx_packet[4]=INST_SYNC_WRITE;
	tx_packet[5]=0x1e;
	tx_packet[adchksum]=0xff;
	tx_packet[6]=0x02;
	int i_init,h=0;
	for(i_init=0;i_init<NOM;i_init++)
	{
		tx_packet[7+i_init*3]=bid[i_init];
	}
	float val[5];
	float *fval;
	int value;
	fval=generateik(390,0,0,0);
	for(h=0;h<5;h++)
	val[h]=fval[h];

	for(i_init=0;i_init<NOM;i_init++)
	{
		if(tx_packet[7+i_init*3]==0||tx_packet[7+i_init*3]==20)
		{
			value=val[0]+offset[tx_packet[7+i_init*3]];
			tx_packet[8+i_init*3]=value%256;
			tx_packet[9+i_init*3]=value/256;
			//printf("value %d \t %d\n",tx_packet[7+i_init*3],value);
		}
		if(tx_packet[7+i_init*3]==1||tx_packet[7+i_init*3]==21)
		{
			value=val[1]+offset[tx_packet[7+i_init*3]];
			tx_packet[8+i_init*3]=value%256;
			tx_packet[9+i_init*3]=value/256;
			//printf("value %d \t %d\n",tx_packet[7+i_init*3],value);
		}
		if(tx_packet[7+i_init*3]==2||tx_packet[7+i_init*3]==12||tx_packet[7+i_init*3]==22||tx_packet[7+i_init*3]==32)
		{
			value=val[2]+offset[tx_packet[7+i_init*3]];
			tx_packet[8+i_init*3]=value%256;
			tx_packet[9+i_init*3]=value/256;
			//printf("value %d \t %d\n",tx_packet[7+i_init*3],value);
		}
		if(tx_packet[7+i_init*3]==3||tx_packet[7+i_init*3]==23)
		{
			value=val[3]+offset[tx_packet[7+i_init*3]];
			tx_packet[8+i_init*3]=value%256;
			tx_packet[9+i_init*3]=value/256;
			//printf("value %d \t %d\n",tx_packet[7+i_init*3],value);
		}
		if(tx_packet[7+i_init*3]==5||tx_packet[7+i_init*3]==25)
		{
			value=val[4]+offset[tx_packet[7+i_init*3]];
			tx_packet[8+i_init*3]=value%256;
			tx_packet[9+i_init*3]=value/256;
			//printf("value %d \t %d\n",tx_packet[7+i_init*3],value);
		}
		if(tx_packet[7+i_init*3]==4||tx_packet[7+i_init*3]==24)
		{
			value=rotation(tx_packet[7+i_init*3],0)+offset[tx_packet[7+i_init*3]];
			tx_packet[8+i_init*3]=value%256;
			tx_packet[9+i_init*3]=value/256;
			//printf("value %d \t %d\n",tx_packet[7+i_init*3],value);
		}
		if(tx_packet[7+i_init*3]==6||tx_packet[7+i_init*3]==26)
		{
			value=512+offset[tx_packet[7+i_init*3]];
			tx_packet[8+i_init*3]=value%256;
			tx_packet[9+i_init*3]=value/256;
			//printf("value %d \t %d\n",tx_packet[7+i_init*3],value);
		}
		if(tx_packet[7+i_init*3]==7)
		{
			value=698+offset[tx_packet[7+i_init*3]];
			tx_packet[8+i_init*3]=value%256;
			tx_packet[9+i_init*3]=value/256;
			//printf("value %d \t %d\n",tx_packet[7+i_init*3],value);
		}
		if(tx_packet[7+i_init*3]==27)
		{
			value=325+offset[tx_packet[7+i_init*3]];
			tx_packet[8+i_init*3]=value%256;
			tx_packet[9+i_init*3]=value/256;
			//printf("value %d \t %d\n",tx_packet[7+i_init*3],value);
		}
		if(tx_packet[7+i_init*3]==8)
		{
			value=500+offset[tx_packet[7+i_init*3]];
			tx_packet[8+i_init*3]=value%256;
			tx_packet[9+i_init*3]=value/256;
			//printf("value %d \t %d\n",tx_packet[7+i_init*3],value);
		}
		if(tx_packet[7+i_init*3]==28)
		{
			value=512+offset[tx_packet[7+i_init*3]];
			tx_packet[8+i_init*3]=value%256;
			tx_packet[9+i_init*3]=value/256;
			//printf("value %d \t %d\n",tx_packet[7+i_init*3],value);
		}
		if(tx_packet[7+i_init*3]==9)
		{
			value=150+offset[tx_packet[7+i_init*3]];
			tx_packet[8+i_init*3]=value%256;
			tx_packet[9+i_init*3]=value/256;
			//printf("value %d \t %d\n",tx_packet[7+i_init*3],value);
		}
		if(tx_packet[7+i_init*3]==29)
		{
			value=865+offset[tx_packet[7+i_init*3]];
			tx_packet[8+i_init*3]=value%256;
			tx_packet[9+i_init*3]=value/256;
			//printf("value %d \t %d\n",tx_packet[7+i_init*3],value);
		}
		if(tx_packet[7+i_init*3]==10)
		{
			value=823+offset[tx_packet[7+i_init*3]];
			tx_packet[8+i_init*3]=value%256;
			tx_packet[9+i_init*3]=value/256;
			//printf("value %d \t %d\n",tx_packet[7+i_init*3],value);
		}
		if(tx_packet[7+i_init*3]==30)
		{
			value=200+offset[tx_packet[7+i_init*3]];
			tx_packet[8+i_init*3]=value%256;
			tx_packet[9+i_init*3]=value/256;
			//printf("value %d \t %d\n",tx_packet[7+i_init*3],value);
		}
		if(tx_packet[7+i_init*3]==15)
		{
			value=1863+offset[tx_packet[7+i_init*3]];
			tx_packet[8+i_init*3]=value%256;
			tx_packet[9+i_init*3]=value/256;
			//printf("value %d \t %d\n",tx_packet[7+i_init*3],value);
		}
		if(tx_packet[7+i_init*3]==16)
		{
			value=2048+offset[tx_packet[7+i_init*3]];
			tx_packet[8+i_init*3]=value%256;
			tx_packet[9+i_init*3]=value/256;
			//printf("value %d \t %d\n",tx_packet[7+i_init*3],value);
		}
		if(tx_packet[7+i_init*3]==17)
		{
			value=546+offset[tx_packet[7+i_init*3]];
			tx_packet[8+i_init*3]=value%256;
			tx_packet[9+i_init*3]=value/256;
			//printf("value %d \t %d\n",tx_packet[7+i_init*3],value);
		}
		if(tx_packet[7+i_init*3]==18)
		{
			value=588+offset[tx_packet[7+i_init*3]];
			tx_packet[8+i_init*3]=value%256;
			tx_packet[9+i_init*3]=value/256;
			//printf("value %d \t %d\n",tx_packet[7+i_init*3],value);
		}
		
		
	}
	//for(i_init=0;i_init<=adchksum;i_init++)
	//printf("%d\t",tx_packet[i_init]);
	for(i_init=2;i_init<adchksum;i_init++)
	{
		tx_packet[adchksum]-=tx_packet[i_init];
	}
	speed(0);	
	//if(ftdi_write_data(&ftdic1,tx_packet,tx_packet[3]+4)>0)
	if(iwrite(MOTOR_DATA,&fd,&ftdic1,tx_packet,tx_packet[3]+4)>0)
	printf("INTIALIZED\n");
	sleep(5);
	speed(1);	
	for(i_init=0;i_init<128;i_init++)
	{
		last_tx_packet[i_init]=tx_packet[i_init];
	}
	/*
	speed(0);
	if(acyut==4)
	act_fpacket("acyut4/moves/init4");
	else if (acyut==3)
	act_fpacket("acyut3/moves/init3");
	sleep(5);
	speed(1);
	printf("AcYut Initialized\n");
	*/
	
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
	
	FILE *f;
	f=fopen("Graphs/COUNT","r");
	if(f!=NULL)
	{
		fscanf(f,"%d",&COUNT);
		printf("COUNT : %d\n",COUNT);		
		fclose(f);
	}
	char STR[4];
	//itoa(COUNT,STR,10);
	sprintf(STR,"%d",COUNT);
	COUNT++;
	strcat(comfile,"Graphs/COM");
	strcat(comfile,STR);
	strcat(leftfile,"Graphs/LEFT");
	strcat(leftfile,STR);
	strcat(rightfile,"Graphs/RIGHT");
	strcat(rightfile,STR);
	strcat(expect_com,"Graphs/Expected");
	strcat(expect_com,STR);
	strcat(hipfile,"Graphs/Hip");
	strcat(hipfile,STR);
	printf("%s\t%s\t%s\t%s\n",comfile,leftfile,rightfile,hipfile);
	
	f=fopen(comfile,"w");
	if(f!=NULL)
	{
		fprintf(f,"#X\tY\tZ\n");
		fclose(f);
	}
	f=fopen(leftfile,"w");
	if(f!=NULL)
	{
		fprintf(f,"#X\tY\tZ\n");
		fclose(f);
	}
	f=fopen(rightfile,"w");
	if(f!=NULL)
	{
		fprintf(f,"#X\tY\tZ\n");
		fclose(f);
	}
	f=fopen(expect_com,"w");
	if(f!=NULL)
	{
		fprintf(f,"#X\tY\tZ\n");
		fclose(f);
	}
	f=fopen(hipfile,"w");
	if(f!=NULL)
	{
		fprintf(f,"#X\tY\tZ\n");
		fclose(f);
	}
	f=fopen("Graphs/COUNT","w");
	if(f!=NULL)
	{
		fprintf(f,"%d",COUNT);
		fclose(f);
	}
	byte read;
	//torque(0);
	//pingtest();
	
	int flag=1;
	COUNTER=0;
	//ftdi_set_latency_timer(&imu,1);
	//ftdi_read_data_set_chunksize(&imu,4096);
	initialize_acyut();
	imu_initialize();
	walk3(1-flag,0,STEP,0,-STEP,0,0,0,0,0,0,0,0);
	usleep(16670);
	while(!kbhit2())
	{
		//printf("%d\n",read_position(5));
		walk3(flag,-STEP,STEP,STEP,-STEP,0,0,0,0,0,0,0,0);
		flag=1-flag;
		//projection_system(flag);
		usleep(16670);
	}
	walk3(flag,-STEP,0,STEP,0,0,0,0,0,0,0,0,0);
	compliance(0,32,32);
	usleep(16670);
	f=fopen("WALK","w");
	if(f!=NULL)
	{
		fwrite(&zmp,sizeof(zmp),1,f);
		fclose(f);
	}
	close_files();
	f=fopen("script","w");
	if(f!=NULL)
	{	
		fprintf(f,"splot \"%s\" using 2:3:1 with lines, \"%s\" using 2:3:1 with lines, \"%s\" using 2:3:1  with lines, \"%s\" using 2:3:1 with lines, \"%s\" using 2:3:1 with lines\n",comfile,leftfile,rightfile,expect_com,hipfile);
		//fprintf(f,"splot \"%s\" using 2:3:1 with lines, \"%s\" using 2:3:1 with lines\n",comfile,expect_com);
		fclose(f);
		system("gnuplot < \"script\" -persist");
	}
	f=fopen("script2","w");
	if(f!=NULL)
	{
		fprintf(f,"plot \"%s\" using 2 with lines, \"%s\" using 2 with lines\n",comfile,expect_com);
		fclose(f);
		system("gnuplot < \"script2\" -persist");
	}
			
//	sleep(2);
//	projection_system(1);
//	forward_kinematics(1,cnst);
//	generateik(390,0,0,0);
//	int i=0;
//	for(i=0;i<5;i++)
//	printf("%f\n",mval[i]);	

}
