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
#include "xsens/imu.h"

#define IMU_XSENS
#define MASS (5.733)
#define PLOT 0

#define MOTOR_DATA 0
#define IMU_DATA 0
#define FT4232 "FTSBY5CA"//"FTSC152E"//"FTSC152E"
#define NOM 28                                  //Current number of Motors
#define LEN_SYNC_WRITE ((NOM*3)+4) 		//Total Lenght Of Packet
#define INST_SYNC_WRITE     0x83
#define SMOOTHSTEP(x) ((x) * (x)*(3-2*(x)))
#define max( a, b ) ( ((a) > (b)) ? (a) : (b) )
#define min( a, b ) ( ((a) < (b)) ? (a) : (b) )
#define sgn( a ) ( ((a) > 0) ? (1) : ( ((a) < 0) ? -1 : 0 ) )
#define D2R (3.14/180)
#define acyut 4  //acyut using...........................................change here..............
#define E 2.7183
#define MAX_BOUNDARIES 160
#define TRUE 1
#define serialbluetooth "A8008c3o"    //"A6007WeE"  
#define serialusb2d "A7003N1d"  //"A700eSSZ" //"A900fDpz"//"A900fDhp" //"A600cBa7" //"A600cURy"  //"A900fC5U"//nd USB2D: A7003N1d  A7003NDp	 A7005LC8 //A700eST2
#define serialimu "A8004Yt8" //"A8008c3o"
typedef uint8_t byte; 

Imu imu;
byte ignore_ids[]={10,15,16,17,18,30};  // enter ids to ignore :)
float mval[5];

int fd;
struct ftdi_context ftdic1,ftdic2;
int offset[33]={0};
int motor_val[33]={0};
int target_left[6],target_right[6];

byte tx_packet[128]={0xff,0xff,0xfe,LEN_SYNC_WRITE,INST_SYNC_WRITE,0x1e,0x02};

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
			r_leg_coods[i]=right_joints[6][i]-left_joints[6][i];
		}
	}
	else if(leg==1)
	{
		for(i=0;i<3;i++)
		{
			com_leg_frame[i]=com[i]-right_joints[6][i];
			hip_coods[i]=-right_joints[6][i];
			r_leg_coods[i]=left_joints[6][i]-right_joints[6][i];
		
		}
	}
	
	//printf(" %f %f %f\n",r_leg_coods[0],r_leg_coods[1],r_leg_coods[2]);
	hip[0]=hip_coods[0];
	hip[1]=hip_coods[1]+(-50)*(1-leg)+(50)*leg;
	hip[2]=hip_coods[2];
	//printf("\n\n%f %f %f\n",hip[0],hip[1],hip[2]);
	

	//// GETTING ANGLES FROM IMU
	get_imu_data();
	roll=current_angle[0]*3.14/180;
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
	rot_com[1]=-rot_x+(-50)*(1-leg)+50*leg;
	rot_com[2]= rot_y;

	float hip_x=-hip_coods[1];
	float hip_y=hip_coods[2];
	float hip_z=-hip_coods[0];
	
	float hip_rot_x=cos(pitch)*hip_x+sin(pitch)*hip_z;
	float hip_rot_y=sin(roll)*sin(pitch)*hip_x+cos(roll)*hip_y+sin(roll)*cos(pitch)*hip_z;
	float hip_rot_z=-cos(roll)*sin(pitch)*hip_x+sin(roll)*hip_y+cos(roll)*cos(pitch)*hip_z;
	
	//printf("%f %f %f\n",hip_rot_x,hip_rot_y,hip_rot_z);
	rot_hip[0]=-hip_rot_z;
	rot_hip[1]=-hip_rot_x+(-50)*(1-leg)+50*leg;
	rot_hip[2]=hip_rot_y;


	float rleg_x=-r_leg_coods[1];
	float rleg_y=r_leg_coods[2];
	float rleg_z=-r_leg_coods[0];

	//printf("LEG %f %f %f\n",rleg_x,rleg_y,rleg_z);
	
	float rot_rleg_x=cos(pitch)*rleg_x+sin(pitch)*rleg_z;
	float rot_rleg_y=sin(roll)*sin(pitch)*rleg_x+cos(roll)*rleg_y+sin(roll)*cos(pitch)*rleg_z;
	float rot_rleg_z=-cos(roll)*sin(pitch)*rleg_x+sin(roll)*rleg_y+cos(roll)*cos(pitch)*rleg_z;

	rot_rleg[0]=-rot_rleg_z;
	rot_rleg[1]=-rot_rleg_x+(-50)*(1-leg)+50*leg;
	rot_rleg[2]=rot_rleg_y;
	
	//printf("RLEG %f %f %f\n",rot_rleg[0],rot_rleg[1],rot_rleg[2]);
	if(rot_rleg[0]<=10)
	{
		//printf("!!!!!TOUCHDOWN!!!!!\n");
		touchdown=rot_rleg[0];
	}
	else
	touchdown=10;
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
	#if PLOT==1
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
		fprintf(f,"%lf\t%lf\t%lf\n",-hip_rot_z,-hip_rot_x+(-50)*(1-leg)+(50)*leg,hip_rot_y+COUNTER);
		fclose(f);
	}
	#endif
	return EXIT_SUCCESS;
	
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

#ifndef IMU_XSENS
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
#endif

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
	return mval;
	
	cons[0]=1675;
	cons[1]=2952+255;
	cons[2]=2008+973;
	cons[3]=1459+255;
	cons[4]=1703;
	
	// CALCULATING LEG LENGTH
	float val=(leg_l-l22)/(2*b_len);
	if(val<=1&&val>=-1)
	{
		theta[3]=acos(val); // angle for motor 3 from XZ Acyut plane 
		theta[2]=asin(val);	// angle for motor 2 12 from YZ Acyut plane 
		theta[1]=theta[3];	// angle for motor 1 from XZ Acyut plane 
	}
	else
		return mval;
	// CALCULATING LEG POSITION
	if(y<=x)
		phi[3]=atan(y/x);
	else
		return mval;
	if(z<=x)
		phi[5]=atan(z/x);
	else
		return mval;	
	phi[1]=phi[3];
	phi[0]=phi[5];
	
	// GETTING ANGLES
	//printf("%lf asdjhkdshdk\n",avg_angle[1]);
	ang[0]=phi[0];//+(current_angle[1]*pow(-1,leg))*0.2/r2d;
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
	return mval;

}


void set_offset()
{
	if(acyut==4)
	{
		offset[0]=0+15;
		offset[1]=235-50 + 25-3; 
		offset[2]=325+30; 
		offset[3]=180+20+10-10+7-20;//232-20-25-25-10;    // 142s  
		offset[4]=-10; 
		offset[5]=-15;//+7; 
		offset[6]=346-40; 
		offset[7]=0; 
		offset[8]=0; 
		offset[9]=0;//-(1023-865);//-150; 
		offset[10]=0; 	 
		offset[11]=0; 
		offset[12]=274+30; 
		offset[13]=0; 
		offset[14]=0; 
		offset[15]=-119; 
		offset[16]=42; 
		offset[17]=0; 
		offset[18]=-150; 
		offset[19]=0; 
		offset[20]=24-15;
		offset[21]=224-50 + 25-3; 
		offset[22]=343+30+10; 
		offset[23]=383+10-10+7-20;//-20; 
		offset[24]=-2; 
		offset[25]=-15+7+10-20;//-7; 
		offset[26]=-306; 
		offset[27]= 0; 
		offset[28]=0;// 90*1023/300; 
		offset[29]=0;//s1023-865; 
		offset[30]=0;
		offset[31]=0;
		offset[32]=273+30;
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


void initialize_acyut()
{
	int h=0;
	float val[5];
	float *fval;
	int value;

	//fval=generateik(390,30,6,0);
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
	
	//printf("%lf \n",val[4]);
	
	motor_val[15]=1863+offset[15];
	motor_val[16]=2048+offset[16];
	motor_val[17]=546+offset[17];
	motor_val[18]=588+offset[18];
	
	//fval=generateik(390,-30,6,0);
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
	motor_val[32]=val[2]+offset[32];
	
	speed(0);
	if(generatePacket()==1 && iwrite(MOTOR_DATA,&fd,&ftdic1,tx_packet,tx_packet[3]+4)>0)
	printf("INITIALIZED\n");
	sleep(5);
	speed(1);
	
	
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
	
#ifndef IMU_XSENS
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
#endif
	if(ret_motor>=0&&ret_imu>=0)
	return EXIT_SUCCESS;	
	else
	return EXIT_FAILURE;

}

int main()
{
	bootup_files();
	imu.init();
	initialize_acyut();
