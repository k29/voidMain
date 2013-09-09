/*
 *Changelog:
 * Included Pie Slice YUV
//Change from v8	- removed main and opencv functions to allow inclusion in ballTrack programs
//			- adding prelocalize
//			- write_motor now uses sync_write_gp function
//			- stop_motor function added
//			- ismoving_motor function added
//			- changed red thresholding in marker
//			- removed cvRelease of infoimg from localize()
//			- changed findReal - it was using thetaX while thetaY should have been used. it now returns in degrees
//			- changed offset for baseAngle : dec 17
//			- changed ismoving_motor and read_pos functions : there was stack smashing because memory beyond packet[7] was being used
//			- changed marker function - variables changed to #define, also added hue and saturation (refer linetry9.cpp for original marker)
//			- changed camera_input structure implementation jan 09
//			- changed update_gps
//			- added blue goal probability
//			- added doCopyArray int implementation
//			- removed doGetBallAbsolute
//			- removed a lot of commented code
//			- program won't crash if acyut-apogee.jpg is not present
//			- fixed relativeField bug (wrong variables were being used)
//			- new marker implementation using lookuptable added
//			- infoimg made static, not being reallocated everytime

//TO DO: 
//~Change morphology implementation in marker function - no morphology is performed there now
//~Check YBY and BYB to sensitivity to thresholding. Maybe make a new image for them from infoimg in function definition? - new image implemented
//!White Filter NOT WORKING properly - will have to change to histogram algorithm
//~Updategps problem - need to do morphology- Done
//!Make global morphKernels as they are not being released anywhere?
//!Error flagging
//!Data sending - here or in ballt prog?
//~Check BaseAngle calculations, offset is required - done
//!Change yellow thresholding
//!Check blue goal probability - hasn't been checked before
//!Merge categories of global variables for showFieldImage and for sending data
//!Why is cvConvertScale there in conversion of image from bayertobgr?

Test for projective transformation included.
IMU Angles included
*/
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <math.h>
#include <ftdi.h>
#include <sys/time.h>
#include <termios.h> /* POSIX terminal control definitions */
#include <limits.h>

//#include <iomanip>

#ifdef WIN32
#include <cv.h>
#include <highgui.h>
#else
// #include <opencv/cv.h>
// #include <opencv/highgui.h>
// #include <opencv2/highgui/highgui.hpp>
// #include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#endif

//STRUCTURES FOR SENDING DATA BEGIN
#define SEND_BALL_SEARCH 1
#define SEND_LOCALIZE 2
typedef struct
{
	float r;
	float theta;
	int x;
	int y;
}ball_coords;


typedef struct
{
	float r;
	float theta;
	int x;
	int y;
	float wr;
}obs_coords;

typedef struct 
{
	float x;
	float y;
	float theta;
}coordinates;

typedef struct 
{
float r;
float theta;
int x;
int y;
int type;
}landmark;

///flags//
typedef struct
{
int current_action;
int ball_found;
float localized;
// int goal_blue;
// int goal_yellow;
}flags;

typedef struct
{
	ball_coords b;
	coordinates c;
	coordinates oc;
	obs_coords o[20];
	int no_of_obstacles;
/*	goal_coords gblue;
	goal_coords gyellow;*/
	landmark l[15];
	int n_landmarks;
	flags f;
	int count;
}camera_input;

camera_input cam_in;


// typedef struct 
// {
// 	int count;
// 	byte find_ball;
// 	byte localise;
// 	byte walking;
// 	byte isFallen;
// }instruct;

// instruct Command;
//STRUCTURES FOR SENDING DATA END





//Color Coding!
#define NOCOLS 7 //Plz update this!
#define YELLOWC 4
#define GREENC 2
#define BLUEC 1
#define ORANGEC 8
#define BLACKC 16
#define REDC 32
#define WHITEC 64


#define get_lb(x)	x&0xff
#define	get_hb(x)	x>>8
#define deg2rad(x) (x)*(3.14159265/180.0)
#define rad2deg(x) (x)*(180.0/3.14159265)
#define pix2cm 0.14
#define serialusb2d_camera "A700eSSZ"//"A4007sgG"//"A900fDpz"//"A900fDhp"// "A900fDpA"//"A900fDpA"//

#define MOTOR_SPEED 120

#define GRIDSIZE 6
#define percentWhite 0.1
#define fieldConversion 0.0629629//0.064814
#define XGridNum 54
#define YGridNum XGridNum*2/3
#define weightT 5
#define weightX 1
#define weightC 10
#define weightGP 5
#define weightLP 5
int baseAngle;	//Global Declaration for baseAngle required, as it is used as is in many functions
//typedef unsigned char byte;
#define safe(x) ((x)>0?(x):0)

#define LAND_X 0
#define LAND_T 1
#define LAND_GPY 2
#define LAND_GPB 3
#define LAND_YBY 4
#define LAND_BYB 5

#define ASC 1
#define DESC 2

///////////IMU VARIABLES//////////////////////////
#define FT4232 "FTSBY5CA"
#define MOTOR_DATA 0
#define IMU_DATA 0
#define MAX_BOUNDARIES 160
#define serialimu "A4007rXR"//"A8004Yt8"//"A600ak9F"
#define ERROR_IMU -1000
#define ERROR_MOTORS -900
#define ERROR_GET_IMAGE -800

float avg_angle[3]={0};
int imu_init=0;
float base_angle[3]={0};
float thetaX=0,thetaY=0;
int boundaries_size=0, local_boundaries_size=0;
float boundaries[MAX_BOUNDARIES][3]={0};
float local_boundaries[MAX_BOUNDARIES][3]={0};

int fd_camera,fb_camera,imu_t;
long int COUNTER_CAMERA=0;
struct ftdi_context imu;

/////////////////////////////////////////////////



//Global Variables used by localize BEGIN

//Declarations and constants
float s_height=67.0;
float neck_len=7.0,constanty=4.5,rangey=10.0,constantx=8.0,rangex=16.0,rminx=70.0;
int mltplr_x=1;
int i=0;

uchar* lut_marker;
CvCapture *capture;
IplImage *camera_stream;
CvSize image_size;
IplImage *color_stream;
IplImage *frame;
IplImage* yuv;
IplImage *segmentated =NULL; 
IplImage *infoimg;
int tempAngle = 0, initAngle;

IplConvKernel* morphKernel;

//Global Variables used by localize END

//Global Variables used by search_ball BEGINS
#define BALL_FOUND 1
#define BALL_NOTFOUND 0
#define BALL_CANTMOVE -1
#define BALL_READERROR -2

float ballDistance;
float ballAngleDeg;
float ballDistanceArray[20];
float ballAngleDegArray[20];
float ballThetaXPrev, ballThetaYPrev;
float ballIntThetaXPrev[2], ballIntThetaYPrev[2];
int ballX = 0, ballY = 0;
int ballXPrev = 0, ballYPrev = 0;
int ballIntXPrev[2] = {0,0}, ballIntYPrev[2] = {0,0};
int ballIntN = 0;
int ballFoundPrev = 0;
float ballStateOfMotion = 0;
//Global Variables used by search_ball END

//Following 2 sets of global variables should come under same category

//Global variables used by doShowFieldImage BEGINS

int drawBallX, drawBallY;
int drawAcyutX, drawAcyutY;
double drawProbHigh;

//Global variables used by doShowFieldImage ENDS


byte tx_packet_camera[128]={0xff,0xff,0xfe,0x0a,0x83,0x1e,0x02,17,0,0,18};

int offsetx=210,offsety=95,stop_search=0,max_noat=5,disable_noat=0;
float current_thetax=150, current_thetay=45;
struct ftdi_context ft1;


#define FPS_MOTOR 120

#define FPS 120
#define RED 2
#define GREEN 1
#define BLUE 0
//Dangerous macro:
#define returnPixel1C(image, x, y) ((uchar*)(image->imageData + image->widthStep*y))[x]
#define returnPixel3C(image, x, y, color) ((uchar*)(image->imageData + image->widthStep*y))[x*3 + color]
using namespace std;
//Inline functions may be made macros
//Changes from linetry7 - 
//doWhiteFilter changed - definition plus 1 parameter reduced
//added function doReduceImageS. old function should be removed

struct info{
	int Tcoord[6][2];
	
	float Tvar[6];
	int Xcoord[2][2];
	
	float Xvar[2];
	int GPcoord[4][2];
	
	float GPvar[4];
	int LPcoord[2][2];
	
	float LPvar[2];
	
	int Ccoord[1][2];
	float Cvar[1];
	float AngleXvar[2];
	float AngleTvar[6];
	float AngleGPvar[4];
	float AngleLPvar[2];
	
};
info landmarks;

float position[3][(int)XGridNum+1][(int)YGridNum+1];

int getcol(IplImage* src,int vert, int hori,int col,int dim =3)
{
	if(dim==1) col =0;
	return (int)((uchar*)(src->imageData + vert * src->widthStep))[hori*dim+col];
}

void putcol(IplImage* src,int vert, int hori, int data, int col,int dim =3)
{
	if(dim==1) col =0;
	((uchar*)(src->imageData + vert * src->widthStep))[hori*dim+col]=data;
}


inline int doCheck3C(int a, int b, int c, int mina, int maxa, int minb, int maxb, int minc, int maxc)
{
	if((a<=maxa)&&(a>=mina)&&(b<=maxb)&&(b>=minb)&&(c<=maxc)&&(c>=minc))
	{
		return 1; 
		
	}
	else
		return 0;
}

inline void setBlank1C(IplImage* image)
{
	int x, y;
	for (x = 0; x< image->width; x++)
	{
		for(y = 0; y< image->height; y++)
		{
			returnPixel1C(image, x, y) = 0;
		}
	}
}

int doCopyArray(int *source, int* dest, int size = 2)
{
	for(int i=0; i<size; i++)
	{
		dest[i] = source[i];
	}
	return 1;
}

int doCopyArray(float* source, float *dest, int size=2)
{
	for(int i=0; i<size; i++)
	{
		dest[i] = source[i];
	}
	return 1;
}

//IMU FUNCTIONS BEGIN//


int bootup_filesIMU()
{
	int ret_motor,ret_imu;
	/*for (chk_i_ids = 0; chk_i_ids < 24; chk_i_ids += 1)
	{
		printf("bid[%d]= %d\n",chk_i_ids,bid[chk_i_ids]);
	}*/
	
	
	if(IMU_DATA==0)
	{
		if (ftdi_init(&imu) < 0)
    		{
	        	fprintf(stderr, "ftdi_init failed\n");
        		ret_imu=-99;
        		//return EXIT_FAILURE;
    		}	
    	
   		//ftdi_set_interface(&imu,3);
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
    	else
	printf("UNKNOWN IMU CONNECTION\n");
	
	if(ret_imu>=0)
	return EXIT_SUCCESS;	
	else
	return EXIT_FAILURE;

}



int iread_camera(int choose, int *termios_pointer, struct ftdi_context *ftdi_pointer, byte *packet, int length)
{
	switch(choose)
	{
		case 0: return ftdi_read_data(ftdi_pointer,packet,length);
			break;
		case 1 :return read(*termios_pointer,packet,length);
			break;
		default : return ERROR_IMU;
	}
}


void imucal_init()
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
			if(iread_camera(IMU_DATA,&imu_t,&imu,&im[0],1)>0)
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
					if(iread_camera(IMU_DATA,&imu_t,&imu,&im[i],8)>0 && im[6]==254 && im[7]==254)
					{	
						flag++;
						flag2=1;
						for(j=0,k=0;j<6;j+=2,k+=1)
						{
							ang[k]=im[j]+im[j+1]*256;
						//	ang[k]=(ang[k] - 0) * (180 + 180) / (720 - 0) + -180;
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
	
	/*for(k=0;k<3;k++)		
	{
		printf("BASE ANGLE : %d is %lf\n",k,base_angle[k]);
	}*/
	//sleep(1);
}


int ipurge_camera(int choose,int *termios_pointer, struct ftdi_context *ftdi_pointer)
{
	switch(choose)
	{
		case 0 : return ftdi_usb_purge_buffers(ftdi_pointer);
			break;
		case 1 : return tcflush(*termios_pointer,TCIOFLUSH);
			break;
		default : return ERROR_IMU;
	}
}

inline int imucalBase()
{
	byte im[10],i,j,k=0,a=3,fallen=0;
	float ang[3]={0};
	byte flag=0,imu_no_of_times=0;
	avg_angle[0]=0;
	avg_angle[1]=0;
	avg_angle[2]=0;
	int start=0,count=0;
	int n = 0;
	ipurge_camera(MOTOR_DATA,&imu_t,&imu);
	//printf("BASE_ANG 0: %f BASE_ANG 1: %f BASE_ANG 2: %f\n",base_angle[0],base_angle[1],base_angle[2]);	
	//if(imu_init!=1)
	//printf("INITIALIZE THE IMU\n");
	//tcflush(imu,TCIFLUSH);
	//tcflow(imu,TCIOFLUSH);
	while((imu_no_of_times<1)&& (imu_init==1))
	{		
		flag=0;
		while(flag==0)
		{   
			//if(ftdi_read_data(&imu,&im[0],1)>0)
			n++;
			if(iread_camera(IMU_DATA,&imu_t,&imu,&im[0],1)>0)
			{
			
				if(im[0]==0xff)
				start++;
				else
				start=0;
				
				if(start==2)
				{
					start=0;
				//	printf("PACKET RECIEVED\n");
					i=0;
					//if(ftdi_read_data(&imu,&im[i],8)>0 && im[6]==254 && im[7]==254)
					if(iread_camera(IMU_DATA,&imu_t,&imu,&im[0],8)>0 && im[6]==254 && im[7]==254)
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
						//	ang[k]=(ang[k] - 0) * (180 + 180) / (720 - 0) + -180 -base_angle[k];
						//	avg_angle[k]=avg_angle[k]+ang[k];
							avg_angle[k]  = ((float)ang[k])/2.0;
						}
						
					}	
				}
			}
		}
		imu_no_of_times++;
	}
	
	if(count!=0)
	{	
		return avg_angle[2];
		//printf("%ld\t",COUNTER++);
	//	for(k=0;k<3;k++)		
	//	{
	//		avg_angle[k]=avg_angle[k]/count;
	//		printf("ANGLE %d : %lf     ",k,avg_angle[k]);
	//		printf("Loops taken: %d  ", n);
	//	}
	//	printf("\n");
	}
	else
	{
	//	printf("IMU READING FAILED\n");
		return ERROR_IMU;
		//usleep(16670*0.6);
	}
	ipurge_camera(MOTOR_DATA,&imu_t,&imu);

	//ftdi_usb_purge_buffers(&imu);
	//tcflush(imu,TCIFLUSH);
	//tcflow(imu,TCIOFLUSH);
}



//IMU FUNCTIONS END//


inline uchar pixelColor3C(IplImage* image, int x, int y, int color)
{
	return ((uchar*)(image->imageData + image->widthStep*y))[x*3 + color];
}

inline void pixelColor3C(IplImage* image, int x, int y, int *rgb)
{
	rgb[0] = ((uchar*)(image->imageData + image->widthStep*y))[x*3 + 2];
	rgb[1] = ((uchar*)(image->imageData + image->widthStep*y))[x*3 + 1];
	rgb[2] = ((uchar*)(image->imageData + image->widthStep*y))[x*3 + 0];
	return;
}

inline uchar pixelColor1C(IplImage* image, int x, int y)
{
	return ((uchar*)(image->imageData + image->widthStep*y))[x];
}

//Maybe use left shift operators instead of multiplying??
uchar* marker_create_lut()
{
	uchar* lut = new unsigned char [256*256*256];//[16777216];	//256*256*256

	float r,g,b;
	float Reds, Greens, Blues, Ys, Crs, Cbs;
	int value;


	for(r=0.0; r<256.0; r++)
	for(g=0.0; g<256.0; g++)
	for(b=0.0; b<256.0; b++)
	{
		value = 0;
		Ys = 0.299*r + 0.587*g + 0.114*b;
    	Crs = (r-Ys)*0.713 + 128.0;
    	Cbs = (b-Ys)*0.564 + 128.0;
    	Reds = r/255.0;
    	Greens = g/255.0;
    	Blues = b/255.0;
		//Values hardcoded. Should be loaded from config file
		//Red (means ball color)
		if(doCheck3C(Ys, Crs, Cbs, 0, 203, 153, 255, 66, 116))
		//if(doCheck3C(Ys, Hue, Sat, 46, 255, 194, 255, 128, 164))
			value += REDC;
		//Blue	
		//if(doCheck3C(Ys, Crs, Cbs, 134, 255, 66, 115, 136, 164))
		if(doCheck3C(Ys, Crs, Cbs, 90, 255, 131, 145, 115, 149))
		value+=BLUEC;
		// Yellow 
		
		if(doCheck3C(Ys, Crs, Cbs, 0, 255, 2, 161, 70, 104))
		value+=YELLOWC;
		//  Green
		//RGB
		if((Greens<2.9*Reds)&&(Greens>2.1*Reds)&&(Blues<2.1*Reds)&&(Blues>1.5*Reds))
		value+=GREENC;
		//YUV
//		if(doCheck3C(Ys, Crs, Cbs, 0, 147, 70, 111, 210, 134))
//		value+=GREENC
//		// Orange
//		if((Reds>0.50)&&(Greens>1.45*Blues)&&(Greens<2.5*Blues)&&(Reds>1.4*Greens))
//		value+=ORANGEC;
		// Black
		if((Reds<0.15)&&(Greens<0.15)&&(Blues<0.15)&&(Reds>0.06)&&(Greens>0.06)&&(Blues>0.05))
		value+=BLACKC;

		// White
	//	if(doCheck3C(Ys, Crs, Cbs, 237, 255, 89, 130, 122, 133))
		if(Reds>0.7&&Greens>0.7&&Blues>0.7)
		value+=WHITEC;

		lut[(uchar)r + (((uchar)g)<<8) + (((uchar)b)<<16)] = value;

	}
	return lut;
}

IplImage* marker_lut(IplImage* color, uchar* lut, IplImage* marked = infoimg)
{
	
	for(int i=0; i<color->width; i++)
	for(int j=0; j<color->height; j++)
	{
		returnPixel1C(marked, i, j) = lut[pixelColor3C(color, i, j, RED)
										|(pixelColor3C(color, i, j, GREEN)<<8)
										|(pixelColor3C(color, i, j, BLUE)<<16)];
	}
	return marked;

}

IplImage* marker(IplImage* color, IplImage* marked = infoimg)
{
//	#define DATA(i,j) ((uchar*)(marked->imageData + (j) * marked->widthStep))[(i)]
	//IplImage* marked = cvCreateImage(cvGetSize(color),color->depth,1);
	
	cvCvtColor(color,yuv, CV_BGR2YCrCb);
	
	#define rad2pix(x) x*(256/3.14159265)
	#define Blues ((uchar*)(color->imageData + j * color->widthStep))[i*3]/255.0
	#define Greens ((uchar*)(color->imageData + j * color->widthStep))[i*3+1]/255.0
	#define Reds ((uchar*)(color->imageData + j * color->widthStep))[i*3+2]/255.0
	#define Ys ((uchar*)(yuv->imageData + j * yuv->widthStep))[i*3]
	#define Crs ((uchar*)(yuv->imageData + j * yuv->widthStep))[i*3+1]
	#define Cbs ((uchar*)(yuv->imageData + j * yuv->widthStep))[i*3+2]
	#define Hue rad2pix(atan2(Crs,Cbs))+127
	#define Sat sqrt((Crs*Crs+Cbs*Cbs)/2)
	
	for(int i=0; i<color->width; i++)
	for(int j=0; j<color->height; j++)
	{
		((uchar*)(marked->imageData + j * marked->widthStep))[i]=0;
		//Id Targets
		// Blue
	
		//Values hardcoded. Should be loaded from config file
	
		//Blue	
		//if(doCheck3C(Ys, Crs, Cbs, 134, 255, 66, 115, 136, 164))
		if(doCheck3C(Ys, Crs, Cbs, 90, 255, 131, 145, 115, 149))
		returnPixel1C(marked, i, j)+=BLUEC;
		// Yellow 
		
		if(doCheck3C(Ys, Crs, Cbs, 0, 255, 2, 161, 70, 104))
		returnPixel1C(marked, i, j)+=YELLOWC;
		//  Green
		//RGB
		if((Greens<2.9*Reds)&&(Greens>2.1*Reds)&&(Blues<2.1*Reds)&&(Blues>1.5*Reds))
		returnPixel1C(marked, i, j)+=GREENC;
		//YUV
//		if(doCheck3C(Ys, Crs, Cbs, 0, 147, 70, 111, 210, 134))
//		returnPixel1C(marked, i, j)+=GREENC
//		// Orange
//		if((Reds>0.50)&&(Greens>1.45*Blues)&&(Greens<2.5*Blues)&&(Reds>1.4*Greens))
//		returnPixel1C(marked, i, j)+=ORANGEC;
		// Black
		if((Reds<0.15)&&(Greens<0.15)&&(Blues<0.15)&&(Reds>0.06)&&(Greens>0.06)&&(Blues>0.05))
		returnPixel1C(marked, i, j)+=BLACKC;

		// White
	//	if(doCheck3C(Ys, Crs, Cbs, 237, 255, 89, 130, 122, 133))
		if(Reds>0.7&&Greens>0.7&&Blues>0.7)
		returnPixel1C(marked, i, j)+=WHITEC;
	}
	//	Rudimentary cvMorphologyEx cheaper than original
	
//		IplConvKernel* morphKernel = cvCreateStructuringElementEx(9, 9, 1, 1, CV_SHAPE_RECT, NULL);
//		cvMorphologyEx(marked, clean, NULL, morphKernel, CV_MOP_OPEN, 1);
//		#define cross 5
//		for(int i=0; i<color->width-4; i++)
//		for(int j=0; j<color->height-4; j++)
//		{
//			if( ( DATA(i,j) == DATA(i+cross/2,j+cross/2))&&( DATA(i,j) == DATA(i+cross-1,j+cross-1))&&( DATA(i,j) == DATA(i+cross-1,j) )&&( DATA(i,j) == DATA(i,j+cross-1) )&&DATA(i,j)==YELLOWC )
//			((uchar*)(clean->imageData + j * clean->widthStep))[i] =((uchar*)(marked->imageData + j * marked->widthStep))[i];
//			else ((uchar*)(clean->imageData + j * clean->widthStep))[i] =0;
//		}
//	cvReleaseImage(&marked);
	
	#undef Blues
	#undef Reds
	#undef Greens
	#undef Ys
	#undef Cbs
	#undef Crs
	#undef Hue
	#undef Sat
	#undef rad2pix
	
	return marked; 
}

IplImage* marker_ball(IplImage* color, IplImage* marked = infoimg)
{
//	#define DATA(i,j) ((uchar*)(marked->imageData + (j) * marked->widthStep))[(i)]
	
	cvCvtColor(color,yuv, CV_BGR2YCrCb);
	
	#define rad2pix(x) x*(256/3.14159265)
	#define Blues ((uchar*)(color->imageData + j * color->widthStep))[i*3]/255.0
	#define Greens ((uchar*)(color->imageData + j * color->widthStep))[i*3+1]/255.0
	#define Reds ((uchar*)(color->imageData + j * color->widthStep))[i*3+2]/255.0
	#define Ys ((uchar*)(yuv->imageData + j * yuv->widthStep))[i*3]
	#define Crs ((uchar*)(yuv->imageData + j * yuv->widthStep))[i*3+1]
	#define Cbs ((uchar*)(yuv->imageData + j * yuv->widthStep))[i*3+2]
	#define Hue rad2pix(atan2(Crs,Cbs))+127
	#define Sat sqrt((Crs*Crs+Cbs*Cbs)/2)
	
	for(int i=0; i<color->width; i++)
	for(int j=0; j<color->height; j++)
	{

		if(doCheck3C(Ys, Crs, Cbs, 0, 203, 153, 255, 66, 116))
		//if(doCheck3C(Ys, Hue, Sat, 46, 255, 194, 255, 128, 164))
			returnPixel1C(marked, i, j)=REDC;
		else
			((uchar*)(marked->imageData + j * marked->widthStep))[i]=0;
	
	}
	//	Rudimentary cvMorphologyEx cheaper than original
	
//		IplConvKernel* morphKernel = cvCreateStructuringElementEx(9, 9, 1, 1, CV_SHAPE_RECT, NULL);
//		cvMorphologyEx(marked, clean, NULL, morphKernel, CV_MOP_OPEN, 1);
//		#define cross 5
//		for(int i=0; i<color->width-4; i++)
//		for(int j=0; j<color->height-4; j++)
//		{
//			if( ( DATA(i,j) == DATA(i+cross/2,j+cross/2))&&( DATA(i,j) == DATA(i+cross-1,j+cross-1))&&( DATA(i,j) == DATA(i+cross-1,j) )&&( DATA(i,j) == DATA(i,j+cross-1) )&&DATA(i,j)==YELLOWC )
//			((uchar*)(clean->imageData + j * clean->widthStep))[i] =((uchar*)(marked->imageData + j * marked->widthStep))[i];
//			else ((uchar*)(clean->imageData + j * clean->widthStep))[i] =0;
//		}
//	cvReleaseImage(&marked);

	
	#undef Blues
	#undef Reds
	#undef Greens
	#undef Ys
	#undef Cbs
	#undef Crs
	#undef Hue
	#undef Sat
	#undef rad2pix
	
	return marked; 
}


bool chkcolor(IplImage* dataimg,int hori,int vert,int colorcode)
{
	int rawcode =(int)((uchar*)(dataimg->imageData + vert * dataimg->widthStep))[hori];
	return (bool)((rawcode/colorcode)%2);
}


float findDistance(int x,int y, float distance)
{
	int projY=0,projX=0;
	//printf("Distance sent %f %d %d %f\n",distance,x,y,thetaX);
	float s=1,focal=533.33;	
	float minprojY=(((0)+(focal/s)*tan(thetaX))/(1-(s/focal)*(0)*tan(thetaX)));
	projY=(int)(((240-y)+(focal/s)*tan(thetaX))/(1-(s/focal)*(240-y)*tan(thetaX)));
	projX=(int)(x-320)*((s/focal)*(projY)*sin(thetaX)+cos(thetaX));
		
	distance+=(projY-minprojY)*pix2cm;
	distance=sqrt(distance*distance+(projX)*(projX)*pix2cm*pix2cm);
	return distance;
}

float findAngle(int x, int y, float distance)
{
	int projY=0,projX=0;
	float s=1,focal=533.33;	
	float minprojY=(((0)+(focal/s)*tan(thetaX))/(1-(s/focal)*(0)*tan(thetaX)));
	projY=(int)(((240-y)+(focal/s)*tan(thetaX))/(1-(s/focal)*(240-y)*tan(thetaX)));
	projX=(int)(x-320)*((s/focal)*(projY)*sin(thetaX)+cos(thetaX));
	projY=projY-minprojY;
	projX*=pix2cm;
	projY*=pix2cm;
	float theta=rad2deg(thetaY)-150+rad2deg(atan2(projX,(distance+projY)));
	return theta;
}

void findReal(int x,int y, float &objdis, float &objangdeg)
{
	float s=1,focal=533.33;
	
	objdis=(((240-y)+(focal/s)*tan(thetaX))/(1-(s/focal)*(240-y)*tan(thetaX)));
	float perpend=(x-320)*((s/focal)*(objdis)*sin(thetaX)+cos(thetaX))*pix2cm;
	objdis=pix2cm*(objdis-(focal/s)*tan(thetaX)) + (s_height+(neck_len/sin(thetaX)))*tan(thetaX);
	//printf("%f %f %f %f\n",perpend,objdis,rad2deg(atan2(perpend,objdis)),rad2deg(thetaY));
//	printf("thetaX is %f\t",rad2deg(thetaY));
	objangdeg=rad2deg(thetaY) - 150 + rad2deg(atan2(perpend,objdis));
	objdis=sqrt(objdis*objdis+perpend*perpend);
	
}


float findVarianceDist(float x, float y)
{
	float variance=0,mean=0;
	for(int i=0;i<=XGridNum;i++)
		for(int j=0;j<=YGridNum;j++)
			mean+=sqrt(pow(i-x,2)+pow(j-y,2));
	mean=mean/((XGridNum+1)*(YGridNum+1));
//	printf("Mean %f\n",mean);
	for(int i=0;i<=XGridNum;i++)
		for(int j=0;j<=YGridNum;j++)
			variance+=pow((sqrt(pow(i-x,2)+pow(j-y,2))-mean),2);
	variance=variance/((XGridNum+1)*(YGridNum+1));
	return variance;	
}


//Returns in degrees
float findVarianceAngle(float x,float y)
{
	float variance=0,mean=0;
	for(int i=0;i<=XGridNum;i++)
		for(int j=0;j<=YGridNum;j++)
		//	if(x!=i)
			mean+=(rad2deg(atan2((j-y),(x-i))));
	mean=mean/((XGridNum+1)*(YGridNum+1));
//	printf("Mean Angle%f\n",mean);
	for(int i=0;i<=XGridNum;i++)
		for(int j=0;j<=YGridNum;j++)
		//	if(x!=i)
			variance+=pow(((rad2deg(atan2((j-y),(x-i))))-mean),2);
	variance=variance/((XGridNum+1)*(YGridNum+1));
	return variance;	
}


void probDistance(float distance, int type, int weight = 5)
{
	//return;
	int coord[2][2];
	int var[2];
	int n=1;
	switch(type)
	{
		case LAND_X: 
		coord[0][0] = landmarks.Xcoord[0][0];
		coord[0][1] = landmarks.Xcoord[0][1];
		coord[1][0] = landmarks.Xcoord[1][0];
		coord[1][1] = landmarks.Xcoord[1][1];
		
		var[0] = landmarks.Xvar[0];
		var[1] = landmarks.Xvar[1];
		
		n = 2;
		weight = weightX;
		break;
		
		case LAND_T:
		coord[0][0] = landmarks.Tcoord[0][0];
		coord[0][1] = landmarks.Tcoord[0][1];
		coord[1][0] = landmarks.Tcoord[5][0];
		coord[1][1] = landmarks.Tcoord[5][1];
		
		var[0] = landmarks.Tvar[0];
		var[1] = landmarks.Tvar[5];
		
		n = 2;
		weight = weightT;
		break;
		
		case LAND_GPY:
		coord[0][0] = landmarks.GPcoord[0][0];
		coord[0][1] = landmarks.GPcoord[0][1];
		coord[1][0] = landmarks.GPcoord[1][0];
		coord[1][1] = landmarks.GPcoord[1][1];
		
		var[0] = landmarks.GPvar[0];
		var[1] = landmarks.GPvar[1];
		
		n = 2;
		weight = weightGP;
		break;
		
		case LAND_GPB: 
		coord[0][0] = landmarks.GPcoord[2][0];
		coord[0][1] = landmarks.GPcoord[2][1];
		coord[1][0] = landmarks.GPcoord[3][0];
		coord[1][1] = landmarks.GPcoord[3][1];
		
		var[0] = landmarks.GPvar[2];
		var[1] = landmarks.GPvar[3];
		
		n = 2;
		weight = weightGP;
		break;
		
		case LAND_BYB:
		coord[0][0] = landmarks.LPcoord[1][0];
		coord[0][1] = landmarks.LPcoord[1][1];
		var[0] = landmarks.LPvar[1];
		n = 1;
		weight = weightLP;
		break;
		
		case LAND_YBY:
		coord[0][0] = landmarks.LPcoord[0][0];
		coord[0][1] = landmarks.LPcoord[0][1];
		var[0] = landmarks.LPvar[0];
		n = 1;
		weight = weightLP;
		break;
	}
				
	distance=distance/(100*fieldConversion);
	double p1, p2;
	int i, j;
	if(n==1)
	{
		for(i=0;i<=XGridNum;i++)
		for(j=0;j<=YGridNum;j++)
		{
			position[0][i][j]*= pow(exp(-1*(abs(sqrt(pow(i-coord[0][0],2)+pow(j-coord[0][1],2))-distance))/(2*pow(var[0],2))),weight);
		}
	}
	else if(n==2)
	{	
		for(i=0;i<=XGridNum;i++)
		for(j=0;j<=YGridNum;j++)
		{
		
		
			p1 = pow(exp(-1*(abs(sqrt(pow(i-coord[0][0],2)+pow(j-coord[0][1],2))-distance))/(2*pow(var[0],2))),weight);
			p2 = pow(exp(-1*(abs(sqrt(pow(i-coord[1][0],2)+pow(j-coord[1][1],2))-distance))/(2*pow(var[1],2))),weight);
			if(p1>p2)
				position[0][i][j]*=p1;
			else
				position[0][i][j]*=p2;
		
		}
	}
}


void probAngle(int x,int y,float distance, int type)
{
	return;
	double p1, p2;
	double var[2];
	double coord[2][2];
	int n;
	int weight;
	switch(type)
	{
		case LAND_X: 
		coord[0][0] = landmarks.Xcoord[0][0];
		coord[0][1] = landmarks.Xcoord[0][1];
		coord[1][0] = landmarks.Xcoord[1][0];
		coord[1][1] = landmarks.Xcoord[1][1];
		
		var[0] = landmarks.AngleXvar[0];
		var[1] = landmarks.AngleXvar[1];
		
		n = 2;
		weight = weightX;
		break;
		
		case LAND_T:
		coord[0][0] = landmarks.Tcoord[0][0];
		coord[0][1] = landmarks.Tcoord[0][1];
		coord[1][0] = landmarks.Tcoord[5][0];
		coord[1][1] = landmarks.Tcoord[5][1];
		
		var[0] = landmarks.AngleTvar[0];
		var[1] = landmarks.AngleTvar[5];
		
		n = 2;
		weight = weightT;
		break;
		
		case LAND_GPY:
		coord[0][0] = landmarks.GPcoord[0][0];
		coord[0][1] = landmarks.GPcoord[0][1];
		coord[1][0] = landmarks.GPcoord[1][0];
		coord[1][1] = landmarks.GPcoord[1][1];
		
		var[0] = landmarks.AngleGPvar[0];
		var[1] = landmarks.AngleGPvar[1];
		
		n = 2;
		weight = weightGP;
		break;
		
		case LAND_GPB: 
		coord[0][0] = landmarks.GPcoord[2][0];
		coord[0][1] = landmarks.GPcoord[2][1];
		coord[1][0] = landmarks.GPcoord[3][0];
		coord[1][1] = landmarks.GPcoord[3][1];
		
		var[0] = landmarks.AngleGPvar[2];
		var[1] = landmarks.AngleGPvar[3];
		
		n = 2;
		weight = weightGP;
		break;
		
		case LAND_BYB:
		coord[0][0] = landmarks.LPcoord[1][0];
		coord[0][1] = landmarks.LPcoord[1][1];
		var[0] = landmarks.AngleLPvar[1];
		n = 1;
		weight = weightLP;
		break;
		
		case LAND_YBY:
		coord[0][0] = landmarks.LPcoord[0][0];
		coord[0][1] = landmarks.LPcoord[0][1];
		var[0] = landmarks.AngleLPvar[0];
		n = 1;
		weight = weightLP;
		break;
	}
	
	// int projY=0,projX=0;
	// float s=1,focal=533.33;	
	// float minprojY=(((0)+(focal/s)*tan(thetaX))/(1-(s/focal)*(0)*tan(thetaX)));
	// projY=(int)(((240-y)+(focal/s)*tan(thetaX))/(1-(s/focal)*(240-y)*tan(thetaX)));
	// projX=(int)(x-320)*((s/focal)*(projY)*sin(thetaX)+cos(thetaX));
	// projY=projY-minprojY;
	// projX*=pix2cm;
	// projY*=pix2cm;
	// float theta=baseAngle-rad2deg(thetaY)+150+rad2deg(atan((-1)*projX/(distance+projY)));
	float theta = findAngle(x, y, distance);
	//for(int k=0;k<6;k+=5)
	//{
	if(n==1)
	{
		for(int i=0;i<=XGridNum;i++)
		for(int j=0;j<=YGridNum;j++)
		{				
		//	float thetaGrid=rad2deg(atan((j-coord[0][1])/(float)(coord[0][0]-i)));
			float thetaGrid=rad2deg(atan2((j-coord[0][1]),(float)(i-coord[0][0])));
		//	if(thetaGrid<0)
		//	thetaGrid=180+thetaGrid;
			position[0][i][j]*=pow(exp(-1*abs(theta-thetaGrid)/(2*pow(var[0],1))),weight);
		}
	}
	
	//flagflag
	else if(n==2)
	{
	
		for(int i=0;i<=XGridNum;i++)
		for(int j=0;j<=YGridNum;j++)
		{				
		//	float thetaGrid=rad2deg(atan((j-coord[0][1])/(float)(coord[0][0]-i)));
		//	float thetaGrid2=rad2deg(atan((j-coord[1][1])/(float)(coord[1][0]-i)));
			float thetaGrid=rad2deg(atan2((j-coord[0][1]),(float)(i-coord[0][0])));
			float thetaGrid2=rad2deg(atan2((j-coord[1][1]),(float)(i-coord[1][0])));
		//	if(thetaGrid<0)
		//	thetaGrid=180+thetaGrid;
		//	if(thetaGrid2<0)
		//	thetaGrid2=180+thetaGrid2;
			p1=pow(exp(-1*abs(theta-thetaGrid)/(2*pow(var[0],1))),weight);
			p2=pow(exp(-1*abs(theta-thetaGrid2)/(2*pow(var[1],1))),weight);
			if(p1>p2)
				position[0][i][j]*= p1;
			else
				position[0][i][j]*= p2;
		}
	}
	
}


inline int greenCheck(double b, double g, double r)
{
	if((g<2.9*r)&&(g>2.1*r)&&(b<2.1*r)&&(b>1.5*r))
		return 1;
	else
		return 0;
	
}


void update_gps(IplImage* in, int *gp2, int color = YELLOWC)
{
	int color2 = (color == YELLOWC) ? BLUEC:YELLOWC;
	IplImage* image = cvCreateImage(cvSize(in->width, in->height), in->depth, 1);
	#define cross 5
	int i,j;
	for(i=0; i<in->width-4; i++)
		for(j=0; j<in->height-4; j++)
		{
			if( ( pixelColor1C(in,i,j) == pixelColor1C(in,i+cross/2,j+cross/2))&&( pixelColor1C(in,i,j) == pixelColor1C(in,i+cross-1,j+cross-1))&&( pixelColor1C(in,i,j) == pixelColor1C(in,i+cross-1,j) )&&( pixelColor1C(in,i,j) == pixelColor1C(in,i,j+cross-1) ))
			((uchar*)(image->imageData + j * image->widthStep))[i] =((uchar*)(in->imageData + j * in->widthStep))[i];
			else ((uchar*)(image->imageData + j * image->widthStep))[i] =0;
		}
	//Clear Edges
	for(i=in->width-4;i<in->width;i++)
		for(j=0; j<in->height-4; j++)
			((uchar*)(image->imageData + j * image->widthStep))[i] =0;
	for(i=0;i<in->width;i++)
		for(j=in->height-4; j<in->height; j++)
			((uchar*)(image->imageData + j * image->widthStep))[i] =0;
	

	int gp[2][2]={-1,-1,-1,-1}, jmin=0, jmax=0, j1=400,j2=400; 
	int gset=0, gcount, k;
	for (i = image->height-10; i >= 0 ; i--)
		for (j = image->width-1; (j >= 0 && gset < 2) ; j--)
			{start:
			if( ( chkcolor(image,j,i,color) ) && ( (j<jmin) || (j>jmax) || (gset==0) ) ) //Check yellow and boundaries
				{
					bool nyellow =1;
					//printf("Entered Function and found yellow");
					//Check Yellow in GPost
					for(k=1; k<10; k++)
					if( chkcolor(image,j,safe(i-10*k),color) ) nyellow=0;
					//Check Green
					for(k=6, gcount=0; k<40; k++)
					if( chkcolor(image,j,i+k,GREENC) ) gcount++;
					if(  (gcount<3) || nyellow ) {j--; goto start;} //
					else
					{
						i=safe(i-3);  //Accounting for round bottom
						//while(getcol(image,i,j,0,1)) j++;
						j1=jmax;
						jmax=j+60;
						while(chkcolor(image,j,i,color)) j--;
						j2=jmin;
						jmin=safe(j-60);
						i+=3;
						if( ((jmax+jmin)/2 < j1) && ((jmax+jmin)/2 > j2) )
						{
							//printf("Obsolete!\n");
							jmax=j1;
							jmin=j2;
							j=j--;
						}
						else
						{	
							for(k=6, gcount=0; k<40; k++)
							if( chkcolor(image,(jmin+jmax)/2,i+k,GREENC) ) gcount++;
							
							//if(gcount>2)
							//{
							//printf("%d %d %d\n",j,jmin,jmax);
							for(k=-50; k<50; k++)
							if(chkcolor(image,(jmax+jmin)/2,safe(i-2*k),color2) )
							{
							printf("Discarded1  value\n");
						
							break;
							 }
							 if(k!=50)
							 	continue;
							gp[gset][0]=i;
							gp[gset++][1]=(jmax+jmin)/2;
							//printf("No of Goal Posts Detected: %d\n",gset);
							//}
						}
					}
						
					
				}}
	//if(gset==2) 
	//printf("Goal Position %d %d and %d %d\n\n\n",gp[0][0],gp[0][1],gp[1][],gp[1][1]);
	
	//cvCircle(main,cvPoint(1,10),2,cvScalar(255,255,255),2);
	 if(gp[0][1]&&gp[0][0])  
	 {
	 //	cvCircle(main,cvPoint(gp[0][1],gp[0][0]),2,cvScalar(125),2);
	 	gp2[0] = gp[0][1];
	 	gp2[1] = gp[0][0];
	 }
	 if(gp[1][1]&&gp[1][0]) 
	 {
	  //	cvCircle(main,cvPoint(gp[1][1],gp[1][0]),2,cvScalar(125),2);
	  	gp2[2] = gp[1][1];
	  	gp2[3] = gp[1][0];
	  }
	  cvReleaseImage(&image);
	//return gp[2][];
}

void doSegment(IplImage* image, IplImage* out)
{
	//IplImage* out = cvCreateImage(cvSize(image->width, image->height),image->depth, 1);
	int x,y;

	for(x = 0; x < out->width; x++)
	{
		for(y = 0; y < out->height; y++)
		{
			
			if(chkcolor(image, x, y, GREENC))	//green check
			{
				returnPixel1C(out, x, y) = 127;
				continue;
			}
			else if(chkcolor(image, x, y, WHITEC))
			{
				returnPixel1C(out, x, y) = 255;
				continue;
			}
			else if(chkcolor(image, x, y, BLACKC))
			{
				returnPixel1C(out, x, y) = 50;
				continue;
			}
			else 
			{
				returnPixel1C(out, x, y) = 0;
			}
		}
	}
	
	return;
}
			

inline int checkBox(IplImage* image, int x0, int y0, int width, int height, int color, int noOfColor)
{
	int x1 = x0 - width;
	int y1 = y0 - height;
	int x2 = x0 + width;
	int y2 = y0 + height;
	int x, y;
	int count=0;
	if (x1 < 0 ) x1 = 0;
	if (y1 < 0) y1 = 0;
	
	if (x2 >= image->width) x2 = image->width - 1;
	if (y2 >= image->height) y2 = image->height-1;
	
	for (x = x1; x <= x2; x++)
	{
		for(y = y1; y <= y2; y++)
		{
			
			if (pixelColor1C(image, x, y)==color)
				count++;
		}
	}
	
	
	if (count>=noOfColor)
		return 1;
	else
		return 0;
	
}

void doBWInGreen(IplImage* image, IplImage* out, int nLoops=4)
{
	int x, y;
	setBlank1C(out);
	int loopCounter = 0;
	int count = 0;
	for(x = 0; x< image->width; x++)
	{
		for(y = 0; y< image->height; y++)
		{
			if(pixelColor1C(image, x, y)==255 || pixelColor1C(image, x, y)==50)
			{
				if(checkBox(image, x, y, 1, 1, 127, 2))
				{
					returnPixel1C(out, x, y) = pixelColor1C(image, x, y);
				}
			}
		}
	}
	//Boundary created. Filling of boundary follows.
	do
	{
		loopCounter = 0;
		for(x = 0; x< image->width; x++)
		{
			for(y = 0; y< image->height; y++)
			{
				if(pixelColor1C(image, x, y) == 255)
				{
					if(pixelColor1C(out, x, y)!=255)
					{
						if(checkBox(out, x, y, 1, 1, 255, 2))
						{
							returnPixel1C(out, x, y) = pixelColor1C(image, x, y);
							loopCounter++;
						}
					}
				}
				else if(pixelColor1C(image, x, y) == 50)
				{
					if(pixelColor1C(out, x, y)!=50)
					{
						if(checkBox(out, x, y, 1, 1, 50, 2))
						{
							returnPixel1C(out, x, y) = pixelColor1C(image, x, y);
							loopCounter++;
						}
					}
				}
			}
		}
		count++;
		
	}while((loopCounter > 0));
	
	for(x = 0; x< image->width; x++)
	{
		for(y = 0; y< image->height; y++)
		{
			if(pixelColor1C(image, x, y)==127)
			{
				returnPixel1C(out, x, y) = 100;
		
			}
		}
	}

	//printf("Count %d .", count);
	return;
}

void doWhiteInGreen(IplImage* image, IplImage* out, int nLoops=4)
{
	int x, y;
	setBlank1C(out);
	int loopCounter = 0;
	int count = 0;
// /Extra filter begins	
//	//This extra filter removes excess top and right whites (those that are before 1st green)
//	//If removing extra filter, remove following declaration of image
//	//and change parameter in to image
//	//To add filter, uncomment all this and change parameter image to in
//	IplImage* image = cvCreateImage(cvSize(in->width, in->height), in->depth, 1);
//	setBlank1C(image);
//	

//	int flag = 0;

//	for (x = 0; x< in->width; x++)
//	{	
//		flag = 0;
//		for(y = 0; y< in->height; y++)
//		{
//			
//			if(flag)
//			{
//				returnPixel1C(image, x, y) = pixelColor1C(in, x, y);
//			}
//			else
//			{
//				if(pixelColor1C(in, x, y)==127)
//				{
//					returnPixel1C(image, x, y) = 127;
//					flag = 1;
//				}
//			}
//		}
//	}
//			
//			

//	for(y = 0; y< in->height; y++)
//	{
//		flag = 0;
//		for(x = in->width-1; x>=0; x--)
//		{
//			if(flag)
//			{
//				returnPixel1C(image, x, y) = pixelColor1C(in, x, y);
//			}
//			else
//			{
//				if(pixelColor1C(in, x, y)==127)
//				{
//					returnPixel1C(image, x, y) = 127;
//					flag = 1;
//				}
//			}
//			
//		}
//	}
//	
////Extra filter ends		
	for(x = 0; x< image->width; x++)
	{
		for(y = 0; y< image->height; y++)
		{
			if(pixelColor1C(image, x, y)==255)
			{
				if(checkBox(image, x, y, 1, 1, 127, 2))
				{
					returnPixel1C(out, x, y) = pixelColor1C(image, x, y);
				}
			}
		}
	}
	//White boundary created. Filling of boundary follows.
	do
	{
		loopCounter = 0;
		for(x = 0; x< image->width; x++)
		{
			for(y = 0; y< image->height; y++)
			{
				if(pixelColor1C(image, x, y) == 255)
				{
					if(pixelColor1C(out, x, y)!=255)
					{
						if(checkBox(out, x, y, 1, 1, 255, 2))
						{
							returnPixel1C(out, x, y) = pixelColor1C(image, x, y);
							loopCounter++;
						}
					}
				}
			}
		}
		count++;
		
	}while((loopCounter > 0) && count<=nLoops);
	
	//printf("Count %d .", count);
	return;
}
	
	
//might have problem showing last row/column
void doIncreaseSize(IplImage* in, IplImage* out, int gridSize)
{
	int x, y;
	for (x=0; x< out->width; x++)
	{
		for(y = 0; y<out->height; y++)
		{
			returnPixel1C(out, x, y) = pixelColor1C(in, x/gridSize, y/gridSize);
		}
	}
}

void doReduceSizeS(IplImage* in, IplImage* out, int gridSize)
{
	int x, y, i, j;
	int countg, countw, countb;
	for (x=0; x< out->width; x++)
	{
		for(y = 0; y<out->height; y++)
		{
			countg = 0;
			countw = 0;
			countb = 0;
			for(i = 0; i<gridSize; i++)
			{
				for(j=0; j<gridSize; j++)
				{
					if(pixelColor1C(in,x*gridSize+i, y*gridSize+j)==255)
					{
						countw++;
					}
					else if(pixelColor1C(in, x*gridSize+i, y*gridSize+j)==127)
					{
						countg++;
					}
					else if(pixelColor1C(in, x*gridSize+i, y*gridSize+j)==50)
					{
						countb++;
					}
				}
			}
			if ((countg==0)&&(countw==0)&&(countb==0))
				returnPixel1C(out, x, y) = 0;
			else
			{
				//changing the following condition does little (no visible difference)
				if((countg==0)&&(countw==0))
					returnPixel1C(out, x, y) = 50;
				else if (countg > 4*countw)
					returnPixel1C(out, x, y) = 127;
				else
					returnPixel1C(out, x, y) = 255;
			}
		}
	}
}

/* Set of functions for performing alternate whiteingreen. 
   This makes a histogram of green on each column, ie counts
   number of greens in each column, then smooths the histogram
   using some technique (median here). Then, only those whites 
   below the histogram are used.
*/
/* ALT whiteingreen BEGIN */
inline void getHist(IplImage* image, IplImage* hist)
{
	int x, y;
	for(x = 0; x< image->width; x++)
	{	
		for(y = 0; y< image->height; y++)
		{
			if(pixelColor1C(image, x, y)==127)
			{
				returnPixel1C(hist, x, 0) = y;
				break;
			}
		}
	}

}

//Change 5 to larger number for better smoothing. 3 doesn't work well.
inline void smoothHist(IplImage* hist, IplImage* hist2, int size)
{
	cvSmooth(hist, hist2, 5);
	cvSmooth(hist2, hist, CV_MEDIAN,5);
}


inline void doWhiteBelowHist(IplImage* image, IplImage* out, IplImage* hist)
{
	setBlank1C(out);
	int x, y;
	for(x = 0; x < image->width; x++)
	{
		for(y = pixelColor1C(hist, x, 0); y< image->height; y++)
		{
			if(pixelColor1C(image, x, y)==255)
				returnPixel1C(out, x, y) = 255;
		}
	}
}

/* ALT whiteingreen END */

IplImage* doWhiteFilter(IplImage* frame2, int gridSize, IplConvKernel* morphKernel)
{	
	IplImage* frame3 = cvCreateImage(cvSize(frame2->width,frame2->height),8, 1);
	IplImage* frame4 = cvCreateImage(cvSize(frame2->width/gridSize,frame2->height/gridSize),8, 1);
	IplImage* frame5 = cvCreateImage(cvSize(frame4->width, frame4->height), 8, 1);
	
//	IplImage* histogram = cvCreateImage(cvSize(frame4->width, 1), 8, 1);
//	IplImage* histogram2 = cvCreateImage(cvSize(frame4->width, 1), 8, 1);
	
/* 	doSegment now segments white, green, black */
	doSegment(frame2, frame3);
	
	cvMorphologyEx(frame3, frame3, NULL, morphKernel, CV_MOP_OPEN, 1);
	
	doReduceSizeS(frame3, frame4, gridSize);
//	getHist(frame4, histogram);
//	
//	smoothHist(histogram, histogram2, frame5->width);
//
	//doWhiteInGreen(frame4, frame5);
	/* BWInGreen is used as black in white is also required for obstacle detection */
	doBWInGreen(frame4,frame5);
	//cvShowImage("Seg", frame5);

	//doWhiteBelowHist(frame4, frame5, histogram);

	cvReleaseImage(&frame3);
	cvReleaseImage(&frame4);
	
	return frame5;	
}


//READING/WRITING TO FTDI CODE BEGINS

#define WRITE ftdi_write_data(&ftdic1_camera,pack,pack[3]+4)

#define NO_OF_TRIES 20
struct ftdi_context ftdic1_camera;

int global_id=1;
int read_pos();

int bootup_files()
{
    int ret;
    if (ftdi_init(&ftdic1_camera) < 0)
    {
        fprintf(stderr, "ftdi_init failed\n");
        return -1;
    }
    
    if ((ret = ftdi_usb_open_desc (&ftdic1_camera, 0x0403, 0x6001, NULL, serialusb2d_camera)) < 0)
    {
        fprintf(stderr, "unable to open ftdi device: %d (%s)\n", ret, ftdi_get_error_string(&ftdic1_camera));
        printf("unable to open USB2D_camera\n");
        sleep(1);
        return -1;
    }
    else
    printf("USB2D ftdi successfully open\n");

    ftdi_set_baudrate(&ftdic1_camera,1000000);
    return ret;
}

int checksum(byte *pack)
{
	byte checksum = 0;
	int i;
	for (i=2; i<pack[3]+3; i++)
	{
		checksum +=pack[i];
	}
	checksum = 255 - checksum;
	pack[pack[3]+3]=checksum;
	return (int)checksum;
}

int read_pos(int id)
{


  byte pack2[6], pack[]={0xff,0xff,id,0x04,0x02,0x24,0x02,0x00};
  byte c1 = 0, c2=0, lpos = 0, hpos = 0;
  int j=0, pos=0;
  long i = 0;
  checksum(pack);
  ftdi_usb_purge_buffers(&ftdic1_camera);
  WRITE;
  usleep(1000);
  while((c1!=255)||(c2!=255))
  {
  	i++;
 	c2=c1;
	ftdi_read_data(&ftdic1_camera,&c1,1);
 	if(i==NO_OF_TRIES)
 	break;
  } 
 
  if(c1==255&&c2==255)
  {
  	while(ftdi_read_data(&ftdic1_camera,&c1,1)==1)
  	{
  		pack[j++] = c1;
  		if(j>=8)
  		{
  			break;
  		}

  	}
  	
  	if (pack[2]==0)
  	{
  		lpos = pack[3];
  		hpos = pack[4];
  		pos = lpos + hpos*256;
  	//	printf("Position is %d ",pos);
  		return pos;
  	}
  	else
  	{
  		printf("Error (pos): %d",pack[2]);
  		return -pack[2];
  	}
  	
  }
  else
  {
  	printf("Unable to read (pos)  %d  %d  ",c1,c2);
  	return -1;
  }
 
}

int ismoving_motor(int id)
{
  byte pack[]={0xff,0xff,id,0x04,0x02,0x2E,0x01,0x00};
  byte c1 = 0, c2=0;
  int j=0, pos=0;
  long i = 0;
  checksum(pack);
  ftdi_usb_purge_buffers(&ftdic1_camera);
  WRITE;
  usleep(1000);
 
  while((c1!=255)||(c2!=255))
  {
  	i++;
 	c2=c1;
	ftdi_read_data(&ftdic1_camera,&c1,1);
 	if(i==NO_OF_TRIES)
 	break;
  }
 
  if(c1==255&&c2==255)
  {
 	while(ftdi_read_data(&ftdic1_camera,&c1,1)==1)
  	{
 		pack[j++] = c1;
  		if(j>=8)
  		{
			break;
		}
  	}

  	if (pack[2]==0)
  	{
  		return pack[3];
  	}
  	else
  	{
  		printf("Error (ismoving): %d\n",pack[2]);
  		return -pack[2];
  	}
  	
  }
  else
  {
  	printf("Unable to read (ismoving) %d  %d  ",c1,c2);
  	return -1;
  }

}

int set_gp(int id,int gp)
{  
	byte gpl, gph;
	gpl = gp%256;
	gph = gp/256;
	
  	byte pack[] = {0xff, 0xff, id,0x05 ,0x03, 0x1e, gpl,gph,0x00};
  	checksum(pack);
  	
	WRITE;
}

int set_gp(int gp)
{
	set_gp(global_id,gp);
}

int set_speed(int id, int sp)
{
	byte sl, sh;
	sl=sp%256;
	sh=sp/256;	
	byte pack[] = {0xff, 0xff, id, 0x05 ,0x03, 0x20,sl,sh,0x00};
	checksum(pack);
	WRITE;
}	

//Motor IDS hardcoded
//Shouldn't be a problem as they are a standard

int speed_motor(int speed1, int speed2)
{

	set_speed(18, speed1);
	set_speed(17, speed2);

}

int sync_write_gp(int id[], int gp[], int n)
{
	int i, c=7;
	byte pack[3*n+8];
	pack[0] = pack[1] = 0xff;
	pack[2]=0xfe;
	pack[3]=3*n+4;
	pack[4]=0x83;
	pack[5]=0x1e;
	pack[6]=2;
	
	for(i=0;i<n;i++)
	{
		pack[c++]=id[i];
		pack[c++]=gp[i]%256;
		pack[c++]=gp[i]/256;
	        
	}
	
	checksum(pack);
//	for (i=0; i<(3*n + 8);i++)
//	{
//		printf("%d ", pack[i]);
//	}
	WRITE;
}

int sync_write_gp()
{
	int id[]={1,2};
	int n = 2;
	int gp[2];
	printf("Enter gp1 and gp2 : ");
	scanf("%d %d",&gp[0],&gp[1]);
	sync_write_gp(id, gp, n);
}

int write_motor(float thetax, float thetay)
{
	int id[2] = {18,17};
	int gp[2]= {(300.0 - thetay)*(1023.0/300.0) - offsety, (300.0 - thetax)*(1023.0/300.0) - offsetx};
	sync_write_gp(id, gp, 2);
//	set_gp(18, (300.0 - thetay)*(1023.0/300.0) - offsety);
//	set_gp(17, (300.0 - thetax)*(1023.0/300.0) - offsetx);
}

int read_motor(float &thetax, float &thetay)
{
	int temp18, temp17;
	temp18 = read_pos(18);
	temp17 = read_pos(17);
	if((temp17<0)||(temp18<0))
		return ERROR_MOTORS;
	thetay = 300.0 - ((temp18+offsety)*300.0/1023.0);
	thetax = 300.0 - ((temp17+offsetx)*300.0/1023.0);
	return 1;	
}

int stop_motor()
{
	float tx, ty;
	if(read_motor(tx, ty)==1)
	{
		thetaX = deg2rad(tx);
		thetaY = deg2rad(ty);
		write_motor(tx, ty);
		return 1;
	}
	else
	{
		return 0;
	}
}


void scurve(int id[],int final[], float time)
{
	#define vmax 3
	#define Smoo(x) 3*x*x - 2*x*x*x;//vmax*x + x*x*x -vmax*x*x;//(vmax*((x)+((x)*(x)*(x)/vmax)-(x)*(x)))
	float x,y;
	float t = 0;
	float init[2];
	int gp[2];
	init[0] = read_pos(id[0]);
	init[1] = read_pos(id[1]);
	t = time;
	if (t<0)
		t = -t;
	
	
	x = 1/(FPS*t);
	for (; x<=1; x = x+ 1/(FPS*t))
	{
		y= Smoo(x);
		gp[0]= y*final[0] + (1-y)*init[0];	
		gp[1]= y*final[1] + (1-y)*init[1];	
		sync_write_gp(id,gp,2);
		usleep(1000000.0/FPS);
	}
	
	#undef Smoo
	#undef vmax
}


int swrite(float thetax, float thetay, float ttime=1.0)
{

	int id[2] = {17,18};
	int final[2] = {(300.0 - thetax)*(1023.0/300.0) - offsetx,(300.0 - thetay)*(1023.0/300.0) - offsety};
	scurve(id, final, ttime);
	return 1;


}

/*
int swrite(int thetax, int thetay, int ttime=2)
{
	#define Smoo(x) (3*(x)*(x)-2*(x)*(x)*(x))
	ttime=30*ttime;
	int dtx=thetax-deg2rad(thetaX);
	int dty=thetay-deg2rad(thetaY);
	thetax=deg2rad(thetaX);
	thetay=deg2rad(thetaY);
	for(int ctime=0; ctime < ttime; ctime++)
	{
		write_motor((int)(Smoo((float)ctime/ttime)*dtx)+thetax,(int)(Smoo((float)ctime/ttime)*dty)+thetay);
		usleep(33333);
	}
	#undef Smoo
	return 1;
}
*/

int go2state(int state)
{
	state=state%6;
	int c[2];
	switch(state)
	{
		case 2:
		c[0] = 60;
		c[1] = 90;
		break;
		case 1:
		
		c[0] = 60;
		c[1] = 150;
		break;
		case 0:
		
		c[0] = 60;
		c[1] = 210;
		break;
		case 3:
		
		c[0] = 45;
		c[1] = 90;
		break;
		case 4:
		
		c[0] = 45;
		c[1] = 150;
		break;
		case 5:
		
		c[0] = 45;
		c[1] = 210;
		break;
		
		default: return -1;
	}
	write_motor(c[0], c[1]);	
//	write_motor(c[0], c[1]);
	
	
	return 0;
}



//READING/WRITING TO FTDI CODE ENDS

int xyCompare(int *xy1, int* xy2, int size)
{
	if (fabs(xy1[0] - xy2[0]) <= size)
	{
		if (fabs(xy1[1] - xy2[1]) <= size)
		{
			return 1;
		}
	}
	return 0;
}


inline int ifTrue(IplImage* image, int x, int y)
{
  if(pixelColor1C(image, x, y)>0)
    {
      return 1;
    }
  else
    return 0;
}
	
	

inline void getPattern(IplImage* image, int x, int y, int *p)
{
	int width = image->width;
	int height = image->height;
	
	if (x >0)
	{
		p[6] = ifTrue(image, x-1, y);
		if (y>0)
			p[7] = ifTrue(image, x-1, y-1);
			
		if (y<height-1)
			p[5] = ifTrue(image, x-1, y+1);
	}
	
	if (x < width-1)
	{
		p[2] = ifTrue(image, x +1, y);
		
		if (y > 0)
			p[1] = ifTrue(image, x +1, y-1);
		if (y < height - 1)
			p[3] = ifTrue(image, x+1, y+1);
	}
	
	if (y>0)
		p[0] = ifTrue(image, x, y-1);
	
	if (y<height - 1)
		p[4] = ifTrue(image, x, y+1);
		
}
	


inline int bHild(IplImage* image, int x, int y)
{
	int p[8] = {0,0,0,0,0,0,0,0};
	getPattern(image, x, y, p);
	int i = 0;
	int count = 0;
	for(i=0; i<8; i++)
	{
		if (p[i] > 0)
			count++;
	}	
	return count;
}


inline int aHild(IplImage* image, int x, int y)
{
	int p[8] = {0,0,0,0,0,0,0,0};
	getPattern(image, x, y, p);
	int i, count = 0;
	for (i = 0; i< 7; i++)
	{
		if ((p[i]==0)&&(p[i+1]==1))
			count++;
	}
	if((p[7]==0)&&(p[0]==1))
		count++;
		
	return count;
}


void doHilditch(IplImage* image)
{
	int x, y;
	int change = 0, change2=0;
	int temp;
	int p[8] = {0,0,0,0,0,0,0,0};
	int loops= 0;

// The following code should set all useful stuff to 1, others to 0, else Hilditch won't work
// 50 is BLACKC and 100 is GREENC (after obstacle modification)
	for(x = 0; x<image->width; x++)
	{
		for(y = 0; y<image->height; y++)
		{
			if((pixelColor1C(image, x, y)==50)||(pixelColor1C(image, x, y)==100))
			{
				returnPixel1C(image, x, y) = 0;
			}
			else if(pixelColor1C(image, x, y)>0)
			{
				returnPixel1C(image, x, y) = 1;
			}
		}
	}
	
	  
		
	
	
	do
	{
		change = 0;loops++;
		change2 = 0;
		
		for(x = 0; x<image->width; x++)
		{
			for(y = 0; y<image->height; y++)
			{
				
				if(pixelColor1C(image, x, y) > 0)
				{
					getPattern(image, x, y, p);
					if(p[2]*p[4]*p[6]==0)
					{
						if(p[0]*p[2]*p[4]==0)
						{
							temp = bHild(image, x, y);
							if ((temp>=2)&&(temp<=6))
							{
								if(aHild(image, x, y) ==1)
								{
									returnPixel1C(image, x, y) = 2;
									change++;
							
										
								}
							}			
						}
					}
				}
			}
		}

		for(x = 0; x<image->width; x++)
		  for(y=0; y<image->height; y++)
		    if(pixelColor1C(image, x, y)==2)
		      returnPixel1C(image, x, y)=0;


		//Second subiteration



		for(x = 0; x<image->width; x++)
		{
			for(y = 0; y<image->height; y++)
			{
				
				if(pixelColor1C(image, x, y) > 0)
				{
					getPattern(image, x, y, p);
					if(p[0]*p[2]*p[6]==0)
					{
						if(p[0]*p[4]*p[6]==0)
						{
							temp = bHild(image, x, y);
							if ((temp>=2)&&(temp<=6))
							{
								if(aHild(image, x, y) ==1)
								{
									returnPixel1C(image, x, y) = 2;
									change2++;
							
										
								}
							}			
						}
					}
				}
			}
		}



		for(x = 0; x<image->width; x++)
		  for(y=0; y<image->height; y++)
		    if(pixelColor1C(image, x, y)==2)
		      returnPixel1C(image, x, y)=0;
		
	}while((change>0)&&(change2>0));
	//Condition should actually be &&
	
	
	
	
	for(x = 0; x<image->width; x++)
	{
		for(y = 0; y<image->height; y++)
		{
			if (pixelColor1C(image, x, y) == 1)
				returnPixel1C(image, x, y) = 255;
			
		}
		
	}
	
	//	printf("Loops = %d", loops);
}


int D1[5][5]={
1,0,0,1,2,
0,1,1,1,2,
0,1,1,0,2,
2,1,0,1,2,
2,2,2,2,2};

int D2[5][5]={
2,1,0,1,2,
0,0,1,1,2,
0,1,1,1,2,
2,1,0,0,2,
2,2,2,2,2};


inline int doPass(IplImage* image, int x0, int y0, int matrix[5][5])
{
	int x, y;
	
	for (x=-2; x<=2; x++)
	{
		for(y=-2; y<=2; y++)
		{
			if ( matrix[y+2][x+2]== 2)
				continue;
			if(ifTrue(image, x0+x, y0+y)!=matrix[y+2][x+2])
				return 0;
		}
	}
//	printf ("it returned 1 somewhere");
	return 1;
	
}



void doDetectXT(IplImage* image, int (*arrT)[2], int (*arrX)[2], int &Tnum, int&Xnum, int size=4)
{
	Tnum = 0;
	Xnum = 0;
	int temp[25][2];
	int x, y, count1, count2;
	//Replacing 255 with 1
	for(x = 0; x<image->width; x++)
	{
		for(y = 0; y<image->height; y++)
		{
			if (pixelColor1C(image, x, y) == 255)
				returnPixel1C(image, x, y) = 1;
			
		}
		
	}
	
	
	for(x = 0; x < image->width; x++)
	{
		for(y = 0; y< image -> height; y++)
		{
			if(pixelColor1C(image, x, y)==1)
			{
				
				if(bHild(image, x, y)>=3)
				{
					//printf("bhild only");  BIG PROBLEM HERE
					
					
					if(aHild(image, x, y)>=3)
					{
					//	printf("Got something");
						temp[Tnum][0] = x;
						temp[Tnum][1] = y;
						Tnum++;
					}
				}
			}
		}
	}
	
	
	
	for(count1 = 0; count1 < Tnum; count1++)
	{
		if (temp[count1][0]!= -1)
		{
			for(count2 = 0; count2 < Tnum; count2++)
			{
				if (count1 != count2)
				{
				if(temp[count2][0] != -1)
				{	
					if ((fabs(temp[count1][0] - temp[count2][0]) <=size)&&(fabs(temp[count1][1] - temp[count2][1]) <=size))
					//if (xyCompare(xy[count1], xy[count2], 6))
					{
						arrX[Xnum][0] = (temp[count1][0] + temp[count2][0])/2;
						arrX[Xnum][1] = (temp[count1][1] + temp[count2][1])/2;
						Xnum++;
						temp[count1][0] = -1;
						temp[count2][0] = -1;
						break;
					}
				}
				}
			}
		}
		
	}
	
	for(count1 = 0, count2 = 0; count1 < Tnum; count1++)
	{
		if(temp[count1][0]!= -1)
		{
			arrT[count2][0] = temp[count1][0];
			arrT[count2][1] = temp[count1][1];
			count2++;
		}
	}

	Tnum = count2;
	//Loop for special Xs
	for(x = 2; x< image->width-2; x++)
	{
		for(y = 2; y< image->height-2; y++)
		{
			if(pixelColor1C(image, x, y)==1)
			{
				if(doPass(image, x, y, D1))
				{
					arrX[Xnum][0] = x;
					arrX[Xnum][1] = y;
					Xnum++;
					
				}
				else if(doPass(image, x, y, D2))
				{
					arrX[Xnum][0] = x;
					arrX[Xnum][1] = y;
					Xnum++;
				
				}
				
			}
		}
	}

				
	//Replacing 1 with 255
	for(x = 0; x<image->width; x++)
	{
		for(y = 0; y<image->height; y++)
		{
			if (pixelColor1C(image, x, y) == 1)
				returnPixel1C(image, x, y) = 255;
			
		}
		
	}

}

inline int xyEqual(int arr1[2], int arr2[2])
{
	if ((arr1[0]==arr2[0])&&(arr1[1]==arr2[1]))
		return 1;
	else
		return 0;
}

inline char checkLandmarks(int x, int y)
{
	//T Check
	int i=0;
	int pose[2]={x, y};
	for(i=0; i<6; i++)
		if(xyEqual(landmarks.Tcoord[i], pose))
			return 'T';
	
	//X Check
	
	for(i=0; i<2; i++)
		if(xyEqual(landmarks.Xcoord[i], pose))
			return 'X';
	
	//GP Check
	
	for(i=0; i<4; i++)
		if(xyEqual(landmarks.GPcoord[i], pose))
			return 'G';
	
	//LP Check
	
	for(i=0; i<2; i++)
		if(xyEqual(landmarks.LPcoord[i], pose))
			return 'L';
	
	//None found
	return 'n';
	
}


float pos[1944][2];

void doSwap(int &x, int &y)
{
	int temp;
	temp = x;
	x = y;
	y = temp;
	return;
}

void doSwap(float &x, float &y)
{
	float temp;
	temp = x;
	x = y;
	y = temp;
	return;
}

void doSort(int* arr, int size, int type = ASC)
{
	int i, j;
	if(type==ASC)
	{
		for(j = 0; j < size; j++)
		{
			for(i = j; i< size; i++)
			{
				if(arr[j] > arr[i])
					doSwap(arr[j], arr[i]);
			}
		}
	}
	else if(type==DESC)
	{
		for(j = 0; j < size; j++)
		{
			for(i = j; i< size; i++)
			{
				if(arr[j] < arr[i])
					doSwap(arr[j], arr[i]);
			}
		}
	}
	return;
	
}

void doSort(float* arr, int size, int type = ASC)
{
	int i, j;
	if(type==ASC)
	{
		for(j = 0; j < size; j++)
		{
			for(i = j; i< size; i++)
			{
				if(arr[j] > arr[i])
					doSwap(arr[j], arr[i]);
			}
		}
	}
	else if(type==DESC)
	{
		for(j = 0; j < size; j++)
		{
			for(i = j; i< size; i++)
			{
				if(arr[j] < arr[i])
					doSwap(arr[j], arr[i]);
			}
		}
	}
	return;
	
}

int doDetectYBY(IplImage* image, int *pos)
{
	int x, y;
	int yflag,  bflag;
	int arrx[100];
	int arry[100];
	int limit = 10;
	int limit2 = 5;
	int limit3 = 5;
	int arrnum = 0;
	int count1, count2, count3, count4;	//maybe only one of these is required
	int y2flag = 0;
	
//	IplImage* image = cvClone(in);//cvCreateImage(cvSize(in->width, in->height), in->depth, 1);
	// IplConvKernel* morphKernel = cvCreateStructuringElementEx(5, 5, 1, 1, CV_SHAPE_RECT, NULL);
	// cvMorphologyEx(in, image, NULL, morphKernel, CV_MOP_OPEN, 1);
	//image = cvClone(in);
	
	for(x = 0; x< image->width; x++)
	{
		

		yflag = 0;
		
		bflag = 0;
		count1 = 0;
		count2 = 0;
		count3 = 0;
		count4 = 0;
		y2flag = 0;
		for(y = 0; y< image->height; y++)
		{
			
			if(yflag==0)
			{
				
				if(chkcolor(image, x, y, YELLOWC))
				{
				//	printf("1st Yellow detected\n\n");
					yflag = 1;
					continue;
				}
			}
			else
			{
				//First yellow detected
				if(bflag==0)
				{
					if(chkcolor(image, x, y, YELLOWC))
					{
						continue;
					}
					else if(chkcolor(image, x, y, BLUEC))
					{
						bflag = 1;
						continue;
					}
					else
					{
						count1++;
						if(count1>limit)
						{
							yflag = 0;
							count1 = 0;
							count2 = 0;
							count3 = 0;
							continue;
						}
						//break;
					}
				}
				else
				{
					//Yellow and first blue detected
				//	printf("IN LAST LOOP!");
					if(chkcolor(image, x, y, BLUEC))
					{
						count3++;
						continue;
					}
					else if(chkcolor(image, x, y, YELLOWC)&&(count3>=limit2))
					{
					//	printf("FOUND %d\n", arrnum+1);
						//Yellow, blue and first yellow detected
						count4++;
						if(count4>=limit3)
							y2flag = 1;
						
						
					}
					else
					{
						count2++;
						if(y2flag)
						{
							arrx[arrnum] = x;
							arry[arrnum] = y;
							arrnum++;
							break;
						}
						else if(count2 > limit)
						{
							bflag = 0;
							yflag = 0;
							count1 = 0;
							count2 =0;
							count3 = 0;	
							continue;
						}
					//	break;
					}
						
						
				}
			}
		}
		
	}
		
//	cvReleaseImage(&image);
	if(arrnum)
	{
		doSort(arrx, arrnum, ASC);
		doSort(arry, arrnum, ASC);
		pos[0] = arrx[(arrnum-1)/2];
		pos[1] = arry[(arrnum-1)/2];
		return 1;
	}
	else
	{
		return 0;
	}
		
		
				
							
			

}


int doDetectBYB(IplImage* image, int *pos)
{
	int x, y;
	int yflag,  bflag;
	int arrx[100];
	int arry[100];
	int limit = 10;
	int limit2 = 10;
	int limit3 = 10;
	int arrnum = 0;
	int count1, count2, count3, count4;	//maybe only one of these is required
	int y2flag = 0;
	
	// IplImage* image = cvCreateImage(cvSize(in->width, in->height), in->depth, 1);
	// IplConvKernel* morphKernel = cvCreateStructuringElementEx(5, 5, 1, 1, CV_SHAPE_RECT, NULL);
	// cvMorphologyEx(in, image, NULL, morphKernel, CV_MOP_OPEN, 1);
	
	for(x = 0; x< image->width; x++)
	{
		

		yflag = 0;
		
		bflag = 0;
		count1 = 0;
		count2 = 0;
		count3 = 0;
		count4 = 0;
		y2flag = 0;
		for(y = 0; y< image->height; y++)
		{
			
			if(yflag==0)
			{
				
				if(chkcolor(image, x, y, BLUEC))
				{
				//	printf("1st Yellow detected\n\n");
					yflag = 1;
					continue;
				}
			}
			else
			{
				//First yellow detected
				if(bflag==0)
				{
					if(chkcolor(image, x, y, BLUEC))
					{
						continue;
					}
					else if(chkcolor(image, x, y, YELLOWC))
					{
						bflag = 1;
						continue;
					}
					else
					{
						count1++;
						if(count1>limit)
						{
							yflag = 0;
							count1 = 0;
							count2 = 0;
							count3 = 0;
							continue;
						}
						//break;
					}
				}
				else
				{
					//Yellow and first blue detected
				//	printf("IN LAST LOOP!");
					if(chkcolor(image, x, y, YELLOWC))
					{
						count3++;
						continue;
					}
					else if(chkcolor(image, x, y, BLUEC)&&(count3>=limit2))
					{
						//	printf("FOUND %d\n", arrnum+1);
							//Yellow, blue and first yellow detected
						count4++;
						if(count4>=limit3)
							y2flag = 1;				
					}
					else
					{
						count2++;
						if(y2flag)
						{
							arrx[arrnum] = x;
							arry[arrnum] = y;
							arrnum++;
							break;
						}
						else if(count2 > limit)
						{
							bflag = 0;
							yflag = 0;
							count1 = 0;
							count2 =0;
							count3 = 0;	
							continue;
						}
					//	break;
					}
				}
			}
		}
	}
	
//	cvReleaseImage(&image);
	
	if(arrnum)
	{
		doSort(arrx, arrnum, ASC);
		doSort(arry, arrnum, ASC);
		pos[0] = arrx[(arrnum-1)/2];
		pos[1] = arry[(arrnum-1)/2];
		return 1;
	}
	else
	{
		return 0;
	}
	
}

int doCompare3C(int* col1, int* col2)
{
	if((col1[0]==col2[0])&&(col1[1]==col2[1])&&(col1[2]==col2[2]))
	{
		return 1;
	}
	else
	{
		return 0;
	}
}



int doImageOverlay3C(IplImage* base, IplImage* sprite, int x, int y, int transparency=1, int *col = NULL)
{
	int color[3] = {0,255,0};
	if(col!=NULL)
	{
		doCopyArray(col, color, 3);	
	}

	x = x - sprite->width/2;
	y = y - sprite->height/2;


	int temp[3];
	for(int i=0; ((i < sprite->width)&&(x + i < base->width)); i++)
	{
		for(int j=0; ((j< sprite->height)&&(y+j < base->height)); j++)
		{
			if(((x+i)<0)||((y+j)<0))
			{
				continue;
			}
			if(transparency==1)
			{
				pixelColor3C(sprite, i, j, temp);
				if(temp[1]>200)
					continue;
					
				if(doCompare3C(temp,color))
				{
					continue;
				}
			}

			returnPixel3C(base, (x+i), (y+j), RED) = pixelColor3C(sprite, i, j, RED);
//			((uchar*)(base->imageData + base->widthStep*(y+j)))[(x+i)*3 + RED] = pixelColor3C(sprite, i, j, RED);
			returnPixel3C(base, (x+i), (y+j), GREEN) = pixelColor3C(sprite, i, j, GREEN);
			returnPixel3C(base, (x+i), (y+j), BLUE) = pixelColor3C(sprite, i, j, BLUE);

			
		}
	}	
	return 1;
}

void doDrawField(IplImage* in, int GS = 10)
{
	int line_thickness = 2;
	
	CvScalar line_color= {255,255,255};

	for(int i=0; i< (XGridNum+2)*GS; i++)
		cvLine(in, cvPoint(i, 0), cvPoint(i, (YGridNum+2)*GS), cvScalar(0,255,0));		//Painting green
	cvLine(in, cvPoint(GS, GS), cvPoint((XGridNum+1)*GS, GS), line_color, line_thickness);
	cvLine(in, cvPoint(GS, GS), cvPoint(GS, (YGridNum+1)*GS), line_color, line_thickness);
	cvLine(in, cvPoint(GS, (YGridNum+1)*GS), cvPoint((XGridNum+1)*GS, (YGridNum+1)*GS), line_color, line_thickness);
	cvLine(in, cvPoint((XGridNum+1)*GS, GS), cvPoint((XGridNum+1)*GS, (YGridNum+1)*GS), line_color, line_thickness);
	
	cvLine(in, cvPoint((XGridNum/2+1)*GS, GS), cvPoint((XGridNum/2+1)*GS, (YGridNum+1)*GS), line_color, line_thickness);
	
	cvCircle(in, cvPoint((XGridNum/2+1)*GS, (YGridNum/2+1)*GS), (YGridNum/8)*GS, line_color, line_thickness);
	
	cvLine(in, cvPoint(GS, (YGridNum/8+1)*GS), cvPoint((XGridNum/9+1)*GS, (YGridNum/8+1)*GS), line_color, line_thickness);
	cvLine(in, cvPoint((XGridNum/9+1)*GS, (YGridNum*7/8+1)*GS), cvPoint((XGridNum/9+1)*GS, (YGridNum/8+1)*GS), line_color, line_thickness);
	cvLine(in, cvPoint(GS, (YGridNum*7/8+1)*GS), cvPoint((XGridNum/9+1)*GS, (YGridNum*7/8+1)*GS), line_color, line_thickness);

	cvLine(in, cvPoint((XGridNum+1)*GS, (YGridNum/8+1)*GS), cvPoint((XGridNum*8/9+1)*GS, (YGridNum/8+1)*GS), line_color, line_thickness);
	cvLine(in, cvPoint((XGridNum*8/9+1)*GS, (YGridNum*7/8+1)*GS), cvPoint((XGridNum*8/9+1)*GS, (YGridNum/8+1)*GS), line_color, line_thickness);
	cvLine(in, cvPoint((XGridNum+1)*GS, (YGridNum*7/8+1)*GS), cvPoint((XGridNum*8/9+1)*GS, (YGridNum*7/8+1)*GS), line_color, line_thickness);

//	for(int i=1; i< XGridNum+2; i++)
//		cvLine(in, cvPoint(i*10, 10), cvPoint(i*10, (YGridNum+1)*10), cvScalar(55,55,55));
//	
//	for(int j=1; j< YGridNum+2; j++)
//		cvLine(in, cvPoint(10, j*10), cvPoint((XGridNum+1)*10, j*10), cvScalar(55,55,55));
	
}

IplImage* acyut_image;
int doDrawProb(IplImage* in, int GS=10, int (*pos_array)[2] = NULL, int pos_num = 0)
{

	CvScalar ball_color = {0, 140, 255};
	CvScalar high_prob_color = {0, 0, 255};
	CvScalar acyut_color = {200, 200, 200};
	 

	int line_thickness = 2;
	doImageOverlay3C(in, acyut_image, (drawAcyutX+1)*GS , (drawAcyutY+1)*GS, 1);

	int i, j;
	
	for(j=0;j<=YGridNum;j++)	
	{
		for(i=0;i<=XGridNum;i++)
		{

			if((i==drawBallX)&&(j==drawBallY)&&(ballFoundPrev==1))
			{
				cvCircle(in, cvPoint((i+1)*GS, (j+1)*GS), GS, ball_color, line_thickness);
				continue;
			}
			
			if((i==drawAcyutX)&&(j==drawAcyutY))
			{
				
				cvEllipse(in, cvPoint((i+1)*GS, (j+1)*GS), cvSize(20, 1), baseAngle, -90, 90, acyut_color); 
				cvCircle(in, cvPoint((i+1)*GS, (j+1)*GS),  5, cvScalar(255,0,0), 5);
				
			}
			 if(position[1][i][j]>=drawProbHigh*0.9999)
			{
				cvCircle(in, cvPoint((i+1)*GS, (j+1)*GS),  5, cvScalar(0,0,0), 5);
			}
			else if(position[1][i][j]>0.99*drawProbHigh)
			{
//				cvCircle(in, cvPoint((i+1)*GS, (j+1)*GS),(position[0][i][j] - 0.99*drawProbHigh)*8/(0.01*drawProbHigh), high_prob_color);
		//		cvEllipse(in, cvPoint((i+1)*GS, (j+1)*GS), cvSize((position[0][i][j] - 0.99*drawProbHigh)*20/(0.01*drawProbHigh), 1), baseAngle, -90, 90, high_prob_color); 	
			}
			
		}

	}
	
	for(i = 0; i < pos_num; i++)
	{
		cvCircle(in, cvPoint((pos_array[i][0]+1)*GS, (pos_array[i][1]+1)*GS),  3, cvScalar(0,0,255), 5);
	}

}


int doMetricToField(int &x, int &y, float distance, float theta)
{

	x = drawAcyutX - cos(deg2rad((theta+baseAngle)))*distance/(fieldConversion*100);
	y = drawAcyutY - sin(deg2rad((theta+baseAngle)))*distance/(fieldConversion*100);
//	printf("In doMetricToField x=%d y=%d, drawAcyutX=%d, drawAcyutY = %d, theta = %f, bangle = %d\n",x,y,drawAcyutX, drawAcyutY, theta, baseAngle);
	return 1;
}

int doMetricToField_cm(int &x, int &y, float distance, float theta)
{
	doMetricToField(x,y,distance,theta);
	x = x*fieldConversion*100;
	y = (YGridNum-y)*fieldConversion*100;
	return 1;
}


int doShowFieldImage(int GS=10, int (*pos_array)[2] = NULL, int pos_num = 0)
{
	IplImage* fieldImage = cvCreateImage(cvSize((XGridNum+2)*GS, (YGridNum+2)*GS), 8, 3);
	cvZero(fieldImage);
	doDrawField(fieldImage, GS);
	doDrawProb(fieldImage, GS, pos_array, pos_num);
	cvShowImage("Field", fieldImage);
	cvReleaseImage(&fieldImage);
}

//50 = black
//200 = special black
//100 = green
int know_your_enemy(IplImage* coded, IplImage* infoimg,int* obstacles)
{
	// Obstacles is x,y,r
	int nob=0,j,i,x1,x2,y1,y2,t,w=coded->width,h=coded->height;
	for(i=0;i<w;i++)
		for(j=0;j<h-2;j++)
		{
			if( pixelColor1C(coded,i,j)==50)
			//if( chkcolor(infoimg,i,j+2,GREENC) )
			if(pixelColor1C(coded, i, j+1)==100)
			{
				returnPixel1C(coded,i,j)=200;
			}
		}
	//cvShowImage("Coded", coded);
	for(j=0;j<h-2;j++)
		for(i=0;i<w;i++)
		{
			if( pixelColor1C(coded,i,j)==200 )
			{
				x1=i;
				y1=j;
				i++;
				while( (pixelColor1C(coded,i,j)==200 || pixelColor1C(coded,i,j)==50) && i<w-1) i++;
				while( (pixelColor1C(coded,i,j)==200 || pixelColor1C(coded,i,j)==50) && j<h-1) j++;
				x2=i;
				y2=j;
				
				if(y2+x2-x1-y1 > 5)
				{
					cvCircle(frame, cvPoint(((x1+x2)/2 + 0.5)*GRIDSIZE, ((y1+y2)/2+0.5)*GRIDSIZE ), 5, cvScalar(255, 0, 0));
					obstacles[3*nob]=(x1+x2)/2;
					obstacles[3*nob+1]=(y1+y2)/2;
					obstacles[3*nob+2]=(y2+x2-x1-y1)/2;
					nob++;
				}		
			//	printf("Value %d %d %d\n", i, j,y2+x2-x1-y1);
				
			}
		}
	return nob;
}

inline int getDistance_special(int x1, int y1, int x2, int y2)
{
	return (x2-x1)*(x2-x1) + (y2-y1)*(y2-y1);
}

inline int getMin_special(int x1, int x2)
{
	return (x1<x2)? x1 : x2;
}

inline int doSquare_special(int x)
{
	return x*x;
}

int doNegativeAssociation(landmark* l, int *pos[2], int n)
{
	
}
int doAssociation(landmark* l, int lcount, IplImage* in, float distance, int GS=10)
{

	/*relativeField is used to show only landmarks relative to the acyut position (and angle) returned by the complete algorithm*/
	IplImage* relativeField = cvCreateImage(cvSize(GS*XGridNum, GS*XGridNum), 8,3);
	cvZero(relativeField);
	int ax ;
	int ay ;
	int aangle = 0;
	
	float xl, yl;
	int x, y, i, j;
	int c1;
	int param = 0;

/*	
	Projecting every 10 white points from skeletonized image, and storing the actual (in cms) distance and
	angle in white[][] 
	The points are also displayed in IplImage* image with acyut angle 90 degrees (scaled to grid field). Position
	or rotation does not matter in this image, it is just for reference - to see what acyut is seeing and how
	it is projecting it.
	Only those points that are in a certain bounding box are used!
*/
/* 	Code has been moved to where it is actually required	*/
	float tempDist, tempAngle;
	float white[100][2];
	int whitec = 0;

	int skipCount = 0;
	IplImage* image = cvCreateImage(cvSize(200,200),8,1);
	cvZero(image);
	/* CvRect syntax : x, y, width, height */
	CvRect boundingRect = cvRect(0/GRIDSIZE, 0/GRIDSIZE, 640/GRIDSIZE, 320/GRIDSIZE);

/*
	whiteField is the image from which comparisons are made. 
	Need to add all the field lines in the image
*/
	IplImage* whiteField = cvCreateImage(cvSize(XGridNum, YGridNum), 8, 1);
	CvScalar lineColor = {255};
	int line_thickness = 1;
	cvZero(whiteField);

	cvRectangle(whiteField, cvPoint(0,0), cvPoint(XGridNum-1,YGridNum-1), lineColor, line_thickness);
	cvLine(whiteField, cvPoint(XGridNum/2,0), cvPoint(XGridNum/2, YGridNum-1), lineColor,line_thickness);
	cvCircle(whiteField,cvPoint(XGridNum/2 ,YGridNum/2),YGridNum/8,lineColor,line_thickness);
#ifdef LOCALIZE_CHECK	
	cvShowImage("White Field", whiteField);
#endif
/*	Sample landmarks list for testing */

	// l[0].r = 175.55;
	// l[0].theta = -30.0;
	// l[0].type = LAND_YBY;

	// l[1].r = 86.0;
	// l[1].theta = -60.0;
	// l[1].type = LAND_X;

	// lcount = 2;

/*	Getting highest probability points (40 maximum). Goto is used for breaking multiple nested loops.*/
	float high=0;
	for(i=0;i<=XGridNum;i++)
		for(j=0;j<=YGridNum;j++)
			if(position[1][i][j]>high)
			{
				high=position[1][i][j];
			}

	drawProbHigh = high;

	int highlist[40][2];
	int highlist_angle[40];
	int highlist_param[40];
	int highlist_count = 0;
	
	for(i=0;i<=XGridNum;i++)
		for(j=0;j<=YGridNum;j++)
			if(position[1][i][j]>=0.9999*high)
			{
				highlist[highlist_count][0] = i;
				highlist[highlist_count][1] = j;
				highlist_count++;
				if(highlist_count>39)
					goto exit_highlist;

			}
	
	exit_highlist:
	
	printf("High = %f Highlist count = %d \n", high, highlist_count);

/*	Main calculation loops */
/*	
	Iterating through all high probability points as per probDistance
	and calculating params for 360 angles around the point.
	This is (maybe) a way of associating only, except angle is calculated also without referring to just a single landmark.
	Parameter being used currently : (least distance)^2
	NOTE: limits.h has been included for INT_MAX.
*/
	int param_min=INT_MAX;
	for(c1 = 0; c1 < highlist_count; c1++)
	{
		highlist_param[c1] = INT_MAX;	
		for(aangle = 0; aangle < 360; aangle++)
		{
			param = 0;
			for(i=0; i<lcount; i++)
			{
				xl = highlist[c1][0] + (l[i].r/(fieldConversion*100))*cos(deg2rad(aangle - l[i].theta));
				yl = highlist[c1][1] - (l[i].r/(fieldConversion*100))*sin(deg2rad(aangle - l[i].theta));
				switch(l[i].type)
				{
				case LAND_YBY:
					param+= getDistance_special(xl, yl, landmarks.LPcoord[0][0], landmarks.LPcoord[0][1]);
					break;
				case LAND_X:
					param+=getMin_special(getDistance_special(xl, yl,landmarks.Xcoord[0][0], landmarks.Xcoord[0][1]), getDistance_special(xl, yl, landmarks.Xcoord[1][0], landmarks.Xcoord[1][1]));
					break;
				case LAND_GPB:
					param+=getMin_special(getDistance_special(xl, yl,landmarks.GPcoord[2][0], landmarks.Xcoord[2][1]), getDistance_special(xl, yl, landmarks.GPcoord[3][0], landmarks.GPcoord[3][1]));
					break;
				case LAND_GPY:
					param+=getMin_special(getDistance_special(xl, yl,landmarks.GPcoord[0][0], landmarks.Xcoord[0][1]), getDistance_special(xl, yl, landmarks.GPcoord[1][0], landmarks.GPcoord[1][1]));
					break;
				case LAND_BYB:
					param+=getDistance_special(xl, yl, landmarks.LPcoord[1][0], landmarks.LPcoord[1][1]);	
				}
				
			}
			if(param < 0)
				continue;
			if (param < highlist_param[c1])
			{
				highlist_angle[c1] = aangle;
				highlist_param[c1] = param;
			}
		}
		if(highlist_param[c1] < param_min)
		{
			param_min = highlist_param[c1];
		}
	}

	/*
		Checking if there are multiple points still. 
		Not just least param, but those within distance = 2 grids.
		All these points' params are set to 0, and param_min is set to 0 (because further calculation uses these
		to identify least param points)
	*/
	int highcount_white = 0;
	printf("After Landmarks, param_min is %d\n", param_min);
	for (i = 0; i < highlist_count; i++)
	{
		if(highlist_param[i] - param_min <= 0)
		{
			highcount_white++;
			highlist_param[i] = 0;
		}
		else
		{
			highlist_param[i] = INT_MAX;
		}
	}
	param_min = 0;
	printf("Before white, number of points = %d\n", highcount_white);

	/* If highcount is greater than 1, running white thing */
	int param_min2 = INT_MAX;
	if(highcount_white > 1)
	{
	/*	White field calculation moved here 	*/
		for(x=boundingRect.x; x< boundingRect.x + boundingRect.width; x++)
		{
			for(y=boundingRect.y; y< boundingRect.y + boundingRect.height; y++)
			{
				if(pixelColor1C(in, x, y))
				{
					if(skipCount>4)
					{
						skipCount = 0;
					}
					else
					{
						skipCount++;
						continue;
					}
					tempDist = findDistance(x*GRIDSIZE, y*GRIDSIZE, distance);
					tempAngle = findAngle(x*GRIDSIZE, y*GRIDSIZE, tempDist);

					xl = 100 + (tempDist/(fieldConversion*100))*cos(deg2rad(90 -tempAngle));
					yl = 100 - (tempDist/(fieldConversion*100))*sin(deg2rad(90 -tempAngle));
					returnPixel1C(image, (int)xl, (int)yl) = 255;

					white[whitec][0] = tempDist;
					white[whitec][1] = tempAngle;
					whitec++;
				}
			}
		}

		printf("White count is : %d\n", whitec);
	

		for(c1 = 0; c1 < highlist_count; c1++)
		{
			if(highlist_param[c1] != param_min)
				continue;
			highlist_param[c1] = INT_MAX;	
			for(aangle = 0; aangle < 360; aangle++)
			{
				param = 0;
				for(i=0; i<lcount; i++)
				{
					xl = highlist[c1][0] + (l[i].r/(fieldConversion*100))*cos(deg2rad(aangle - l[i].theta));
					yl = highlist[c1][1] - (l[i].r/(fieldConversion*100))*sin(deg2rad(aangle - l[i].theta));
					switch(l[i].type)
					{
					case LAND_YBY:
						param+= getDistance_special(xl, yl, landmarks.LPcoord[0][0], landmarks.LPcoord[0][1]);
						break;
					case LAND_X:
						param+=getMin_special(getDistance_special(xl, yl,landmarks.Xcoord[0][0], landmarks.Xcoord[0][1]), getDistance_special(xl, yl, landmarks.Xcoord[1][0], landmarks.Xcoord[1][1]));
						break;
					case LAND_GPB:
						param+=getMin_special(getDistance_special(xl, yl,landmarks.GPcoord[2][0], landmarks.Xcoord[2][1]), getDistance_special(xl, yl, landmarks.GPcoord[3][0], landmarks.GPcoord[3][1]));
						break;
					case LAND_GPY:
						param+=getMin_special(getDistance_special(xl, yl,landmarks.GPcoord[0][0], landmarks.Xcoord[0][1]), getDistance_special(xl, yl, landmarks.GPcoord[1][0], landmarks.GPcoord[1][1]));
						break;
					case LAND_BYB:
						param+=getDistance_special(xl, yl, landmarks.LPcoord[1][0], landmarks.LPcoord[1][1]);	
					}
					
				}	
				for(i=0; i<whitec; i++)
				{
					xl = highlist[c1][0] + (white[i][0]/(fieldConversion*100))*cos(deg2rad(aangle - white[i][1]));
					yl = highlist[c1][1] - (white[i][0]/(fieldConversion*100))*sin(deg2rad(aangle - white[i][1]));
						
					if((xl>0)&&(yl>0)&&(xl<XGridNum)&&(yl<YGridNum))
					{
						if(pixelColor1C(whiteField, xl, yl)==0)
						{
							param +=1;
						}
						/* 
						Maybe change the 2 increases? ie give them different weights
						1st one is when point is not white but in the field, 2nd one is when it is outside field.
						*/
					}
					else
					{
						param+=1;
					}
				}

				if (param < highlist_param[c1])
				{
					highlist_angle[c1] = aangle;
					highlist_param[c1] = param;
				}
			}
			if(highlist_param[c1] < param_min2)
			{
				param_min2 = highlist_param[c1];
			}
		}
	}
	else
	{
		param_min2 = param_min;
	}

/*
	Printing and storing all the positions with param within 3 of lowest param
	Also calculating average position and angle to assign to drawAcyutX and Y
*/
	int count_after_white = 0;
	drawAcyutX = 0;
	drawAcyutY = 0;
	baseAngle = 0;
	int pos_array[40][2];
	printf ("Acyut Positions: ");
	for(i = 0; i < highlist_count; i++)
	{
		if((highlist_param[i] - param_min2) < 3)
		{
			printf("\n\t\tX = %d, Y = %d, Angle = %d ", highlist[i][0], highlist[i][1], highlist_angle[i]);
			drawAcyutX += highlist[i][0];
			drawAcyutY += highlist[i][1];
			baseAngle += highlist_angle[i]; 
			pos_array[count_after_white][0] = highlist[i][0];
			pos_array[count_after_white][1] = highlist[i][1];
			count_after_white++;
		}
	}
	printf ("\n");
	drawAcyutX = drawAcyutX/count_after_white;
	drawAcyutY = drawAcyutY/count_after_white;
	baseAngle = baseAngle/count_after_white;
			
	/*
		Relative field is plotted here. Offset is there as sometimes angle is incorrect, and acyut might
		see objects behind it, and cause segmentation faults.
	*/
	int offset = 20*GS;
	cvCircle(relativeField,cvPoint(drawAcyutX*GS ,drawAcyutY*GS+offset),2,cvScalar(255,0,255),2);
	for (i=0; i<lcount; i++)
	{
		xl = drawAcyutX + (l[i].r/(fieldConversion*100))*cos(deg2rad(baseAngle - l[i].theta));
		yl = drawAcyutY - (l[i].r/(fieldConversion*100))*sin(deg2rad(baseAngle - l[i].theta));
		cvCircle(relativeField,cvPoint(xl*GS,yl*GS+offset),2,cvScalar(255,255,255),2);
	}
#ifdef LOCALIZE_CHECK
	cvShowImage("Projected Image", image);
#endif
	cvReleaseImage(&image);
#ifdef LOCALIZE_CHECK
	cvShowImage("Relative Field", relativeField);
#endif
	cvReleaseImage(&relativeField);
	cvReleaseImage(&whiteField);
	/*	Plotting field for localize_check	*/
	#ifdef LOCALIZE_CHECK
	doShowFieldImage(15, pos_array, count_after_white); // count_after_white contains no of elements in pos_array
	#endif

}




int foundL=0;				//Keeps track of landmarks found by storing weights. Used as condition in while loop in localize()
	
bool foundLandmarks(IplImage *src, IplImage* infoimg, IplImage* color_stream, float distance)
{
	// Finding LandMarks:
	//Insert enemy

	landmark l[20];
	int lcount = 0;
//	int obstacles[60];
//-	int init_obs_n = cam_in.no_of_obstacles;

//-	cam_in.no_of_obstacles += know_your_enemy(src,infoimg,obstacles);

//	cvShowImage("Main", frame);
	//printf("%d Obstacles found\n", cam_in.no_of_obstacles);


/*	for(int i=0, j=init_obs_n; j<cam_in.no_of_obstacles; i++, j++)
	{
		float obsDist = findDistance(obstacles[3*i],obstacles[3*i+1], distance);
//		printf("R = %d, Theta = %d, Width = %d",obstacles[3*i],obstacles[3*i+1],obstacles[3*i+2]);
		cam_in.o[j].r = obsDist;
		cam_in.o[j].theta = findAngle(obstacles[3*i],obstacles[3*i+1], obsDist);
		cam_in.o[j].wr=obstacles[3*i+2];
	}
*/




	{
		IplImage* bny = cvCreateImage(cvSize(infoimg->width, infoimg->height), infoimg->depth, 1);

		for(int x = 0; x< infoimg->width; x++)
			for(int y = 0; y< infoimg->height; y++)
			{
				if(chkcolor(infoimg, x, y, BLACKC))
					returnPixel1C(bny, x, y) = 255;
				else if(chkcolor(infoimg, x, y, GREENC))
					returnPixel1C(bny, x, y) = 127;
				else
					returnPixel1C(bny, x, y) = 0;
			}		
	
			
		//cvShowImage("Black", bny);

		cvReleaseImage(&bny);
	}
	
	

	doHilditch(src);
	IplImage* largeSrc = cvCreateImage(cvSize(src->width*GRIDSIZE, src->height*GRIDSIZE), src->depth, 1);
	doIncreaseSize(src, largeSrc, GRIDSIZE);
	
	int Xnum, Tnum, arrX[25][2], arrT[25][2];
	int gp[4] = {-1, -1, -1, -1};
	int yby[2] = {-1, -1};
	int byb[2] = {-1, -1};
	doDetectXT(src, arrT, arrX, Tnum, Xnum, 5);
	
	int loopT=0;

//	if(Tnum>0)
//	{
//		for(loopT=0;loopT<Tnum;loopT++)
//		{
//			int loopT1=0;
//			if((abs(arrT[loopT][0]*GRIDSIZE+GRIDSIZE/2-gp[0])+abs(arrT[loopT][1]*GRIDSIZE+GRIDSIZE/2-gp[1])<80)||(abs(arrT[loopT][0]*GRIDSIZE+GRIDSIZE/2-gp[2])+abs(arrT[loopT][1]*GRIDSIZE+GRIDSIZE/2-gp[3])<80))
//			{
//			for(loopT1=loopT;loopT1<Tnum-1;loopT1++)
//				{
//				arrT[loopT][0]=arrT[loopT+1][0];
//				arrT[loopT][1]=arrT[loopT+1][1];
//				}					
//			Tnum--;
//			}
//		}
//	}
	int x, y;

	if(doDetectYBY(infoimg, yby))
	{
		foundL+=3;
		float YBYdist = findDistance(yby[0],yby[1],distance);
		float YBYangle = findAngle(yby[0],yby[1],YBYdist);
//		cam_in.l[cam_in.n_landmarks].r=YBYdist;
//		cam_in.l[cam_in.n_landmarks].type = LAND_YBY;
//		cam_in.l[cam_in.n_landmarks++].theta=findAngle(yby[0],yby[1],YBYdist);

		printf("YBY Distance : %f Angle : %f ", YBYdist, YBYangle);
		l[lcount].r=YBYdist;
		l[lcount].theta = YBYangle;
		l[lcount].type = LAND_YBY;
		lcount++;
		probDistance(YBYdist, LAND_YBY);

		probAngle(yby[0], yby[1], distance, LAND_YBY);
//		printf("YBY X = %d, Y = %d\n", yby[0], yby[1]);
//		cvCircle(largeSrc,cvPoint(yby[0],yby[1]),2,cvScalar(255,255,255),2);
	}
	
	if(doDetectBYB(infoimg, byb))
	{
		foundL+=3;
		
		float BYBdist = findDistance(byb[0],byb[1],distance);
		float BYBangle = findAngle(byb[0],byb[1],BYBdist);
//		cam_in.l[cam_in.n_landmarks].r=BYBdist;
//		cam_in.l[cam_in.n_landmarks].type = LAND_BYB;
//		cam_in.l[cam_in.n_landmarks++].theta=findAngle(byb[0],byb[1],BYBdist);
		printf("BYB Distance : %f  Angle : %f", BYBdist, BYBangle);
		l[lcount].r = BYBdist;
		l[lcount].theta = BYBangle;
		l[lcount].type = LAND_BYB;
		lcount++;
		probDistance(BYBdist, LAND_BYB);

		probAngle(byb[0], byb[1], distance, LAND_BYB);
		printf("BYB X = %d, Y = %d\n", byb[0], byb[1]);
		cvCircle(largeSrc,cvPoint(byb[0],byb[1]),2,cvScalar(200),2);
	}
	
	
//	cvRectangle(largeSrc, cvPoint(318,238), cvPoint(322,242), cvScalar(50), 1);
	
	
	{
		
		IplImage* bny = cvCreateImage(cvSize(infoimg->width, infoimg->height), infoimg->depth, 1);
		for(x = 0; x< infoimg->width; x++)
			for(y = 0; y< infoimg->height; y++)
			{
				if(chkcolor(infoimg, x, y, BLUEC))
					returnPixel1C(bny, x, y) = 255;
				else if(chkcolor(infoimg, x, y, YELLOWC))
					returnPixel1C(bny, x, y) = 55;
				else
					returnPixel1C(bny, x, y) = 0;
			}		
		cvCircle(bny,cvPoint(byb[0],byb[1]),2,cvScalar(200),2);
			
#ifdef LOCALIZE_CHECK
		cvShowImage("Yellow and Blue", bny);
#endif
		cvReleaseImage(&bny);
	}
	
	
	update_gps(infoimg, gp, YELLOWC);
	loopT=0;
	//YELLOW goal post
	printf("GP Yellow 0 1 2 3 are : %d, %d, %d, %d\n", gp[0], gp[1], gp[2], gp[3]);
	if(gp[0]>=0)
	{
		if((yby[0]<0)||(!(xyCompare(gp, yby, 30))))
		{
			foundL+=2;
			
			cvCircle(color_stream,cvPoint(gp[0],gp[1]),2,cvScalar(255, 215, 0),2);
			float GPdist = findDistance(gp[0],gp[1],distance);
			
//			cam_in.l[cam_in.n_landmarks].r=GPdist;
//			cam_in.l[cam_in.n_landmarks].type = LAND_GPY;
//			cam_in.l[cam_in.n_landmarks++].theta=findAngle(gp[0], gp[1],GPdist);
//			printf("GP Yellow 1 Distance : %f Angle : %f\n", GPdist, findAngle(gp[0], gp[1],GPdist));
			float GPangle = findAngle(gp[0],gp[1],GPdist);
			l[lcount].r = GPdist;
			l[lcount].theta = GPangle;
			l[lcount].type = LAND_GPY;
			lcount++;
			probDistance(GPdist, LAND_GPY);
			probAngle(gp[0], gp[1], distance, LAND_GPY);
			
			
		}
		if(gp[2]>=0)
		{
			
			if((yby[0]<0)||(!(xyCompare(gp+2, yby, 30))))
			{
				foundL+=2;
				
				cvCircle(color_stream,cvPoint(gp[2],gp[3]),2,cvScalar(255, 215, 0),2);  
				float GPdist2 = findDistance(gp[2],gp[3],distance);
//				cam_in.l[cam_in.n_landmarks].r=GPdist2;
//				cam_in.l[cam_in.n_landmarks].type = LAND_GPY;
//				cam_in.l[cam_in.n_landmarks++].theta=findAngle(gp[2], gp[3], GPdist2);
//				printf("GP Yellow 2 Distance : %f, Angle : %f\n", GPdist2,findAngle(gp[2], gp[3], GPdist2));
				float GPangle2 = findAngle(gp[2],gp[3],GPdist2);
				l[lcount].r = GPdist2;
				l[lcount].theta = GPangle2;
				l[lcount].type = LAND_GPY;
				lcount++;
				probDistance(GPdist2, LAND_GPY);
				probAngle(gp[2], gp[3],distance, LAND_GPY);
				
			}
		}
	}
	
	for(loopT = 0; loopT < Xnum; loopT++)
	{
		arrX[loopT][0]=arrX[loopT][0]*GRIDSIZE+GRIDSIZE/2;
		arrX[loopT][1]=arrX[loopT][1]*GRIDSIZE+GRIDSIZE/2;
		if(gp[0]>=0)
		if(xyCompare(gp,arrX[loopT],125))
			continue;
		if(gp[2]>=0)
		if(xyCompare(gp+2,arrX[loopT],125))
			continue;
					
		foundL++;
		float Xdist = findDistance(arrX[loopT][0],arrX[loopT][1],distance);
//		cam_in.l[cam_in.n_landmarks].r=Xdist;
//		cam_in.l[cam_in.n_landmarks].type = LAND_X;
//		cam_in.l[cam_in.n_landmarks++].theta=findAngle(arrX[loopT][0],arrX[loopT][1],Xdist);
		float Xangle = findAngle(arrX[loopT][0],arrX[loopT][1],Xdist);
		l[lcount].r = Xdist;
		l[lcount].theta = Xangle;
		l[lcount].type = LAND_X;
		lcount++;
		probDistance(Xdist, LAND_X);
		probAngle(arrX[loopT][0],arrX[loopT][1],distance, LAND_X);
		printf("X %d Distance %f, Angle : %f \tx  = %d, y = %d\n",loopT,Xdist, Xangle,arrX[loopT][0], arrX[loopT][1]);
	}
	
	

	// BLUE goal post
	update_gps(infoimg, gp, BLUEC);
	printf("GP Blue 0 1 2 3 are : %d, %d, %d, %d\n", gp[0], gp[1], gp[2], gp[3]);
	if(gp[0]>=0)
	{
		if((byb[0]<0)||(!(xyCompare(gp, byb, 30))))
		{
			foundL+=2;
			
			cvCircle(color_stream,cvPoint(gp[0],gp[1]),2,cvScalar(0,0,255),2);
			float GPdist = findDistance(gp[0],gp[1],distance);
//			cam_in.l[cam_in.n_landmarks].r=GPdist;
//			cam_in.l[cam_in.n_landmarks].type = LAND_GPB;
//			cam_in.l[cam_in.n_landmarks++].theta=findAngle(gp[0], gp[1],GPdist);
//			printf("GP Blue 1 Distance : %f\n", GPdist);
			float GPangle = findAngle(gp[0],gp[1],GPdist);
			l[lcount].r = GPdist;
			l[lcount].theta = GPangle;
			l[lcount].type = LAND_GPB;
			lcount++;
			probDistance(GPdist, LAND_GPB);
			probAngle(gp[0], gp[1], distance, LAND_GPB);
			
		}
		if(gp[2]>=0)
		{
			
			if((byb[0]<0)||(!(xyCompare(gp+2, byb, 30))))
			{
				foundL+=2;
				
				cvCircle(color_stream,cvPoint(gp[2],gp[3]),2,cvScalar(0,0,255),2);  
				float GPdist2 = findDistance(gp[2],gp[3],distance);
//				cam_in.l[cam_in.n_landmarks].r=GPdist2;
//				cam_in.l[cam_in.n_landmarks].type = LAND_GPB;
//				cam_in.l[cam_in.n_landmarks++].theta=findAngle(gp[2], gp[3],GPdist2);
//				printf("GP Blue 2 Distance : %f\n", GPdist2);
				float GPangle2 = findAngle(gp[2],gp[3],GPdist2);
				l[lcount].r = GPdist2;
				l[lcount].theta = GPangle2;
				l[lcount].type = LAND_GPB;
				lcount++;
				probDistance(GPdist2, LAND_GPB);
				probAngle(gp[2], gp[3],distance, LAND_GPB);
				
			}
		}
	}
	
	cvShowImage("Skeleton", largeSrc);	
	cvReleaseImage(&largeSrc);	
	
	// for(loopT=0;loopT<Tnum;loopT++)		
	// {
	// 	float Tdist=findDistance(arrT[loopT][0]*GRIDSIZE+GRIDSIZE/2,arrT[loopT][1]*GRIDSIZE+GRIDSIZE/2,distance);
	// 	printf("T %d Distance %f %d %d\n",loopT,Tdist,arrT[loopT][0]*GRIDSIZE+GRIDSIZE/2,arrT[loopT][1]*GRIDSIZE+GRIDSIZE/2);
	// 	/*
	// 	//probT and probAngleT are for landmark pole
	// //	probDistance(Tdist, LAND_T);
	// //	probDistance(Tdist, LAND_BYB);	//This is a substitute for a landmark pole
	// //	probT2(Tdist);		
		
	// //	probAngle(arrT[loopT][0]*GRIDSIZE+GRIDSIZE/2,arrT[loopT][1]*GRIDSIZE+GRIDSIZE/2,distance, LAND_T);
		
	// //	probAngleT(arrT[loopT][0]*GRIDSIZE+GRIDSIZE/2,arrT[loopT][1]*GRIDSIZE+GRIDSIZE/2,thetaX,thetaY,distance);
	// 	*/
	// }	

	
	

	
	
	// Evaluating LandMarks:
	if(foundL>0)
	{
		for(int a=0;a<=XGridNum;a++)
			for(int b=0;b<=YGridNum;b++)
				position[1][a][b]*=position[0][a][b];
	}

	doAssociation(l, lcount, src, distance);	

	if(foundL>=1) return 1;
	else return 0;
}


void makeLandmarks()
{
//Initializing LandMarks
//Landmarks changed
	landmarks.Tcoord[0][0]=XGridNum/2;
	landmarks.Tcoord[0][1]=0;
	landmarks.Tcoord[1][0]=0;
	landmarks.Tcoord[1][1]=YGridNum/8;
	landmarks.Tcoord[2][0]=XGridNum;
	landmarks.Tcoord[2][1]=YGridNum/8;
	landmarks.Tcoord[3][0]=0;
	landmarks.Tcoord[3][1]=(7*YGridNum)/8;
	landmarks.Tcoord[4][0]=XGridNum;
	landmarks.Tcoord[4][1]=(7*YGridNum)/8;
	landmarks.Tcoord[5][0]=XGridNum/2;
	landmarks.Tcoord[5][1]=YGridNum;
	
	landmarks.Xcoord[0][0]=XGridNum/2;
	landmarks.Xcoord[0][1]=YGridNum/2-YGridNum/8;
	landmarks.Xcoord[1][0]=XGridNum/2;
	landmarks.Xcoord[1][1]=YGridNum/2+YGridNum/8;

	landmarks.GPcoord[0][0]=0;
	landmarks.GPcoord[0][1]=YGridNum*17/60;
	landmarks.GPcoord[1][0]=0;
	landmarks.GPcoord[1][1]=YGridNum*43/60;
	landmarks.GPcoord[2][0]=XGridNum;
	landmarks.GPcoord[2][1]=YGridNum*17/60;
	landmarks.GPcoord[3][0]=XGridNum;
	landmarks.GPcoord[3][1]=YGridNum*43/60;
	
	landmarks.LPcoord[0][0]=XGridNum/2;
	landmarks.LPcoord[0][1]=-YGridNum/15;
	landmarks.LPcoord[1][0]=XGridNum/2;
	landmarks.LPcoord[1][1]=YGridNum + YGridNum/15;
	
	landmarks.Ccoord[0][0] = XGridNum/2;
	landmarks.Ccoord[0][1] = YGridNum/2;
	printf("Done landmarks\n");
}


void makeVariance()
{
	printf("Variance of distance %f\n",landmarks.Tvar[0]=findVarianceDist(landmarks.Tcoord[0][0],landmarks.Tcoord[0][1]));
	printf("Variance of distance %f\n",landmarks.Tvar[1]=findVarianceDist(landmarks.Tcoord[1][0],landmarks.Tcoord[1][1]));
	printf("Variance of distance %f\n",landmarks.Tvar[2]=findVarianceDist(landmarks.Tcoord[2][0],landmarks.Tcoord[2][1]));
	printf("Variance of distance %f\n",landmarks.Tvar[3]=findVarianceDist(landmarks.Tcoord[3][0],landmarks.Tcoord[3][1]));
	printf("Variance of distance %f\n",landmarks.Tvar[4]=findVarianceDist(landmarks.Tcoord[4][0],landmarks.Tcoord[4][1]));
	printf("Variance of distance %f\n",landmarks.Tvar[5]=findVarianceDist(landmarks.Tcoord[5][0],landmarks.Tcoord[5][1]));

	printf("Variance of X distance %f\n",landmarks.Xvar[0]=findVarianceDist(landmarks.Xcoord[0][0],landmarks.Xcoord[0][1]));
	printf("Variance of X distance %f\n",landmarks.Xvar[1]=findVarianceDist(landmarks.Xcoord[1][0],landmarks.Xcoord[1][1]));
	
//	printf("Variance of C distance %f\n",landmarks.Cvar[0]=findVarianceDist(landmarks.Ccoord[0][0],landmarks.Ccoord[0][1]));
	printf("Variance of LGoal1 Distance %f\n",landmarks.GPvar[0]=findVarianceDist(landmarks.GPcoord[0][0],landmarks.GPcoord[0][1]));
	printf("Variance of LGoal2 Distance %f\n",landmarks.GPvar[1]=findVarianceDist(landmarks.GPcoord[1][0],landmarks.GPcoord[1][1]));
	printf("Variance of RGoal1 Distance %f\n",landmarks.GPvar[2]=findVarianceDist(landmarks.GPcoord[2][0],landmarks.GPcoord[2][1]));
	printf("Variance of RGoal2 Distance %f\n",landmarks.GPvar[3]=findVarianceDist(landmarks.GPcoord[3][0],landmarks.GPcoord[3][1]));
	
	printf("Variance of LPYBY Distance %f\n",landmarks.LPvar[0]=findVarianceDist(landmarks.LPcoord[0][0],landmarks.LPcoord[0][1]));
	printf("Variance of LPBYB Distance %f\n",landmarks.LPvar[1]=findVarianceDist(landmarks.LPcoord[1][0],landmarks.LPcoord[1][1]));
//	printf("Variance of X Angle %f\n",landmarks.AngleXvar[0]=findVarianceAngleX(landmarks.Xcoord[0][0],landmarks.Xcoord[0][1],landmarks.Xcoord[1][0],landmarks.Xcoord[1][1]));
	printf("Variance of T Angle %f\n",landmarks.AngleTvar[0]=findVarianceAngle(landmarks.Tcoord[0][0],landmarks.Tcoord[0][1]));
	printf("Variance of T Angle %f\n",landmarks.AngleTvar[5]=findVarianceAngle(landmarks.Tcoord[5][0],landmarks.Tcoord[5][1]));
	
	printf("Variance of X Angle %f\n",landmarks.AngleXvar[0]=findVarianceAngle(landmarks.Xcoord[0][0],landmarks.Xcoord[0][1]));
	printf("Variance of X Angle %f\n",landmarks.AngleXvar[1]=findVarianceAngle(landmarks.Xcoord[1][0],landmarks.Xcoord[1][1]));
	
	printf("Variance of GPY Angle %f\n",landmarks.AngleGPvar[0]=findVarianceAngle(landmarks.GPcoord[0][0],landmarks.GPcoord[0][1]));
	printf("Variance of GPY Angle %f\n",landmarks.AngleGPvar[1]=findVarianceAngle(landmarks.GPcoord[1][0],landmarks.GPcoord[1][1]));
	printf("Variance of GPB Angle %f\n",landmarks.AngleGPvar[2]=findVarianceAngle(landmarks.GPcoord[2][0],landmarks.GPcoord[2][1]));
	printf("Variance of GPB Angle %f\n",landmarks.AngleGPvar[3]=findVarianceAngle(landmarks.GPcoord[3][0],landmarks.GPcoord[3][1]));
	
	printf("Variance of LPYBY Angle %f\n",landmarks.AngleLPvar[0]=findVarianceAngle(landmarks.LPcoord[0][0],landmarks.LPcoord[0][1]));
	printf("Variance of LPBYB Angle %f\n",landmarks.AngleLPvar[1]=findVarianceAngle(landmarks.LPcoord[1][0],landmarks.LPcoord[1][1]));
	
	
//	landmarks.AngleTvar[5] = landmarks.AngleTvar[0];
	
	

//	landmarks.AngleXvar[0] = 0.001;
	printf("Done variance\n");
}


inline void resetProbTable(int i=0)
{
	int loop1=0,loop2=0;
	for(loop1=0;loop1<=XGridNum;loop1++)
		for(loop2=0;loop2<=YGridNum;loop2++)
			position[i][loop1][loop2]=1;
}

int file_exists(const char* filename)
{
	if(FILE *file = fopen(filename, "r"))
	{
		fclose(file);
		return 1;
	}
	return 0;
}


int localize_init()
{
	makeLandmarks();
	makeVariance();
	
	usleep(800000);
	

	//Declarations and constants
	int i=0;
	if(file_exists("acyut-apogee.jpg"))
		acyut_image= cvLoadImage("acyut-apogee.jpg");
	else
	{
		printf("Acyut image not present! Will not be able to overlay image.\n");
		acyut_image = cvCreateImage(cvSize(1,1),8,3);
		returnPixel3C(acyut_image, 0, 0, RED) = 0;
		returnPixel3C(acyut_image, 0, 0, BLUE) = 0;
		returnPixel3C(acyut_image, 0, 0, GREEN) = 255;
	}
	capture = cvCaptureFromCAM(300);
	
	while(!cvGrabFrame(capture))
		printf("No Image\n");
	/* Multiple captures were put here because it was though that there is only
	   an initial lag. However, this is not the case (although the problem still isn't
	   properly understood). Still, multiple initial captures remain.
	*/
	for(int i=0; i< 4; i++)		//Change MULTIPLE CAPTURES INIT here
		cvGrabFrame(capture);
		
	camera_stream= cvRetrieveFrame(capture);
	image_size=cvGetSize(camera_stream);
	color_stream=cvCreateImage(image_size,camera_stream->depth,3);
	frame = cvCreateImage(image_size, color_stream->depth, color_stream->nChannels);
	yuv = cvCreateImage(image_size,color_stream->depth,3);
	infoimg=cvCreateImage(image_size,8,1);

	printf("Making lookup table\n");
	lut_marker = marker_create_lut();
	printf("Lookup table completed.\n");
	tempAngle = imucalBase();
	if(tempAngle==ERROR_IMU)
	{
		printf("ERROR READING IMU");
	}
	else
	{
		initAngle = tempAngle;
	}
	
	morphKernel = cvCreateStructuringElementEx(2, 2, 1, 1, CV_SHAPE_RECT, NULL);
	
}

inline int doReadIMU()
{
		tempAngle = imucalBase();
		if(tempAngle==ERROR_IMU)
			return ERROR_IMU;
		else
			baseAngle = tempAngle;
	//	printf("Base Angle Prev= %d\n", baseAngle);

//		baseAngle=360-baseAngle;
//		baseAngle-=21;			//Change offset here
//		baseAngle = 180 - baseAngle;
//		baseAngle+=15;
//		if(baseAngle<90)
//		{
//			double x = (double)baseAngle;
//			x = -(61.0*x*x*x*x - 28309.0*x*x*x + 3585276.0*x*x - 243673884.0*x + 4154159520.0)/59675616.0;
//			baseAngle = x;
//		}
//		//baseAngle = 35;
		if(baseAngle>210&&baseAngle<280)
			baseAngle=90+2.8125*(baseAngle-246);
//		printf("Base Angle = %d\n", baseAngle);

}

inline int doReadMotors()
{
	if(read_motor(thetaX, thetaY)==0)
	{
		printf("Unable to read motors");
		return ERROR_MOTORS;
	}
//	thetaX-=2.375;		//Correction added for incorrect tilt of base. 
				//Will have to take IMU values later on to accurately define thetaX.
//	printf("thetaX in degrees.........................%fthetaY in degrees.............%f\n", thetaX, thetaY);
	thetaX=deg2rad(thetaX);
	thetaY=deg2rad(thetaY);	
	return 1;
}

inline int doGetImage(int ntime = 2)		
{
	if(!cvGrabFrame(capture))
		return ERROR_GET_IMAGE;
//	printf("No Image\n");
	for(i=1; i<ntime; i++)	/* Multiple grabbing is there to remove (reduce) frame lag. */	
		cvGrabFrame(capture);	
					
	return 0;
}

// inline int doMakeInfoImage()
// {
// 	cvCvtColor(camera_stream,color_stream, CV_BayerBG2BGR);
// 	cvConvertScale(color_stream, frame, 1, 0);
// 	infoimg=marker(color_stream);
// }

inline int doMakeInfoImage(uchar* lut = lut_marker)
{
	cvCvtColor(camera_stream,color_stream, CV_BayerBG2BGR);
	cvConvertScale(color_stream, frame, 1, 0);
	marker_lut(color_stream,lut);
}

// inline int doMakeInfoImage_ball()
// {
// 	cvCvtColor(camera_stream,color_stream, CV_BayerBG2BGR);
// 	cvConvertScale(color_stream, frame, 1, 0);
// 	infoimg=marker_ball(color_stream);
// }

//Add some error *checking* in following function, especially purple img
// int prelocalize(int ntime = 4)
// {
// //		doReadIMU();
// 		doReadMotors();
// 		doGetImage(ntime);
// 		doMakeInfoImage();
// 		resetProbTable(0);		
// }

int prelocalize(int ntime = 4)
{
	doReadMotors();
	doGetImage(ntime);
	doMakeInfoImage();
	resetProbTable(0);
}

bool dolocalize()
{
		bool localized = 0;
		float height=s_height+(neck_len/sin(thetaX));
			
		float distance=height*tan(thetaX);
//		printf("Center Distance: %f\n",distance);
//	
		segmentated = doWhiteFilter(infoimg, GRIDSIZE, morphKernel);
		IplImage* largeSeg = cvCreateImage(cvSize(segmentated->width*GRIDSIZE, segmentated->height*GRIDSIZE), segmentated->depth, 1);
		doIncreaseSize(segmentated, largeSeg, GRIDSIZE);
		//cvShowImage("Segmentated", largeSeg);
		//	printf("Bottom distance: %f\n",findDistance(320,480,distance,thetaX));
		cvShowImage("Main", frame);
		if(foundLandmarks(segmentated,infoimg,frame, distance))
		{
			
			localized=1;
		}
		
		cvReleaseImage(&segmentated);
		cvReleaseImage(&largeSeg);
		//~ cvReleaseImage(&infoimg);
		
		return localized;
}

#ifndef LOCALIZE_CHECK
int doUpdateCameraInput(int x)
{

	cam_in.oc.theta = 180 - baseAngle;
	cam_in.c.theta =180 - baseAngle;


	cam_in.oc.x = drawAcyutX*fieldConversion*100;
	cam_in.oc.y = (YGridNum-drawAcyutY)*fieldConversion*100;

	if(x!=5)
	{

//Add weighted mean of motion model to get cam_in.c as well as drawAcyut//
		printf("Cam got %f %f as motion_coords\n",Command.motion_coords.abs_x,Command.motion_coords.abs_y);
		printf("Pure localisation in grid %d %d\n",drawAcyutX,drawAcyutY);
		if(Command.motion_coords.abs_y>-1)
		{
			cam_in.c.x = (drawAcyutX*fieldConversion*100 + Command.motion_coords.abs_x)/2;
			cam_in.c.y = ((YGridNum-drawAcyutY)*fieldConversion*100+ Command.motion_coords.abs_y)/2;
		}
		else
		{
			cam_in.c.x = drawAcyutX*fieldConversion*100;
			cam_in.c.y = (YGridNum-drawAcyutY)*fieldConversion*100;
		}
		cam_in.f.localized = 1 - drawProbHigh;
		drawAcyutX = cam_in.c.x/(fieldConversion*100);
		drawAcyutY = YGridNum - (cam_in.c.y/(fieldConversion*100));

	}
	else
	{
		
//Only motion model input to get cam_in.c as well as drawAcyut
		cam_in.c.x=Command.motion_coords.abs_x;
		cam_in.c.y=Command.motion_coords.abs_y;
		drawAcyutX = cam_in.c.x/(fieldConversion*100);
		drawAcyutY = YGridNum - (cam_in.c.y/(fieldConversion*100));
		cam_in.f.localized = 0;
	}


	doMetricToField(drawBallX, drawBallY, ballDistance, ballAngleDeg);
	doMetricToField_cm(cam_in.b.x, cam_in.b.y, ballDistance, ballAngleDeg);
	
	for(int i=0; i< cam_in.no_of_obstacles; i++)
	{
		doMetricToField_cm(cam_in.o[i].x, cam_in.o[i].y, cam_in.o[i].r, cam_in.o[i].theta);
	}

	for(int i=0; i< cam_in.n_landmarks; i++)
	{
		doMetricToField_cm(cam_in.l[i].x, cam_in.l[i].y, cam_in.l[i].r, cam_in.l[i].theta);
	}

	
}
#endif

bool localize()
{
	int x=0;
	cam_in.n_landmarks=0;
	resetProbTable(1);
	speed_motor(MOTOR_SPEED, 700);
	foundL = 0;

	prelocalize(7);
	dolocalize();
	
	cvWaitKey(10);
	if(foundL<3)
	{
		go2state(0);
		sleep(1);
	}

	while( (foundL<3) && (x<5) )
	{
		prelocalize(7);
		dolocalize();
		if(foundL>3)
		{
			break;
		}
		go2state(++x);
		if ((cvWaitKey(60000/MOTOR_SPEED + 500)&0xff)==27)
			break;
		
	}
	printf("Localization finis\n");
#ifndef LOCALIZE_CHECK
	pthread_mutex_lock(&mutex_Command);
	pthread_mutex_lock(&mutex_cam_in);

	
	doUpdateCameraInput(x);

	pthread_mutex_unlock(&mutex_cam_in);
	pthread_mutex_unlock(&mutex_Command);
#endif
	//doShowFieldImage(15);
		
}
