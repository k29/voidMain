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
#include <termios.h>
#include <sys/ioctl.h>
#include <sys/types.h>

#define FT4232 "FTSC152E"//"FTSBY5CA"//"FTSC152E"
#define NOM 28                                  //Current number of Motors
#define LEN_SYNC_WRITE ((NOM*3)+4) 		//Total Lenght Of Packet
#define INST_SYNC_WRITE     0x83
#define SMOOTHSTEP(x) ((x) * (x)*(3-2*(x)))

//#define acyut 2  //acyut using...........................................change here..............

#define MAX_BOUNDARIES 160
#define TRUE 1
#define serialbluetooth "A8008c3o"    //"A6007WeE"  
#define serialusb2d "A600cBa7" //"A900fC5U"//nd USB2D: A7003N1d  A7003NDp	 A7005LC8	A600cURy  A700eST2

typedef uint8_t byte;
byte ignore_ids[]={10,15,16,17,18};  // enter ids to ignore :)
float q[5];
float output[20];
///////////AKASH IMU//////////////////////////
float avg_angle[3]={0};
int imu_init=0;
float base_angle[3]={0};

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
 
#define max( a, b ) ( ((a) > (b)) ? (a) : (b) )
#define min( a, b ) ( ((a) < (b)) ? (a) : (b) )
 
struct ftdi_context imu;

int flag_imu=1,temp_x;
#define serialimu "A800ctBf" 
 
int gflag=0;
 
FILE *f;
FILE *f1;
int fd,fb;
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
int element,noat,acyut,offset_status=0;
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
///////////////////////////

void reset();
void check();
void loads();
float* generateik(float x,float y,float z,float z_ang);
float* generatespline(int gn,float gx[],float gy[]);
int scurve(int in,int fi,float t, float tot);

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
  float precision = 0.00001;
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




void send_sync()
{ 
	ftdi_write_data(&ftdic1,tx_packet, tx_packet[3]+4);
}



void close_files()
{
	int ret;
    if ((ret=ftdi_usb_close(&ftdic1)) < 0)
    {
        fprintf(stderr, "unable to close usb2d ftdi device: %d (%s)\n", ret, ftdi_get_error_string(&ftdic1));
    }
    else ftdi_deinit(&ftdic1);
//	free(&ftdic1);
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


/*
char *findall()
{
    int ret, i;
    struct ftdi_context ftdic;
    struct ftdi_device_list *devlist, *curdev;
    char manufacturer[128], description[128],serialname[128];

    if (ftdi_init(&ftdic) < 0)
    {
        fprintf(stderr, "ftdi_init failed\n");
        return EXIT_FAILURE;
    }

    if ((ret = ftdi_usb_find_all(&ftdic, &devlist, 0x0403, 0x6001)) < 0)
    {
        fprintf(stderr, "ftdi_usb_find_all failed: %d (%s)\n", ret, ftdi_get_error_string(&ftdic));
        return EXIT_FAILURE;
    }

    printf("Number of FTDI devices found: %d\n", ret);

    i = 0;
    for (curdev = devlist; curdev != NULL; i++)
    {
        printf("Checking device: %d\n", i);
        if ((ret = ftdi_usb_get_strings(&ftdic, curdev->dev, manufacturer, 128, description, 128,serial1,128)) < 0)
        {
            fprintf(stderr, "ftdi_usb_get_strings failed: %d (%s)\n", ret, ftdi_get_error_string(&ftdic));
            return EXIT_FAILURE;
        }
        printf("Manufacturer: %s\nDescription: %s\nSerial: %s\n", manufacturer, description,serialname);
        curdev = curdev->next;
    }

    ftdi_list_free(&devlist);
    ftdi_deinit(&ftdic);
	
	return serialname;
}
*/

int bootup_files()
{
    initialize_ids();
	int chk_i_ids;
	/*for (chk_i_ids = 0; chk_i_ids < 24; chk_i_ids += 1)
	{
		printf("bid[%d]= %d\n",chk_i_ids,bid[chk_i_ids]);
	}*/
	
	
    int ret;
    if (ftdi_init(&ftdic1) < 0)
    {
        fprintf(stderr, "ftdi_init failed\n");
        return EXIT_FAILURE;
    }
    
    if ((ret = ftdi_usb_open_desc (&ftdic1, 0x0403, 0x6001, NULL, serialusb2d)) < 0)
    {
        fprintf(stderr, "unable to open ftdi device: %d (%s)\n", ret, ftdi_get_error_string(&ftdic1));
        printf("unable to open USB2D");
        return EXIT_FAILURE;
    }
    else
    printf("USB2D ftdi successfully open\n");

    ftdi_set_baudrate(&ftdic1,1000000);
	
	int ret1;
    if (ftdi_init(&imu) < 0)
    {
        fprintf(stderr, "ftdi_init failed\n");
        return EXIT_FAILURE;
    }
   
    if ((ret1= ftdi_usb_open_desc (&imu, 0x0403, 0x6001, NULL, serialimu)) < 0)
    {
        fprintf(stderr, "unable to open ftdi device: %d (%s)\n", ret1, ftdi_get_error_string(&imu));
        printf("unable to open IMU");
        return EXIT_FAILURE;
    }

    ftdi_set_baudrate(&imu,57600);

	
	
	
	
	
/*	struct termios options;
	struct timeval start, end;
	long mtime, seconds, useconds;
	if((fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY))==-1)
	{	printf("Serial Port unavailable\n");
		close_files();
		exit(0);
	}
 	tcgetattr(fd, &options);
  	cfsetispeed(&options, B1000000);
  	cfsetospeed(&options, B1000000);
  	options.c_cflag |= (CLOCAL | CREAD);
  	options.c_cflag &= ~PARENB;
  	options.c_cflag &= ~CSTOPB;
     	options.c_cflag &= ~CSIZE;
     	options.c_cflag |= CS8;
     	options.c_oflag |= OPOST;
     	options.c_oflag |= ONOCR;
     	options.c_oflag |= FF0;
     	options.c_oflag &= ~FF1;
     	options.c_oflag &= ~ONLCR;

	options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);    
    	options.c_cflag &= ~CRTSCTS;
//	tcflush(fd, TCIFLUSH);
  	tcsetattr(fd, TCSANOW, &options);
  	fcntl(fd, F_SETFL, FNDELAY);r
	byte packet[8]={0xff,0xff,0,0x04,0x02,0x1e,0x02,0};
	byte *bufptr,*packptr;
	byte pack[2]={0},p,n,n1;
	byte buffer[127];
	int gpos,lemp,lenp;
	int t=0,counter=1;
	char temp[20];

	//byte read[];

*/
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
int sync_write_i,i;
switch(a)
{
	case 0:
	tx_packet[5]=0x20;
	    for(sync_write_i=0;sync_write_i<NOM;sync_write_i++)
	    {
	      tx_packet[7+(sync_write_i*3)]= bid[sync_write_i];
	      tx_packet[8+(sync_write_i*3)]=0x20;
	      tx_packet[9+(sync_write_i*3)]=0x00;
	    }
	
	    tx_packet[3*NOM+7]=checksum();
	//	printf("presendsync\n");
for(i=0;i<tx_packet[3]+4;i++)
printf("%x\t",tx_packet[i]);
	    send_sync();		
	    printf("\n200 Speed Written\n");
		break;
	case 1:
		tx_packet[5]=0x20;
	    for(sync_write_i=0;sync_write_i<NOM;sync_write_i++)
	    {	
	      tx_packet[7+(sync_write_i*3)]= bid[sync_write_i];
	      tx_packet[8+(sync_write_i*3)]=0x00;
	      tx_packet[9+(sync_write_i*3)]=0x00;
	    }
	
	   tx_packet[3*NOM+7]=checksum();
for(i=0;i<tx_packet[3]+4;i++)
printf("%x\t",tx_packet[i]);
	    send_sync();		
	    printf("\n0 Speed Written");
		    break;
}
//tx_packet[128]={0xff,0xff,0xfe,LEN_SYNC_WRITE,INST_SYNC_WRITE,0x1e,0x02};
tx_packet[2]=0xfe;
tx_packet[3]=LEN_SYNC_WRITE;
tx_packet[4]=INST_SYNC_WRITE;
tx_packet[5]=0x1e;
tx_packet[6]=0x02;
}


// .........................................imu graphing functions..............................................................


void graph(float boundaries[80][3],int size)
{
		int axis=2;   // no of graphs (for diff axes)
		int dfac=0.6989;   // decimal precision wanted
		
		int gr_base=400;  //no of rows used for graph
		int aftr_frames2=4;   // no of values to be prodused from 1 given value
		int nof2=size/(aftr_frames2);  // no of points for which value available
		char grph[MAX_BOUNDARIES*aftr_frames2][gr_base*2 + 1];
		
			
		//float boundaries[axis][nof2];  // remove if boundary declared
		int b,c;
		int x=0;
		/*for(b=0;b<axis;b++)
		{
			x=0;
			for(c=0;c<nof2*aftr_frames2;c++)
			{
				boundaries[c][b]=-x;  // eneter function here 
				x++;
			}	
		}
		*/
		
		int ang,i,j,k;
		for(ang=0;ang<axis;ang++)
		{
			memset(grph,' ',sizeof(grph));
			grph[MAX_BOUNDARIES*aftr_frames2][gr_base*2]=' ';
			int u;
			for( u=0;u<MAX_BOUNDARIES*aftr_frames2;u++)
			{
				grph[u][gr_base]='-'	;
			}
			printf("GRAPH %d",ang);
			printf("Y=NO. OF AST x %f || X= NO. OF DASH x%f",pow(10,-1*dfac),((float)5/(float)aftr_frames2));
			for (i=0;i<size;i++)
			{
			
				float point_boundary=boundaries[i/aftr_frames2][ang]+((boundaries[i/aftr_frames2+1][ang]-boundaries[i/aftr_frames2][ang])/aftr_frames2)*(i%	aftr_frames2);  // interpolation 
				int g;
				if(point_boundary<=gr_base/pow(10,dfac) && point_boundary>0)
				{
					for(g=1;g<(int)(point_boundary*pow(10,dfac));g++)
					{
						grph[i][g+gr_base]='*';
					}
				}
				else if(point_boundary>=-1*gr_base/pow(10,dfac) && point_boundary<0)
				{
					for(g=(int)(point_boundary*pow(10,dfac));g<0;g++)
					{
						grph[i][g+gr_base]='*';
					}
				}
			}
			/*printf(" PLOTTED\n");	
			for(u=0;u<=206;u++)printf("-");
			printf("\n");
			for(j=gr_base*2;j>=0;j--)
			{
				for(k=0; k<MAX_BOUNDARIES*aftr_frames2-1;k++)
				{
					printf("%c",grph[k][j]);
				}
				puts("");
			}
			for(u=0;u<=206;u++)printf("-");
			printf("\n");*/
			
				
			
		}
	}
					
void graph_gnuplot(float grp_boundaries[80][3],int size, char * move, int caller)
{
		int axis=2;   // no of graphs (for diff axes)
		int nof2=MAX_BOUNDARIES;  // no of points for which value available
		int aftr_frames2 =4;   // no of values to be prodused from 1 given value
	
		int ang,i;
		for(ang=0;ang<axis;ang++)
		{
			
			char name[50]="\0";
			if(acyut==3)
			strcat(name,"acyut3/imu/");
			else if(acyut==4)
			strcat(name,"acyut4/imu/");
			
			if(caller==0)
				strcat(name,"calibration/");
			else
				strcat(name,"graphs/");
			
			strcat(name,move);
			if(ang==0)
				strcat(name,"_x");
			else if(ang==1)
				strcat(name,"_y");
			else
				strcat(name,"_z");
			
			FILE *f1;
			f1=fopen(name,"w");
			if(f1!=NULL)
			{
				for (i=0;i<size;i++)
				{
					float point_boundary=grp_boundaries[i/aftr_frames2][ang]+((grp_boundaries[i/aftr_frames2+1][ang]-grp_boundaries[i/aftr_frames2][ang])/aftr_frames2)*(i%	aftr_frames2);  // interpolation 
					fprintf(f1,"%d\t%f\n",(i+1),point_boundary);				
				}
				fclose(f1);
			}
			else 
			printf("FILE NOT OPENING : %s",name);
		}
}

	
//...........................................................imu calibrating functions........................................	
void read_bounds(char* move)
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
			printf("%lf\t %lf\n",bounds[i].x,boundaries[i][0]);
			printf("%lf\t %lf\n",bounds[i].y,boundaries[i][1]);
		}	
	}
	else
		printf("ERROR: File not found");
	
	fclose(fr);
	//sleep(10);
}
	
void write_bounds(char* move)
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
		}
		
		fwrite(&bounds,sizeof(bounds),1,fw);	
		printf("New Calibration file made/updated : %s \n",move);
	}
	else
	printf("File Not Found\n");	
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


/*void imucal_init()
{
	printf("entered imu cal init\n");
	//sleep(2);
	byte im[8],i,j,k=0,a=3;
//	byte buffer[59];
	float ang[3]={0};
	byte flag=0,imu_no_of_times=0;
	int start=0;
	base_angle[0]=base_angle[1]=base_angle[2]=0;
	
	while(imu_no_of_times<6)
	{
		flag=0;
	    while(flag==0)
		{   
			if(ftdi_read_data(&imu,&im[0],1)>0)
			{
	//			printf("first ff detected....\n");
				if(im[0]==0xff && start==0)
				{
					start=1;
				}
				else
				{
					if(im[0]==0xff && start==1)
					{	i=0;
	
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
 		for(k=0; k<3; k++)
		base_angle[k]=base_angle[k]+ang[k]/6;
		imu_no_of_times++;
	}
		
		/*for(k=0;k<3;k++)		
		{
		printf("BASE ANGLE : %d is %lf",k,base_angle[k]);
		}
		*//*

	imu_init=1;
	
	
	sleep(1);

}
*/

void imucal_init()
{
	printf("entered imu cal init\n");
	//sleep(2);
	byte im[8],i,j,k=0,a=3;
//	byte buffer[59];
	float ang[3]={0};
	byte flag=0,imu_no_of_times=0,imu_noat=0;
	int start=0;
	base_angle[0]=base_angle[1]=base_angle[2]=0;
	while(imu_no_of_times<6)
	{
		imu_noat=0;
		while(flag<=imu_no_of_times&&imu_noat<10)
		{   
			if(ftdi_read_data(&imu,&im[0],1)>0)
			{
		//		printf("first ff detected....\n");
				if(im[0]==0xff)
				{
					start++;
				}
				else
				start=0;
				
				if(start==2)
				{	
					start=0;
					i=0;
					if(ftdi_read_data(&imu,&im[i],8)>0 && im[6]==254 && im[7]==254)
					{	
						flag++;
						for(j=0,k=0;j<6;j+=2,k+=1)
						{
							ang[k]=im[j]+im[j+1]*256;
							ang[k]=(ang[k] - 0) * (180 + 180) / (720 - 0) + -180;
							base_angle[k]+=ang[k];
						}
						printf("FLAG : %d BASE_ANG 0: %f BASE_ANG 1: %f BASE_ANG 2: %f\n",flag,base_angle[0],base_angle[1],base_angle[2]);	
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
	sleep(1);

}




inline void imucal()
{
	byte im[10],i,j,k=0,a=3,fallen=0;
//	byte buffer[59];
	float ang[3]={0};
	byte flag=0,imu_no_of_times=0;
	avg_angle[0]=avg_angle[1]=avg_angle[2]=0;
	int start=0;
	
	ftdi_usb_purge_buffers(&imu);
		
	while(imu_no_of_times<1 && imu_init==1)
	{		
		flag=0;
		while(flag==0)
		{   
			if(ftdi_read_data(&imu,&im[0],1)>0);
			{
				if(im[0]==0xff)
				start++;
				else
				start=0;
				
				if(start==2)
				{
					start=0;
					printf("PACKET RECIEVED\n");
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
							//ang[k]=base_angle[k]-ang[k];
							//printf("ang %d is %lf and base ang is %lf\n",k, ang[k],base_angle[k]);
						}
					
					}	
				}
			}
		}
	
		for(k=0;k<3;k++)		
		{
			avg_angle[k]=avg_angle[k]+ang[k];
			printf("ANGLE %d : %lf     ",k,avg_angle[k]);
		}
		printf("\n");
		imu_no_of_times++;
	}
	//return fallen;

}




//-----------------------------------------------------------imu functions end----------------------------------------------------------------

void calibrate_boundaries(char* s)
{
	char s1[24];
	if(acyut==3)
	{
		strcpy(s1,"acyut3/moves/");
	}
	else if(acyut==4)
	{
		strcpy(s1,"acyut4/moves/");
	}
	strcat(s1,s);
	printf("Name :%s\n",s1);
	f=fopen(s1,"r");
	if(f==NULL)
	{
		printf("Error opening file.....\n");
		return EXIT_FAILURE;
	}
	int gpos;
	int readfiles_i,j;
	byte buf[2];
	move r;
	fread(&r,sizeof(move),1,f);
	byte checksum;
	frame temp;
//	fread(buf,1,2,f);
	


	//memset(boundaries,0,sizeof(boundaries));

//------------------------------------------------------------------------------------------imu declarations--------------------------------------------------------------------------------------
	byte im[8],imu_i,k=0,a=3;
	byte buffer[59];
	float ang[3]={0},v=0;
	float init_ang[3]={0};	
	int aftr_frames=4;
	float change[3]={0},actual_change[3]={0};
	int flag_imu_first=0,flag_imu_read=0;
	int checksum_change=0,checksum_new=0, change_enuf[3]={0};	
	int imu_gpos=0,imu_gpos1=0,imu_gpos2=0,imu_gpos3=0,imu_gpos4=0;
	int imu_gpos_init=0,imu_gpos1_init=0,imu_gpos2_init=0,imu_gpos3_init=0,imu_gpos4_init=0;
//----------------------------------------------------------------------------------------imu declarations end------------------------------------------------------------------------------------




	int nof=0;
//	if(r.header[0]==0xff && r.header[1]==0xff)
	{	

	
			if(imu_init==0)
				imucal_init();
			 	
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
	


	

//			if(flag_imu_c==1 && nof%aftr_frames==1)
			if(nof%aftr_frames==0)
			{
				ftdi_usb_purge_buffers(&imu);
				flag_imu_read=0;
				checksum_change=0;
				change[0]=change[1]=change[2]=0;
				change_enuf[0]=change_enuf[1]=change_enuf[2]=0;
				
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
		
				/*for(k=0;k<3;k++)		
				{
					avg_angle[k]=avg_angle[k]+ang[k];
					printf("avg angle %d is %lf     ",k,avg_angle[k]);
				}
				*/
				printf("\n");
				imu_no_of_times++;
				}	
					
				
				
				
				//imucal();
				
				for(k=0; k<3 ; k++)
				{
					avg_angle[k]=avg_angle[k]+ang[k];
					printf("avg angle %d is %lf     ",k,avg_angle[k]);
					//change[k]=avg_angle[k];
					//if((change[k]>5 || change[k]<-5) && (change[k]<100 || change[k]>-100))		// threshold changes to apply....
					/*
					if((change[k]<100 || change[k]>-100))
					{
						change_enuf[k]=1;
					}
					change[k]=change[k]*(4096/300);	
					*/
					boundaries[nof/aftr_frames][k]=avg_angle[k];
				}
			//	printf("change enuf is : %d as change is :%lf\n",change_enuf[0], change[0]);		
			//	printf("change enuf is : %d as change is :%lf\n",change_enuf[1], change[1]);		
			/*
				change[0]=(change[0]*4)/4;			// to change x axis total change...............change here......................
				change[1]=(change[1]*6)/4;			// to change y axis total change...............change here......................	
			*/
			}
			checksum_change=0;
			
			for(j=0;j<r.frames[nof].num_motors;j++)
			{
				tx_packet[j*3+7]=r.frames[nof].motorvalues[j].ID;				
				tx_packet[j*3+8]=r.frames[nof].motorvalues[j].LB;
				tx_packet[j*3+9]=r.frames[nof].motorvalues[j].HB;	
					
			}
			
			checksum=255-checksum;
		
			checksum-=r.frames[nof].num_motors+r.frames[nof].address;
			checksum+=tx_packet[2]+tx_packet[3]+tx_packet[4]+tx_packet[5]+tx_packet[6];
			checksum_new=checksum;
			//checksum_new+=checksum_change;
			printf("checksum change is %d and old checksum was %d new checksum is %d\n", checksum_change, checksum, checksum_new);
			tx_packet[7+NOM*3]=(255-checksum_new);

			
		printf("\nFrame : %d\n",nof+1);
		
	/*	for(j=0;j<r.frames[nof].num_motors;j++)
		{
		if(r.frames[nof].address==0x1c||r.frames[nof].address==0x1a)
		printf("ID : %d		Value : %d\n",r.frames[nof].motorvalues[j].ID,r.frames[nof].motorvalues[j].LB);
		else
		printf("ID : %d		Value : %d\n",r.frames[nof].motorvalues[j].ID,r.frames[nof].motorvalues[j].LB+r.frames[nof].motorvalues[j].HB*256);
		}
	*/	
		ftdi_write_data(&ftdic1,tx_packet, tx_packet[3]+4);
		}
	}
	if(f!=NULL)
	fclose(f);
	
	
	boundaries_size=r.noframes;

	
	/*printf("BOUNDARIES:- \n");
	for(j=0; j<boundaries_size/aftr_frames; j++)
	{
		for(k=0;k<3;k++)
			printf("boundary %d is %lf\t",k,boundaries[j][k]);
		printf("\n");
	}
	printf("CALLING GRAPH FILE MAKER\n 	");*/
	printf("MOVE CALIBRATED\n");
	graph_gnuplot(boundaries,boundaries_size,s,0);
	printf("GNUPLOT DONE\n");
	write_bounds(s);

}





void readfiles(char* s)
{
	
	char s1[60];
	if(acyut==3)
	{
		if(offset_status==1)
			strcpy(s1,"acyut3/master_offseted_moves/");
		else
			strcpy(s1,"acyut3/moves/");
	}
	else
	{
		if(offset_status==1)
			strcpy(s1,"acyut4/master_offseted_moves/");
		else
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
			if(nof%aftr_frames==0)
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
		
				/*for(k=0;k<3;k++)		
				{
					avg_angle[k]=avg_angle[k]+ang[k];
					printf("avg angle %d is %lf     ",k,avg_angle[k]);
				}
				*/
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
					present_boundary[k]=avg_angle[k];
				
				change[k]=avg_angle[k]-present_boundary[k];
				local_boundaries[nof/aftr_frames][k]=avg_angle[k];
				
				printf("\npresent boundary  %d is : %lf \t",k, present_boundary[k]);
				if(( change[k] > (1) || change[k]< (-1) ) && ( change[k] < (200) || change[k]> (-200)))				// threshold changes to apply....
				{
					change_enuf[k]=1;
					printf("Sufficient change in %d and change is: %lf and avg angle is %lf\n",k, change[k], avg_angle[k]);
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
		ftdi_write_data(&ftdic1,tx_packet, tx_packet[3]+4);
		}

	}
	if(f!=NULL)
	fclose(f);
	graph_gnuplot(local_boundaries,local_boundaries_size,s,1);
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



void calibration()
{
	calibrate_boundaries("walk1");
	calibrate_boundaries("walk2");	
	calibrate_boundaries("walk3");
	calibrate_boundaries("walk4");	

}




void longwalk()
{
	readfiles("walk1");
	readfiles("walk2");
	readfiles("walk3");
	readfiles("walk2");
	readfiles("walk3");
	readfiles("walk2");
	readfiles("walk3");
	readfiles("walk2");
	readfiles("walk3");
	readfiles("walk2");
	readfiles("walk3");
	readfiles("walk2");
	readfiles("walk3");
	readfiles("walk2");
	readfiles("walk3");
	readfiles("walk2");
	readfiles("walk3");
	readfiles("walk2");
	readfiles("walk3");
	readfiles("walk2");
	readfiles("walk3");
	readfiles("walk2");
	readfiles("walk3");
	readfiles("walk2");
	readfiles("walk3");
	
	readfiles("walk4");

}
void dance()
{
//----------istart
	speed(0);
	readfiles("p1");
	usleep(6000000);
	speed(1);
	readfiles("d-1");


//-------------point hands
	readfiles("d3");
	readfiles("d4");



//--------------guitar
	readfiles("d0");
	readfiles("d1");
	readfiles("d0a");
	readfiles("d1a");
	readfiles("d0b");
	readfiles("d1b");
	readfiles("d0c");
	readfiles("d1");


//-------------point hands
	readfiles("d3ag");
	readfiles("d4");

//------------palm up hand move
	readfiles("d2");
	readfiles("d5a");
	readfiles("d5b");
	readfiles("d6a");


//-------------point hands
	readfiles("d3");
	readfiles("d4");



//--------------guitar
	readfiles("d0");
	readfiles("d1");
	readfiles("d0a");
	readfiles("d1a");
	readfiles("d0b");
	readfiles("d1b");
	readfiles("d0c");
	readfiles("d1");


//-------------point hands
	readfiles("d3ag");
	readfiles("d4");

//------------palm up hand move
	readfiles("d2");
	readfiles("d5a");
	readfiles("d5b");
	readfiles("d6a");


	readfiles("hi");

}
void istart()
{
speed(0);

readfiles("init");

usleep(6000000);
int l=0;
for(l=0;l<128;l++)
	last_tx_packet[l]=tx_packet[l];
speed(1);
}




/*
//------------------INTERNAL MIRROR for id 25.....reverse drive mode not working....
if(buffer[i]==25)
{
int gpos;
gpos=buffer[i+1]+buffer[i+2]*256;
gpos=4096-gpos;
buffer[i+1]=get_lb(gpos);
buffer[i+2]=get_hb(gpos);
}
*/

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
	ftdi_write_data(&ftdic1,baud, baud[3]+4);
	printf("drive mode 0 written for %d id.",id);
}



/*
int imu()
{   // ftdi_usb_purge_buffers(&imu);
     int a=0;
     byte imu_lb,imu_hb;
     byte imu[5];
     int imu_sum;
     int diff;
     ftdi_usb_purge_buffers(&imu);
     while(a==0)
     {
     a=ftdi_read_data(&imu,&imu_lb,1);
     //printf("%d",a);
     }
     if(imu_lb==65&&a==1)
     {  
                    a=ftdi_read_data(&imu,imu,5);
                    if(a!=0)
                    {
                     imu_sum=((imu[2]*256)+imu[3]);
                    // usleep(50000);
                     }
                    
     }
    printf("%d\n",imu_sum);
    //usleep(50000);
    //ftdi_usb_purge_buffers(&imu);
    
//     ftdi_read_data(&imu,&imu_lb,1);
     
    if(imu[4]==90&&flag_imu==0)
     {
            diff=imu_sum-temp_imu;
            temp_imu=imu_sum;
            if(diff>0)
                {     
                    return ((diff*100)/30);  
                }
      }
      
     if(flag_imu==1)
     flag_imu--;
     return 0;
}
*/



/*----------------------------including imu change--------------------------------------------------
void play_acyut()
{
	
	int a=0;
     	byte imu_lb,imu_hb;
   	byte imu[5];
   	int imu_sum=0;
   	int diff=0;
	ftdi_usb_purge_buffers(&imu);
	printf("Play acyut........\n");	int l=0;
	for(l=0;l<128;l++)
		last_tx_packet[l]=tx_packet[l];
	byte nof_lb,nof_hb,checksum;
	int change=0,imu_gpos=0,lb_change=0,hb_change=0;
	int nof;
	ftdi_read_data(&ftdic2,&nof_lb,1);
	ftdi_read_data(&ftdic2,&nof_hb,1);
	nof=nof_hb*256+nof_lb;
	printf("Number of frames : %x\n",nof);


	int size=sizeof(frame)*nof;
	int sof=sizeof(frame);
	byte *buffer=(byte *)malloc(size);
	byte *buf=(byte *)malloc(sof);
	byte pack;
	int i;
	int j,k,nbytes;

//	for(i=0,j=0;i<nof;i++)
//	{
//		usleep(90000);
//		printf("\nBuffer frame %d :-\n",i);
//		ftdi_read_data(&ftdic2,buf,sof);
//		for(k=0;k<sof;)
//		{
//			buffer[j]=buf[k++];
//			printf("%x  %d\n",buffer[j++],k);
//		}
//	}


int gpos;
	byte p,n;
	i=0;
	int stop=0,t=0;

	while(stop==0)
	{
	if(nbytes=ftdi_read_data(&ftdic2,&pack,1)>0 )
	{
		if(!t){p=n=pack;buffer[i++]=p;t++;}
		else 
		{
			 p=n;
			 n=pack;
		//if(!(i%sof))
		//	printf("\nBuffer frame %d :-\n",i/sof);
	
			if(p==0xee&&n==0xee)
			stop=1;
			else
			{
			buffer[i]=pack;
			printf("%x\n",buffer[i]);
			i++;
			}

	
		}
	}
	}
	int max;
	max=i;

byte num_motors=28, address=0x1e;
int frameno=0;

	printf("\n\n");

	for(i=0;i<max-1;)
	{

		usleep(16670);
		

		checksum=0;
		
		num_motors=buffer[i++];
		address=buffer[i++];
		
		checksum+=num_motors+address;
		tx_packet[3]=(num_motors*3)+4;
		tx_packet[5]=address;
		
		for(j=0;j<num_motors;j++)
		{
		
		tx_packet[7+j*3]=buffer[i++];
		tx_packet[8+j*3]=buffer[i++];
		tx_packet[9+j*3]=buffer[i++];
		/*if(frameno%3==2)
		if(tx_packet[7+j*3]==15&&tx_packet[7+j*3]==16)
		{
			
			a=0;
			imu_sum=0;
			imu_gpos=tx_packet[8+j*3]+tx_packet[9+j*3]*256;
	//		change=imu();
			while(a==0)
   			  {
				a=ftdi_read_data(&imu,&imu_lb,1);
				//printf("%d",a);
			  }
		     if(imu_lb==65&&a==1)
			{  
    	                	a=ftdi_read_data(&imu,imu,5);
    	                	if(a!=0)
				{
				imu_sum=((imu[2]*256)+imu[3]);
                	     usleep(10000);
                    		}
			}
 			   printf("%d\n",imu_sum);
    			//usleep(50000);
   			 //ftdi_usb_purge_buffers(&imu);
    
			//     ftdi_read_data(&imu,&imu_lb,1);
			
			
			if(imu[4]==90&&flag_imu==0)
			{
				diff=imu_sum-temp_imu;
				temp_imu=imu_sum;
				
				if((diff>20&&diff<50)||(diff<-20&&diff>-50))
                		change=(-2*(diff)); 
                		else 
                		change=0; 
                		
			}
      
			if(flag_imu==1)
			flag_imu--;
			
			
			imu_gpos+=change;
			printf("change :%d\n", change);
			lb_change=get_lb(imu_gpos)-tx_packet[8+j*3];
			hb_change=get_hb(imu_gpos)-tx_packet[9+j*3];
			checksum-=(lb_change+hb_change);
			tx_packet[8+j*3]=get_lb(imu_gpos);
			tx_packet[9+j*3]=get_hb(imu_gpos);
			change=0;
			
		}
		*/
		
/*		checksum+=tx_packet[7+j*3]+tx_packet[8+j*3]+tx_packet[9+j*3];
		}
		
		checksum=255-checksum;
		if(checksum!=buffer[i++])
			{frameno++;printf("Checksum error in frame : %d\n",frameno);continue;}

		checksum=255-checksum;
		checksum-=num_motors+address;
//		checksum+=(lb_change+hb_change);
		checksum+=tx_packet[2]+tx_packet[3]+tx_packet[4]+tx_packet[5]+tx_packet[6];

	if(tx_packet[5]==0x1c)
	printf("\n\nsending slope frame.......\n\n");

//	if(tx_packet[5]==0x1e)
//	{
//		double factor=(double)280/250;
//		double deltazero=15*(double)4096/250;
		
//	for(j=0;j<num_motors;j++)
//		{



//			if(tx_packet[5]==0x1e)
//			{
	//		double factor=(double)280/250;
	//		double deltazero=15*(double)4096/250;
			
			
			//----------------------------------------change for imu data---------------------------------------------------
			
//			if(tx_packet[7+j*3]==15)
//				{	
					//checksum-=tx_packet[7+j*3]+tx_packet[8+j*3]+tx_packet[9+j*3];
//					gpos=tx_packet[8+j*3]+256*tx_packet[9+j*3];
				//	printf(" old gpos for id %d was: %d",tx_packet[7+j*3],gpos);
//					change=imu();
//					gpos=gpos+change;
//					printf(" changed to %d\n",gpos);
					//tx_packet[8+j*3]=get_lb(gpos);
					//tx_packet[9+j*3]=get_hb(gpos);
					//checksum+=tx_packet[7+j*3]+tx_packet[8+j*3]+tx_packet[9+j*3];
//				}
			
			
			
			
			
			
			
			
			/*
	
				transferred to below 106+ mapping

				if(tx_packet[7+j*3]==21 || tx_packet[7+j*3]==1) 
				{	
					checksum-=tx_packet[7+j*3]+tx_packet[8+j*3]+tx_packet[9+j*3];
					gpos=tx_packet[8+j*3]+256*tx_packet[9+j*3];
					printf(" old gpos for id %d was: %d",tx_packet[7+j*3],gpos);
					gpos=gpos-50;
					printf(" changed to %d\n",gpos);
					tx_packet[8+j*3]=get_lb(gpos);
					tx_packet[9+j*3]=get_hb(gpos);
					checksum+=tx_packet[7+j*3]+tx_packet[8+j*3]+tx_packet[9+j*3];
				}


				if(tx_packet[7+j*3]==5 || tx_packet[7+j*3]==25) 
				{
					checksum-=tx_packet[7+j*3]+tx_packet[8+j*3]+tx_packet[9+j*3];
					gpos=tx_packet[8+j*3]+256*tx_packet[9+j*3];
					printf(" old gpos for id %d was: %d",tx_packet[7+j*3],gpos);
					gpos=gpos+30;
					printf(" changed to %d\n",gpos);
					tx_packet[8+j*3]=get_lb(gpos);
					tx_packet[9+j*3]=get_hb(gpos);
					checksum+=tx_packet[7+j*3]+tx_packet[8+j*3]+tx_packet[9+j*3];
				}
			*/


			/*
				if(tx_packet[7+j*3]==20 || tx_packet[7+j*3]==0 || tx_packet[7+j*3]==2 || tx_packet[7+j*3]==22 || tx_packet[7+j*3]==12 || tx_packet[7+j*3]==32 || tx_packet[7+j*3]==23 ||  					tx_packet[7+j*3]==3 || tx_packet[7+j*3]==21 || tx_packet[7+j*3]==1 || tx_packet[7+j*3]==5 || tx_packet[7+j*3]==25 || tx_packet[7+j*3]==15)
				{
					checksum-=tx_packet[7+j*3]+tx_packet[8+j*3]+tx_packet[9+j*3];
					gpos=tx_packet[8+j*3]+256*tx_packet[9+j*3];
					printf(" old gpos for id %d was: %d",tx_packet[7+j*3],gpos);
					//gpos=gpos-367+100-50;
					gpos=gpos*factor-deltazero;
					printf(" changed to %d\n",gpos);
					tx_packet[8+j*3]=get_lb(gpos);
					tx_packet[9+j*3]=get_hb(gpos);
					checksum+=tx_packet[7+j*3]+tx_packet[8+j*3]+tx_packet[9+j*3];
				}
			*/		
			// from above 106+ mapping-------
				
			/*
			//---------------------------------------------- ACYUT 3 final tps start here------------------------------------------------------------------
				if(tx_packet[7+j*3]==21 || tx_packet[7+j*3]==1) 
				{
					checksum-=tx_packet[7+j*3]+tx_packet[8+j*3]+tx_packet[9+j*3];
					gpos=tx_packet[8+j*3]+256*tx_packet[9+j*3];
					printf(" old gpos for id %d was: %d",tx_packet[7+j*3],gpos);
					gpos=gpos-56;
					printf(" changed to %d\n",gpos);
					tx_packet[8+j*3]=get_lb(gpos);
					tx_packet[9+j*3]=get_hb(gpos);
					checksum+=tx_packet[7+j*3]+tx_packet[8+j*3]+tx_packet[9+j*3];
				}
	

				if(tx_packet[7+j*3]==5 || tx_packet[7+j*3]==25) 
				{
					checksum-=tx_packet[7+j*3]+tx_packet[8+j*3]+tx_packet[9+j*3];
					gpos=tx_packet[8+j*3]+256*tx_packet[9+j*3];
					printf(" old gpos for id %d was: %d",tx_packet[7+j*3],gpos);
					gpos=gpos+33;
					printf(" changed to %d\n",gpos);
					tx_packet[8+j*3]=get_lb(gpos);
					tx_packet[9+j*3]=get_hb(gpos);
					checksum+=tx_packet[7+j*3]+tx_packet[8+j*3]+tx_packet[9+j*3];
				}
		
			//------------------------------------------------------------------------------------



				if(tx_packet[7+j*3]==1)
				{				
					checksum-=tx_packet[7+j*3]+tx_packet[8+j*3]+tx_packet[9+j*3];
					gpos=tx_packet[8+j*3]+256*tx_packet[9+j*3];
					printf(" old gpos for id %d was: %d",tx_packet[7+j*3],gpos);
					gpos=gpos+12;
					printf(" changed to %d\n",gpos);
					tx_packet[8+j*3]=get_lb(gpos);
					tx_packet[9+j*3]=get_hb(gpos);
					checksum+=tx_packet[7+j*3]+tx_packet[8+j*3]+tx_packet[9+j*3];
				}
				if(tx_packet[7+j*3]==3)
				{
					checksum-=tx_packet[7+j*3]+tx_packet[8+j*3]+tx_packet[9+j*3];
					gpos=tx_packet[8+j*3]+256*tx_packet[9+j*3];
					printf(" old gpos for id %d was: %d",tx_packet[7+j*3],gpos);
					gpos=gpos+12;
					printf(" changed to %d\n",gpos);
					tx_packet[8+j*3]=get_lb(gpos);
					tx_packet[9+j*3]=get_hb(gpos);		
					checksum+=tx_packet[7+j*3]+tx_packet[8+j*3]+tx_packet[9+j*3];
				}
				if(tx_packet[7+j*3]==2)
				{
					checksum-=tx_packet[7+j*3]+tx_packet[8+j*3]+tx_packet[9+j*3];
					gpos=tx_packet[8+j*3]+256*tx_packet[9+j*3];
					printf(" old gpos for id %d was: %d",tx_packet[7+j*3],gpos);
					gpos=gpos+72;
					printf(" changed to %d\n",gpos);
					tx_packet[8+j*3]=get_lb(gpos);
					tx_packet[9+j*3]=get_hb(gpos);
					checksum+=tx_packet[7+j*3]+tx_packet[8+j*3]+tx_packet[9+j*3];
				}
				if(tx_packet[7+j*3]==12)
				{
					checksum-=tx_packet[7+j*3]+tx_packet[8+j*3]+tx_packet[9+j*3];
					gpos=tx_packet[8+j*3]+256*tx_packet[9+j*3];
					printf(" old gpos for id %d was: %d",tx_packet[7+j*3],gpos);
					gpos=gpos-12;
					printf(" changed to %d\n",gpos);
					tx_packet[8+j*3]=get_lb(gpos);
					tx_packet[9+j*3]=get_hb(gpos);
					checksum+=tx_packet[7+j*3]+tx_packet[8+j*3]+tx_packet[9+j*3];
				}
			


				if(tx_packet[7+j*3]==3 || tx_packet[7+j*3]==23)
				{
					checksum-=tx_packet[7+j*3]+tx_packet[8+j*3]+tx_packet[9+j*3];
					gpos=tx_packet[8+j*3]+256*tx_packet[9+j*3];
					printf(" old gpos for id %d was: %d",tx_packet[7+j*3],gpos);
					gpos=gpos-30;	
					printf(" changed to %d\n",gpos);
					tx_packet[8+j*3]=get_lb(gpos);
					tx_packet[9+j*3]=get_hb(gpos);
					checksum+=tx_packet[7+j*3]+tx_packet[8+j*3]+tx_packet[9+j*3];
				}
	
				if(tx_packet[7+j*3]==20)
				{
					checksum-=tx_packet[7+j*3]+tx_packet[8+j*3]+tx_packet[9+j*3];
					gpos=tx_packet[8+j*3]+256*tx_packet[9+j*3];
					printf(" old gpos for id %d was: %d",tx_packet[7+j*3],gpos);
					gpos=gpos-19;
					printf(" changed to %d\n",gpos);
					tx_packet[8+j*3]=get_lb(gpos);
					tx_packet[9+j*3]=get_hb(gpos);
					checksum+=tx_packet[7+j*3]+tx_packet[8+j*3]+tx_packet[9+j*3];
				}	

				if(tx_packet[7+j*3]==0) 
				{	
					checksum-=tx_packet[7+j*3]+tx_packet[8+j*3]+tx_packet[9+j*3];
					gpos=tx_packet[8+j*3]+256*tx_packet[9+j*3];
					printf(" old gpos for id %d was: %d",tx_packet[7+j*3],gpos);
					gpos=gpos+19;
					printf(" changed to %d\n",gpos);
					tx_packet[8+j*3]=get_lb(gpos);
					tx_packet[9+j*3]=get_hb(gpos);
					checksum+=tx_packet[7+j*3]+tx_packet[8+j*3]+tx_packet[9+j*3];
				}
			





				////////////---------------------------------------------CHECK THIS---------------------------------------------------------


				if(tx_packet[7+j*3]==15)
				{
					checksum-=tx_packet[7+j*3]+tx_packet[8+j*3]+tx_packet[9+j*3];
					gpos=tx_packet[8+j*3]+256*tx_packet[9+j*3];
					printf(" old gpos for id %d was: %d",tx_packet[7+j*3],gpos);
					gpos=gpos+100;
					printf(" changed to %d\n",gpos);
					tx_packet[8+j*3]=get_lb(gpos); 
					tx_packet[9+j*3]=get_hb(gpos);
					checksum+=tx_packet[7+j*3]+tx_packet[8+j*3]+tx_packet[9+j*3];
				}


				////////////////-----------------------------------------------------------------------------------------------------------------

				//---------------------------------------------- ACYUT 3 final tps start here------------------------------------------------------------------

/*
if(tx_packet[7+j*3]==23)
{
checksum-=tx_packet[7+j*3]+tx_packet[8+j*3]+tx_packet[9+j*3];
gpos=tx_packet[8+j*3]+256*tx_packet[9+j*3];
printf(" old gpos for id %d was: %d",tx_packet[7+j*3],gpos);
//gpos=gpos-367+100-50;
gpos=gpos-40;
printf(" changed to %d\n",gpos);
tx_packet[8+j*3]=get_lb(gpos); 
tx_packet[9+j*3]=get_hb(gpos);
checksum+=tx_packet[7+j*3]+tx_packet[8+j*3]+tx_packet[9+j*3];
}



if(tx_packet[7+j*3]==20)
{
checksum-=tx_packet[7+j*3]+tx_packet[8+j*3]+tx_packet[9+j*3];
gpos=tx_packet[8+j*3]+256*tx_packet[9+j*3];
printf(" old gpos for id %d was: %d",tx_packet[7+j*3],gpos);
//gpos=gpos-367+100-50;
//gpos=gpos*factor-deltazero;
gpos=gpos-19;//-50
printf(" changed to %d\n",gpos);
tx_packet[8+j*3]=get_lb(gpos);
tx_packet[9+j*3]=get_hb(gpos);
checksum+=tx_packet[7+j*3]+tx_packet[8+j*3]+tx_packet[9+j*3];
}

if(tx_packet[7+j*3]==0) 
{
checksum-=tx_packet[7+j*3]+tx_packet[8+j*3]+tx_packet[9+j*3];
gpos=tx_packet[8+j*3]+256*tx_packet[9+j*3];
printf(" old gpos for id %d was: %d",tx_packet[7+j*3],gpos);
//gpos=gpos-367+100-50;
//gpos=gpos*factor-deltazero;
gpos=gpos+19;//-50
printf(" changed to %d\n",gpos);
tx_packet[8+j*3]=get_lb(gpos);
tx_packet[9+j*3]=get_hb(gpos);
checksum+=tx_packet[7+j*3]+tx_packet[8+j*3]+tx_packet[9+j*3];
}




if(tx_packet[7+j*3]==25)
{
checksum-=tx_packet[7+j*3]+tx_packet[8+j*3]+tx_packet[9+j*3];
gpos=tx_packet[8+j*3]+256*tx_packet[9+j*3];
printf(" old gpos for id %d was: %d",tx_packet[7+j*3],gpos);
gpos=gpos-19;//+10;
printf(" changed to %d\n",gpos);
tx_packet[8+j*3]=get_lb(gpos);
tx_packet[9+j*3]=get_hb(gpos);
checksum+=tx_packet[7+j*3]+tx_packet[8+j*3]+tx_packet[9+j*3];
}
if(tx_packet[7+j*3]==5)
{
checksum-=tx_packet[7+j*3]+tx_packet[8+j*3]+tx_packet[9+j*3];
gpos=tx_packet[8+j*3]+256*tx_packet[9+j*3];
printf(" old gpos for id 5 %d",gpos);
gpos=gpos+19;//-40;
printf(" changed to %d\n",gpos);
tx_packet[8+j*3]=get_lb(gpos);
tx_packet[9+j*3]=get_hb(gpos);
checksum+=tx_packet[7+j*3]+tx_packet[8+j*3]+tx_packet[9+j*3];
}








if(tx_packet[7+j*3]==32 || tx_packet[7+j*3]==22) 
{
checksum-=tx_packet[7+j*3]+tx_packet[8+j*3]+tx_packet[9+j*3];
gpos=tx_packet[8+j*3]+256*tx_packet[9+j*3];
printf(" old gpos for id %d was: %d",tx_packet[7+j*3],gpos);
//gpos=gpos-367+100-50;
gpos=gpos-20;
printf(" changed to %d\n",gpos);
tx_packet[8+j*3]=get_lb(gpos); 
tx_packet[9+j*3]=get_hb(gpos);
checksum+=tx_packet[7+j*3]+tx_packet[8+j*3]+tx_packet[9+j*3];
}


*/




//				}
//		}
/*
	int l=0;
		checksum=255-checksum;
		tx_packet[7+j*3]=checksum;
		ftdi_write_data(&ftdic1,tx_packet,tx_packet[3]+4);
		for(l=0;l<128;l++)
		last_tx_packet[l]=tx_packet[l];
		frameno++;
		if(tx_packet[5]==0x1c)
		printf("\n\nslope frame sent.......\n\n");

		printf("frame sent : %d\n",frameno);
	}
	free(buffer);
	free(buf);

}
*/



/*27-11-2010
void play_acyut()
{
	printf("Play acyut........\n");
	byte nof_lb,nof_hb,checksum=0;
	int nof;
	ftdi_read_data(&ftdic2,&nof_lb,1);
	ftdi_read_data(&ftdic2,&nof_hb,1);
	nof=nof_hb*256+nof_lb;
	printf("Number of frames : %x\n",nof);


	int size=sizeof(frame)*nof;
	int sof=sizeof(frame);
	byte *buffer=(byte *)malloc(size);
	byte *buf=(byte *)malloc(sof);
	byte pack;
	int i;
	int j,k,nbytes;

/*	for(i=0,j=0;i<nof;i++)
	{
		usleep(90000);
		printf("\nBuffer frame %d :-\n",i);
		ftdi_read_data(&ftdic2,buf,sof);
		for(k=0;k<sof;)
		{
			buffer[j]=buf[k++];
			printf("%x  %d\n",buffer[j++],k);
		}
	}

*/
/*27-11-2010
int gpos;
	byte p,n;
	i=0;
	int stop=0,t=0;

	while(stop==0)
	{
	if(nbytes=ftdi_read_data(&ftdic2,&pack,1)>0 )
	{
		if(!t){p=n=pack;buffer[i++]=p;t++;}
		else 
		{
			 p=n;
			 n=pack;
		//if(!(i%sof))
		//	printf("\nBuffer frame %d :-\n",i/sof);
	
			if(p==0xee&&n==0xee)
			stop=1;
			else
			{
			buffer[i]=pack;
			printf("%x\n",buffer[i]);
			i++;
			}

	
		}
	}
	}
	int max;
	max=i;

byte num_motors=28, address=0x1e;
int frameno=0;

	printf("\n\n");

	for(i=0;i<max-1;)
	{

		usleep(16670);
		

		checksum=0;
		
		num_motors=buffer[i++];
		address=buffer[i++];
		
		checksum+=num_motors+address;
		tx_packet[3]=(num_motors*3)+4;
		tx_packet[5]=address;
		
		for(j=0;j<num_motors;j++)
		{
		
		tx_packet[7+j*3]=buffer[i++];
		tx_packet[8+j*3]=buffer[i++];
		tx_packet[9+j*3]=buffer[i++];
		
		checksum+=tx_packet[7+j*3]+tx_packet[8+j*3]+tx_packet[9+j*3];
		}
		
		checksum=255-checksum;
		if(checksum!=buffer[i++])
			{frameno++;printf("Checksum error in frame : %d\n",frameno);continue;}

		checksum=255-checksum;
		checksum-=num_motors+address;
		checksum+=tx_packet[2]+tx_packet[3]+tx_packet[4]+tx_packet[5]+tx_packet[6];

	if(tx_packet[5]==0x1c)
	printf("\n\nsending slope frame.......\n\n");

//	if(tx_packet[5]==0x1e)
//	{
//		double factor=(double)280/250;
//		double deltazero=15*(double)4096/250;
		
//		for(j=0;j<num_motors;j++)
//		{



	//		if(tx_packet[5]==0x1e)
	//		{
	//		double factor=(double)280/250;
	//		double deltazero=15*(double)4096/250;
			/*
	
				transferred to below 106+ mapping

				if(tx_packet[7+j*3]==21 || tx_packet[7+j*3]==1) 
				{	
					checksum-=tx_packet[7+j*3]+tx_packet[8+j*3]+tx_packet[9+j*3];
					gpos=tx_packet[8+j*3]+256*tx_packet[9+j*3];
					printf(" old gpos for id %d was: %d",tx_packet[7+j*3],gpos);
					gpos=gpos-50;
					printf(" changed to %d\n",gpos);
					tx_packet[8+j*3]=get_lb(gpos);
					tx_packet[9+j*3]=get_hb(gpos);
					checksum+=tx_packet[7+j*3]+tx_packet[8+j*3]+tx_packet[9+j*3];
				}


				if(tx_packet[7+j*3]==5 || tx_packet[7+j*3]==25) 
				{
					checksum-=tx_packet[7+j*3]+tx_packet[8+j*3]+tx_packet[9+j*3];
					gpos=tx_packet[8+j*3]+256*tx_packet[9+j*3];
					printf(" old gpos for id %d was: %d",tx_packet[7+j*3],gpos);
					gpos=gpos+30;
					printf(" changed to %d\n",gpos);
					tx_packet[8+j*3]=get_lb(gpos);
					tx_packet[9+j*3]=get_hb(gpos);
					checksum+=tx_packet[7+j*3]+tx_packet[8+j*3]+tx_packet[9+j*3];
				}
			*/


			/*
				if(tx_packet[7+j*3]==20 || tx_packet[7+j*3]==0 || tx_packet[7+j*3]==2 || tx_packet[7+j*3]==22 || tx_packet[7+j*3]==12 || tx_packet[7+j*3]==32 || tx_packet[7+j*3]==23 ||  					tx_packet[7+j*3]==3 || tx_packet[7+j*3]==21 || tx_packet[7+j*3]==1 || tx_packet[7+j*3]==5 || tx_packet[7+j*3]==25 || tx_packet[7+j*3]==15)
				{
					checksum-=tx_packet[7+j*3]+tx_packet[8+j*3]+tx_packet[9+j*3];
					gpos=tx_packet[8+j*3]+256*tx_packet[9+j*3];
					printf(" old gpos for id %d was: %d",tx_packet[7+j*3],gpos);
					//gpos=gpos-367+100-50;
					gpos=gpos*factor-deltazero;
					printf(" changed to %d\n",gpos);
					tx_packet[8+j*3]=get_lb(gpos);
					tx_packet[9+j*3]=get_hb(gpos);
					checksum+=tx_packet[7+j*3]+tx_packet[8+j*3]+tx_packet[9+j*3];
				}
			*/		
			// from above 106+ mapping-------
				
			/*
			//---------------------------------------------- ACYUT 3 final tps start here------------------------------------------------------------------
				if(tx_packet[7+j*3]==21 || tx_packet[7+j*3]==1) 
				{
					checksum-=tx_packet[7+j*3]+tx_packet[8+j*3]+tx_packet[9+j*3];
					gpos=tx_packet[8+j*3]+256*tx_packet[9+j*3];
					printf(" old gpos for id %d was: %d",tx_packet[7+j*3],gpos);
					gpos=gpos-56;
					printf(" changed to %d\n",gpos);
					tx_packet[8+j*3]=get_lb(gpos);
					tx_packet[9+j*3]=get_hb(gpos);
					checksum+=tx_packet[7+j*3]+tx_packet[8+j*3]+tx_packet[9+j*3];
				}
	

				if(tx_packet[7+j*3]==5 || tx_packet[7+j*3]==25) 
				{
					checksum-=tx_packet[7+j*3]+tx_packet[8+j*3]+tx_packet[9+j*3];
					gpos=tx_packet[8+j*3]+256*tx_packet[9+j*3];
					printf(" old gpos for id %d was: %d",tx_packet[7+j*3],gpos);
					gpos=gpos+33;
					printf(" changed to %d\n",gpos);
					tx_packet[8+j*3]=get_lb(gpos);
					tx_packet[9+j*3]=get_hb(gpos);
					checksum+=tx_packet[7+j*3]+tx_packet[8+j*3]+tx_packet[9+j*3];
				}
		
			//------------------------------------------------------------------------------------



				if(tx_packet[7+j*3]==1)
				{				
					checksum-=tx_packet[7+j*3]+tx_packet[8+j*3]+tx_packet[9+j*3];
					gpos=tx_packet[8+j*3]+256*tx_packet[9+j*3];
					printf(" old gpos for id %d was: %d",tx_packet[7+j*3],gpos);
					gpos=gpos+12;
					printf(" changed to %d\n",gpos);
					tx_packet[8+j*3]=get_lb(gpos);
					tx_packet[9+j*3]=get_hb(gpos);
					checksum+=tx_packet[7+j*3]+tx_packet[8+j*3]+tx_packet[9+j*3];
				}
				if(tx_packet[7+j*3]==3)
				{
					checksum-=tx_packet[7+j*3]+tx_packet[8+j*3]+tx_packet[9+j*3];
					gpos=tx_packet[8+j*3]+256*tx_packet[9+j*3];
					printf(" old gpos for id %d was: %d",tx_packet[7+j*3],gpos);
					gpos=gpos+12;
					printf(" changed to %d\n",gpos);
					tx_packet[8+j*3]=get_lb(gpos);
					tx_packet[9+j*3]=get_hb(gpos);		
					checksum+=tx_packet[7+j*3]+tx_packet[8+j*3]+tx_packet[9+j*3];
				}
				if(tx_packet[7+j*3]==2)
				{
					checksum-=tx_packet[7+j*3]+tx_packet[8+j*3]+tx_packet[9+j*3];
					gpos=tx_packet[8+j*3]+256*tx_packet[9+j*3];
					printf(" old gpos for id %d was: %d",tx_packet[7+j*3],gpos);
					gpos=gpos+72;
					printf(" changed to %d\n",gpos);
					tx_packet[8+j*3]=get_lb(gpos);
					tx_packet[9+j*3]=get_hb(gpos);
					checksum+=tx_packet[7+j*3]+tx_packet[8+j*3]+tx_packet[9+j*3];
				}
				if(tx_packet[7+j*3]==12)
				{
					checksum-=tx_packet[7+j*3]+tx_packet[8+j*3]+tx_packet[9+j*3];
					gpos=tx_packet[8+j*3]+256*tx_packet[9+j*3];
					printf(" old gpos for id %d was: %d",tx_packet[7+j*3],gpos);
					gpos=gpos-12;
					printf(" changed to %d\n",gpos);
					tx_packet[8+j*3]=get_lb(gpos);
					tx_packet[9+j*3]=get_hb(gpos);
					checksum+=tx_packet[7+j*3]+tx_packet[8+j*3]+tx_packet[9+j*3];
				}
			


				if(tx_packet[7+j*3]==3 || tx_packet[7+j*3]==23)
				{
					checksum-=tx_packet[7+j*3]+tx_packet[8+j*3]+tx_packet[9+j*3];
					gpos=tx_packet[8+j*3]+256*tx_packet[9+j*3];
					printf(" old gpos for id %d was: %d",tx_packet[7+j*3],gpos);
					gpos=gpos-30;	
					printf(" changed to %d\n",gpos);
					tx_packet[8+j*3]=get_lb(gpos);
					tx_packet[9+j*3]=get_hb(gpos);
					checksum+=tx_packet[7+j*3]+tx_packet[8+j*3]+tx_packet[9+j*3];
				}
	
				if(tx_packet[7+j*3]==20)
				{
					checksum-=tx_packet[7+j*3]+tx_packet[8+j*3]+tx_packet[9+j*3];
					gpos=tx_packet[8+j*3]+256*tx_packet[9+j*3];
					printf(" old gpos for id %d was: %d",tx_packet[7+j*3],gpos);
					gpos=gpos-19;
					printf(" changed to %d\n",gpos);
					tx_packet[8+j*3]=get_lb(gpos);
					tx_packet[9+j*3]=get_hb(gpos);
					checksum+=tx_packet[7+j*3]+tx_packet[8+j*3]+tx_packet[9+j*3];
				}	

				if(tx_packet[7+j*3]==0) 
				{	
					checksum-=tx_packet[7+j*3]+tx_packet[8+j*3]+tx_packet[9+j*3];
					gpos=tx_packet[8+j*3]+256*tx_packet[9+j*3];
					printf(" old gpos for id %d was: %d",tx_packet[7+j*3],gpos);
					gpos=gpos+19;
					printf(" changed to %d\n",gpos);
					tx_packet[8+j*3]=get_lb(gpos);
					tx_packet[9+j*3]=get_hb(gpos);
					checksum+=tx_packet[7+j*3]+tx_packet[8+j*3]+tx_packet[9+j*3];
				}
			





				////////////---------------------------------------------CHECK THIS---------------------------------------------------------


				if(tx_packet[7+j*3]==15)
				{
					checksum-=tx_packet[7+j*3]+tx_packet[8+j*3]+tx_packet[9+j*3];
					gpos=tx_packet[8+j*3]+256*tx_packet[9+j*3];
					printf(" old gpos for id %d was: %d",tx_packet[7+j*3],gpos);
					gpos=gpos+100;
					printf(" changed to %d\n",gpos);
					tx_packet[8+j*3]=get_lb(gpos); 
					tx_packet[9+j*3]=get_hb(gpos);
					checksum+=tx_packet[7+j*3]+tx_packet[8+j*3]+tx_packet[9+j*3];
				}


				////////////////-----------------------------------------------------------------------------------------------------------------

				//---------------------------------------------- ACYUT 3 final tps start here------------------------------------------------------------------

/*
if(tx_packet[7+j*3]==23)
{
checksum-=tx_packet[7+j*3]+tx_packet[8+j*3]+tx_packet[9+j*3];
gpos=tx_packet[8+j*3]+256*tx_packet[9+j*3];
printf(" old gpos for id %d was: %d",tx_packet[7+j*3],gpos);
//gpos=gpos-367+100-50;
gpos=gpos-40;
printf(" changed to %d\n",gpos);
tx_packet[8+j*3]=get_lb(gpos); 
tx_packet[9+j*3]=get_hb(gpos);
checksum+=tx_packet[7+j*3]+tx_packet[8+j*3]+tx_packet[9+j*3];
}



if(tx_packet[7+j*3]==20)
{
checksum-=tx_packet[7+j*3]+tx_packet[8+j*3]+tx_packet[9+j*3];
gpos=tx_packet[8+j*3]+256*tx_packet[9+j*3];
printf(" old gpos for id %d was: %d",tx_packet[7+j*3],gpos);
//gpos=gpos-367+100-50;
//gpos=gpos*factor-deltazero;
gpos=gpos-19;//-50
printf(" changed to %d\n",gpos);
tx_packet[8+j*3]=get_lb(gpos);
tx_packet[9+j*3]=get_hb(gpos);
checksum+=tx_packet[7+j*3]+tx_packet[8+j*3]+tx_packet[9+j*3];
}

if(tx_packet[7+j*3]==0) 
{
checksum-=tx_packet[7+j*3]+tx_packet[8+j*3]+tx_packet[9+j*3];
gpos=tx_packet[8+j*3]+256*tx_packet[9+j*3];
printf(" old gpos for id %d was: %d",tx_packet[7+j*3],gpos);
//gpos=gpos-367+100-50;
//gpos=gpos*factor-deltazero;
gpos=gpos+19;//-50
printf(" changed to %d\n",gpos);
tx_packet[8+j*3]=get_lb(gpos);
tx_packet[9+j*3]=get_hb(gpos);
checksum+=tx_packet[7+j*3]+tx_packet[8+j*3]+tx_packet[9+j*3];
}




if(tx_packet[7+j*3]==25)
{
checksum-=tx_packet[7+j*3]+tx_packet[8+j*3]+tx_packet[9+j*3];
gpos=tx_packet[8+j*3]+256*tx_packet[9+j*3];
printf(" old gpos for id %d was: %d",tx_packet[7+j*3],gpos);
gpos=gpos-19;//+10;
printf(" changed to %d\n",gpos);
tx_packet[8+j*3]=get_lb(gpos);
tx_packet[9+j*3]=get_hb(gpos);
checksum+=tx_packet[7+j*3]+tx_packet[8+j*3]+tx_packet[9+j*3];
}
if(tx_packet[7+j*3]==5)
{
checksum-=tx_packet[7+j*3]+tx_packet[8+j*3]+tx_packet[9+j*3];
gpos=tx_packet[8+j*3]+256*tx_packet[9+j*3];
printf(" old gpos for id 5 %d",gpos);
gpos=gpos+19;//-40;
printf(" changed to %d\n",gpos);
tx_packet[8+j*3]=get_lb(gpos);
tx_packet[9+j*3]=get_hb(gpos);
checksum+=tx_packet[7+j*3]+tx_packet[8+j*3]+tx_packet[9+j*3];
}








if(tx_packet[7+j*3]==32 || tx_packet[7+j*3]==22) 
{
checksum-=tx_packet[7+j*3]+tx_packet[8+j*3]+tx_packet[9+j*3];
gpos=tx_packet[8+j*3]+256*tx_packet[9+j*3];
printf(" old gpos for id %d was: %d",tx_packet[7+j*3],gpos);
//gpos=gpos-367+100-50;
gpos=gpos-20;
printf(" changed to %d\n",gpos);
tx_packet[8+j*3]=get_lb(gpos); 
tx_packet[9+j*3]=get_hb(gpos);
checksum+=tx_packet[7+j*3]+tx_packet[8+j*3]+tx_packet[9+j*3];
}


*/




	//			}
	//	}
/*27-11-2010	
		checksum=255-checksum;
		tx_packet[7+j*3]=checksum;
		int l=0;
		ftdi_write_data(&ftdic1,tx_packet,tx_packet[3]+4);
		for(l=0;l<128;l++)
		last_tx_packet[l]=tx_packet[l];
		frameno++;
		if(tx_packet[5]==0x1c)
		printf("\n\nslope frame sent.......\n\n");

		printf("frame sent : %d\n",frameno);
	}
	free(buffer);
	free(buf);
/*
if(nof==45 || nof==33)
{
	punch(0);
}
*/



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

/*int read_acyut()
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
						//else
						{
							if (++no_of_try<3)
							{
								element--;
								count--;
								flag=1;
							}//
						else
						{
							rcv_packet[0]=gui_packet[element-1];
							rcv_packet[3]=((noat>=10)?22:29);
							rcv_packet[4]=0xee;
						//	flag=0;
							printf("skipped a motor\n");
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
				printf("skipped a motor\n");
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
	//int deepak=0;
	for(;deepak<4+nom*3;deepak++)
	{
		printf("pack [%d] = %d\n",deepak,return_data[deepak]);
	}//
	if(!ftdi_write_data(&ftdic2,return_data,4+nom*3))
			return 21;
	return 1;
}
*/

/*void read_acyut()
{

	byte read_packet[9]={0xff,0xff,0,0x04,0x02,0x24,0x02,0};
	byte buffer[40],nom,p,n,gen;
	byte *bufptr,*packptr,pack;
	byte id,checksum=0;
	byte buf[10]={0xff,0xff,0x02};
	int t=0,i,counter=3,nbytes,lenp;
	int j=0;
	i=1;
	while(i)
	{
	if(ftdi_read_data(&ftdic2,&nom,1)>0)
		{
		printf("nom:%d\n",nom);
//		usleep(200000);
//		ftdi_read_data(&ftdic2,buffer,nom+1);
		for(j=0;j<=nom;)
		{
			if(ftdi_read_data(&ftdic2,&buffer[j],1)>0)
				j++;
		}
		i=0;
		}
	}
	buf[3]=nom;
	for(i=0;i<nom;i++)
		checksum+=buffer[i];
	checksum=255-checksum;
	if(checksum!=buffer[i])
	{
		printf("Bluetooth Checksum error....\n");
		return;
	}
	ftdi_write_data(&ftdic2,buf,4);
	checksum=0;
	printf("Buffer.......\n");
	for(i=0;i<=nom;i++)
	printf("%x\n",buffer[i]);
	
	printf("End of buffer\n\n");

	int buf_i=0;
	id=0xff;
	for(i=0;i<nom;i++)	
	{
		read_packet[2]=buffer[i];
		read_packet[7]=0x2c+read_packet[2];
		read_packet[7]=255-read_packet[7];
		ftdi_write_data(&ftdic1,read_packet,8);
		bufptr = buf;

		while((nbytes=ftdi_read_data(&ftdic1,&pack,1))>0)
		{
			if(!t){p=n=id=pack;t++;}
			else 
			{
				 p=n;
				 n=id;
				 id=pack;	//id

				 if(p==255&&n==255&&id<=NOM+5)
				 {			
					nbytes=ftdi_read_data(&ftdic1,&pack,1);
					lenp=pack;
					nbytes=ftdi_read_data(&ftdic1,&pack,1);
					n=pack;
					if(n&&counter)
					{
						ftdi_write_data(&ftdic2,&id,1);
						printf("ERROR :%x\n",n);
						i--;counter--;break;
						ftdi_write_data(&ftdic2,&n,1);
						pack=0xee;
						ftdi_write_data(&ftdic2,&pack,1);
						checksum+=n+pack+id;
					}


					if(!n)
					{	
					nbytes=ftdi_read_data(&ftdic1,bufptr,lenp-1);
//				gpos=buf[1]*256+buf[0];
					ftdi_write_data(&ftdic2,&id,1);
					ftdi_write_data(&ftdic2,buf,2);
					printf("id	%d\ngoal position: %d\n\n",id,buf[0]+buf[1]*256);
					checksum+=id+buf[0]+buf[1];
					nbytes=ftdi_read_data(&ftdic1,packptr,1);
					n=pack;
					t++;
					counter=3;
					}
				 }
			}
		}

		if(buffer[buf_i]==id)buf_i++;
		else
		{
			ftdi_write_data(&ftdic2,&buffer[buf_i],1);
			printf("Not Responding : %d\n",buffer[buf_i]);
			pack=0xee;
			ftdi_write_data(&ftdic2,&pack,1);
			ftdi_write_data(&ftdic2,&pack,1);
			checksum+=pack+pack+buffer[buf_i];
			buf_i++;
		}

		usleep(20000);
	}
	checksum=255-checksum;
	ftdi_write_data(&ftdic2,&checksum,1);
}*/

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
/*
int dir_tran()
{
	byte checksum=0;
	int len=(element+3)+gui_packet[element+3];
	
	byte packet[len];
	packet[0]=gui_packet[element++];
	packet[1]=gui_packet[element++];
	for(;element<len
	
}*/
/*
void dir_tran()
{
	printf("Direct transmission....\n");
	int nbytes;
	byte pack,p,i=0,j=0;
	byte packet[128];
	while(1)
	{
	nbytes=ftdi_read_data(&ftdic2,&pack,1);
	if(p==0xee&&pack==0xee)break;
	if(pack!=0xee&&p==0xee)
	ftdi_write_data(&ftdic1,&pack,1);
	if(nbytes>0)
	{
		if(pack!=0xee)
		{
//		printf("%x\n",pack);
		//ftdi_write_data(&ftdic1,&pack,1);
		packet[i]=pack;
		i++;
		}
		p=pack;
	}
	}
for(j=0;j<packet[3]+4;j++)
	printf("%x\n",packet[j]);
ftdi_write_data(&ftdic1,packet,packet[3]+4);
printf("End of Direct transmission.\n");
}

*/

/*
int imucal(float init_ang[])
{
	byte im[8],i,j,k=0,a=3;
	byte buffer[59];
	float ang[3]={0};
	byte flag=0;
		
          while(flag==0)
		{   
		if(ftdi_read_data(&imu,&im[0],1)>0)
		{
			
			if(im[0]==0xff)
			{	i=0;

					while(ftdi_read_data(&imu,&im[i],1)>0 && im[i]!=254)
					{	buffer[i]=im[i];
						i++;
						flag=1;	
					}


				for(j=0; j<7; j++)
				{
					 im[j]=buffer[j];
					//printf("%d ",im[j]);
				}
				printf("\n");	
				
			for(j=0,k=0;j<6;j+=2,k+=1)
				{
					ang[k]=im[j]+im[j+1]*256;
					ang[k]=(ang[k] - 0) * (180 + 180) / (720 - 0) + -180;
					init_ang[k]=ang[k];		

				}
			for(k=0; k<3; k++)
				printf("ang %d is %lf\n",k,ang[k]);

			}
		}
	}
	return 0;
}
*/


int temp_save()
{
	move mov;
	FILE *movwr;
	int i=0;
	if(acyut==3)
	{
		movwr=fopen("acyut3/temp","w");
		if(movwr==NULL)
			return 40;
	}
	else if(acyut==4)
	{
		movwr=fopen("acyut4/temp","w");
		if(movwr==NULL)
			return 40;
	}
	else
		return 41;
	mov.noframes=gui_packet[element++];
	mov.noframes=mov.noframes+256*gui_packet[element++];
	for(;i<mov.noframes;i++)
	{
		mov.frames[i].checksum=0;
		mov.frames[i].address=gui_packet[element++];
		mov.frames[i].num_motors=gui_packet[element++];
		mov.frames[i].checksum+=mov.frames[i].address+mov.frames[i].num_motors;
		int j=0;
		if((mov.frames[i].address==0x1c)||(mov.frames[i].address==0x1a))
		{
			for(;j<mov.frames[i].num_motors;j++)
			{
				mov.frames[i].motorvalues[j].ID=gui_packet[element++];
				mov.frames[i].motorvalues[j].LB=gui_packet[element];
				mov.frames[i].motorvalues[j].HB=gui_packet[element++];
				mov.frames[i].checksum+=(mov.frames[i].motorvalues[j].ID+mov.frames[i].motorvalues[j].LB+mov.frames[i].motorvalues[j].HB);
			}
		}else
		{
			for(;j<mov.frames[i].num_motors;j++)
			{
				mov.frames[i].motorvalues[j].ID=gui_packet[element++];
				mov.frames[i].motorvalues[j].LB=gui_packet[element++];
				mov.frames[i].motorvalues[j].HB=gui_packet[element++];
				mov.frames[i].checksum+=(mov.frames[i].motorvalues[j].ID+mov.frames[i].motorvalues[j].LB+mov.frames[i].motorvalues[j].HB);
			}
		}
		mov.frames[i].checksum=255-mov.frames[i].checksum;
	}
	if(fwrite(&mov,sizeof(move),1,movwr)!=1)
		return 42;
	//act_fpacket("temp");
	return 1; //SUCCESS!!
}
int perm_save()                                                //MOVE STORE
{
	move mov;
	FILE *movwr;
	int i=0;
	
	const int len=(int)gui_packet[element++];
	char name[len+8];
	if(acyut==3)
	strcpy(name,"acyut3/moves/");
	else if(acyut==4)
	strcpy(name,"acyut4/moves/");
	else
	return 41; 
	
	char temp[len+1];
	//printf("Can declare array.....\n");
	for(;(element-3)<len;element++)
	temp[element-3]=(char)gui_packet[element];
	temp[len]='\0';

	strcat(name,temp);
	printf("name of file %s\n",name);
	//element++;
	//printf("Can declare array1....\n");
	if((movwr=fopen(name,"w"))==NULL)
	{
		//printf("Bhak\n");
		return 43;
	}
	//printf("Can declare array2....\n");
	mov.noframes=gui_packet[element++];
	mov.noframes=mov.noframes+256*gui_packet[element++];
	printf("no of frames....%d\n",mov.noframes);
	for(;i<mov.noframes;i++)
	{
		mov.frames[i].checksum=0;
		mov.frames[i].address=gui_packet[element++];
		mov.frames[i].num_motors=gui_packet[element++];
		mov.frames[i].checksum+=mov.frames[i].address+mov.frames[i].num_motors;
		//printf("ADD=%d\t NUM=%d\t \n",mov.frames[i].address,mov.frames[i].num_motors);
		int j=0;
		if((mov.frames[i].address==0x1c)||(mov.frames[i].address==0x1a))
		{
			for(;j<mov.frames[i].num_motors;j++)
			{
				mov.frames[i].motorvalues[j].ID=gui_packet[element++];
				mov.frames[i].motorvalues[j].LB=gui_packet[element];
				mov.frames[i].motorvalues[j].HB=gui_packet[element++];
				mov.frames[i].checksum+=(mov.frames[i].motorvalues[j].ID+mov.frames[i].motorvalues[j].LB+mov.frames[i].motorvalues[j].HB);
			}
		}else
		{
			for(;j<mov.frames[i].num_motors;j++)
			{
				mov.frames[i].motorvalues[j].ID=gui_packet[element++];
				mov.frames[i].motorvalues[j].LB=gui_packet[element++];
				mov.frames[i].motorvalues[j].HB=gui_packet[element++];
				mov.frames[i].checksum+=(mov.frames[i].motorvalues[j].ID+mov.frames[i].motorvalues[j].LB+mov.frames[i].motorvalues[j].HB);
			}
		}
		mov.frames[i].checksum=255-mov.frames[i].checksum;
		//printf("%d\n",mov.frames[i].checksum);
	}
	//printf("Can declare array3....\n");
	if(fwrite(&mov,sizeof(move),1,movwr)!=1)
	{
		printf("Error in write\n");
		return 42;
	}//printf("Can declare array4....\n");
	fclose(movwr);
	return 1;//SUCCESS;
	
}

int remote_del(char *filename)
{
	FILE *add,*sort;
	remote_content del_move,ex_move;
	del_move.code1=(double)gui_packet[element++];
	del_move.code1=del_move.code1+(double)gui_packet[element++]*1000;
	int movename_len=(int)gui_packet[element++],count=0;
	do
	{
		del_move.remote_movename[count++]=(char)gui_packet[element++];
	}while(count<movename_len);
	if((sort=fopen(filename,"r"))==NULL)
	{
		//here will appear the not existent file error statement  for gui
		return 140;
	}
	else
	{
		add=fopen("temp_remote_file.remote","w");
		while(TRUE)
		{
			fread(&ex_move,sizeof(remote_content),1,sort);
			if(feof(sort))
			{
				//empty file or move not present in file
				return 141;
			}
			else if(((ex_move.code1)==(del_move.code1))&&(strcmp(ex_move.remote_movename,del_move.remote_movename==0)))
			{
				do
				{
					fwrite(&ex_move,sizeof(remote_content),1,add);
					fread(&ex_move,sizeof(remote_content),1,sort);
				}while(!feof(sort));
				break;
			}
			else
			fwrite(&ex_move,sizeof(remote_content),1,add);
		}
		fclose(add);
		fclose(sort);
		remove(filename);
		rename("temp_remote_file.remote",filename);
	}
	return 1;
}

int remote_add(char *filename)
{
	FILE *add,*sort;
	remote_content new_move,ex_move;
	new_move.code1=(double)gui_packet[element++];
	new_move.code1=new_move.code1+(double)gui_packet[element++]*1000;
	new_move.code2=gui_packet[element++];
	int movename_len=(int)gui_packet[element++],count=0;
	do
	{
		new_move.remote_movename[count++]=(char)gui_packet[element++];
	}while(count<movename_len);
	if(count<30)
	new_move.remote_movename[count]='\0';
	printf("movename %s \n", new_move.remote_movename);
	
	sort=fopen(filename,"r");
	if(sort==NULL)
	{
		sort=fopen(filename,"w");
		fwrite(&new_move,sizeof(remote_content),1,sort);
		fclose(sort);
	}	
	else
	{
		add=fopen("temp_remote_file.remote","w");
		while(TRUE)
		{
			fread(&ex_move,sizeof(remote_content),1,sort);
			if(feof(sort))
			{
				fwrite(&new_move,sizeof(remote_content),1,add);
				break;
			}
			else if(((ex_move.code1)==(new_move.code1))&&(strcmp(ex_move.remote_movename,new_move.remote_movename)==0))
			{
				fclose(add);
				fclose(sort);
				remove("temp_remote_file.remote");
				return 120;
			}else if((ex_move.code1)>(new_move.code1))
			{
				fwrite(&new_move,sizeof(remote_content),1,add);
				do
				{
					fwrite(&ex_move,sizeof(remote_content),1,add);
					fread(&ex_move,sizeof(remote_content),1,sort);
				}while(!feof(sort));
				break;
			}
			else
			fwrite(&ex_move,sizeof(remote_content),1,add);
		}
		fclose(add);
		fclose(sort);
		remove(filename);
		rename("temp_remote_file.remote",filename);
	}
	return 1;
}

int add_status()
{
	//printf("entered add_status\n");
	int iterator_name,name_l;
	height_status move_status,rem_status;
	name_l=gui_packet[element++];
	//printf("name_l : %d\n",name_l);
	if(name_l>=30)
		return 91;
	for(iterator_name=0;iterator_name<name_l;)
		move_status.rem_move_name[iterator_name++]=gui_packet[element++];
	if(iterator_name<30)
		move_status.rem_move_name[iterator_name]='\0';
	//printf("Name : %s \n",move_status.rem_move_name);
	move_status.init_h=gui_packet[element++];
	move_status.init_h+=gui_packet[element++]*256;
	move_status.final_h=gui_packet[element++];
	move_status.final_h+=gui_packet[element++]*256;
	move_status.init_status=gui_packet[element++];
	move_status.final_status=gui_packet[element++];
	printf("move fianl status: %d \n",move_status.final_status);
	FILE *status_add,*status_temp;
	//printf("saved all vaeriables about to open files\n");
	status_add=fopen("status.status","r");
	status_temp=fopen("temp_status.status","w");
	if(status_add==NULL)
	{
		printf("File does not exist\n");
		status_add=fopen("status.status","w");
		fwrite(&move_status,sizeof(move_status),1,status_add);
		fclose(status_add);
		return 1;
	}
	else if(status_temp==NULL)
		return 92;
	//printf("files opened\n");
	while(1)
	{
		fread(&rem_status,sizeof(rem_status),1,status_add);
		//printf("feof : %d\n",feof(status_add));
		if(feof(status_add))
		{
			//printf("End of file reached about to write\n");
			fwrite(&move_status,sizeof(move_status),1,status_temp);
			//printf("streams closing\n");
			fclose(status_temp);
			fclose(status_add);
			//printf("streams closed\n");
			remove("status.status");
			rename("temp_status.status","status.status");
			return 1;
		}
		//printf("Name : %s , file name : %s\n",move_status.rem_move_name,rem_status.rem_move_name);
		if(strcmp(rem_status.rem_move_name,move_status.rem_move_name)==0)
		{
			//printf("overwriting existing file\n");
			fwrite(&move_status,sizeof(move_status),1,status_temp);
			while(1)
			{
				fread(&rem_status,sizeof(rem_status),1,status_add);
				if(feof(status_add))
					break;
				fwrite(&rem_status,sizeof(rem_status),1,status_temp);
			}
			//printf("streams closing\n");
			fclose(status_temp);
			fclose(status_add);
			//printf("streams closed\n");
			remove("status.status");
			rename("temp_status.status","status.status");
			return 1;
		}
		printf("move fianl status: %d \n",move_status.final_status);
		fwrite(&rem_status,sizeof(rem_status),1,status_temp);
	}
}/*{
	int iterator_name,name_l;
	height_status move_status,chk_status;
	name_l=gui_packet[element++];
	if(name_l>=30)
		return 91;
	for(iterator_name=0;iterator_name<name_l;)
		move_status.rem_move_name[iterator_name]=gui_packet[element++];
	if(iterator_name<30)
		move_status.rem_move_name[iterator_name]='\0';
	move_status.init_h=gui_packet[element++];
	move_status.init_h+=gui_packet[element++]*256;
	move_status.final_h=gui_packet[element++];
	move_status.final_h+=gui_packet[element++]*256;
	move_status.init_status=gui_packet[element++];
	move_status.final_status=gui_packet[element++];
	FILE *status_save,*read_status;
	read_status=fopen("temp_status.status"
	status_save=fopen("status.status","w");
	if(status_save==NULL)
		return 92;
	fwrite(&move_status,sizeof(move_status),1,status_save);
	fclose(status_save);
	return 1;
}*/
/*
int del_status()
{
	int iterator_name,name_l;
	height_status move_status,rem_status;
	name_l=gui_packet[element++];
	if(name_l>=30)
		return 91;
	for(iterator_name=0;iterator_name<name_l;)
		move_status.rem_move_name[iterator_name]=gui_packet[element++];
	if(iterator_name<30)
		move_status.rem_move_name[iterator_name]='\0';
	FILE *status_del,*status_temp;
	status_del=fopen("status.status","r");
	status_temp=fopen("temp_status.status","w");
	if(status_del==NULL)
		return 93;
	else if(status_temp==NULL)
		return 92;
	while(1)
	{
		fread(&rem_status,sizeof(rem_status),1,status_del);
		if(!feof(status_del))
			return 94;
		if(strcmp(rem_status.rem_move_name,move_status.rem_move_name)==0)
		{
			while(1)
			{
				fread(&rem_status,sizeof(rem_status),1,status_del);
				if(!feof(status_del))
					break;
				fwrite(&rem_status,sizeof(rem_status),1,status_temp);
			}
			fclose(status_temp);
			fclose(status_del);
			remove("status.status");
			rename("temp_status.status","status.status");
			return 1;
		}
		fwrite(&rem_status,sizeof(rem_status),1,status_temp);
	}
}
*/
int combine_save()
{
	printf("Entered Combine save\n");
	const int len=(int)gui_packet[element++];
	char name[len+15];
	if(acyut==3)
	strcpy(name,"acyut3/combine/");
	else if(acyut==4)
	strcpy(name,"acyut4/combine/");
	else
	return 70;
		char temp[len+1];
	int count=0,count1=0,len_f_name;
	//printf("Can declare array.....\n");
	while(count<len)
	temp[count++]=(char)gui_packet[element++];
	temp[len]='\0';
	strcat(name,temp);
	printf("NAME : %s\n",name);
	combine_f cmbine;
	cmbine.no_of_files=(int)gui_packet[element++];
	printf("NO. OF FILES : %d\n",cmbine.no_of_files);
	for(count=0;count<cmbine.no_of_files;count++)
	{
		len_f_name=(int)gui_packet[element++];
		if(len_f_name<30)
		{
			for(count1=0;count1<len_f_name;count1++)
			{
				cmbine.cmb_moves[count].file_name[count1]=(char)gui_packet[element++];
			}
			cmbine.cmb_moves[count].file_name[count1]='\0';
			//printf("NAME OF FILES %s\n",cmbine.cmb_moves[count].file_name);
			cmbine.cmb_moves[count].dlay=(int)gui_packet[element++];
		}
		else
		{
			printf("ERROR : file_name too long\n");
			return 71;
		}
	}
	FILE *cmb9;
	cmb9=fopen(name,"w");
	if(cmb9==NULL)
	{
		printf("ERROR : unable to open file\n");
		return 72;
	}
	/*else
	printf("FILE SAVED\n");*/
	if(fwrite(&cmbine,sizeof(combine_f),1,cmb9)!=1)
	{
		printf("ERROR : unable to write to file\n");
		return 73;
	}
	if(fclose(cmb9)!=0)
	{
		printf("ERROR : unable to close file\n");
		return 74;
	}
	printf("FILE CLOSED\n");
	return 1;
}

int combine_play(char *fn)
{
	printf("COMBINE PLAY CALLED\n");
	char *name;
	name=(char *)malloc(18+(sizeof(fn)/sizeof(char)));
	if(acyut==3)
	strcpy(name,"acyut3/combine/");
	else if(acyut==4)
	strcpy(name,"acyut4/combine/");
	else
	return 70;
	strcat(name,fn);
	printf("PLAYING COMBINE : %s\n",name);
	FILE *run_cmb9,*chk_f;
	run_cmb9=fopen(name,"r");
	//free(name);
	if(run_cmb9==NULL)
	{
		printf("ERROR : unable to open file\n");
		return 75;
	}
	/*else 
	printf("FILE FOUND\n");*/
	combine_f cmbine;
	char *move;
	int X=1,I=0,J;
	fread(&cmbine,sizeof(combine_f),1,run_cmb9);
	while(I<cmbine.no_of_files)
	{
		
		/*if(feof(run_cmb9))
		{
			printf("End of combine moves\n");
			if(fclose(run_cmb9)!=0)
			{
				printf("ERROR : unable to close file");
				return 76;
			}
			return 1;
		}*/
		move=(char *)malloc(sizeof(char));
		free(move);
		move=(char *)malloc(25+sizeof(cmbine.cmb_moves[I].file_name));
		if(acyut==3)
		strcpy(move,"acyut3/imu/calibration/");
		else if(acyut==4)
		strcpy(move,"acyut4/imu/calibration/");
		
		strcat(move,cmbine.cmb_moves[I].file_name);
		printf("cmbine filename : %s , size : %d\n",cmbine.cmb_moves[I].file_name,sizeof(cmbine.cmb_moves[I].file_name));
		//move[16+sizeof(cmbine.file_name)]='\0';
		strcat(move,".bin");
		printf("NAME of file : %s\n",move);
		chk_f=fopen(move,"r");
		//if((acyut==2)&&(chk_f!=NULL))
		if(chk_f!=NULL)
		{
			fclose(chk_f);
			free(move);
			readfiles(cmbine.cmb_moves[I].file_name);
		}
		else
		{
			free(move);
			move=(char *)malloc(15+sizeof(cmbine.cmb_moves[I].file_name));
			if(acyut==3)
				if(offset_status==1)
					strcpy(move,"acyut3/master_offseted_moves/");
				else
					strcpy(move,"acyut3/moves/");
			else if(acyut==4)
				if(offset_status==1)
					strcpy(move,"acyut4/master_offseted_moves/");
				else
					strcpy(move,"acyut4/moves/");
			strcat(move,cmbine.cmb_moves[I].file_name);
			X=act_fpacket(move);
			//printf("X=%d\n",X);
			free(move);
			if(X!=1)
			{
				fclose(run_cmb9);
				return X;
			}
		}
		usleep(16670*cmbine.cmb_moves[I++].dlay);
	}
	return 1;
}

inline int packet_recieve()
{
	printf("packet recieve called\n"); 
	byte temp=0,chksum=0;
	element=0;length=0;noat=0;
	while ((element<2)&&(noat<1000))
	{
		if(ftdi_read_data(&ftdic2,&temp,1)>0)
		{
			length=length+(((int)temp)*pow(256,element));
			chksum=chksum+temp;
			element++;
			noat=0;
		}
		else
		noat++;
	}
	printf("CHECKSUM:%d\tLength:%d\n",chksum,length);
	
	if (noat>=1000)
	{
		printf("noat err\n");
		return 10;
	}
	free(gui_packet);
	gui_packet=(byte *)malloc((length-1)*sizeof(byte));
	element=0;
	while(element<1&&noat<1000)
	{
		if (ftdi_read_data(&ftdic2,&temp,1)==1)
		{
			acyut=(int)temp;
			chksum+=temp;
			element++;
			noat=0;
		}
	}
	printf("CHECKSUM:%d\tACYUT:%d\n",chksum,acyut);
		
	if(noat>=1000)
	{
		printf("noat err\n");
		return 10;
	}
	element=0;
	while ((element<length-1)&&(noat<1000))
	{
		if(ftdi_read_data(&ftdic2,&temp,1)==1)
		{
			gui_packet[element++]=temp;
			printf("GUI_PACKET[%d]=%d\n",element-1,gui_packet[element-1]);
			if(element==length-1)
			continue;
			chksum=chksum+temp;
			noat=0;
			//printf("CHKSUM:%d\n",chksum);
	
		}
		else
		noat++;
	}
	if (noat>=1000)
	{
		printf("noat err\n");
		return 10;
		
	}
	else
		printf("got all data\n");
	printf("%d ?? %d\n",temp,255-chksum);
	if(255-chksum==temp)
	{
		printf("chksum correct\n");
		 return 1;
	}
	else
	{
		printf("chksum wrong\n");
		return 11;
	}
}

int act_fpacket(char *movname)
{
	printf("entering fpacket for playing file : %s \n",movname);
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
		printf("\nftdi done\n");
		usleep(16670);
		tx_packet[3]=mov.frames[i].num_motors*3+4;
		tx_packet[5]=mov.frames[i].address;
		adchksum=7+mov.frames[i].num_motors*3;
		tx_packet[adchksum]=0;
		
		//int tp_gen=0;
		printf("FRAME NO. %d\n",i);
		for(j=0;j<mov.frames[i].num_motors;j++)
		{
			tx_packet[7+j*3]=mov.frames[i].motorvalues[j].ID;
			tx_packet[8+j*3]=mov.frames[i].motorvalues[j].LB;
			tx_packet[9+j*3]=mov.frames[i].motorvalues[j].HB;
			//tx_packet[adchksum]+=tx_packet[7+j*3]+tx_packet[8+j*3]+tx_packet[9+j*3];
			tx_packet[adchksum]+=tx_packet[7+j*3]+tx_packet[8+j*3]+mov.frames[i].motorvalues[j].HB;
			//printf("%d\t%d\t%d\n",mov.frames[i].motorvalues[j].ID,mov.frames[i].motorvalues[j].LB,mov.frames[i].motorvalues[j].HB,tx_packet[adchksum]);
			printf("ID%d\tfile [%d] lb =%d\t hb=%d\t chksum=%d\n",tx_packet[7+j*3],i,tx_packet[8+j*3],tx_packet[9+j*3],tx_packet[adchksum]);
		}
		/*for (tp_gen = 0; tp_gen < mov.frames[i].num_motors*3+8; tp_gen += 1)
		{
			printf("%d\n",tx_packet[tp_gen]);
		}*/
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
		printf("about to write in ftdi\n");
		printf("Length %d\n",tx_packet[3]+4);
		
		////////////////////////////
		
		int dhairya=0;
		for(dhairya=0;dhairya<tx_packet[3]+4;dhairya++)
		{
			printf("DATA : %d\n",tx_packet[dhairya]);
			
		}
		ftdi_write_data(&ftdic1,tx_packet,tx_packet[3]+4);
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
		ftdi_write_data(&ftdic1,tx_packet,tx_packet[3]+4);
	}
	for(i=0;i<130;i++)
		last_tx_packet[i]=tx_packet[i];
		
	return err;

}

int save_master_offsets()
{
	printf("entered save_master_offset\n");
	offsets save_offset[NOM];
	int i=0;
	printf("int declared\n");
	for(i=0;i<NOM;i++)
	{
		printf("i=%d;motor=%d;\n",i,gui_packet[element]);
		save_offset[i].Id=gui_packet[element++];
		save_offset[i].sign=gui_packet[element++];
		save_offset[i].lB=gui_packet[element++];
		save_offset[i].hB=gui_packet[element++];
		printf("no %d, ID %d , sign %d, lb %d ,hb %d;\n",i,save_offset[i].Id,save_offset[i].sign,save_offset[i].lB,save_offset[i].hB);
	}
	printf("file open\n");
	FILE *save_master;
	save_master=fopen("master_offsets.offset","w");
	if(save_master==NULL)
		return 82;
	printf("file open successful\n");
	fwrite(&save_offset,sizeof(save_offset),NOM,save_master);
	printf("file write successful\n");
	printf("closing save_master_offset\n");
	fclose(save_master);
	return 1;
}

int apply_master_offsets()
{
	int rfl_error=1,chk_err=0;
	FILE *master_file;
	master_file=fopen("master_offsets.offset","r");
	if(master_file==NULL)
		return 81;
	fread(&master,sizeof(master),NOM,master_file);
	fclose(master_file);
	//printf("entered apply_master_offsets\n");
	if(acyut==4)	
		system("ls acyut4/moves > filelist.txt");
	else if(acyut==3)	
		system("ls acyut3/moves > filelist.txt");
	FILE *list_file;
	int end_flag=0,iter=0;
	char file_name[40],temp;
	system("chmod 777 filelist.txt");
	list_file=fopen("filelist.txt","r");
	if(list_file==NULL)
	{
		rfl_error=84;
	}
	else
	{
		while(1)
		{
			for(iter=0;iter<40;iter++)
			{
				fread(&temp,sizeof(temp),1,list_file);
				if(feof(list_file)!=0)
				{
					fclose(list_file);
					end_flag=1;
					break;
				}
				if(temp==10)
				{
					file_name[iter]='\0';
					break;
				}
				file_name[iter]=temp;
			}
			if(end_flag==1)
			{
				break;
			}
			printf("filename : %s\n",file_name);
			if(strcmp(file_name,"filename.txt")!=0)
				chk_err=apply_master_offsets_to_file(file_name);		
		}
		if(rfl_error>1)
			rfl_error=85;
	}
	offset_status=0;  //1
	return rfl_error;
}

int apply_master_offsets_to_file(char *f_name)
{
	printf("entered apply_master_offsets_to_file\n");
	FILE *old_file,*new_file;
	move of_move;
	int lb,hb,i,j,k=0,noat=0;
	char old_name[100],new_name[100];
	//printf("defined all variables\n");
	if(acyut==4)
	{
		strcpy(old_name,"acyut4/moves/");
		strcpy(new_name,"acyut4/master_offseted_moves/");
	}
	else if(acyut==3)
	{
		strcpy(old_name,"acyut3/moves/");
		strcpy(new_name,"acyut3/master_offseted_moves/");
	}
	//printf("copied names\n");
	strcat(old_name,f_name);
	strcat(new_name,f_name);
	printf("new name : %s\n",new_name);
	printf("old name : %s\n",old_name);
	old_file=fopen(old_name,"r");
//	free(old_name);
	if(old_file==NULL)
	{
		printf("unable to open old file");
		printf("old name : %s\n",old_name);
		return 1;
	}
	fread(&of_move,sizeof(of_move),1,old_file);
	for(i=0;i<of_move.noframes;i++)
	{
		if(of_move.frames[i].address==0x1c||of_move.frames[i].address==0x1a)
		{
			i++;
			continue;
		}
		k=0;
		for(j=0;j<NOM;j++)
		{
			//printf("i,j : %d,%d\n",i,j);
			printf("of_move.frames[%d].motorvalues[%d].ID:%d,,,master[%d].Id:%d\n",i,j,of_move.frames[i].motorvalues[j].ID,k,master[k].Id);
			noat=0;
			while(of_move.frames[i].motorvalues[j].ID!=master[k].Id)
			{
				if(k<NOM-1)
						k++;
				else
				{
					k=0;
					noat++;
					if(noat==2)
					break;
				}
				printf("of_move.frames[%d].motorvalues[%d].ID:%d,,,master[%d].Id:%d\n",i,j,of_move.frames[i].motorvalues[j].ID,k,master[k].Id);
			}
			if(noat==2)
			{
				continue;
			}
			if(master[k].sign==0)
			{
				
				hb=(((int)of_move.frames[i].motorvalues[j].LB+((int)of_move.frames[i].motorvalues[j].HB)*256)-((int)master[k].lB+((int)master[k].hB*256)))/256;
				lb=(((int)of_move.frames[i].motorvalues[j].LB+((int)of_move.frames[i].motorvalues[j].HB)*256)-((int)master[k].lB+((int)master[k].hB*256)))%256;
			}
			else if(master[k].sign==1)
			{
				hb=(((int)of_move.frames[i].motorvalues[j].LB+((int)of_move.frames[i].motorvalues[j].HB)*256)+((int)master[k].lB+((int)master[k].hB)*256))/256;
				lb=(((int)of_move.frames[i].motorvalues[j].LB+((int)of_move.frames[i].motorvalues[j].HB)*256)+((int)master[k].lB+((int)master[k].hB)*256))%256;
			}
			printf("hb=%d,lb=%d,\n",hb,lb);
			of_move.frames[i].checksum=255-of_move.frames[i].checksum;
			of_move.frames[i].checksum=of_move.frames[i].checksum-(of_move.frames[i].motorvalues[j].LB+of_move.frames[i].motorvalues[j].HB);
			of_move.frames[i].motorvalues[j].LB=(byte)lb;
			of_move.frames[i].motorvalues[j].HB=(byte)hb;
			of_move.frames[i].checksum=of_move.frames[i].checksum+(of_move.frames[i].motorvalues[j].LB+of_move.frames[i].motorvalues[j].HB);
			of_move.frames[i].checksum=255-of_move.frames[i].checksum;
		}
	}
	fclose(old_file);
	new_file=fopen(new_name,"w");
	if(new_file==NULL)
	{
		printf("new_file not opened\n");
		return 1;
	}
	else
		fwrite(&of_move,sizeof(of_move),1,new_file);
		
	system("chmod 777 acyut4/master_offseted_moves/genAcYut");
	fclose(new_file);
	//printf("exiting apply_offsets_to_file\n");
	return 0;
}



void weight_shift(int side,int thrsh)
{
	int s=0,preload=0,rpreload=0,predir=0,rpredir=0,load=0,noat=0,change=0,i=0,val=0,adchksum=7+NOM*3,t=0,mov=1,moved=0;
	int mot=0,revhip=0,revmot=0;
	int value[33],dir[33],move[33];
	int read[9];
	int exit=0;
	
	if(side==1)
	{
		change=5;
		mot=20;
		revmot=0;
		revhip=5;
		//fval1=2000;
		//fval2=1350;
	}
	else if(side==0)
	{
		change=-5;
		mot=0;
		revmot=20;
		revhip=25;
		//fval1=1350;
		//fval2=2000;
	}
	
	int nof=10;
	int offset[]={-4,120,304,341,0,0,300,0,0,0,0,0,254,0,0,-119,0,0,0,0,-19,321,304,341,0,0,-279,0,0,0,0,0,254};
	while(noat<5&&t<nof)
	{
	
		usleep(16670);
		/*read[0]=0;read[1]=20,read[2]=1,read[3]=21,read[4]=12,read[5]=22,read[6]=32,read[7]=5,read[8]=25;
		for(i=0;i<9;i++)
		{
			value[read[i]]=read_load(read[i]);
			dir[read[i]]=value[read[i]]%10;
			move[read[i]]=(value[read[i]]%100)/10;
			value[read[i]]=pow(-1,dir[read[i]])*value[read[i]]/100;
			printf("ID %d\t%d\t%d\n",read[i],value[read[i]],dir[read[i]]); 	
		}
		printf("LEFT LEG %d\n",value[0]+value[1]+value[12]+value[5]);
		printf("RIGHT LEG %d\n",value[20]+value[21]+value[32]+value[25]);
		
		if(dir[mot]==9)
		{
			noat++;
			printf("NOAT %d \n",noat);
			printf("SKIPPING\n");
			continue;
		}
		else
		{
			load=value[mot];
			mov=move[mot];
		}
		
		printf("Load %d : %d MOV : %d\n",mot,load,mov);
		
		if(mov==1)
		{
			printf("MOTOR MOVING\n");
			continue;
		}
		if(t==0)
		{
			preload=value[21];
			rpreload=value[revmot];
			predir=dir[mot];
			rpredir=dir[revmot];
		}
		//if(t>1&&((value[mot]-preload<0)&&(value[revmot]-rpreload<0)))
		if((value[21]!=preload))
		exit++;
		if(exit==1)
		break;
		/*if(load==preload)
		{
			s++;
			printf("NO LOAD CHANGE\n");
			if(s<=10)
			continue;
			else
			s=0;
		}
		else
		{
			if(s!=0)
			printf("S %d\n",s);
			
			s=0;
		}*/
		/*if(s==2)
		{
			s=0;
			continue;
		}*/
		for(i=0;i<128;i++)
		{
			tx_packet[i]=last_tx_packet[i];
			//printf("%d\t",tx_packet[i]);
		}
		//loads();
		for(i=0;i<NOM;i++)
		{
			if(tx_packet[7+i*3]==0)
			{
				val=tx_packet[8+i*3]+tx_packet[9+i*3]*256;
				tx_packet[adchksum]+=tx_packet[8+i*3]+tx_packet[9+i*3];
				//val=scurve(last_tx_packet[8+i*3]+last_tx_packet[9+i*3]*256,fval1,t,tot);
				val+=change;
				tx_packet[8+i*3]=val%256;
				tx_packet[9+i*3]=val/256;
				tx_packet[adchksum]-=tx_packet[8+i*3]+tx_packet[9+i*3];
				//printf("VAL %d \t %d\n",tx_packet[7+i*3],val);				
			}
			if(tx_packet[7+i*3]==20)
			{
				val=tx_packet[8+i*3]+tx_packet[9+i*3]*256;
				tx_packet[adchksum]+=tx_packet[8+i*3]+tx_packet[9+i*3];
				//val=scurve(last_tx_packet[8+i*3]+last_tx_packet[9+i*3]*256,fval2,t,tot);
				val-=change;
				tx_packet[8+i*3]=val%256;
				tx_packet[9+i*3]=val/256;
				tx_packet[adchksum]-=(tx_packet[8+i*3]+tx_packet[9+i*3]);
				//printf("VAL %d \t %d\n",tx_packet[7+i*3],val);
			}
			if(tx_packet[7+i*3]==5)
			{
				val=tx_packet[8+i*3]+tx_packet[9+i*3]*256;
				tx_packet[adchksum]+=tx_packet[8+i*3]+tx_packet[9+i*3];
				//val=scurve(last_tx_packet[8+i*3]+last_tx_packet[9+i*3]*256,fval1,t,tot);
				val+=change;
				tx_packet[8+i*3]=val%256;
				tx_packet[9+i*3]=val/256;
				tx_packet[adchksum]-=tx_packet[8+i*3]+tx_packet[9+i*3];
				//printf("VAL %d \t %d\n",tx_packet[7+i*3],val);				
			}
			if(tx_packet[7+i*3]==25)
			{
				val=tx_packet[8+i*3]+tx_packet[9+i*3]*256;
				tx_packet[adchksum]+=tx_packet[8+i*3]+tx_packet[9+i*3];
				//val=scurve(last_tx_packet[8+i*3]+last_tx_packet[9+i*3]*256,fval2,t,tot);
				val-=change;
				tx_packet[8+i*3]=val%256;
				tx_packet[9+i*3]=val/256;
				tx_packet[adchksum]-=(tx_packet[8+i*3]+tx_packet[9+i*3]);
				//printf("VAL %d \t %d\n",tx_packet[7+i*3],val);
			}
		}
		
		//printf("\n");
		if(ftdi_write_data(&ftdic1,tx_packet,tx_packet[3]+4)>0)
		printf("PACKET SENT %d\n",t++);
		else
		printf("ERROR SENDING PACKET\n");
		
		for(i=0;i<128;i++)
		{	
			last_tx_packet[i]=tx_packet[i];
			//printf("%d\t",tx_packet[i]);
		}		
		//preload=value[mot];
		//rpreload=value[revmot];	
	}
	//preload=load;
		
	t=0;
	//sleep(2);
	
	read[0]=0;read[1]=20,read[2]=1,read[3]=21,read[4]=12,read[5]=22,read[6]=32,read[7]=5,read[8]=25;
	do
	{
		printf("Reading\n");
		for(i=0;i<9;i++)
		{	
			value[read[i]]=read_load(read[i]);
			dir[read[i]]=value[read[i]]%10;
			move[read[i]]=(value[read[i]]%100)/10;
			value[read[i]]=pow(-1,dir[read[i]])*value[read[i]]/100;
			printf("ID %d\t%d\t%d\n",read[i],value[read[i]],dir[read[i]]); 	
		}
	}
	while(dir[mot]==9);
	
	noat=0;
	preload=value[mot];
	int flag=0;
	//change=change/abs(change);
	int change2=change;
	
	int gnx=3;
	float gx[]={390,370,390};
	float gxt[]={0,0.5,1};
	int gny=2;
	float gy[]={0,50};
	float gyt[]={0,1};

	int h=0;
	float* ox;
	float outputx[20];
	ox=generatespline(gnx-1,gxt,gx);
	for(h=0;h<20;h++)
	outputx[h]=ox[h];
	float* oy;
	float outputy[20];
	oy=generatespline(gny-1,gyt,gy);
	for(h=0;h<20;h++)
	outputy[h]=oy[h];
	
	float x,y,z=0,zang=0;
	float* fval;
	printf("SLEEPING....\n");
	sleep(5);
	
	int k=0;
	float t1=0;
	int j=0;
	float pcent=50;
	pcent=pcent/100;
	float tot=1;
	int prev[4];
	for(;t1<tot;t1+=((float)1/60))
	{
		for(i=0;i<128;i++)
		{
			tx_packet[i]=last_tx_packet[i];
			//printf("%d\t",tx_packet[i]);
		}
	
		printf("Time : %lf\n",t1);
		
		if(t1>=gxt[k+1])
		k++;
		if(t1>=gyt[j+1])
		j++;
		//printf("k : %d j : %d\n",k,j);
		//printf("%lf\t%lf\t%lf\t%lf\n",outputx[k*4],outputx[k*4+1],outputx[k*4+2],outputx[k*4+3]);
		//printf("%lf\t%lf\t%lf\t%lf\n",outputy[j*4],outputy[j*4+1],outputy[j*4+2],outputy[j*4+3]);
		
		x=outputx[k*4]+outputx[k*4+1]*pow(t1-gxt[k],1)+outputx[k*4+2]*pow(t1-gxt[k],2)+outputx[k*4+3]*pow(t1-gxt[k],3);
		y=outputy[j*4]+outputy[j*4+1]*pow(t1-gyt[j],1)+outputy[j*4+2]*pow(t1-gxt[j],2)+outputy[j*4+3]*pow(t1-gyt[j],3);
		//printf("X : %lf\tY : %lf\n",x,y);		
		fval=generateik(x,y,z,zang);
				
		read[0]=0;read[1]=20,read[2]=1,read[3]=21,read[4]=12,read[5]=22,read[6]=32,read[7]=5,read[8]=25;
		if(t1<pcent*tot)
		{
			for(i=0;i<9;i++)
			{
				value[read[i]]=read_load(read[i]);
				dir[read[i]]=value[read[i]]%10;
				move[read[i]]=(value[read[i]]%100)/10;
				value[read[i]]=pow(-1,dir[read[i]])*value[read[i]]/100;
				printf("ID %d\t%d\t%d\t%d\n",read[i],value[read[i]],dir[read[i]],move[read[i]]); 	
			}
			
			if(dir[mot]==9)
			{
				noat++;
				printf("NOAT %d \n",noat);
				printf("SKIPPING\n");
				t1=t1-((float)1/60);
				printf("Time : %lf\n",t1);
				continue;
			}
			if(move[mot]==1)
			{
				printf("MOTOR MOVING\n");
				t1=t1-((float)1/60);
				printf("Time : %lf\n",t1);
				continue;
			}
			
			if(preload-10>value[mot])
			{
				flag=1;
				change2=-(preload-value[mot])/15;
			}
			else if(preload+10<value[mot])
			{
				flag=1;
			change2=(preload-value[mot])/15;
			}
			else
			flag=0;
		}
		else
		flag=0;
		
		float chk_val=t1-(pcent*tot);
		
		if((int)(chk_val*100)==0)
		{
			printf("HELLO....................................\n");
			for(i=0;i<NOM;i++)
			{
				if(tx_packet[7+i*3]==0)
				{
					prev[0]=last_tx_packet[8+i*3]+last_tx_packet[9+i*3]*256;
					printf("PREV%d\n",prev[0]);
				} 
				if(tx_packet[7+i*3]==20)
				prev[1]=last_tx_packet[8+i*3]+last_tx_packet[9+i*3]*256;
				
				if(tx_packet[7+i*3]==5)
				prev[2]=last_tx_packet[8+i*3]+last_tx_packet[9+i*3]*256;
				
				if(tx_packet[7+i*3]==25)
				prev[3]=last_tx_packet[8+i*3]+last_tx_packet[9+i*3]*256;
			}
		}
		
			
		for(i=0;i<NOM;i++)
		{
			if(flag==1)
			{
				if(tx_packet[7+i*3]==0)
				{
					val=tx_packet[8+i*3]+tx_packet[9+i*3]*256;
					tx_packet[adchksum]+=tx_packet[8+i*3]+tx_packet[9+i*3];
					//val=scurve(last_tx_packet[8+i*3]+last_tx_packet[9+i*3]*256,fval1,t,tot);
					val+=change2;
					tx_packet[8+i*3]=val%256;
					tx_packet[9+i*3]=val/256;
					tx_packet[adchksum]-=tx_packet[8+i*3]+tx_packet[9+i*3];
					moved+=change2;
					printf("VAL %d \t %d\n",tx_packet[7+i*3],val);				
				}
				if(tx_packet[7+i*3]==5)
				{
					val=tx_packet[8+i*3]+tx_packet[9+i*3]*256;
					tx_packet[adchksum]+=tx_packet[8+i*3]+tx_packet[9+i*3];
					//val=scurve(last_tx_packet[8+i*3]+last_tx_packet[9+i*3]*256,fval1,t,tot);
					val+=change2;
					tx_packet[8+i*3]=val%256;
					tx_packet[9+i*3]=val/256;
					tx_packet[adchksum]-=tx_packet[8+i*3]+tx_packet[9+i*3];
					//printf("VAL %d \t %d\n",tx_packet[7+i*3],val);				
				}
				if(tx_packet[7+i*3]==20)
				{
					val=tx_packet[8+i*3]+tx_packet[9+i*3]*256;
					tx_packet[adchksum]+=tx_packet[8+i*3]+tx_packet[9+i*3];
					//val=scurve(last_tx_packet[8+i*3]+last_tx_packet[9+i*3]*256,fval2,t,tot);
					val-=change2;
					tx_packet[8+i*3]=val%256;
					tx_packet[9+i*3]=val/256;
					tx_packet[adchksum]-=(tx_packet[8+i*3]+tx_packet[9+i*3]);
					//printf("VAL %d \t %d\n",tx_packet[7+i*3],val);
				}
				if(tx_packet[7+i*3]==25)
				{
					val=tx_packet[8+i*3]+tx_packet[9+i*3]*256;
					tx_packet[adchksum]+=tx_packet[8+i*3]+tx_packet[9+i*3];
					//val=scurve(last_tx_packet[8+i*3]+last_tx_packet[9+i*3]*256,fval2,t,tot);
					val-=change2;
					tx_packet[8+i*3]=val%256;
					tx_packet[9+i*3]=val/256;
					tx_packet[adchksum]-=(tx_packet[8+i*3]+tx_packet[9+i*3]);
					//printf("VAL %d \t %d\n",tx_packet[7+i*3],val);
				}
			}
			if(t1>=pcent*tot)
			{
				if(tx_packet[7+i*3]==0)
				{
					val=tx_packet[8+i*3]+tx_packet[9+i*3]*256;
					tx_packet[adchksum]+=tx_packet[8+i*3]+tx_packet[9+i*3];
					val=scurve(prev[0],prev[0]-moved-change*nof,t1-tot*pcent,tot*(1-pcent));
					tx_packet[8+i*3]=val%256;
					tx_packet[9+i*3]=val/256;
					tx_packet[adchksum]-=tx_packet[8+i*3]+tx_packet[9+i*3];
					printf("VAL %d \t %d\n",tx_packet[7+i*3],val);				
				}
				if(tx_packet[7+i*3]==5)
				{
					val=tx_packet[8+i*3]+tx_packet[9+i*3]*256;
					tx_packet[adchksum]+=tx_packet[8+i*3]+tx_packet[9+i*3];
					val=scurve(prev[2],prev[2]-moved-change*nof,t1-tot*pcent,tot*(1-pcent));
					tx_packet[8+i*3]=val%256;
					tx_packet[9+i*3]=val/256;
					tx_packet[adchksum]-=tx_packet[8+i*3]+tx_packet[9+i*3];
					//printf("VAL %d \t %d\n",tx_packet[7+i*3],val);				
				}
				if(tx_packet[7+i*3]==20)
				{
					val=tx_packet[8+i*3]+tx_packet[9+i*3]*256;
					tx_packet[adchksum]+=tx_packet[8+i*3]+tx_packet[9+i*3];
					val=scurve(prev[1],prev[1]+moved+change*nof,t1-tot*pcent,tot*(1-pcent));
					tx_packet[8+i*3]=val%256;
					tx_packet[9+i*3]=val/256;
					tx_packet[adchksum]-=(tx_packet[8+i*3]+tx_packet[9+i*3]);
					//printf("VAL %d \t %d\n",tx_packet[7+i*3],val);
				}
				if(tx_packet[7+i*3]==25)
				{
					val=tx_packet[8+i*3]+tx_packet[9+i*3]*256;
					tx_packet[adchksum]+=tx_packet[8+i*3]+tx_packet[9+i*3];
					val=scurve(prev[3],prev[3]+moved+change*nof,t1-tot*pcent,tot*(1-pcent));
					tx_packet[8+i*3]=val%256;
					tx_packet[9+i*3]=val/256;
					tx_packet[adchksum]-=(tx_packet[8+i*3]+tx_packet[9+i*3]);
					//printf("VAL %d \t %d\n",tx_packet[7+i*3],val);
				}
			}						
			if(tx_packet[7+i*3]==revmot+1)
			{
				val=tx_packet[8+i*3]+tx_packet[9+i*3]*256;
				tx_packet[adchksum]+=tx_packet[8+i*3]+tx_packet[9+i*3];
				val=fval[1]+offset[tx_packet[7+i*3]];
				tx_packet[8+i*3]=val%256;
				tx_packet[9+i*3]=val/256;
				tx_packet[adchksum]-=tx_packet[8+i*3]+tx_packet[9+i*3];
				//printf("VAL %d \t %d\n",tx_packet[7+i*3],val);				
			}
			if(tx_packet[7+i*3]==revmot+3)
			{
				val=tx_packet[8+i*3]+tx_packet[9+i*3]*256;
				tx_packet[adchksum]+=tx_packet[8+i*3]+tx_packet[9+i*3];
				val=fval[3]+offset[tx_packet[7+i*3]];
				tx_packet[8+i*3]=val%256;
				tx_packet[9+i*3]=val/256;
				tx_packet[adchksum]-=tx_packet[8+i*3]+tx_packet[9+i*3];
				//printf("VAL %d \t %d\n",tx_packet[7+i*3],val);				
			}
			if(tx_packet[7+i*3]==revmot+2||tx_packet[7+i*3]==revmot+12)
			{
				val=tx_packet[8+i*3]+tx_packet[9+i*3]*256;
				tx_packet[adchksum]+=tx_packet[8+i*3]+tx_packet[9+i*3];
				val=fval[2]+offset[tx_packet[7+i*3]];
				tx_packet[8+i*3]=val%256;
				tx_packet[9+i*3]=val/256;
				tx_packet[adchksum]-=(tx_packet[8+i*3]+tx_packet[9+i*3]);
				//printf("VAL %d \t %d\n",tx_packet[7+i*3],val);
			}
		}
		if(ftdi_write_data(&ftdic1,tx_packet,tx_packet[3]+4)>0)
		printf("PACKET SENT %d\n",t++);
		else
		printf("ERROR SENDING PACKET\n");
		
		usleep(16670);
		for(i=0;i<128;i++)
		{	
			last_tx_packet[i]=tx_packet[i];
			//printf("%d\t",tx_packet[i]);
		}		
		
	}
		
}
	

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

float* generateik(float x,float y,float z,float z_ang)
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
  q[4]=q4;

// convert to EX106+............................................
  int tp;
  for(tp=0;tp<5;tp++)
     q[tp]= (float) (  q[tp]*280  - 15*4096  )/250;

  return q;
}

	
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
		if(ftdi_write_data(&ftdic1,snd_packet,snd_packet[3]+4)==8)
		{
			noat=0;
			chksum=0xff;
			while(noat<10)
			{
				if(ftdi_read_data(&ftdic1,&val,1)==1)
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
					if(ftdi_read_data(&ftdic1,rcv_packet,10)==10)
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

void check()
{
	int val1=read_load(0);
	int val2=read_load(20);
	int dir1=val1%10;
	int dir2=val2%10;
	int val3=read_load(1);
	int val4=read_load(21);
	int dir3=val1%10;
	int dir4=val2%10;
	val1=val1/100;
	val2=val2/100;
	val3=val3/100;
	val4=val4/100;
	printf("%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t\n",dir1,val1,dir2,val2,dir3,val3,dir4,val4); 	
}
void loads()
{
	int read[]={0,20,1,21,12,22,32,5,25};
	int val[NOM],dir[NOM];
	int i=0;
	for(i=0;i<9;i++)
	{
		val[read[i]]=read_load(read[i]);
		dir[read[i]]=val[read[i]]%10;
		val[read[i]]=val[read[i]]/100;
		printf("ID %d\t%d\t%d\n",read[i],val[read[i]],dir[read[i]]); 	
	}
	printf("LEFT LEG %d\n",val[0]+val[1]+val[12]+val[5]);
	printf("RIGHT LEG %d\n",val[20]+val[21]+val[32]+val[25]);
}


int scurve(int in,int fi,float t, float tot)
{
	float frac=(float)t/(float)tot;
	float retfrac=frac*frac*(3-2*frac);
	int mval=in + (fi-in)*retfrac;
	return mval;
}

void check2(int side)
{
	FILE *lzl;
	lzl=fopen("loads.txt","wr");
	int i=0,j=0,val=0,adchksum=7+NOM*3,change=0,mot,revmot,fval1=0,fval2=0,tot=60,t=0;
	int read[10],value[33],dir[33],move[33];
	if(side==1)
	{
		change=5;
		mot=20;
		revmot=0;
		fval1=1800;
		fval2=1550;
	}
	else if(side==0)
	{
		change=-5;
		mot=0;
		revmot=20;
		fval1=1550;
		fval2=1800;
	}
		
	fprintf(lzl,"x\t0\t20\t1\t21\t12\t32\t3\t23\t5\t25\n");
	while(t<15)
	{
		
		fprintf(lzl,"%d\t",t);
		read[0]=0;read[1]=20,read[2]=1,read[3]=21,read[4]=12,read[5]=32,read[6]=3,read[7]=23,read[8]=5,read[9]=25;
		for(i=0;i<10;i++)
		{
			value[read[i]]=read_load(read[i]);
			dir[read[i]]=value[read[i]]%10;
			move[read[i]]=(value[read[i]]%100)/10;
			value[read[i]]=value[read[i]]/100;
			printf("ID %d\t%d\t%d\n",read[i],value[read[i]],dir[read[i]]); 	
			fprintf(lzl,"%lf\t",pow(-1,dir[read[i]])*value[read[i]]);
		}
		fprintf(lzl,"\n");
		for(i=0;i<128;i++)
		{
		//	printf("%d\t",last_tx_packet[i]);
			tx_packet[i]=last_tx_packet[i];
			///printf("%d\t",tx_packet[i]);
		}	
	
		t++;
		usleep(16670);
		for(i=0;i<NOM;i++)
		{
			if(tx_packet[7+i*3]==0||tx_packet[7+i*3]==5)
			{
				val=tx_packet[8+i*3]+tx_packet[9+i*3]*256;
				tx_packet[adchksum]+=tx_packet[8+i*3]+tx_packet[9+i*3];
				//val=scurve(last_tx_packet[8+i*3]+last_tx_packet[9+i*3]*256,fval1,t,tot);
				val+=change;
				tx_packet[8+i*3]=val%256;
				tx_packet[9+i*3]=val/256;
				tx_packet[adchksum]-=tx_packet[8+i*3]+tx_packet[9+i*3];
			//	printf("VAL %d \t %d\n",tx_packet[7+i*3],val);				
			}
			if(tx_packet[7+i*3]==20||tx_packet[7+i*3]==25)
			{
				val=tx_packet[8+i*3]+tx_packet[9+i*3]*256;
				tx_packet[adchksum]+=tx_packet[8+i*3]+tx_packet[9+i*3];
				//val=scurve(last_tx_packet[8+i*3]+last_tx_packet[9+i*3]*256,fval2,t,tot);
				val-=change;
				tx_packet[8+i*3]=val%256;
				tx_packet[9+i*3]=val/256;
				tx_packet[adchksum]-=tx_packet[8+i*3]+tx_packet[9+i*3];
			//	printf("VAL %d \t %d\n",tx_packet[7+i*3],val);
			}
		}
		for(i=0;i<128;i++)
		{
			last_tx_packet[i]=tx_packet[i];
			//printf("%d\t",tx_packet[i]);
		}
		
		//check();
		
		if(ftdi_write_data(&ftdic1,tx_packet,tx_packet[3]+4)>0)
		printf("PACKET SENT %d\n",t);
		else
		printf("ERROR SENDING PACKET\n");
	}
	fclose(lzl);
		
}	


int dribble(int leg)
{
	
	byte im[8];
	float change[3];
	float tchange[3];
	int count=0;
	int aftr_frames=2;
	int noat=0,a=0,b=0,c=0;
	int nof=0;
	float present_boundary[3]={0};
	float future_boundary[3]={0};


	int offset[33];
	offset[0]=-4; offset[1]=125; offset[2]=304; offset[3]=341; offset[4]=0; offset[5]=0; offset[6]=300; offset[7]=0; offset[8]=0; offset[9]=0; offset[10]=0; offset[11]=0; offset[12]=254; offset[13]=0; offset[14]=0; offset[15]=-119; offset[16]=0; offset[17]=0; offset[18]=0; offset[19]=0; offset[20]=-19; offset[21]=326; offset[22]=304; offset[23]=341; offset[24]=0; offset[25]=0; offset[26]=-279; offset[27]= 0; offset[28]= 0; offset[29]=0; offset[30]=0; offset[31]=0; offset[32]=254;
	int adchksum=7+NOM*3;
	
	int k=0,j=0,i=0,h=0;
	int flag=0;
	float x,z,zr;
	
	int mot,revmot;
	float t=0;
	int value;
	
	float* fval;
	float val[5],valr[5]; 
	int gnx=5;
	float gx[]={390,390,360,390,390};
	float gxt[]={0,0.25,0.37,0.50,0.75};
	
	/*if(leg==0)
	{
		gxt[3]=0.45;
	}*/
	float* ox;
	float outputx[20];
	ox=generatespline(gnx-1,gxt,gx);
	for(h=0;h<4;h++)
	{
		if(gx[h]==gx[h+1])
		{
			outputx[h*4]=gx[h];
			outputx[h*4+1]=0;
			outputx[h*4+2]=0;
			outputx[h*4+3]=0;
		}
		else
		{
			outputx[h*4]=ox[h*4];
			outputx[h*4+1]=ox[h*4+1];
			outputx[h*4+2]=ox[h*4+2];
			outputx[h*4+3]=ox[h*4+3];
		}
	}
	
	
	int gnzr=4;
	float gzr[]={0,-30,-30,0};
	float gztr[]={0,0.25,0.50,0.75};
	
	/*if(leg==0)
	{
		gzr[2]=-40;
		gzr[1]=-40;
		gztr[2]=0.6;
	}*/
	
	float* ozr;
	float outputzr[20];
	ozr=generatespline(gnzr-1,gztr,gzr);
	for(h=0;h<4;h++)
	{
		if(gzr[h]==gzr[h+1])
		{
			outputzr[h*4]=gzr[h];
			outputzr[h*4+1]=0;
			outputzr[h*4+2]=0;
			outputzr[h*4+3]=0;
		}
		else
		{
			outputzr[h*4]=ozr[h*4];
			outputzr[h*4+1]=ozr[h*4+1];
			outputzr[h*4+2]=ozr[h*4+2];
			outputzr[h*4+3]=ozr[h*4+3];
		}
	}
	
	int gnz=4;
	float gz[]={0,30,30,0};
	float gzt[]={0,0.25,0.50,0.75};
	
	/*if(leg==0)
	{
		gz[2]=40;
		gz[1]=40;
		gzt[2]=0.6;
	}*/
	float* oz;
	float outputz[20];
	oz=generatespline(gnz-1,gzt,gz);
	for(h=0;h<4;h++)
	{
		if(gz[h]==gz[h+1])
		{
			outputz[h*4]=gz[h];
			outputz[h*4+1]=0;
			outputz[h*4+2]=0;
			outputz[h*4+3]=0;
		}
		else
		{
			outputz[h*4]=oz[h*4];
			outputz[h*4+1]=oz[h*4+1];
			outputz[h*4+2]=oz[h*4+2];
			outputz[h*4+3]=oz[h*4+3];
		}
	}
	
	if(leg==0)
	{
		mot=0;
		revmot=20;
		read_bounds("a4dribble_left2");
		
	}
	else if(leg==1)
	{
		mot=20;
		revmot=0;
		read_bounds("a4dribble_right2");
	}
	else
	{
		printf("TAKE LITE\n");
		return EXIT_FAILURE;
	}
	
	byte cp=32,cpr=32;
	byte compliance_packet[]={0xff,0xff,0xfe,0x28,0x83,0x1c,0x02,mot,cp,cp,mot+1,cp,cp,mot+2,cp,cp,mot+12,cp,cp,mot+3,cp,cp,mot+5,cp,cp,revmot,cpr,cpr,revmot+1,cpr,cpr,revmot+2,cpr,cpr,revmot+12,cpr,cpr,revmot+3,cpr,cpr,revmot+5,cpr,cpr,0xff};
	for(h=2;h<43;h++)
	compliance_packet[43]-=compliance_packet[h];
	
	if(ftdi_write_data(&ftdic1,compliance_packet,compliance_packet[3]+4)>0)
	printf("Compliance Written\n");
	else
	printf("ERROR : Compliance\n");
	
	usleep(16670);
	for(;t<=gxt[gnx-1];t+=((float)1/60))
	{
		nof=(int)(t*60);
		for(i=0;i<128;i++)
		{
			tx_packet[i]=last_tx_packet[i];
			//printf("%d\t",tx_packet[i]);
		}
		
		printf("Time : %lf Frame : %d\n",t,(int)(t*60));
	
		if((int)((t-0.60)*100)==0)
		{
			cp=32;
			cpr=32;
			for(h=2;h<43;h++)
			compliance_packet[43]-=compliance_packet[h];
			
			if(ftdi_write_data(&ftdic1,compliance_packet,compliance_packet[3]+4)>0)
			printf("Compliance Written\n");
			else
			printf("ERROR : Compliance\n");
			usleep(16670);
		}
		
				
		if(t>=gxt[k+1])
		k++;
		if(t>=gzt[j+1])
		j++;
		//printf("k : %d j : %d\n",k,j);
		//printf("%lf\t%lf\t%lf\t%lf\n",outputx[k*4],outputx[k*4+1],outputx[k*4+2],outputx[k*4+3]);
		//printf("%lf\t%lf\t%lf\t%lf\n",outputy[j*4],outputy[j*4+1],outputy[j*4+2],outputy[j*4+3]);
		
		x=outputx[k*4]+outputx[k*4+1]*pow(t-gxt[k],1)+outputx[k*4+2]*pow(t-gxt[k],2)+outputx[k*4+3]*pow(t-gxt[k],3);
		z=outputz[j*4]+outputz[j*4+1]*pow(t-gzt[j],1)+outputz[j*4+2]*pow(t-gzt[j],2)+outputz[j*4+3]*pow(t-gzt[j],3);
		zr=outputzr[j*4]+outputzr[j*4+1]*pow(t-gztr[j],1)+outputzr[j*4+2]*pow(t-gztr[j],2)+outputzr[j*4+3]*pow(t-gztr[j],3);
		printf("LEG : %d\tX : %lf\tZ : %lf\n",leg,x,z);		
		fval=generateik(x,0,z,0);
		for(h=0;h<5;h++)
		val[h]=fval[h];
		fval=generateik(390,0,zr,0);
		for(h=0;h<5;h++)
		valr[h]=fval[h];
		change[0]=0;
		change[1]=0;
		imu_init=9;
		if(nof%aftr_frames==0&&imu_init==1)
		{
			imucal();
			if(nof>aftr_frames)
			{
				for(a=0;a<2;a++)
				{
					present_boundary[a]=local_boundaries[(nof/aftr_frames)-1][a]+(boundaries[(nof/aftr_frames)][a]-boundaries[(nof/aftr_frames)-1][a]);
					tchange[a]=(avg_angle[a]-present_boundary[a]);		
					future_boundary[a]=local_boundaries[(nof/aftr_frames)-1][a]+(boundaries[(nof/aftr_frames)+1][a]-boundaries[(nof/aftr_frames)-1][a]);
					
					//printf("Diff Boundary %d : %lf\n",a,boundaries[(nof/aftr_frames)][a]-boundaries[(nof/aftr_frames)-1][a]);
					//printf("P Boundary %d : %lf\n",a,present_boundary[a]);
					//printf("Avg_angle %d : %lf\n",a,avg_angle[a]);
					//printf("Future_boundary %d : %lf\n",a,future_boundary[a]);
					
					printf("TCHANGE %d : %lf\n",a,tchange[a]);
					tchange[a]=tchange[a];
					if((tchange[a] < (1000) && tchange[a] > (1)) || (tchange[a] > (-1000) && tchange[a] < (-1)))	// threshold changes to apply....
					{
						if((tchange[a]*(future_boundary[a]-present_boundary[a]))>0)
						{
							tchange[a]=0;
							printf("IMU CORRECTION IGNORED.......................................................................\n");						
						}
					}
					else
					{
						tchange[a]=0;
					}
				}
			}
			local_boundaries[nof/aftr_frames][a]=present_boundary[a];
		}
		
		change[0]=((float)((nof%aftr_frames)+1)/(float)aftr_frames)*tchange[0];
		change[1]=((float)((nof%aftr_frames)+1)/(float)aftr_frames)*tchange[1];
		//change[2]=((float)((nof%aftr_frames)+1)/(float)aftr_frames)*tchange[2];
		if(change[0]!=0)
		printf("IMU WORKING CHANGE 0 : %lf\n",change[0]);
		if(change[1]!=0)
		printf("IMU WORKING CHANGE 1 : %lf\n",change[1]);
			
		
		change[0]=-change[0]*3/4;
		change[1]=pow(-1,leg)*change[1]*2/4;
		
		for(i=0;i<NOM;i++)
		{
			if(tx_packet[7+i*3]==mot)
			{
				value=tx_packet[8+i*3]+tx_packet[9+i*3]*256;
				tx_packet[adchksum]+=tx_packet[8+i*3]+tx_packet[9+i*3];
				value=val[0]+offset[tx_packet[7+i*3]]-change[1];
				tx_packet[8+i*3]=value%256;
				tx_packet[9+i*3]=value/256;
				tx_packet[adchksum]-=tx_packet[8+i*3]+tx_packet[9+i*3];
				printf("value %d \t %d\n",tx_packet[7+i*3],value);				
			}
			if(tx_packet[7+i*3]==mot+1)
			{
				value=tx_packet[8+i*3]+tx_packet[9+i*3]*256;
				tx_packet[adchksum]+=tx_packet[8+i*3]+tx_packet[9+i*3];
				value=val[1]+offset[tx_packet[7+i*3]]+(change[0]*3/7);
				tx_packet[8+i*3]=value%256;
				tx_packet[9+i*3]=value/256;
				tx_packet[adchksum]-=tx_packet[8+i*3]+tx_packet[9+i*3];
				printf("value %d \t %d\n",tx_packet[7+i*3],value);				
			}
			if(tx_packet[7+i*3]==mot+2||tx_packet[7+i*3]==mot+12)
			{
				value=tx_packet[8+i*3]+tx_packet[9+i*3]*256;
				tx_packet[adchksum]+=tx_packet[8+i*3]+tx_packet[9+i*3];
				value=val[2]+offset[tx_packet[7+i*3]];
				tx_packet[8+i*3]=value%256;
				tx_packet[9+i*3]=value/256;
				tx_packet[adchksum]-=(tx_packet[8+i*3]+tx_packet[9+i*3]);
				printf("value %d \t %d\n",tx_packet[7+i*3],value);
			}
			if(tx_packet[7+i*3]==mot+3)
			{
				value=tx_packet[8+i*3]+tx_packet[9+i*3]*256;
				tx_packet[adchksum]+=tx_packet[8+i*3]+tx_packet[9+i*3];
				value=val[3]+offset[tx_packet[7+i*3]]-(change[0]*4/7);
				tx_packet[8+i*3]=value%256;
				tx_packet[9+i*3]=value/256;
				tx_packet[adchksum]-=tx_packet[8+i*3]+tx_packet[9+i*3];
				printf("value %d \t %d\n",tx_packet[7+i*3],value);				
			}
			if(tx_packet[7+i*3]==mot+5)
			{
				value=tx_packet[8+i*3]+tx_packet[9+i*3]*256;
				tx_packet[adchksum]+=tx_packet[8+i*3]+tx_packet[9+i*3];
				value=val[4]+offset[tx_packet[7+i*3]]-change[1];
				tx_packet[8+i*3]=value%256;
				tx_packet[9+i*3]=value/256;
				tx_packet[adchksum]-=tx_packet[8+i*3]+tx_packet[9+i*3];
				printf("value %d \t %d\n",tx_packet[7+i*3],value);				
			}
			if(tx_packet[7+i*3]==revmot)
			{
				value=tx_packet[8+i*3]+tx_packet[9+i*3]*256;
				tx_packet[adchksum]+=tx_packet[8+i*3]+tx_packet[9+i*3];
				value=valr[0]+offset[tx_packet[7+i*3]]+change[1];
				tx_packet[8+i*3]=value%256;
				tx_packet[9+i*3]=value/256;
				tx_packet[adchksum]-=tx_packet[8+i*3]+tx_packet[9+i*3];
				printf("value %d \t %d\n",tx_packet[7+i*3],value);				
			}
			if(tx_packet[7+i*3]==revmot+1)
			{
				value=tx_packet[8+i*3]+tx_packet[9+i*3]*256;
				tx_packet[adchksum]+=tx_packet[8+i*3]+tx_packet[9+i*3];
				value=valr[1]+offset[tx_packet[7+i*3]]+(change[0]*3/7);
				tx_packet[8+i*3]=value%256;
				tx_packet[9+i*3]=value/256;
				tx_packet[adchksum]-=tx_packet[8+i*3]+tx_packet[9+i*3];
				printf("value %d \t %d\n",tx_packet[7+i*3],value);				
			}
			if(tx_packet[7+i*3]==revmot+2||tx_packet[7+i*3]==revmot+12)
			{
				value=tx_packet[8+i*3]+tx_packet[9+i*3]*256;
				tx_packet[adchksum]+=tx_packet[8+i*3]+tx_packet[9+i*3];
				value=valr[2]+offset[tx_packet[7+i*3]];
				tx_packet[8+i*3]=value%256;
				tx_packet[9+i*3]=value/256;
				tx_packet[adchksum]-=(tx_packet[8+i*3]+tx_packet[9+i*3]);
				printf("value %d \t %d\n",tx_packet[7+i*3],value);
			}
			if(tx_packet[7+i*3]==revmot+3)
			{
				value=tx_packet[8+i*3]+tx_packet[9+i*3]*256;
				tx_packet[adchksum]+=tx_packet[8+i*3]+tx_packet[9+i*3];
				value=valr[3]+offset[tx_packet[7+i*3]]-(change[0]*4/7);
				tx_packet[8+i*3]=value%256;
				tx_packet[9+i*3]=value/256;
				tx_packet[adchksum]-=tx_packet[8+i*3]+tx_packet[9+i*3];
				printf("value %d \t %d\n",tx_packet[7+i*3],value);				
			}
			if(tx_packet[7+i*3]==revmot+5)
			{
				value=tx_packet[8+i*3]+tx_packet[9+i*3]*256;
				tx_packet[adchksum]+=tx_packet[8+i*3]+tx_packet[9+i*3];
				value=valr[4]+offset[tx_packet[7+i*3]]+change[1];
				tx_packet[8+i*3]=value%256;
				tx_packet[9+i*3]=value/256;
				tx_packet[adchksum]-=tx_packet[8+i*3]+tx_packet[9+i*3];
				printf("value %d \t %d\n",tx_packet[7+i*3],value);				
			}
			
		}
		if(ftdi_write_data(&ftdic1,tx_packet,tx_packet[3]+4)>0)
		printf("PACKET SENT \n");
		else
		printf("ERROR SENDING PACKET\n");
		
		usleep(16670*1.5); // do not ask why....
		for(i=0;i<128;i++)
		{	
			last_tx_packet[i]=tx_packet[i];
			//printf("%d\t",tx_packet[i]);
		}		
		
	}
	return EXIT_SUCCESS;
}

int walk2(int leg,float yin,float yfi,float yrin,float yrfi, float zin, float zfi,float zrin, float zrfi, float time)
{
	// IMU DEFINITIONS
	byte im[8];
	float change[3]={0};
	float tchange[3]={0};
	int count=0;
	int aftr_frames=2;
	int noat=0,a=0,b=0,c=0;;
	int nof=0;
	float present_boundary[3]={0};
	float future_boundary[3]={0};

	// GEN DEFINITIONS
	int i=0,h=0;
	
	//COOD DEFINITIONS
	float x,y,z,yr,zr;
	float t=0;
	float t_frac;
	float l=0;
	float freq=2*3.14;    //controls how fast x varies with time
	
	
	// VARIABLES TO NOT MESS WITH 
			//X CONTROL
	float dis=0.5;        //controls how much sinx is displaced above x axis
	float phase=-1.57;	  //controls initial phase of sinx graph  
	float xmax=350;    //maximum x in walk
			//Z SHIFT CONTROL
	float xzval=390;   // x at which zmax is achieved   
	//float zmax=40+5*abs(zfi)/6;        //maximum z in weight shift
	float zmax=35;
	float ztime=(asin(((390-xzval)/(390-xmax)*(1+dis))-(dis))-phase)/freq*time; // time of zmax ,xzval 
	float zfreq=zmax/sin(3.14*ztime/time);       //how quickly z increases to max value-- redundant
			//Y CONTROL
	float xyvali=380;
	float xyvalf=380;
	float yi_tfrac=(asin(((390-xyvali)/(390-xmax)*(1+dis))-(dis))-phase)/freq*time;
	float yf_tfrac=(freq-asin(((390-xyvalf)/(390-xmax)*(1+dis))-(dis))+phase)/freq*time;;
	
			//Z CONTROL
	float xzvali=380;
	float xzvalf=380;
	float zi_tfrac=(asin(((390-xzvali)/(390-xmax)*(1+dis))-(dis))-phase)/freq*time;
	float zf_tfrac=(freq-asin(((390-xzvalf)/(390-xmax)*(1+dis))-(dis))+phase)/freq*time;;
	
	//PACKET DEFINITIONS
	int offset[33];
	offset[0]=-4; offset[1]=125; offset[2]=304; offset[3]=341; offset[4]=0; offset[5]=0; offset[6]=300; offset[7]=0; offset[8]=0; offset[9]=0; offset[10]=0; offset[11]=0; offset[12]=254; offset[13]=0; offset[14]=0; offset[15]=-119; offset[16]=0; offset[17]=0; offset[18]=0; offset[19]=0; offset[20]=-19; offset[21]=326; offset[22]=304; offset[23]=341; offset[24]=0; offset[25]=0; offset[26]=-279; offset[27]= 0; offset[28]= 0; offset[29]=0; offset[30]=0; offset[31]=0; offset[32]=254;
	int adchksum=7+NOM*3;
	float* fval;
	float val[5],valr[5]; 
	
	//MOTOR DEFINITIONS
	int mot,revmot;
	int value;
	
	//COMILANCE DEFINITIONS
	byte cp=32,cpr=32;
	byte compliance_packet[]={0xff,0xff,0xfe,0x28,0x83,0x1c,0x02,mot,cp,cp,mot+1,cp,cp,mot+2,cp,cp,mot+12,cp,cp,mot+3,cp,cp,mot+5,cp,cp,revmot,cpr,cpr,revmot+1,cpr,cpr,revmot+2,cpr,cpr,revmot+12,cpr,cpr,revmot+3,cpr,cpr,revmot+5,cpr,cpr,0xff};
	for(h=2;h<43;h++)
	compliance_packet[43]-=compliance_packet[h];
	
	printf("LEG : %d\n",leg);
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
	
	cp=10;
	cpr=10;
	for(h=2;h<43;h++)
	compliance_packet[43]-=compliance_packet[h];
	
	if(ftdi_write_data(&ftdic1,compliance_packet,compliance_packet[3]+4)>0)
	printf("Compliance Written\n");
	else
	printf("ERROR : Compliance\n");
	
	for(;t<time;t+=(float)1/60)
	{
		t_frac=t/time;
		//z=(1-t_frac)*zin+t_frac*zfi;
		//zr=(1-t_frac)*(zin)+t_frac*(zfi);
		//y=(1-t_frac)*yin+t_frac*yfi;
		//yr=(1-t_frac)*(-yrin)+t_frac*(-yfi);
		//printf("T_FRAC %lf YI_tfrac %lf \n",t_frac,mi_tfrac);
		if((int)(t_frac*100)<(int)(yi_tfrac*100))
		{
			y=yin;
			yr=yrin;
		}
		else if((int)(t_frac*100)<(int)(yf_tfrac*100))
		{
			y=yin+(yfi-yin)*((t_frac-yi_tfrac)/(yf_tfrac-yi_tfrac));
			yr=yrin+(yrfi-yrin)*((t_frac-yi_tfrac)/(yf_tfrac-yi_tfrac));
		}
		else
		{
			y=yfi;
			yr=yrfi;
		}
		
		if((int)(t_frac*100)<(int)(zi_tfrac*100))
		{
			z=zin;
			zr=zrin;
		}
		else if((int)(t_frac*100)<(int)(zf_tfrac*100))
		{
			z=zin+(zfi+zrfi-zin)*((t_frac-zi_tfrac)/(zf_tfrac-zi_tfrac));
			zr=zrin;
		}
		else
		{
			z=zrfi+zfi+(zfi-zrfi-zfi)*((t_frac-zf_tfrac)/(1-zf_tfrac));;
			zr=zrin+(zrfi-zrin)*((t_frac-zf_tfrac)/(1-zf_tfrac));
		}
		
		x=390-(390-xmax)*(max(0,sin((freq*t_frac)+phase)+dis)/(1+dis));
		z+=min(zmax,zfreq*sin(3.14*t_frac));
		zr-=min(zmax,zfreq*sin(3.14*t_frac));
		
		for(i=0;i<128;i++)
		{
			tx_packet[i]=last_tx_packet[i];
			//printf("%d\t",tx_packet[i]);
		}
		
		printf("Time : %lf\n",t);
	
		/*if((int)((t-0.75)*100)==0)
		{
			cp=32;
			cpr=32;
			for(h=2;h<43;h++)
			compliance_packet[43]-=compliance_packet[h];
			
			if(ftdi_write_data(&ftdic1,compliance_packet,compliance_packet[3]+4)>0)
			printf("Compliance Written\n");
			else
			printf("ERROR : Compliance\n");
		}
		*/
				
		//if(t>=gxt[k+1])
		//k++;
		//if(t>=gzt[j+1])
		//j++;
		//if(t>=gyt[l+1])
		//l++;
		
		//printf("k : %d j : %d\n",k,j);
		//printf("%lf\t%lf\t%lf\t%lf\n",outputx[k*4],outputx[k*4+1],outputx[k*4+2],outputx[k*4+3]);
		//printf("%lf\t%lf\t%lf\t%lf\n",outputy[j*4],outputy[j*4+1],outputy[j*4+2],outputy[j*4+3]);
		
		/*
		x=outputx[k*4]+outputx[k*4+1]*pow(t-gxt[k],1)+outputx[k*4+2]*pow(t-gxt[k],2)+outputx[k*4+3]*pow(t-gxt[k],3);
		z=outputz[j*4]+outputz[j*4+1]*pow(t-gzt[j],1)+outputz[j*4+2]*pow(t-gzt[j],2)+outputz[j*4+3]*pow(t-gzt[j],3);
		zr=outputzr[j*4]+outputzr[j*4+1]*pow(t-gztr[j],1)+outputzr[j*4+2]*pow(t-gztr[j],2)+outputzr[j*4+3]*pow(t-gztr[j],3);
		y=outputy[j*4]+outputy[j*4+1]*pow(t-gyt[j],1)+outputy[j*4+2]*pow(t-gyt[j],2)+outputy[j*4+3]*pow(t-gyt[j],3);
		yr=outputyr[j*4]+outputyr[j*4+1]*pow(t-gytr[j],1)+outputyr[j*4+2]*pow(t-gytr[j],2)+outputyr[j*4+3]*pow(t-gytr[j],3);
		*/
		
		change[0]=0;
		change[1]=0;
		imu_init=9;
		if(nof%aftr_frames==0&&imu_init==1)
		{
			imucal();
			if(nof>aftr_frames)
			{
				for(a=0;a<2;a++)
				{
					present_boundary[a]=local_boundaries[(nof/aftr_frames)-1][a]+(boundaries[(nof/aftr_frames)][a]-boundaries[(nof/aftr_frames)-1][a]);
					tchange[a]=(avg_angle[a]-present_boundary[a]);		
					future_boundary[a]=local_boundaries[(nof/aftr_frames)-1][a]+(boundaries[(nof/aftr_frames)+1][a]-boundaries[(nof/aftr_frames)-1][a]);
					
					//printf("Diff Boundary %d : %lf\n",a,boundaries[(nof/aftr_frames)][a]-boundaries[(nof/aftr_frames)-1][a]);
					//printf("P Boundary %d : %lf\n",a,present_boundary[a]);
					//printf("Avg_angle %d : %lf\n",a,avg_angle[a]);
					//printf("Future_boundary %d : %lf\n",a,future_boundary[a]);
					
					printf("TCHANGE %d : %lf\n",a,tchange[a]);
					tchange[a]=tchange[a];
					if((tchange[a] < (1000) && tchange[a] > (1)) || (tchange[a] > (-1000) && tchange[a] < (-1)))	// threshold changes to apply....
					{
						if((tchange[a]*(future_boundary[a]-present_boundary[a]))>0)
						{
							tchange[a]=0;
							printf("IMU CORRECTION IGNORED.......................................................................\n");						
						}
					}
					else
					{
						tchange[a]=0;
					}
				}
			}
			local_boundaries[nof/aftr_frames][a]=present_boundary[a];
		}
		
		printf("X : %lf\tY : %lf\tYR : %lf\tZ : %lf\tZr : %lf\n",x,y,yr,z,zr);		
		
		//change[0]=((float)((nof%aftr_frames)+1)/(float)aftr_frames)*tchange[0];
		//change[1]=((float)((nof%aftr_frames)+1)/(float)aftr_frames)*tchange[1];
				
		//y=y-y*change[0]/180*3.14;
		//z=z-z*change[1]/180*3.14;
				
		//printf("X : %lf\tY : %lf\tZ : %lf\n",x,y,z);		
		fval=generateik(x,y,z,0);
		for(h=0;h<5;h++)
		val[h]=fval[h];
		fval=generateik(390,yr,zr,0);
		for(h=0;h<5;h++)
		valr[h]=fval[h];
		
		
		/*change[2]=((float)((nof%aftr_frames)+1)/(float)aftr_frames)*tchange[2];
		if(change[0]!=0)
		printf("IMU WORKING CHANGE 0 : %lf\n",change[0]);
		if(change[1]!=0)
		printf("IMU WORKING CHANGE 1 : %lf\n",change[1]);
			
		
		//change[0]=-change[0]*3/4;
		//change[1]=pow(-1,leg)*change[1]*2/4;
		change[0]=0;
		change[1]=0;
		*/
		for(i=0;i<NOM;i++)
		{
			if(tx_packet[7+i*3]==mot)
			{
				value=tx_packet[8+i*3]+tx_packet[9+i*3]*256;
				tx_packet[adchksum]+=tx_packet[8+i*3]+tx_packet[9+i*3];
				value=val[0]+offset[tx_packet[7+i*3]]-change[1];
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
				value=val[1]+offset[tx_packet[7+i*3]]+(change[0]*3/7);
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
				value=val[3]+offset[tx_packet[7+i*3]]-(change[0]*4/7);
				if(value>=0 && value<=4096)
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
				value=val[4]+offset[tx_packet[7+i*3]]-change[1];
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
				value=valr[0]+offset[tx_packet[7+i*3]]+change[1];
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
				value=valr[1]+offset[tx_packet[7+i*3]]+(change[0]*3/7);
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
				value=valr[3]+offset[tx_packet[7+i*3]]-(change[0]*4/7);
				if(value>=0 && value<=4096)
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
				value=valr[4]+offset[tx_packet[7+i*3]]+change[1];
				if(value>=0 && value<=4096)
				{
					tx_packet[8+i*3]=value%256;
					tx_packet[9+i*3]=value/256;
				}
				tx_packet[adchksum]-=tx_packet[8+i*3]+tx_packet[9+i*3];
				//printf("value %d \t %d\n",tx_packet[7+i*3],value);				
			}
			
		}
		if(ftdi_write_data(&ftdic1,tx_packet,tx_packet[3]+4)>0)
		printf("PACKET SENT \n");
		else
		printf("ERROR SENDING PACKET\n");
		
		usleep(16670);
		for(i=0;i<128;i++)
		{	
			last_tx_packet[i]=tx_packet[i];
			//printf("%d\t",tx_packet[i]);
		}		
		
	}
	return EXIT_SUCCESS;
}		
		


int walk(int leg,float yin,float yfi)
{
	
	byte im[8];
	float change[3]={0};
	float tchange[3]={0};
	int count=0;
	int aftr_frames=2;
	int noat=0,a=0,b=0,c=0;;
	int nof=0;
	float present_boundary[3]={0};
	float future_boundary[3]={0};



	int offset[33];
	offset[0]=-4; offset[1]=125; offset[2]=304; offset[3]=341; offset[4]=0; offset[5]=0; offset[6]=300; offset[7]=0; offset[8]=0; offset[9]=0; offset[10]=0; offset[11]=0; offset[12]=254; offset[13]=0; offset[14]=0; offset[15]=-119; offset[16]=0; offset[17]=0; offset[18]=0; offset[19]=0; offset[20]=-19; offset[21]=326; offset[22]=304; offset[23]=341; offset[24]=0; offset[25]=0; offset[26]=-279; offset[27]= 0; offset[28]= 0; offset[29]=0; offset[30]=0; offset[31]=0; offset[32]=254;
	int adchksum=7+NOM*3;
	
	int k=0,j=0,i=0,h=0,l=0;
	int flag=0;
	float x,z,zr,y,yr;
	
	int mot,revmot;
	float t=0;
	int value;
	
	float* fval;
	float val[5],valr[5]; 
	int gnx=5;
	float gx[]={390,390,360,390,390};
	float gxt[]={0,0.20,0.60,0.80,1.0};
	float* ox;
	float outputx[20];
	ox=generatespline(gnx-1,gxt,gx);
	for(h=0;h<4;h++)
	{
		if(gx[h]==gx[h+1])
		{
			outputx[h*4]=gx[h];
			outputx[h*4+1]=0;
			outputx[h*4+2]=0;
			outputx[h*4+3]=0;
		}
		else
		{
			outputx[h*4]=ox[h*4];
			outputx[h*4+1]=ox[h*4+1];
			outputx[h*4+2]=ox[h*4+2];
			outputx[h*4+3]=ox[h*4+3];
		}
	}
	
	int gny=4;
	float gy[]={yin,yin,yfi,yfi};
	float gyt[]={0,0.1,0.8,1.0};
	float* oy;
	float outputy[20];
	oy=generatespline(gny-1,gyt,gy);
	for(h=0;h<4;h++)
	{
		if(gy[h]==gy[h+1])
		{
			outputy[h*4]=gy[h];
			outputy[h*4+1]=0;
			outputy[h*4+2]=0;
			outputy[h*4+3]=0;
		}
		else
		{
			outputy[h*4]=oy[h*4];
			outputy[h*4+1]=oy[h*4+1];
			outputy[h*4+2]=oy[h*4+2];
			outputy[h*4+3]=oy[h*4+3];
		}
	}
	
	int gnyr=4;
	float gyr[]={-yin,-yin,-yfi,-yfi};
	float gytr[]={0,0.1,0.90,1.0};
	float* oyr;
	float outputyr[20];
	oyr=generatespline(gnyr-1,gytr,gyr);
	for(h=0;h<4;h++)
	{
		if(gyr[h]==gyr[h+1])
		{
			outputyr[h*4]=gyr[h];
			outputyr[h*4+1]=0;
			outputyr[h*4+2]=0;
			outputyr[h*4+3]=0;
		}
		else
		{
			outputyr[h*4]=oyr[h*4];
			outputyr[h*4+1]=oyr[h*4+1];
			outputyr[h*4+2]=oyr[h*4+2];
			outputyr[h*4+3]=oyr[h*4+3];
		}
	}
	
	
	int gnzr=4;
	float gzr[]={0,-(35-(float)abs(yin)/4),-(35-(float)abs(yin)/4),0};
	float gztr[]={0,0.2,0.9,1.0};
	float* ozr;
	float outputzr[20];
	ozr=generatespline(gnzr-1,gztr,gzr);
	for(h=0;h<4;h++)
	{
		if(gzr[h]==gzr[h+1])
		{
			outputzr[h*4]=gzr[h];
			outputzr[h*4+1]=0;
			outputzr[h*4+2]=0;
			outputzr[h*4+3]=0;
		}
		else
		{
			outputzr[h*4]=ozr[h*4];
			outputzr[h*4+1]=ozr[h*4+1];
			outputzr[h*4+2]=ozr[h*4+2];
			outputzr[h*4+3]=ozr[h*4+3];
		}
	}
	
	int gnz=4;
	float gz[]={0,(35-(float)abs(yin)/4),(35-(float)abs(yin)/4),0};
	float gzt[]={0,0.20,0.90,1.0};
	float* oz;
	float outputz[20];
	oz=generatespline(gnz-1,gzt,gz);
	for(h=0;h<4;h++)
	{
		if(gz[h]==gz[h+1])
		{
			outputz[h*4]=gz[h];
			outputz[h*4+1]=0;
			outputz[h*4+2]=0;
			outputz[h*4+3]=0;
		}
		else
		{
			outputz[h*4]=oz[h*4];
			outputz[h*4+1]=oz[h*4+1];
			outputz[h*4+2]=oz[h*4+2];
			outputz[h*4+3]=oz[h*4+3];
		}
	}
	
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
	
	byte cp=32,cpr=32;
	byte compliance_packet[]={0xff,0xff,0xfe,0x28,0x83,0x1c,0x02,mot,cp,cp,mot+1,cp,cp,mot+2,cp,cp,mot+12,cp,cp,mot+3,cp,cp,mot+5,cp,cp,revmot,cpr,cpr,revmot+1,cpr,cpr,revmot+2,cpr,cpr,revmot+12,cpr,cpr,revmot+3,cpr,cpr,revmot+5,cpr,cpr,0xff};
	for(h=2;h<43;h++)
	compliance_packet[43]-=compliance_packet[h];
	
	if(ftdi_write_data(&ftdic1,compliance_packet,compliance_packet[3]+4)>0)
	printf("Compliance Written\n");
	else
	printf("ERROR : Compliance\n");
	
	for(;t<gxt[gnx-1];t+=((float)1/60))
	{
		nof=(int)(t*60);
		
		for(i=0;i<128;i++)
		{
			tx_packet[i]=last_tx_packet[i];
			//printf("%d\t",tx_packet[i]);
		}
		
		printf("Time : %lf\n",t);
	
		if((int)((t-0.75)*100)==0)
		{
			cp=32;
			cpr=32;
			for(h=2;h<43;h++)
			compliance_packet[43]-=compliance_packet[h];
			
			if(ftdi_write_data(&ftdic1,compliance_packet,compliance_packet[3]+4)>0)
			printf("Compliance Written\n");
			else
			printf("ERROR : Compliance\n");
		}
		
				
		if(t>=gxt[k+1])
		k++;
		if(t>=gzt[j+1])
		j++;
		if(t>=gyt[l+1])
		l++;
		
		//printf("k : %d j : %d\n",k,j);
		//printf("%lf\t%lf\t%lf\t%lf\n",outputx[k*4],outputx[k*4+1],outputx[k*4+2],outputx[k*4+3]);
		//printf("%lf\t%lf\t%lf\t%lf\n",outputy[j*4],outputy[j*4+1],outputy[j*4+2],outputy[j*4+3]);
		
		x=outputx[k*4]+outputx[k*4+1]*pow(t-gxt[k],1)+outputx[k*4+2]*pow(t-gxt[k],2)+outputx[k*4+3]*pow(t-gxt[k],3);
		z=outputz[j*4]+outputz[j*4+1]*pow(t-gzt[j],1)+outputz[j*4+2]*pow(t-gzt[j],2)+outputz[j*4+3]*pow(t-gzt[j],3);
		zr=outputzr[j*4]+outputzr[j*4+1]*pow(t-gztr[j],1)+outputzr[j*4+2]*pow(t-gztr[j],2)+outputzr[j*4+3]*pow(t-gztr[j],3);
		y=outputy[j*4]+outputy[j*4+1]*pow(t-gyt[j],1)+outputy[j*4+2]*pow(t-gyt[j],2)+outputy[j*4+3]*pow(t-gyt[j],3);
		yr=outputyr[j*4]+outputyr[j*4+1]*pow(t-gytr[j],1)+outputyr[j*4+2]*pow(t-gytr[j],2)+outputyr[j*4+3]*pow(t-gytr[j],3);
		
		change[0]=0;
		change[1]=0;
		//imu_init=9;
		if(nof%aftr_frames==0&&imu_init==1)
		{
			imucal();
			if(nof>aftr_frames)
			{
				for(a=0;a<2;a++)
				{
					present_boundary[a]=local_boundaries[(nof/aftr_frames)-1][a]+(boundaries[(nof/aftr_frames)][a]-boundaries[(nof/aftr_frames)-1][a]);
					tchange[a]=(avg_angle[a]-present_boundary[a]);		
					future_boundary[a]=local_boundaries[(nof/aftr_frames)-1][a]+(boundaries[(nof/aftr_frames)+1][a]-boundaries[(nof/aftr_frames)-1][a]);
					
					//printf("Diff Boundary %d : %lf\n",a,boundaries[(nof/aftr_frames)][a]-boundaries[(nof/aftr_frames)-1][a]);
					//printf("P Boundary %d : %lf\n",a,present_boundary[a]);
					//printf("Avg_angle %d : %lf\n",a,avg_angle[a]);
					//printf("Future_boundary %d : %lf\n",a,future_boundary[a]);
					
					printf("TCHANGE %d : %lf\n",a,tchange[a]);
					tchange[a]=tchange[a];
					if((tchange[a] < (1000) && tchange[a] > (1)) || (tchange[a] > (-1000) && tchange[a] < (-1)))	// threshold changes to apply....
					{
						if((tchange[a]*(future_boundary[a]-present_boundary[a]))>0)
						{
							tchange[a]=0;
							printf("IMU CORRECTION IGNORED.......................................................................\n");						
						}
					}
					else
					{
						tchange[a]=0;
					}
				}
			}
			local_boundaries[nof/aftr_frames][a]=present_boundary[a];
		}
		
		printf("X : %lf\tY : %lf\tZ : %lf\n",x,y,z);		
		
		change[0]=((float)((nof%aftr_frames)+1)/(float)aftr_frames)*tchange[0];
		change[1]=((float)((nof%aftr_frames)+1)/(float)aftr_frames)*tchange[1];
				
		y=y-y*change[0]/180*3.14;
		z=z-z*change[1]/180*3.14;
				
		printf("X : %lf\tY : %lf\tZ : %lf\n",x,y,z);		
		fval=generateik(x,y,z,0);
		for(h=0;h<5;h++)
		val[h]=fval[h];
		fval=generateik(390,yr,zr,0);
		for(h=0;h<5;h++)
		valr[h]=fval[h];
		
		
		//change[2]=((float)((nof%aftr_frames)+1)/(float)aftr_frames)*tchange[2];
		if(change[0]!=0)
		printf("IMU WORKING CHANGE 0 : %lf\n",change[0]);
		if(change[1]!=0)
		printf("IMU WORKING CHANGE 1 : %lf\n",change[1]);
			
		
		//change[0]=-change[0]*3/4;
		//change[1]=pow(-1,leg)*change[1]*2/4;
		change[0]=0;
		change[1]=0;
		
		for(i=0;i<NOM;i++)
		{
			if(tx_packet[7+i*3]==mot)
			{
				value=tx_packet[8+i*3]+tx_packet[9+i*3]*256;
				tx_packet[adchksum]+=tx_packet[8+i*3]+tx_packet[9+i*3];
				value=val[0]+offset[tx_packet[7+i*3]]-change[1];
				if(value>=0 && value<=4096)
				{
					tx_packet[8+i*3]=value%256;
					tx_packet[9+i*3]=value/256;
				}
				tx_packet[adchksum]-=tx_packet[8+i*3]+tx_packet[9+i*3];
				printf("value %d \t %d\n",tx_packet[7+i*3],value);				
			}
			if(tx_packet[7+i*3]==mot+1)
			{
				value=tx_packet[8+i*3]+tx_packet[9+i*3]*256;
				tx_packet[adchksum]+=tx_packet[8+i*3]+tx_packet[9+i*3];
				value=val[1]+offset[tx_packet[7+i*3]]+(change[0]*3/7);
				if(value>=0 && value<=4096)
				{
					tx_packet[8+i*3]=value%256;
					tx_packet[9+i*3]=value/256;
				}
				tx_packet[adchksum]-=tx_packet[8+i*3]+tx_packet[9+i*3];
				printf("value %d \t %d\n",tx_packet[7+i*3],value);				
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
				printf("value %d \t %d\n",tx_packet[7+i*3],value);
			}
			if(tx_packet[7+i*3]==mot+3)
			{
				value=tx_packet[8+i*3]+tx_packet[9+i*3]*256;
				tx_packet[adchksum]+=tx_packet[8+i*3]+tx_packet[9+i*3];
				value=val[3]+offset[tx_packet[7+i*3]]-(change[0]*4/7);
				if(value>=0 && value<=4096)
				{
					tx_packet[8+i*3]=value%256;
					tx_packet[9+i*3]=value/256;
				}
				tx_packet[adchksum]-=tx_packet[8+i*3]+tx_packet[9+i*3];
				printf("value %d \t %d\n",tx_packet[7+i*3],value);				
			}
			if(tx_packet[7+i*3]==mot+5)
			{
				value=tx_packet[8+i*3]+tx_packet[9+i*3]*256;
				tx_packet[adchksum]+=tx_packet[8+i*3]+tx_packet[9+i*3];
				value=val[4]+offset[tx_packet[7+i*3]]-change[1];
				if(value>=0 && value<=4096)
				{
					tx_packet[8+i*3]=value%256;
					tx_packet[9+i*3]=value/256;
				}
				tx_packet[adchksum]-=tx_packet[8+i*3]+tx_packet[9+i*3];
				printf("value %d \t %d\n",tx_packet[7+i*3],value);				
			}
			if(tx_packet[7+i*3]==revmot)
			{
				value=tx_packet[8+i*3]+tx_packet[9+i*3]*256;
				tx_packet[adchksum]+=tx_packet[8+i*3]+tx_packet[9+i*3];
				value=valr[0]+offset[tx_packet[7+i*3]]+change[1];
				if(value>=0 && value<=4096)
				{
					tx_packet[8+i*3]=value%256;
					tx_packet[9+i*3]=value/256;
				}
				tx_packet[adchksum]-=tx_packet[8+i*3]+tx_packet[9+i*3];
				printf("value %d \t %d\n",tx_packet[7+i*3],value);				
			}
			if(tx_packet[7+i*3]==revmot+1)
			{
				value=tx_packet[8+i*3]+tx_packet[9+i*3]*256;
				tx_packet[adchksum]+=tx_packet[8+i*3]+tx_packet[9+i*3];
				value=valr[1]+offset[tx_packet[7+i*3]]+(change[0]*3/7);
				if(value>=0 && value<=4096)
				{
					tx_packet[8+i*3]=value%256;
					tx_packet[9+i*3]=value/256;
				}
				tx_packet[adchksum]-=tx_packet[8+i*3]+tx_packet[9+i*3];
				printf("value %d \t %d\n",tx_packet[7+i*3],value);				
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
				printf("value %d \t %d\n",tx_packet[7+i*3],value);
			}
			if(tx_packet[7+i*3]==revmot+3)
			{
				value=tx_packet[8+i*3]+tx_packet[9+i*3]*256;
				tx_packet[adchksum]+=tx_packet[8+i*3]+tx_packet[9+i*3];
				value=valr[3]+offset[tx_packet[7+i*3]]-(change[0]*4/7);
				if(value>=0 && value<=4096)
				{
					tx_packet[8+i*3]=value%256;
					tx_packet[9+i*3]=value/256;
				}
				tx_packet[adchksum]-=tx_packet[8+i*3]+tx_packet[9+i*3];
				printf("value %d \t %d\n",tx_packet[7+i*3],value);				
			}
			if(tx_packet[7+i*3]==revmot+5)
			{
				value=tx_packet[8+i*3]+tx_packet[9+i*3]*256;
				tx_packet[adchksum]+=tx_packet[8+i*3]+tx_packet[9+i*3];
				value=valr[4]+offset[tx_packet[7+i*3]]+change[1];
				if(value>=0 && value<=4096)
				{
					tx_packet[8+i*3]=value%256;
					tx_packet[9+i*3]=value/256;
				}
				tx_packet[adchksum]-=tx_packet[8+i*3]+tx_packet[9+i*3];
				printf("value %d \t %d\n",tx_packet[7+i*3],value);				
			}
			
		}
		if(ftdi_write_data(&ftdic1,tx_packet,tx_packet[3]+4)>0)
		printf("PACKET SENT \n");
		else
		printf("ERROR SENDING PACKET\n");
		
		usleep(16670);
		for(i=0;i<128;i++)
		{	
			last_tx_packet[i]=tx_packet[i];
			//printf("%d\t",tx_packet[i]);
		}		
		
	}
	return EXIT_SUCCESS;
}
		
void make_dribble()
{
	imucal_init();
	int flag=0;
	while(!kbhit2())
	{
		dribble(flag);
		if(flag==0)
		{
			flag=1;
			usleep(16670*5);
		}
		else if(flag==1)
		{
			flag=0;
			usleep(16670*15);
		}		
	}
}

void make_walk()
{
	imucal_init();
	//walk(1,0,40);
	int flag=0;
	while(!kbhit2())
	{
		walk(flag,0,0);
		usleep(16670*5);
		if(flag==0)
		flag=1;
		else if(flag==1)
		flag=0;
	}
}

void make_walk2()
{
	int y=20;
	int count=0;
	//imucal_init();
	//walk2(1,0,30,0,-40,0,0,0,0,0.75);
	int flag=0;
	while(!kbhit2())
	{
		/*if(y<30&&count==4)
		{
			count=0;		
			y+=10;
		}*/
		//count++;
		//walk2(0,-40,30,30,-40,0,0,0,0,1);
		//usleep(16670*10);
		walk2(flag,0,0,0,0,0,0,0,0,1);
		usleep(16670*10);
		//walk2(1,0,0,0,0,0,30,1);
		if(flag==0)
		flag=1;
		else if(flag==1)
		flag=0;
		
	}
	//walk2(flag,-40,0,30,0,0,0,.75);
		
}


int bluecall()
{
/*	struct termios options;
	struct timeval start, end;
	long mtime, seconds, useconds;
	if((fb = open("/dev/ttyUSB1", O_RDWR | O_NOCTTY | O_NDELAY))==-1)
	{	printf("Serial Port unavailable\n");
		close(fb);
		return;
	}
 	tcgetattr(fb, &options);
  	cfsetispeed(&options, B115200);
  	cfsetospeed(&options, B115200);
  	options.c_cflag |= (CLOCAL | CREAD);
  	options.c_cflag &= ~PARENB;
  	options.c_cflag &= ~CSTOPB;
     	options.c_cflag &= ~CSIZE;
     	options.c_cflag |= CS8;
     	options.c_oflag |= OPOST;
     	options.c_oflag |= ONOCR;
     	options.c_oflag |= FF0;
     	options.c_oflag &= ~FF1;
     	options.c_oflag &= ~ONLCR;
        options.c_cc[VSTART] = 0;
	options.c_cc[VSTOP] = 0;

	options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);    
    	options.c_cflag &= ~CRTSCTS;
//	tcflush(fb, TCIFLUSH);
  	tcsetattr(fb, TCSANOW, &options);
  	fcntl(fb, F_SETFL, FNDELAY);	
*/

	char ch;
//------------------------------------------------------------------------------------------imu declarations--------------------------------------------------------------------------------------
	byte im[8],imu_i,j,k=0,a=3;
	byte buffer[59];
	float ang[3]={0};
	
	float change[3]={0};
	int flag_imu_first=-1,pref=0,run_imu=0,imu_on=0,change_enuf[3]={0,0,0};
	int checksum_change=0,checksum_new=0;	
	int base_val[NOM]={0},imu_gpos[NOM]={0};
	int frac=1;
//----------------------------------------------------------------------------------------imu declarations end------------------------------------------------------------------------------------
	
	printf("BLUECALL CALLED\n");
	FILE *check_offset;
	check_offset=fopen("master_offsets.offset","r");
	if(check_offset!=NULL)
	{
		//offset_status=1;
		fclose(check_offset);
		printf("Master Offsets Enabled\n");
	}
	int ret;
    if (ftdi_init(&ftdic2) < 0)
    {
        fprintf(stderr, "ftdi_init failed\n");
        return EXIT_FAILURE;
    }
	
// ftdi_read_data_set_chunksize(&ftdic2, 512);
    
    if ((ret = ftdi_usb_open_desc (&ftdic2, 0x0403, 0x6001, NULL, serialbluetooth)) < 0)
    {
        fprintf(stderr, "unable to open ftdi bluetooth device: %d (%s)\n", ret, ftdi_get_error_string(&ftdic2));
        return EXIT_FAILURE;
    }
    /*else
    printf("GUI Linking DONE\n");*/

    ftdi_set_baudrate(&ftdic2,115200);
	/*int packet_size=ftdic2 -> max_packet_size;
printf("max packet size is %d\n",packet_size);
*/
	//FILE *find;
	FILE *read_cmbine;
	combine_f moves_cmbine;
	byte *bufptr,*packptr,chksum=0;
	byte pack=0,pack1;
	int i=2,len=30,count=0,exit_flag=0,iterator1,remote_loop,remote_loop_i,status_loop,status_loop_i;
	byte startkar=0xff,startkar1=0xff,Xcode=1;
	char move[len+8],move2[len+25],*temp2,cmbine_name[65];
	while(exit_flag!=1)
	{
		if(flag_imu_first==-1)
		{
			ch='n';
			printf("DO YOU WANT TO RUN IMU (STATIC MODE)? (Y/N)\n");
			scanf("%s",&ch);
			if(ch=='y')
			run_imu=1;
			else 
			run_imu=0;
			flag_imu_first=0;
		}
		//run_imu=0;
		//flag_imu_first=0;
		if(gflag>=4&&imu_on==0&&run_imu==1)
		{
			printf("IMU INITIALIZE?(y/n)\n");
			scanf("%s",&ch);
			if(ch=='y')
			imu_on=1;
			
		}
		
		
		if(ftdi_read_data(&ftdic2,&pack,1)>0)
		{
		
			printf("PACK:%x\n",pack);
			if(pack==0xff)
			i--;
			else
			i=2;
			if(i==0)
			{
				i=2;
				Xcode=(byte)packet_recieve();
				if(Xcode==1)
				{
					element=0;
					ftdi_write_data(&ftdic2,&Xcode,1);
					switch(gui_packet[element++])
					{
						case 0: exit_flag=1;
								/*readfiles("walk1");
								readfiles("walk2");
								readfiles("walk3");
								readfiles("walk2");
								readfiles("walk3");
								readfiles("walk4");
								*/break;
						case 1:	//play_acyut();gflag=0;sleep(2);
								pref=gui_packet[element++];
								pref=pref*10+gui_packet[element++];
								switch(pref)
								{
									case 00: if(acyut==3)
											 Xcode=(byte)act_fpacket("acyut3/temp");
											 else if(acyut==4)
											 Xcode=(byte)act_fpacket("acyut4/temp");
											 break;
									case 01: if(acyut==3)
											 Xcode=(byte)act_ppacket("acyut3/temp",gui_packet[element++],gui_packet[element++]);
											 else if(acyut==4)
											 Xcode=(byte)act_ppacket("acyut4/temp",gui_packet[element++],gui_packet[element++]);
											 break;
									case 10: len=gui_packet[element++];
											 temp2=(byte *)malloc(sizeof(byte));
											 free(temp2);	
											 temp2=(byte *)malloc((len)*sizeof(byte));
											 for(count=0;count<len;count++)
											 temp2[count]=gui_packet[element++];
											 temp2[len]='\0';
											 FILE *find;
											 if(acyut==3)
											 {
											 	strcpy(move2,"acyut3/imu/calibration/");
											 	strcat(move2,temp2);
											 	strcat(move2,".bin");
											 	printf("file name::%s\n",move2);
											 	find=fopen(move2,"r");
											 	if(find!=NULL)
											 	{
											 		fclose(find);
											 		readfiles(temp2);
											 		Xcode=1;
											 		break;
											 	}
											 	if(offset_status==1)
											 		strcpy(move,"acyut3/master_offseted_moves/");
											 	else
											 		strcpy(move,"acyut3/moves/");
											 }
											 else if(acyut==4)
											 {
											 	strcpy(move2,"acyut4/imu/calibration/");
											 	strcat(move2,temp2);
											 	strcat(move2,".bin");
											 	printf("file name::%s\n",move2);
											 	find=fopen(move2,"r");
											 	if(find!=NULL)
											 	{
											 		fclose(find);
											 		readfiles(temp2);
											 		Xcode=1;
											 		break;
											 	}
											 	if(offset_status==1)
											 		strcpy(move,"acyut4/master_offseted_moves/");
											 	else
											 		strcpy(move,"acyut4/moves/");
											 }
											 else
											 Xcode=10;
											 strcat(move,temp2);
											 Xcode=(byte)act_fpacket(move);
											 //if(acyut==2)readfiles("walk1");
											 break;
									case 11: len=gui_packet[element++];
											 temp2=(byte *)malloc((len+1)*sizeof(byte));
											 if(acyut==3)
											 	if(offset_status==1)
											 		strcpy(move,"acyut3/master_offseted_moves/");
											 	else
											 		strcpy(move,"acyut3/moves/");
											 else if(acyut==4)
											 	if(offset_status==1)
											 		strcpy(move,"acyut4/master_offseted_moves/");
											 	else
											 		strcpy(move,"acyut4/moves/");
											 else
											 Xcode=10;
											 for(count=0;count<len;count++)
											 temp2[count]=gui_packet[element++];
											 temp2[len]='\0';
											 strcat(move,temp2);
											 Xcode=act_ppacket(move,gui_packet[element++],gui_packet[element++]);
											 break;
									case 12: len=gui_packet[element++];
											 temp2=(byte *)malloc(sizeof(byte));
											 free(temp2);
											 temp2=(byte *)malloc((len)*sizeof(byte));
											 for(count=0;count<len;count++)
											 temp2[count]=gui_packet[element++];
											 temp2[len]='\0';
											 Xcode=combine_play(temp2);
											 break;
									default:Xcode=102;
								};
								sleep(2);
								break;
						case 2:	Xcode=read_acyut();
								break;
						case 3:	Xcode=dir_tran();
								//printf("_______________________________________________________________________________________________________________________________________________DIRTRAN CALLED\n");
								gflag++;
								break;
						case 4: switch(gui_packet[element++])
								{
									case 0: Xcode=temp_save();
											break;
									case 1: Xcode=perm_save();
											break;
									case 2: Xcode=combine_save();
											break;
									default:Xcode=49;
								}
								break; 
						case 5: len=(int)gui_packet[element++];
								count=0;
								char filename[30];
								do
								{
									filename[count++]=gui_packet[element++];
								}while(count<len);
								filename[count]='\0';
								switch(gui_packet[element++])
								{
									case 0: remote_loop=gui_packet[element++];
											for(remote_loop_i=0;remote_loop_i<remote_loop;remote_loop_i++)
											{
												Xcode=(byte)remote_add(filename);
												if(Xcode!=1)
													printf("Error code :: %d in file no : %d\n",Xcode,remote_loop_i);
											}
											break;
									//case 1: Xcode=(byte)remote_edit(filename);
									//		break;
									case 2: Xcode=(byte)remote_del(filename);
											break;
									default:Xcode=49; 
											printf("\nwrong data in gui_packet\n");
								}
								break;
						case 6: switch(gui_packet[element++])
								{
									case 0:	len=gui_packet[element++];
											temp2=(byte *)malloc(sizeof(byte));
											free(temp2);	
											temp2=(byte *)malloc((len)*sizeof(byte));
											for(count=0;count<len;count++)
												temp2[count]=gui_packet[element++];
											temp2[len]='\0';
											calibrate_boundaries(temp2);
											break;
									case 1:	len=gui_packet[element++];
											temp2=(byte *)malloc(sizeof(byte));
											free(temp2);	
											temp2=(byte *)malloc((len)*sizeof(byte));
											for(count=0;count<len;count++)
												temp2[count]=gui_packet[element++];
											temp2[len]='\0';
											if(acyut==3)
											{
												strcpy(cmbine_name,"acyut3/combine/");
											}
											else if(acyut==4)
											{
												strcpy(cmbine_name,"acyut4/combine/");
											}
											strcat(cmbine_name,temp2);
											read_cmbine=fopen(cmbine_name,"r");
											if(read_cmbine==NULL)
											{
												Xcode=61;
												break;
											}
											fread(&moves_cmbine,sizeof(moves_cmbine),1,read_cmbine);
											iterator1=0;
											while(iterator1<moves_cmbine.no_of_files)
											{	
												calibrate_boundaries(moves_cmbine.cmb_moves[iterator1++].file_name);
											}
											fclose(read_cmbine);
											break;
									default:printf("Incorrect Options Packet\n");
											Xcode=9;
								}
								break;
						case 7:	imucal_init();
								break;
						case 8: Xcode=save_master_offsets();
								if(Xcode==1)
									Xcode=apply_master_offsets();
								printf("about to write\n");
								ftdi_write_data(&ftdic1,&startkar,1);
								printf("write successful\n");
								break;
						case 9:	status_loop=gui_packet[element++];
								for(status_loop_i=0;status_loop_i<status_loop;status_loop_i++)
								{
									Xcode=add_status();
									if(Xcode!=1)
										printf("Error code :: %d in file no : %d\n",Xcode,status_loop_i);
								}
						/*switch(gui_packet[element++])
								{
									/*case 0:
											break;
									case 1:	status_loop=gui_packet[element++];
											for(status_loop_i=0;status_loop_i<status_loop;status_loop_i++)
											{
												Xcode=del_status();
												if(Xcode!=1)
													printf("Error code :: %d in file no : %d\n",Xcode,status_loop_i);
											}
											break;
											break;
								}*/
								break;	
						default:printf("Incorrect Options Packet\n");
								Xcode=9;
					}
					if(Xcode==1)
						printf("SUCCESS\n");
					else
						printf("Error code :: %d\n",Xcode);
					ftdi_write_data(&ftdic2,&startkar,1);
					ftdi_write_data(&ftdic2,&startkar1,1);
					if(ftdi_write_data(&ftdic2,&Xcode,1)!=1)
						printf("FTDI write failed\n");
				}
				else
				ftdi_write_data(&ftdic2,&Xcode,1);
		
			}
			//		printf("%d\n",nbytes);
		
		
//		printf("%d\n",errno);
		}
		else if(run_imu==1&&imu_on==1)
		{
			/*if(frac<=24)
			{
				frac=kbhit2();
				
			}///golden ratio for motor 1/21/3/23 is  8/24=1/3
			///golden ratio for motor 0/20 is  7/24
			*/
			
			//printf("FRAC: %d",frac);
			{
				if(flag_imu_first==0)
				{
					//printf("INITIALIZING IMU\n\n");
 					imucal_init();
	 				//sleep(8);
					/*for (k = 0; k < 130; k++)
					printf("last_tx_packet[k]=%d\n",last_tx_packet[k]);*/
						
					for(k=0; k<28; k++)
					{
						base_val[last_tx_packet[7+k*3]]=last_tx_packet[8+k*3]+last_tx_packet[9+k*3]*256;
						//printf("BASE %d : %d \n",last_tx_packet[7+k*3],base_val[last_tx_packet[7+k*3]]);
						//last_tx_packet[k]=tx_packet[k];
						}
 				}
				//printf("checking imu\n");
				checksum_change=0;
				change[0]=change[1]=change[2]=0;
//------------------------------------------------------------------------------------------akash's imu read------------------------------------------------------------------------------------------------------
				/*if(flag_imu_first==0)
				{
					printf("calling imu cal init\n\n");
 					imucal_init();
 					sleep(8);
// 					for(k=0; k<130; k++)
 //						last_tx_packet[k]=tx_packet[k];
					printf("\n");
				}*/
	
				//printf("\n\n----------------------------------\n\n");
				//printf("READING IMU PACKET\n");
				ftdi_usb_purge_buffers(&imu);
			
				imucal();
			
			
				
				for(k=0; k<3 ; k++)
				{
					change_enuf[k]=0;
					change[k]=base_angle[k]-avg_angle[k];	
					if(( change[k] > (2) || change[k]< (-2) ) && ( change[k] < (50) || change[k]> (-50)))				// threshold changes to apply....
					{
						change_enuf[k]=1;
						//printf("Sufficient change in %d and change is: %lf and avg angle is %lf\n",k, change[k], avg_angle[k]);
						//printf("=> SUFFICIENT CHANGE\n");
					}
					if(change[k]<-60)
					{
						change_enuf[0]=2;
						printf("BACKGETUP RUN\n");
					}
					if(change[k]>60)
					{
						change_enuf[k]=3;
						printf("FRONTGETUP RUN\n");
					}
					printf("ANGLE %d VALUE %lf CHANGE %lf  CHANGE_ENUF %d\t",k,avg_angle[k],change[k],change_enuf[k]);
					change[k]=change[k]*(4096/300);
				}
			
				change[0]=(change[0]*4)/4;//4/4
				change[1]=(change[1]*1)/3;//2/4
				//printf("final......................................change in motor angles  is      %lf\n",change[0]);
//--	-----------------------------------------------------------------------------------akash's imu read ends---------------------------------------------------------------	-------------------------------------------
	
int l=0;
		/*for(l=0;l<128;l++)
		printf("%d\n",last_tx_packet[l]);
		
		printf("\n\n");
		*/
	
		//printf("FLAG_IMU_FIRST %d\n",flag_imu_first);
							
		//if((change[0]>5 || change[0]<-5) && (change[0]<70 || change[0]>-70))
		
			for(j=0; j<NOM; j++)
			{
				
				if(change_enuf[0]==1)
				{
					if(last_tx_packet[7+j*3]==3 || last_tx_packet[7+j*3]==23)
					{
						checksum_change-=(last_tx_packet[8+j*3]+last_tx_packet[9+j*3]);
						//base_val[last_tx_packet[7+j*3]]=last_tx_packet[8+j*3]+last_tx_packet[9+j*3]*256;
						//printf("BASE: %d \n",base_val[last_tx_packet[7+j*3]]);
						imu_gpos[last_tx_packet[7+j*3]]=base_val[last_tx_packet[7+j*3]];
						imu_gpos[last_tx_packet[7+j*3]]+=(change[0]*10)/24;// 3/24	//9/14//4/7//7/24
//						imu_gpos[last_tx_packet[7+j*3]]+=(change[0]*7)/5;
						//printf("position for id %d changed by %lf to new value %d\n",last_tx_packet[7+j*3],change[0], imu_gpos[last_tx_packet[7+j*3]]);
						last_tx_packet[8+j*3]=get_lb(imu_gpos[last_tx_packet[7+j*3]]);
						last_tx_packet[9+j*3]=get_hb(imu_gpos[last_tx_packet[7+j*3]]);
						checksum_change+=(last_tx_packet[8+j*3]+last_tx_packet[9+j*3]);
					}	
					if(last_tx_packet[7+j*3]==21)
					{
						checksum_change-=(last_tx_packet[8+j*3]+last_tx_packet[9+j*3]);
						//base_val[last_tx_packet[7+j*3]]=last_tx_packet[8+j*3]+last_tx_packet[9+j*3]*256;
						//printf("base val is: %d \n",base_val[last_tx_packet[7+j*3]]);
						imu_gpos[last_tx_packet[7+j*3]]=base_val[last_tx_packet[7+j*3]];
						imu_gpos[last_tx_packet[7+j*3]]-=(change[0]*6)/24;//3/24	//5/14//5/24
//						imu_gpos[last_tx_packet[7+j*3]]-=(change[0]*2)/5;
						//printf("position for id %d changed by %lf to new value %d\n",last_tx_packet[7+j*3],change[0], imu_gpos[last_tx_packet[7+j*3]]);
						last_tx_packet[8+j*3]=get_lb(imu_gpos[last_tx_packet[7+j*3]]);
						last_tx_packet[9+j*3]=get_hb(imu_gpos[last_tx_packet[7+j*3]]);
						checksum_change+=(last_tx_packet[8+j*3]+last_tx_packet[9+j*3]);
					}
					if(last_tx_packet[7+j*3]==1)
					{
						checksum_change-=(last_tx_packet[8+j*3]+last_tx_packet[9+j*3]);
						//base_val[last_tx_packet[7+j*3]]=last_tx_packet[8+j*3]+last_tx_packet[9+j*3]*256;
						//printf("base val is: %d \n",base_val[last_tx_packet[7+j*3]]);
						imu_gpos[last_tx_packet[7+j*3]]=base_val[last_tx_packet[7+j*3]];
						imu_gpos[last_tx_packet[7+j*3]]-=(change[0]*6)/24;//3/24	//5/14//3/7//5/24
//						imu_gpos[last_tx_packet[7+j*3]]-=(change[0]*2)/5;
						//printf("position for id %d changed by %lf to new value %d\n",last_tx_packet[7+j*3],change[0], imu_gpos[last_tx_packet[7+j*3]]);
						last_tx_packet[8+j*3]=get_lb(imu_gpos[last_tx_packet[7+j*3]]);
						last_tx_packet[9+j*3]=get_hb(imu_gpos[last_tx_packet[7+j*3]]);
						checksum_change+=(last_tx_packet[8+j*3]+last_tx_packet[9+j*3]);
					}
				}
				if(change_enuf[1]==1)
				{
					
					if(last_tx_packet[7+j*3]==0)
					{
						checksum_change-=(last_tx_packet[8+j*3]+last_tx_packet[9+j*3]);
						//base_val[last_tx_packet[7+j*3]]=last_tx_packet[8+j*3]+last_tx_packet[9+j*3]*256;
						//printf("base val is: %d \n",base_val[last_tx_packet[7+j*3]]);
						imu_gpos[last_tx_packet[7+j*3]]=base_val[last_tx_packet[7+j*3]];
						imu_gpos[last_tx_packet[7+j*3]]-=(change[1]*7)/24;
						//printf("position for id %d changed by %lf to new value %d\n",last_tx_packet[7+j*3],change[1], imu_gpos[last_tx_packet[7+j*3]]);
						last_tx_packet[8+j*3]=get_lb(imu_gpos[last_tx_packet[7+j*3]]);
						last_tx_packet[9+j*3]=get_hb(imu_gpos[last_tx_packet[7+j*3]]);
						checksum_change+=(last_tx_packet[8+j*3]+last_tx_packet[9+j*3]);
					}
					if(last_tx_packet[7+j*3]==20)
					{
						checksum_change-=(last_tx_packet[8+j*3]+last_tx_packet[9+j*3]);
						//base_val[last_tx_packet[7+j*3]]=last_tx_packet[8+j*3]+last_tx_packet[9+j*3]*256;
						//printf("base val is: %d \n",base_val[last_tx_packet[7+j*3]]);
						imu_gpos[last_tx_packet[7+j*3]]=base_val[last_tx_packet[7+j*3]];
						imu_gpos[last_tx_packet[7+j*3]]+=(change[1]*7)/24;
						//printf("position for id %d changed by %lf to new value %d\n",last_tx_packet[7+j*3],change[1], imu_gpos[last_tx_packet[7+j*3]]);
						last_tx_packet[8+j*3]=get_lb(imu_gpos[last_tx_packet[7+j*3]]);
						last_tx_packet[9+j*3]=get_hb(imu_gpos[last_tx_packet[7+j*3]]);
						checksum_change+=(last_tx_packet[8+j*3]+last_tx_packet[9+j*3]);
					}
				}
			}
				
			checksum_new=last_tx_packet[7+j*3];
			checksum_new=255-checksum_new;
			checksum_new+=checksum_change;
		//	printf("checksum change is %d and new checksum is %d\n", checksum_change, checksum_new);
			last_tx_packet[7+j*3]=(255-checksum_new);
//			last_tx_packet[7+j*3]=checksum_last();
	
			//if((change[0]>2 || change[0]<-2) && (change[0]<50 || change[0]>-50))
			if(change_enuf[0]==1||change_enuf[1]==1)
			ftdi_write_data(&ftdic1,last_tx_packet,last_tx_packet[3]+4);
	
	
			if(flag_imu_first==0)
			flag_imu_first=1;
	
			if(change_enuf[0]==2||change_enuf[1]==2)
			{
				printf("\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n.\n.\n.\n.\n.\nPREGETUP\n");
				speed(0);
				if(acyut==4)
				act_fpacket("acyut4/moves/pregetup");
				else if (acyut==3)
				act_fpacket("acyut3/moves/pregetup");
				speed(1);
				usleep(6000000);
				printf("BACKGETUP\n");
				sleep(3);
				if(acyut==4)
				act_fpacket("acyut4/moves/backgetup");
				else if (acyut==3)
				act_fpacket("acyut3/moves/backgetup");
				
				/*
				int l=0;
				for(l=0;l<128;l++)
				last_tx_packet[l]=tx_packet[l];
				*/
			}
			if(change_enuf[0]==3||change_enuf[1]==3)
			{
				printf("\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n.\n.\n.\n.\n.\nPREGETUP\n");
				speed(0);
				if(acyut==4)
				act_fpacket("acyut4/moves/pregetup");
				else if (acyut==3)
				act_fpacket("acyut3/moves/pregetup");
				speed(1);
				usleep(6000000);
				printf("FRONTGETUP\n");
				sleep(3);
				if(acyut==4)
				act_fpacket("acyut4/moves/frontgetup");
				else if (acyut==3)
				act_fpacket("acyut3/moves/frontgetup");
				
				/*
				int l=0;
				for(l=0;l<128;l++)
				last_tx_packet[l]=tx_packet[l];
				*/
			}
			}
		}
	}

	if((ret = ftdi_usb_close(&ftdic2)) < 0)
    {
        fprintf(stderr, "unable to close ftdi device: %d (%s)\n", ret, ftdi_get_error_string(&ftdic2));
        return EXIT_FAILURE;
    }
    else 
	{
	ftdi_deinit(&ftdic2);
//	free(&ftdic2);
    return EXIT_SUCCESS;
	}
}








void move_processing()
{
	char s[24]="righthandup",mn[24],t[100],temp='a';
	int id[31],pos[31],i;
	printf("Give move name (without \".move\" only) : ");
	scanf("%s",s);

	FILE *mb=fopen(s,"w");
	f=fopen(strcat(s,".omove\0"),"r");

	if(f==NULL)
	{
			printf("Unable to open file\n");
			return ;
	}
	int nof, num_motors;
	int address,time, counter=0;
	fscanf(f,"%s%d\n",mn,&nof);
	//printf("File name : %s\nNOF : %d\n",mn,nof);
	if(strcmp(mn,s)!=0)
	{	
	    printf("Error in file name.....\n");
		return ;
	}

	//printf("t.....\n");
	int j;
	move r;
	byte a,lb,hb;
	r.header[0]=r.header[1]=0xff;
	r.noframes=nof;
	for(j=0;j<nof;j++)
	{
		//printf("\n\n%d\n",j+1);
//		fscanf(f,"%s%d%d\n",t,&num_motors,&time);//&address);
		if(j==0)
		{
			fgets(t,100,f);
		}
		temp=fgetc(f);
		while(temp!='\t')
		{
		t[counter++]=temp;
		temp=fgetc(f);
		}	
		t[counter]='\0';
		temp='a';
		counter=0;
		fscanf(f,"%d%d%x\n",&num_motors,&time,&address);
		//printf("Frame name: %s\tNumber of motors: %d\tTime: %d\tAddress: %d\nValues:\n",t,num_motors,time,address);
		r.frames[j].checksum=0;
		r.frames[j].num_motors=num_motors;
		r.frames[j].address=address;
		r.frames[j].checksum+=num_motors+address;
		for(i=0;i<num_motors;i++)
		{
			fscanf(f,"%d%d\n",&id[i],&pos[i]);
			//printf("%d	%d\n",id[i],pos[i]);
			a=id[i];
			if(address==0x1c || address==0x1a)	//Slope/Margin
			{
				lb=pos[i];			//CW Slope/Margin
				hb=pos[i];			//CCW Slope/Margin
			}
			else
			{
				lb=pos[i] & 255;
				hb=pos[i]>>8;
			}
			r.frames[j].motorvalues[i].LB=lb;
			r.frames[j].motorvalues[i].HB=hb;
	//		printf("lb	: %d	hb: %d\n",lb,hb);
			r.frames[j].checksum+=a+lb+hb;
			r.frames[j].motorvalues[i].ID=a;
		}
		r.frames[j].checksum=255-r.frames[j].checksum;
	}
	
	fwrite(&r,sizeof(r),1,mb);
	//	int qw=113;
	//	byte asd=qw;
/*	int d=2456;
	lb=d & 255;
	hb=d>>8;
	printf("\nlb: %d	hb: %d	~lb: %d\n",lb,hb,255-lb);
	printf("%d	%d\n",hb*256+lb,d);
*/
	fclose(f);
	fclose(mb);

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
			ftdi_write_data(&ftdic1,txp,txp[3]+4);
			noat2=0;
			while(noat2<5)
			{
				noat2++;
				//printf("COUNT:%d \n",noat2);
				if((ftdi_read_data(&ftdic1,&chkff,1)>0)&&chkff==0xff)
				count--;
				else
				count=2;
				//printf("pack: %d\n",chkff);
				
				if(count==0)
				{
					ftdi_read_data(&ftdic1,rxp,4);
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
//		for (j = 0; j < 6; j+= 1)
//			printf("%x\n",txp[j]);
/*		usleep(50000);
		
		ftdi_read_data(&ftdic1,rxp,12);
		c=1;	
		
		for (j = 1; j < 8; j += 1)
		{

			if(rxp[j]==0xff && rxp[j-1]==0xff)
			{
				printf("packet rec\n");
				if(rxp[j+1]==txp[2] && rxp[j+2]==2)
				{
					if(rxp[j+3])
					{
						printf("\nerror in motor %d :	%d\n",bid[i],rxp[j+3]);
						for (ping_test_k = 0; ping_test_k < sizeof(ignore_ids)/sizeof(byte); ping_test_k += 1)
						{
							if(rxp[j+1]==ignore_ids[ping_test_k])
							{
								printf("****%d has been ignored****\n",ignore_ids[ping_test_k]);
								er+=i;
							}
						}
					}
					else
						{
							printf("\nsuccess pinging motor : %d\n",bid[i]);c=0;
							er+=i;
						}
				}
			}
			
			else if(j==7 && c)
			{
				printf("\nfailure pinging motor : %d\n",bid[i]);
				for (ping_test_k = 0; ping_test_k < sizeof(ignore_ids)/sizeof(byte); ping_test_k += 1)
				{
					if(rxp[j+1]==ignore_ids[ping_test_k])
					{
						printf("****%d has been ignored****\n",ignore_ids[ping_test_k]);
						er+=i;
					}
				}
			}
		}
		
	}
*/
	//printf("%d",er);
	//printf("er = %d\n",er);
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

void reset()
{
	int n;
	//printf("ENTER THE ID \n");
	//scanf("%d",&n);
	/*n=22;
	byte pack[]={0xff,0xff,n,0x04,0x03,0x03,2,0x00};
	pack[7]=0xff-(n+0x04+0x03 +0x03 +12);
	//act_fpacket("acyut4/moves/ml");*/
	//act_fpacket("acyut4/moves/init4");
	
}

void initialize_acyut(int acyut)
{
	speed(0);
	if(acyut==4)
	act_fpacket("acyut4/moves/init4");
	else if (acyut==3)
	act_fpacket("acyut3/moves/init3");
	sleep(10);
	speed(1);
	sleep(5);
}
					

int main()
{
	int call=0;
	if(bootup_files()==EXIT_FAILURE)
	printf("\nUnable to open USB2DYNAMIXEL serial or imu serial\n");
	
	if(pingtest())
	call=1;
	else
	{
		printf("Bypass ping?(y/n)\n");
		char ch='n';
		scanf("%c",&ch);
		if(ch=='y'||ch=='Y')
		call=1;
	
	}
	if(call==1)
	{
		acyut=4;
		printf("use GUI...\n");
		int var;
		//printf("\n\nGO GO GO!!!!\n");
		printf("///////MENU////////\n1. DRIBBLE\n2. WALK\n3. WALK (SIN CURVE)\n");
		scanf("%d",&var);
		
		switch(var)
		{
			case 1: initialize_acyut(acyut);
					sleep(1);
					make_dribble();
					break;				
			case 2: initialize_acyut(acyut);
					sleep(1);
					make_walk();
					break;				
			case 3 : initialize_acyut(acyut);
					sleep(1);
					make_walk2();
					break;
					
			default:printf("LALALA\n");
		}
		//if(bluecall()==EXIT_FAILURE)
		//printf("Accessing error bluetooth serial device\n");
	}
	else 
	{
		printf("-----------------ACYUT BACKEND CLOSING-----------------\n");
		return 1;
	}
	
	
	/*{
		char s[15];
		int opt=1;
		int tp;
		while(opt)
		{
			printf("1...............Calibrate a specific move with IMU.\n");
			printf("2...............View Bluetooth data.\n");
			printf("3...............Play a aspecific move with IMU\n");
			printf("4...............Speed min\n");
			printf("5...............Speed max\n");
			printf("6...............Dance\n");
			printf("7...............Istart\n");
			printf("8...............Change Drivemode\n");		
			printf("9...............Graphing\n");
			printf("0...............Exit\n");
			printf("Give an option : ");
			scanf("%d",&opt);
			switch(opt)
			{
				case 1: {
							
							printf("Enter move name : ");
							scanf("%s",s);
							calibrate_boundaries(s);
							//printf("Do you want to play the imu corrected move???? (1:YES 0:NO) ");
							//scanf("%d",&c);
							//if(c==1)
							{
							//	printf("playing move %s\n",s);
							//	readfiles(s);
							}
							break;
						}
				case 2: {
							if(bluecall()==EXIT_FAILURE)printf("Accessing error bluetooth serial device\n");
							break;
						}
				case 3: {	printf("Enter move name : ");
							scanf("%s",s);
							printf("playing move %s\n",s);
							readfiles(s);						
							break;
						}
				case 4: {
							speed(0);
							break;
						}
				case 5: {
							speed(1);
							break;
						}
				case 6:
					dance();
					break;
				case 7:
					istart();
					break;
				case 8:
							tp=0;
							printf("Calibrate walk(0) or IMU running longwalk(1).....? ");
							scanf("%d",&tp);
							if(tp==0)	
								calibration();
							else
								longwalk();
					
				
//					drivemode();

					break;
				case 9 : 
							tp=0;
							printf("Calibrating time(0) or IMU running time(1).....? ");
							scanf("%d",&tp);
							if(tp==0)	
								graph(boundaries,boundaries_size);
							else
								graph(local_boundaries,boundaries_size);
					
							break;
				case 0: break;

			}
		}
	}*/
	close_files();
	return 0;
}
