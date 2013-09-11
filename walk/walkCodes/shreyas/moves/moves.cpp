#include "AcYut.h"
#include "communication.h"
#include "xsens/imu.h"

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
//	    printf("eqn");
	    output[tp]=ga[gt];
//	    printf("%f    ",ga[gt]);
	    tp++;
	    output[tp]=gb[gt];
	    //printf("%f    ",gb[gt]);
	    tp++;
	    output[tp]=gc[gt];
	    //printf("%f    ",gc[gt]);
	    tp++;
	    output[tp]=gd[gt];
	    //printf("%f    \n",gd[gt]);
	    tp++;
	  }
	  return output;
}

int main()
{
	Imu imu;
	imu.init();
	
	Communication comm;
	AcYut bot(&comm,NULL);

	// LEFT HAND
	int mot07[]={};
	int mot08[]={};
	int mot10[]={};
	
	//RIGHT HAND
	int mot27[]={};
	int mot28[]={};
	int mot30[]={};

	//LEFT LEG
	int llegx[]={};
	int llegy[]={};
	int llegz[]={};
	
	//RIGHT LEG
	int rlegx[]={};
	int rlegy[]={};
	int rlegz[]={};
	
	//TIME
	float t1[]={};
	float t2[]={};
