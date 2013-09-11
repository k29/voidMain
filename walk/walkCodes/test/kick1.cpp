int walk(int leg,float yin,float yfi)
{
	
	//byte im[8];
	float change[3]={0};
	float tchange[3]={0};
	int count=0;
	int aftr_frames=2;
	int noat=0,a=0,b=0,c=0;
	int nof=0;
	float present_boundary[3]={0};
	float future_boundary[3]={0};

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

