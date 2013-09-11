
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
							base_acc[k]+=(acc[k])*9.8/260;	
						}
						for(j=0,k=0;j<6;j+=2,k+=1)
						{
							gyro[k]=im[j+12]+im[j+13]*256;
							gyro[k]=(gyro[k] - 0) * (300 + 300) / (60000 - 0) + -300;
							base_gyro[k]+=gyro[k];	
						}
						//printf("FLAG : %d BASE_ANG 0: %f BASE_ANG 1: %f BASE_ANG 2: %f\n",flag,base_angle[0],base_angle[1],base_angle[2]);				
						//printf("FLAG : %d BASE_ANG 0: %f BASE_ANG 1: %f BASE_ANG 2: %f BASE_ACC 0: %f BASE_ACC 1: %f BASE_ACC 2: %f BASE_GYRO 0: %f BASE_GYRO 1: %f BASE_GYRO 2: %f\n",flag,ang[0],ang[1],ang[2],acc[0],acc[1],acc[2],gyro[0],gyro[1],gyro[2]);	
						
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
		gravity_magnitude=sqrt(pow(base_acc[0],2)+pow(base_acc[1],2)+pow(base_acc[2],2));
		
		gravity[0]=0;//base_acc[0];
		gravity[1]=0;//base_acc[1];
		gravity[2]=base_acc[2];
		printf("GRAVITY : %f\t%f\t%f\n",gravity[0],gravity[1],gravity[2]);		
	}
	
}

int initialize_acyut()
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



int walk_ahead(int leg)
{
	
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

	initialize_acyut();
	imu_initialize();
	walk4(flag);
	while(!kbhit2())
	{
		//printf("%d\n",read_position(5));
		walk3(flag,-STEP,STEP,STEP,-STEP,0,0,0,0,0,0,0,0);
		flag=1-flag;
		//projection_system(flag);
		//usleep(16670*5);
	}
