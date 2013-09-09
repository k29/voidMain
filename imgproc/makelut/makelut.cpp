#include <cstdio>

typedef unsigned char uchar;

#define NOCOLS 7 //Plz update this!
#define YELLOWC 4
#define GREENC 2
#define BLUEC 1
#define ORANGEC 8
#define BLACKC 16
#define REDC 32
#define WHITEC 64

inline int doCheck3C(int a, int b, int c, int mina, int maxa, int minb, int maxb, int minc, int maxc)
{
	if((a<=maxa)&&(a>=mina)&&(b<=maxb)&&(b>=minb)&&(c<=maxc)&&(c>=minc))
	{
		return 1; 
	}
	else
		return 0;
}

uchar* marker_create_lut_red()
{
	uchar* lut = new unsigned char [256*256*256];//[16777216];	//256*256*256

	float r,g,b;
	float Reds, Greens, Blues, Ys, Crs, Cbs;
	int value;

	for(r=0.0; r<256.0; r++)
	for(g=0.0; g<256.0; g++)
	for(b=0.0; b<256.0; b++)
	{
// 		value = 0;
		Ys = 0.299*r + 0.587*g + 0.114*b;
     	Crs = (r-Ys)*0.713 + 128.0;
     	Cbs = (b-Ys)*0.564 + 128.0;
//     	Reds = r/255.0;
//     	Greens = g/255.0;
//     	Blues = b/255.0;
// 		//Values hardcoded. Should be loaded from config file
// 		//Red (means ball color)
// 		//if(doCheck3C(Ys, Crs, Cbs, 0, 203, 153, 255, 66, 116))
// 		//if(doCheck3C(Ys, Hue, Sat, 46, 255, 194, 255, 128, 164))
// 	//	if(doCheck3C(Ys, Crs, Cbs, 0, 255, 132, 255, 0, 129))
// 		if(doCheck3C(Ys, Crs, Cbs, 0, 255, 131, 255, 0, 127))
// 			value += REDC;
// 		//Blue	
// 		//if(doCheck3C(Ys, Crs, Cbs, 134, 255, 66, 115, 136, 164))
// 		if(doCheck3C(Ys, Crs, Cbs, 90, 255, 131, 145, 115, 149))
// 		value+=BLUEC;
// 		// Yellow 
		
// 		if(doCheck3C(Ys, Crs, Cbs, 0, 255, 2, 161, 70, 104))
// 		value+=YELLOWC;
// 		//  Green
// 		//RGB
// 		if((Greens<2.9*Reds)&&(Greens>2.1*Reds)&&(Blues<2.1*Reds)&&(Blues>1.5*Reds))
// 		value+=GREENC;
// 		//YUV
// //		if(doCheck3C(Ys, Crs, Cbs, 0, 147, 70, 111, 210, 134))
// //		value+=GREENC
// //		// Orange
// //		if((Reds>0.50)&&(Greens>1.45*Blues)&&(Greens<2.5*Blues)&&(Reds>1.4*Greens))
// //		value+=ORANGEC;
// 		// Black
// 		if((Reds<0.15)&&(Greens<0.15)&&(Blues<0.15)&&(Reds>0.06)&&(Greens>0.06)&&(Blues>0.05))
// 		value+=BLACKC;

// 		// White
// 	//	if(doCheck3C(Ys, Crs, Cbs, 237, 255, 89, 130, 122, 133))
// 		if(Reds>0.7&&Greens>0.7&&Blues>0.7)
// 		value+=WHITEC;
		//if(doCheck3C(Ys, Crs, Cbs, 0, 255, 131, 255, 0, 127)) //60fps
		//if(doCheck3C(Ys, Crs, Cbs, 0, 255, 143, 255, 0, 255))	//15fps
     	//if(doCheck3C(Ys, Crs, Cbs, 0, 255, 166, 255, 0, 255))
    // 	if(doCheck3C(Ys, Crs, Cbs, 0, 255, 170, 255, 0, 255))
    // 	if(doCheck3C(Ys, Crs, Cbs, 0, 255, 139, 255, 0, 255))
     //	if(doCheck3C(Ys, Crs, Cbs, 0, 80, 135, 146, 113, 128))
     	//if(doCheck3C(Ys, Crs, Cbs, 0, 91, 133, 255, 0, 121))
    // 	if(doCheck3C(Ys, Crs, Cbs, 0, 255, 135, 255, 0, 255))
     	if(doCheck3C(Ys, Crs, Cbs, 0, 255, 136, 255, 0, 255))
		{
			lut[(uchar)r + (((uchar)g)<<8) + (((uchar)b)<<16)] = 255;
		}
		else
		{
			lut[(uchar)r + (((uchar)g)<<8) + (((uchar)b)<<16)] = 0;
		}

	}
	return lut;
}


uchar* marker_create_lut_yellow()
{
	uchar* lut = new unsigned char [256*256*256];//[16777216];	//256*256*256

	float r,g,b;
	float Reds, Greens, Blues, Ys, Crs, Cbs;
	int value;


	for(r=0.0; r<256.0; r++)
	for(g=0.0; g<256.0; g++)
	for(b=0.0; b<256.0; b++)
	{
		 Ys = 0.299*r + 0.587*g + 0.114*b;
     	Crs = (r-Ys)*0.713 + 128.0;
     	Cbs = (b-Ys)*0.564 + 128.0;

	 //if(doCheck3C(Ys, Crs, Cbs, 0, 255, 2, 161, 70, 104))
	 // if(doCheck3C(Ys, Crs, Cbs, 0, 255, 0, 130, 0, 118))
     //	if(doCheck3C(Ys, Crs, Cbs, 0, 255, 0, 255, 0, 121))
     //	if(doCheck3C(Ys, Crs, Cbs, 0, 255, 0, 129, 0, 119))
	if(doCheck3C(Ys, Crs, Cbs, 0, 255, 0, 132, 0, 122))
		{
			lut[(uchar)r + (((uchar)g)<<8) + (((uchar)b)<<16)] = 255;
		}
		else
		{
			lut[(uchar)r + (((uchar)g)<<8) + (((uchar)b)<<16)] = 0;
		}

	}
	return lut;
}


uchar* marker_create_lut_blue()
{
	uchar* lut = new unsigned char [256*256*256];//[16777216];	//256*256*256

	float r,g,b;
	float Reds, Greens, Blues, Ys, Crs, Cbs;
	int value;


	for(r=0.0; r<256.0; r++)
	for(g=0.0; g<256.0; g++)
	for(b=0.0; b<256.0; b++)
	{
		 Ys = 0.299*r + 0.587*g + 0.114*b;
     	Crs = (r-Ys)*0.713 + 128.0;
     	Cbs = (b-Ys)*0.564 + 128.0;

	//	if(doCheck3C(Ys, Crs, Cbs, 72, 224, 124, 255, 0, 255))
     //	if(doCheck3C(Ys, Crs, Cbs, 62, 255, 123, 255, 129, 154))
     //	if(doCheck3C(Ys, Crs, Cbs, 45, 255, 125, 255, 129, 255))
    //	if(doCheck3C(Ys, Crs, Cbs, 40, 255, 124, 255, 129, 255))
		// if(doCheck3C(Ys, Crs, Cbs, 51, 255, 125, 132, 128, 255))
		if(doCheck3C(Ys, Crs, Cbs, 48, 255, 124, 255, 128, 255))
		{
			lut[(uchar)r + (((uchar)g)<<8) + (((uchar)b)<<16)] = 255;
		}
		else
		{
			lut[(uchar)r + (((uchar)g)<<8) + (((uchar)b)<<16)] = 0;
		}

	}
	return lut;
}



uchar* marker_create_lut_green()
{
	uchar* lut = new unsigned char [256*256*256];//[16777216];	//256*256*256

	float r,g,b;
	float Reds, Greens, Blues, Ys, Crs, Cbs;
	int value;


	for(r=0.0; r<256.0; r++)
	for(g=0.0; g<256.0; g++)
	for(b=0.0; b<256.0; b++)
	{
		 Ys = 0.299*r + 0.587*g + 0.114*b;
     	Crs = (r-Ys)*0.713 + 128.0;
     	Cbs = (b-Ys)*0.564 + 128.0;

    	Reds = r/255.0;
    	Greens = g/255.0;
    	Blues = b/255.0;
		//if(doCheck3C(Ys, Crs, Cbs, 0, 255, 0, 93, 0, 134))
	//	if(doCheck3C(Ys, Crs, Cbs, 0, 53, 105, 165, 112, 140))
     //	if((Greens<2.90*Reds)&&(Greens>1.73*Reds)&&(Blues<2.45*Reds)&&(Blues>1.50*Reds))
    	if((Greens<2.90*Reds)&&(Greens>1.71*Reds)&&(Blues<2.26*Reds)&&(Blues>1.19*Reds))
		{
			lut[(uchar)r + (((uchar)g)<<8) + (((uchar)b)<<16)] = 255;
		}
		else
		{
			lut[(uchar)r + (((uchar)g)<<8) + (((uchar)b)<<16)] = 0;
		}

	}
	return lut;
}


uchar* marker_create_lut_white()
{
	uchar* lut = new unsigned char [256*256*256];//[16777216];	//256*256*256

	float r,g,b;
	float Reds, Greens, Blues, Ys, Crs, Cbs;
	int value;


	for(r=0.0; r<256.0; r++)
	for(g=0.0; g<256.0; g++)
	for(b=0.0; b<256.0; b++)
	{
		 Ys = 0.299*r + 0.587*g + 0.114*b;
     	Crs = (r-Ys)*0.713 + 128.0;
     	Cbs = (b-Ys)*0.564 + 128.0;

		//if(doCheck3C(Ys, Crs, Cbs, 218, 255, 0, 255, 0, 255))
		//if(doCheck3C(Ys, Crs, Cbs, 50, 255, 0, 120, 90, 221))
		if(doCheck3C(Ys, Crs, Cbs, 50, 255, 0, 255, 4, 255))
		{
			lut[(uchar)r + (((uchar)g)<<8) + (((uchar)b)<<16)] = 255;
		}
		else
		{
			lut[(uchar)r + (((uchar)g)<<8) + (((uchar)b)<<16)] = 0;
		}

	}
	return lut;
}



uchar* marker_create_lut_black()
{
	uchar* lut = new unsigned char [256*256*256];//[16777216];	//256*256*256

	float r,g,b;
	float Reds, Greens, Blues, Ys, Crs, Cbs;
	int value;


	for(r=0.0; r<256.0; r++)
	for(g=0.0; g<256.0; g++)
	for(b=0.0; b<256.0; b++)
	{
		 Ys = 0.299*r + 0.587*g + 0.114*b;
     	Crs = (r-Ys)*0.713 + 128.0;
     	Cbs = (b-Ys)*0.564 + 128.0;

		//if(doCheck3C(Ys, Crs, Cbs, 218, 255, 0, 255, 0, 255))
		//if(doCheck3C(Ys, Crs, Cbs, 50, 255, 0, 120, 90, 221))
		if(doCheck3C(Ys, Crs, Cbs, 0, 14, 0, 255, 0, 255))		
		{
			lut[(uchar)r + (((uchar)g)<<8) + (((uchar)b)<<16)] = 255;
		}
		else
		{
			lut[(uchar)r + (((uchar)g)<<8) + (((uchar)b)<<16)] = 0;
		}

	}
	return lut;
}



int main()
{
	int choice;
	uchar* mylut;
	FILE *fp;
	printf("Which lookup table do you want to make?\n");
	printf("1. Red\n2. Blue\n3. Yellow\n4. Green\n5. White\n6. Black\n7. All\nEnter choice: ");
	scanf("%d", &choice);
	switch(choice)
	{
		case 1:
		printf("Making red lut\n");
		mylut = marker_create_lut_red();
		printf("done\n");
		
		fp=fopen("../red.lut","wb");
		fwrite(mylut,sizeof(uchar),256*256*256,fp);
		fclose(fp);
		break;


		case 2:
		printf("Making blue lut\n");
		mylut = marker_create_lut_blue();
		printf("done\n");
		
		fp=fopen("../blue.lut","wb");
		fwrite(mylut,sizeof(uchar),256*256*256,fp);
		fclose(fp);
		break;


		case 3:
		printf("Making yellow lut\n");
		mylut = marker_create_lut_yellow();
		printf("done\n");
		
		fp=fopen("../yellow.lut","wb");
		fwrite(mylut,sizeof(uchar),256*256*256,fp);
		fclose(fp);
		break;


		case 4:
		printf("Making green lut\n");
		mylut = marker_create_lut_green();
		printf("done\n");
		
		fp=fopen("../green.lut","wb");
		fwrite(mylut,sizeof(uchar),256*256*256,fp);
		fclose(fp);
		break;


		case 5:
		printf("Making white lut\n");
		mylut = marker_create_lut_white();
		printf("done\n");
		
		fp=fopen("../white.lut","wb");
		fwrite(mylut,sizeof(uchar),256*256*256,fp);
		fclose(fp);
		break;

		case 6:
		printf("Making black lut\n");
		mylut = marker_create_lut_black();
		printf("done\n");
		
		fp=fopen("../black.lut","wb");
		fwrite(mylut,sizeof(uchar),256*256*256,fp);
		fclose(fp);
		break;


		case 7:
		printf("Making red lut\n");
		mylut = marker_create_lut_red();
		printf("done\n");
		
		fp=fopen("../red.lut","wb");
		fwrite(mylut,sizeof(uchar),256*256*256,fp);
		fclose(fp);
		
		printf("Making blue lut\n");
		mylut = marker_create_lut_blue();
		printf("done\n");
		
		fp=fopen("../blue.lut","wb");
		fwrite(mylut,sizeof(uchar),256*256*256,fp);
		fclose(fp);

		printf("Making yellow lut\n");
		mylut = marker_create_lut_yellow();
		printf("done\n");
		
		fp=fopen("../yellow.lut","wb");
		fwrite(mylut,sizeof(uchar),256*256*256,fp);
		fclose(fp);

		printf("Making green lut\n");
		mylut = marker_create_lut_green();
		printf("done\n");
		
		fp=fopen("../green.lut","wb");
		fwrite(mylut,sizeof(uchar),256*256*256,fp);
		fclose(fp);

		printf("Making white lut\n");
		mylut = marker_create_lut_white();
		printf("done\n");
		
		fp=fopen("../white.lut","wb");
		fwrite(mylut,sizeof(uchar),256*256*256,fp);
		fclose(fp);

		printf("Making black lut\n");
		mylut = marker_create_lut_black();
		printf("done\n");
		
		fp=fopen("../black.lut","wb");
		fwrite(mylut,sizeof(uchar),256*256*256,fp);
		fclose(fp);

		break;

		default:
		printf("Incorrect choice entered.\n");
	}
		return 0;
}
