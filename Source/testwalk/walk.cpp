#include "walk.h"

#include <cstdio>
// #include <iostream>
#include <string.h>


// #define WHEELDIAmm 100.0 //mm
// #define WHEELDIST 240.0 //mm
#define tbFORWARD 0
#define tbBACKWARD 1
#define tbLEFT 3
#define tbRIGHT 2
#define tbSTOP 0,0

// struct ftdi_context ftdi;
byte packet[18];
byte packet_recieved[18];
int packet_length;
int len_to_read;

Walk::Walk()
{
	// printf("efwo\n");
}

/*Walk::Walk(testBot* bot)
{
	this->bot=bot;
	WHEELDIAmm=this->bot->wheelDiameter;
	WHEELDIST=this->bot->wheelDistance;
	omega=this->bot->angularVelocity;
	// ftdi_bootup();
	// start2();
	printf("START completed\n\n\n");
};*/

/* theta is in radians , not degrees*/
void Walk::move(double walkr,double walktheta)
{


	printf("Move called with instructions %lf %lf\n",walkr,walktheta);

	/*if(walktheta>=0&&walktheta<=180)
		walktheta+=180;
	else
		walktheta=walktheta-180;


	walkr=(walkr+2.5)/0.145;*/ // Already incorporated in ARDUINO'S CODE


	FILE *file;

    //char S1[MAX_STRING_LENGTH],S2[MAX_STRING_LENGTH];

	char S1[80],S2[80];
    
    file = fopen("/dev/ttyACM0","w");  //Opening device file
    // printf("file opened\n");
    if(file==NULL)
    	printf("File not opened\n");
    sprintf(S1, "%d", (int)walkr);
    sprintf(S2, "%d", (int)walktheta);
    // printf("\n Radius in path fn to serial monitor %s\n angle in path fn to serial monitor %s  \n",S1,S2 );
    fprintf(file,"%s %s\n",S1,S2);
    printf("\n Radius in path fn to serial monitor %s\n angle in path fn to serial monitor %s  \n",S1,S2 );



    fclose(file);
		return ;
	
}