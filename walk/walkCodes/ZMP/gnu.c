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

void main()
{

	int n[]={107,109,111,352,660,661,662,664,674,728,730,739,740};
	//scanf("%d",&n);
	char STR[4];
	char comfile[100]="\0";
	char expect_com[100]="\0";
	char output[100]="\0";
	FILE *f;
	int lasthit=0;
	int i=0;

	while(1)
	{
		i++;
		sprintf(STR,"%d",n[i]);
		strcat(comfile,"Graphs/COM");
		strcat(comfile,STR);
		strcat(expect_com,"Graphs/Expected");
		strcat(expect_com,STR);
		strcat(output,"OUTPUT");
		strcat(output,STR);
		printf("%d\n",n[i]);
		f=fopen("gnu1","w");
		if(f!=NULL)
		{	
			fprintf(f,"set terminal jpeg\n");
			fprintf(f,"set output \"%s.jpeg\"\n",output); ///fprintf(f,"set terminal X11\n");
			fprintf(f,"plot \"%s\" using 2 with lines, \"%s\" using 2 with lines\n",comfile,expect_com);
			fclose(f);
			system("gnuplot < \"gnu1\" -persist");
		}
		comfile[0]='\0';
		expect_com[0]='\0';
		output[0]='\0';
		sleep(1);
		
	}
}
