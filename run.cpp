#include <curses.h>
#include <time.h>
#include "./Source/common/common.h"
#include "./Source/walk/walk_thread.h"
#include "./Source/gameController/gamecontrollerfunc.h"

pthread_t thread_id_walk;
pthread_t thread_id_gc;
pthread_mutex_t mutex_walkflag=PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t mutex_walkstr=PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t mutex_bodycommand=PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t mutex_changewalkflag=PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t mutex_GCData=PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t mutex_pathpacket=PTHREAD_MUTEX_INITIALIZER;

//Imu imu;
WalkStructure prevwalkstr;
WalkStructure walkstr;
RoboCupGameControlData GCData;
PathPacket pathpackvar;


 
int main(void)
{	
	
	// Create walkthread 
	
	//pthread_create (&thread_id_walk, NULL, walk_thread, NULL);
	#ifdef GC_IS_ON
    pthread_create (&thread_id_gc, NULL, readGameController, NULL);
	#endif
    
  	 /* main.cpp */

    registerXABSL();
	
	/* main.cpp */
	start();



    #ifdef GC_IS_ON
	pthread_join(thread_id_gc,NULL);
	#endif
	pthread_join(thread_id_walk,NULL);


	return 0;
}
