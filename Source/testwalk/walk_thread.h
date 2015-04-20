#ifndef WALK_THREAD_H
#define WALK_THREAD_H

#include "../common/common.h" 
#include "walk.h"
#include "../behavior/motionmodel.h"

#include <signal.h>
#include <fstream>
#include <opencv2/opencv.hpp>  
#include <ncurses.h>
#include <stdio.h>
#include <math.h>
#include <time.h>
#include <cstring>
#include <set>
#include <unistd.h>
#include <sys/ioctl.h>	
#include <sys/types.h>
#include <inttypes.h>
#include <sys/time.h>
#include <termios.h>

void* walk_thread(void*);



int arc_angle(float x1, float x2, float y1, float y2);

int forward_move_length(float x1, float x2, float y1, float y2);

bool vector_cross_reflex();   // whether to follow relfex or non reflex angle

int vector_cross_clock();  // whether to turn clockwise or anticlockwise

#endif