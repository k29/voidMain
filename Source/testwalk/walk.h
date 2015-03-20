#ifndef _WALK_H_
#define _WALK_H_

// #include "testBot.h"
// #include "communication.h"
// #include "commondefswalk.h"
#include <cstdio>
#include <cstdlib>


#define broadcast_id 0xfe
#define id1 0
#define id2 1 


typedef char byte;

enum {SSP,DSP};


class Walk
{
public:
	Walk();
	void move(double r,double theta);
};
#endif
