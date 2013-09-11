#ifndef COMMONDEFS_H
#define COMMONDEFS_H

//enum Goal{BLUE,YELLOW};
enum BallReturns{BALLFOUND=0,BALLFINDING=1,TURNRIGHT,TURNLEFT};
enum GoalReturns{GOALFINDING,ENCIRCLER,ENCIRCLEL,GOALFOUND};
typedef struct 
{
	float r;
	float theta;
	int updated;
}MotionModel;

typedef struct 
{
	float r;
	float theta;
}Coords;

//These are the variables that must be present in CamControl class,
//And must store the corresponding data.
	// Coords bluegoal;
	// Coords yellowgoal;
	// Coords ball;

#endif