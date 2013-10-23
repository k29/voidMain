#include "path.hpp"
#define deg2rad(x) (x)*(3.1415/180.0)

PathStructure ps; //Coming from main cognition.
PathPacket pathpackvar; //The class for writing the files.
void call_main() //Function made to replicate path calling from the cognition module.
{
	srand(time(NULL));
	ps.n_obstacles = 2;
	ps.absObstacles[0].x=20; 
	ps.absObstacles[0].y=31;
	ps.absObstacles[1].x=60 ;
	ps.absObstacles[1].y=100;
	ps.absObstacles[2].x=-40;
	ps.absObstacles[2].y=-80;

	ps.ball.x =110;
	ps.ball.y =110;

	ps.goal.x=190;
	ps.goal.y=-	180;
	for(int i=0;i<ps.n_obstacles;i++)
	{
		// ps.absObstacles[i].x=rand() % 200 - 100;
		// ps.absObstacles[i].y=rand() % 200 - 100;
	}
	// ps.ball.x=rand() % 200 - 100;
	// ps.ball.y=rand() % 200 - 100;	
	// ps.goal.x = rand() % 200 - 100;
	// ps.goal.y = rand() % 200 - 100;s	
	int r=200;
	int theta=100;
	
	Path p;
	// for(int r=50;r>0;r--)
	// 	{		
	// 		ps.ball.x=r*cos(deg2rad(theta));
	// 		ps.ball.y=r*sin(deg2rad(theta));
	// 		std::cout<<"\n\nr...."<<r<<"\n\n\n";
	// 		p.path_return(ps); // the main function that is called which returns the final path.
	// 		cvWaitKey();
	// 	}
	p.path_return(ps);
	p.updatePathPacket(pathpackvar);//true if file writing to be on...else off.
	cvWaitKey();
}
int main()
{
	call_main();
	return 0;
}

