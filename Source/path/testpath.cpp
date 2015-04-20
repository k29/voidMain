#include "path.hpp"

#define TESTPATH

using namespace cv;
using namespace std;

PathStructure ps; //Coming from main cognition.
PathPacket pathpackvar; //The class for writing the files.

double test_x = 50, test_y = 50;
double ball_x = 25, ball_y = 25;

void callBack(int event, int x, int y, int flags, void* userdata)
{
     
     if  ( event == EVENT_LBUTTONDOWN )
     {
        cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
        test_x = x - 250;
        test_y = y - 250;
     }
     else if  ( event == EVENT_RBUTTONDOWN )
     {
        cout << "Right button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
        ball_x = x - 250;
        ball_y = y - 250;
     }
     else if  ( event == EVENT_MBUTTONDOWN )
     {
        cout << "Middle button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
      	test_x = x - 250;
     	test_y = y - 250;
     }
     // else if ( event == EVENT_MOUSEMOVE )
     // {
     //      cout << "Mouse move over the window - position (" << x << ", " << y << ")" << endl;
     // }
}

void call_main() //Function made to replicate path calling from the cognition module.
{
	srand(time(NULL));
	
	// extern double test_x,test_y;
	ps.n_obstacles = 3;
	ps.absObstacles[0].x=35; 
	ps.absObstacles[0].y=-75;
	ps.absObstacles[1].x=15;
	ps.absObstacles[1].y=67;
	ps.absObstacles[2].x=-40;
	ps.absObstacles[2].y=-80;

	ps.ball.x =92;
	ps.ball.y =73;

	ps.goal.x=-39;	//get from click
	ps.goal.y=-90;	//get from click

	#ifdef TESTPATH
	ps.absObstacles[0].x = test_x;
	ps.absObstacles[0].y = test_y;
	ps.ball.x = ball_x;
	ps.ball.y = ball_y;	
	#endif

	// for(int i=0;i<ps.n_obstacles;i++)
	//  {
	// // 	ps.absObstacles[i].x=rand() % 200 - 100;
	// // 	ps.absObstacles[i].y=rand() % 200 - 100;
	// 	std::cout<<"obstacle "<<i<<" x value "<<ps.absObstacles[i].x<<std::endl;
	// 	std::cout<<"obstacle "<<i<<" y value "<<ps.absObstacles[i].y<<std::endl;
	//  }
	// ps.ball.x=rand() % 200 - 100;
	// ps.ball.y=rand() % 200 - 100;	
	// ps.goal.x = rand() % 200 - 100;
	// ps.goal.y = rand() % 200 - 100;
	// std::cout<<"ball "<<ps.ball.x<<" "<<ps.ball.y<<std::endl;	
	// std::cout<<"goal "<<ps.goal.x<<" "<<ps.goal.y<<std::endl;	

	int r=200;
	int theta=100;
	
	Path p;
	// for(int r=50;r>0;r--)
	// 	{		
	// 		ps.ball.x=r*cos(deg2rad(theta));
	// 		ps.ball.y=r*sin(deg2rad(theta));
	// 		std:cout<<"\n\nr...."<<r<<"\n\n\n";
	// 		p.path_return(ps); // the main function that is called which returns the final path.
	// 		cvWaitKey();
	// 	}
	p.path_return(ps);
	p.updatePathPacket();//true if file writing to be on...else off.
}
int main()
{
	// #ifdef TESTPATH
	while(1)
	{
		// IplImage *img = cvCreateImage(cvSize(500,500), 8 ,1);
		// cvNamedWindow("field", CV_WINDOW_AUTOSIZE);
		setMouseCallback("Field", callBack, NULL);
		// cvShowImage("field", img);
		call_main();
		cvWaitKey(10);
		// cvReleaseImage(&img);
		// #endif
	}
	return 0;
}

