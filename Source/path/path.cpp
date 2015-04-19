
#include "path.hpp"
#define car2pol(x,y) sqrt((x)*(x)+(y)*(y))
#define dist(x1,y1,x2,y2) sqrt(pow(((x1)-(x2)),2)+pow(((y1)-(y2)),2))
#define PI 3.14159265359
// #define FRAMEPAINTING
//#define OLDENCIRCLING
//#define NEWENCIRCLING
//#define NEWNEWENCIRCLING
#define OLDDELETION
// #define NEWDELETION


using namespace std;
using namespace graph_implementation;
void Path::calculate_points(struct Point main_point,struct Point obstacle_centre, Point &p1, Point &p2) //Returns you the four points on the obstacle for drawing the tangent.
{
	double m1,m2;
	m1=(((main_point.x-obstacle_centre.x)*(main_point.y-obstacle_centre.y))+(obstacle_centre.obstacle_radius*sqrt((pow((main_point.x-obstacle_centre.x),2))+(pow((main_point.y-obstacle_centre.y),2))-(pow((double)obstacle_centre.obstacle_radius,2)))))/(pow((main_point.x-obstacle_centre.x),2)-pow((double)obstacle_centre.obstacle_radius,2));
	if(((main_point.y-obstacle_centre.y)-(m1*(main_point.x-obstacle_centre.x)))<0)
	{
		p1.x=(m1*obstacle_centre.obstacle_radius)/(sqrt(1+pow(m1,2)))+obstacle_centre.x;
		p1.y=-(obstacle_centre.obstacle_radius)/(sqrt(1+pow(m1,2)))+obstacle_centre.y;
	}
	else
	{
		p1.x=-(m1*obstacle_centre.obstacle_radius)/(sqrt(1+pow(m1,2)))+obstacle_centre.x;
		p1.y=(obstacle_centre.obstacle_radius)/(sqrt(1+pow(m1,2)))+obstacle_centre.y;
	}
	m2=(((main_point.x-obstacle_centre.x)*(main_point.y-obstacle_centre.y))-(obstacle_centre.obstacle_radius*sqrt((pow((main_point.x-obstacle_centre.x),2))+(pow((main_point.y-obstacle_centre.y),2))-(pow((double)obstacle_centre.obstacle_radius,2)))))/(pow((main_point.x-obstacle_centre.x),2)-pow((double)obstacle_centre.obstacle_radius,2));
	if(((main_point.y-obstacle_centre.y)-(m2*(main_point.x-obstacle_centre.x)))<0)
	{
		p2.x=(m2*obstacle_centre.obstacle_radius)/(sqrt(1+pow(m2,2)))+obstacle_centre.x;
		p2.y=-(obstacle_centre.obstacle_radius)/(sqrt(1+pow(m2,2)))+obstacle_centre.y;
	}
	else
	{
		p2.x=-(m2*obstacle_centre.obstacle_radius)/(sqrt(1+pow(m2,2)))+obstacle_centre.x;
		p2.y=(obstacle_centre.obstacle_radius)/(sqrt(1+pow(m2,2)))+obstacle_centre.y;
	}
}

bool Path::on_which_side(double a, double b,struct Point n1, struct Point n2,struct Point obstacle) //returns true if on the other side of the origin....i.e. the side favourable to us
{
	double m,c1,c2;
	m=b/a;
	c1=n1.y-(m*n1.x);
	c2=n2.y-(m*n2.x);
	if(((obstacle.y-(m*obstacle.x)-c1)*(obstacle.y-(m*obstacle.x)-c2))<0)
		return true;
	else
		return false;
}

bool Path::side_check(struct Point n1,struct Point n2, struct Point n1a, struct Point n2a)//returns true if and only if n1a and n2a are on the same side of the line joining
{
	double m,c;
	m=(n2.y-n1.y)/(n2.x-n1.x);
	c=n1.y-(m*n1.x);
	if(((n1a.y-(m*n1a.x)-c)*(n2a.y-(m*n2a.x)-c))>0)
		return true;
	else
		return false;
}

bool Path::side_check_orientation(Point a,Point b) // a is the parent point , b is the child point
{
	//cout<<" the coordinates for the parent points are "<<a.x<<" "<<a.y<<endl;
	//cout<<" the coordinates for the child points are "<<b.x<<" "<<b.y<<endl;
	double m = (a.y-ball.y)/(a.x-ball.x);
	//cout<<"m "<<m<<endl;
	double c = ball.y-(m*ball.x);
	//cout<<"c "<<c<<endl;
	//cout<<"the coordinates of the ball are "<<ball.x<<" "<<ball.y<<endl;
	//cout<<"the equation of the line joining ball and the parent point is y="<<m<<"x+"<<c<<endl;
	if(((goal.y-(m*goal.x)-c)*(b.y-(m*b.x)-c))<0) // other side of the line...point to be kept

		return true;
	else
		return false;
}

bool Path::new_point_orientation(size_t point_index)
{
	double m=(goal.y-ball.y)/(goal.x-ball.x);
	double m_prime=-1/m;
	double c=ball.y-m_prime*ball.x;
	if((goal.y-m_prime*goal.x-c)*(tree[point_index].y-m_prime*tree[point_index].x-c)<0) //i.e the point is same side as the goal wrt the line per to joing the goal and ball
		return true;
	else
		return false;
}


int Path::closest_intersecting_obstacle(Point n1,Point n2) //Returns the closest intersecting obstacles in the interested region.
{
	double a,b,c,slope,dist,min_dist_from_n1=DBL_MAX,dist_from_n1;//k is initially -1,if no obstacle is found then k remains -1
	int k = -1;
	slope=(n2.y-n1.y)/(n2.x-n1.x);
	b=1;
	a=-slope;
	c=(slope*n2.x)-n2.y;
	for(int i=0;i<NO_OF_OBSTACLES+2;i++)
	{
		if(on_which_side(a,b,n1,n2,obstacle[i]))//i.e only the obstacles which are in between the two lines will be checked
		{
			dist=fabs((a*obstacle[i].x)+(b*obstacle[i].y)+(c))/sqrt(pow(a,2)+pow(b,2));
			if(dist<obstacle[i].obstacle_radius-0.05)
			{
				dist_from_n1=sqrt(pow((n1.x - obstacle[i].x),2)+pow((n1.y - obstacle[i].y),2));
				if(dist_from_n1<min_dist_from_n1)
				{
					min_dist_from_n1 = dist_from_n1;
					k=i;
				}
			}
		}
	}
	return k;
}

Intersecting_Cases Path::main_function(Point n1, Point n2)
{
	temp_global_k=0;
	same_side_flag=true;
	int k=closest_intersecting_obstacle(n1,n2);
	if(k==-1)
	{
		//////////cout<<"\t\tNo Intersection\n";
		return no_intersection;
	}

	if((k == NO_OF_OBSTACLES || k == NO_OF_OBSTACLES +1) && n2.x==ball.x && n2.y==ball.y)
	{
		calculate_points(n1,obstacle[k],n1a,n1b);
		n1a.obstacle_id=k;
		n1b.obstacle_id=k;
		//////////cout<<"\t\tFinal Orientation Intersection\n";
		return final_orientation_intersection;
	}

	if(n1.obstacle_id==k || n2.obstacle_id==k)
	{
		Point midpoint, p1, p2;
		p1.x=0;
		p2.x=0;
		p1.y=0;
		p2.y=0;
		midpoint.x=(obstacle[n1.obstacle_id].x + obstacle[n2.obstacle_id].x)/2;
		midpoint.y=(obstacle[n1.obstacle_id].y + obstacle[n2.obstacle_id].y)/2;
		if((((pow((midpoint.x-obstacle[n1.obstacle_id].x),2))+(pow((midpoint.y-obstacle[n1.obstacle_id].y),2))-pow(obstacle[n1.obstacle_id].obstacle_radius,2))>0) && (((pow((midpoint.x-obstacle[n2.obstacle_id].x),2))+(pow((midpoint.y-obstacle[n2.obstacle_id].y),2))-pow(obstacle[n2.obstacle_id].obstacle_radius,2))>0))
		{
			calculate_points(midpoint,obstacle[n1.obstacle_id],p1,p2);
			double dist1,dist2,min;
			int z=1;
			dist1=sqrt(pow(p1.x-n1.x,2)+pow(p1.y-n1.y,2));
			min=dist1;
			dist2=sqrt(pow(p2.x-n1.x,2)+pow(p2.y-n1.y,2));
			if(dist2<min)
			{
				min=dist2;
				z=2;
			}
			if(z==1)
				n1a=p1;
			else
				n1a=p2;
			calculate_points(midpoint,obstacle[n2.obstacle_id],p1,p2);
			z=1;
			dist1=sqrt(pow(p1.x-n2.x,2)+pow(p1.y-n2.y,2));
			min=dist1;
			dist2=sqrt(pow(p2.x-n2.x,2)+pow(p2.y-n2.y,2));
			if(dist2<min)
			{
				min=dist2;
				z=2;
			}
			if(z==1)
				n2a=p1;
			else
				n2a=p2;
			temp_global_k = k;
		}
		else
		{
			midpoint_flag=true;
		}
		//////////cout<<"\t\tSpecial Intersection\n";
		return special_intersection;
	}
	temp_global_k = k;
	calculate_points(n1,obstacle[k], n1a, n1b);
	calculate_points(n2,obstacle[k], n2a, n2b);
	n1a.obstacle_id=k;
	n1b.obstacle_id=k;
	n2a.obstacle_id=k;
	n2b.obstacle_id=k;
	same_side_flag=side_check(n1,n2,n1a,n2a); //same_side_flag is true if and only if they are on the same side
	//////////cout<<"\t\tNormal Intersection\n";
	return normal_intersection;
}

double Path::cost_calculation(std::size_t source,std::size_t target, Graph <Point> &tree) //Returns cost of the path b/w the two points given. Note that the if the points are on circle then the cost is calculated accordingly.
{
	double cost;
	Point source_point=tree[source];
	Point target_point=tree[target];
	if(tree.is_onCircle(source,target)==false) //
	{
		if(tree.path_crash)
			return -1.0;
		cost=sqrt(pow((source_point.y - target_point.y),2)+pow((source_point.x - target_point.x),2));
	}
	else
	{
		if(tree.path_crash)
			return -1.0;
		assert(source_point.obstacle_id==target_point.obstacle_id);
		Point obstacle_point=obstacle[source_point.obstacle_id];
		double m1=(source_point.y - obstacle_point.y)/(source_point.x - obstacle_point.x);
		double m2=(target_point.y - obstacle_point.y)/(target_point.x - obstacle_point.x);
		double m=fabs((m1-m2)/(1+(m1*m2)));
		double theta=atan(m);
		cost=obstacle_point.obstacle_radius*theta;
	}
	return cost;
}

bool Path::checkLines(AbsCoords lGoal, AbsCoords rGoal, AbsCoords ball)
{
	AbsCoords midGoal;
	midGoal.x = (lGoal.x + rGoal.x)/2;
	midGoal.y = (lGoal.y + rGoal.y)/2;

	double l1 = (midGoal.y-ball.y)-(((lGoal.y-ball.y)/(lGoal.x - ball.x))*(midGoal.x-ball.x));
	double l2 = (midGoal.y-ball.y)-(((rGoal.y-ball.y)/(rGoal.x - ball.x))*(midGoal.x-ball.x));
	// printf("%lf\n", l1);
	// printf("%lf\n", l2);
	// printf("%lf\n", rGoal.y);
	// printf("%lf\n", rGoal.x);	
	double selfl1 = (-ball.y)-((lGoal.y-ball.y)/(lGoal.x - ball.x))*(-ball.x);
	double selfl2 = (-ball.y)-((rGoal.y-ball.y)/(rGoal.x - ball.x))*(-ball.x);
	// printf("%lf\n", selfl1);
	// printf("%lf\n", selfl2);

	if(l1*l2*selfl1*selfl2>0)
		return true;
	else
		return false;
}

void Path::orientSelf(PathStructure ps)
{
	
		bool Rotate;
		bool Back_Walk;
		bool Ballfollow;
if(checkLines(ps.gpleft, ps.gpright, ps.ball))
	{
		//send atan2(ball.y, ball.x);
		// printf("checklines\n");
		// bool Rotate;
		// bool Back_Walk;
		// bool Ballfollow;
		pthread_mutex_lock(&mutex_pathpacket);
		pathpackvar.theta = atan2(ps.ball.y, ps.ball.x);
		Rotate = 1;
		if(pathpackvar.ROTATE!= Rotate)
			pathpackvar.UPDATE_FLAG = 1;
		pathpackvar.ROTATE = 1;
		pathpackvar.BACK_WALK = 0;
		pathpackvar.BALLFOLLOW = 0;
		if(ps.ball.y>0)
			pathpackvar.ROTATE_RIGHT = 1;
		else
			pathpackvar.ROTATE_RIGHT = 0;		
			
		if(abs(pathpackvar.theta)<15*PI/180)
		{
			pathpackvar.ROTATE = 0;
			pathpackvar.distance = sqrt(pow(ps.ball.y, 2)+pow(ps.ball.x, 2))-5;
			pathpackvar.no_of_points = 1;
			Ballfollow = 1;
			if(pathpackvar.BALLFOLLOW!=Ballfollow)
				pathpackvar.UPDATE_FLAG = 1;
			pathpackvar.BALLFOLLOW = 1;
		}	
		// printf("BACK WALK %d\n", pathpackvar.BACK_WALK);
		// printf("IGNORE ARC %d\n", pathpackvar.IGNORE_ARC);
		// printf("UPDATE FLAG %d\n", pathpackvar.UPDATE_FLAG);
		// printf("ROTATE %d\n", pathpackvar.ROTATE);
		// printf("BALLFOLLOW %d\n",pathpackvar.BALLFOLLOW );
		// printf("check\n");

		pthread_mutex_unlock(&mutex_pathpacket);
	}
	else
	{
		// printf("no checklines\n");
		AbsCoords pointOnLine;
		pointOnLine.y = 0;
		pointOnLine.x = ((-1.0)*((goal.x-ball.x)/(goal.y-ball.y))*ball.y)+ball.x;

		AbsCoords pointOnCircle;
		pointOnCircle.y = 0;
		pointOnCircle.x = (-1.0)*(sqrt(pow(INITIAL_ORIENTATION_RADIUS+10,2)-pow(ps.ball.y,2))) + ps.ball.x;
		pthread_mutex_lock(&mutex_pathpacket);
		Rotate = 0;
		if(pathpackvar.ROTATE != Rotate)
			pathpackvar.UPDATE_FLAG = 1;
		pathpackvar.ROTATE = 0;
		Ballfollow = 0;
		if(Ballfollow!= pathpackvar.BALLFOLLOW)
			pathpackvar.UPDATE_FLAG = 1;
		pathpackvar.BALLFOLLOW = 0;
		if(pointOnCircle.x<pointOnLine.x)
		{
			//send pointonline
			Back_Walk = 1;
			if(Back_Walk!=pathpackvar.BACK_WALK)
				pathpackvar.UPDATE_FLAG = 1;
			pathpackvar.BACK_WALK = 1;
			pathpackvar.finalpath[0].x = pointOnLine.x;
			pathpackvar.finalpath[0].y = 0;
			pathpackvar.no_of_points = 1;
		}
		else
		{
			//send point oncircle
			Back_Walk = 1;
			if(Back_Walk!=pathpackvar.BACK_WALK)
				pathpackvar.UPDATE_FLAG = 1;
			pathpackvar.BACK_WALK = 1;
			pathpackvar.finalpath[0].x = pointOnCircle.x;
			pathpackvar.finalpath[0].y = 0;
			pathpackvar.no_of_points = 1;			
		}
			// printf("BACK WALK %d\n", pathpackvar.BACK_WALK);
			// printf("IGNORE ARC %d\n", pathpackvar.IGNORE_ARC);
			// printf("UPDATE FLAG %d\n", pathpackvar.UPDATE_FLAG);
			// printf("ROTATE %d\n", pathpackvar.ROTATE);
			// printf("BALLFOLLOW %d\n",pathpackvar.BALLFOLLOW );



		pthread_mutex_unlock(&mutex_pathpacket);
	}
}

PathReturns Path::path_return(PathStructure ps)
{
	// printf("PR line 226\n");
	//cout<<".....................................................PATH CALLED.......................................................";
	srand(time(NULL));
	IplImage* image= cvCreateImage(cvSize(SIZEX*5, SIZEY*5), 8, 3); //displaying the results. beacuse pranet had a problem.
	cvZero(image);
	int flag=1; //Flag used for iteration in the graph...i.e. assigned the value 0 once iteration through the entire graph is done to break out of loop.
	start.x=0;
	start.y=0;
	start.obstacle_id=-1;
	start.parent_id=0;
	cvCircle(image, cvPoint(start.x+250, start.y+250), 2, cvScalar(0,255,0)); //Initialising the start point and painting it.
	NO_OF_OBSTACLES = ps.n_obstacles;
	ball.x=ps.ball.x;
	ball.y=ps.ball.y;
	ball.obstacle_id=-1;
	ball.parent_id=0;
	////////cout<<"Ball Position:\n"<<ball.x<<" "<<ball.y<<endl;
	cvCircle(image, cvPoint(ball.x+250, ball.y+250), 2, cvScalar(255,0,255)); //Initialising the ball point and painting it.
	goal.x=ps.goal.x;
	goal.y=ps.goal.y;
	goal.obstacle_id=-1;
	////////cout<<"Goal Position\n"<<goal.x<<" "<<goal.y<<endl;
	cvCircle(image, cvPoint(goal.x+250, goal.y+250), 10, cvScalar(255,0,255)); //Initialising the goal point and painting it.
	// cvCircle(image, cvPoint(goal.x+250, goal.y+250+10), 5, cvScalar(255,0,255)); //Initialising the goal point and painting it.
	// cvCircle(image, cvPoint(goal.x+250, goal.y+250-10), 5, cvScalar(255,0,255)); //Initialising the goal point and painting it.
	
	tree.cleartree(); //Clearing all previous data from the graphs....i.e. initialises the graph for a fresh start.
	for(int i = 0; i < NO_OF_OBSTACLES; i++)
	{
		obstacle[i].x = ps.absObstacles[i].x;
		obstacle[i].y = ps.absObstacles[i].y;
		obstacle[i].obstacle_radius=OBSTACLE_RADIUS;
		obstacle[i].type= CIRCLE;
		////////cout<<"Obstacle Position(s)\n"<<obstacle[i].x<<" "<<obstacle[i].y<<endl;
		cvCircle(image, cvPoint(obstacle[i].x+250,obstacle[i].y+250), obstacle[i].obstacle_radius, cvScalar(255,0,0));
	} //Initialising the obstacles and painting them.
	cvCircle(image, cvPoint(ball.x+250, ball.y+250), ENCIRCLE_THRESHOLD , cvScalar(0,130,70));
	//Making the two orientation markers near the ball  ---->
	double m=(goal.y-ball.y)/(goal.x-ball.x);
	double m_prime=-1/m;
	double cos_prime=1/(sqrt(pow(m_prime,2)+1));
	double sin_prime=m_prime/(sqrt(pow(m_prime,2)+1));
#ifdef NEWNEWENCIRCLING
	if(car2pol(ball.x,ball.y) <= 2*ORIENTATION_RADIUS)
	{
		ORIENTATION_RADIUS=car2pol(ball.x,ball.y)/2-0.5;

	}

#endif
	obstacle[NO_OF_OBSTACLES].obstacle_radius= ORIENTATION_RADIUS;
	obstacle[NO_OF_OBSTACLES].x=ball.x - ( ORIENTATION_RADIUS*cos_prime);
	obstacle[NO_OF_OBSTACLES].y=ball.y - ( ORIENTATION_RADIUS*sin_prime);
	obstacle[NO_OF_OBSTACLES].obstacle_id=-2;
	obstacle[NO_OF_OBSTACLES].type= CIRCLE;
	cvCircle(image, cvPoint(obstacle[NO_OF_OBSTACLES].x+250, obstacle[NO_OF_OBSTACLES].y+250),  ORIENTATION_RADIUS, cvScalar(255,0,0));
	obstacle[NO_OF_OBSTACLES+1].obstacle_radius= ORIENTATION_RADIUS;
	obstacle[NO_OF_OBSTACLES+1].x=ball.x + ( ORIENTATION_RADIUS*cos_prime);
	obstacle[NO_OF_OBSTACLES+1].y=ball.y + ( ORIENTATION_RADIUS*sin_prime);
	obstacle[NO_OF_OBSTACLES+1].obstacle_id=-2;
	obstacle[NO_OF_OBSTACLES+1].type= CIRCLE;
	cvCircle(image, cvPoint(obstacle[NO_OF_OBSTACLES+1].x+250, obstacle[NO_OF_OBSTACLES+1].y+250),  ORIENTATION_RADIUS, cvScalar(255,0,0));
	for(int i=0;i< NO_OF_OBSTACLES;i++)
	{
		for(int j=NO_OF_OBSTACLES;j<NO_OF_OBSTACLES+2;j++)
		{
			double d=sqrt(pow((obstacle[i].x - obstacle[j].x),2)+pow((obstacle[i].y - obstacle[j].y),2));
			if(d < (obstacle[i].obstacle_radius+obstacle[j].obstacle_radius)-0.005)
			{										//now the obstacles are intersecting
				obstacle[j].x=1000;
				obstacle[j].y=1000;
			}

		}

	}
//implementing circle design
	for(int i=0;i< NO_OF_OBSTACLES;i++)
	{
		for(int j=i+1;j<NO_OF_OBSTACLES;j++)
		{
			double d=sqrt(pow((obstacle[i].x - obstacle[j].x),2)+pow((obstacle[i].y - obstacle[j].y),2));

			if(d < (obstacle[i].obstacle_radius+obstacle[j].obstacle_radius)-0.005)
			{//now the obstacles are intersecting
				obstacle[i].x=obstacle[i].x/2+obstacle[j].x/2;
				obstacle[i].y=obstacle[i].y/2+obstacle[j].y/2;
				obstacle[i].obstacle_radius= (obstacle[i].obstacle_radius+obstacle[j].obstacle_radius+d)/2;
				obstacle[j].type=DNE;
			}
		}
	}
	for(int i=0;i<NO_OF_OBSTACLES+2;i++)
	{
		if(obstacle[i].type==DNE)
		{
			for (int j = i; j < NO_OF_OBSTACLES-1+2; ++j)
			{
				obstacle[j]=obstacle[j+1];
			}
			NO_OF_OBSTACLES--;
		}
	}

	Near_Obstacle = 0;
	for(int i=0;i<NO_OF_OBSTACLES;i++)
	{
		if(dist(obstacle[i].x,obstacle[i].y,0,0)<obstacle[i].obstacle_radius + 10)
		{
			// if(dist(obstacle[i].x,obstacle[i].y,0,0)<obstacle[i].obstacle_radius + 5)
				Near_Obstacle = 1;
			// obstacle[i].x=1000;
			// obstacle[i].y=1000;
		}
	}
	for(int i=0;i<NO_OF_OBSTACLES+2;i++)
	{
		if(dist(obstacle[i].x,obstacle[i].y,0,0)<obstacle[i].obstacle_radius)
		{
			if(dist(obstacle[i].x,obstacle[i].y,0,0)<2*obstacle[i].obstacle_radius)
				Near_Obstacle = 1;
			obstacle[i].x=1000;
			obstacle[i].y=1000;
		}
	}
//****************Kick if ball is within 5 cm of bot***************************
	if(car2pol(ball.x,ball.y) <= KICKDIST)
	{
		//cout<<"\npath returning \n\nDOKICK\n\n " ;

		return DOKICK;
	}

#ifdef OLDENCIRCLING
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------
	//Encircling, Walking, Kicking decision ----->
	CvFont font;
    cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, 0.5, 0.5, 0, 1, 8);
    char A[100];
    char B[100];					
    sprintf(A,"Ball Position:  %lf, %lf",ball.x,ball.y);
    sprintf(B,"Goal Position:  %lf, %lf",goal.x,goal.y);
    cvPutText(image,A,cvPoint(10,25),&font,cvScalar(255,255,255));
    cvPutText(image,B,cvPoint(10,45),&font,cvScalar(255,255,255));
	cvShowImage("Field", image);

	if(car2pol(ball.x,ball.y) <= KICKDIST)
	{
		//cout<<"\npath returning \n\nDOKICK\n\n " ;

		return DOKICK;
	}
	//ENCIRCLE_THRES is always > KICKDIST
	else if(car2pol(ball.x,ball.y) <= ENCIRCLE_THRESHOLD)
	{
		//cout<<"\n\nless than encircel threshold......\n\n";
		//ballgoalangle=>angle made by line joining acyut and ball with line joining ball and goal
		tolerance_angle=rad2deg(atan(BADANGLEDIST/sqrt(pow(ball.x,2)+pow(ball.y,2)))); //dyanmically generating tolerance angle for orientation purposes(i.e facing the ball before orienting)
		double ballangle=rad2deg(atan2(ball.y,ball.x));
		if(abs(ballangle)>tolerance_angle)
		{
			if(ballangle>0)
			{
				next.theta=(abs(ballangle)-tolerance_angle)*-1; //-1 is due to the ip convention and walk convention
				next.r=0;
				//cout<<"\npath returning \n\nDOWALK\n\n  theta "<<next.theta<< " distance "<<next.r<<endl;
				return DOWALK;
			}
			else
			{
				next.theta=(-(abs(ballangle)-tolerance_angle))*-1;
				next.r=0;
				//cout<<"\npath returning \n\nDOWALK\n\n  theta "<<next.theta<< " distance "<<next.r<<endl;
				return DOWALK;
			}
		} // acyut is now facing the ball .... ready to encircle
		double a=car2pol(ball.x,ball.y);
		double b=car2pol(ball.x-goal.x,ball.y-goal.y);
		double c=car2pol(goal.x,goal.y);
		double ballgoalangle=180- rad2deg(acos((a*a+b*b-c*c)/(2*a*b))); //using cosine rule to get angle required to encircle ( for understanding refer encircleangleidea.jpg in source)
		NextEncircleAngle=ballgoalangle*-1; // the -1 is due to ip convention and walk convention.
		//cout<<"\npath returning \n\nDOENCIRCLE\n\n  theta "<<NextEncircleAngle<<endl;
		return DOENCIRCLE; //encircle
	}
	else if(car2pol(ball.x,ball.y) <= 2*ORIENTATION_RADIUS) //if i am inside the orientation circle while on the path then follow straight till u get to the encircle radius
	{
		//cvWaitKey();
		//cout<<"\n\ninside 2*orientaion radius\n\n";
		double ballangle=rad2deg(atan2(ball.y,ball.x));
		next.r=car2pol(ball.x,ball.y);
		next.theta=ballangle;
		//cout<<"\npath returning \n\nDOWALK\n\n  theta "<<next.theta<< " distance "<<next.r<<endl;
		return DOWALK;
	}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#endif
	tree.add_vertex(start);
	if(tree.path_crash)
		return NOPATH;
	tree.add_vertex(ball);
	if(tree.path_crash)
		return NOPATH;
	tree.add_edge(0,1,cost_calculation(0,1,tree));
	if(tree.path_crash)
		return NOPATH;
	cvLine(image,cvPoint(250,250),cvPoint(ball.x+250,ball.y+250),cvScalar(255,255,0)); // creates the nodes for start and ball point and paints a line.

	while(flag) // iterates over the entire tree.
	{
		flag=0;
		for(size_t n1_index=0;n1_index<tree.size(); n1_index++)
		{
			for(size_t n2_index = 0; n2_index < tree.size(); n2_index++) // looping through the entire tree.
			{
				if(tree.is_edge(n1_index, n2_index))
				{
					if(tree.path_crash)
						return NOPATH;
					//cout<<"\nanalysing edges "<<n1_index<<" "<<n2_index<<endl;
					Point n1 = tree[n1_index];
					Point n2 = tree[n2_index];

					#ifdef FRAMEPAINTING
					//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
					//code for frame by frame painting.						//n1_indiex-->ni_index.....n2_index-->nj_index
						for(size_t ni_index=0;ni_index<tree.size(); ni_index++)
						{
							for(size_t nj_index = 0; nj_index < tree.size(); nj_index++)
							{
								if(tree.is_edge(ni_index, nj_index))
								{
									if(tree.path_crash)
										return NOPATH;
									Point n1 = tree[ni_index];
									Point n2 = tree[nj_index];
									cvLine(image,cvPoint(n1.x+250,n1.y+250),cvPoint(n2.x+250,n2.y+250),cvScalar(255,255,0));
								}
							}
						}
						for(int j=0;j<NO_OF_OBSTACLES+2;j++)
						{
							cvCircle(image, cvPoint(obstacle[j].x+250, obstacle[j].y+250), 2, cvScalar(255,0,0));
							cvCircle(image, cvPoint(obstacle[j].x+250, obstacle[j].y+250), obstacle[j].obstacle_radius , cvScalar(255,0,0));
						}
						cvCircle(image, cvPoint(n1.x+250, n1.y+250), 2, cvScalar(255,255,255), 2);
						cvCircle(image, cvPoint(n2.x+250, n2.y+250), 2, cvScalar(0,0,255), 2);
						cvCircle(image, cvPoint(start.x+250, start.y+250), 2, cvScalar(0,255,0));
						cvCircle(image, cvPoint(ball.x+250, ball.y+250), 2, cvScalar(255,0,255));
						cvCircle(image, cvPoint(goal.x+250, goal.y+250), 10, cvScalar(255,0,255));
						CvFont font;
					    cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, 0.5, 0.5, 0, 1, 8);
					    char A[100];
					    char B[100];					
					    sprintf(A,"Ball Position:  %lf, %lf",ball.x,ball.y);
					    sprintf(B,"Goal Position:  %lf, %lf",goal.x,goal.y);
					    cvPutText(image,A,cvPoint(10,25),&font,cvScalar(255,255,255));
					    cvPutText(image,B,cvPoint(10,45),&font,cvScalar(255,255,255));
					    cvNamedWindow("Field");
    					// cvMoveWindow("Field",950,400);
						cvShowImage("Field", image);
						cvZero(image);
					 	cvWaitKey(0);
					//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
					#endif
					if(tree.is_onCircle(n1_index, n2_index)==false)
					{
						if(tree.path_crash)
							return NOPATH;
						switch(main_function(n1,n2))
						{

							case normal_intersection:
							{
								flag = 1;
								tree.remove_edge(n1_index, n2_index); //removing the edges for which there is an obstacle in b/w.
								if(tree.path_crash)
									return NOPATH;
								n1a.parent_id=n1_index;							//
								size_t n1a_index = tree.add_vertex(n1a);
								if(tree.path_crash)
									return NOPATH;			//
								n1b.parent_id=n1_index;
								size_t n1b_index = tree.add_vertex(n1b);
								if(tree.path_crash)
									return NOPATH;			//
								n2a.parent_id=n1_index;									//adding the four new nodes
								size_t n2a_index = tree.add_vertex(n2a);
								if(tree.path_crash)
									return NOPATH;			//
								n2b.parent_id=n1_index;								//
								size_t n2b_index = tree.add_vertex(n2b);
								if(tree.path_crash)
									return NOPATH;			//
								tree.add_edge(n1_index, n1a_index,cost_calculation(n1_index,n1a_index,tree));
								if(tree.path_crash)
									return NOPATH;
								if(closest_intersecting_obstacle(tree[n1.parent_id],tree[n1a_index])==-1)//check the obstacle intersection from n1's parent to the point n1a...n1b.....
								{

									n1a.parent_id=n1.parent_id;
									tree.update_vertex(n1a,n1a_index);
									tree.add_edge(n1.parent_id,n1a_index,cost_calculation(n1.parent_id,n1a_index,tree));//add edge b/w the parent point and the point n1a,nb...
									if(tree.path_crash)
										return NOPATH;
								}
								tree.add_edge(n1_index, n1b_index,cost_calculation(n1_index,n1b_index,tree));
								if(tree.path_crash)
									return NOPATH;
								if(closest_intersecting_obstacle(tree[n1.parent_id],tree[n1b_index])==-1)//check the obstacle intersection from n1's parent to the point n1a...n1b.....
								{
									n1b.parent_id=n1.parent_id;
									tree.update_vertex(n1b,n1b_index);
									tree.add_edge(n1.parent_id,n1b_index,cost_calculation(n1.parent_id,n1b_index,tree));//add edge b/w the parent point and the point n1a,nb...
									if(tree.path_crash)
										return NOPATH;
								}
								tree.add_edge(n2a_index, n2_index,cost_calculation(n2a_index,n2_index,tree));
								if(closest_intersecting_obstacle(tree[n2a.parent_id],tree[n2_index])==-1)//check the obstacle intersection from n1's parent to the point n1a...n1b.....
								{
									n2.parent_id=n2a.parent_id;
									tree.update_vertex(n2,n2_index);
									tree.add_edge(n2a.parent_id,n2_index,cost_calculation(n2a.parent_id,n2_index,tree));//add edge b/w the parent point and the point n1a,nb...
									if(tree.path_crash)
										return NOPATH;
								}
								tree.add_edge(n2b_index, n2_index,cost_calculation(n2b_index,n2_index,tree));
								if(closest_intersecting_obstacle(tree[n2b.parent_id],tree[n2_index])==-1)//check the obstacle intersection from n1's parent to the point n1a...n1b.....
								{
									n2.parent_id=n2b.parent_id;
									tree.update_vertex(n2,n2_index);
									tree.add_edge(n2b.parent_id,n2_index,cost_calculation(n2b.parent_id,n2_index,tree));//add edge b/w the parent point and the point n1a,nb...
									if(tree.path_crash)
										return NOPATH;
								}
								if(same_side_flag)
								{
									tree.add_edge(n1a_index, n2a_index,cost_calculation(n1a_index,n2a_index,tree), true);
									tree.add_edge(n1b_index, n2b_index,cost_calculation(n1b_index,n2b_index,tree), true);
									if(tree.path_crash)
										return NOPATH;
								}
								else
								{
									tree.add_edge(n1a_index, n2b_index,cost_calculation(n1a_index,n2b_index,tree), true);
									tree.add_edge(n1b_index, n2a_index,cost_calculation(n1b_index,n2a_index,tree), true);
									if(tree.path_crash)
										return NOPATH;
								}
								break;
							}
							case special_intersection:
							{
								if(midpoint_flag==false)
								{
									flag=1;
									//the special function returns n1a and n1b
									tree.remove_edge(n1_index,n2_index);
									n1a.parent_id=n1_index;
									n2a.parent_id=n1_index;
									size_t n1a_index = tree.add_vertex(n1a);
									if(tree.path_crash)
										return NOPATH;
									size_t n2a_index = tree.add_vertex(n2a);
									if(tree.path_crash)
										return NOPATH;
									tree.add_edge(n1_index, n1a_index,cost_calculation(n1_index,n1a_index,tree), true);
									for(std::size_t i=0; i<tree.size();i++)
									{
										if(tree.is_edge(n2_index,i))
										{
											if(tree.path_crash)
												return NOPATH;
											if(tree.is_onCircle(n2_index,i))
												tree.add_edge(n2a_index, i,cost_calculation(n2a_index,i,tree), true);
											else
												tree.add_edge(n2a_index, i,cost_calculation(n2a_index,i,tree));
											if(tree.path_crash)
												return NOPATH;
											tree.remove_edge(n2_index,i);
											if(tree.path_crash)
												return NOPATH;
										}
									}
									tree.add_edge(n1a_index, n2a_index,cost_calculation(n1a_index,n2a_index,tree));
									if(tree.path_crash)
										return NOPATH;

									// tree.add_edge(n2a_index, n2_index,cost_calculation(n2a_index,n2_index,tree),true);
									// tree.remove_edge(n1a_index,n2a_index);
									// tree.remove_edge(n2_index,n2a_index);
									// tree.add_edge(n1a_index, n2a_index,cost_calculation(n1a_index,n2a_index,tree), true);
									//note the circles on which n1a and n2a are lying.....
									break;
								}
								else
									break;

							}
							case no_intersection:
							{
								break;
							}
							case final_orientation_intersection:
							{

								flag=1;
								tree.remove_edge(n1_index,n2_index);
								size_t n1a_index = tree.add_vertex(n1a);
								if(tree.path_crash)
									return NOPATH;
								size_t n1b_index = tree.add_vertex(n1b);
								if(tree.path_crash)
									return NOPATH;
								tree.add_edge(n1_index, n1a_index,cost_calculation(n1_index,n1a_index,tree));
								if(tree.path_crash)
									return NOPATH;
								if(closest_intersecting_obstacle(tree[n1.parent_id],tree[n1a_index])==-1)//check the obstacle intersection from n1's parent to the point n1a...n1b.....
								{
									n1a.parent_id=n1.parent_id;
									tree.update_vertex(n1a,n1a_index);
									tree.add_edge(n1.parent_id,n1a_index,cost_calculation(n1.parent_id,n1a_index,tree));//add edge b/w the parent point and the point n1a,nb...
									if(tree.path_crash)
										return NOPATH;
								}
								tree.add_edge(n1_index, n1b_index,cost_calculation(n1_index,n1b_index,tree));
								if(tree.path_crash)
									return NOPATH;
								if(closest_intersecting_obstacle(tree[n1.parent_id],tree[n1b_index])==-1)//check the obstacle intersection from n1's parent to the point n1a...n1b.....
								{
									n1b.parent_id=n1.parent_id;
									tree.update_vertex(n1b,n1b_index);
									tree.add_edge(n1.parent_id,n1b_index,cost_calculation(n1.parent_id,n1b_index,tree));//add edge b/w the parent point and the point n1a,nb...
									if(tree.path_crash)
										return NOPATH;
								}
								tree.add_edge(n1a_index,n2_index,cost_calculation(n1a_index,n2_index,tree),true);
								tree.add_edge(n1b_index,n2_index,cost_calculation(n1b_index,n2_index,tree),true);
								if(tree.path_crash)
									return NOPATH;
							}
						}
						if(tree.path_crash)
							return NOPATH;
							if(flag)
							break;
					}
				}
			}
			if(flag)
				break;
		}
	}
	if(tree.path_crash)
		return NOPATH;
	removed_index to_be_removed[tree.size()];
	for(std::size_t i=1;i<tree.size();i++)
	{
		to_be_removed[i].connected_count=0;
		to_be_removed[i].remove_count=0;
	}


#ifdef OLDDELETION
	for(std::size_t i=2;i<tree.size();i++) //code for final orientation i.e deleting the node from whose side we do not orient.
	{
		if(tree.is_edge(i,1)) //i is the point connected to 1
		{
			if(tree.path_crash)
				return NOPATH;
			//cout<<"\n\n\n\n\n\n\n\n\n";
			//cout<<" node "<<i<<" is connected to 1\n";
			for(std::size_t j =0 ; j < tree.size() ; j++)
			{
				if(tree.is_edge(j,i)) //j is the point connected to i
				{
					if(tree.path_crash)
						return NOPATH;
					//cout<<"node "<<j<< " is further connected to this "<<i<<endl;
					//cout<<"increasing the connected count \n";
					to_be_removed[i].connected_count+=1;
					if(!side_check_orientation(tree[j],tree[i]))
					{
						//cout<<"side check orientation passed increasing the remove conunt\n";
						to_be_removed[i].remove_count+=1;
					}
					else
					{
						//cout<<"oops side check orientaion failed for the nodes "<<i<<" and "<<j<<endl;
					}
						
				}

			}
		}
	}
	//cout<<"\nto be removed remove count "<<to_be_removed[11].remove_count<<" "<<"to be removed connected count "<<to_be_removed[11].connected_count<<endl;
#endif
	// printf("PR line 612\n");
#ifdef NEWDELETION
	for(std::size_t i=2;i<tree.size();i++) //code for final orientation i.e deleting the node from whose side we do not orient.
	{
		if(tree.is_edge(i,1)) //i is the point connected to 1
		{
			if(tree.path_crash)
				return NOPATH;
			if(!new_point_orientation(i))
			{
				tree.remove_edge(i,1);
				if(tree.path_crash)
					return NOPATH;
			}
		}
	}

#endif

	for(std::size_t i=1;i<tree.size();i++)
	{
		if(to_be_removed[i].remove_count==to_be_removed[i].connected_count && to_be_removed[i].remove_count!=0)
		{
			tree.remove_edge(i,1);
			if(tree.path_crash)
				return NOPATH;
			//cout<<"\nremoving edge  "<<i<<endl;;
		}
	}
	for(size_t n1_index=0;n1_index<tree.size(); n1_index++)
	{
		for(size_t n2_index = 0; n2_index < tree.size(); n2_index++)
		{
			if(tree.is_edge(n1_index, n2_index))
			{
				if(tree.path_crash)
					return NOPATH;
				Point n1 = tree[n1_index];
				Point n2 = tree[n2_index];
				cvLine(image,cvPoint(n1.x+250,n1.y+250),cvPoint(n2.x+250,n2.y+250),cvScalar(255,255,0));
			}
		}
	}

	////////cout<<"\nCalling Dijkstras\n";
	tree.dijkstra();
	std::size_t a,b;
	a=1;
	////////cout<<"Before return path point 1";
	
	b=tree.returnPathPoint(1);
	while(b!=0)
	{
		// cvLine(image,cvPoint(tree[b].x+250,tree[b].y+250),cvPoint(tree[a].x+250,tree[a].y+250),cvScalar(0,0,255));
		a=b;
		b=tree.returnPathPoint(b);
	}

	if(	//(sqrt(pow(tree[1].x,2) + pow(tree[1].y-INITIAL_ORIENTATION_RADIUS, 2))<INITIAL_ORIENTATION_RADIUS || sqrt(pow(tree[1].x,2) + pow(tree[1].y+INITIAL_ORIENTATION_RADIUS, 2))<INITIAL_ORIENTATION_RADIUS) ||
		
		(sqrt(pow(tree[tree.returnPathPoint(1)].x,2) + pow(tree[tree.returnPathPoint(1)].y-INITIAL_ORIENTATION_RADIUS, 2))<INITIAL_ORIENTATION_RADIUS || sqrt(pow(tree[tree.returnPathPoint(1)].x,2) + pow(tree[tree.returnPathPoint(1)].y+INITIAL_ORIENTATION_RADIUS, 2))<INITIAL_ORIENTATION_RADIUS)) 
		// (sqrt(pow(tree[a].x,2) + pow(tree[a].y-INITIAL_ORIENTATION_RADIUS, 2))<INITIAL_ORIENTATION_RADIUS || sqrt(pow(tree[a].x,2) + pow(tree[a].y+INITIAL_ORIENTATION_RADIUS, 2))<INITIAL_ORIENTATION_RADIUS))
	{
		Near_Flag = 1;
	}
	else if(sqrt(pow(tree[tree.returnPathPoint(1)].x,2) + pow(tree[tree.returnPathPoint(1)].y-INITIAL_ORIENTATION_RADIUS, 2))<INITIAL_ORIENTATION_RADIUS || sqrt(pow(tree[tree.returnPathPoint(1)].x,2) + pow(tree[tree.returnPathPoint(1)].y+INITIAL_ORIENTATION_RADIUS, 2))> INITIAL_ORIENTATION_RADIUS + 10)
	{
		Near_Flag = 0;
	}

	if(sqrt(pow(tree[1].x, 2) + pow(tree[1].y, 2))<THRESHOLD && abs(ball.y)>30)
	{
		Near_Flag = 0;
		Back_Walk = 1;
	}
	else
		Back_Walk = 0;

	if(Back_Walk)
	{
		BackWalkX = ((-1.0)*((goal.x-ball.x)/(goal.y-ball.y))*ball.y)+ball.x;
		Near_Obstacle = 0;
	}
	if(sqrt(pow(tree[a].x, 2) + pow(tree[a].y, 2))<100)
	{
		if(abs(atan(tree[a].y/tree[a].x))*180/PI<10)
		{
			Ignore_Arc = 1;
		}
		else if(abs(atan(tree[a].y/tree[a].x))*180/PI>20)
			Ignore_Arc = 0;
	}
	else
	{
		if(abs(atan(tree[a].y/tree[a].x))*180/PI<10)
		{
			Ignore_Arc = 1;
		}
		else if(abs(atan(tree[a].y/tree[a].x))*180/PI>35)
			Ignore_Arc = 0;		
	}

	// cout<<"after updation"<<endl;
	//introducing initial obsacles between 0 and a
	obstacle[NO_OF_OBSTACLES + 2].obstacle_radius = INITIAL_ORIENTATION_RADIUS;
	obstacle[NO_OF_OBSTACLES + 2].x=start.x;
	obstacle[NO_OF_OBSTACLES + 2].y=start.y - ( INITIAL_ORIENTATION_RADIUS);
	obstacle[NO_OF_OBSTACLES + 2].obstacle_id=-3;
	obstacle[NO_OF_OBSTACLES + 2].type= CIRCLE;
	cvCircle(image, cvPoint(obstacle[NO_OF_OBSTACLES + 2].x+250, obstacle[NO_OF_OBSTACLES + 2].y+250),  INITIAL_ORIENTATION_RADIUS, cvScalar(255,0,0));

	obstacle[NO_OF_OBSTACLES + 3].obstacle_radius= INITIAL_ORIENTATION_RADIUS;
	obstacle[NO_OF_OBSTACLES + 3].x=start.x;
	obstacle[NO_OF_OBSTACLES + 3].y=start.y + ( INITIAL_ORIENTATION_RADIUS);
	obstacle[NO_OF_OBSTACLES + 3].obstacle_id=-3;
	obstacle[NO_OF_OBSTACLES + 3].type= CIRCLE;
	cvCircle(image, cvPoint(obstacle[NO_OF_OBSTACLES + 3].x+250, obstacle[NO_OF_OBSTACLES+3].y+250),  INITIAL_ORIENTATION_RADIUS, cvScalar(255,0,0));
	Point p1, p2, i1, i2;
	p1.x=0;
	p2.x=0;
	p1.y=0;
	p2.y=0;
	i1=start;
	i2=tree[a];
	size_t i2a_index;
	if(i2.y > 0)
	{
		start.obstacle_id = NO_OF_OBSTACLES + 3;
		calculate_points(i2, obstacle[NO_OF_OBSTACLES + 3], p1, p2);
	}
	else
	{
		start.obstacle_id = NO_OF_OBSTACLES + 2;		
		calculate_points(i2, obstacle[NO_OF_OBSTACLES + 2], p1, p2);
	}
	if(pow(p1.x,2)+pow(p1.y,2) < pow(p2.x,2)+pow(p2.y,2))
	{
		//implies p1 is near and you consider this and not p2
		tree.remove_edge(0, a);
		// cout<<"obstacle[tree[a].obstacle_id].obstacle_radius: "<<obstacle[tree[a].obstacle_id].obstacle_radius<<endl;
		if(tree.path_crash)
			return NOPATH;

		i2a_index = tree.add_vertex(p1);
		if(tree.path_crash)
			return NOPATH;
		tree.add_edge(0,i2a_index,cost_calculation(0,i2a_index,tree),true);
		// cout<<"i2a_index obstacle_id: "<<tree[i2a_index].obstacle_id<<endl;
		if(tree.path_crash)
			return NOPATH;
		tree.add_edge(i2a_index,a,cost_calculation(i2a_index,a,tree));
		if(tree.path_crash == true)
			return NOPATH;
	}
	else
	{
		//implies p2 is near and you consider this and not p1
		tree.remove_edge(0, a);
		if(tree.path_crash)
			return NOPATH;
		i2a_index = tree.add_vertex(p2);
		if(tree.path_crash)
			return NOPATH;
		tree.add_edge(0,i2a_index,cost_calculation(0,i2a_index,tree),true);
		if(tree.path_crash)
			return NOPATH;
		// cout<<"i2a_index obstacle_id: "<<tree[i2a_index].obstacle_id<<endl;
		tree.add_edge(i2a_index,a,cost_calculation(i2a_index,a,tree));
		if(tree.path_crash == true)
			return NOPATH;
	}
	// cout<<"before"<<endl;
	// obstacle[tree[i2a_index].obstacle_id].obstacle_radius = INITIAL_ORIENTATION_RADIUS;
	// cout<<"after"<<endl;
	#ifdef FRAMEPAINTING
	//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
	//painting the new frame
	for(size_t ni_index=0;ni_index<tree.size(); ni_index++)
	{
		for(size_t nj_index = 0; nj_index < tree.size(); nj_index++)
		{
			if(tree.is_edge(ni_index, nj_index))
			{
				if(tree.path_crash)
					return NOPATH;
				Point n1 = tree[ni_index];
				Point n2 = tree[nj_index];
				cvLine(image,cvPoint(n1.x+250,n1.y+250),cvPoint(n2.x+250,n2.y+250),cvScalar(255,255,0));
			}
		}
	}
	for(int j=0;j<NO_OF_OBSTACLES+4;j++)
	{
		cvCircle(image, cvPoint(obstacle[j].x+250, obstacle[j].y+250), 2, cvScalar(255,0,0));
		cvCircle(image, cvPoint(obstacle[j].x+250, obstacle[j].y+250), obstacle[j].obstacle_radius , cvScalar(255,0,0));
	}
	
	cvCircle(image, cvPoint(start.x+250, start.y+250), 2, cvScalar(0,255,0));
	cvCircle(image, cvPoint(ball.x+250, ball.y+250), 2, cvScalar(255,0,255));
	cvCircle(image, cvPoint(goal.x+250, goal.y+250), 10, cvScalar(255,0,255));
	cvNamedWindow("Field");
	// cvMoveWindow("Field",950,400);
	cvShowImage("Field", image);
	cvZero(image);
 	cvWaitKey();
 	//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
	#endif
	// cout<<"\nCalling Dijkstras\n";
	tree.dijkstra();
	// cout<<"post 2nd dijkstra implementation\n";
	a,b;
	a=1;
	// cout<<"Before return path point 1";
	b=tree.returnPathPoint(1);
	// cout<<"1\n";
	// cout<<"b: "<<b<<"\n";
	// cout<<"before if"<<endl;

	while(b!=0)
	{
		cvLine(image,cvPoint(tree[b].x+250,tree[b].y+250),cvPoint(tree[a].x+250,tree[a].y+250),cvScalar(0,0,255));
		a=b;
		b=tree.returnPathPoint(b);
		// cout<<"b: "<<b<<"\n";
	}
	cvLine(image,cvPoint(tree[0].x+250,tree[0].y+250),cvPoint(tree[a].x+250,tree[a].y+250),cvScalar(0,0,255));

	path_completed_flag=true;
	for(int i=0;i<NO_OF_OBSTACLES+4;i++)
	{
		cvCircle(image, cvPoint(obstacle[i].x+250, obstacle[i].y+250), obstacle[i].obstacle_radius, cvScalar(255,0,0));
	}
	cvCircle(image, cvPoint(start.x+250, start.y+250), 2, cvScalar(0,255,0));
	cvCircle(image, cvPoint(ball.x+250, ball.y+250), 2, cvScalar(255,0,255));
	cvCircle(image, cvPoint(goal.x+250, goal.y+250), 10, cvScalar(255,0,255));
	cvLine(image,cvPoint(tree[0].x+250,tree[0].y+250),cvPoint(tree[a].x+250,tree[a].y+250),cvScalar(0,0,255));
	CvFont font;
    cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, 0.5, 0.5, 0, 1, 8);
    char A[100];
    char B[100];					
    sprintf(A,"Ball Position:  %lf, %lf",ball.x,ball.y);
    sprintf(B,"Goal Position:  %lf, %lf",goal.x,goal.y);
    cvPutText(image,A,cvPoint(10,25),&font,cvScalar(255,255,255));
    cvPutText(image,B,cvPoint(10,45),&font,cvScalar(255,255,255));
	cvShowImage("Field", image);
	//--->> calculating the next points r and theta.
	// if(tree.is_onCircle(0, a))
	// {
	// 	if(tree.path_crash)
	// 		return NOPATH;
	// 	cout<<"after ret"<<endl;
	// 	cout<<"obstacle[tree[a].obstacle_id].obstacle_radius: "<<obstacle[tree[a].obstacle_id].obstacle_radius<<endl;
	// 	// cout<<"obstacle[tree[0].obstacle_id].obstacle_radius: "<<obstacle[tree[0].obstacle_id].obstacle_radius<<endl;
	// 	next.r=(10*180/3.1415926)*obstacle[tree[a].obstacle_id].obstacle_radius;
	// 	cout<<"next.r "<<next.r<<endl;
	// 	double m_prime= (tree[a].y - obstacle[tree[a].obstacle_id].y)/(tree[a].x - obstacle[tree[a].obstacle_id].x);
	// 	double m=-1/m_prime;
	// 	next.theta=rad2deg(atan(m));
	// 	////////cout<<"\nnext.r "<<next.r;
	// 	////////cout<<"\nnext.theta "<<next.theta;
	// }
	// else
	// {
	// 	cout<<"entered else"<<endl;
	// 	if(tree.path_crash)
	// 		return NOPATH;
	// 	next.r = sqrt(pow((tree[a].x-tree[0].x),2)+pow((tree[a].y-tree[0].y),2));
	// 	next.theta=rad2deg((atan2((tree[a].y-tree[0].y),(tree[a].x-tree[0].x))));
	// 	////////cout<<"\nnext.r "<<next.r;
	// 	////////cout<<"\nnext.theta "<<next.theta;
	// }
	
//copying it to an array
	std::size_t c,d;
	d=1;
	curve[0]=1;
	for(int i=0;i<tree.size();i++)
	{
		// printf("%d \n", i);
		curve[i+1]=tree.returnPathPoint(d);
		d=curve[i+1];
		if(d==0)
		{
			len_curve=i+2;
			break;
		}
	}

	// if(len_curve%2 != 0)
	// 	pathpackvar.NEAR_FLAG = 1;
	// else
	// 	pathpackvar.NEAR_FLAG = 0;


	// cout<<len_curve<<endl;
	std::size_t tmp=0;
	len_curvenext=0;
	for(int i=0,j=len_curve-1;i<len_curve/2,j>=len_curve/2;i++,j--)
	{
			tmp=curve[i];
			curve[i]=curve[j];
			curve[j]=tmp;
	}
	
#ifdef NEWENCIRCLING
//^^^^^^^^^^^^^^^^^^^^^^^^^^^new encircling and kicking and positioning^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
	if(car2pol(ball.x,ball.y)>2*ORIENTATION_RADIUS)
	{
		len_curvearray=len_curvenext;
		for(int i=0;i<len_curvearray;i++)
		{
			curvearray[i].r=curvenext[i].r*10;
			curvearray[i].theta=rad2deg(curvenext[i].theta);
			//cout<<curvearray[i].r<<"\t"<<curvearray[i].theta<<"\n";
		}

	}
	if(car2pol(ball.x,ball.y)<2*ORIENTATION_RADIUS)
	{
		//usleep(5000000);
		//cout<<"\n\n\n\n\n\n\n\n\n\n\n\n\n..............calling DOORIENT........................\n\n\n\n\n\n\n\n\n\n\n\n\n";
		//cout<<"\ncurvearray that is passed \n";
		for(int i=0;i<len_curvearray;i++)
		{
			//cout<<curvearray[i].r<<"\t"<<curvearray[i].theta<<"\n";
		}
		//cout<<"\n\n\n\n\n\n\n\n";
		//cvWaitKey();
		return DOORIENT;
	}

//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
#endif

	for(int i=0;i<NO_OF_OBSTACLES+4;i++)
	{
		cvCircle(image, cvPoint(obstacle[i].x+250, obstacle[i].y+250), obstacle[i].obstacle_radius, cvScalar(255,0,0));
	}
	cvCircle(image, cvPoint(start.x+250, start.y+250), 2, cvScalar(0,255,0));
	cvCircle(image, cvPoint(ball.x+250, ball.y+250), 2, cvScalar(255,0,255));
	cvCircle(image, cvPoint(goal.x+250, goal.y+250), 10, cvScalar(255,0,255));
	cvLine(image,cvPoint(tree[0].x+250,tree[0].y+250),cvPoint(tree[a].x+250,tree[a].y+250),cvScalar(0,0,255));

    sprintf(A,"Ball Position:  %lf, %lf",ball.x,ball.y);
    sprintf(B,"Goal Position:  %lf, %lf",goal.x,goal.y);
    cvPutText(image,A,cvPoint(10,25),&font,cvScalar(255,255,255));
    cvPutText(image,B,cvPoint(10,45),&font,cvScalar(255,255,255));
	cvShowImage("Field", image);
	if(tree.no_path_flag==-1)
	{
		tree.no_path_flag=0;
		return NOPATH;
	}
	if(Near_Flag)
	{
		return DOORIENT;
	}
	//cout<<"\npath returning \n\nDOWALK\n\n  theta "<<next.theta<< " distance "<<next.r<<endl;
	cvReleaseImage(&image);
    return DOWALK;
}

void Path::updatePathPacket()
{

	// pthread_mutex_lock(&mutex_pathpacket);

	// cout<<"abc"<<endl;
	if(!Near_Flag && !Back_Walk)
	{	
		// cout<<"before lock update pathpackvar"<<endl;
		// cout<<"before lock path not near"<<endl;
		pthread_mutex_lock(&mutex_pathpacket);
		pathpackvar.ROTATE = 0;
		pathpackvar.BALLFOLLOW = 0;
		pathpackvar.updated=1;
		pathpackvar.id=com_id;
		pathpackvar.NEAR_FLAG = 0;
		pathpackvar.BACK_WALK = 0;
		com_id=com_id+1;
		std::size_t b;
		b=tree.returnPathPoint(1);
		// pathpackvar.finalpath[0].x=ball.x;
		// pathpackvar.finalpath[0].y=ball.y;
		if(tree.size() >= 30)
		{
			tree.path_crash = true;
			// pthread_mutex_unlock(&mutex_pathpacket);
			return;
		}
		assert(tree.size()<30);
		// cout<<"before pathpackvar updated"<<endl;
		//commented original
		int i = 0;
		for(i=0;i<len_curve-1;i++)
		{
			pathpackvar.finalpath[i].x=tree[curve[i+1]].x;
			pathpackvar.finalpath[i].y=tree[curve[i+1]].y;
			if(tree.is_onCircle(curve[i], curve[i+1]))
			{
				pathpackvar.finalpath[i].isOnCircle = 1;
			}
			else
			{
				pathpackvar.finalpath[i].isOnCircle = 0;
			}
			// cout<<"before"<<endl;
			// pathpackvar.finalpath[i].obstacle_radius = obstacle[tree[curve[i+1]].obstacle_id].obstacle_radius;
			if(i)
				pathpackvar.finalpath[i].obstacle_radius = INITIAL_ORIENTATION_RADIUS;
			else
				pathpackvar.finalpath[i].obstacle_radius = OBSTACLE_RADIUS;
			// cout<<"after"<<endl;
			// printf("path %d x: %lf y: %lf\n", i, tree[b].x, tree[b].y);
			// b=tree.returnPathPoint(b);
		}
		if(pathpackvar.IGNORE_ARC != Ignore_Arc)
			pathpackvar.UPDATE_FLAG = 1;			

		if(Ignore_Arc)
		{
			pathpackvar.IGNORE_ARC = 1;
		}
		else
		{
			pathpackvar.IGNORE_ARC = 0;
		}

		// if(pathpackvar.NEAR_FLAG!=Near_Flag)
		// 	pathpackvar.UPDATE_FLAG = 1;
		if(pathpackvar.BACK_WALK != Back_Walk)
			pathpackvar.UPDATE_FLAG = 1;			

		pathpackvar.no_of_points=len_curve;

		pathpackvar.finalpath[i].x = goal.x;
		pathpackvar.finalpath[i].y = goal.y;
		pathpackvar.finalpath[i].isOnCircle = 1;
		// cout<<"goal.x: "<<goal.x<<endl;
		// cout<<pathpackvar.finalpath[i].x<<endl;
		// cout<<"No. of Points: "<<pathpackvar.no_of_points<<endl;
		
		if(pathpackvar.no_of_points<29)
		{
			pathpackvar.finalpath[pathpackvar.no_of_points+1].x=-1;
			pathpackvar.finalpath[pathpackvar.no_of_points+1].y=-1;
		}

		// for (int i = 0; i < pathpackvar.no_of_points; ++i)
		// {
		// 	cout<<"x: "<<pathpackvar.finalpath[i].x<<"y; "<<pathpackvar.finalpath[i].y<<endl;
		// }
		// for (int i = 0; i < pathpackvar.no_of_points; ++i)
		// {
		// 	cout<<"x: "<<pathpackvar.finalpath[i].x<<"y: "<<pathpackvar.finalpath[i].y<<endl;
		// }
			
			// pthread_mutex_unlock(&mutex_pathpacket);

		
		for (int i = 0; i < pathpackvar.no_of_points; ++i)
		{
			// cout<<"x: "<<pathpackvar.finalpath[i].x<<" y: "<<pathpackvar.finalpath[i].y<<" r: "<<pathpackvar.finalpath[i].obstacle_radius<<" ioc: "<<pathpackvar.finalpath[i].isOnCircle<<endl;
		}
		// cout<<"ignore arc: "<<pathpackvar.IGNORE_ARC<<endl;
		// cout<<endl;
		// cout<<"after unlock update pathpackvar"<<endl;
		pthread_mutex_unlock(&mutex_pathpacket);
	}

	if(Back_Walk && !Near_Flag)
	{
		pthread_mutex_lock(&mutex_pathpacket);
		pathpackvar.ROTATE = 0;
		pathpackvar.BALLFOLLOW = 0;
		pathpackvar.updated=1;
		pathpackvar.id=com_id;
		com_id=com_id+1;
		if(pathpackvar.BACK_WALK != Back_Walk)
			pathpackvar.UPDATE_FLAG = 1;
		pathpackvar.BACK_WALK = 1;
		pathpackvar.finalpath[0].x = BackWalkX;
		pathpackvar.finalpath[0].y = 0.0;
		pathpackvar.no_of_points = 1;

	}
		pthread_mutex_unlock(&mutex_pathpacket);

	if(Near_Flag && !Back_Walk)
	{
		pthread_mutex_lock(&mutex_pathpacket);
		pathpackvar.updated=1;
		pathpackvar.ROTATE = 0;
		pathpackvar.BALLFOLLOW = 0;
		pathpackvar.id=com_id;
		com_id=com_id+1;
		// if(pathpackvar.NEAR_FLAG!=Near_Flag)
		// 	pathpackvar.UPDATE_FLAG = 1;
		if(pathpackvar.BACK_WALK != Back_Walk)
			pathpackvar.UPDATE_FLAG = 1;			
		pathpackvar.NEAR_FLAG = 1;
		// pathpackvar.BACK_WALK = 0;
		//pathpackvar.finalpath[0].x = BackWalkX;
		// pathpackvar.finalpath[0].y = 0.0;
		pthread_mutex_unlock(&mutex_pathpacket);
	}
	// cout<<"NF: "<<Near_Flag<<" BW: "<<Back_Walk<<endl;
	if(Near_Obstacle)
	{
		pthread_mutex_lock(&mutex_pathpacket);		
		pathpackvar.updated=1;
		pathpackvar.ROTATE = 0;	
		pathpackvar.BALLFOLLOW = 0;	
		pathpackvar.id=com_id;
		com_id=com_id+1;
		// pathpackvar.NEAR_OBSTACLE = 1;
		if(pathpackvar.BACK_WALK != Back_Walk)
			pathpackvar.UPDATE_FLAG = 1;					
		pathpackvar.BACK_WALK = 1;
		pathpackvar.finalpath[0].x = -10.0;
		pathpackvar.finalpath[0].y = 0.0;
		pathpackvar.no_of_points = 1;
		pthread_mutex_unlock(&mutex_pathpacket);		
	}
	// if(Near_Flag || Back_Walk || Near_Obstacle)
	// 	cout<<"NF: "<<Near_Flag<<" BW: "<<Back_Walk<<" NO: "<<Near_Obstacle<<endl;
}
