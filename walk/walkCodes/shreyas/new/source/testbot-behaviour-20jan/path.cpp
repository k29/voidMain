	
#include "path.hpp"
#define PI 3.14159265
#define deg2rad(x) (x)*(PI/180.0)
#define rad2deg(x) (x)*(180.0/PI)
#define car2pol(x,y) sqrt((x)*(x)+(y)*(y))	
#define dist(x1,y1,x2,y2) sqrt(pow(((x1)-(x2)),2)+pow(((y1)-(y2)),2))
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
	double m = (a.y-ball.y)/(a.x-ball.x);
	double c = ball.y-(m*ball.x);
	if(((goal.y-(m*goal.x)-c)*(b.y-(m*b.x)-c))<0) // other side of the line...point to be kept
		
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
			if(dist<obstacle[i].obstacle_radius-0.2)
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
		cout<<"\t\tNo Intersection\n";
		return no_intersection;
	}

	if((k == NO_OF_OBSTACLES || k == NO_OF_OBSTACLES +1) && n2.x==ball.x && n2.y==ball.y)
	{
		calculate_points(n1,obstacle[k],n1a,n1b); 
		n1a.obstacle_id=k;
		n1b.obstacle_id=k;
		cout<<"\t\tFinal Orientation Intersection\n";
		return final_orientation_intersection;
	}

	if(n1.obstacle_id==k || n2.obstacle_id==k)
	{
		Point midpoint, p1, p2;
		p1.x=0;
		p2.x=0;
		p1.y=0;
		p2.y=0;

		cout<<"the obstacle obstacle[n1.obstacle_id] "<<obstacle[n1.obstacle_id].x<<" "<<obstacle[n1.obstacle_id].y<<endl;
		cout<<"the obstacle obstacle[n2.obstacle_id] "<<obstacle[n2.obstacle_id].x<<" "<<obstacle[n2.obstacle_id].y<<endl;

		midpoint.x=(obstacle[n1.obstacle_id].x + obstacle[n2.obstacle_id].x)/2;
		midpoint.y=(obstacle[n1.obstacle_id].y + obstacle[n2.obstacle_id].y)/2;
		// cout<<"midpoint "<<midpoint.x<<" "<<midpoint.y<<endl;
		// cout<<"midpoint.x "<<midpoint.x<<endl;
		// cout<<"obstacle[n1.obstacle_id].x "<<obstacle[n1.obstacle_id].x<<endl;
		// cout<<"midpoint.y "<<midpoint.y<<endl;
		// cout<<"obstacle[n1.obstacle_id].y "<<obstacle[n1.obstacle_id].y<<endl;

		// if(((pow((midpoint.x-obstacle[n1.obstacle_id].x),2))+(pow((midpoint.y-obstacle[n1.obstacle_id].y),2))-pow(obstacle[n1.obstacle_id].obstacle_radius,2))>0)
		// 	cout<<"yes\n\n\n\n\n......\n\n\n\n";
		if((((pow((midpoint.x-obstacle[n1.obstacle_id].x),2))+(pow((midpoint.y-obstacle[n1.obstacle_id].y),2))-pow(obstacle[n1.obstacle_id].obstacle_radius,2))>0) && (((pow((midpoint.x-obstacle[n2.obstacle_id].x),2))+(pow((midpoint.y-obstacle[n2.obstacle_id].y),2))-pow(obstacle[n2.obstacle_id].obstacle_radius,2))>0))
		{
			calculate_points(midpoint,obstacle[n1.obstacle_id],p1,p2);
			cout<<"p1 on n1 "<<p1.x<<" "<<p1.y<<endl;
			cout<<"p2 on n1 "<<p2.x<<" "<<p2.y<<endl;
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
			cout<<"p1 on n2 "<<p1.x<<" "<<p1.y<<endl;	
			cout<<"p2 on n2 "<<p2.x<<" "<<p2.y<<endl;
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
			cout<<"the midpoint is lying inside the circle taking lite!!\n";
			midpoint_flag=true;
		}
		cout<<"\t\tspecial intersection\n";
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
	
	cout<<"\t\tnormal intersection\n";
	return normal_intersection;
}

double Path::cost_calculation(std::size_t source,std::size_t target, Graph <Point> &tree) //Returns cost of the path b/w the two points given. Note that the if the points are on circle then the cost is calculated accordingly. 
{
	double cost;
	
	Point source_point=tree[source];
	Point target_point=tree[target];
	if(tree.is_onCircle(source,target)==false) //
	{
		cost=sqrt(pow((source_point.y - target_point.y),2)+pow((source_point.x - target_point.x),2));
	}
	else
	{
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




PathReturns Path::path_return(PathStructure ps)
{
	srand(time(NULL));
	IplImage* image= cvCreateImage(cvSize(SIZEX*2, SIZEY*2), 8, 3); //displaying the results.
	cvZero(image);

	int flag=1; //Flag used for iteration in the graph...i.e. assigned the value 0 once iteration through the entire graph is done to break out of loop.
	
	start.x=0;
	start.y=0;
	start.obstacle_id=-1;
	start.parent_id=0;
	cvCircle(image, cvPoint(start.x + 200, start.y + 200), 2, cvScalar(0,255,0)); //Initialising the start point and painting it.

	NO_OF_OBSTACLES = ps.n_obstacles;

	ball.x=ps.ball.x;
	ball.y=ps.ball.y;	
	ball.obstacle_id=-1;
	ball.parent_id=0;
	cout<<"ball position"<<ball.x<<" "<<ball.y<<endl;
	cvCircle(image, cvPoint(ball.x+200, ball.y+200), 2, cvScalar(255,0,255)); //Initialising the ball point and painting it.
	
	goal.x=ps.goal.x;
	goal.y=ps.goal.y;	
	goal.obstacle_id=-1;
	cout<<"goal position"<<goal.x<<" "<<goal.y<<endl;
	cvCircle(image, cvPoint(goal.x+200, goal.y+200), 10, cvScalar(255,0,255)); //Initialising the goal point and painting it.

	tree.cleartree(); //Clearing all previous data from the graphs....i.e. initialises the graph for a fresh start.

	for(int i = 0; i < NO_OF_OBSTACLES; i++)
	{
		obstacle[i].x = ps.absObstacles[i].x;
		obstacle[i].y = ps.absObstacles[i].y;
		obstacle[i].obstacle_radius=OBSTACLE_RADIUS;
		obstacle[i].type= CIRCLE;
		cout<<"obstacle position"<<obstacle[i].x<<" "<<obstacle[i].y<<endl;
		cvCircle(image, cvPoint(obstacle[i].x+200,obstacle[i].y+200), obstacle[i].obstacle_radius, cvScalar(255,0,0));

	} //Initialising the obstacles and painting them.



	cvCircle(image, cvPoint(ball.x+200, ball.y+200), ENCIRCLE_THRESHOLD , cvScalar(0,130,70));

	//Making the two orientation markers near the ball  ---->
	double m=(goal.y-ball.y)/(goal.x-ball.x);
	double m_prime=-1/m;
	double cos_prime=1/(sqrt(pow(m_prime,2)+1));
	double sin_prime=m_prime/(sqrt(pow(m_prime,2)+1));
	obstacle[NO_OF_OBSTACLES].obstacle_radius= ORIENTATION_RADIUS;
	obstacle[NO_OF_OBSTACLES].x=ball.x - ( ORIENTATION_RADIUS*cos_prime);
	obstacle[NO_OF_OBSTACLES].y=ball.y - ( ORIENTATION_RADIUS*sin_prime);
	obstacle[NO_OF_OBSTACLES].obstacle_id=-2;
	obstacle[NO_OF_OBSTACLES].type= CIRCLE;
	cvCircle(image, cvPoint(obstacle[NO_OF_OBSTACLES].x+200, obstacle[NO_OF_OBSTACLES].y+200),  ORIENTATION_RADIUS, cvScalar(255,0,0));
	obstacle[NO_OF_OBSTACLES+1].obstacle_radius= ORIENTATION_RADIUS;
	obstacle[NO_OF_OBSTACLES+1].x=ball.x + ( ORIENTATION_RADIUS*cos_prime);
	obstacle[NO_OF_OBSTACLES+1].y=ball.y + ( ORIENTATION_RADIUS*sin_prime);
	obstacle[NO_OF_OBSTACLES+1].obstacle_id=-2;
	obstacle[NO_OF_OBSTACLES+1].type= CIRCLE;
	cvCircle(image, cvPoint(obstacle[NO_OF_OBSTACLES+1].x+200, obstacle[NO_OF_OBSTACLES+1].y+200),  ORIENTATION_RADIUS, cvScalar(255,0,0)); 
	
	//cout<<"orientation circle position "<<obstacle[NO_OF_OBSTACLES].x<<" "<<obstacle[NO_OF_OBSTACLES].y<<endl;
	//cout<<"orientation circle position "<<obstacle[NO_OF_OBSTACLES+1].x<<" "<<obstacle[NO_OF_OBSTACLES+1].y<<endl;
	//cout<<"orientaion radius "<<ORIENTATION_RADIUS<<endl;


	for(int i=0;i< NO_OF_OBSTACLES;i++)
	{
		for(int j=NO_OF_OBSTACLES;j<NO_OF_OBSTACLES+2;j++)
		{			
			double d=sqrt(pow((obstacle[i].x - obstacle[j].x),2)+pow((obstacle[i].y - obstacle[j].y),2));
			if(d < (obstacle[i].obstacle_radius+obstacle[j].obstacle_radius)-0.005)
			{//now the obstacles are intersecting
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

	for(int i=0;i<NO_OF_OBSTACLES+2;i++)
	{
		if(dist(obstacle[i].x,obstacle[i].y,0,0)<obstacle[i].obstacle_radius)
		{
			obstacle[i].x=1000;
			obstacle[i].y=1000;
		}
	}



	
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------
	//Encircling, Walking, Kicking decission ----->
	
	if(car2pol(ball.x,ball.y) <= KICKDIST)
	{

		return DOKICK;
	}	
	//ENCIRCLE_THRES is always > KICKDIST
	else if(car2pol(ball.x,ball.y) <= ENCIRCLE_THRESHOLD) 
	{		
		//ballgoalangle=>angle made by line joining acyut and ball with line joining ball and goal
		tolerance_angle=rad2deg(atan(BADANGLEDIST/sqrt(pow(ball.x,2)+pow(ball.y,2)))); //dyanmically generating tolerance angle for orientation purposes(i.e facing the ball before orienting)
		double ballangle=rad2deg(atan2(ball.y,ball.x)); 
		if(abs(ballangle)>tolerance_angle)  
		{
			if(ballangle>0)
			{
				next.theta=(abs(ballangle)-tolerance_angle)*-1; //-1 is due to the ip convention and walk convention
				next.r=0;
				cout<<"Turning to orient by "<<next.theta<<endl;
				return DOWALK;
			}
			else
			{
				next.theta=(-(abs(ballangle)-tolerance_angle))*-1;
				next.r=0;
				cout<<"Turning to orient by "<<next.theta<<endl;
				return DOWALK;
			}
		} // acyut is now facing the ball .... ready to encircle
		double a=car2pol(ball.x,ball.y);
		double b=car2pol(ball.x-goal.x,ball.y-goal.y);
		double c=car2pol(goal.x,goal.y);
		double ballgoalangle=180- acos((a*a+b*b-c*c)/(2*a*b)); //using cosine rule to get angle required to encircle ( for understanding refer encircleangleidea.jpg in source)
		NextEncircleAngle=ballgoalangle*-1; // the -1 is due to ip convention and walk convention.
		cout<<"Normal encircling"<<endl;
		return DOENCIRCLE; //encircle
	}
	else if(car2pol(ball.x,ball.y) <= 2*ORIENTATION_RADIUS) //if i am inside the orientation circle while on the path then follow straight till u get to the encircle radius
	{
		//cvWaitKey();
		double ballangle=rad2deg(atan2(ball.y,ball.x)); 
		next.r=car2pol(ball.x,ball.y);
		next.theta=ballangle;
		cout<<"In Orientation outside Encircle"<<endl;
		return DOWALK;
	}
	
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
	
	tree.add_vertex(start);
	tree.add_vertex(ball);
	tree.add_edge(0,1,cost_calculation(0,1,tree));
	cvLine(image,cvPoint(200,200),cvPoint(ball.x+200,ball.y+200),cvScalar(255,255,0)); // creates the nodes for start and ball point and paints a line.

	while(flag) // iterates over the entire tree.
	{	
		flag=0; 
		
		for(size_t n1_index=0;n1_index<tree.size(); n1_index++)
		{
			for(size_t n2_index = 0; n2_index < tree.size(); n2_index++) // looping through the entire tree.
			{
				if(tree.is_edge(n1_index, n2_index))
				{
					cout<<"Analysing:- "<<n1_index<<" "<<n2_index<<"\n";
					Point n1 = tree[n1_index];
					Point n2 = tree[n2_index];
					
					//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
					// //code for frame by frame painting.						
					// 	for(size_t n1_index=0;n1_index<tree.size(); n1_index++)
					// 	{
					// 		for(size_t n2_index = 0; n2_index < tree.size(); n2_index++)
					// 		{
					// 			if(tree.is_edge(n1_index, n2_index))
					// 			{
					// 				Point n1 = tree[n1_index];
					// 				Point n2 = tree[n2_index];
					// 				cvLine(image,cvPoint(n1.x+200,n1.y+200),cvPoint(n2.x+200,n2.y+200),cvScalar(255,255,0));
					// 			}
					// 		}
					// 	}

					// 	for(int i=0;i<NO_OF_OBSTACLES+2;i++)
					// 	{										
					// 		cvCircle(image, cvPoint(obstacle[i].x+200, obstacle[i].y+200), 2, cvScalar(255,0,0));
					// 		cvCircle(image, cvPoint(obstacle[i].x+200, obstacle[i].y+200), obstacle[i].obstacle_radius , cvScalar(255,0,0));
					// 	}
					// 	cvCircle(image, cvPoint(n1.x+200, n1.y+200), 2, cvScalar(255,255,255), 2);
					// 	cvCircle(image, cvPoint(n2.x+200, n2.y+200), 2, cvScalar(0,0,255), 2);
					// 	cvCircle(image, cvPoint(start.x + 200, start.y + 200), 2, cvScalar(0,255,0));
					// 	cvCircle(image, cvPoint(ball.x+200, ball.y+200), 2, cvScalar(255,0,255));
					// 	cvCircle(image, cvPoint(goal.x+200, goal.y+200), 10, cvScalar(255,0,255));
					// 	cvShowImage("Field", image);
					// 	cvZero(image);
					//  	cvWaitKey();
					//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
					
					if(tree.is_onCircle(n1_index, n2_index)==false)
					{							
						switch(main_function(n1,n2))
						{

							case normal_intersection: 
							{
								flag = 1;
								cout<<"analysing "<<n1_index<<" "<<n2_index<<endl;
								//cout<<"parent id of "<<n1_index<<" is "<<n1.parent_id<<endl;
								//cout<<"parent id of "<<n2_index<<" is "<<n2.parent_id<<endl;
								//cout<<"removing edges"<<endl;
								tree.remove_edge(n1_index, n2_index); //removing the edges for which there is an obstacle in b/w. 
								n1a.parent_id=n1_index;							//			
								size_t n1a_index = tree.add_vertex(n1a);			//
								n1b.parent_id=n1_index;
								size_t n1b_index = tree.add_vertex(n1b);			//
								n2a.parent_id=n1_index;									//adding the four new nodes				
								size_t n2a_index = tree.add_vertex(n2a);			//
								n2b.parent_id=n1_index;								//	
								size_t n2b_index = tree.add_vertex(n2b);			//
								 //adding parent ids...note that parent ids for the all the four points is the parent point i.e. n1.
								// cout<<"parent id of "<<n1a_index<<" is "<<n1a.parent_id<<endl;
								// cout<<"parent id of "<<n1b_index<<" is "<<n1b.parent_id<<endl;
								// cout<<"parent id of "<<n2a_index<<" is "<<n2a.parent_id<<endl;
								// cout<<"parent id of "<<n2b_index<<" is "<<n2b.parent_id<<endl;

								tree.add_edge(n1_index, n1a_index,cost_calculation(n1_index,n1a_index,tree));
								if(closest_intersecting_obstacle(tree[n1.parent_id],tree[n1a_index])==-1)//check the obstacle intersection from n1's parent to the point n1a...n1b..... 
								{			

									n1a.parent_id=n1.parent_id;	
									tree.update_vertex(n1a,n1a_index);						
									tree.add_edge(n1.parent_id,n1a_index,cost_calculation(n1.parent_id,n1a_index,tree));//add edge b/w the parent point and the point n1a,nb...
																	
								}
								tree.add_edge(n1_index, n1b_index,cost_calculation(n1_index,n1b_index,tree));
								if(closest_intersecting_obstacle(tree[n1.parent_id],tree[n1b_index])==-1)//check the obstacle intersection from n1's parent to the point n1a...n1b..... 
								{
									n1b.parent_id=n1.parent_id;	
									tree.update_vertex(n1b,n1b_index);	
									tree.add_edge(n1.parent_id,n1b_index,cost_calculation(n1.parent_id,n1b_index,tree));//add edge b/w the parent point and the point n1a,nb...
														
								}
							
								tree.add_edge(n2a_index, n2_index,cost_calculation(n2a_index,n2_index,tree));
								if(closest_intersecting_obstacle(tree[n2a.parent_id],tree[n2_index])==-1)//check the obstacle intersection from n1's parent to the point n1a...n1b..... 
								{
									n2.parent_id=n2a.parent_id;
									tree.update_vertex(n2,n2_index);	
									tree.add_edge(n2a.parent_id,n2_index,cost_calculation(n2a.parent_id,n2_index,tree));//add edge b/w the parent point and the point n1a,nb...
									
								}
								
								tree.add_edge(n2b_index, n2_index,cost_calculation(n2b_index,n2_index,tree));
								if(closest_intersecting_obstacle(tree[n2b.parent_id],tree[n2_index])==-1)//check the obstacle intersection from n1's parent to the point n1a...n1b..... 
								{
									n2.parent_id=n2b.parent_id;
									tree.update_vertex(n2,n2_index);	
									tree.add_edge(n2b.parent_id,n2_index,cost_calculation(n2b.parent_id,n2_index,tree));//add edge b/w the parent point and the point n1a,nb...
									
								}
								
								if(same_side_flag)
								{
									tree.add_edge(n1a_index, n2a_index,cost_calculation(n1a_index,n2a_index,tree), true);
									tree.add_edge(n1b_index, n2b_index,cost_calculation(n1b_index,n2b_index,tree), true);
								}
								else
								{								
									tree.add_edge(n1a_index, n2b_index,cost_calculation(n1a_index,n2b_index,tree), true);
									tree.add_edge(n1b_index, n2a_index,cost_calculation(n1b_index,n2a_index,tree), true);
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
									cout<<"removed edges b/w "<<n1_index<<" "<<n2_index<<endl;
									n1a.parent_id=n1_index;	
									n2a.parent_id=n1_index;	
									size_t n1a_index = tree.add_vertex(n1a);
									size_t n2a_index = tree.add_vertex(n2a);
									cout<<"n1a "<<n1a.x<<" "<<n1a.y<<endl;
									cout<<"n2a "<<n2a.x<<" "<<n2a.y<<endl;
									tree.add_edge(n1_index, n1a_index,cost_calculation(n1_index,n1a_index,tree), true);
									tree.add_edge(n1a_index, n2a_index,cost_calculation(n1a_index,n2a_index,tree));
									tree.add_edge(n2a_index, n2_index,cost_calculation(n2a_index,n2_index,tree),true);
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

								cout<<".......................................................................\n";
								
								flag=1;
								tree.remove_edge(n1_index,n2_index);
								
								size_t n1a_index = tree.add_vertex(n1a);
								size_t n1b_index = tree.add_vertex(n1b);
													
								tree.add_edge(n1_index, n1a_index,cost_calculation(n1_index,n1a_index,tree));
								

								if(closest_intersecting_obstacle(tree[n1.parent_id],tree[n1a_index])==-1)//check the obstacle intersection from n1's parent to the point n1a...n1b..... 
								{
									n1a.parent_id=n1.parent_id;		
									tree.update_vertex(n1a,n1a_index);	
									tree.add_edge(n1.parent_id,n1a_index,cost_calculation(n1.parent_id,n1a_index,tree));//add edge b/w the parent point and the point n1a,nb...														
								}
								
								tree.add_edge(n1_index, n1b_index,cost_calculation(n1_index,n1b_index,tree));	
								
								if(closest_intersecting_obstacle(tree[n1.parent_id],tree[n1b_index])==-1)//check the obstacle intersection from n1's parent to the point n1a...n1b..... 
								{
									n1b.parent_id=n1.parent_id;	
									tree.update_vertex(n1b,n1b_index);
									tree.add_edge(n1.parent_id,n1b_index,cost_calculation(n1.parent_id,n1b_index,tree));//add edge b/w the parent point and the point n1a,nb...																	
								}								
								tree.add_edge(n1a_index,n2_index,cost_calculation(n1a_index,n2_index,tree),true);								
								tree.add_edge(n1b_index,n2_index,cost_calculation(n1b_index,n2_index,tree),true);									
							}		
						}

						

//****************************************************************************************************************************************************************
// //code for frame by frame painting.						
						// for(size_t n1_index=0;n1_index<tree.size(); n1_index++)
						// {
						// 	for(size_t n2_index = 0; n2_index < tree.size(); n2_index++)
						// 	{
						// 		if(tree.is_edge(n1_index, n2_index))
						// 		{
						// 			Point n1 = tree[n1_index];
						// 			Point n2 = tree[n2_index];
						// 			cvLine(image,cvPoint(n1.x+200,n1.y+200),cvPoint(n2.x+200,n2.y+200),cvScalar(255,255,0));
						// 		}
						// 	}
						// }

						// for(int i=0;i<NO_OF_OBSTACLES+2;i++)
						// {										
						// 	cvCircle(image, cvPoint(obstacle[i].x+200, obstacle[i].y+200), 2, cvScalar(255,0,0));
						// 	cvCircle(image, cvPoint(obstacle[i].x+200, obstacle[i].y+200), obstacle[i].obstacle_radius , cvScalar(255,0,0));
						// }
						// cvCircle(image, cvPoint(n1.x+200, n1.y+200), 2, cvScalar(255,255,255), 2);
						// cvCircle(image, cvPoint(n2.x+200, n2.y+200), 2, cvScalar(0,0,255), 2);
						// cvCircle(image, cvPoint(start.x + 200, start.y + 200), 2, cvScalar(0,255,0));
						// cvCircle(image, cvPoint(ball.x+200, ball.y+200), 2, cvScalar(255,0,255));
						// cvCircle(image, cvPoint(goal.x+200, goal.y+200), 10, cvScalar(255,0,255));
						// cvShowImage("Field", image);
						// cvZero(image);
			