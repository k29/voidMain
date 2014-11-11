#include "AcYut.h"
#include "communication.h"
#include "xsens/imu.h"
#include <fstream>
#include <cstdio>
#include <vector>

using namespace std;


// float bot.init_val[14] = {4096-2048, 4096-1100, 2048, 4096-3000, 2048, 1100, 2048, 3000, 390, 0, 0, 390, 0, 0};
float t_default = 2;


struct move_pose
{
	float hands[2][4];
	float legs[2][4];
	char name[15];
};

struct move
 {
 	char name [15];
 	float t_pos[30];
 	int no_of_pos;
 	move_pose mp[30];
 }; 

float output[480];
float scurve(float in,float fi,float t, float tot)
{
	float frac=(float)t/(float)tot;
	if(frac>1)
		frac=1;
	float retfrac=frac*frac*(3-2*frac);
	float mval=in + (fi-in)*retfrac;
	return mval;
}

void goto_movepose(move_pose mp1, move_pose mp2, float t_pos, AcYut &bot, Communication &comm)
{


	int vm7,vm8,vm9,vm10,vm27,vm28,vm29,vm30; 
	float vlx,vly,vlz,vrx,vry,vrz,vltheta,vrtheta;
	
	int i;
	
	int t_cur;
	
	int arr_l[4],arr_r[4];
	
	for(t_cur = 0; t_cur<t_pos*60; t_cur++)
	{
		arr_l[0]=vm7=(int)scurve(mp1.hands[0][0],mp2.hands[0][0],t_cur, t_pos*60);
		arr_l[1]=vm8=(int)scurve(mp1.hands[0][1],mp2.hands[0][1],t_cur, t_pos*60);
		arr_l[2]=vm9=(int)scurve(mp1.hands[0][2],mp2.hands[0][2],t_cur, t_pos*60);
		arr_l[3]=vm10=(int)scurve(mp1.hands[0][3],mp2.hands[0][3],t_cur, t_pos*60);

		arr_r[0]=vm27=(int)scurve(mp1.hands[1][0],mp2.hands[1][0],t_cur, t_pos*60);
		arr_r[1]=vm28=(int)scurve(mp1.hands[1][1],mp2.hands[1][1],t_cur, t_pos*60);
		arr_r[2]=vm29=(int)scurve(mp1.hands[1][2],mp2.hands[1][2],t_cur, t_pos*60);
		arr_r[3]=vm30=(int)scurve(mp1.hands[1][3],mp2.hands[1][3],t_cur, t_pos*60);
		
		vlx=scurve(mp1.legs[0][0],mp2.legs[0][0],t_cur, t_pos*60);
		vly=scurve(mp1.legs[0][1],mp2.legs[0][1],t_cur, t_pos*60);
		vlz=scurve(mp1.legs[0][2],mp2.legs[0][2],t_cur, t_pos*60);
		vltheta=scurve(mp1.legs[0][3],mp2.legs[0][3],t_cur, t_pos*60);
		
		vrx=scurve(mp1.legs[1][0],mp2.legs[1][0],t_cur, t_pos*60);
		vry=scurve(mp1.legs[1][1],mp2.legs[1][1],t_cur, t_pos*60);
		vrz=scurve(mp1.legs[1][2],mp2.legs[1][2],t_cur, t_pos*60);
		vrtheta=scurve(mp1.legs[1][3],mp2.legs[1][3],t_cur, t_pos*60);
		
		bot.left_hand->setGoalPositionSync(arr_l);
		bot.right_hand->setGoalPositionSync(arr_r);
		bot.left_leg->runIK(vlx,vly,vlz,vltheta);
		bot.left_leg->setGoalPositionSync();
		bot.right_leg->runIK(vrx,vry,vrz,vrtheta);
		bot.right_leg->setGoalPositionSync();
		comm.syncFlush();
		usleep(16666);
		printf("%d %d %d %d %d %d %d %d %f %f %f %f %f %f %f %f\n",vm7,vm8,vm9,vm10,vm27,vm28,vm29,vm30,vlx,vly,vlz,vltheta,vrx,vry,vrz,vrtheta);
						
	}
}


void access_move_list(move_pose mpinit, AcYut &bot, Communication &comm)
{
	move m;

	fstream file;
	file.open ("movestore.data", ios::in|ios::binary);
	
	vector<move> marr;

	int i = 0;

	while (file.read((char*)&m, sizeof(m)))
	{
		marr.push_back(m);
		i++;
	}

	file.close();

	int ch = 1;


	
	while(ch)
	{
		int count = 0;
		cout<<endl<<endl;
		while (count++<i)
			cout<<count<<". "<<marr[count-1].name<<endl;
		cout<<"\n\n0. Go back	1. Execute a move (Enter position number)\n2. Delete a move (Enter negative of move number)\n\n";
		cin>>ch;
		if (ch<=i&&ch>0)
		{
			cout<<endl<<"Executing "<<marr[ch-1].name<<endl;
			int c = 0;
			goto_movepose(mpinit,marr[ch-1].mp[0],marr[ch-1].t_pos[c], bot, comm);
			c++;

			while (c < marr[ch-1].no_of_pos)
			{
				cout<<"\nGoing to "<<marr[ch-1].mp[c].name<<" in "<<marr[ch-1].t_pos[c]<<endl;
				goto_movepose(marr[ch-1].mp[c-1],marr[ch-1].mp[c],marr[ch-1].t_pos[c], bot, comm);
				c++;
			}
			goto_movepose(marr[ch-1].mp[c-1],mpinit,t_default, bot, comm);
		}
		else if(ch>=-i&&ch<0)
		{
			count = 0;

			fstream file;
			file.open ("movestore.data", ios::out|ios::binary);
			
			while(count++<i)
			{	
				if (count!=-ch)
					file.write((char*)&marr[count-1],sizeof(marr[count-1]));
				else
				{
					marr.erase(marr.begin() + count-1);
					count--;
					i--;
					ch = 1;
				}
			}
			file.close();			
		}
	}

}

void setdefault(float arr[], int j, AcYut &bot)
{
	switch	(j)
	{
		case 0: arr[0] = bot.init_val[0];
				arr[1] = bot.init_val[1];
				arr[2] = bot.init_val[2];
				arr[3] = bot.init_val[3]; 
			break;
		case 1:	arr[0] = bot.init_val[4];
				arr[1] = bot.init_val[5];
				arr[2] = bot.init_val[6];
				arr[3] = bot.init_val[7];
			break;
		case 2:	arr[0] = bot.init_val[8];
				arr[1] = bot.init_val[9];
				arr[2] = bot.init_val[10];
				arr[3] = bot.init_val[11];
			break;
		case 3:	arr[0] = bot.init_val[12];
				arr[1] = bot.init_val[13];
				arr[2] = bot.init_val[14];
				arr[3] = bot.init_val[15];
			break;
	}
}

int select_position(move_pose &mp)
{
	fstream file;
	file.open ("positionstore.data", ios::in|ios::binary);	

	vector<move_pose> mparr;

	int i = 0;

	while (file.read((char*)&mp, sizeof(mp)))
	{
		mparr.push_back(mp);
		cout<<i+1<<". "<<mparr[i].name<<endl;
		i++;
	}

	file.close();

	int ch;
	
	cout<<"\n\nEnter the next position to insert into the move: ";
	cin>>ch;

	if (ch<=i+1 && ch>0)
	{
		mp = mparr[ch-1];
		return 1;
	}

	else
		return 0;
}

void create_move_position(move_pose mpinit, AcYut &bot, Communication &comm)
{

	//Must handle evil inputs to secure integrity of file.

	move_pose mp;
	cout<<"\nEnter a name for your move position : ";
	cin>>mp.name;

	float arr[4][4];

	cout<<"\n\nFor any limb, you may enter -1 to send it to the default (init) position\n";

	for (int j = 0; j < 4; ++j)
	{
		switch (j)
		{
			case 0: cout<<"\nLeft hand: Motors 7,8,9,10:\n"<<"Defaults : "<<bot.init_val[0]<<" "<<bot.init_val[1]<<" "<<bot.init_val[2]<<" "<<bot.init_val[3]<<endl;
				break;
			case 1: cout<<"\nRight hand: Motors 27,28,29,30:\n"<<"Defaults : "<<bot.init_val[4]<<" "<<bot.init_val[5]<<" "<<bot.init_val[6]<<" "<<bot.init_val[7]<<endl;
				break;
			case 2: cout<<"\nLeft leg: x, y, z, theta\n"<<"Defaults : "<<bot.init_val[8]<<" "<<bot.init_val[9]<<" "<<bot.init_val[10]<<" "<<bot.init_val[11]<<endl;
				break;
			case 3: cout<<"\nRight leg: x, y, z, theta:\n"<<"Defaults : "<<bot.init_val[12]<<" "<<bot.init_val[13]<<" "<<bot.init_val[14]<<" "<<bot.init_val[15]<<endl;
				break;
		}
		
		int b = 1;
		for (int i = 0; (i < 4)&&b; ++i)
		{
			if (j-2<0)
			{
				cin>>mp.hands[j][i];
				if (mp.hands[j][i] < 0)
				{
					setdefault(mp.hands[j],j,bot);
					b = 0;
				}
			}
			else
			{
				cin>>mp.legs[j-2][i];
				if (mp.legs[j-2][i] < 0)
				{
					setdefault(mp.legs[j-2],j,bot);
					b = 0;
				}

			}
		}
	}
	fstream file;
	file.open ("positionstore.data", ios::out|ios::binary|ios::app);
	file.write((char*)&mp,sizeof(mp));
	file.close();	
}

void display_move_pose_list(move_pose mpinit, AcYut &bot, Communication &comm)
{
	move_pose mp;

	fstream file;
	file.open ("positionstore.data", ios::in|ios::binary);

	

	vector<move_pose> mparr;

	int i = 0;

	while (file.read((char*)&mp, sizeof(mp)))
	{
		mparr.push_back(mp);
		i++;
	}

	file.close();

	int ch = 1;


	
	while(ch)
	{
		int count = 0;
		cout<<endl<<endl;
		while (count++<i)
			cout<<count<<". "<<mparr[count-1].name<<endl;
		cout<<"\n\n0. Go back	1. Go to a position (Enter position number)\n2. Delete a position (Enter negative of position number)\n\n";
		cin>>ch;
		if (ch<=i&&ch>0)
		{
			cout<<endl<<"Going to "<<mparr[ch-1].name<<endl;
			goto_movepose(mpinit,mparr[ch-1],t_default,bot,comm);

			cout<<"Return to init position(y/n)?	\n\n**NOTE: Always send bot to a specific position from init position only**\n\n";
			char ret;
			cin>>ret;
			if (ret == 'y')
				goto_movepose(mparr[ch-1],mpinit,t_default,bot,comm);				

		}
		else if(ch>=-i&&ch<0)
		{
			count = 0;

			fstream file;
			file.open ("positionstore.data", ios::out|ios::binary);
			
			while(count++<i)
			{	
				if (count!=-ch)
					file.write((char*)&mparr[count-1],sizeof(mparr[count-1]));
				else
				{
					mparr.erase(mparr.begin() + count-1);
					count--;
					i--;
					ch = 1;
				}
			}
			file.close();			
		}
	}

}

void create_move(move_pose mpinit, AcYut &bot, Communication &comm)
{
	move m;
	cout<<"\nEnter a name for your move position : ";
	cin>>m.name;

	int ch = 1;
	int i = 0;
	while (ch<3&&ch>0)
	{
		cout<<"\n\n1. Enter new position into move 	2. Delete last position added\n3. Complete move   4. Discard Move \n\n";
		cin>>ch;
		switch(ch)
		{
			case 1:	if (select_position(m.mp[i]))
					{
						cout<<"\n\nEnter the time for attaining given position: ";
						cin>>m.t_pos[i];
			/*			cout<<"\n\nSend bot to the position in given time(y/n) ?" ;
						char go;
						cin>>go;
			*/			if (i == 0)
							goto_movepose(mpinit, m.mp[0],m.t_pos[i], bot, comm);
						else
							goto_movepose(m.mp[i-1],m.mp[i],m.t_pos[i], bot, comm);
						i++;
					}
				break;
			case 2:	i--;
				break;
			case 3:	goto_movepose(m.mp[i-1],mpinit,t_default, bot, comm);

					m.no_of_pos = i;
					
					fstream file;
					file.open ("movestore.data", ios::out|ios::binary|ios::app);
					file.write((char*)&m,sizeof(m));
					file.close();
				break; 		
		}
					 
	}
}

int main()
{
/*	Imu imu;
	imu.init();
*/	
	Communication comm;
	AcYut bot(&comm,NULL);

	move_pose mpinit;
	setdefault(mpinit.hands[0],0,bot);
	setdefault(mpinit.hands[1],1,bot);
	setdefault(mpinit.legs[0],2,bot);
	setdefault(mpinit.legs[1],3,bot);

	int i = 1;
	while (i)
	{
		cout<<"0. End Program\n1. Create a position  2. Access position list\n3. Create move    4. Access Move List	\n";
		cin>>i;

		switch(i)
		{
			case 1: create_move_position(mpinit, bot, comm);
					break;
			case 2:	display_move_pose_list(mpinit, bot, comm);
					break;
			case 3: create_move(mpinit, bot, comm);
					break;
			case 4: access_move_list(mpinit, bot, comm);
					break;
			default: i = 0;
		}

	}

/*	// LEFT HAND
	float mot07[]={4096-2048,4096-2048,4096-2048,4096-2048,4096-2048,4096-2048,4096-2048};
	float mot08[]={4096-1100,4096-1100,4096-1100,4096-1100,4096-1100,4096-1100,4096-1100};
	float mot10[]={4096-2198,4096-2198,4096-2198,4096-2198,4096-2198,4096-2198,4096-2198};
	
	//RIGHT HAND
	float mot27[]={2048,3072,3072,3072,3072,2048,2048};
	float mot28[]={1100,2048,2048,2048,2048,1100,1100};
	float mot30[]={2048,2048,3072,2048,3072,2048,2048};

	//LEFT LEG
	float llegx[]={390,390,390,390,390,390,390};
	float llegy[]={0,0,0,0,0,0,0};
	float llegz[]={0,0,0,0,0,0,0};
	
	//RIGHT LEG
	float rlegx[]={390,390,390,390,390,390,390};
	float rlegy[]={0,0,0,0,0,0,0};
	float rlegz[]={0,0,0,0,0,0,0};
	
	//TIME
	float t1[]={0,1.5,2,3.5};
	float t=6.0;
	int fps=60;
	
	int vm7,vm8,vm10,vm27,vm28,vm30; 
	float vlx,vly,vlz,vrx,vry,vrz;
	
	int i;
	
	int t3;
	
	int arr_l[4],arr_r[4];
	
	for(t3=0;t3<360;t3+=1)
	{
		int j= (int)(t3/60.0);
		arr_l[0]=vm7=(int)scurve(mot07[j],mot07[j+1],t3%60, 60);
		arr_l[1]=vm8=(int)scurve(mot08[j],mot08[j+1],t3%60, 60);
		arr_l[1]=vm9=(int)scurve(mot09[j],mot09[j+1],t3%60, 60);
		arr_l[3]=vm10=(int)scurve(mot10[j],mot10[j+1],t3%60, 60);
		arr_r[0]=vm27=(int)scurve(mot27[j],mot27[j+1],t3%60, 60);
		arr_r[1]=vm28=(int)scurve(mot28[j],mot28[j+1],t3%60, 60);
		arr_r[3]=vm30=(int)scurve(mot30[j],mot30[j+1],t3%60, 60);
		vlx=scurve(llegx[j],llegx[j+1],t3%60, 60);
		vly=scurve(llegy[j],llegy[j+1],t3%60, 60);
		vlz=scurve(llegz[j],llegz[j+1],t3%60, 60);
		vrx=scurve(rlegx[j],rlegx[j+1],t3%60, 60);
		vry=scurve(rlegy[j],rlegy[j+1],t3%60, 60);
		vrz=scurve(rlegz[j],rlegz[j+1],t3%60, 60);
		arr_l[2]=arr_r[2]=0;
		bot.left_hand->setGoalPositionSync(arr_l);
		bot.right_hand->setGoalPositionSync(arr_r);
		bot.left_leg->runIK(vlx,vly,vlz,0);
		bot.left_leg->setGoalPositionSync();
		bot.right_leg->runIK(vrx,vry,vrz,0);
		bot.right_leg->setGoalPositionSync();
		comm.syncFlush();
		usleep(16666);
		printf("%d %d %d %d %d %d %f %f %f %f %f %f\n",vm7,vm8,vm10,vm27,vm28,vm30,vlx,vly,vlz,vrx,vry,vrz);
						
	}
*/	
	return 0;	
}	


