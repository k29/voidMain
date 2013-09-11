#include <stdio.h>
#include <iostream>
#include "communication.h"
#include "motor.h"
using namespace std;
int main()
{
	Communication comm;
	int id;
	char option;
	int baud;
	cout<<"Want to set ID??(y/n)";
	cin>>option;
	if(option=='y')
	{
		cout<<"\nPacket is sent at broascast ID and hence if more than one motor id connected all motor will be set to same ID\nEnter id to be set to the motor ";
		cin>>id;
		Motor mset(MX64,id,&comm);
		mset.setID(id);
		cout<<"\nId set!! :D\n";
		return 0;
	}
	cout<<"Want to read ID?(y/n)";
	cin>>option;
	if(option=='y')
	{
		Motor mset(MX64,id,&comm);
		cout<<"\n";
		cout<<mset.getID();
		cout<<"\n";
		return 0;
	}
	else
	{
		printf("\nEnter ID\n");
		scanf("%d",&id);
		Motor m(MX64,id,&comm);
		int inp=0;
		while(inp!=7)
		{
			printf("\n1.Set Goal Position\n2.Get Goal Position\n3.Torque on\n4.Torque Off\n5.LedOn\n6.LedOff\n7.Set Buad\n8.Exit\n");
			scanf("%d",&inp);
			switch(inp)
			{
				case 1:printf("Enter goal Posn\n");
						int val;
						scanf("%d",&val);
						m.setGoalPosition(val);
						break;
				case 2:	m.getPresentPosition();
				
						break;
				case 3:m.torqueEnable(); break;
				
				case 4:m.torqueDisable(); break;
				
				case 5:m.ledOn(); break;
				case 6:m.ledOff(); break;
				case 7: 
				{
					cout<<"\nenter baud ";cin>>baud;
					m.setBaud(baud);
				} break;
				default: continue;
			}
		}
	}

	
	
	return 0;
}

