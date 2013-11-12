#include <cstdio>
#include "WorldState.h"
#include "Tools.h"
#include "MyBasicBehaviors.h"
#include "../../Xabsl/XabslEngine/XabslEngine.h"
#include "../common/common.h"
#include "../common/commondefs.h"

using namespace std;

playerstate p;
MyErrorHandler myErrorHandler;
xabsl::Engine* engine = new xabsl::Engine(myErrorHandler,&getCurrentSystemTime);

BasicBehaviorInitialize basicBehaviorInitialize(myErrorHandler,p);
BasicBehaviorPrint basicBehaviorPrint(myErrorHandler,p);
BasicBehaviorUpdate basicBehaviorUpdate(myErrorHandler,p);
BasicBehaviormoveAcYuttemp basicBehaviormoveAcYuttemp(myErrorHandler,p);
BasicBehaviorLocalize basicBehaviorLocalize(myErrorHandler,p);
BasicBehaviorPathToWalk basicBehaviorPathToWalk(myErrorHandler,p);
BasicBehaviorMakePath basicBehaviorMakePath(myErrorHandler,p);
BasicBehaviorFindBall basicBehaviorFindBall(myErrorHandler,p);
BasicBehaviorReset basicBehaviorReset(myErrorHandler,p);


void registerXABSL()
{
	

	engine->registerEnumeratedInputSymbol("ballreturn", "BallReturns", (int*)&p.ballreturn);
	engine->registerEnumElement("BallReturns", "BallReturns.BALLFOUND",BALLFOUND);
	engine->registerEnumElement("BallReturns", "BallReturns.BALLFINDING",BALLFINDING);
	engine->registerEnumElement("BallReturns", "BallReturns.TURNRIGHT",TURNRIGHT);
	engine->registerEnumElement("BallReturns", "BallReturns.TURNLEFT",TURNLEFT);


	engine->registerEnumeratedInputSymbol("pathreturn", "PathReturns", (int*)&p.pathreturn);
	engine->registerEnumElement("PathReturns", "PathReturns.DOWALK",DOWALK);
	engine->registerEnumElement("PathReturns", "PathReturns.DOKICK",DOKICK);
	engine->registerEnumElement("PathReturns", "PathReturns.DOENCIRCLE",DOENCIRCLE);
	engine->registerEnumElement("PathReturns", "PathReturns.NOPATH",NOPATH);

	// engine->registerEnumeratedInputSymbol("init.state", "init.state"nt*)&p.initstate);
	// engine->registerEnumElement("init.state", "init.state.DONE",DONE);
	// engine->registerEnumElement("init.state", "init.state.NOTDONE",NOTDONE);


	engine->registerDecimalInputSymbol("theta", &playerstate::getTheta);
	engine->registerDecimalInputSymbol("confidence", &playerstate::getConfidence);
	engine->registerDecimalInputSymbol("resetflag", &playerstate::getReset);
	engine->registerDecimalInputSymbol("RoboCup.state",&playerstate::getRoboCupState);
	engine->registerDecimalInputSymbol("ball.distance",&playerstate::getDistance);


	engine->registerBasicBehavior(basicBehaviorInitialize);
	engine->registerBasicBehavior(basicBehaviorLocalize);
	engine->registerBasicBehavior(basicBehaviorPathToWalk);
	engine->registerBasicBehavior(basicBehaviorPrint);
	engine->registerBasicBehavior(basicBehaviorUpdate);
	engine->registerBasicBehavior(basicBehaviormoveAcYuttemp);
	engine->registerBasicBehavior(basicBehaviorMakePath);
	engine->registerBasicBehavior(basicBehaviorFindBall);
	engine->registerBasicBehavior(basicBehaviorReset);	



	MyFileInputSource input("intermediate-code.dat");
    engine->createOptionGraph(input);
    
}

void start()
	{
	p.resetflag=1; /*Resets all variables in the first run */
	

	while(1)
		{
				printf("start();");
			/*Get GC data from GC thread */
			// pthread_mutex_lock(&mutex_GCData);
   //          p.localgcdata=GCData;
   //          pthread_mutex_unlock(&mutex_GCData);
   //          p.GC.update(p.localgcdata);
	//		printf("Going to execute engine ");		
			p.localgcdata.state=STATE_PLAYING;
			engine->execute();
	//		printf("Executed");
			p.resetflag=0;
		}
	
	}
int getImuAngle()
{
		fstream f1;
		f1.open("cpp-src/xsens/imuyaw.angle", ios::in);
		int value;
		f1>>value;
		printf("Value is %d\n", 1280);
		f1.close();
		return 1280;
}
