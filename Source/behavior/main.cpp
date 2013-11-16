#include <cstdio>
#include "WorldState.h"
#include "Tools.h"
#include "MyBasicBehaviors.h"
#include "../../Xabsl/XabslEngine/XabslEngine.h"
#include "../common/common.h"
#include "../common/commondefs.h"

using namespace std;

playerstate ACYUT;
MyErrorHandler myErrorHandler;
xabsl::Engine* engine = new xabsl::Engine(myErrorHandler,&getCurrentSystemTime);

BasicBehaviorInitialize basicBehaviorInitialize(myErrorHandler,ACYUT);
BasicBehaviorPrint basicBehaviorPrint(myErrorHandler,ACYUT);
BasicBehaviorUpdate basicBehaviorUpdate(myErrorHandler,ACYUT);
BasicBehaviormoveAcYuttemp basicBehaviormoveAcYuttemp(myErrorHandler,ACYUT);
BasicBehaviorLocalize basicBehaviorLocalize(myErrorHandler,ACYUT);
BasicBehaviorPathToWalk basicBehaviorPathToWalk(myErrorHandler,ACYUT);
BasicBehaviorMakePath basicBehaviorMakePath(myErrorHandler,ACYUT);
BasicBehaviorFindBall basicBehaviorFindBall(myErrorHandler,ACYUT);



void registerXABSL()
{
        

        engine->registerEnumeratedInputSymbol("ballreturn", "BallReturns", (int*)&ACYUT.ballreturn);
        engine->registerEnumElement("BallReturns", "BallReturns.BALLFOUND",BALLFOUND);
        engine->registerEnumElement("BallReturns", "BallReturns.BALLFINDING",BALLFINDING);
        engine->registerEnumElement("BallReturns", "BallReturns.TURNRIGHT",TURNRIGHT);
        engine->registerEnumElement("BallReturns", "BallReturns.TURNLEFT",TURNLEFT);


        engine->registerEnumeratedInputSymbol("pathreturn", "PathReturns", (int*)&ACYUT.pathreturn);
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
        



        MyFileInputSource input("intermediate-code.dat");
    engine->createOptionGraph(input);
    
}

void start()
        {
        ACYUT.resetflag=1; /*Resets all variables in the first run */
        

        while(1)
                {
                        printf("start();");
                        #ifndef GC_IS_ON                
                        ACYUT.GCData.state=STATE_PLAYING;
                        #endif
                        #ifdef GC_IS_ON
                        pthread_mutex_lock(&mutex_GCData);
            ACYUT.GCData=GCData;
            pthread_mutex_unlock(&mutex_GCData);
                        #endif
                        //printf("State passed is %lf\n",ACYUT.getRoboCupState());
                        //ACYUT.ACTIVE_GOAL=1;
                        engine->execute();
                        
                        ACYUT.resetflag=0;
                }
        
        }
int getImuAngle()
{
                fstream f1;
                f1.open("cpp-src/xsens/imuyaw.angle", ios::in);
                int value;
                f1>>value;
                printf("Value is %d\n", value);
                f1.close();
                return value;
}