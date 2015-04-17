#include <cstdio>
#include "WorldState.h"
#include "Tools.h"
#include "MyBasicBehaviors.h"
#include "../../Xabsl/XabslEngine/XabslEngine.h"
#include "../common/common.h"
#include "../common/commondefs.h"
#include "opencv2/opencv.hpp"

using namespace std;

playerstate ACYUT;
MyErrorHandler myErrorHandler;
MotionModel motionModel;
xabsl::Engine* engine = new xabsl::Engine(myErrorHandler,&getCurrentSystemTime);

BasicBehaviorInitialize basicBehaviorInitialize(myErrorHandler,ACYUT);
BasicBehaviorPrint basicBehaviorPrint(myErrorHandler,ACYUT);
BasicBehaviorUpdate basicBehaviorUpdate(myErrorHandler,ACYUT);
BasicBehaviormoveAcYuttemp basicBehaviormoveAcYuttemp(myErrorHandler,ACYUT);
BasicBehaviorLocalize basicBehaviorLocalize(myErrorHandler,ACYUT);
BasicBehaviorRotate basicBehaviorRotate(myErrorHandler,ACYUT);
BasicBehaviorPathToWalk basicBehaviorPathToWalk(myErrorHandler,ACYUT);
BasicBehaviorMakePath basicBehaviorMakePath(myErrorHandler,ACYUT);
// BasicBehaviorFindBall basicBehaviorFindBall(myErrorHandler,ACYUT);
BasicBehaviorReset basicBehaviorReset(myErrorHandler,ACYUT);
BasicBehaviorMakePathFromMotionModel basicBehaviorMakePathFromMotionModel(myErrorHandler,ACYUT);
BasicBehaviorGoalKeep basicBehaviorGoalKeep(myErrorHandler,ACYUT);
BasicBehaviorDoOrient basicBehaviorDoOrient(myErrorHandler,ACYUT);

void registerXABSL()
{
    engine->registerEnumeratedInputSymbol("localizationState", "LocalizationState", (int*)&ACYUT.localizationState);
    engine->registerEnumElement("LocalizationState", "LocalizationState.CRITICAL",CRITICAL);
    engine->registerEnumElement("LocalizationState", "LocalizationState.LOCALIZED",LOCALIZED);
    engine->registerEnumElement("LocalizationState", "LocalizationState.MOTIONMODEL",MOTIONMODEL);


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
    engine->registerEnumElement("PathReturns", "PathReturns.DOORIENT",DOORIENT);

    engine->registerDecimalInputSymbol("theta", &playerstate::getTheta);
    engine->registerDecimalInputSymbol("confidence", &playerstate::getConfidence);
    engine->registerDecimalInputSymbol("resetflag", &playerstate::getReset);
    engine->registerDecimalInputSymbol("RoboCup.state",&playerstate::getRoboCupState);
    engine->registerDecimalInputSymbol("ball.distance",&playerstate::getDistance);
    engine->registerDecimalInputSymbol("goalkeepflag", &playerstate::getGoalKeepFlag);

    engine->registerBasicBehavior(basicBehaviorInitialize);
    engine->registerBasicBehavior(basicBehaviorLocalize);
    engine->registerBasicBehavior(basicBehaviorRotate);
    engine->registerBasicBehavior(basicBehaviorPathToWalk);
    engine->registerBasicBehavior(basicBehaviorPrint);
    engine->registerBasicBehavior(basicBehaviorUpdate);
    engine->registerBasicBehavior(basicBehaviormoveAcYuttemp);
    engine->registerBasicBehavior(basicBehaviorMakePath);
    // engine->registerBasicBehavior(basicBehaviorFindBall);
    engine->registerBasicBehavior(basicBehaviorReset);
    engine->registerBasicBehavior(basicBehaviorMakePathFromMotionModel);
    engine->registerBasicBehavior(basicBehaviorGoalKeep);

    MyFileInputSource input("intermediate-code.dat");
    engine->createOptionGraph(input);   
}

void start()
{
    ACYUT.resetflag=1; /*Resets all variables in the first run */
    ACYUT.ACTIVE_GOAL=0;

    #ifndef GC_IS_ON 

        ACYUT.GCData.state=STATE_PLAYING;
        while(1)
            {
                //printf("started without GC");
                
                engine->execute();
                
                ACYUT.resetflag=0;
            }

    #endif

    #ifdef GC_IS_ON
        
        while(1)
            {
                // printf("started with GC");
                
                pthread_mutex_lock(&mutex_GCData);
                    ACYUT.GCData=GCData;
                pthread_mutex_unlock(&mutex_GCData);
            
                engine->execute();
            
                ACYUT.resetflag=0;
            }

    #endif
}

int getImuAngle()
{        
        #ifdef IMU_IS_ON
            return -(imu.yaw-IMU_INITIAL_ANGLE);
        #endif

        #ifndef IMU_IS_ON
            return -9999;
        #endif
}

void callBackFunc(int event, int x, int y, int flags, void* userdata)
{
    AbsCoords* goal = (AbsCoords*) userdata;

    if(event == cv::EVENT_LBUTTONDOWN)
    {
        goal->x = x - 250.0;
        goal->y = y - 250.0;
    }
    if(event == cv::EVENT_RBUTTONDOWN)
    {
        goal->x = x - 250.0;
        goal->y = y - 250.0;
    }
    if(event == cv::EVENT_MBUTTONDOWN)
    {
        goal->x = x - 250.0;
        goal->y = y - 250.0;
    }
}