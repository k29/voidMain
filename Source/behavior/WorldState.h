#ifndef WSTATE_H
#define WSTATE_H

#include "../../Xabsl/XabslEngine/XabslSymbols.h"

#include "../imgProc/camcapture.h"
#include "../imgProc/camcontrol.h"
#include "../imgProc/featuredetection.h"
#include "../imgProc/headmotor.h"
#include "../imgProc/localize.h"
#include "../imgProc/goalkeeper.h"
#include "../path/path.hpp"
#include "../common/common.h"
#include "../common/commondefs.h"
#include "motionmodel.h"

#ifdef SEGWAY_MODE
#include "../testwalk/commondefswalk.h"
#endif
#ifndef SEGWAY_MODE
#include "../walk/commondefswalk.h"
#endif


class playerstate

	{
	    public:


                        /* To register XABSL input symbols */
                        static double getTheta();
                        static double getConfidence();
                        static playerstate* theInstance;
			static double getRoboCupState();
                        static double getDistance();  
                        static double getGoalKeepFlag();    

                        /* If true resets all XABSL parameters */
                        int resetflag;
                        static double getReset();

                        playerstate();
                
                        BallReturns ballreturn;
                        LocalizationState localizationState;
                        
                        double ACTIVE_GOAL;
		       

                        Flags globalflags;
		        // MotionModel motionModel; //shifted to global so walkthread can access it. 
		        //InitState initstate;
        
                        /* Image */ 
                        CamCapture capture;
                        FeatureDetection *fd;
                        HeadMotor hdmtr;
                        CamControl *camcont;
                        GoalKeeper gk;
                
       

        Localize loc;
        // float bwt,lmwt;
        int ballnotinframe;
        
        
        // float prevballr,velocity;
        
        

        

        /* Path Variables */
        Path path;
        PathStructure pathstr; 
        PathReturns pathreturn;
        PathReturns pathreturnLast;

        /*Game Controller */
        GameControllerAcyut GC;
        RoboCupGameControlData GCData;

        
        double localizationConfidence;
        double confidence;
        int timesteps;
        bool GOAL_KEEPER_FLAG;         
                        



	};


#endif
