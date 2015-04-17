#include "MyBasicBehaviors.h"


#ifdef PATH_MOUSE_CLICK_ON
AbsCoords goal_pos;
#endif

void BasicBehaviorPrint::execute()
{
    #ifdef XABSL_PRINTING_IS_ON
    printf("\nu asked for %lf\n",o);
    #endif
}

void BasicBehaviorInitialize::execute()
{
    // printf("BasicBehaviorInitialize\n");
    #ifdef IP_IS_ON
    printf("Initializing\n");
    

    p.hdmtr.bootup_files();
    

    while(!p.capture.init())
            {
                usleep(500000);
                continue;
            }
    p.globalflags.reset();

    p.confidence=0;
    motionModel.confidence=0;
    goal_pos.x = 200.0;
    goal_pos.y = 200.0;

    #ifdef GOAL_KEEPER_MODE
    p.GOAL_KEEPER_FLAG = true;
    #endif

    printf("Initialized\n");
    #endif
}
void BasicBehaviorUpdate::execute()
{
        // printf("BasicBehaviorUpdate\n");
        #ifdef IP_IS_ON

        // printf("Entered update\n");       
        // p.hdmtr.update();
        p.capture.getImage();
        p.fd->getLandmarks(p.capture, p.hdmtr, motionModel);
        p.loc.doLocalize(*p.fd, motionModel, p.capture, getImuAngle());

        ///////////////REFINE RELOCALISATION///////////////
        AbsCoords goalcoords=p.loc.getGoalCoords(p.ACTIVE_GOAL);
        double tempx=goalcoords.x-p.loc.selfX;
        double tempy=goalcoords.y-p.loc.selfY;
        double x = (tempx*cos(deg2rad(p.loc.selfAngle))) - (tempy* sin(deg2rad(p.loc.selfAngle)));//Rotating coordinate system.
        double y = (tempx*sin(deg2rad(p.loc.selfAngle))) + (tempy* cos(deg2rad(p.loc.selfAngle)));
        // printf("%d %d\n", tempx, tempy);
        // if(sqrt(x*x + y*y) > 200)
        // {
        //     p.loc.randomize();
        // }

        /* Set flags for XABSL */


        /* localizationState flag */
        #ifndef PATH_MOUSE_CLICK_ON
        pthread_mutex_lock(&mutex_motionModel);

        p.confidence = ALPHA*p.loc.confidence + BETA*motionModel.confidence;

        if(p.confidence >= 0.2)
        {
            p.localizationState = LOCALIZED;
            p.timesteps = 0;
            pthread_mutex_lock(&mutex_rotate);
            DO_ROTATE = false;
            pthread_mutex_unlock(&mutex_rotate);
        }
        else if(p.loc.confidence >= 0.2 && p.loc.confidence < 5.0)
        {
            p.localizationState = LOCALIZED;
            motionModel.refresh(p.loc.selfX, p.loc.selfY, p.loc.confidence);
            p.timesteps = 0;
            pthread_mutex_lock(&mutex_rotate);
            DO_ROTATE = false;
            pthread_mutex_unlock(&mutex_rotate);
        }
        else
        {
            p.localizationState = CRITICAL;
            p.timesteps++;
        }
                    
        // if(motionModel.confidence < p.loc.confidence && p.loc.confidence<5.0)
        //     motionModel.refresh(p.loc.selfX,p.loc.selfY,p.loc.confidence);
        
        // p.confidence=max( motionModel.confidence , p.loc.confidence);

        
        // if(p.confidence < 0.2)
        // {
        //     p.localizationState=CRITICAL;
        //     p.timesteps++;
        // }
        // // else if(motionModel.confidence > p.loc.confidence)
        // //     p.localizationState=MOTIONMODEL;
        //     // p.localizationState=LOCALIZED;
        // else
        // {
        //     p.localizationState=LOCALIZED;
        //     p.timesteps = 0;
        // }

        if(p.timesteps > 20)
        {
            p.loc.randomize();
            p.timesteps = 0;
        }
        pthread_mutex_unlock(&mutex_motionModel);
        #endif
        /* localizationState flag end */
        #ifdef PATH_MOUSE_CLICK_ON
            p.confidence = 1.0;
            p.localizationState = LOCALIZED;
        #endif


        /* ball found flag */     
        p.ballreturn=p.camcont->findBall(*(p.fd),p.hdmtr);   
        /* ball found flag end */


        IplImage* flags = cvCreateImage(cvSize(220,75),8,1);
        cvZero(flags);
        CvFont font;

        double theta=getImuAngle();
        
        cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, 0.35, 0.35, 0, 1.5, 8);
        char A[100],B[100],C[100],D[100];
        if(p.localizationState == CRITICAL)
            sprintf(A,"LOCALISATION STATE : CRITICAL");
        if(p.localizationState == MOTIONMODEL)
            sprintf(A,"LOCALISATION STATE : MOTIONMODEL");
        if(p.localizationState == LOCALIZED)
            sprintf(A,"LOCALISATION STATE : LOCALIZED");
        sprintf(B,"CONFIDENCE : %lf",p.confidence);

        sprintf(C,"IMU ANGLE : %lf",theta);

        sprintf(D,"BALL: %s",p.ballreturn==BALLFOUND?"FOUND":"NOT FOUND");
        
        cvPutText(flags,A,cvPoint(10,15),&font,cvScalar(255,255,255));
        cvPutText(flags,B,cvPoint(10,30),&font,cvScalar(255,255,255));
        cvPutText(flags,C,cvPoint(10,45),&font,cvScalar(255,255,255));
        cvPutText(flags,D,cvPoint(10,60),&font,cvScalar(255,255,255));

        // printf("localization updated to %lf\n",p.conf);
        cvNamedWindow("Flags");
        cvNamedWindow("Real Time Feed");
        cvNamedWindow("Localization");
        #ifdef INTEL_BOARD_DISPLAY
        cvMoveWindow("Flags",20,30);
        cvMoveWindow("Real Time Feed",240,30);
        cvMoveWindow("Localization",850,30);
        #endif
        cvMoveWindow("Flags",50,50);
        cvMoveWindow("Real Time Feed",300,50);
        cvMoveWindow("Localization",950,50);
        cvShowImage("Flags",flags);
        cvShowImage("Real Time Feed", p.capture.rgbimg);
        cvShowImage("Localization", p.loc.dispImage);
        int c = cvWaitKey(25);
        if(c == 'S' || c == 's')
            if(cvSaveImage("image.bmp", p.capture.rgbimg))
                printf("saved\n");
        cvReleaseImage(&flags);
        if(c == 27)
            exit(0);
        #endif

        #ifndef IP_IS_ON
        p.confidence=1;
        p.ballreturn=BALLFOUND;
        #endif
}

void BasicBehaviorRotate::execute()
{
    printf("BasicBehaviorRotate\n");
    pthread_mutex_lock(&mutex_pathpacket);
    pathpackvar.no_of_points=1;
    pathpackvar.updated=1;
    pathpackvar.pathType=1;
    pathpackvar.finalpath[0].x=0.0;
    pathpackvar.finalpath[0].y=deg2rad(5);
    pthread_mutex_unlock(&mutex_pathpacket);
}
void BasicBehaviorLocalize::execute()
{   
        
        // #ifdef IP_IS_ON
        // // printf("Confidence %lf, localizing\n",p.confidence);
        // int i=50;
        // while(i--)
        // {
        
        
        // // p.hdmtr.update();
        
        // // while(!p.capture.getImage())
        // //     {
        // //         continue;
        // //     }
        // // {
        // //     printf("worked\n");
        // //     continue;
        // // }
        //     // printf("After capture\n");
        // p.capture.getImage();
        // p.fd->getLandmarks(p.capture, p.hdmtr, motionModel);
        // // printf("After getLandmarks\n");
        // // p.camcont->search(p.hdmtr);
        // p.loc.doLocalize(*p.fd, motionModel, p.capture, getImuAngle()); 
        // cvShowImage("Real Time Feed", p.capture.rgbimg);
        // cvShowImage("Localization", p.loc.dispImage);
    
        // p.confidence = p.loc.confidence;
        // // printf("%lf\n, %d", p.conf, i);
        // cvWaitKey(5);
        // }
        // #endif

        // #ifndef IP_IS_ON
        // p.conf=1;
        // #endif
    
        // pthread_mutex_lock(&mutex_pathpacket);
        // pathpackvar.no_of_points=1;
        // pathpackvar.updated=1;
        // pathpackvar.pathType=1;
        // pathpackvar.finalpath[0].x=0.005; /* customary */
        // pathpackvar.finalpath[0].y=deg2rad(5);
        // pthread_mutex_unlock(&mutex_pathpacket);

}
void BasicBehaviormoveAcYuttemp::execute()
{
    unsigned int _x=int(x);
    // printf("BasicBehaviormoveAcYuttemp\n");
    pthread_mutex_lock(&mutex_walkstr);    
        // printf("to walk %c\n",_x);
        walkstr.instr = _x;
        walkstr.isFresh = true;
    pthread_mutex_unlock(&mutex_walkstr);
        
}

void BasicBehaviorMakePath::execute()
{
    //      p.pathstr.n_obstacles = 2;
    //     p.pathstr.absObstacles[0].x=20; 
    //     p.pathstr.absObstacles[0].y=31;
    //     p.pathstr.absObstacles[1].x=60 ;
    //     p.pathstr.absObstacles[1].y=100;
    //     p.pathstr.absObstacles[2].x=-40;
    //     p.pathstr.absObstacles[2].y=-80;
    // printf("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
    //     for(int i=0;i<fd.o.size();i++)//assuming right positive and left neagative for theta
    // {
    //     p.pathstr.absObstacles[i].x=(p.fd->o[i].distance)*cos(deg2rad(p.fd->o[i].angle));
    //     p.pathstr.absObstacles[i].y=(p.fd->o[i].distance)*sin(deg2rad(p.fd->o[i].angle));
    // }

    // for (int i = 0; i < pathstr.n_obstacles; ++i)
    // {
    //     printf("Passed-->> obstacle %d : %lf %lf\n", i, p.pathstr.absObstacles[i].x, p.pathstr.absObstacles[i].y);
    // }
    // printf("BasicBehaviorMakePath\n");
    #ifdef IP_IS_ON
    
    #ifndef PATH_MOUSE_CLICK_ON
    AbsCoords goalcoords=p.loc.getGoalCoords(p.ACTIVE_GOAL);
    AbsCoords selfm = motionModel.read();
    double selfx = ALPHA*p.loc.selfX + BETA*selfm.x;
    double selfy = ALPHA*p.loc.selfY + BETA*selfm.y;
    selfx /= (ALPHA + BETA);
    selfy /= (ALPHA + BETA);
    double tempx = goalcoords.x - selfx;
    double tempy = goalcoords.y - selfy;
    // double tempx=goalcoords.x-p.loc.selfX;
    // double tempy=goalcoords.y-p.loc.selfY;
    p.pathstr.goal.x= (tempx*cos(deg2rad(p.loc.selfAngle))) - (tempy* sin(deg2rad(p.loc.selfAngle)));//Rotating coordinate system.
    p.pathstr.goal.y= (tempx*sin(deg2rad(p.loc.selfAngle))) + (tempy* cos(deg2rad(p.loc.selfAngle)));
    #endif
    #ifdef PATH_MOUSE_CLICK_ON
    cv::setMouseCallback("Field", callBackFunc, &goal_pos);
    p.pathstr.goal.x = goal_pos.x;
    p.pathstr.goal.y = goal_pos.y;
    #endif
    // printf("Passed:-->>>>goal coords x:%lf  y:%lf\n",p.pathstr.goal.x,p.pathstr.goal.y);
    //printf("goal coords y:%lf\n",pathstr.goal.x);
    p.pathstr.ball.x=p.fd->ball.r*cos(deg2rad(p.fd->ball.theta));
    p.pathstr.ball.y=p.fd->ball.r*sin(deg2rad(p.fd->ball.theta));

    //OBSTACLE DETECTION
    p.pathstr.n_obstacles = p.fd->o.size();
    for (int i = 0; i < p.fd->o.size(); ++i)
    {
        p.pathstr.absObstacles[i].x = p.fd->o[i].distance*cos(deg2rad(p.fd->o[i].angle));
        p.pathstr.absObstacles[i].y = p.fd->o[i].distance*sin(deg2rad(p.fd->o[i].angle));
    }
    
    // printf("relative ball----> %f  %f\n",p.fd->ball.r,p.fd->ball.theta);
    // printf("Passed:-->>>>ball coords x:%lf  y:%lf\n",p.pathstr.ball.x,p.pathstr.ball.y);

    p.pathreturn=p.path.path_return(p.pathstr);
    // printf("before crash\n");
    if(p.path.tree.path_crash)
    {
        // printf("path crashed\n");
        p.path.tree.path_crash = false;
        cvZero(p.path.image);
    }
    // printf("Path Made\n");
    #endif
}


void BasicBehaviorMakePathFromMotionModel::execute()
{
        
    // printf("BasicBehaviorMakePathFromMotionModel\n");
    #ifdef IP_IS_ON
    
    AbsCoords self;
    AbsCoords goalcoords=p.loc.getGoalCoords(p.ACTIVE_GOAL);
    self=motionModel.read();

    double tempx=goalcoords.x-self.x;
    double tempy=goalcoords.y-self.y;
    double theta=getImuAngle(); // add headmotor considerations too eventually
    // double theta=p.loc.selfAngle;

    p.pathstr.goal.x= (tempx*cos(deg2rad(theta))) - (tempy* sin(deg2rad(theta)));//Rotating coordinate system.
    p.pathstr.goal.y= (tempx*sin(deg2rad(theta))) + (tempy* cos(deg2rad(theta)));
    // printf("Passed:-->>>>goal coords x:%lf  y:%lf\n",p.pathstr.goal.x,p.pathstr.goal.y);

    //printf("goal coords y:%lf\n",pathstr.goal.x);
    p.pathstr.ball.x=p.fd->ball.r*cos(deg2rad(p.fd->ball.theta));
    p.pathstr.ball.y=p.fd->ball.r*sin(deg2rad(p.fd->ball.theta));
    
    // printf("relative ball----> %f  %f\n",p.fd->ball.r,p.fd->ball.theta);
    // printf("Passed:-->>>>ball coords x:%lf  y:%lf\n",p.pathstr.ball.x,p.pathstr.ball.y);

    p.pathreturn=p.path.path_return(p.pathstr);
    
    // printf("Path Made\n");
    #endif
}


void BasicBehaviorPathToWalk::execute()
{
    // printf("BasicBehaviorPathToWalk\n");
        #ifdef IP_IS_ON
        #ifdef WALK_IS_ON
    
        p.path.updatePathPacket();
        if(p.path.tree.path_crash)
        {
            // printf("path crashed\n");
            p.path.tree.path_crash = false;
        }
        
        // printf("Path updated\n");
        #endif
        #endif

    #ifndef IP_IS_ON

    
    //printf("UPDATING PATHPACKVAR\n");
    

    fstream fil1;
    fil1.open("Source/path/path.data", ios::in|ios::binary);
    fil1.read((char*)&pathpackvar,sizeof(pathpackvar));
    fil1.close();
    

    #endif
}

void BasicBehaviorFindBall::execute()
{    
   
}

void BasicBehaviorReset::execute()
{
    // printf("BasicBehaviorReset\n");
    p.confidence=0;        
}

void BasicBehaviorGoalKeep::execute()
{
    // printf("BasicBehaviorGoalKeep\n");
    GoalKeeperAction ret = p.gk.keeperUpdate(p.capture, p.hdmtr, p.camcont, p.fd, motionModel);
    if(ret == STAY)
        printf("STAY\n");
    if(ret == FALLLEFT)
        printf("FALLLEFT\n");
    if(ret == FALLRIGHT)
        printf("FALLRIGHT\n");
    cvNamedWindow("Real Time Feed");
    cvMoveWindow("Real Time Feed",300,50);
    cvShowImage("Real Time Feed", p.capture.rgbimg);
    cvWaitKey(25);
}