#ifndef FACE_DETECT
#define FACE_DETECT
#include <iostream>
#include <stdio.h>

//OpenCV Headers
#include <opencv/cv.h>
#include <opencv/highgui.h>
//Input-Output
#include <stdio.h>
//Blob Library Headers
#include <cvblob.h>
#include "../common/common.h"

#include <flycapture/FlyCapture2.h>
#include "camcapture.h"


using namespace std;
using namespace cvb;
using namespace cv;

// enum CamError {CAM_SUCCESS = 1, CAM_FAILURE = 0};

//CvScalar cColor = CV_RGB(255, 255, 255);

int detectAndDraw( Mat& img, CascadeClassifier& cascade,
                    CascadeClassifier& nestedCascade,
                    double scale, bool tryflip );
int faceDetect(CamCapture &capture);
#endif