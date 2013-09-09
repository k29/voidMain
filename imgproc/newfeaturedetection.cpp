#include "featuredetection.h"
//NOTE: THIS FILE ASSUMES IN SEVERAL PLACES THAT rgbimg_small IS HALF THE SIZE OF rgbimg!!!!!!!!!!!!!!!!!!!
//1. Someplace in ballFind, i have multiplied ballX_var by 2 assuming main image is double the size of image used here
//2. in finding ballRatio, same assumption has been made
//3. in line detection etc. 1/4th of image used here is used(in the end assuming it was half of main image, as 1/8 of main image is needed)
using namespace cvb;
using namespace tbb;
using namespace LOCALIZE_INTERNALS;

FeatureDetection::FeatureDetection(CamCapture &cam): IMAGE_HEIGHT(cam.height_small()), IMAGE_WIDTH(cam.width_small())
{
	seg_red = cvCreateImage(cvSize(IMAGE_WIDTH, IMAGE_HEIGHT), 8, 1);
	seg_yellow = cvCreateImage(cvSize(IMAGE_WIDTH, IMAGE_HEIGHT), 8, 1);
	labelImg = cvCreateImage(cvSize(IMAGE_WIDTH, IMAGE_HEIGHT), IPL_DEPTH_LABEL, 1);
	labelImg_small = cvCreateImage(cvSize(IMAGE_WIDTH/4, IMAGE_HEIGHT/4), IPL_DEPTH_LABEL, 1);
	seg_white_count = cvCreateImage(cvSize(IMAGE_WIDTH/4, IMAGE_HEIGHT/4), 8, 1);
	seg_white = cvCreateImage(cvSize(IMAGE_WIDTH/4, IMAGE_HEIGHT/4), 8, 1);
	seg_black = cvCreateImage(cvSize(IMAGE_WIDTH/4, IMAGE_HEIGHT/4), 8, 1);
	seg_green = cvCreateImage(cvSize(IMAGE_WIDTH/4, IMAGE_HEIGHT/4), 8, 1);
	ballRatio=1.0;
	ballFound_var = false;
	tempnLand = 0;
	n_obstacles = 0;
	l.clear();
}



//TODO: re-write this function to make it readable and maybe more efficient.
//TODO: document how it works (if documenting somewhere else, give a link)
void FeatureDetection::findReal(int x,int y, float &objdis, float &objangdeg, HeadMotor &hm)
{
	float s=1,focal=533.33;
	float thetaX = hm.thetaX();
	float thetaY = hm.thetaY();
	thetaX += forwardTiltCorrection*(PI/180.);
	objdis=(((IMAGE_HEIGHT/2-y)+(focal/s)*tan(thetaX))/(1-(s/focal)*(IMAGE_HEIGHT/2-y)*tan(thetaX)));
	float perpend=(x-(IMAGE_WIDTH/2))*((s/focal)*(objdis)*sin(thetaX)+cos(thetaX))*pix2cm;
	objdis=pix2cm*(objdis-(focal/s)*tan(thetaX)) + (s_height+(neck_len/sin(thetaX)))*tan(thetaX);
	//printf("%f %f %f %f\n",perpend,objdis,rad2deg(atan2(perpend,objdis)),rad2deg(thetaY));
//	printf("thetaX is %f\t",rad2deg(thetaY));
	objangdeg=rad2deg(thetaY) - 150 + rad2deg(atan2(perpend,objdis));
	objdis=sqrt(objdis*objdis+perpend*perpend);
	
}



//For parallel segmenting
class SegmentImages {
	IplImage* my_seg_yellow;
	IplImage* my_seg_red;
	CamCapture* my_cam;
public:
    void operator()( const blocked_range2d<size_t>& r ) const {
        for( size_t x=r.rows().begin(); x!=r.rows().end(); ++x )
		{
            for( size_t y=r.cols().begin(); y!=r.cols().end(); ++y ) 
            {
            	if(my_cam->isYellow_small(x, y))
					returnPixel1C(my_seg_yellow, x, y) = 255;
				else
					returnPixel1C(my_seg_yellow, x, y) = 0;
				if(my_cam->isRed_small(x, y))
					returnPixel1C(my_seg_red, x, y) = 255;
				else
					returnPixel1C(my_seg_red, x, y) = 0;
            }
        }
    }
    SegmentImages(IplImage* &seg_yellow, IplImage* &seg_red, CamCapture &cam) :
        my_seg_yellow(seg_yellow), my_seg_red(seg_red), my_cam(&cam)
    {}
};



//TODO: Morphology operation being applied, so that spurious yellow pixels dont come
//Think of a way to remove that
void FeatureDetection::getBlobs(CamCapture &cam)
{
	parallel_for( blocked_range2d<size_t>(0, IMAGE_WIDTH, 16, 0, IMAGE_HEIGHT, 32),     
                  SegmentImages(seg_yellow,seg_red,cam) );
	IplConvKernel *morphkernel = cvCreateStructuringElementEx(3,3,0,0,CV_SHAPE_RECT);
	cvMorphologyEx(seg_yellow, seg_yellow, NULL, morphkernel, CV_MOP_OPEN, 1);
	cvLabel(seg_yellow, labelImg, blobs_yellow);
	cvLabel(seg_red, labelImg, blobs_red);
	cvLabel(seg_black, labelImg_small, blobs_black);
	cvFilterByArea(blobs_yellow, 100, 1000000);
	cvFilterByArea(blobs_red, 30, 1000000);
	cvFilterByArea(blobs_black, 5, 1000);
	int i = 0;

    // i= 0;
    // for (CvBlobs::const_iterator it=blobs_yellow.begin(); it!=blobs_yellow.end(); ++it, i++)
    // {
    //   std::cout << "Yellow #" << i << ": Area=" << it->second->area << ", Centroid=(" << it->second->centroid.x << ", " << it->second->centroid.y << ")" << std::endl;
    // }
}



//TODO: if opposite color blob found nearby, then it is not a goalpost
void FeatureDetection::getGoals(HeadMotor &hm)
{
	int nGoals = 0;
	for (CvBlobs::const_iterator it=blobs_yellow.begin(); it!=blobs_yellow.end(); ++it)
    {
    	if(((it->second->maxy - it->second->miny)/(it->second->maxx - it->second->minx)) >= 2)
    	{
    		//Check if on image edge
    		if(isOnImageEdge((it->second->maxx + it->second->minx)/2, it->second->maxy)==true)
    			continue;

    		printf("Found GP\n");
			findReal((it->second->maxx + it->second->minx)/2, it->second->maxy, templ[tempnLand].distance, templ[tempnLand].angle, hm);
			templ[tempnLand].type = LAND_GP;
			tempnLand++;
			nGoals++;
			if(nGoals>=4) break;
    	}
    }
}



struct Point
{
	int x;
	int y;
};

struct Node
{
	int degree;
	Point n[5];	//5 Maximum connections?
};

struct NodeXY
{
	int degree;
	Point n[5];	//5 Maximum connections?
	Point p;
};



bool FeatureDetection::checkSkeleton(int x, int y, int x1, int y1)
{
	for(int i = -1; i <2; i++)
	{
		for(int j = -1; j < 2; j++)
		{
			if((x1 + i > 0)&&(y1 + j > 0)&&(x1 + i < IMAGE_WIDTH/4)&&(y1 + j < IMAGE_HEIGHT/4)&&(i!=j||i!=0))
			{
				if(pixelColor1C(seg_white_count, x1+i, y1+j))
				{
					for(int i1 = -1; i1 <2; i1++)
					{
						for(int j1 = -1; j1 < 2; j1++)
						{
							if((x + i1 > 0)&&(y + j1 > 0)&&(x + i1 < IMAGE_WIDTH/4)&&(y + j1 < IMAGE_HEIGHT/4)&&(i1!=j1||i1!=0))
							{
								if(pixelColor1C(seg_white_count, x+i1, y+j1))
								{
									if((x1+i-i1-x==0||x1+i-i1-x==1||x1+i-i1-x==-1)&&(y1+j-j1-y==0||y1+j-j1-y==1||y1+j-j1-y==-1))
										return true;
								}
							}
						}
					}
				}
			}
		}
	}
	return false;
}


//Fine tuning of constants might be required
// void FeatureDetection::getCorners(CamCapture &cam, HeadMotor &hm)
// {
// 	//Assumption: getInGreen has been called, so 
// 	// seg_white_count stores white in green with densities
// 	// seg_white stores white in green with 255


	
// 	cvSmooth(wg_seg, wg_seg, CV_GAUSSIAN, 5);
// 	cvResize(wg_seg, wg_seg_show);
// 	cvShowImage("wg_seg2", wg_seg_show);

// 	IplImage* peak_image = cvCreateImage(cvSize(IMAGE_WIDTH/4, IMAGE_HEIGHT/4), 8, 1);
// 	IplImage* node_image = cvCreateImage(cvSize(IMAGE_WIDTH/4, IMAGE_HEIGHT/4), 8, 1);
// 	cvZero(node_image);
// 	cvZero(peak_image);
// 	//Getting peaks in image - gives skeleton
// 	for(int x = 0; x < IMAGE_WIDTH/4; x++)
// 	{
// 		for(int y = 0; y < IMAGE_HEIGHT/4; y++)
// 		{
// 			if(!pixelColor1C(wg_seg, x, y))
// 				continue;
// 			int temp = pixelColor1C(wg_seg, x, y);
// 			int count = 0;
// 			for(int i = -1; i <2; i++)
// 			{
// 				for(int j = -1; j < 2; j++)
// 				{
// 					if((x + i > 0)&&(y + j > 0)&&(x + i < IMAGE_WIDTH/4)&&(y + j < IMAGE_HEIGHT/4)&&(i!=j||i!=0))
// 					{
// 						if(returnPixel1C(wg_seg, x+i, y+j) >= temp)
// 							count++;
// 					}
// 				}
// 			}
// 			switch(count)
// 			{
// 				case 0:
// 				returnPixel1C(peak_image, x, y) = 255;
// 				break;
// 				case 1:
// 				returnPixel1C(peak_image, x, y) = 127;
// 				break;
// 				case 2:
// 				returnPixel1C(peak_image, x, y) = 63;
// 				break;
// 				default:
// 				returnPixel1C(wg_seg, x, y) = 0;
// 			}
// 		}
// 	}


// 	//Choosing nodes - all 0 ones, plus spaced 1, 2 ones

// 	for(int x = 0; x < IMAGE_WIDTH/4; x++)
// 	{
// 		for(int y = 0; y < IMAGE_HEIGHT/4; y++)
// 		{
// 			if(pixelColor1C(peak_image, x, y)!=255&&pixelColor1C(peak_image, x, y)!=0)
// 			{
// 				int flag = 0;
// 				for(int i = -1; i <2; i++)
// 				{
// 					for(int j = -1; j < 2; j++)
// 					{
// 						if((x + i > 0)&&(y + j > 0)&&(x + i < IMAGE_WIDTH/4)&&(y + j < IMAGE_HEIGHT/4)&&(i!=j||i!=0))
// 						{
// 							if(pixelColor1C(peak_image, x+i, y+j) == 255)
// 							{
// 								flag = 1;
// 								break;
// 							}
// 						}
// 					}
// 					if(flag)
// 						break;
// 				}
// 				if(flag)
// 					returnPixel1C(peak_image, x, y) = 0;
// 				else
// 					returnPixel1C(peak_image, x, y) = 255;
// 			}
// 		}
// 	}

	

// 	//peak_image now stores all nodes. skeleton along with its brightness is stored in wg_seg
// 	//Creating Node 2d array. Big so that it is easy to access particular node
// 	Node n[IMAGE_WIDTH/4][IMAGE_HEIGHT/4];
// 	NodeXY nList[1000];
// 	int nCount = 0;
// 	for(int x = 0; x < IMAGE_WIDTH/4; x++)
// 	{
// 		for(int y = 0; y < IMAGE_HEIGHT/4; y++)
// 		{
// 			if(pixelColor1C(peak_image, x, y)==255)
// 			{
// 				//Node found, now check for all connections
// 				nList[nCount].p.x = x;
// 				nList[nCount].p.y = y;
// 				nList[nCount].degree = 0;
// 				n[x][y].degree = 0;
// 				for(int i = -3; i <4; i++)
// 				{
// 					for(int j = -3; j < 4; j++)
// 					{
// 						if((i<=-2||i>=2||j<=-2||j>=2)&&((x + i > 0)&&(y + j > 0)&&(x + i < IMAGE_WIDTH/4)&&(y + j < IMAGE_HEIGHT/4)))
// 						{
// 							if(pixelColor1C(peak_image, x+i, y+j)==255)	//Node found in vicinity
// 							{
// 								if(checkSkeleton(x, y, x+i, y+j))
// 								{
// 									n[x][y].n[n[x][y].degree].x = x + i;
// 									n[x][y].n[n[x][y].degree].y = y + j;
// 									n[x][y].degree++;
// 									nList[nCount].n[nList[nCount].degree].x = x + i;
// 									nList[nCount].n[nList[nCount].degree].y = y + j;
// 									nList[nCount].degree++;
// 								}
// 							}
// 						}
// 					}
// 				}
// 				nCount++;
// 			}
// 		}
// 	}
	
// 	printf("%d Nodes\n", nCount);

// 	cvResize(peak_image, wg_seg_show);

// 	// //drawing connections in large peak_image

// 	for(int i = 0; i < nCount; i++)
// 	{
// 		for(int j = 0; j < nList[i].degree; j++)
// 		{
// 			cvLine(wg_seg_show, cvPoint(nList[i].p.x*8, nList[i].p.y*8), cvPoint(nList[i].n[j].x*8, nList[i].n[j].y*8), cvScalar(255),1);
// 		}
// 	}

// 	cvShowImage("peak_image", wg_seg_show);
// 	cvResize(wg_seg, wg_seg_show);
// 	for(int i = 0; i < nCount; i++)
// 	{
// 		for(int j = 0; j < nList[i].degree; j++)
// 		{
// 			cvLine(wg_seg_show, cvPoint(nList[i].p.x*8, nList[i].p.y*8), cvPoint(nList[i].n[j].x*8, nList[i].n[j].y*8), cvScalar(255),1);
// 		}
// 	}
// 	cvShowImage("wg_seg", wg_seg_show);
// 	cvReleaseImage(&peak_image);
// 	cvReleaseImage(&node_image);
// }



void FeatureDetection::getInGreen(CamCapture &cam)
{
	// Make reduced image
	int rowsum[IMAGE_HEIGHT/4];
	IplImage* binary_image = cvCreateImage(cvSize(IMAGE_WIDTH/4, IMAGE_HEIGHT/4), 8, 1);
	IplImage* prob_image = cvCreateImage(cvSize(IMAGE_WIDTH/4, IMAGE_HEIGHT/4), 8, 1);
	
	cvZero(seg_white);
	cvZero(seg_black);
	cvZero(seg_green);
	cvZero(seg_white_count);
 //New Begin
 	const int tmin = 27;
 	const int trow = IMAGE_WIDTH*18/4;
 	const int tsum = 36*18;
 	for(int y = 0; y < IMAGE_HEIGHT/4; y++)
 	{
 		rowsum[y] = 0;
 		for(int x = 0; x < IMAGE_WIDTH/4; x++)
 		{
 			int wcount = 0;
 			int gcount = 0;
 			int bcount = 0;
 			int rcount = 0;
 			for(int xx = 0; xx < 4; xx++)
 			for(int yy=0; yy < 4; yy++)
 			{
 				int tx = x*4 + xx;
 				int ty = y*4 + yy;
 				if(cam.isGreen_small(tx, ty))
 					gcount++;
 				if(cam.isWhite_small(tx, ty))
 					wcount++;
 				if(cam.isBlack_small(tx, ty))
 					bcount++;
 				if(cam.isRed_small(tx, ty))
 					rcount++;
 			}
 			returnPixel1C(prob_image, x, y) = wcount + gcount*2 + rcount*8 + bcount;
 			if(wcount)
 			{
 				if(wcount==16)
 					returnPixel1C(seg_white_count, x, y) = 255;
 				else	
 					returnPixel1C(seg_white_count, x, y) = (wcount*16)%256;
 				returnPixel1C(seg_white, x, y) = 255;
 			}
			else if(gcount>4)
 			{
 				returnPixel1C(seg_green, x, y) = 255;
 			}
			else if(bcount > 4)
 			{
 				returnPixel1C(seg_black, x, y) = 255;
 			}
 			else if(rcount)
 			{
			
 			}
			else
 			{
 	//			returnPixel1C(seg_black, x, y) = 255;
 			}
 			rowsum[y] += pixelColor1C(prob_image, x, y);
 		}
 	}
 	//New End
/*	const int tmin = 70;
	const int trow = IMAGE_WIDTH*20/4;
	const int tsum = 36*15;
	for(int y = 0; y < IMAGE_HEIGHT/4; y++)
	{
		rowsum[y] = 0;
		for(int x = 0; x < IMAGE_WIDTH/4; x++)
		{
			int wcount = 0;
			int gcount = 0;
			int bcount = 0;
			for(int xx = 0; xx < 4; xx++)
			for(int yy=0; yy < 4; yy++)
			{
				if(cam.isGreen_small(x*4 + xx, y*4 + yy))
					gcount++;
				else if(cam.isWhite_small(x*4 + xx, y*4 + yy))
					wcount++;
				else if(cam.isBlack_small(x*4 + xx, y*4 + yy))
					bcount++;
			}
			if(wcount)
			{
				returnPixel1C(seg_white, x, y) = 255;
				returnPixel1C(seg_white_count, x, y) = (wcount*16)%255;
				returnPixel1C(prob_image, x, y) = ((wcount*16)%255)>>1;
			}
			else if(gcount>4)
			{
				returnPixel1C(prob_image, x, y) = (gcount*16)%255;
				returnPixel1C(seg_green, x, y) = 255;
			}
			else if(bcount > 4)
			{
				returnPixel1C(prob_image, x, y) = (bcount*16)%255;	
				returnPixel1C(seg_black, x, y) = 255;
			}
			else
			{
				returnPixel1C(prob_image, x, y) = 0;
			}
			rowsum[y] += pixelColor1C(prob_image, x, y);
		}
	}
*/

	for(int y = 0; y < IMAGE_HEIGHT/4; y++)
	{
		for(int x = 0; x < IMAGE_WIDTH/4; x++)
		{
			//for each pixel, first check tmin
			if(pixelColor1C(prob_image, x, y) < tmin)
			{
				returnPixel1C(binary_image, x, y) = 0;
				continue;
			}

			//Now, check row sum is above certain threshold
			if(rowsum[y] > trow)
			{
				returnPixel1C(binary_image, x, y) = 255;
				continue;	
			}

			//Now do the expensive test
			int sum = 0;
			for(int i = -4; i <5; i++)
			{
				for(int j = -8; j < -4; j++)
				{
					if((x + i > 0)&&(y + j > 0)&&(x + i < IMAGE_WIDTH/4))
					{
						sum = sum + pixelColor1C(prob_image, x +i, y+j);
					}
				}
			}

			if(sum >= tsum)
			{
				returnPixel1C(binary_image, x, y) = 255;
				continue;	
			}
			else
			{
				returnPixel1C(binary_image, x, y) = 0;
				continue;
			}
		}
	}

	IplImage* histogram = cvCreateImage(cvSize(IMAGE_WIDTH/4, 1), 8, 1);

	for(int x = 0; x < IMAGE_WIDTH/4; x++)
	{
		int y;
		for(y = 0; y < IMAGE_HEIGHT/4; y++)
		{
			if(pixelColor1C(binary_image, x, y))
				break;
		}
		returnPixel1C(histogram, x, 0) = y;
	}

	cvSmooth(histogram, histogram, CV_GAUSSIAN, 3);
	cvSmooth(histogram, histogram, CV_MEDIAN, 3);

	for(int x = 0; x < IMAGE_WIDTH/4; x++)
	{
		for(int y = 0; y < returnPixel1C(histogram, x, 0); y++)
		{
			returnPixel1C(seg_white, x, y) = 0;
			returnPixel1C(seg_black, x, y) = 0;
			returnPixel1C(seg_white_count, x, y) = 0;
		}
	}

	//convex corner algo?


	IplImage* show_image = cvCreateImage(cvSize(IMAGE_WIDTH*2, IMAGE_HEIGHT*2), 8, 1);
	cvResize(seg_black, show_image);
	cvShowImage("Black", show_image);
	cvResize(seg_white, show_image);
	cvShowImage("White", show_image);
	cvResize(seg_green, show_image);
	cvShowImage("Green", show_image);
	cvResize(binary_image, show_image);
	cvShowImage("Binary", show_image);
	cvResize(prob_image, show_image);
	cvShowImage("Prob", show_image);
	cvReleaseImage(&show_image);

	//Segmentation done
	cvReleaseImage(&binary_image);
	cvReleaseImage(&prob_image);
	cvReleaseImage(&histogram);
}



void FeatureDetection::getLandmarks(CamCapture &cam, HeadMotor &hm)
{
	tempnLand = 0;
	//getInGreenKartik(cam);
	getInGreen(cam);
 	getBlobs(cam);
 	getGoals(hm);
 //	getCorners(cam, hm);
 	

 	for(std::vector<Landmark>::iterator it = l.begin(); it != l.end();)
 	{
 		it->counter++;
 		if(it->counter > LANDMARK_PERSISTENCE)
 		{
 			it = l.erase(it);
 		}
 		else
 		{
 			++it;
 		}
 	}
 	//Now assigning templ landmarks to main Landmark vector l
 	for(int i = 0; i < tempnLand; i++)
 	{
 		bool flag = false;

 		for(std::vector<Landmark>::iterator it = l.begin(); it != l.end(); ++it)
 		{
 			if(isSameLandmark(templ[i], *it)==true)
 			{
 				*it = templ[i];
 				it->counter = 0;
 				flag = true;
 				break;
 			}
 		}
 		if(flag==false)
 		{
 			//need to add new element now
 			templ[i].counter = 0;
 			l.push_back(templ[i]);
 		}
 	}

 	getBall(cam, hm);
 	int tempx = ballX_var - IMAGE_WIDTH;
 	int tempy = ballY_var - IMAGE_HEIGHT;
 	/*coordinates from center of image
		Ratio=(|x|/w + |y|/h)/2
		*/
	double t1 = (tempx>0?tempx:-tempx)/((double)IMAGE_WIDTH);
	double t2 = (tempy>0?tempy:-tempy)/((double)IMAGE_HEIGHT);
	ballRatio = t1>t2? t1: t2;

	for(std::vector<Landmark>::iterator it = l.begin(); it != l.end(); ++it)
 	{
 		printf("Type: %d Distance: %f Angle: %f Counter: %d\n",it->type, it->distance, it->angle, it->counter);
 	}
}



void FeatureDetection::getBall(CamCapture &cam, HeadMotor &hm)
{
	int area = 0;
	CvBlobs::const_iterator it2;
	//Select blob of largest area
	for (CvBlobs::const_iterator it=blobs_red.begin(); it!=blobs_red.end(); ++it)
    {
    	if((it->second->maxy - it->second->miny)/(it->second->maxx - it->second->minx)>=2) continue;
    	if(it->second->area > area)
    	{
    		area = it->second->area;
    		it2 = it;
    	}
    }
    if(area==0)	//Meaning no ball found
    {
    	ballFound_var = false;
    }
    else
    {
    	ballX_var = it2->second->centroid.x*2;
    	ballY_var = it2->second->centroid.y*2;
    	ballFound_var = true;
    	findReal(it2->second->centroid.x, it2->second->centroid.y, ball.r, ball.theta, hm);
    }
    //offset for ball is 6.0cm
    ball.r -= 6.0;
	/* Simple Begin */
	// int bx = 0, by = 0, count = 0;
	// for(int x = 0; x < IMAGE_WIDTH; x++)
	// {
	// 	for(int y = 0; y < IMAGE_HEIGHT; y++)
	// 	{
	// 		if(cam.isRed_small(x,y))
	// 		{
	// 			count++;
	// 			bx += x;
	// 			by += y;
	// 		}
	// 	}
	// }

	// printf("Count = %d\n", count);
	// if(count > 50)
	// {
	// 	bx = bx/count;
	// 	by = by/count;
	// 	ballX_var = bx*2;
	// 	ballY_var = by*2;
	// 	ballFound_var = true;
	// 	findReal(bx, by, ball.r, ball.theta, hm);
	// }
	// else
	// {
	// 	ballFound_var = false;
	// }
	/* Simple End */
}

void FeatureDetection::updatePacket(FeaturesPacket &fp)
{
	fp.n_obstacles = n_obstacles;
	for(int i = 0; i < n_obstacles; i++)
	{
		fp.obstacles[i] = obstacles[i];
	}

	fp.n_landmarks = l.size();
	for(int i = 0; i < l.size(); i++)
	{
		fp.landmarks[i] = l[i];
	}

	fp.ball = ball;
}