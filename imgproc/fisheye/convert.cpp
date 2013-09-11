#include <stdio.h>
#include <iostream>
#include <math.h>

#include <cvblob.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>

using namespace std;
using namespace cvb;
using namespace cv;

// #define HEIGHT 480
// #define WIDTH 752
#define PI 3.14159265359

int HEIGHTf, WIDTHf, HEIGHTl, WIDTHl, WIDTHu, HEIGHTu;


double factor1 = -6.4e-06;
double factor2 = -3.1e-06;//-0.0002*0.9;
void func0(int xd, int yd, int* xu, int* yu )
{
	double id,jd,iu,ju;
	double ax = factor1, ay = factor2;
	id = (double)xd - WIDTHf/2;
	jd = (double)-yd + HEIGHTf/2;
	double r2 = id*id + jd*jd;
	// double px = factor1;
	// double py = factor2;

	// iu = id*(1 - px*r)/(1 - px);
	// ju = jd*(1 - py*r)/(1 - py);
	iu = id/(1+ax*r2);
	ju = jd/(1+ax*r2);

 	*xu = iu + WIDTHu/2;
 	*yu = -ju + HEIGHTu/2;

}

void func1(int xu, int yu, int* xd, int* yd )
{
	double id,jd,iu,ju;
	iu = (double)(2*xu - WIDTHu)/WIDTHu;
	ju = (double)(2*-yu + HEIGHTu)/HEIGHTu;

	double ax=factor1, ay=factor2;
	double r2 = iu*iu + ju*ju;
	//if(r>1)
	//	cout<<"error"<<"\t"<<xu<<"\t"<<yu<<"\t"<<iu<<"\t"<<ju<<endl;
    id = iu*(1 - (ax*r2));
    jd = ju*(1 - (ay*r2));
 	
 	*xd = (id*WIDTHu + WIDTHf)/2;
 	*yd = (-jd*HEIGHTu + HEIGHTf)/2;

}

void func2(int xu, int yu, int* xd, int* yd )
{
	double id,jd,iu,ju;
	iu = (double)(2*xu - WIDTHu)/WIDTHu;
	ju = (double)(2*-yu + HEIGHTu)/HEIGHTu;

	double ax=factor1, ay=factor2;
	double r = sqrt(iu*iu + ju*ju);

    id = iu/(1 - (ax*r*r));
    jd = ju/(1 - (ay*r*r));
 	
 	*xd = (id*WIDTHu + WIDTHf)/2;
 	*yd = (-jd*HEIGHTu + HEIGHTf)/2;

}

void func3(int xu, int yu, int* xd, int* yd )
{
	double id,jd,iu,ju;
	iu = (double)(2*xu - WIDTHu)/WIDTHu;
	ju = (double)(2*-yu + HEIGHTu)/HEIGHTu;

	double ax=factor1, ay=factor2;
	double P =sqrt(iu*iu + ju*ju);
	double Px = P/(1 - ax*P*P);
	double Py = P/(1 - ay*P*P);


	
	id=iu/(1 - (ax*Px*Px));
	jd=ju/(1 - (ay*Py*Py));


 	*xd = (id*WIDTHu + WIDTHf)/2;
 	*yd = (-jd*HEIGHTu + HEIGHTf)/2;

 	//cout<<"xd="<<xd<<"\tyd="<<yd<<"\txu="<<*xu<<"\tyu="<<*yu<<endl;
 	//cout<<"id="<<id<<"\tjd="<<jd<<"\tiu="<<iu<<"\tju="<<ju<<endl;
}

//inverse of func0
void func4(int xu, int yu, int* xd, int* yd )
{
	double id,jd,iu,ju;
	iu = (double)(xu - WIDTHu/2);
	ju = (double)(-yu + HEIGHTu/2);

	double ax=factor1, ay=factor1;
	double ru =iu*iu + ju*ju;
	double rd = sqrt(1/(-ax+(1/ru)));
		id = iu*(1+ax*rd*rd);
		jd = ju*(1+ay*rd*rd);

 	*xd = (id + WIDTHf/2);
 	*yd = (-jd + HEIGHTf/2);

 	//cout<<"xd="<<xd<<"\tyd="<<yd<<"\txu="<<*xu<<"\tyu="<<*yu<<endl;
 	//cout<<"id="<<id<<"\tjd="<<jd<<"\tiu="<<iu<<"\tju="<<ju<<endl;
}

//use factor = 6.1
void fish(int xu,int yu, int* xd, int* yd)
{
	xu = (xu - WIDTHu/2);
	yu = -(yu - HEIGHTu/2);
	float theta;
	float ru = (sqrt(xu*xu + yu*yu))/(sqrt(HEIGHTu*HEIGHTu + WIDTHu*WIDTHu)/factor1);
	

	if(ru==0)
		theta = 1;
	else
		theta = atan(ru)/ru;
	
	*xd = xu*theta;
	*yd = yu*theta;

	*xd = *xd + WIDTHf/2;
 	*yd = -*yd + HEIGHTf/2;
 	

}

int main()
{
	Mat temp = imread("ueye.bmp", CV_LOAD_IMAGE_COLOR);
	Mat undis(Size(752, 752), CV_8UC3);

	Rect croppedRect(136,0,480,480);
	Mat fsh = temp(croppedRect).clone();


	HEIGHTf = fsh.rows;
	WIDTHf = fsh.cols;
	cout<<fsh.cols<<"x"<<fsh.rows<<endl;

	HEIGHTu = undis.rows;
	WIDTHu = undis.cols;

	cvNamedWindow("original image");
	cvNamedWindow("new image");
	
	int ro=0, co=0;
	int xf=0, yf=0;
	int xu=0, yu=0;
	char t='b';

/*
//	reverse mapping --> maps every pixel in undistorted image to the equivalent in distorted image 
	while(t!='q')
	{	

		for(ro=0; ro < HEIGHTu; ro++)
		{
			for(co=0; co<WIDTHu; co++)
			{
				func4(co, ro, &xf, &yf);
				if(xf>=0 && yf>=0 && xf<WIDTHf && yf<HEIGHTf)
					undis.at<Vec3b>(ro,co) = fsh.at<Vec3b>(yf,xf);
			}
		}
	
		imshow("new image", undis);
		t=cvWaitKey();

		if(t == 'w')
			factor1 += 0.001;
		else if(t == 'd')
			factor1 += 0.0000001;
		else if(t == 's')
			factor1 -= 0.001;
		else if(t == 'a')
			factor1 -= 0.0000001;

		if(t == 'i')
			factor2 += 0.001;
		else if(t == 'l')
			factor2 += 0.0001;
		else if(t == 'k')
			factor2 -= 0.001;
		else if(t == 'j')
			factor2 -= 0.0001;
		//fsh=undis.clone();
		undis = temp.clone();
		//imwrite("newimage3.bmp", fsh);
		cout<<"factors:"<<factor1<< "   " <<factor2<<endl;
	}	*/

///*
//	forward mapping --> maps every pixel in distorted image to the equivalent in undistorted image. May get many empty pixels undistorted image.
	while(t!='q')
	{	

		for(ro=0; ro < HEIGHTu; ro++)
		{
			for(co=0; co<WIDTHu; co++)
			{
				func0(co, ro, &xu, &yu);
				if(xu>=0 && yu>=0 && xu<WIDTHu && yu<HEIGHTu)
					undis.at<Vec3b>(yu,xu) = fsh.at<Vec3b>(ro,co);
			}
		}
	
		imshow("new image", undis);
		imshow("original", fsh);
		t=cvWaitKey();

		if(t == 'w')
			factor2 += 1e-07;
		else if(t == 'd')
			factor1 += 1e-07;
		else if(t == 's')
			factor2 -= 1e-07;
		else if(t == 'a')
			factor1 -= 1e-07;

		// if(t == 'i')
		// 	factor2 += 0.001;
		// else if(t == 'l')
		// 	factor2 += 1e-07;
		// else if(t == 'k')
		// 	factor2 -= 0.001;
		// else if(t == 'j')
		// 	factor2 -= 1e-07;

		//fsh=undis.clone();
		//imwrite("newimage3.bmp", fsh);
		cout<<"factors:"<<factor1<<"   "<<factor2<<endl;
	}	//*/
	return 0;

}