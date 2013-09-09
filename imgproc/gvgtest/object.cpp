#include "object.h"

Object::Object() {


}

Object::~Object() {

}

/**
 * Paints the rectangle object
 *
 * @param img image to paint on
 * @param scale scaling factor of image
 */
void RectangleObject::paint(IplImage *img, int scale) {
	if(rectangle.x == -1 || img == 0) {
		return;
	}

	CvFont font;
	cvInitFont( &font, CV_FONT_HERSHEY_SIMPLEX, 0.35, 0.35, 0.0, 1, CV_AA );
	CvScalar colorPaint;

	switch(color) {
	case Ball:
		colorPaint = red; break;
	case BlueGoal:
		colorPaint = blue; break;
	case YellowGoal:
		colorPaint = yellow; break;
	case Cyan:
		colorPaint = cyan; break;
	case Magenta:
		colorPaint = magenta; break;
	default:
		colorPaint = black;
	}

	switch(type) {
	case GOAL_POLE_LEFT_OBJECT:
		cvPutText(img,"L",cvPoint((rectangle.x + rectangle.x + rectangle.width) / (2* scale), (rectangle.y + rectangle.y + rectangle.height) / (2* scale)), &font,black);
		break;
	case GOAL_POLE_RIGHT_OBJECT:
		cvPutText(img,"R",cvPoint((rectangle.x + rectangle.x + rectangle.width) / (2* scale), (rectangle.y + rectangle.y + rectangle.height) / (2* scale)), &font,black);
		break;
	case GOAL_POLE_UNKNOWN_OBJECT:
		cvPutText(img,"U",cvPoint((rectangle.x + rectangle.x + rectangle.width) / (2* scale), (rectangle.y + rectangle.y + rectangle.height) / (2* scale)), &font,black);
		break;
	case POLE_OBJECT:
		const char *t;
		if(color == BlueGoal) {
			t = "BYB";
		}
		else {
			t = "YBY";
		}
		cvPutText(img,t,cvPoint((rectangle.x + rectangle.x + rectangle.width) / (2* scale), (rectangle.y + rectangle.y + rectangle.height) / (2* scale)), &font,black);
		break;
	default:
		break;
	}

	cvRectangle(img, cvPoint(rectangle.x / scale, rectangle.y / scale),
			cvPoint((rectangle.x + rectangle.width) / scale, (rectangle.y + rectangle.height) / scale), colorPaint);
	cvCircle(img, cvPoint(basePoint.x / scale, basePoint.y / scale),2,black,1,CV_AA,0);
}

/**
 * Paints the cluster object
 *
 * @param img image to paint on
 * @param scale scaling factor of image
 */
void ClusterObject::paint(IplImage *img, int scale) {
	if(img == 0) {
		return;
	}

	std::vector<PointVector>::const_iterator iter;
	PointVector::const_iterator iterP;

	for(iter = cluster.begin(); iter != cluster.end(); ++iter) {
		const PointVector &ps = *iter;
		CvPoint lastPoint = { 0, 0 };

		for(iterP = ps.begin(); iterP != ps.end(); ++iterP) {
			CvPoint pp = cvPoint((*iterP).x / scale, (*iterP).y / scale);
			if(iterP == ps.begin()) {
				lastPoint = pp;
			}

			// interconnect points
			cvLine(img, lastPoint, pp, magenta, 1, CV_AA, 0);

			cvCircle(img, pp, 2, white, 1, CV_AA, 0);

			lastPoint = pp;
		}
	}

}
