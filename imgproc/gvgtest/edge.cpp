#include "edge.h"
#include "math/fixedPointMath.h"
#include "math/utils.h"

#include <math.h>

/**
 * Distance between the current edge "this" and the parameter edge.
 *
 * @param edge
 * @return distance
 */
int Edge::distance(const Edge &edge) {
	int dist = min(getFirstPoint().getDistance(edge.getFirstPoint()), 65535);
	dist = min(dist, getFirstPoint().getDistance(edge.getLastPoint()));
	dist = min(dist, getLastPoint().getDistance(edge.getFirstPoint()));
	dist = min(dist, getLastPoint().getDistance(edge.getLastPoint()));

	return dist;
}

/**
 * Converts an edge to a bounding box in which every EdgePoint fits into.
 *
 * @return bounding box
 */
BoundingBox Edge::getBoundingBox() {
	int16_t minX = 32000;
	int16_t maxX = -1;
	int16_t minY = 32000;
	int16_t maxY = -1;

	BoundingBox box;

	std::vector<EdgePoint>::const_iterator iter;
	for (iter = this->linePoints.begin(); iter != this->linePoints.end(); ++iter) {
		const EdgePoint &p = *iter;
		minX = min(minX, p.getX());
		maxX = max(maxX, p.getX());
		minY = min(minY, p.getY());
		maxY = max(maxY, p.getY());
		if(box.basePoint.y == -1 || p.getY() > box.basePoint.y) {
			box.basePoint.x = p.getX();
			box.basePoint.y = p.getY();
		}
	}

	box.rectangle.x = minX;
	box.rectangle.y = minY;
	box.rectangle.width = maxX - minX;
	box.rectangle.height = maxY - minY;
	box.numberOfEdgePoints = (int16_t) this->linePoints.size();
	return box;

}

/**
 * Paints the edge.
 *
 * @param img image to paint on
 * @param scale scaling factor
 */
void Edge::paint(IplImage *img, int scale) {
	if(img == 0) {
		return;
	}


	CvPoint prevPoint = {0, 0};
	std::vector<EdgePoint>::iterator iter;
//	int i = 0;
	for (iter = linePoints.begin(); iter != linePoints.end(); ++iter) {
		EdgePoint &p = *iter;
		CvPoint pp = cvPoint(p.getX() / scale, p.getY() / scale);
		if(iter == linePoints.begin()) {
			prevPoint = pp;
		}

		// interconnect points
		cvLine(img, prevPoint, pp, magenta, 1, CV_AA, 0);

		// draw direction (cyan point is arrow point)
//		CvPoint dirP = cvPoint((p.x + (int) (toFloat(p.dirX)* 10)) / scale, (p.y + (int) (toFloat(p.dirY) * 10)) / scale);
//		cvLine(img, pp, dirP, magenta, 1, CV_AA, 0);
//		cvCircle(img, dirP, 2, cyan, 1, CV_AA, 0);

		// draw edgel index
//		if(linePoints.size() > 1) {
//			char tmp[5];
//			sprintf(tmp, "%d", i);
//			CvFont font;
//			cvInitFont( &font, CV_FONT_HERSHEY_SIMPLEX, 0.35, 0.35, 0.0, 1, CV_AA );
//			cvPutText(img, tmp, cvPoint((p.x - (int) toFloat(p.dirY)*10) / scale, (p.y + (int) toFloat(p.dirX)*10) / scale), &font, yellow);
//			++i;
//		}

		// draw orthogonal line with colors
		FixedPointMath::fixedpoint dirY = (*iter).getDirY();
		FixedPointMath::fixedpoint dirX = (*iter).getDirX();
		CvPoint orthoPoint1 = cvPoint((int)(pp.x - toFloat(dirY) * 6 / scale), (int)(pp.y + toFloat(dirX) * 6 / scale));
		CvPoint orthoPoint2 = cvPoint((int)(pp.x + toFloat(dirY) * 6 / scale), (int)(pp.y - toFloat(dirX) * 6 / scale));

		CvScalar c1 = black;
		CvScalar c2 = black;
		switch(mostFrequentColor1) {
		case Ball: c1 = red; break;
		case YellowGoal: c1 = yellow; break;
		case BlueGoal: c1 = blue; break;
		case Magenta: c1 = magenta; break;
		case Cyan: c1 = cyan; break;
		case Obstacle: c1 = gray; break;
		case Field: c1 = green; break;
		case White: c1 = white; break;
		default: ;
		}
		switch(mostFrequentColor2) {
		case Ball: c2 = red; break;
		case YellowGoal: c2 = yellow; break;
		case BlueGoal: c2 = blue; break;
		case Magenta: c2 = magenta; break;
		case Cyan: c2 = cyan; break;
		case Obstacle: c2 = gray; break;
		case Field: c2 = green; break;
		case White: c2 = white; break;
		default: ;
		}
		cvLine(img, pp,orthoPoint1,c1,1,CV_AA,0);
		cvLine(img, pp,orthoPoint2,c2,1,CV_AA,0);

		prevPoint = pp;
	}

}

/**
 * Find all edges from the given input near the point (x,y).
 * @param x
 * @param y
 * @param distance
 * @param edges
 * @return
 */
std::vector<const Edge*> Edge::edgesNearToPoint(int16_t x, int16_t y, int16_t distance, const std::vector<Edge*> &edges) {
	std::vector<const Edge*> nearEdges;

	// check every edge
	for(EdgeVector::const_iterator iter = edges.begin(); iter != edges.end(); ++iter) {

		const Edge *e = *iter;

		// iterate over all points on line and check distance to point
		for(std::vector<EdgePoint>::const_iterator piter = e->linePoints.begin(); piter != e->linePoints.end(); ++piter) {
			const EdgePoint &p = *piter;

			if( FixedPointMath::isqrt((x- p.getX()) * (x - p.getX()) + (y - p.getY()) * (y - p.getY())) < distance) {
				nearEdges.push_back(e);
				break;
			}

		}

	}

	return nearEdges;
}

/**
 * Returns the edge containing the point (x,y) or NULL if no edge was found.
 * @param x
 * @param y
 * @return
 */
Edge* Edge::getEdgeWithPoint(int16_t x, int16_t y, std::vector<Edge*> &edges) {
	EdgeVector::iterator iter;

	// iterate over edges and check point by point with given (x,y)
	for(iter = edges.begin(); iter != edges.end(); ++iter) {
		Edge *e = *iter;

		std::vector<EdgePoint>::iterator piter;
		for(piter = e->linePoints.begin(); piter != e->linePoints.end(); ++piter) {
			const EdgePoint &p = *piter;
			if(p.getX() == x && p.getY() == y) {
				return e;
			}
		}
	}

	return NULL;
}
