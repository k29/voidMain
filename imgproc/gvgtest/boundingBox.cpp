#include "boundingBox.h"

BoundingBox::BoundingBox() {
	rectangle.x = -1;
	rectangle.y = -1;
	rectangle.width = 0;
	rectangle.height = 0;
	basePoint.x = -1;
	basePoint.y = -1;
	numberOfEdgePoints = 0;
}

BoundingBox::BoundingBox(const CvRect &r, const CvPoint &p,
		int16_t nrEdgePoints) {
	rectangle.x = r.x;
	rectangle.y = r.y;
	rectangle.width = r.width;
	rectangle.height = r.height;

	basePoint.x = p.x;
	basePoint.y = p.y;
	numberOfEdgePoints = nrEdgePoints;
}

BoundingBox::~BoundingBox() { }


/**
 * Merges two rectangles to a new bounding box.
 *
 * @param r1
 * @param r2
 * @return the new minimal bounding box, which contains r1 and r2
 */
BoundingBox BoundingBox::merge(const BoundingBox &r1, const BoundingBox &r2) {
	int16_t minX, maxX, minY, maxY;
	minX = min(r1.rectangle.x, r2.rectangle.x);
	maxX = max(r1.rectangle.x + r1.rectangle.width, r2.rectangle.x + r2.rectangle.width);

	minY = min(r1.rectangle.y, r2.rectangle.y);
	maxY = max(r1.rectangle.y + r1.rectangle.height, r2.rectangle.y + r2.rectangle.height);

	BoundingBox bb;
	bb.rectangle.x = minX;
	bb.rectangle.y = minY;
	bb.rectangle.width = maxX - minX;
	bb.rectangle.height = maxY - minY;

	if(r1.basePoint.y > r2.basePoint.y) {
		bb.basePoint.x = r1.basePoint.x;
		bb.basePoint.y = r1.basePoint.y;
	}
	else {
		bb.basePoint.x = r2.basePoint.x;
		bb.basePoint.y = r2.basePoint.y;
	}

	bb.numberOfEdgePoints = r1.numberOfEdgePoints + r2.numberOfEdgePoints;

	return bb;
}

/**
 * This merges the list of rectangles to a new list of rectangles,
 * where rectangles get merged when the distance between them is less then the threshold.
 *
 * @param inputRects rectangles to merge
 * @param outputRects merged list of rectangles -> should be empty
 * @param distanceX threshold value
 * @param distanceY threshold value
 */
void BoundingBox::mergeRectsInDistance(std::list<BoundingBox> &inputRects, std::list<BoundingBox> &outputRects, int16_t _distanceX, int16_t _distanceY) {
	// outputRects has to be empty!
	outputRects.clear();

	while(inputRects.size() > 0) {
		// get next input rectangle
		BoundingBox r = inputRects.back();
		inputRects.pop_back();

		// test, if there is already a bounding box in the specified distance to merge with
		bool collision = false;
		std::list<BoundingBox>::iterator iter;
		BoundingBox rr;
		for(iter = outputRects.begin(); iter != outputRects.end(); ++iter) {
			int16_t distX = r.distanceX(*iter);
			int16_t distY = r.distanceY(*iter);
			if(distX <= _distanceX && distY <= _distanceY) {
				collision = true;
				rr = *iter;
				break;
			}
		}

		if(collision) {
			outputRects.erase(iter);
			BoundingBox merged = merge(r, rr);
			inputRects.push_back(merged);
		}
		else {
			outputRects.push_back(r);
		}

	}
//	  output = new RectangleSet
//	  while input.length > 0 do
//	    nextRect = input.pop()
//	    intersected = output.findIntersected(nextRect)
//	    if intersected then
//	      output.remove(intersected)
//	      input.push(nextRect.merge(intersected))
//	    else
//	      output.insert(nextRect)
//	  done
}

/**
 * Paints the bounding box.
 *
 * @param img image to paint on
 * @param scale scaling factor
 */
void BoundingBox::paint(IplImage *img, int scale) {
	if(rectangle.x != -1) {
		CvPoint scaledP1 = { rectangle.x / scale, rectangle.y / scale };
		CvPoint scaledP2 = { (rectangle.x + rectangle.width) / scale,  (rectangle.y + rectangle.height) / scale };
		CvPoint scaledBasePoint = { basePoint.x / scale, basePoint.y / scale };
		cvRectangle(img, scaledP1, scaledP2, white, 1, CV_AA, 0);
		cvCircle(img, scaledBasePoint, 2, black, 1, CV_AA, 0);
	}
}
