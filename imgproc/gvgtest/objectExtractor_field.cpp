#include "objectExtractor_field.h"
#include "camera/theCameraModel.h"
#include "math/utils.h"
#include <numeric>


FieldExtractor::FieldExtractor() {
	fieldContourSize = 0;
}

FieldExtractor::~FieldExtractor() {
	originalFieldContour.clear();
	filteredFieldContour.clear();

	delete[] fieldContour;

	fieldContour = 0;
	image = 0;
}

/**
 * Clears all data
 */
void FieldExtractor::clear() {
	originalFieldContour.clear();
	filteredFieldContour.clear();
	memset(fieldContour, 0, sizeof(int16_t) * fieldContourSize);
}

/**
 * Extracts the field contour.
 * We scan in scanlines from 2/3 of the image height in direction to the image top and find the last green point
 * on this way (with some thresholding and stuff).
 * Then, the points are filtered and compared with points some pixels above.
 * Finally, the field contour is calculated as linear interpolation between the filtered points.
 * This is like a function, for every x value you get the heighest y-value in the image, which belongs to the field
 * (so everything under this y value is on the field).
 * @return true, if we found a contour, otherwise false
 */
bool FieldExtractor::extract() {
	if(image == 0)
		return false;

	uint16_t width = image->getImageWidth();
	if(fieldContourSize != width) {
		fieldContourSize = width;
		if(fieldContour != 0) {
			delete[] fieldContour;
			fieldContour = 0;
		}
		fieldContour = new int16_t[fieldContourSize];
	}

	clear();

	// extract the fieldcontour
	scanlines();

    if (originalFieldContour.size() == 0) {
//    	WARNING("objectExtractor_field: Original field contour is empty.");
    	// nothing else to do with an empty contour
    	return false;
    }

	filterFieldContour();
	interpolateFieldContour();

	return true;
}

/**
 * Go in scanlines up and find last green points.
 */
void FieldExtractor::scanlines() {
	int16_t horizonY = TheCameraModel::getInstance().horizon(900);
	if(horizonY < 0) {
		horizonY = 0;
	}

	uint16_t width = image->getImageWidth();
	uint16_t height = image->getImageHeight();

	// go in scanlines up along the x axis and find the field contour
	for(int16_t x = 0; x < (int16_t) width; x += X_STEP) {

		CvPoint lastGreenPoint = {-1, -1};
		int16_t missGreenCounter = 0;
		int16_t whiteCounter = 0;

		for(int16_t y = 2* (int16_t) height / 3; y > horizonY; y -= 5) {
			Color color = colorMgr->getPixelColor(*image, x, y);

			if (color == Field) {
				uint8_t greenNeighbors = 0;
				// TODO is computation time worth it?
				// check if neighbors are field pixel too
				greenNeighbors += isFieldPixel(x-1,y);
				greenNeighbors += isFieldPixel(x,y-1);
				greenNeighbors += isFieldPixel(x+1,y);
				greenNeighbors += isFieldPixel(x,y+1);

				whiteCounter = 0;

				// if there are at least two green neighbors
				if (greenNeighbors > 1) {
					missGreenCounter = 0;
					lastGreenPoint.x = x;
					lastGreenPoint.y = y;
				}
				// else: Field pixel was an outlier, count as missed field
				else {
					++missGreenCounter;
				}
			}
			else if (color == White) {
				// ignore white pixels e.g. field lines to some extend
				if(y < height/2)
					++whiteCounter;
			} else {
				// count missed field pixels
				++missGreenCounter;
			}

			if(missGreenCounter > 5 || whiteCounter > 10) {
				break;
			}
		}

		if(lastGreenPoint.y != -1) {
			originalFieldContour.push_back(lastGreenPoint);
		}

	}
}

/**
 * Check, if the given pixel (x, y) belongs to the field, that means its color is green
 * @param x
 * @param y
 * @return
 */
bool FieldExtractor::isFieldPixel(int16_t x, int16_t y) {
	if (x < 0 || y < 0 || x > image->getImageWidth() - 1 || y > image->getImageHeight() - 1)
		return 0;

	Color color = colorMgr->getPixelColor(*image, x, y);
	return color == Field;
}

/**
 * Filter the field contour
 */
void FieldExtractor::filterFieldContour() {
	if(originalFieldContour.empty()) {
//		WARNING("Original field contour points empty, can't filter field contour points!");
		return;
	}

	uint8_t neighbors = 7;
	std::vector<int> ys;

	for(uint16_t i = 0; i < (int16_t) originalFieldContour.size(); ++i) {
		ys.clear();

		for(int16_t j=-neighbors/2; j <= neighbors/2; ++j) {
			if(i+j < 0) continue;
			if(i+j >= (int16_t)originalFieldContour.size()) break;

			ys.push_back(originalFieldContour[i+j].y);
		}
		uint16_t mid = 0;

		if (ys.size() < neighbors) {
			// compute mean
			mid = std::accumulate(ys.begin(), ys.end(),0) / ys.size();
		} else {
			// compute median
			std::sort(ys.begin(), ys.end());
			mid = ys[ys.size()/2];
		}

		// if distance between original height and the mean/median height
		// is greater than some threshold -> i is an outlier -> replace it
		if(mid-originalFieldContour[i].y > 10) {
			// filter up peaks
			filteredFieldContour.push_back(cvPoint(originalFieldContour[i].x,mid));
		} else if(originalFieldContour[i].y-mid > 30) {
			// filter downward peaks
			filteredFieldContour.push_back(cvPoint(originalFieldContour[i].x,mid+10));
		} else {
			// keep original point
			filteredFieldContour.push_back(originalFieldContour[i]);
		}

	}

	// convert filteredFieldContour to CvSeq
	CvMemStorage* memStorage = cvCreateMemStorage(0);
	CvSeq* seq = cvCreateSeq(CV_SEQ_ELTYPE_POINT, sizeof(CvSeq), sizeof(CvPoint), memStorage);
	cvSeqPushMulti(seq, filteredFieldContour.data(), filteredFieldContour.size());

	// now compute convex hull of filtered points using OpenCV
	CvSeq* hull;
	hull = cvConvexHull2(seq, 0, CV_COUNTER_CLOCKWISE, 0 );
	filteredFieldContour.clear();

	std::vector<CvPoint> tmpContour;
	for (int16_t i = 0; i < (int16_t) hull->total; i++) {
		CvPoint pt = **CV_GET_SEQ_ELEM(CvPoint*, hull, i);
		tmpContour.push_back(pt);
	}

	// split upper and lower hull and just keep the lower hull (in the image the upper one) and the first point of upper hull
	int splitIndex = -1;
	for(int16_t i = 0; i < (int16_t) tmpContour.size() - 1; ++i) {
		if(tmpContour.at(i).x < tmpContour.at(i+1).x && splitIndex == -1) {
			splitIndex = i;
			break;
		}
	}
	if(splitIndex != -1) {
		filteredFieldContour.assign(tmpContour.begin() + splitIndex, tmpContour.end());
		filteredFieldContour.push_back(tmpContour.front());
	}
	else {
//		WARNING("Filtered field contour can't be split into upper and lower convex hull.");
		filteredFieldContour.assign(tmpContour.begin(), tmpContour.end());
	}

	cvReleaseMemStorage(&memStorage);
}

/**
 * Linear Interpolation between the field contour points build the field contour.
 */
void FieldExtractor::interpolateFieldContour() {
	if(filteredFieldContour.empty()) {
//		WARNING("Filtered field contour points empty, can't interpolate field contour!");
		return;
	}

	uint16_t width = image->getImageWidth();

	// interpolate the field contour (linear between two points)

	for(int16_t i = 0; i < (int16_t) filteredFieldContour.size() - 1; ++i) {

		const CvPoint &p1 = filteredFieldContour.at(i);
		const CvPoint &p2 = filteredFieldContour.at(i + 1);

		if(p1.x == p2.x) {
			continue;
		}

		// enlarge field contour at the beginning and in the end
		if(i == 0) {
			for(int16_t x = 0; x < p1.x; x++) {
				fieldContour[x] = p1.y;
			}
		}
		else if (i == (int16_t) filteredFieldContour.size() - 2) { //
			for(int16_t x = p2.x; x < width; x++) {
				fieldContour[x] = p2.y;
			}
		}

		int16_t distX = p2.x - p1.x;
		int16_t distY = p2.y - p1.y;
		for(int16_t x = p1.x; x <= p2.x; ++x) {

			int16_t y = p1.y + distY * (x - p1.x) / distX;

			if (fieldContour[x] == 0) { // currently not set
				fieldContour[x] = y;
			}
			else if (fieldContour[x] > y) {
				fieldContour[x] = y;
			}

		}
	}
}
