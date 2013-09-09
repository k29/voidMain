#include "objectExtractor_fieldlinefeature.h"
#include "gvg.h"
#include "vision.h"
#include "position.h"
#include "math/Math.h"
#include "utils/histogram1D.h"

#include <inttypes.h>

FieldLineFeatureExtractor::FieldLineFeatureExtractor() {
	nrCellsX = 0;
	nrCellsY = 0;
	lineDirectionMap = 0;

	fieldLineCluster = new ClusterObject();
}

FieldLineFeatureExtractor::~FieldLineFeatureExtractor() {
	free(lineDirectionMap);
	lineDirectionMap = 0;

	delete fieldLineCluster;
	fieldLineCluster = 0;
}

/**
 * Clears all data
 */
void FieldLineFeatureExtractor::clear() {
	memset(lineDirectionMap, 0, sizeof(FixedPointMath::fixedpoint) * nrCellsY * nrCellsX * 2);

	fieldLineEdges.clear();
	fieldLineCluster->clear();
	rawLineFeatures.clear();
	lineFeatures.clear();
}

/**
 * Extract the field line features
 * @param edges
 * @return
 */
bool FieldLineFeatureExtractor::extract(const EdgeVector &edges) {
	if(image->getImageWidth() / GRID_CELL_SIZE + 1 != nrCellsX ||
			image->getImageHeight() / GRID_CELL_SIZE + 1 != nrCellsY) {

		free(lineDirectionMap);
		lineDirectionMap = 0;

		nrCellsX = image->getImageWidth() / GRID_CELL_SIZE + 1;
		nrCellsY = image->getImageHeight() / GRID_CELL_SIZE + 1;

		lineDirectionMap = (FixedPointMath::fixedpoint*) malloc( sizeof(FixedPointMath::fixedpoint) * nrCellsY * nrCellsX * 2);
	}


	clear();

	// filter field line edges
	for (EdgeVector::const_iterator iter = edges.begin(); iter != edges.end(); ++iter) {
		Edge *e = *iter;
		int minY = e->getFirstPoint().getY() < e->getLastPoint().getY() ? e->getFirstPoint().getY() : e->getLastPoint().getY();
		// test if the field lines have the required length
		if((minY < 160 && e->linePoints.size() < 2) || (minY >= 160 && minY < 320 && e->linePoints.size() < 4)
				|| (minY >= 320 && e->linePoints.size() < 6)) {
			continue;
		}
		fieldLineEdges.push_back(e);
	}

	// store field lines in cluster object
	if(fieldLineEdges.size() > 0) {
		fieldLineCluster->cluster.reserve(fieldLineEdges.size());

		EdgeVector::iterator iter;
		for (iter = fieldLineEdges.begin(); iter != fieldLineEdges.end(); ++iter) {
			Edge *e = *iter;

			PointVector points;
			std::vector<EdgePoint>::iterator pIter;
			for(pIter = e->linePoints.begin(); pIter != e->linePoints.end(); ++pIter) {
				points.push_back(cvPoint(pIter->getX(), pIter->getY()));
			}
			fieldLineCluster->cluster.push_back(points);
		}
	}

	// field line features

	withHistogram(fieldLineEdges); // extract raw field line features
	verifyFieldLineFeatures(); // verify the raw field line features

	return true;
}

/**
 * Extract the field line features by creating histograms along both sides of a field line
 * counting the number of field line cells (of the GVG-grid)
 *
 * @param edges field lines to analyze
 */
void FieldLineFeatureExtractor::withHistogram(const EdgeVector &edges) {

	int threshold = 2;

	// create the map
	createFieldLineMap(edges);

	EdgeVector::const_iterator iter;
	for(iter = edges.begin(); iter != edges.end(); ++iter) {
		const Edge *e = *iter;

		std::vector<EdgePoint> linePoints(e->linePoints);

		// extend lines
		EdgePoint firstP = linePoints.front();
		EdgePoint lastP = linePoints.back();
		for(int i = 1; i < 4; ++i) {
			EdgePoint fp(
					firstP.getX() + toInt(fmul(firstP.getDirX(), toFixed(i*8))),
					firstP.getY() + toInt(fmul(firstP.getDirY(), toFixed(i*8))),
					firstP.getDirX(),
					firstP.getDirY());

			linePoints.insert(linePoints.begin(), fp);

			EdgePoint lp(
					lastP.getX() + toInt(fmul(lastP.getDirX(), toFixed(-i*8))),
					lastP.getY() + toInt(fmul(lastP.getDirY(), toFixed(-i*8))),
					lastP.getDirX(),
					lastP.getDirY());

			linePoints.push_back(lp);
		}


		// create histograms for both sides of the edge
		int16_t size = linePoints.size();
		Histogram1D histogramSite1(size, false);
		Histogram1D histogramSite2(size, false);

		std::vector<EdgePoint>::const_iterator piter;
		int16_t i = 0;
		for(piter = linePoints.begin(); piter != linePoints.end(); ++piter) {
			const EdgePoint &p = *piter;

			// get orthogonal direction
			FixedPointMath::fixedpoint orthodirX = -p.getDirY();
			FixedPointMath::fixedpoint orthodirY = p.getDirX();

			histogramSite1.setValue(i, Histogram1D::Increment1, countFieldLineCellsInDirection(p.getX(), p.getY(), orthodirX, orthodirY));
			histogramSite2.setValue(i, Histogram1D::Increment1, countFieldLineCellsInDirection(p.getX(), p.getY(), -orthodirX, -orthodirY));

			++i;

		}

		// Analyze histogram
		for(i = 0; i < (int16_t) linePoints.size(); ++i) {
//			printf("%d count: %d %d\n", i, histogramSite1.data()[i], histogramSite2.data()[i]);

			const EdgePoint &p = linePoints[i];
			FieldLineFeature f(p.getX(), p.getY());

			if(i < 3 || linePoints.size()-1 - i < 3) { // check for T or L at the end of the line (there it can't be a X)
				if(histogramSite1.data()[i]  > threshold && histogramSite2.data()[i] > threshold) { // T
					f.type = TCrossingFieldLine;
					rawLineFeatures.push_back(f);
				}
				else if(histogramSite1.data()[i]  > threshold && histogramSite2.data()[i] <= threshold) { // L
					f.type = LCrossingFieldLine;
					rawLineFeatures.push_back(f);
				}
				else if(histogramSite1.data()[i] <= threshold && histogramSite2.data()[i] > threshold) { // L
					f.type = LCrossingFieldLine;
					rawLineFeatures.push_back(f);
				}
			}

			else if (histogramSite1.data()[i] > threshold && histogramSite2.data()[i] > threshold) { // X
				f.type = XCrossingFieldLine;
				rawLineFeatures.push_back(f);
			}
			else if (histogramSite1.data()[i]  > threshold && histogramSite2.data()[i] <= threshold) { // T
				f.type = TCrossingFieldLine;
				rawLineFeatures.push_back(f);
			}
			else if (histogramSite1.data()[i] <= threshold && histogramSite2.data()[i] > threshold) { // T
				f.type = TCrossingFieldLine;
				rawLineFeatures.push_back(f);
			}

		}
	}

}

/**
 * Verifies the raw field line features. Group near ones.
 */
void FieldLineFeatureExtractor::verifyFieldLineFeatures() {

	// group field line features to bounding boxes
	std::list<BoundingBox> tmpBoxes;
	std::vector<FieldLineFeature>::iterator iter;
	for(iter = rawLineFeatures.begin(); iter != rawLineFeatures.end(); ++iter) {
		tmpBoxes.push_back(convertLineFeatureToBoundingBox(*iter));
	}

	BoundingBox::mergeRectsInDistance(tmpBoxes, groupedFeatures, 30, 30);

	bool circleMap[72]; // 360 / 5

	// iterate over all possible feature positions and check for white around the bounding box along a circle

	std::list<BoundingBox>::iterator biter;
	for(biter = groupedFeatures.begin(); biter != groupedFeatures.end(); ++biter) {
		memset(circleMap, false, sizeof(circleMap));
		const BoundingBox &box = *biter;

		// calculate center of bounding box
		int centerX = (box.rectangle.x + box.rectangle.x + box.rectangle.width) / 2;
		int centerY = (box.rectangle.y + box.rectangle.y + box.rectangle.height) / 2;

		// go on circle around possible position and save white occurrences
		for(int angle = -180; angle < 180; angle += 5) {

			int x = centerX + toInt(fmul(fcos(toFixed(angle)), toFixed(30)));
			int y = centerY + toInt(fmul(fsin(toFixed(angle)), toFixed(30)));

			// check for boundary
			if(x < 0 || x > image->getImageWidth() -1 || y < 0 || y > image->getImageHeight() - 1) {
				continue;
			}

			// check if outside the field
			if(Vision::getInstance().getFieldExtractor().getHeighestFieldCoordinate(x) > y) {
				continue;
			}

			Color color = colorMgr->getPixelColor(*image, x, y);
			if(color == White) {
				circleMap[(angle + 180) / 5] = true;
			}
//			printf("%d", circleMap[(angle + 180) / 5]);
		}
//		printf("\n");


		// Analyze histogram of white occurrences for segments
		// TODO take length of segments into account to
		int countWhiteSegments = 0;
		int indexLastSegment = -1;
		int distToLastSegment = 0;
		for(int i = 0; i < 72; ++i) {
			if(circleMap[i] && !circleMap[(72 + i-1) % 72]) { // found beginning of a new white segment
				distToLastSegment = i - indexLastSegment;
				if(indexLastSegment == -1) {
					distToLastSegment = -1;
				}
				if(distToLastSegment > 3 || distToLastSegment == -1) {
					indexLastSegment = i;
					countWhiteSegments++;
				}
//				printf("indexLastSeg %d dist %d\n", indexLastSegment, distToLastSegment);
			}
		}
//		printf("white segments: %d\n", countWhiteSegments);

		// determine feature type
		FieldLineFeature flf( centerX, centerY);
		if(countWhiteSegments == 2) {
			flf.type = LCrossingFieldLine;
			lineFeatures.push_back(flf);
		}
		else if(countWhiteSegments == 3) {
			flf.type = TCrossingFieldLine;
			lineFeatures.push_back(flf);
		}
		else if(countWhiteSegments == 4) {
			flf.type = XCrossingFieldLine;
			lineFeatures.push_back(flf);
		}

	}

}

/**
 * Creates a map with the directions of the field lines
 * @param edges Edges representing the field lines
 */
void FieldLineFeatureExtractor::createFieldLineMap(const EdgeVector &edges) {
	for(uint16_t i = 0; i < edges.size(); ++i) {
		Edge *e = edges[i];

		for(std::vector<EdgePoint>::iterator iter = e->linePoints.begin(); iter != e->linePoints.end(); ++iter) {
			const EdgePoint &p = *iter;

			uint8_t indexX = p.getX() / GRID_CELL_SIZE;
			uint8_t indexY = p.getY() / GRID_CELL_SIZE;
			FixedPointMath::fixedpoint dir = FixedPointMath::fatan2(p.getDirY(), p.getDirX());

			// we have max two directions per cell
			if(lineDirectionMap[indexY * nrCellsX * 2 + indexX * 2] != 0) {
				lineDirectionMap[indexY * nrCellsX * 2 + indexX * 2 + 1] = dir;
			}
			else {
				lineDirectionMap[indexY * nrCellsX * 2 + indexX * 2] = dir;
			}

		}

	}
}

/**
 * Counts the number of field line cells lying parallel / anti parallel to the given direction
 * (calculated by following from the start point in this direction and check for each cell on the way the cell angles)
 * @param startX
 * @param startY
 * @param dirX
 * @param dirY
 * @return number of cells an the way
 */
int FieldLineFeatureExtractor::countFieldLineCellsInDirection(int16_t startX, int16_t startY, FixedPointMath::fixedpoint dirX, FixedPointMath::fixedpoint dirY) {
	int16_t nextX = startX;
	int16_t nextY = startY;
	int16_t currentCellX = startX / GRID_CELL_SIZE;
	int16_t currentCellY = startY / GRID_CELL_SIZE;
	int16_t nextCellX = startX / GRID_CELL_SIZE;
	int16_t nextCellY = startY / GRID_CELL_SIZE;

	uint16_t width = image->getImageWidth();
	uint16_t height = image->getImageHeight();

	FixedPointMath::fixedpoint angle = FixedPointMath::fatan2(dirY, dirX);

	int hitCounter = 0;
	int missCounter = 0;

	// go in orthogonal direction to the current edge point, as long as there is an edge in this direction
	while(true) {

		// calculate next cell in gradient direction ...
		FixedPointMath::fixedpoint i = toFixed(1);
		while(true) {
			nextX += toInt(fmul(i, dirX));
			nextY += toInt(fmul(i, dirY));

			nextCellX = nextX / GRID_CELL_SIZE;
			nextCellY = nextY / GRID_CELL_SIZE;

			if(currentCellX != nextCellX || currentCellY != nextCellY) {
				break;
			}
			i += toFixed(1);
		}

		// check for boundary
		if(nextCellX < 0 || nextCellY < 0 || nextCellX > width / GRID_CELL_SIZE - 1 || nextCellY > height / GRID_CELL_SIZE - 1) {
			break;
		}

		FixedPointMath::fixedpoint cellAngle1 = lineDirectionMap[nextCellY * nrCellsX * 2 + nextCellX * 2];
		FixedPointMath::fixedpoint cellAngle2 = lineDirectionMap[nextCellY * nrCellsX * 2 + nextCellY * 2 + 1];

		currentCellX = nextCellX;
		currentCellY = nextCellY;

		if(cellAngle1 == 0 && cellAngle2 == 0) { // there is no line, so continue with next cell!
			missCounter++;
			if(missCounter > 1) {
				break;
			}
			continue;
		}

		// check difference between the current cell angles and the direction angle,
		// if they are nearly parallel (or antiparallel) it's a hit!
		int diffAngle = Math::angleDiff(toInt(angle), toInt(cellAngle1));

		if(diffAngle < 27 || abs(diffAngle - 180) < 27) { // hit in right direction!
			++hitCounter;
		}
		else if (cellAngle2 != 0) { // test for hit with other direction
			diffAngle = Math::angleDiff(toInt(angle), toInt(cellAngle2));

			if(diffAngle < 27 || abs(diffAngle - 180) < 27) {
				++hitCounter;
			}
			else { // false direction found
				break;
			}
		}
		else { // false direction found
			break;
		}

	}

	return hitCounter;
}

/**
 * Converts a field line feature to a surrounding bounding box
 * @param lineFeature
 * @return
 */
BoundingBox FieldLineFeatureExtractor::convertLineFeatureToBoundingBox(const FieldLineFeature &lineFeature) {
	BoundingBox box;

	box.rectangle.x = lineFeature.getX();
	box.rectangle.y = lineFeature.getY();
	box.basePoint.x = lineFeature.getX();
	box.basePoint.y = lineFeature.getY();

	return box;
}
