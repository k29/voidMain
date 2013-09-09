#include "gvg.h"
#include "vision.h"
#include "camera/theCameraModel.h"
#include "math/fixedPointMath.h"
#include "debug.h"

static const uint8_t GRADIENT_SCALE = 2;

GradientVectorGriding::GradientVectorGriding() {
	colorCount = 0;
	gridcells = 0;
	nrGridCellsX = 0;
	nrGridCellsY = 0;
	reddishPixels = 0;

	stepSizeX = 1;
	stepSizeY = 1;
}

GradientVectorGriding::~GradientVectorGriding() {
	clear();

	free(gridcells);
	gridcells = 0;
	free(colorCount);
	colorCount = 0;
}

void GradientVectorGriding::clear() {
	for (EdgeVector::iterator iter = edges.begin(); iter != edges.end(); ++iter) {
		Edge *e = *iter;
		delete e;
		e = 0;
	}
	edges.clear();

	for (EdgeVector::iterator iter = fieldLineEdges.begin(); iter != fieldLineEdges.end(); ++iter) {
		Edge *e = *iter;
		delete e;
		e = 0;
	}
	fieldLineEdges.clear();
}

/**
 * Inits the GVG
 */
void GradientVectorGriding::init() {
	centerX = TheCameraModel::getInstance().centerX();
	centerY = TheCameraModel::getInstance().centerY();
	radius = (int16_t) TheCameraModel::getInstance().focalLengthX();
	range = cvRNG(rand());

	fieldExtractor = &Vision::getInstance().getFieldExtractor();

	if(image != 0) {
		nrGridCellsX = image->getImageWidth() / GRID_CELL_SIZE + 1;
		nrGridCellsY = image->getImageHeight() / GRID_CELL_SIZE + 1;

		gridcells = (GridCell*) malloc( sizeof(GridCell) * nrGridCellsY * nrGridCellsX);
		colorCount = (int16_t*) malloc( sizeof(int16_t) * nrGridCellsY * nrGridCellsX * 9);

		// init to zero
		memset(gridcells, 0, sizeof(GridCell) * nrGridCellsX * nrGridCellsY);
	}

}

/**
 * Runs the GVG algorithm
 */
void GradientVectorGriding::run() {
	// the image size has changed -> re-initialize all the stuff
	if(image->getImageHeight() / GRID_CELL_SIZE + 1 != nrGridCellsY ||
			image->getImageWidth() / GRID_CELL_SIZE + 1 != nrGridCellsX) {

		// clear old stuff
		if(gridcells != 0) {
			free(gridcells);
			gridcells = 0;
		}
		if(colorCount != 0) {
			free(colorCount);
			colorCount = 0;
		}

		nrGridCellsX = image->getImageWidth() / GRID_CELL_SIZE + 1;
		nrGridCellsY = image->getImageHeight() / GRID_CELL_SIZE + 1;

		INFO("Image size changed: new grid size: (%d, %d)", nrGridCellsX, nrGridCellsY);

		init();
	}

	clear();

	createGridCells();
	analyzeColors();
	markPossibleFieldlineCells();
	connectEdgeRepresenters();
	extractEdgesFromCells();
}

/**
 * Creates the grid cells with the edge information
 */
void GradientVectorGriding::createGridCells() {

	// calculates the horizon
	int16_t minY = TheCameraModel::getInstance().horizon(900);
	uint16_t width = image->getImageWidth();
	uint16_t height = image->getImageHeight();

	if (minY < GRADIENT_SCALE / 2)
		minY = GRADIENT_SCALE / 2;

	int magnitudeThreshold = Vision::getInstance().calibration.getMagnitudeThreshold();
	if(magnitudeThreshold < 0) {
		magnitudeThreshold = 20; // default value!
		//WARNING("Gradient magnitude threshold not set or invalid value! Default value is used (20)!");
	}

	// clear old values
	memset(gridcells, 0, sizeof(GridCell) * nrGridCellsY * nrGridCellsX);

	// for every image row (or nearly every ;-))
	for (int16_t y = minY; y < height - GRADIENT_SCALE / 2 - 1; y += stepSizeY) {

		// throw away up to 75% of the lines (12/16 = 0.75)
		uint8_t factor  = 40;
	
#ifdef ROBOT20011
		// throw away up to 44% of the lines (7/16 = 0.44)
		factor  = 68;
#endif

		// discard randomly some lines in the way,
		// that the probability for higher y is bigger then for lower ones
		if(( ((int) cvRandInt(&range)) & 15) < y / factor) {
			continue;
		}


		// calculate minimum x and maximum x for this line with circle
		int32_t discriminant = radius * radius - (centerY - y) * (centerY - y);
		if(discriminant < 0) {
			continue;
		}

		int16_t sqrt = (int16_t) FixedPointMath::isqrt(discriminant);
		int16_t x_left  = max(centerX - sqrt, GRADIENT_SCALE / 2);
		int16_t x_right = min(centerX + sqrt, width - GRADIENT_SCALE / 2 - 1);

		// don't search for edges directly on circle
		x_left  = max(x_left + GRID_CELL_SIZE,  GRADIENT_SCALE / 2);
		x_right = min(x_right - GRID_CELL_SIZE, width - GRADIENT_SCALE / 2 - 1);

		// for every image column
		for (int16_t x = x_left; x < x_right; x += stepSizeX) {

			int16_t grad_x = 0, grad_y = 0;
			uint16_t magn = 0;
			image->getGradient(x, y, &grad_x, &grad_y, &magn, GRADIENT_SCALE);

			if (magn > magnitudeThreshold) {
				GridCell *cell = &gridcells[(y >> 4) * nrGridCellsX + (x >> 4)]; // get right cell

				cell->hasRed = false;

				EdgeRepresenter &er1 = cell->er1;
				EdgeRepresenter &er2 = cell->er2;

				// inner product > 0 => acute angle
				// inner product < 0 => obtuse angle
				if(er1.dx * grad_x + er1.dy * grad_y >= 0) {
					er1.x += x & 15; // we want relative coordinates in the 16 x 16 cell (it's like % 16)
					er1.y += y & 15;
					er1.dx += grad_x;
					er1.dy += grad_y;
					er1.magnitude += magn;
					++er1.usedPixelCounter;
					er1.edge = NULL;
				}
				else if (er2.dx * grad_x + er2.dy * grad_y >= 0) {
					er2.x += x & 15; // we want relative coordinates in the 16 x 16 cell (it's like % 16)
					er2.y += y & 15;
					er2.dx += grad_x;
					er2.dy += grad_y;
					er2.magnitude += magn;
					++er2.usedPixelCounter;
					er2.edge = NULL;
				}

			}
		}
	}
}

/**
 * Analyzes each grid cell for the the two most frequent colors. The colors are set in each GridCell.
 */
void GradientVectorGriding::analyzeColors() {

	// clear old values
	memset(colorCount, 0, sizeof(int16_t) * nrGridCellsY * nrGridCellsX * 9);

	for(int16_t y = 0; y < nrGridCellsY; ++y) {
		for(int16_t x = 0; x < nrGridCellsX; ++x) {

			GridCell *currentCell = &gridcells[y * nrGridCellsX + x];
			EdgeRepresenter &currentER1 = currentCell->er1;
			EdgeRepresenter &currentER2 = currentCell->er2;

			uint16_t cnt = currentER1.usedPixelCounter + currentER2.usedPixelCounter;
			reddishPixels = 0;

			if (cnt == 0) { // discard cell, because there are is no edge
				continue;
			}
			if (abs(currentER1.dx) + abs(currentER1.dy) == 0 && abs(currentER2.dx) + abs(currentER2.dy) == 0) {
				continue;
			}

			if(currentER1.usedPixelCounter != 0 ) { // dir1 is set

				// get absolute position of cell
				int16_t absPosX1 = currentER1.x / currentER1.usedPixelCounter + x * GRID_CELL_SIZE;
				int16_t absPosY1 = currentER1.y / currentER1.usedPixelCounter + y * GRID_CELL_SIZE;

				// cell outside field -> discard
				if(fieldExtractor->getHeighestFieldCoordinate(absPosX1) > absPosY1) {

				}
				else {

					// analyze colors on both sides of the edge
					for (int16_t k = -10; k <= 10; k += 2) {
						uint16_t xPos = absPosX1 + currentER1.dx * k / currentER1.magnitude;
						uint16_t yPos = absPosY1 + currentER1.dy * k / currentER1.magnitude;

						Color color = colorMgr->getPixelColor(*image, xPos, yPos);
						colorCount[y * nrGridCellsX * 9 + x * 9 + (int16_t) color] += 1;

						// count if cell contains reddish pixels (for ball)
						if (BallExtractorLutFree::isReddishColor(image,xPos,yPos)) {
							++reddishPixels;
						}
					}
				}
			}

			if(currentER2.usedPixelCounter != 0 ) { // dir2 is set

				// get absolute position of cell
				int16_t absPosX2 = currentER2.x / currentER2.usedPixelCounter + x * GRID_CELL_SIZE;
				int16_t absPosY2 = currentER2.y / currentER2.usedPixelCounter + y * GRID_CELL_SIZE;

				// cell outside field -> discard
				if(fieldExtractor->getHeighestFieldCoordinate(absPosX2) > absPosY2) {

				}
				else {

					// analyze colors on both sides of the edge
					for (int16_t k = -10; k <= 10; k += 2) {
						int16_t xPos = absPosX2 + currentER2.dx * k / currentER2.magnitude;
						int16_t yPos = absPosY2 + currentER2.dy * k / currentER2.magnitude;

						Color color = colorMgr->getPixelColor(*image, xPos, yPos);
						colorCount[y * nrGridCellsX * 9 + x * 9 + (int16_t) color] += 1;

						// count if cell contains reddish pixels (for ball)
						if (BallExtractorLutFree::isReddishColor(image,xPos,yPos)) {
							++reddishPixels;
						}
					}
				}
			}

			getMostFrequentColors(x, y, currentCell->color1, currentCell->color2);

			currentCell->hasRed = (reddishPixels > 2)? true : false;
		}
	}


}


/**
 *  Connect the edge representers together.
 */
void GradientVectorGriding::connectEdgeRepresenters() {

	for (int16_t j = 1; j < nrGridCellsY - 1; ++j) {
		for (int16_t i = 1; i < nrGridCellsX - 1; ++i) {

			GridCell *currentCell = &gridcells[j * nrGridCellsX + i];
			EdgeRepresenter &currentER1 = currentCell->er1;
			EdgeRepresenter &currentER2 = currentCell->er2;

			int cnt = currentER1.usedPixelCounter + currentER2.usedPixelCounter;
			if (cnt == 0) { // discard cell, because there is no edge
				continue;
			}
			if (abs(currentER1.dx) + abs(currentER1.dy) == 0 && abs(currentER2.dx) + abs(currentER2.dy) == 0) {
				continue;
			}

			Color color1 = currentCell->color1;
			Color color2 = currentCell->color2;

			CvPoint absPos1 = getAbsolutePosition(currentER1, i, j);
			CvPoint absPos2 = getAbsolutePosition(currentER2, i, j);

			bool canConnectER1 = true;
			bool canConnectER2 = true;

			// check if the cell is inside the field, otherwise discard
			if(!(currentER1.usedPixelCounter != 0 && fieldExtractor->getHeighestFieldCoordinate(absPos1.x) <= absPos1.y))
				canConnectER1 = false;
			if(!(currentER2.usedPixelCounter != 0 && fieldExtractor->getHeighestFieldCoordinate(absPos2.x) <= absPos2.y))
				canConnectER2 = false;

			// connect ERs

			for (int16_t dy = -1; dy <= 1 && (canConnectER1 || canConnectER2); dy++) {
				for (int16_t dx = -1; dx <= 1 && (canConnectER1 || canConnectER2); dx++) {
					if (dx == 0 && dy == 0) { // don't connect with same cell
						continue;
					}

					GridCell *linkedCell = &gridcells[(j + dy) * nrGridCellsX + (i + dx)];
					EdgeRepresenter &linkedER1 = linkedCell->er1;
					EdgeRepresenter &linkedER2 = linkedCell->er2;

					Color linkedColor1 = linkedCell->color1;
					Color linkedColor2 = linkedCell->color2;

					// just connect, if colors match
					if(!((linkedColor1 == color1 || linkedColor1 == color2) &&
							(linkedColor2 == color1 || linkedColor2 == color2))) {
						continue;
					}

					cnt = linkedER1.usedPixelCounter + linkedER2.usedPixelCounter;

					// we found something in the cell, so try to connect to it.
					if(cnt != 0) {

						if(canConnectER1) {
							canConnectER1 = ! tryConnectingEdgeRepresenters(currentER1, linkedER1, i, j, dx, dy);
							if(canConnectER1)
								canConnectER1 = ! tryConnectingEdgeRepresenters(currentER1, linkedER2, i, j, dx, dy);
						}

						if(canConnectER2) {
							canConnectER2 = ! tryConnectingEdgeRepresenters(currentER2, linkedER1, i, j, dx, dy);
							if(canConnectER2)
								canConnectER2 = ! tryConnectingEdgeRepresenters(currentER2, linkedER2, i, j, dx, dy);
						}

					}

				}
			}


		}
	}

}

/**
 * Extracts the edges from the connected grid cells to Edge objects.
 * The edges get stored in the GradientVectorGriding::edges vector or, if they are fieldlines, in the GradientVectorGriding::fieldLineEdges vector.
 */
void GradientVectorGriding::extractEdgesFromCells() {

	// clear old stuff
	for (EdgeVector::iterator iter = edges.begin(); iter != edges.end(); ++iter) {
		Edge *e = *iter;
		delete e;
		e = 0;
	}
	edges.clear();
	edges.reserve(200);
	for(EdgeVector::iterator iter = fieldLineEdges.begin(); iter != fieldLineEdges.end(); ++iter) {
		Edge *e = *iter;
		delete e;
		e = 0;
	}
	fieldLineEdges.clear();
	fieldLineEdges.reserve(200);

	// go through the whole grid

	for (int y = 0; y < nrGridCellsY; ++y) {
		for (int x = 0; x < nrGridCellsX; ++x) {
			GridCell *currentCell = &gridcells[y * nrGridCellsX + x];

			EdgeRepresenter &currentER1 = currentCell->er1;
			EdgeRepresenter &currentER2 = currentCell->er2;

			//find the beginning of the edge.

			if( ! currentER1.connected && currentER1.usedPixelCounter != 0) {
				extractEdgeFromEdgeRepresenter(currentER1, x, y);
			}

			if( ! currentER2.connected && currentER2.usedPixelCounter != 0) {
				extractEdgeFromEdgeRepresenter(currentER2, x, y);
			}

		}
	}
}

/**
 * Try connection two ERs. Two ERs get connected, if the directions are nearly the same and
 * the difference vector's direction between the two ERs is parallel to their direction
 *
 * @param connectFrom ER to connect from
 * @param connectTo ER to connect to
 * @param cellX position in grid of connectFrom ER
 * @param cellY position in grid of connectFrom ER
 * @param dx relative position from connectTo ER to connectFrom ER
 * @param dy relative position from connectTo ER to connectFrom ER
 *
 * @return true if the connection is done, otherwise false
 */
bool GradientVectorGriding::tryConnectingEdgeRepresenters(EdgeRepresenter &connectFrom, EdgeRepresenter &connectTo, int16_t cellX, int16_t cellY, int16_t dx, int16_t dy) {

	if(connectTo.usedPixelCounter == 0 || connectFrom.connectTo != 0) {
		return false;
	}

	if (!connectTo.connected ) {

		// absolute position of linked ER
		CvPoint absPosConnectTo = getAbsolutePosition(connectTo, cellX + dx, cellY + dy);

		// cell outside field -> discard
		if(fieldExtractor->getHeighestFieldCoordinate(absPosConnectTo.x) > absPosConnectTo.y) {
			return false;
		}

		// connectFrom * connectTo = |connectFrom|*|conenctTo|*cos(theta)
		int cos = connectFrom.dx * connectTo.dx
				+ connectFrom.dy * connectTo.dy;
		// (connectFrom x connectTo)_z = |connectFrom|*|connectTo|*sin(theta) * vec(n)
		int sin = connectFrom.dx * connectTo.dy
				- connectFrom.dy * connectTo.dx;

		// absolute position of current ER
		CvPoint absPosConnectFrom = getAbsolutePosition(connectFrom, cellX, cellY);

		// difference vector in gradient direction (normal difference vector rotated by -90°)
		int16_t deltax = absPosConnectTo.y - absPosConnectFrom.y;
		int16_t deltay = -absPosConnectTo.x + absPosConnectFrom.x;

		int cosDelta = connectFrom.dx * deltax
				+ connectFrom.dy * deltay;
		int sinDelta = connectFrom.dx * deltay
				- connectFrom.dy * deltax;

		// connect if there is less than 27 degree difference between the two vectors
		// and between the connectFrom-direction and the difference vector
		// (tan = sin / cos < 0,5 => 27°)
		if (abs(sin) * 2 < cos && abs(sinDelta) * 2 < cosDelta) {
			connectTo.connected = true;
			connectFrom.to_x = dx;
			connectFrom.to_y = dy;
			connectFrom.connectTo = &connectTo;

			return true;
		}
	}

	return false;

}

/**
 * Extracts an edge starting form the given cell vector.
 *
 * @param startCell ER vector to start from
 * @param startCellX position of ER in grid
 * @param startCellY position of ER in grid
 */
void GradientVectorGriding::extractEdgeFromEdgeRepresenter(EdgeRepresenter &startER, int16_t startCellX, int16_t startCellY) {

	if (startER.usedPixelCounter == 0) {
		return;
	}

	// get the absolute position of the edge
	CvPoint absPos = getAbsolutePosition(startER, startCellX, startCellY);

	// cell outside field -> discard
	if(fieldExtractor->getHeighestFieldCoordinate(absPos.x) > absPos.y) {
		return;
	}

//	bool isFieldLine = false;
	uint8_t fieldLineHits = 0;
	Edge *edge = new Edge();

	// rotate by 90° because in the edge are the edge directions and in the cell the gradient direction
	FixedPointMath::fixedpoint dirY = -fdiv(toFixed(startER.dx), toFixed(startER.magnitude));
	FixedPointMath::fixedpoint dirX = fdiv(toFixed(startER.dy), toFixed(startER.magnitude));

	edge->dirX = dirX;
	edge->dirY = dirY;

	// accumulate colors
	for (int k = 1; k < 9; k++) {
		edge->colorCounter[k] = colorCount[startCellY * nrGridCellsX * 9 + startCellX * 9 + k];
	}

	EdgePoint ep(absPos.x, absPos.y, dirX, dirY);
	edge->linePoints.push_back(ep);

	// follow "line" to next connected cell
	int16_t dx = 0;
	int16_t dy = 0;

	EdgeRepresenter *currentER = &startER;

	while (currentER->to_x != 0 || currentER->to_y != 0) {
		if(currentER->flIndicator == FIELDLINE) {
//			isFieldLine = true;
			fieldLineHits++;
		}

		dx += currentER->to_x;
		dy += currentER->to_y;

		currentER = currentER->connectTo;

		// update direction vector
		dirY = -fdiv(toFixed(currentER->dx), toFixed(currentER->magnitude));
		dirX = fdiv(toFixed(currentER->dy), toFixed(currentER->magnitude));

		edge->dirX += dirX;
		edge->dirY += dirY;

		// update colors
		for (int16_t k = 1; k < 9; k++) {
			edge->colorCounter[k] += colorCount[(startCellY + dy) * nrGridCellsX * 9 + (startCellX + dx) * 9 + k];
		}

		// add point to edge
		CvPoint absPosLinked = getAbsolutePosition(*currentER, startCellX + dx, startCellY + dy);
		if(absPosLinked.x != -1) {
			EdgePoint ep(absPosLinked.x, absPosLinked.y, dirX, dirY);
			edge->linePoints.push_back(ep);
		}

	}

	//find the best two colors

	int maxCol1 = 0, maxCol2 = 0;
	int maxCnt1 = 0, maxCnt2 = 0;
	for (int k = 1; k < 9; k++) { // ignoring unknown color
		if ((maxCol1 != Ball && maxCnt1 < edge->colorCounter[k])
				|| (k == Ball && edge->colorCounter[k] > 0)) {
			if (maxCnt1 > maxCnt2) {
				maxCnt2 = maxCnt1;
				maxCol2 = maxCol1;
			}
			maxCnt1 = edge->colorCounter[k];
			maxCol1 = k;
		} else if (maxCnt2 < edge->colorCounter[k]) {
			maxCnt2 = edge->colorCounter[k];
			maxCol2 = k;
		}

	}

	//at least avg 2 pix from the color should be seen on the edge.
	if (maxCnt1 < (int16_t) edge->linePoints.size() * 2 && maxCol1 != Ball)
		maxCol1 = 0;
	if (maxCnt2 < (int16_t) edge->linePoints.size() * 2)
		maxCol2 = 0;

	edge->mostFrequentColor1 = (Color) maxCol1;
	edge->mostFrequentColor2 = (Color) maxCol2;

	// calculate average direction of edge
	edge->dirX = fdiv(edge->dirX, toFixed(edge->linePoints.size()));
	edge->dirY = fdiv(edge->dirY, toFixed(edge->linePoints.size()));

	if(fieldLineHits > edge->linePoints.size() / 2) {
		fieldLineEdges.push_back(edge);
	}
	else {
		edges.push_back(edge);
	}

	startER.edge = edge;

}

/**
 * Finds the best two colors for the specified cell in the colorcount-grid
 * @param x
 * @param y
 * @param color1
 * @param color2
 */
void GradientVectorGriding::getMostFrequentColors(int16_t cellX, int16_t cellY, Color &color1, Color &color2) {

	int maxCol1 = 0, maxCol2 = 0;
	int maxCnt1 = 0, maxCnt2 = 0;

	for (int k = 1; k < 9; k++) { // ignoring unknown color
		int count = colorCount[cellY * nrGridCellsX * 9 + cellX * 9 + k];

		if ((maxCol1 != Ball && maxCnt1 < count) || (k == Ball && count > 0)) {
			if (maxCnt1 > maxCnt2) {
				maxCnt2 = maxCnt1;
				maxCol2 = maxCol1;
			}
			maxCnt1 = count;
			maxCol1 = k;
		} else if (maxCnt2 < count) {
			maxCnt2 = count;
			maxCol2 = k;
		}

	}

	color1 = (Color) maxCol1;
	color2 = (Color) maxCol2;
}

/**
 * Marks cells as possible field line cell, if they are green/white edges and have a corresponding parallel edge.
 */
void GradientVectorGriding::markPossibleFieldlineCells() {

	for(int16_t y = 1; y < nrGridCellsY - 1; ++y) {
		for(int16_t x = 1; x < nrGridCellsX - 1; ++x) {

			GridCell *cell = &gridcells[y * nrGridCellsX + x];
			EdgeRepresenter &er1 = cell->er1;
			EdgeRepresenter &er2 = cell->er2;

			if(er1.flIndicator != UNKNOWN && er2.flIndicator != UNKNOWN) {
				continue;
			}

			if(er1.usedPixelCounter != 0 && er2.usedPixelCounter != 0 &&
					isParallel(er1, er2, x, y, 0,0) && ((cell->color1 == Field || cell->color1 == White) && (cell->color2 == Field || cell->color2 == White))) {

				er1.flIndicator = FIELDLINE;
				er2.flIndicator = FIELDLINE;
			}
			else {

				for(int16_t j = -1; j <= 1; ++j) {
					for(int16_t i = -1; i <= 1; ++i) {
						if(i == 0 && j == 0) continue;

						GridCell *otherCell = &gridcells[(y + j) * nrGridCellsX + (x + i)];
						EdgeRepresenter &otherER1 = otherCell->er1;
						EdgeRepresenter &otherER2 = otherCell->er2;

						if( ! (((cell->color1 == Field || cell->color1 == White) && (cell->color2 == Field || cell->color2 == White)) &&
								((otherCell->color1 == Field || otherCell->color1 == White) && (otherCell->color2 == Field || otherCell->color2 == White)))) {
							continue;
						}

						if(otherER1.usedPixelCounter != 0 && isParallel(er1, otherER1, x, y, i, j)) {
							er1.flIndicator = FIELDLINE;
							otherER1.flIndicator = FIELDLINE;
						}
						if (otherER2.usedPixelCounter != 0 && isParallel(er1, otherER2, x, y, i, j)) {
							er1.flIndicator = FIELDLINE;
							otherER2.flIndicator = FIELDLINE;
						}
						if(otherER1.usedPixelCounter != 0 && isParallel(er2, otherER1, x, y, i, j)) {
							er2.flIndicator = FIELDLINE;
							otherER1.flIndicator = FIELDLINE;
						}
						if(otherER2.usedPixelCounter != 0 && isParallel(er2, otherER2, x, y, i, j)) {
							er2.flIndicator = FIELDLINE;
							otherER2.flIndicator = FIELDLINE;
						}

					}
				}

			}


		}
	}
}

/**
 * Checks if the given ERs are parallel
 *
 * @param er1
 * @param er2
 * @param cellX
 * @param cellY
 * @param dx
 * @param dy
 * @return
 */
bool GradientVectorGriding::isParallel(EdgeRepresenter &er1, EdgeRepresenter &er2, int16_t cellX, int16_t cellY, int16_t dx, int16_t  dy) {
	if(er1.usedPixelCounter == 0 || er2.usedPixelCounter == 0)
		return false;

	FixedPointMath::fixedpoint angle1 = fatan2(toFixed(er1.dy), toFixed(er1.dx));
	FixedPointMath::fixedpoint angle2 = fatan2(toFixed(er2.dy), toFixed(er2.dx));

	int angleDifference = Math::angleDiff(toInt(angle1), toInt(angle2));

	if(dx == 0 && dy == 0) { // same cell, so just check if gradients are parallel
		return abs(angleDifference - 180) < 27 ||  angleDifference < 27;
	}

	// different cells: parallel, if the connecting vector is parallel to the gradient (= perpendicular to the edge direction)
	CvPoint absPos1 = getAbsolutePosition(er1, cellX, cellY);
	CvPoint absPos2 = getAbsolutePosition(er2, cellX + dx, cellY + dy);

	int16_t deltay = absPos2.y - absPos1.y;
	int16_t deltax = absPos2.x - absPos1.x;

	FixedPointMath::fixedpoint differenceVectorAngle = fatan2( toFixed(deltay), toFixed(deltax));
	int angleDifference2 = Math::angleDiff(toInt(angle1), toInt(differenceVectorAngle));

	return (angleDifference < 27 || abs(angleDifference - 180) < 27)
			&& (abs(angleDifference2 - 180) < 27 || angleDifference2 < 27);
}

/**
 * Get the edges (if present) corresponding to a gridcell.
 * @param x x-coordinate of grid cell
 * @param y y-coordinate of grid cell
 * @param edge1 holds the pointer to edge1 found in this cell (or NULL if nothing was found)
 * @param edge2 holds the pointer to edge2 found in this cell (or NULL if nothing was found)
 */
void GradientVectorGriding::getEdgesInGridCell(uint8_t x, uint8_t y, Edge *edge1, Edge *edge2) {
	GridCell *cell = getGridCell(x, y);

	if(cell == NULL) {
		edge1 = NULL;
		edge2 = NULL;
		return;
	}

	EdgeRepresenter &er1 = cell->er1;
	EdgeRepresenter &er2 = cell->er2;

	CvPoint absPos1 = getAbsolutePosition(er1, x, y);
	CvPoint absPos2 = getAbsolutePosition(er2, x, y);


	if(absPos1.x != -1) {
		edge1 = Edge::getEdgeWithPoint(absPos1.x, absPos1.y, edges);
		if(edge1 == NULL)
			edge1 = Edge::getEdgeWithPoint(absPos1.x, absPos1.y, fieldLineEdges);
	}
	else {
		edge1 = NULL;
	}

	if(absPos2.x != -1) {
		edge2 = Edge::getEdgeWithPoint(absPos2.x, absPos2.y, edges);
		if(edge2 == NULL)
			edge2 = Edge::getEdgeWithPoint(absPos2.x, absPos2.y, fieldLineEdges);
	}
	else {
		edge2 = NULL;
	}

}
