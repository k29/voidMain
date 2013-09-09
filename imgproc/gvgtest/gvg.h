#ifndef GRID_H_
#define GRID_H_

#include "image.h"
#include "color.h"
#include "edge.h"
#include "objectExtractor_field.h"
#include "debug.h"

/**
 * @{
 * @ingroup vision
 */

/**
 * Defines the possible values for a grid cell concerning field lines.
 */
typedef enum {
	UNKNOWN,   //!< UNKNOWN
	FIELDLINE, //!< FIELDLINE
	NOFIELDLINE//!< NOFIELDLINE
} FieldLineIndicator;

/**
 * Struct holding the information of one specific Edge Representer (ER) in a grid cell
 */
typedef struct EdgeRepresenter {
	int16_t x, y;                       //!< relative cell coordinates (accumulated)
	int16_t dx, dy;                     //!< gradient direction in the cell (accumulated)
	uint16_t magnitude;                 //!< magnitude (accumulated)
	uint16_t usedPixelCounter;          //!< counts the good pixels in the cell
	int16_t to_x, to_y;                 //!< connect this cell to ER at (x+to_x,y+to_y)
	struct EdgeRepresenter *connectTo;  //!< connect this cell to conenctTo ER (same as to_x, to_y)
	FieldLineIndicator flIndicator;     //!< indicates a field line
	bool connected;                     //!< true, if another ER links to me
	Edge* edge;                         //!< pointer to an edge, if present in ER
} EdgeRepresenter;

/**
 * Struct holding the information of one grid cell
 */
typedef struct {
	EdgeRepresenter er1, er2;  //!< two possible ERs
	Color color1;              //!< most frequent color in cell
	Color color2;              //!< second most frequent color in cell
	bool hasRed;               //!< true if cell contains some reddishPixels
} GridCell;

static const uint16_t GRID_CELL_SIZE = 16;  //!< defines the side-length of an grid cell. So the grid cell is 16x16 pixels big.

/**
 * The GradientVectorGriding class provides all functionality to extract the edges by means of gradient analysis of an image.
 *
 * The algorithm used is called Gradient Vector Griding.
 *
 * The image is divided into 16x16 sized grid cells. For each cell up to two edge representers are calculated,
 * which describe the passing of an edge through the cell.
 *
 * Edge representers, which have the same colors on both of their sides can get connected to form an edge trace.
 *
 *
 * @ingroup vision
 */
class GradientVectorGriding {
public:
	GradientVectorGriding();
	virtual ~GradientVectorGriding();

	/**
	 * sets the image
	 * @param img
	 */
	inline void setImage(IMAGETYPE *img) {
		image = img;
	}

	/**
	 * sets the color manager
	 * @param cm
	 */
	inline void setColorMgr(ColorManager *cm) {
		colorMgr = cm;
	}

	/**
	 * returns the edges (without field lines)
	 * @return
	 */
	inline EdgeVector& getEdges() {
		return edges;
	}

	/**
	 * returns the field line edges
	 * @return
	 */
	inline EdgeVector& getFieldLineEdges() {
		return fieldLineEdges;
	}

	/**
	 * Returns the grid cell on (x,y) index
	 * @param x
	 * @param y
	 * @return
	 */
	inline GridCell* getGridCell(uint8_t x, uint8_t y) {
		if (x >= nrGridCellsX || y >= nrGridCellsY) {
			WARNING("Trying to read non-existing cell (%d,%d)", x,y);
			return NULL;
		}
		return &gridcells[y * nrGridCellsX + x];
	}

	void getEdgesInGridCell(uint8_t x, uint8_t y, Edge *edge1, Edge *edge2);

	/**
	 * Get absolute image position of the EdgeRepresenter dir in the given grid-cell.
	 * @param edgeRepresenter ER we wan't to know the image position
	 * @param cellX position of ER in grid (x)
	 * @param cellY position of ER in grid (y)
	 * @return cvPoint containing the absolute image position or (-1, -1) on error
	 */
	inline CvPoint getAbsolutePosition(const EdgeRepresenter &edgeRepresenter, uint8_t cellX, uint8_t cellY) {
		if(edgeRepresenter.usedPixelCounter != 0) {
			int16_t absPosX = edgeRepresenter.x / edgeRepresenter.usedPixelCounter + cellX * GRID_CELL_SIZE;
			int16_t absPosY = edgeRepresenter.y / edgeRepresenter.usedPixelCounter + cellY * GRID_CELL_SIZE;
			return cvPoint(absPosX, absPosY);
		}
		else {
			return cvPoint(-1, -1);
		}
	}

	void clear();
	void init();
	void run();


protected:

	uint16_t centerX;
	uint16_t centerY;
	int16_t radius;
	CvRNG range;

	/**
	 * 2D-Array storing the grid cells. Y-dimension in first dimension, X in second.
	 * (Access: of gridcells[y][x] = colorCount(y * nrGridCellsX + x) )
	 */
	GridCell *gridcells;
	uint16_t nrGridCellsX;  //!< grid cells per image column
	uint16_t nrGridCellsY;  //!< grid cells per image row
	uint16_t stepSizeX;     //!< the stepsize in x direction between two image columns used for grid
	uint16_t stepSizeY;     //!< the stepsize in y direction between two image rows used for grid

	/**
	 *  3D-Array storing the number a color was counted in the specific grid cell.
	 *  First dimension: Y index of cell, second dimension: X index of cell
	 *  Third dimension: color. Indices corresponding to Color enum
	 *  (Access: of colorCount[y][x][c] = colorCount(y * nrGridCellsX * 9 + x * 9 + c) )
	 */
	int16_t *colorCount;

	uint16_t reddishPixels;

	EdgeVector edges;               //!< vector storing the current edges (without field lines)
	EdgeVector fieldLineEdges;      //!< vector storing the current field line edges
	FieldExtractor *fieldExtractor; //!< pointer to the field extractor

	ColorManager *colorMgr;
	IMAGETYPE *image;

	void createGridCells();
	void analyzeColors();
	void connectEdgeRepresenters();
	void extractEdgesFromCells();

	void getMostFrequentColors(int16_t cellX, int16_t cellY, Color &color1, Color &color2);
	bool tryConnectingEdgeRepresenters(EdgeRepresenter &connectFrom, EdgeRepresenter &connectTo, int16_t cellX, int16_t cellY, int16_t dx, int16_t dy);
	void extractEdgeFromEdgeRepresenter(EdgeRepresenter &startER, int16_t startCellX, int16_t startCellY);

	void markPossibleFieldlineCells();
	bool isParallel(EdgeRepresenter &er1, EdgeRepresenter &er2, int16_t cellX, int16_t cellY, int16_t dx, int16_t  dy);
};

/**
 * @}
 */

#endif /* GRID_H_ */
