#ifndef OBJECTEXTRACTOR_FIELDLINE_H_
#define OBJECTEXTRACTOR_FIELDLINE_H_

#include "objectExtractor_base.h"
#include <inttypes.h>
#include <vector>
#include <list>

#include "math/Fixed.h"
#include "boundingBox.h"
#include "edge.h"
#include "thread.h"
#include "object.h"

/**
 * @{
 * @ingroup vision
 */

/**
 *
 */
typedef enum {
	Nothing,            //!< Nothing
	XCrossingFieldLine, //!< XCrossingFieldLine
	TCrossingFieldLine, //!< TCrossingFieldLine
	LCrossingFieldLine  //!< LCrossingFieldLine
} FieldLineFeatureType;

/**
 * Defines a field line feature
 *
 * @ingroup vision
 */
class FieldLineFeature : public PositionImage {
public:
	FieldLineFeature(int16_t _x, int16_t _y) :
		PositionImage(_x, _y), type(Nothing) { }

	FieldLineFeatureType type;   //!< type of feature
};

/**
 * Field line feature extractor
 *
 * @ingroup vision
 */
class FieldLineFeatureExtractor: public Extractor {
public:
	FieldLineFeatureExtractor();
	virtual ~FieldLineFeatureExtractor();

	void clear();
	bool extract(const EdgeVector &edges);

	inline ClusterObject* getFieldLineCluster() {
		return fieldLineCluster;
	}

	inline EdgeVector& getFieldLineEdges() {
		return fieldLineEdges;
	}

	inline std::vector<FieldLineFeature> getLineFeatures() {
		return lineFeatures;
	}

	inline std::list<BoundingBox>& getGroupedFeatures() {
		return groupedFeatures;
	}

	inline std::vector<FieldLineFeature>& getRawLineFeatures() {
		return rawLineFeatures;
	}


protected:
	ClusterObject *fieldLineCluster;

	EdgeVector fieldLineEdges;                      //!< all field line edges
	std::vector<FieldLineFeature> lineFeatures;     //!< field line features

	std::vector<FieldLineFeature> rawLineFeatures;  //!< raw (unfiltered) field line features
	std::list<BoundingBox> groupedFeatures;         //!< grouping of raw field line features
	FixedPointMath::fixedpoint *lineDirectionMap;   //!< map of line directions corresponding to the grid
	uint16_t nrCellsX;
	uint16_t nrCellsY;

	void withHistogram(const EdgeVector &edges);

	void verifyFieldLineFeatures();

	void createFieldLineMap(const EdgeVector &edges);
	int countFieldLineCellsInDirection(int16_t startX, int16_t startY, FixedPointMath::fixedpoint dirX, FixedPointMath::fixedpoint dirY);

	BoundingBox convertLineFeatureToBoundingBox(const FieldLineFeature &lineFeature);
};

/**
 * @}
 */

#endif /* OBJECTEXTRACTOR_FIELDLINE_H_ */
