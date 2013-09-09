#ifndef OBJECTEXTRACTOR_BASE_H_
#define OBJECTEXTRACTOR_BASE_H_

#include "image.h"
#include "color.h"
#include "boundingBox.h"
#include "edge.h"

#include "comm/protobuf/msg_vision.pb.h"

/**
 * Abstract class providing the necessary information needed for the specific object extractors.
 *
 * @ingroup vision
 */
class Extractor {
public:
	virtual ~Extractor() {	}

	/**
	 * Clears the data of the extractor
	 * @return
	 */
	virtual void clear() = 0;

	/**
	 * Extracts the object from the given edges
	 * @param edges
	 * @return bool, if we found it, otherwise false
	 */
	virtual bool extract(const EdgeVector &edges) = 0;

	inline virtual void setImage(IMAGETYPE *img) {
		image = img;
	}

	inline virtual void setColorManager(ColorManager *cm) {
		colorMgr = cm;
	}

	inline virtual void setProtobufExtractorStatus(de::fumanoids::message::ObjectExtractorStatus *status) {
		protobufExtractorStatus = status;
	}

protected:
	IMAGETYPE *image;
	ColorManager *colorMgr;

	de::fumanoids::message::ObjectExtractorStatus *protobufExtractorStatus;

	virtual void getGroundPoint(const BoundingBox &box, CvPoint &lowestPoint);
	BoundingBox regionGrowing(int16_t seedX, int16_t seedY, Color color, int16_t maxWidth, int16_t maxHeight);
	void createBoundingBoxes(const EdgeVector& edges, std::list<BoundingBox> &boundingBoxes, int16_t distanceX, int16_t distanceY);
	void getColorsInNeighbourhood(int16_t x, int16_t y, uint8_t *colorArray);

};

#endif /* OBJECTEXTRACTOR_BASE_H_ */
