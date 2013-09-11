#ifndef OBJECTEXTRACTOR_FIELD_H_
#define OBJECTEXTRACTOR_FIELD_H_

#include "image.h"
#include "color.h"
#include "edge.h"

#include "comm/protobuf/msg_vision.pb.h"

/**
 * This class defines the field contour extraction algorithm.
 *
 * @ingroup vision
 */
class FieldExtractor {
public:
	FieldExtractor();
	virtual ~FieldExtractor();

	void clear();
	bool extract();

	/**
	 * Returns the unfiltered field contour points
	 * @return
	 */
	inline std::vector<CvPoint>& getOriginalFieldContour() {
		return originalFieldContour;
	}

	/**
	 * Returns the filtered field contour points
	 * @return
	 */
	inline std::vector<CvPoint>& getFilteredFieldContour() {
		return filteredFieldContour;
	}

	/**
	 * Get for the x value the corresponding y value where the field ends in the image,
	 * that means y values smaller the the return value (= higher in the image)) lies outside the field
	 * @param x
	 * @return
	 */
	inline int16_t getHeighestFieldCoordinate(int16_t x) {
		if(x < 0 || x >= image->getImageWidth())
			return 0;
		return fieldContour[x];
	}

	inline void setImage(IMAGETYPE *img) {
		image = img;
	}
	inline void setColorManager(ColorManager *cm) {
		colorMgr = cm;
	}
	inline void setProtobufExtractorStatus(de::fumanoids::message::ObjectExtractorStatus *status) {
		protobufExtractorStatus = status;
	}

	static const uint8_t X_STEP = 16; //!<horizontal distance between scanlines
	static const uint8_t Y_STEP = 5;  //!<stepsize along the scanline

protected:
	IMAGETYPE *image;
	ColorManager *colorMgr;

	de::fumanoids::message::ObjectExtractorStatus *protobufExtractorStatus;

	std::vector<CvPoint> originalFieldContour;  //!< unfiltered field contour
	std::vector<CvPoint> filteredFieldContour;  //!< filtered field contour
	int16_t *fieldContour;                      //!< field contour function. fieldContour[x] = heighest image pixel, which belongs to the field
	uint16_t fieldContourSize;                  //!< field contour size (corresponds to the image width)

	void scanlines();
	void filterFieldContour();
	void interpolateFieldContour();
	bool isFieldPixel(int16_t x, int16_t y);
};

#endif /* OBJECTEXTRACTOR_FIELD_H_ */
