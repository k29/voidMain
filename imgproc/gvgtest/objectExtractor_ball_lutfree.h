/*
 *  objectExtractor_ball_lutfree.h
 *
 *  Created on: 01.05.2011
 *      Author: lisa
 */

#ifndef OBJECTEXTRACTOR_BALL_LUTFREE_H_
#define OBJECTEXTRACTOR_BALL_LUTFREE_H_

#include "object.h"
#include "image.h"
#include "gvg.h"
#include "position.h"
#include "math/fixedPointMath.h"

/**
 * @{
 * @ingroup vision
 */

/**
 * A struct to compare two CvPoints with one another. Can be used to sort
 * a list of points or to build a map with points as keys.
 */
struct CvPointCompare
{
	/**
	 * Compares two points based on their x coordinates first. If they are
	 * equal the y coordinates are used.
	 * @param p1 - first point
	 * @param p2 - second point
	 * @return true, if p1 is smaller than p2, i.e. has smaller coordinates (x,y)
	 */
   bool operator() 	(const CvPoint& p1, const CvPoint& p2) const
	{
		if(p1.x != p2.x) return p1.x < p2.x;
			return p1.y < p2.y;
	}
};

/**
 * A class to represent a ball candidate.
 *
 * Rectangular regions that should be processed further become a PossibleBall
 * defined by the coordinates of the upper left corner (x,y), a width and height
 * and an assigned error which reflects the likelihood of this region to be a
 * ball. Lower error means higher likelihood.
 */
class PossibleBall {

public:
	PossibleBall() : upperLeftX(0), upperLeftY(0), width(0), height(0),
					 error(-1), meanHue(-1), processed(false){};

	PossibleBall(uint16_t upperLeftX, uint16_t upperLeftY, uint16_t width,
			uint16_t height, int16_t error, bool processed=false) :
	  upperLeftX(upperLeftX),
	  upperLeftY(upperLeftY),
	  width(width),
	  height(height),
	  error(error),
	  meanHue(-1),
	  processed(processed){};

	~PossibleBall() {};

	uint16_t upperLeftX; /// x coordinate of the upper left corner
	uint16_t upperLeftY; /// y coordinate of the upper left corner
	uint16_t width;      /// width of the rectangular region
	uint16_t height;     /// height of the rectangular region
	int16_t error;       /// the lower the error the better
	int16_t meanHue;     /// mean hue value of the region
	bool processed;      /// indicates if the ball was already processed

	/**
	 * @return The center point of the ball candidate in absolute image
	 * coordinates.
	 */
	inline CvPoint getCenter() const {
		return cvPoint(upperLeftX + width/2, upperLeftY + height/2);
	}

	/**
	 * @return The point that touches the ground, i.e. the
	 * highest y coordinate and the middle of the corner's x coordinates.
	 */
	inline CvPoint getBasePoint() const {
		CvPoint base;
		base.x = upperLeftX + width/2;
		base.y = upperLeftY + height;
		return base;
	}

	/**
	 * @return The distance between the robot's current position and the base
	 * point of this PossibleBall in cm.
	 */
	inline int16_t getRelativeDistance() const {
		CvPoint base = getBasePoint();
		PositionImage pos(base.x,base.y);
		return pos.translateToRelative().getAsPolar().getR();
	}

	/**
	 * Compares two PossibleBalls according to their error.
	 * @return True, if ball1 has a smaller error than ball2 or if ball1 is
	 * closer to the robot and both error rates are close.
	 */
	inline static bool compare(const PossibleBall ball1, const PossibleBall ball2) {
		int16_t err = ::max(ball1.error,ball2.error);

		if(err < ALLOWED_BALL_ERROR-10
				&& abs(ball1.error-ball2.error)<15) {
			// sort closer balls to the front
			return ball1.getRelativeDistance() < ball2.getRelativeDistance();
		}
		return ball1.error < ball2.error;
	}

	/**
	 * Detects if two PossibleBalls collide, i.e. if their regions overlap.
	 * Two rectangles collide if their combined width and height is larger
	 * than the difference between the rightmost and leftmost corner of
	 * the bounding box around both rectangles.
	 * @param ball1
	 * @param ball2
	 * @return True if the two balls collide, otherwise false.
	 */
	inline static bool detectCollision(const PossibleBall ball1, const PossibleBall ball2){
		uint16_t ball_tr, ball_tl, ball_br, ball_bl;

		if(ball1.upperLeftX > ball2.upperLeftX) {
			ball_tl = ball2.upperLeftX;
			ball_tr = ball1.upperLeftX + ball1.width;
		} else {
			ball_tl = ball1.upperLeftX;
			ball_tr = ball2.upperLeftX + ball2.width;
		}

		if(ball1.upperLeftY > ball2.upperLeftY) {
			ball_bl = ball2.upperLeftY;
			ball_br = ball1.upperLeftY + ball1.height;
		} else {
			ball_bl = ball1.upperLeftY;
			ball_br = ball2.upperLeftY + ball2.height;
		}


		if (ball1.width+ball2.width > ball_tr-ball_tl
				&& ball1.height+ball2.height > ball_br-ball_bl)
			return true;

		return false;
	}

	/// Maximal allowed error. Balls with higher errror are discarded.
	static const int16_t ALLOWED_BALL_ERROR = 65;

};

/*------------------------------------------------------------------------------------------------*/

/**
 * Class to search for the ball in the image.
 * For different parts of the image, different techniques can used: One can
 * either scan the image line-by-line with a step size that depends on the
 * distance to the robot or use the edges to determine cells that might
 * contain a ball.
 *
 * ----------------------
 * |    Small ball      |
 * |--------------------| 140-180 cm
 * |    Middle Ball     |
 * |--------------------| 35-60 cm
 * |    Large ball      |
 * ----------------------
 */
class BallExtractorLutFree {

public:
	BallExtractorLutFree();
	~BallExtractorLutFree();

	/// Map to store grid cells which were processed during extract()
	std::map<CvPoint, uint8_t, CvPointCompare> cells;

	bool extract();

	/**
	 * Returns the ball. The ball's rectangle object has coordinates (-1,-1)
	 * if no ball was found in the frame.
	 * @return current ball
	 */
	inline RectangleObject* getBall() {
		return ball;
	}

	/**
	 * @return True, if the ball is currently seen
	 */
	inline bool ballSeen() {
		return ball->rectangle.x != -1;
	}

	/**
	 * @return The list of all ball candidates, even those with high error.
	 */
	inline std::vector<PossibleBall> const & getPossibleBalls() const {
		return possibleBalls;
	}

	/**
	 * Sets the image to be processed.
	 * @param img
	 */
	inline void setImage(IMAGETYPE* img) {
		image = img;
	}

	/**
	 * Determines if the pixel (x,y) might belong to a ball.
	 *
	 * This method is a little bit faster than isBallPixel(), because it only
	 * considers pixels that have a reddish color before checking them more
	 * thoroughly.
	 * @param x The x coordinate of the pixel
	 * @param y The y coordinate of the pixel
	 * @return True, if (x,y) has high possibility to belong to a ball
	 */
	inline bool isBallPixelFast(uint16_t x, uint16_t y) {
		uint16_t likelihood = 0;
		return isBallPixelFast(x,y,likelihood);
	}

	/**
	 * Determines if the pixel (x,y) might belong to a ball.
	 *
	 * This method is a little bit faster than isBallPixel(), because it only
	 * considers pixels that have a reddish color before checking them more
	 * thoroughly.
	 * @param x  The x coordinate of the pixel
	 * @param y  The y coordinate of the pixel
	 * @param likelihood Is set to the likelihood that this pixel belongs to
	 *                   a ball. Higher values mean higher likelihood.
	 * @return True, if (x,y) has high possibility to belong to a ball
	 */
	inline bool isBallPixelFast(uint16_t x, uint16_t y, uint16_t& likelihood) {

		if(!isReddishColor(image,x,y)) {
			likelihood = 0;
			return false;
		}

		return isBallPixel(x,y,likelihood);
	}

	/**
	 * Determines if the pixel (x,y) might belong to a ball.
	 *
	 * @param x  The x coordinate of the pixel
	 * @param y  The y coordinate of the pixel
	 * @return True, if (x,y) has high possibility to belong to a ball
	 */
	inline bool isBallPixel(uint16_t x, uint16_t y) {
		uint16_t likelihood = 0;
		return isBallPixel(x,y,likelihood,false);
	}

	/**
	 * Determines if the pixel (x,y) might belong to a ball.
	 *
	 * @param x  The x coordinate of the pixel
	 * @param y  The y coordinate of the pixel
	 * @param likelihood Is set to the likelihood that this pixel belongs to
	 *                   a ball. Higher values mean higher likelihood.
	 * @param lkl Optional, defines if likelihood should be computed or not.
	 * @return True, if (x,y) has high possibility to belong to a ball
	 */
	inline bool isBallPixel(uint16_t x, uint16_t y, uint16_t& likelihood, bool lkl=true) {
		uint16_t hue;
		uint8_t sat,val;
		image->getPixelAsHSV(x,y,hue,sat,val);

		return isBallPixel(x,y, hue,sat,val,likelihood);
	}

	/**
	 * Determines if the pixel (x,y) with the given h,s,v values might
	 * belong to a ball.
	 *
	 * @param x  The x coordinate of the pixel
	 * @param y  The y coordinate of the pixel
	 * @param hue The hue from HSV color space
	 * @param sat The saturation from HSV
	 * @param val The value from HSV
	 * @return True, if (x,y) has high possibility to belong to a ball
	 */
	inline bool isBallPixel(int16_t x, uint16_t y, uint16_t hue, uint8_t sat, uint8_t val) {
		uint16_t likelihood = 0;
		return isBallPixel(x,y, hue,sat,val,likelihood, false);
	}

	/**
	 * Determines if the pixel (x,y) with the given h,s,v values might
	 * belong to a ball.
	 *
	 * @param x  The x coordinate of the pixel
	 * @param y  The y coordinate of the pixel
	 * @param hue The hue from HSV color space
	 * @param sat The saturation from HSV
	 * @param val The value from HSV
	 * @param likelihood Is set to the likelihood that this pixel belongs to
	 *                   a ball. Higher values mean higher likelihood.
	 * @param lkl Optional, defines if likelihood should be computed or not.
	 * @return True, if (x,y) has high possibility to belong to a ball
	 */
	inline bool isBallPixel(int16_t x, uint16_t y, uint16_t hue, uint8_t sat, uint8_t val, uint16_t& likelihood, bool lkl=true) {
		++pixels;

		int16_t hueTemp = hue;
		if (hue > 180) hueTemp -= HUE_MAX;

		if ((hueTemp >= HUE_THRESH_LOW && hueTemp <= HUE_THRESH_HIGH)
				&& ((sat>40 && val>55 && val<80) || (sat>60 && val>40))) {
			if (lkl) {
				likelihood = gaussianWeight(hueTemp,currBallHueMean,HUE_BALL_STD);
			}
			return true;
		}
		likelihood = 0;
		return false;
	}

	/**
	 * Returns true if the given cell has unconnected edges, which indicates
	 * that they don't belong to a field line and thus might be ball edges.
	 * @param cell - grid cell to check
	 * @return True, if cell has an edge that might belong to a ball.
	 */
	inline bool hasPossibleBallEdge(GridCell* cell) {
		return getPossibleBallEdge(cell) != NULL;
	}

	/**
	 * An edge in the given cell that is unconnected, indicates that it doesn't
	 * belong to a field line and thus might be ball edge.
	 * @param cell - grid cell to check
	 * @return A possible ball edge or NULL.
	 */
	inline Edge* getPossibleBallEdge(GridCell* cell) {
		if (cell == NULL) return false;

		EdgeRepresenter& cv1 = cell->er1;
		EdgeRepresenter& cv2 = cell->er2;

		if(cv1.edge != NULL && cv1.edge->linePoints.size() == 1)
			return cv1.edge;
		if(cv2.edge != NULL && cv2.edge->linePoints.size() == 1)
			return cv2.edge;

		return NULL;
	}

	/**
	 * @return The top image part's lower boundary in pixels
	 */
	inline uint16_t getTopInPixels() {
		if(&topInImg == 0) return 0;
		int16_t pos = topInImg.getY() - (topInImg.getY()&(GRID_CELL_SIZE-1));
		return ::min(::max(pos,0), image->getImageHeight()-1);
	}

	/**
	 * @return The middle image part's lower boundary in pixels
	 */
	inline uint16_t getMiddleInPixels() {
		if(&middleInImg == 0) return 0;
		int16_t pos = middleInImg.getY() - (middleInImg.getY()&(GRID_CELL_SIZE-1));
		return ::min(::max(pos,0), image->getImageHeight()-1);
	}

	/**
	 * Checks if pixel (x,y) in the given image has a reddish color.
	 * Checking should be done directly in the pixels's color space
	 * because this method is called rather often.
	 * @param image
	 * @param x
	 * @param y
	 * @return True, if (x,y) has a reddish color.
	 */
	static inline bool isReddishColor(IMAGETYPE* image, uint16_t x, uint16_t y) {
		#if defined IMAGEFORMAT_YUV422
			uint8_t c1,c2,c3;
			image->getPixelAsYUV(x,y,&c1,&c2,&c3);
			return isReddishColorByYUV(c1,c2,c3);
		#elif defined IMAGEFORMAT_RGB or defined IMAGEFORMAT_BAYER
			uint8_t c1,c2,c3;
			image->getPixelAsRGB(x,y,&c1,&c2,&c3);
			return isReddishColorByRGB(c1,c2,c3);
		#else
		#error "Unknown image type."
		#endif
	}

	/**
	 * Returns true if r,g,b values form a reddish color
	 * @param r The red value
	 * @param g The green value
	 * @param b the blue value
	 */
	static inline bool isReddishColorByRGB(uint8_t r, uint8_t g, uint8_t b) {
		uint8_t maxRGB = ::max(r,g,b);
		uint8_t minRGB = ::min(r,g,b);
		return (r > g+10 && r > b+10 && maxRGB > 80 && maxRGB-minRGB > 10);
	}

	/**
	 * Returns true if y,u,v values form a reddish color
	 * @param y The luma component
	 * @param u The first chrominance component
	 * @param v The second chrominance component
	 */
	static inline bool isReddishColorByYUV(uint8_t y, uint8_t u, uint8_t v) {
		uint16_t sat = abs(u-128) + abs(v-128);
		return (u<140 && v>140 && ((y>80 && y<128 && sat>45)
				|| (y>128 && y<200 && sat>65)));
	}

#if defined ROBOT2011
	/// Lower bound of top in image in centimeters
	static const uint16_t TOP_BORDER = 140;
	/// Lower bound of middle in image in centimeters
	static const uint16_t MIDDLE_BORDER = 35;
#else
	/// Lower bound of top in image in centimeters
	static const uint16_t TOP_BORDER = 180;
	/// Lower bound of middle in image in centimeters
	static const uint16_t MIDDLE_BORDER = 60;
#endif

protected:
	IMAGETYPE* image;                           /// the image to work on
	RectangleObject *ball;   					/// the ball object
	std::vector<PossibleBall> possibleBalls;	/// vector of potential balls

	PositionImage topInImg;
	PositionImage middleInImg;

	uint8_t currBallHueMean;
	uint16_t frames;
	uint32_t avgTime;
	uint32_t pixels;

	bool findPossibleBalls();
	void findBallsInTop();
	void findBallsInMiddle();
	void findBallsInBottom();

	void scanForBall(uint16_t yStart, uint16_t yEnd, bool useEdges=false);
	bool findSeedPointInCell(uint16_t x, uint16_t y, uint8_t interestRegion,
			uint8_t windowSize, uint8_t x_step, uint8_t y_step);
	bool findSeedPointAtPixel(uint16_t x, uint16_t y, uint8_t windowSize);

	void combineBalls();
	void adjustBallRegion();
	void rateBalls();
	int isSimilar(CvPoint newPoint);
	int16_t weighRegion(PossibleBall& ball);

	/**
	 * Combine, grow and rate the balls in the \c possibleBalls list.
	 */
	inline void processPossibleBalls() {
		combineBalls();
		adjustBallRegion();
		rateBalls();
	}

	/**
	 * Computes gaussian weight for x with the given mean and standard deviation sigma.
	 * @return Gaussian weight as integer in range [0,100]
	 */
	static int gaussianWeight(int x, int mean, float sigma) {
		using namespace FixedPointMath;

		fixedpoint sigmaSqr = fmul(toFixed(sigma), toFixed(sigma));
		fixedpoint fac = toFixed(1);
		fixedpoint dist = fmul(toFixed(x-mean),toFixed(x-mean));
		fixedpoint expo = fdiv(fmul(toFixed(-0.5),dist),sigmaSqr);
		fixedpoint res = fmul(fac, toFixed(exp(toFloat(expo))));

		return toInt(fmul(res, toFixed(100)));
	}

	/*
	 * Define maximal expected ball sizes for each image part in pixels
	 */
#if defined ROBOT2011
	// field of view is smaller for new camera
	static const uint8_t maxSizeTop = 10; // 10x10
	static const uint8_t maxSizeMid = 24; // 24x24
	static const uint8_t maxSizeBot = 36; // 36x36
#else
	static const uint8_t maxSizeTop = 16; // 16x16
	static const uint8_t maxSizeMid = 32; // 32x32
	static const uint8_t maxSizeBot = 45; // 45x45
#endif

	static const uint16_t HUE_MAX = 360; /// maximal allowed hue value

	/*
	 * Hue thresholds for the ball color. Take care when adjusting.
	 * Important: Define thresholds in range [-180, 180]
	 * they are used as follows: HUE_THRESH_LOW <= x <= HUE_THRESH_HIGH
	 */
#if defined IMAGEFORMAT_RGB or defined IMAGEFORMAT_BAYER
	static const uint16_t HUE_THRESH_HIGH = 28;
	static const uint16_t HUE_THRESH_LOW = 10;
	static const uint16_t HUE_BALL_MEAN = 18;
	static const uint16_t HUE_BALL_STD = 8;
#elif defined IMAGEFORMAT_YUV422
	static const uint16_t HUE_THRESH_HIGH = 40;
	static const uint16_t HUE_THRESH_LOW = 10;
	static const uint16_t HUE_BALL_MEAN = 26;
	static const uint16_t HUE_BALL_STD = 5;
#endif
};

/**
 * @}
 */

#endif /* OBJECTEXTRACTOR_BALL_LUTFREE_H_ */
