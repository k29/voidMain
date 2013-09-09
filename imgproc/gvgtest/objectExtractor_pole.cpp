#include "objectExtractor_pole.h"
#include "debug.h"

PoleExtractor::PoleExtractor() {
	ybyPole = new RectangleObject(POLE_OBJECT, YellowGoal);
	bybPole = new RectangleObject(POLE_OBJECT, BlueGoal);
}

PoleExtractor::~PoleExtractor() {
	delete ybyPole;
	ybyPole = 0;
	delete bybPole;
	bybPole = 0;
}

/**
 * Extract the poles. The given edges could be used for the extraction.
 * @param edges
 * @return true, if poles where found, otherwise false
 */
bool PoleExtractor::extract(const EdgeVector &edges) {

	clear();

	bool found = extractContourBased();
	if(!found) {
		found = extractEdgeColorBased(edges);
	}

	// check for intersection of poles, if they intersect, then remove both
	BoundingBox yby(ybyPole->rectangle, ybyPole->basePoint);
	BoundingBox byb(bybPole->rectangle, bybPole->basePoint);

	if((yby.basePoint.x != -1 && byb.basePoint.x != -1) && (yby.intersects(byb) || yby.distance(byb) < image->getImageWidth() / 2)) {
		bybPole->clear();
		ybyPole->clear();
		found = false;
	}

	// dismiss side poles, if to near to the border of the image
//	if(yby.basePoint.x < IMG_WIDTH / 5 || yby.basePoint.x > 4 * IMG_WIDTH / 5	) {
//		ybyPole->clear();
//	}
//	if(byb.basePoint.x < IMG_WIDTH / 5 || byb.basePoint.x > 4 * IMG_WIDTH / 5) {
//		bybPole->clear();
//	}

	if(bybPoleSeen()) {
		setProtobuf(bybPole);
	}
	if(ybyPoleSeen()) {
		setProtobuf(ybyPole);
	}
	return found;
}

/**
 * Extracts the poles from the pole edges (which are both yellow and blue)
 * and try to extend them by the blue and yellow ones
 * @param edges
 * @return true, if poles where found, otherwise false
 */
bool PoleExtractor::extractEdgeColorBased(const EdgeVector &edges) {
	if(image == 0) {
		return false;
	}

	EdgeVector pole, blue, yellow;
	// classify edges
	for(uint16_t i = 0; i < edges.size(); ++i) {
		Edge *e = edges[i];
		if((e->mostFrequentColor1 == BlueGoal && e->mostFrequentColor2 == YellowGoal) ||
				(e->mostFrequentColor1 == YellowGoal && e->mostFrequentColor2 == BlueGoal)) {
			pole.push_back(e);
		} else if (e->mostFrequentColor1 == BlueGoal || e->mostFrequentColor2 == BlueGoal) {
			blue.push_back(e);
		} else if (e->mostFrequentColor1 == YellowGoal || e->mostFrequentColor2 == YellowGoal) {
			yellow.push_back(e);
		}
	}


	if(pole.empty()) {
		return false;
	}

	int16_t imgWidthDiv2 = image->getImageWidth() / 2;

	EdgeVector byEdge, ybEdge;
	int16_t pole_size = pole.size();
	byEdge.reserve(pole_size);
	ybEdge.reserve(pole_size);

	// Split into blue-yellow edges and yellow-blue-edges

	EdgeVector::const_iterator constiter;
	for(constiter = pole.begin(); constiter != pole.end(); ++constiter) {
		Edge *e = *constiter;

		// check, which color is on which side ...
		int16_t yellowCounter1 = 0;
		int16_t yellowCounter2 = 0;
		int16_t blueCounter1 = 0;
		int16_t blueCounter2 = 0;
		const EdgePoint &middlestEdgePoint = e->linePoints[e->linePoints.size() / 2];
		for(int i = 2; i <= 10; i += 2) {
			int32_t orthoPoint1X = middlestEdgePoint.getX() - toInt(middlestEdgePoint.getDirY() * i);
			int32_t orthoPoint1Y = middlestEdgePoint.getY() + toInt(middlestEdgePoint.getDirX() * i);

			int32_t orthoPoint2X = middlestEdgePoint.getX() + toInt(middlestEdgePoint.getDirY() * i);
			int32_t orthoPoint2Y = middlestEdgePoint.getY() - toInt(middlestEdgePoint.getDirX() * i);

			Color c1 = colorMgr->getPixelColor(*image, orthoPoint1X, orthoPoint1Y);
			if(c1 == BlueGoal) {
				++blueCounter1;
			}
			else if(c1 == YellowGoal) {
				++yellowCounter1;
			}

			Color c2 = colorMgr->getPixelColor(*image, orthoPoint2X, orthoPoint2Y);
			if(c2 == BlueGoal) {
				++blueCounter2;
			}
			else if(c2 == YellowGoal) {
				++yellowCounter2;
			}
		}

		if(yellowCounter1 > blueCounter1 && blueCounter2 > yellowCounter2) {
			e->mostFrequentColor1 = YellowGoal;
			e->mostFrequentColor2 = BlueGoal;
		}
		else if (yellowCounter1 < blueCounter1 && blueCounter2 < yellowCounter2) {
			e->mostFrequentColor1 = BlueGoal;
			e->mostFrequentColor2 = YellowGoal;
		}
		else {
			continue;
		}

		if(e->mostFrequentColor1 == YellowGoal && e->mostFrequentColor2 == BlueGoal) {
			byEdge.push_back(e);
		}
		else {
			ybEdge.push_back(e);
		}
	}

	// find poles by combining yellow-blue and blue-yellow edges
	BoundingBox ybyBB;
	BoundingBox bybBB;
	EdgeVector::iterator iterYB, iterBY;

	iterYB = ybEdge.begin();
	while(iterYB != ybEdge.end()) {

		bool match = false;
		BoundingBox ybBB = (**iterYB).getBoundingBox();

		iterBY = byEdge.begin();
		while(iterBY != byEdge.end()) {
			BoundingBox byBB = (**iterBY).getBoundingBox();
			if(ybBB.distance(byBB) < 150) { // we found a pole!
				match = true;
				BoundingBox poleBB = BoundingBox::merge(byBB, ybBB);
				// yellow-blue-edge is over the blue-yellow-edge
				// so it is a YBY-pole
				if(ybBB.rectangle.y < byBB.rectangle.y) {
					if(ybyBB.rectangle.x != -1) {
						ybyBB = BoundingBox::merge(ybyBB, poleBB);
					}
					else {
						ybyBB = poleBB;
					}
				}
				else { // it is a BYB-pole
					if(bybBB.rectangle.x != -1) {
						bybBB = BoundingBox::merge(bybBB, poleBB);
					}
					else {
						bybBB.set(poleBB.rectangle, poleBB.basePoint, poleBB.numberOfEdgePoints);
					}
				}
				iterBY = byEdge.erase(iterBY);
				continue;

			}
			++iterBY;
		}
		if(match) {
			iterYB = ybEdge.erase(iterYB);
			continue;
		}
		++iterYB;
	}

	// until now, we found the element in the middle of the pole, or nothing ...

	EdgeVector::iterator iter;
	// try to extend the poles with edges which are only blue or yellow classified
	if(ybyBB.rectangle.x != -1 || bybBB.rectangle.x != -1) {
		iter = blue.begin();
		while (iter != blue.end()) {
			BoundingBox blueBB = (**iter).getBoundingBox();
			if (ybyBB.rectangle.x != -1) {
				if (blueBB.distance(ybyBB) < 50) {
					ybyBB = BoundingBox::merge(blueBB, ybyBB);
					iter = blue.erase(iter);
					continue;
				}
			}
			if (bybBB.rectangle.x != -1) {
				if (blueBB.distance(bybBB) < 50) {
					bybBB = BoundingBox::merge(blueBB, bybBB);
					iter = blue.erase(iter);
					continue;
				}
			}
			++iter;
		}

		iter = yellow.begin();
		while (iter != yellow.end()) {
			BoundingBox yellowBB = (**iter).getBoundingBox();
			if (bybBB.rectangle.x != -1) {
				if (yellowBB.distance(bybBB) < 50) {
					bybBB = BoundingBox::merge(yellowBB, bybBB);
					iter = yellow.erase(iter);
					continue;
				}
			}
			if (ybyBB.rectangle.x != -1) {
				if (yellowBB.distance(ybyBB) < 50) {
					ybyBB = BoundingBox::merge(yellowBB, ybyBB);
					iter = yellow.erase(iter);
					continue;
				}
			}
			++iter;
		}

	}

	else {
		// we have no poles found yet, so try to combine
		// the yb or by edges with the blue and yellow ones and
		// guess than the pole
		BoundingBox poleL;
		BoundingBox poleR;

		// variables to distinguish the YBY and BYB pole
		Color topColorL = Unknown;
		Color topColorR = Unknown;
		Color bottomColorL = Unknown;
		Color bottomColorR = Unknown;
		int16_t topL = 0;
		int16_t topR = 0;
		int16_t bottomL = 1000;
		int16_t bottomR = 1000;

		// try to combine yellow-blue edges with only yellow or only blue edges
		for(iterYB = ybEdge.begin(); iterYB != ybEdge.end(); ++iterYB) {
			BoundingBox ybBB = (**iterYB).getBoundingBox();

			iter = yellow.begin();
			while(iter != yellow.end()) {
				BoundingBox yBB = (**iter).getBoundingBox();
				if(yBB.distance(ybBB) < 50) {
					BoundingBox merged = BoundingBox::merge(ybBB, yBB);
					// on the left side -> poleL
					if((merged.rectangle.x + merged.rectangle.x + merged.rectangle.width) / 2 < imgWidthDiv2) {
						if(poleL.rectangle.x != -1) {
							poleL = BoundingBox::merge(poleL, merged);
							if(yBB.rectangle.y < topL) {
								topColorL = YellowGoal;
								topL = yBB.rectangle.y;
							}
							else if(yBB.rectangle.y + yBB.rectangle.height > bottomL) {
								bottomColorL = YellowGoal;
								bottomL = yBB.rectangle.y + yBB.rectangle.height;
							}
						}
						else {
							poleL.set(merged.rectangle, merged.basePoint, merged.numberOfEdgePoints);
							if(yBB.rectangle.y < ybBB.rectangle.y) {// yellow is on top, and blue on bottom
								topL = yBB.rectangle.y;
								topColorL = YellowGoal;
								bottomL = ybBB.rectangle.y + ybBB.rectangle.height;
								bottomColorL = BlueGoal;
							}
							else { // yellow on top and on bottom
								topL = ybBB.rectangle.y;
								topColorL = YellowGoal;
								bottomL = yBB.rectangle.y + yBB.rectangle.height;
								bottomColorL = YellowGoal;
							}
						}
					}
					else { // on the right side -> poleR
						if(poleR.rectangle.x != -1) {
							poleR = BoundingBox::merge(poleR, merged);
							if(yBB.rectangle.y < topR) {
								topColorR = YellowGoal;
								topR = yBB.rectangle.y;
							}
							else if(yBB.rectangle.y + yBB.rectangle.height > bottomR) {
								bottomColorR = YellowGoal;
								bottomR = yBB.rectangle.y + yBB.rectangle.height;
							}
						}
						else {
							poleR.set(merged.rectangle, merged.basePoint, merged.numberOfEdgePoints);
							if(yBB.rectangle.y < ybBB.rectangle.y) {// yellow is on top, and blue on bottom
								topR = yBB.rectangle.y;
								topColorR = YellowGoal;
								bottomR = ybBB.rectangle.y + ybBB.rectangle.height;
								bottomColorR = BlueGoal;
							}
							else { // yellow on top and on bottom
								topR = ybBB.rectangle.y;
								topColorR = YellowGoal;
								bottomR = yBB.rectangle.y + yBB.rectangle.height;
								bottomColorR = YellowGoal;
							}
						}
					}

					iter = yellow.erase(iter);
					continue;
				}
				++iter;
			}

			iter = blue.begin();
			while(iter != blue.end()) {
				BoundingBox bBB = (**iter).getBoundingBox();
				if(bBB.distance(ybBB) < 50) {
					BoundingBox merged = BoundingBox::merge(ybBB, bBB);
					// on the left side -> poleL
					if((merged.rectangle.x + merged.rectangle.x + merged.rectangle.width) / 2 < imgWidthDiv2) {
						if(poleL.rectangle.x != -1) {
							poleL = BoundingBox::merge(poleL, merged);
							if(bBB.rectangle.y < topL) {
								topColorL = BlueGoal;
								topL = bBB.rectangle.y;
							}
							else if(bBB.rectangle.y + bBB.rectangle.height > bottomL) {
								bottomColorL = BlueGoal;
								bottomL = bBB.rectangle.y + bBB.rectangle.height;
							}
						}
						else {
							poleL.set(merged.rectangle, merged.basePoint, merged.numberOfEdgePoints);
							if(bBB.rectangle.y < ybBB.rectangle.y) {// blue is on top and blue on bottom
								topL = bBB.rectangle.y;
								topColorL = BlueGoal;
								bottomL = ybBB.rectangle.y + ybBB.rectangle.height;
								bottomColorL = BlueGoal;
							}
							else { // blue on bottom and yellow on top
								topL = ybBB.rectangle.y;
								topColorL = YellowGoal;
								bottomL = bBB.rectangle.y + bBB.rectangle.height;
								bottomColorL = BlueGoal;
							};
						}
					}
					else { // on the right side -> poleR
						if(poleR.rectangle.x != -1) {
							poleR = BoundingBox::merge(poleR, merged);
							if(bBB.rectangle.y < topR) {
								topColorR = BlueGoal;
								topR = bBB.rectangle.y;
							}
							else if(bBB.rectangle.y + bBB.rectangle.height > bottomR) {
								bottomColorR = BlueGoal;
								bottomR = bBB.rectangle.y + bBB.rectangle.height;
							}
						}
						else {
							poleR.set(merged.rectangle, merged.basePoint, merged.numberOfEdgePoints);
							if(bBB.rectangle.y < ybBB.rectangle.y) {// blue is on top and on bottom
								topR = bBB.rectangle.y;
								topColorR = BlueGoal;
								bottomR = ybBB.rectangle.y + ybBB.rectangle.height;
								bottomColorR = BlueGoal;
							}
							else { // yellow on top and blue on bottom
								topR = ybBB.rectangle.y;
								topColorR = YellowGoal;
								bottomR = bBB.rectangle.y + bBB.rectangle.height;
								bottomColorR = BlueGoal;
							}
						}
					}

					iter = blue.erase(iter);
					continue;
				}
				++iter;
			}

		}

		for(iterBY = byEdge.begin(); iterBY != byEdge.end(); ++iterBY) {
			BoundingBox byBB = (**iterBY).getBoundingBox();

			iter = yellow.begin();
			while(iter != yellow.end()) {
				BoundingBox yBB = (**iter).getBoundingBox();
				if(yBB.distance(byBB) < 50) {
					BoundingBox merged = BoundingBox::merge(byBB, yBB);
					// on the left side -> poleL
					if((merged.rectangle.x + merged.rectangle.x + merged.rectangle.width) / 2 < imgWidthDiv2) {
						if(poleL.rectangle.x != -1) {
							poleL = BoundingBox::merge(poleL, merged);
							if(yBB.rectangle.y < topL) {
								topColorL = YellowGoal;
								topL = yBB.rectangle.y;
							}
							else if(yBB.rectangle.y + yBB.rectangle.height > bottomL) {
								bottomColorL = YellowGoal;
								bottomL = yBB.rectangle.y + yBB.rectangle.height;
							}
						}
						else {
							poleL.set(merged.rectangle, merged.basePoint, merged.numberOfEdgePoints);
							if(yBB.rectangle.y < byBB.rectangle.y) {// yellow is on top and on bottom
								topL = yBB.rectangle.y;
								topColorL = YellowGoal;
								bottomL = byBB.rectangle.y + byBB.rectangle.height;
								bottomColorL = YellowGoal;
							}
							else { // blue on top and yellow on bottom
								topL = byBB.rectangle.y;
								topColorL = BlueGoal;
								bottomL = yBB.rectangle.y + yBB.rectangle.height;
								bottomColorL = YellowGoal;
							}
						}
					}
					else { // on the right side -> poleR
						if(poleR.rectangle.x != -1) {
							poleR = BoundingBox::merge(poleR, merged);
							if(yBB.rectangle.y < topR) {
								topColorR = YellowGoal;
								topR = yBB.rectangle.y;
							}
							else if(yBB.rectangle.y + yBB.rectangle.height > bottomR) {
								bottomColorR = YellowGoal;
								bottomR = yBB.rectangle.y + yBB.rectangle.height;
							}
						}
						else {
							poleR.set(merged.rectangle, merged.basePoint, merged.numberOfEdgePoints);
							if(yBB.rectangle.y < byBB.rectangle.y) {// yellow is on top and on bottom
								topR = yBB.rectangle.y;
								topColorR = YellowGoal;
								bottomR = byBB.rectangle.y + byBB.rectangle.height;
								bottomColorR = YellowGoal;
							}
							else { // blue on top and yellow on bottom
								topR = byBB.rectangle.y;
								topColorR = BlueGoal;
								bottomR = yBB.rectangle.y + yBB.rectangle.height;
								bottomColorR = YellowGoal;
							}
						}
					}

					iter = yellow.erase(iter);
					continue;
				}
				++iter;
			}

			iter = blue.begin();
			while(iter != blue.end()) {
				BoundingBox bBB = (**iter).getBoundingBox();
				if(bBB.distance(byBB) < 50) {
					BoundingBox merged = BoundingBox::merge(byBB, bBB);
					// on the left side -> poleL
					if((merged.rectangle.x + merged.rectangle.x + merged.rectangle.width) / 2 < imgWidthDiv2) {
						if(poleL.rectangle.x != -1) {
							poleL = BoundingBox::merge(poleL, merged);
							if(bBB.rectangle.y < topL) {
								topColorL = BlueGoal;
								topL = bBB.rectangle.y;
							}
							else if(bBB.rectangle.y + bBB.rectangle.height > bottomL) {
								bottomColorL = BlueGoal;
								bottomL = bBB.rectangle.y + bBB.rectangle.height;
							}
						}
						else {
							poleL.set(merged.rectangle, merged.basePoint, merged.numberOfEdgePoints);
							if(bBB.rectangle.y < byBB.rectangle.y) {// blue is on top and yellow on bottom
								topL = bBB.rectangle.y;
								topColorL = BlueGoal;
								bottomL = byBB.rectangle.y + byBB.rectangle.height;
								bottomColorL = YellowGoal;
							}
							else { // blue on top and on bottom
								topL = byBB.rectangle.y;
								topColorL = BlueGoal;
								bottomL = bBB.rectangle.y + bBB.rectangle.height;
								bottomColorL = BlueGoal;
							}
						}
					}
					else { // on the right side -> poleR
						if(poleR.rectangle.x != -1) {
							poleR = BoundingBox::merge(poleR, merged);
							if(bBB.rectangle.y < topR) {
								topColorR = BlueGoal;
								topR = bBB.rectangle.y;
							}
							else if(bBB.rectangle.y + bBB.rectangle.height > bottomR) {
								bottomColorR = BlueGoal;
								bottomR = bBB.rectangle.y + bBB.rectangle.height;
							}
						}
						else {
							poleR.set(merged.rectangle, merged.basePoint, merged.numberOfEdgePoints);
							if(bBB.rectangle.y < byBB.rectangle.y) {// blue is on top and yellow on bottom
								topR = bBB.rectangle.y;
								topColorR = BlueGoal;
								bottomR = byBB.rectangle.y + byBB.rectangle.height;
								bottomColorR = YellowGoal;
							}
							else { // blue on top and on bottom
								topR = byBB.rectangle.y;
								topColorR = BlueGoal;
								bottomR = bBB.rectangle.y + bBB.rectangle.height;
								bottomColorR = BlueGoal;
							}
						}
					}

					iter = blue.erase(iter);
					continue;
				}
				++iter;
			}

		}

		if(poleL.rectangle.x != -1) {
			if(topColorL == YellowGoal && bottomColorL == YellowGoal) {
				ybyBB.set(poleL.rectangle, poleL.basePoint, poleL.numberOfEdgePoints);

			}
			else if(topColorL == BlueGoal && bottomColorL == BlueGoal){
				bybBB.set(poleL.rectangle, poleL.basePoint, poleL.numberOfEdgePoints);
			}
		}
		if(poleR.rectangle.x != -1) {
			if(topColorR == YellowGoal && bottomColorR == YellowGoal) {
				ybyBB.set(poleR.rectangle, poleR.basePoint, poleR.numberOfEdgePoints);
			}
			else if(topColorR == BlueGoal && bottomColorR == BlueGoal) {
				bybBB.set(poleR.rectangle, poleR.basePoint, poleR.numberOfEdgePoints);
			}
		}
	}


	bool foundPole = false;

	// Create the objects ...
	if(ybyBB.rectangle.x != -1) {
		ybyPole->set(ybyBB.rectangle, ybyBB.basePoint);
		foundPole = true;
	}
	if(bybBB.rectangle.x != -1) {
		bybPole->set(bybBB.rectangle, bybBB.basePoint);
		foundPole = true;
	}

	return foundPole;
}


/**
 * Extract the side poles using the information known from the goal extraction with the field contour.
 * @return true, if we find something, otherwise false
 */
bool PoleExtractor::extractContourBased() {
	BoundingBox ybyMax, bybMax;

	std::list<BoundingBox>::iterator iter = ybyPoleBoundingBoxes.begin();
	while(iter != ybyPoleBoundingBoxes.end()) {
		const BoundingBox &b = *iter;

		int bsize = b.rectangle.height * b.rectangle.width;
		if(bsize == 0) {
			bsize = b.rectangle.height + b.rectangle.width;
		}

		if(bsize > ybyMax.rectangle.width * ybyMax.rectangle.height || bsize > ybyMax.rectangle.width + ybyMax.rectangle.height) { // bigger then max
			ybyMax.set(b.rectangle, b.basePoint, b.numberOfEdgePoints);
		}
		++iter;
	}

	iter = bybPoleBoundingBoxes.begin();
	while(iter != bybPoleBoundingBoxes.end()) {
		const BoundingBox &b = *iter;

		int bsize = b.rectangle.height * b.rectangle.width;
		if(bsize == 0) {
			bsize = b.rectangle.height + b.rectangle.width;
		}

		if(bsize > bybMax.rectangle.width * bybMax.rectangle.height || bsize > bybMax.rectangle.width + bybMax.rectangle.height) { // bigger then max
			bybMax.set(b.rectangle, b.basePoint, b.numberOfEdgePoints);
		}
		++iter;
	}

	bool found = false;

	if(bybMax.rectangle.x != -1) {
		bybPole->set(bybMax.rectangle, bybMax.basePoint);
		found = true;
	}
	if(ybyMax.rectangle.x != -1) {
		ybyPole->set(ybyMax.rectangle, ybyMax.basePoint);
		found = true;
	}

	return found;
}

void PoleExtractor::setProtobuf(RectangleObject *obj) {
#if VISION_DEBUG
	de::fumanoids::message::ObjectPosition *poleObjStatus = protobufExtractorStatus->add_objectpositions();
	if(obj->color == BlueGoal)
		poleObjStatus->set_type(de::fumanoids::message::ObjectPosition_ObjectType_BYBPole);
	else
		poleObjStatus->set_type(de::fumanoids::message::ObjectPosition_ObjectType_YBYPole);
	de::fumanoids::message::Position *poleObjStatusPos = poleObjStatus->mutable_position();
	poleObjStatusPos->set_positiontype(de::fumanoids::message::Position_PositionType_IMAGE);
	poleObjStatusPos->set_x(obj->basePoint.x);
	poleObjStatusPos->set_y(obj->basePoint.y);
	poleObjStatus->set_isused(true);
#endif
}
