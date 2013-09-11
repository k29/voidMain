/** @file
 **
 ** Singleton class to get the current camera model
 */

#ifndef THECAMERAMODEL_H_
#define THECAMERAMODEL_H_

#include "singleton.h"
#include "fishEyeLensModel.h"

typedef FishEyeLensModel CurrentModel;

class TheCameraModel {
 private:
	TheCameraModel();
	TheCameraModel(TheCameraModel& other);


public:
	static CameraModel& getInstance()
	{
		static CurrentModel instance;
		return instance;
	}
};

#endif /* THECAMERAMODEL_H_ */
