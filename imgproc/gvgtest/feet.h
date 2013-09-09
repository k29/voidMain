#ifndef __FEET_H__
#define __FEET_H__

#include "singleton.h"
#include "comm.h"
#include "boundingBox.h"
#include "position.h"
#include "events.h"

#include <vector>
#include <string>
#include <fstream>

/**
 * Class defining the feet (or actually the bounding box where to find the feet in the image)
 * @ingroup vision
 */
class Feet : public Singleton<Feet>, public OperationCallback, public EventCallback {
protected:
	Feet();
	friend class Singleton<Feet>;

	BoundingBox feetSpace;

public:
	virtual ~Feet();

	void getFeetSpace(int16_t &left, int16_t &top, int16_t &right, int16_t &bottom);
	const BoundingBox&      getFeetSpace()         { return feetSpace;    }

	virtual bool operationCallback(
				OPERATION operation,
				uint8_t   flags,
				uint8_t  *data,
				uint16_t  dataLen,
				struct sockaddr_in *remoteAddress);

	void eventCallback(EventType evtType, void*);
};

#endif
