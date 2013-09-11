#include "blob.h"

bool operator>(Blob const & b1, Blob const & b2) {
	return b1.minMagnitude > b2.minMagnitude;
}
