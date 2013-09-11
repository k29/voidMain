#ifndef BLOB_H_
#define BLOB_H_

#include <algorithm>

struct Blob {
	int x;
	int y;
	int minMagnitude;
	int intensity;
	int scale;
	Blob() : x(0), y(0), minMagnitude(0), intensity(0), scale(0) {}
	Blob(int X, int Y, int minMag, int i, int s)
	: x(X)
	, y(Y)
	, minMagnitude(minMag)
	, intensity(i)
	, scale(s)
	{}
};

bool operator>(Blob const & b1, Blob const & b2);

struct Comp{
	bool operator()(Blob const & a, Blob const & b) {
		return abs(a.x-b.x) + abs(a.y-b.y) <= std::min(a.scale, b.scale);
	}
};

#endif /* BLOB_H_ */
