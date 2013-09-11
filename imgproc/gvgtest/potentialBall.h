#ifndef POTENTIALBALL_H_
#define POTENTIALBALL_H_

struct PotentialBall{
public:
	PotentialBall(int X, int Y, int r) : x(X), y(Y), radius(r) {}
	int x;
	int y;
	int radius;
};

#endif /* POTENTIALBALL_H_ */
