all: distance
distance: distance.cpp 
	g++ -o distance distance.cpp -lm -lueye_api `pkg-config --libs opencv cvblob`
clean: 
	rm distance

