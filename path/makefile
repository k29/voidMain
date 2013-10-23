all: path.o path.hpp path.cpp graph.hpp testpath.cpp 
	g++ path.o testpath.cpp -o testpath `pkg-config --libs opencv`
path.o: path.cpp path.hpp graph.hpp
	g++ -c path.cpp 


