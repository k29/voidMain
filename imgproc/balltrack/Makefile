all: camcapture.o headmotor.o kartik.cpp
	g++ camcapture.o headmotor.o kartik.cpp -o kartik -lopencv_highgui -lopencv_core -lopencv_imgproc -lflycapture -lftdi -I/usr/include/flycapture -fpermissive -O3
camcapture.o: camcapture.cpp camcapture.h
	g++ -c camcapture.cpp -lopencv_highgui -lflycapture -lftdi -I/usr/include/flycapture -lopencv_core -lopencv_imgproc -fpermissive -O3
headmotor.o: headmotor.cpp headmotor.h
	g++ -c headmotor.cpp -lopencv_highgui -lflycapture -lftdi -I/usr/include/flycapture -lopencv_core -lopencv_imgproc -fpermissive -O3
clean:
	rm camcapture.o headmotor.o kartik
