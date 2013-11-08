all: ../Build/imu.a

../Build/imu.a: ../Build/imu.o ../Build/totalxsens.o
	@echo $@
	@if !(test -d ../Build); then mkdir ../Build; fi
	@ar rc $@ ../Build/imu.o ../Build/totalxsens.o
	@ranlib $@ 

../Build/imu.o:	imu.o
	@echo $@
	cp imu.o $@
../Build/totalxsens.o: totalxsens.o
	@echo $@
	cp totalxsens.o $@


all1: imu.o test main
test: test.cpp imu.o
	g++ test.cpp totalxsens.o imu.o -o test -lrt
imu.o: imu.cpp imu.h totalxsens.o
	g++ -c imu.cpp
main: main.cpp imu.o
	g++ main.cpp totalxsens.o imu.o -o main -lrt
