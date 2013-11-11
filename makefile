all: run

run: Source/Build/path.a Source/Build/walk.a Source/Build/behavior.a Source/Build/imgProc.a Source/Build/imu.a run.cpp
	@echo compiling and linking final code
	g++ -w run.cpp Source/Build/path.o Source/Build/walk.a Source/Build/behavior.a Source/Build/imgProc.a Source/Build/imu.a -o run -I/usr/include/flycapture -lflycapture -lftdi -lopencv_highgui -lpthread -fpermissive -lopencv_core -lopencv_imgproc  -lcvblob -ltbb -O3 -fpermissive

Source/Build/path.a: Source/path/*.cpp Source/path/*.hpp Source/common/*.h
	@make -s -C Source/path
Source/Build/imu.a: Source/xsens/*.cpp Source/xsens/*.h Source/common/*.h
	@make -s -C Source/xsens
Source/Build/walk.a: Source/walk/*.cpp Source/walk/*.h Source/common/*.h
	@make -s -C Source/walk
Source/Build/behavior.a: Source/behavior/*.cpp Source/behavior/*.h Source/behavior/xabsl-src/*.xabsl Source/behavior/xabsl-src/options/*.xabsl Source/common/*.h
	@make -s -C Source/behavior
Source/Build/imgProc.a: Source/imgProc/*.cpp Source/imgProc/*.h Source/common/*.h
	@make -s -C Source/imgProc


DOC:
	@make -s -C Source/behavior/xabsl-src DOC
clean:
	@rm -f intermediate-code.dat
	@rm -f debug-symbols.dat 
	@rm -f run
	@make -s -C Source clean
	@make -s -C Xabsl/compiler clean
	
