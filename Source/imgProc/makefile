all: ../Build/imgProc.a


../Build/imgProc.a: ../Build/camcapture.o ../Build/camcontrol.o ../Build/featuredetection.o ../Build/headmotor.o ../Build/imgproc.o ../Build/localize.o
	@echo $@
	@ar rc $@ ../Build/camcapture.o ../Build/camcontrol.o ../Build/featuredetection.o ../Build/headmotor.o ../Build/imgproc.o ../Build/localize.o
	@echo
	@ranlib $@

../Build/camcapture.o: camcapture.cpp camcapture.h
	@echo $@
	@if !(test -d ../Build); then mkdir ../Build; fi
	@g++ -c camcapture.cpp -I/usr/include/flycapture -O3 -o $@

../Build/camcontrol.o: camcontrol.cpp camcontrol.h
	@echo $@
	@if !(test -d ../Build); then mkdir ../Build; fi
	@g++ -c camcontrol.cpp -I/usr/include/flycapture -O3 -o $@

../Build/featuredetection.o: featuredetection.cpp featuredetection.h
	@echo $@
	@if !(test -d ../Build); then mkdir ../Build; fi
	@g++ -c featuredetection.cpp -o $@

../Build/headmotor.o: headmotor.cpp headmotor.h
	@echo $@
	@if !(test -d ../Build); then mkdir ../Build; fi
	@g++ -c headmotor.cpp -I/usr/include/flycapture -O3 -o $@

../Build/localize.o: localize.cpp localize.h
	@echo $@
	@if !(test -d ../Build); then mkdir ../Build; fi
	@g++ -c localize.cpp -I/usr/include/flycapture -O3 -fpermissive -o $@

../Build/imgproc.o: imgproc.cpp imgproc.h
	@echo $@
	@if !(test -d ../Build); then mkdir ../Build; fi
	@g++ -c imgproc.cpp -o $@

