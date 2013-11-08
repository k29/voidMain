all: ../Build/gamecontrollerfunc.a

../Build/gamecontrollerfunc.a: ../Build/gamecontrollerfunc.o
	@echo $@
	@ar rc $@ ../Build/gamecontrollerfunc.o
	@ranlib $@


../Build/gamecontrollerfunc.o: gamecontrollerfunc.cpp gamecontrollerfunc.h RoboCupGameControlData.h
	@echo $@
	@if !(test -d ../Build); then mkdir ../Build; fi
	@g++ -c gamecontrollerfunc.cpp -o $@