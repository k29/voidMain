all: ../Build/testwalk.a

main: main.cpp ../Build/testwalk.a
	g++ main.cpp ../Build/testwalk.a -o main -lftdi -lm -lrt -lpthread `pkg-config --libs opencv`

../Build/testwalk.a: ../Build/walk_thread.o ../Build/walk.o ../Build/testBot.o ../Build/communication.o ../Build/commondefswalk.o
	@echo $@
	@ar rc $@ ../Build/walk.o ../Build/walk_thread.o  ../Build/testBot.o ../Build/communication.o ../Build/commondefswalk.o
	@echo $@
	@ranlib $@


../Build/walk_thread.o: walk_thread.cpp walk_thread.h 
	@echo $@
	@if !(test -d ../Build); then mkdir ../Build; fi
	@g++ -c walk_thread.cpp -fpermissive -std=c++11 -o $@

../Build/walk.o: serial.cpp serial.h
	@echo $@
	@if !(test -d ../Build); then mkdir ../Build; fi
	@g++ -c serial.cpp -fpermissive -o $@	

../Build/testBot.o: testBot.cpp testBot.h
	@echo $@
	@if !(test -d ../Build); then mkdir ../Build; fi
	@g++ -c testBot.cpp -o $@

../Build/communication.o: communication.cpp communication.h
	@echo $@
	@if !(test -d ../Build); then mkdir ../Build; fi
	@g++ -c communication.cpp -o $@

../Build/commondefswalk.o: commondefswalk.cpp commondefswalk.h
	@echo $@
	@if !(test -d ../Build); then mkdir ../Build; fi
	@g++ -c commondefswalk.cpp -o $@


  
