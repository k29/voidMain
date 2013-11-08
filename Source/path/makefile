all: ../Build/path.a

../Build/path.a: ../Build/path.o
	@echo $@
	@ar rc $@ ../Build/path.o
	@echo
	@ranlib $@

../Build/path.o: path.cpp path.hpp graph.hpp
	@echo $@
	@if !(test -d ../Build); then mkdir ../Build; fi
	@g++ -c path.cpp  -o $@