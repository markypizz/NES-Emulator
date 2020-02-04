SRCS := $(wildcard *.cpp)

all:
	g++ -O3 $(SRCS) -o emu
debug:
	g++ -g $(SRCS) -o emu
clean:
	rm emu
