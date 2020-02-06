
ALL_SRCS := $(filter-out unittests.cpp, $(wildcard *.cpp))
TEST_SRCS := $(filter-out unittests.cpp main.cpp, $(wildcard *.cpp))


all:
	g++ -O3 $(ALL_SRCS) -o emu
debug:
	g++ -g $(ALL_SRCS) -o emu
test:
	g++ -g $(TEST_SRCS) unittests.cpp -o test
clean:
	rm emu test -f
