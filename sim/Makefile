VERSION = 1_27_2015 
all: pipes simulator

pipes: cpipes
	mkfifo ../state_in
	mkfifo ../motor_out

simulator:
	g++ -ggdb ./src/simulator.cpp ./src/simulator.hpp ./src/motor.hpp -o ../simulator -larmadillo -std=c++11

clean: cpipes
	rm -f ../simulator

cpipes:
	rm -f ../state_in
	rm -f ../motor_out
