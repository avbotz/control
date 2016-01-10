VERSION = 31_10_2015 
all: pipes simulator

pipes: cpipes
	mkfifo sensor_in
	mkfifo sensor_out
	mkfifo motor_in
	mkfifo motor_out

simulator:
	g++ -ggdb ./src/simulator.cpp ./src/simulator.hpp ./src/motor.hpp -o simulator -larmadillo -std=c++11

clean: cpipes
	rm -f simulator

cpipes:
	rm -f sensor_in
	rm -f sensor_out
	rm -f motor_in
	rm -f motor_out
