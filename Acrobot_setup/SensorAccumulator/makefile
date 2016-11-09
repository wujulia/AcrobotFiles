CC = g++
DEBUG = -g
CFLAGS = -Wall
#ACROBOT = ../../../robotlib-dev/examples/Acrobot
ACROBOT = .
#INCLUDELCM = -I$(ACROBOT) -I~/lcm/lcm -I.
INCLUDELCM = -I$(ACROBOT) -I.
LIBLCM = -llcm

SensorAccumulator: SensorAccumulator.cpp
	lcm-gen -c ../LCMTypes/*.lcm
	$(CC) $(CFLAGS) $(INCLUDELCM) -c lcmt_acrobot_y.c lcmt_motor_state.c
	$(CC) $(CFLAGS) SensorAccumulator.cpp $(INCLUDELCM) $(LIBLCM) -lpthread lcmt_motor_state.o lcmt_acrobot_y.o -o SensorAccumulator

clean:
	\rm SensorAccumulator
	\rm *.o
