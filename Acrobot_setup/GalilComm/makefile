CC = g++
DEBUG = -g
CFLAGS = -Wall
OBJS = $(shell ls $(CML)/c/*.o)
PENDULUM = ../../../Pendulum_robotlib
INCLUDELCM = -I$(PENDULUM) -I~/lcm/lcm -I.
LIBLCM = -llcm
LIBGALIL = -lGalil

GalilComm: GalilComm.cpp
	lcm-gen -c $(PENDULUM)/*.lcm
	lcm-gen -c ../LCMTypes/*.lcm
	$(CC) $(CFLAGS) $(INCLUDELCM) -c lcmt_motor_state.c lcmt_motor_command.c
	$(CC) $(CFLAGS) GalilComm.cpp $(INCLUDELCM) $(INCLUDECML) $(LIBLCM) $(LIBGALIL) -lpthread lcmt_motor_state.o lcmt_motor_command.o -o GalilComm

clean:
	\rm GalilComm
	\rm *.o
