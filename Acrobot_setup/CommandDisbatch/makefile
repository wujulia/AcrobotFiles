CC = g++
DEBUG = -g
CFLAGS = -Wall
#ACROBOT = ../../../robotlib-dev/examples/Acrobot
ACROBOT = ../../../robotlib/examples/Acrobot/
INCLUDELCM = -I$(ACROBOT) -I~/lcm/lcm -I.
LIBLCM = -llcm

CommandDisbatch: CommandDisbatch.cpp
	lcm-gen -c ../LCMTypes/*.lcm
	$(CC) $(CFLAGS) $(INCLUDELCM) -c lcmt_acrobot_u.c lcmt_motor_command.c
	$(CC) $(CFLAGS) CommandDisbatch.cpp $(INCLUDELCM) $(LIBLCM) lcmt_motor_command.o lcmt_acrobot_u.o -o CommandDisbatch

clean:
	\rm CommandDisbatch
	\rm *.o
