#include <stdio.h>
#include <inttypes.h>
#include <lcm/lcm.h>
#include <pthread.h>
#include <unistd.h>
#include <signal.h>
#include <sys/time.h>
#include <iostream>
#include "Galil.h"
#include <string>

#include "lcmt_motor_state.h"
#include "lcmt_motor_command.h"

using namespace std;

pthread_mutex_t mutex;
lcm_t * lcm = NULL;

lcmt_motor_state motor_message;
lcmt_motor_command motor_command;

string state_channel;
string command_channel;
string galil_address;

timeval start;

long timevaldiff(struct timeval *starttime, struct timeval *finishtime)
{
	long msec;
	msec=(finishtime->tv_sec-starttime->tv_sec)*1000;
	msec+=(finishtime->tv_usec-starttime->tv_usec)/1000;
	return msec;
}

/*
Helper methods for the mutex since we only have one of them
*/
void lock(void) 
{
	pthread_mutex_lock(&mutex);
}

void unlock(void) 
{
	pthread_mutex_unlock(&mutex);
}

/*
Handler for the incoming LCM message.
Requires a mutex because the received command gets copied into memory accessed by the motor comm thread.
*/
void lcm_command_handler(const lcm_recv_buf_t *rbuf, const char* channel, const lcmt_motor_command *msg, void *user)
{
	lock();
	memcpy(&motor_command,msg,sizeof(lcmt_motor_command));
	unlock();
}

void* lcm_handle_loop(void* a)
{
	while (1)
		lcm_handle(lcm);
	return 0;
}

void lcm_publish_state(lcm_t* lcm) {
	lcmt_motor_state_publish(lcm,state_channel.c_str(), &motor_message);
}


int main(int argc, char** argv)
{
	pthread_mutex_init(&mutex,NULL);

	if(argc != 4){
		cout<< "suggested use: GalilComm 10.66.67.206 pendulum_motor_state pendulum_motor_command" << endl;
		return 1;
	}

	state_channel = string(argv[2]);
	command_channel = string(argv[3]);
	galil_address = string(argv[1]);
	galil_address += " -udp"; //add a flag to force the controller into UDP mode

	//Set up LCM
	lcm = lcm_create("udpm://239.255.76.67:7667?ttl=0");
	if (!lcm) return 1;

	gettimeofday(&start, 0);

	// subscribe to relevant channels:
	lcmt_motor_command_subscription_t * pend_sub = lcmt_motor_command_subscribe (lcm, command_channel.c_str(),&lcm_command_handler, NULL);

	//Initialize the motor_state message
	motor_message.timestamp = 0;
	motor_message.current = 0;
	motor_message.position = 0;
	motor_message.fault = 0;

	// fork listener thread for LCM
	pthread_t lcm_thread;
	pthread_create( &lcm_thread, NULL, &lcm_handle_loop,NULL);

	try
	{
		Galil g(galil_address);
		cout << g.connection() << endl; //Print connection string
		cout << g.libraryVersion() << endl; //Print Library Version
		g.command("KD 0"); //Set the loops gains to zero so we can take take over the loop via the offset voltage
		g.command("KI 0");
		g.command("KP 0");
		g.command("BR 1"); //Tell the controller it as a brushed motor
		while(1){
			vector<char> r = g.record("QR A");
			motor_message.timestamp = g.sourceValue(r, "TIME");
			motor_message.position = g.sourceValue(r, "_TPA");
			motor_message.position2 = g.sourceValue(r, "_TDA");
			motor_message.current = g.sourceValue(r, "_TTA");
			lcm_publish_state(lcm);
			lock();
			double command = motor_command.command;
			unlock();
			char temp_string[20];
			sprintf(temp_string, "OF %f", command);
			string command_string = string(temp_string);
			g.command(command_string);
		}
	}
	catch(string e)
	{
		cout << e;
	}

	pthread_exit(&lcm_thread);
	pthread_mutex_destroy(&mutex);

	lcmt_motor_command_unsubscribe(lcm, pend_sub);

	lcm_destroy (lcm);
	return 0;
}
