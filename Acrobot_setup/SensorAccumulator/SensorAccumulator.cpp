#include <stdio.h>
#include <inttypes.h>
#include <lcm/lcm.h>
#include <pthread.h>
#include <unistd.h>
#include <signal.h>
#include <sys/time.h>
#include <iostream>

#include "lcmt_motor_state.h"
#include "lcmt_acrobot_y.h"

const int TICKRATE=200; //Main clock for the controller

using namespace std;

pthread_mutex_t mutex;
lcm_t * lcm = NULL;

lcmt_motor_state last_motor_message;
lcmt_acrobot_y outgoing_sensor_message;

timeval start;

int motor_initialized;
const double CURRENT_TO_TORQUE = 0.003455; //Motor constant 0.0525 N-m/A, 4.3:1 planetary, 1.53:1 elbow drive

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
Handlers for the incoming LCM messages. These get called by the LCM handling thread and copy the received data into state estimator.
Each requires a mutex because the the update method which uses these pieces of data could be called at any time
*/
void lcm_motor_handler(const lcm_recv_buf_t *rbuf, const char* channel, const lcmt_motor_state *msg, void *user)
{
	lock();
	memcpy(&last_motor_message,msg,sizeof(lcmt_motor_state));
	motor_initialized = 1;
	unlock();
}


/*
Gets executed on the clock. Take the most current sensor messages and put them together into the complete sensor message.
Don't do the actual publishing here, that's taken care of for you.
*/
void updateState()
{
	if(motor_initialized == 1){
		outgoing_sensor_message.tau = (double)last_motor_message.current*CURRENT_TO_TORQUE;
		outgoing_sensor_message.theta1 = last_motor_message.position;
		outgoing_sensor_message.theta2 = last_motor_message.position2;
	} else {
		outgoing_sensor_message.tau = 0;
		outgoing_sensor_message.theta1 = 0;
		outgoing_sensor_message.theta2 = 0;
	}

	timeval end;
	gettimeofday(&end, 0);
	outgoing_sensor_message.timestamp = timevaldiff(&start, &end);
}

void* lcm_handle_loop(void* a)
{
	while (1)
		lcm_handle(lcm);
	return 0;
}

void lcm_publish_state(lcm_t* lcm) {
	lcmt_acrobot_y_publish(lcm,"acrobot_y", &outgoing_sensor_message);
}

void timer_function(int sigNum)
{
	lock();
	updateState();
    	lcm_publish_state(lcm);
	unlock();
}

void setup_timer(void)
{
	struct itimerval itv;
	struct sigaction act;

	itv.it_interval.tv_sec = 0;
	itv.it_interval.tv_usec = (unsigned long) (1000000 / TICKRATE);
	itv.it_value.tv_sec = 0;
	itv.it_value.tv_usec = (unsigned long) (1000000 / TICKRATE);

	memset (&act, 0, sizeof (act));

	act.sa_handler = &timer_function;
	act.sa_flags = 0;

	sigemptyset(&act.sa_mask);
	sigaddset(&act.sa_mask, SIGALRM);

	if (sigaction(SIGALRM, &act, NULL) == -1) {
		printf("sigaction failed\n");
		exit(1);
	}

	if (setitimer(ITIMER_REAL, &itv, NULL) == -1) {
		printf("setitimer failed\n");
		exit(1);
	}
}

int main(int argc, char** argv)
{
	pthread_mutex_init(&mutex,NULL);

	//Set up LCM
	lcm = lcm_create("udpm://239.255.76.67:7667?ttl=1");
	if (!lcm) return 1;

	motor_initialized = 0;

	gettimeofday(&start, 0);

	// subscribe to relevant channels:
	lcmt_motor_state_subscription_t * motor_sub = lcmt_motor_state_subscribe (lcm, "acrobot_out", &lcm_motor_handler, NULL);

	// create timer for estimation loop
	setup_timer();

	// fork listener thread for LCM
	pthread_t accumulator_thread;
	pthread_create( &accumulator_thread, NULL, &lcm_handle_loop,NULL);

	while (1)
		pause();

	pthread_exit(&accumulator_thread);
	pthread_mutex_destroy(&mutex);

	lcmt_motor_state_unsubscribe(lcm, motor_sub);

	lcm_destroy (lcm);
	return 0;
}
