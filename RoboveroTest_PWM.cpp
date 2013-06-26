/*
 *  RoboveroTest_PMW.cpp
 *
 *  Robovero class to test PMW control on the Gumstix Robovero expansion board
 *
 *  PWM servo is commanded, no checks on valid commands.
 *
 *  Andrew C. Smith
 *  acsmith@stanford.edu
 *  Aerospace Robotics Lab
 *  Stanford University
 *
 *  v1.1 June 26 2013
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include "Robovero.h"
#include "PWMDevice.h"

int main(int argc, char** argv)
{
    int i, nBytes;
	int fd; /* file descriptor */
	char buffer[256];

    // create Robovero object
	Robovero test ("/dev/ttyACM0","testLogging.log");
	
    // connect pwm channels 1 and 4
	PWMDevice pwm1 (&test, 1);
	PWMDevice pwm2 (&test, 4);
	
	for (i=0;i<30;i++)
	{
		pwm1.move(196+(45+90*(i%2))*1820/180);
		pwm2.move(196+(0+90*(i%2))*1820/180);
		sleep(1);
	}
    	return 0;
}
