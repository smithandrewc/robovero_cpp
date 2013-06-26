/*
 *  PWMDevice.h
 *
 *  Robovero class header to communicate with an I2C device attached to the Gumstix Robovero expansion board.
 *
 *  This class is basically a conversion of the python code written by Neil 
 *  MacMunn (neil@gumstix.com) at Gumstix.
 *  https://github.com/robovero
 *
 *
 *  Andrew C. Smith
 *  acsmith@stanford.edu
 *  Aerospace Robotics Lab
 *  Stanford University
 *
 *  v1.1 June 26 2013
 *
 */

#ifndef Robovero_H
#include "Robovero.h"
#endif

class PWMDevice
{
	Robovero *robo_host;
	unsigned int pwm_channel;
	unsigned int pwm_pulseWidth;
	static bool isPeriodSet; // flag to notify if period has been set
	static unsigned int pwm_period; // period is used for all PWM signals
	public:
		void move(unsigned int pwm_command);
		PWMDevice(Robovero *host, unsigned int address);
		PWMDevice(Robovero *host, unsigned int address, unsigned int period, unsigned int pulse);
		~PWMDevice();

	private:
		void initPWM();
};
