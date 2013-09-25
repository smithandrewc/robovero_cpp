/*
 *  ADCDevice.h
 *
 *  Robovero class header to communicate with the ADC on the Gumstix Robovero expansion board.
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
 *  v1.1 September 25 2013
 *
 */

#ifndef Robovero_H
#include "Robovero.h"
#endif

class ADCDevice
{
	Robovero *robo_host;
	public:
		unsigned short readADC(unsigned char channel);
		ADCDevice(Robovero *host);
		~ADCDevice();

	private:
		void initADC();
};
