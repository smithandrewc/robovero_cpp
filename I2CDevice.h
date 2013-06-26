/*
 *  I2CDevice.h
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
 *  v1.2 June 26 2013
 *
 */

#ifndef Robovero_H
#include "Robovero.h"
#endif

class I2CDevice
{
	void* config;
	void* tx_data;
	void* rx_data;
	void* rx_data6;
	Robovero *robo_host;
	
	public:
        unsigned char read();
		unsigned char readReg(void* reg);
        void read6(unsigned char* values);
		void read6Reg(void* reg, unsigned short* values);
    	void read6Reg(void* reg, unsigned char* values);
        void writeReg(unsigned char reg, unsigned char value);        
        void writeReg(unsigned char value);
		I2CDevice(Robovero *host, unsigned int address);
		~I2CDevice();	
};
