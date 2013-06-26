/*
 *  RoboveroTest_IMU.cpp
 *
 *  Robovero class to test IMU data collection on the Gumstix Robovero expansion board
 *
 *  Data is read in from the Overo and printed to the console
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
#include "I2CDevice.h"

int main(int argc, char** argv)
{
	int i, nBytes;
	int fd; /* file descriptor */
	char buffer[256];

	//Robovero test ("/dev/ttyACM0","testLogging.log");
    Robovero test ("/dev/ttyACM0");

	I2CDevice accel(&test,0x18);
	accel.writeReg((void *)0x20,0x27);
	accel.writeReg((void *)0x23,0x00);

	I2CDevice magneto(&test,0x1E);
	magneto.writeReg((void *)0x00,0x18);
	magneto.writeReg((void *)0x01,0x20);
	magneto.writeReg((void *)0x02,0x00);

	I2CDevice gyro(&test, 0x68);
	gyro.writeReg((void *)0x22,0x08);
	gyro.writeReg((void *)0x23,0x80);
	gyro.writeReg((void *)0x20,0x0F);
	
	unsigned short ret_values[3];
    // unsigned char ret_values_char[6]; // could also be used

	for (i=0;i<30;i++)
	{
		accel.read6Reg((void *)0x28, ret_values);
        //accel.read6Reg((void *)0x28, ret_values_char);  // this should work too, get bytes instead of shorts
		printf("a [x, y, z] : [%f, %f, %f]\n",((double)((short)ret_values[0]))/16384.0,((double)((short)ret_values[1]))/16384.0,((double)((short)ret_values[2]))*(2.0/32767.0)); // outputs g's, max range is +/- 2g - finest setting
        //printf("a [x, y, z] : [%f, %f, %f]\n",((double)((short)(ret_values_char)))/16384.0,((double)((short)(ret_values_char+2)))/16384.0,((double)((short)(ret_values_char+4)))*(2.0/32767.0)); // outputs g's, max range is +/- 2g - finest setting
	
		magneto.read6Reg((void *)0x03, ret_values);	
		printf("c [x, y, z] : [%f, %f, %f]\n",((double)((short)ret_values[0]))/1055.0,((double)((short)ret_values[1]))/1055.0,((double)((short)ret_values[2]))/950.0); // outputs Guass, max range is +/- 1.3 Gauss - finest setting

		gyro.read6Reg((void*)0x28, ret_values);
		printf("g [x, y, z] : [%f, %f, %f]\n",((double)((short)ret_values[0]))*(250.0/32767.0),((double)((short)ret_values[1]))*(250.0/32767.0),((double)((short)ret_values[2]))*(250.0/32767.0)); // outputs degrees per second, max range is +/- 250 dps - finest setting
	
		sleep(1);
	}
    	return 0;
}
