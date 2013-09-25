/*
 *  ADCDevice.cpp
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

#include "ADCDevice.h"
#include "lpc_types.h"
#include "lpc17xx_adc.h"
#include "LPC17xx.h"
#include <string.h>

// read from the ADC
unsigned short ADCDevice::readADC(unsigned char channel)
{
	unsigned int args[4];
	char command_string[128];
    unsigned int status;

	// make sure channel is valid, valid channels are 0-3 and 5-7
    // channel 4 pin (P1[30]) is configured for USB bus power detection
    // so it's not available for ADC
	if (channel >= 0 && channel <= 7 && channel != 4)
	{
        // send the ChannelCmd
        args[0] = (unsigned int)LPC_ADC;
		args[1] = channel;
		args[2] = ENABLE;	
		strcpy(command_string, "ADC_ChannelCmd");
        robo_host->robocaller(command_string,args,3);
        
        // send the StartCmd ... start conversion
		args[0] = (unsigned int)LPC_ADC;
		args[1] = ADC_START_NOW;				
		strcpy(command_string, "ADC_StartCmd");		
        robo_host->robocaller(command_string,args,2);
        
        // wait for the data to be available
        do 
        {
            // get the adc status
            args[0] = (unsigned int)LPC_ADC;
            args[1] = channel;
            args[2] = ADC_DATA_DONE;
            strcpy(command_string, "ADC_ChannelGetStatus");
            robo_host->robocaller(command_string,1,&status,args,3);
        } while (!status);
        
        // data ready, read it
        args[0] = (unsigned int)LPC_ADC;
        args[1] = channel;
        strcpy(command_string, "ADC_ChannelGetData");
        robo_host->robocaller(command_string,1,&status,args,2);
        
        return status;
	}
}

// initializing script for ADCDevice
void ADCDevice::initADC()
{
    // nothing to do to initialize ADC
    // everything taken care of with roboveroConfig in Robovero::init
}

// create ADCDevice connected to Robovero host
ADCDevice::ADCDevice(Robovero *host)
{
    // set the Robovero host
	robo_host = host;

	// initialize the device
	initADC();
}

// destructor ... doesn't have to do anything in this case
ADCDevice::~ADCDevice()
{
}
