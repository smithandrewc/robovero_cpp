/*
 *  PWMDevice.cpp
 *
 *  Robovero class to communicate with PWM device attached to the Gumstix Robovero expansion board.
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

#include "PWMDevice.h"
#include "lpc_types.h"
#include "lpc17xx_pwm.h"
#include "LPC17xx.h"
#include <string.h>

bool PWMDevice::isPeriodSet;
unsigned int PWMDevice::pwm_period;

// issue new position command to servo
void PWMDevice::move(unsigned int pwm_command)
{
	unsigned int args[4];
	char command_string[128];

	// make sure command is valid
	if (pwm_command > 0)
	{
		args[0] = (unsigned int)LPC_PWM1;
		args[1] = pwm_channel;
		args[2] = pwm_command;
		//args[3] = PWM_MATCH_UPDATE_NOW;
        args[3] = PWM_MATCH_UPDATE_NEXT_RST;
	
		strcpy(command_string, "PWM_MatchUpdate");
		//robo_host->robocaller(command_string,false,args,4);
        robo_host->robocaller(command_string,args,4);
	}
}

// initializing script for PWMDevice
void PWMDevice::initPWM()
{
	unsigned int args[5];
	char command_string[128];
	
    if (PWMDevice::isPeriodSet) // the period has been set
	{
        // set the pulse
        args[0] = pwm_channel;
        args[1] = pwm_pulseWidth;

        strcpy(command_string, "initMatch");
        //robo_host->robocaller(command_string,false,args,2);
        robo_host->robocaller(command_string,args,2);

        // additional needed commands
        args[0] = (unsigned int)LPC_PWM1;
        args[1] = pwm_channel;
        args[2] = ENABLE;
        strcpy(command_string, "PWM_ChannelCmd");
        //robo_host->robocaller(command_string,false,args,3);
        robo_host->robocaller(command_string,args,3);
    }
    else
	{
		// set the period of all pwm devices
		args[0] = 0;
		args[1] = pwm_period;// 20000us = 20ms = 50Hz
		strcpy(command_string, "initMatch");
		//robo_host->robocaller(command_string,false,args,2);
        robo_host->robocaller(command_string,args,2);
        
        // set the pulse
        args[0] = pwm_channel;
        args[1] = pwm_pulseWidth;

        strcpy(command_string, "initMatch");
        //robo_host->robocaller(command_string,false,args,2);
        robo_host->robocaller(command_string,args,2);
        
        // additional needed commands
        args[0] = (unsigned int)LPC_PWM1;
        args[1] = pwm_channel;
        args[2] = ENABLE;
        strcpy(command_string, "PWM_ChannelCmd");
        //robo_host->robocaller(command_string,false,args,3);
        robo_host->robocaller(command_string,args,3);
		
		// only perform these actions the first time the PWM periferal is used
		args[0] = (unsigned int)LPC_PWM1;
		strcpy(command_string, "PWM_ResetCounter");
		//robo_host->robocaller(command_string,false,args,1);
        robo_host->robocaller(command_string,args,1);
	
		args[1] = ENABLE;
		strcpy(command_string, "PWM_CounterCmd");
		//robo_host->robocaller(command_string,false,args,2);
        robo_host->robocaller(command_string,args,2);

		strcpy(command_string, "PWM_Cmd");
		//robo_host->robocaller(command_string,false,args,2);
        robo_host->robocaller(command_string,args,2);
		
		isPeriodSet = true;
	}

}

// create PWMDevice connected to Robovero host with the given channel
PWMDevice::PWMDevice(Robovero *host, unsigned int channel)
{
	// set the period and pulse width to nominal values
	pwm_period = 20000; // 20000us = 20ms = 50Hz
	pwm_pulseWidth = 1520; // 1520us = 1.5ms
	robo_host = host;
	pwm_channel = channel;

	// initialize the device
	initPWM();
}

// create PWMDevice connected to Robovero host with given, channel, period and pulseWidth
PWMDevice::PWMDevice(Robovero *host, unsigned int channel, unsigned int period, unsigned int pulse)
{
	pwm_period = period;
	pwm_pulseWidth = pulse;
	robo_host = host;
	pwm_channel = channel;

	// initialize the device
	initPWM();
}

// destructor ... doesn't have to do anything in this case
PWMDevice::~PWMDevice()
{
}
