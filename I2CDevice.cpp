/*
 *  I2CDevice.cpp
 *
 *  Robovero class to communicate with an I2C device attached to the Gumstix Robovero expansion board.
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

#include "I2CDevice.h"
#include "lpc17xx_i2c.h"
#include "lpc_types.h"
#include <stdio.h>
#include <string.h>
#include <errno.h>

// read single byte from device
unsigned char I2CDevice::read()
{
	int ret;
    unsigned int ret_value;
	unsigned int args[5];
	char command_string[128];

	//robo_host->store(tx_data,1,(unsigned int)reg);
    errno = 0;

	args[0] = (unsigned int)config;
    args[1] = 0;
	strcpy(command_string,"I2C_M_SETUP_Type_tx_length");
    robo_host->robocaller(command_string,args,2);
	
	args[1] = (unsigned int)rx_data;
	strcpy(command_string,"I2C_M_SETUP_Type_rx_data");
	robo_host->robocaller(command_string,args,2);

	args[1] = 1;
	strcpy(command_string,"I2C_M_SETUP_Type_rx_length");
	robo_host->robocaller(command_string,args,2);

	args[0] = (unsigned int)LPC_I2C0;
	args[1] = (unsigned int)config;
	args[2] = I2C_TRANSFER_POLLING; 
	strcpy(command_string,"I2C_MasterTransferData");
	ret = robo_host->robocaller(command_string,1,&ret_value,args,3);

	if (ret == ERROR)
    {
        sprintf(command_string,"I2C Read Error -1 [%d]",errno);
        perror(command_string);
        //perror("I2C Read Error - 1 [%d]",errno);
    }

	//ret = robo_host->read(rx_data,1);	
	//return ret;
    return robo_host->readC(rx_data);
}

// read single byte from address given by reg
unsigned char I2CDevice::readReg(void* reg)
{
	int ret;
    unsigned int ret_value;
	unsigned int args[5];
	char command_string[128];

	//robo_host->store(tx_data,1,(unsigned int)reg);
    robo_host->store(tx_data,(unsigned int)reg);

	args[0] = (unsigned int)config;
        args[1] = 1;
	strcpy(command_string,"I2C_M_SETUP_Type_tx_length");
        robo_host->robocaller(command_string,args,2);
	
	args[1] = (unsigned int)rx_data;
	strcpy(command_string,"I2C_M_SETUP_Type_rx_data");
	robo_host->robocaller(command_string,args,2);

	args[1] = 1;
	strcpy(command_string,"I2C_M_SETUP_Type_rx_length");
	robo_host->robocaller(command_string,args,2);

	args[0] = (unsigned int)LPC_I2C0;
	args[1] = (unsigned int)config;
	args[2] = I2C_TRANSFER_POLLING; 
	strcpy(command_string,"I2C_MasterTransferData");
	ret = robo_host->robocaller(command_string,1,&ret_value,args,3);

	if (ret == ERROR)
    {
        sprintf(command_string,"I2C Read Error -2 [%d]",errno);
        perror(command_string);
    }
		//perror("I2C Read Error - 2 [%d]",errno);

	//ret = robo_host->read(rx_data,1);	
	//return ret;
    return robo_host->readC(rx_data);
}

// read 6 sequential bytes from starting address given by reg
void I2CDevice::read6Reg(void* reg, unsigned short* values)
{
	unsigned int ret, ret_value;
	char command_string[128];
	unsigned int args[5];

	//robo_host->store(tx_data,1,(unsigned int)reg | 0x80); //0b10000000); most significant bit needs to be 1 for sequential reads
    robo_host->store(tx_data,(unsigned int)reg | 0x80); //0b10000000); most significant bit needs to be 1 for sequential reads
    
	args[0] = (unsigned int)config;
        args[1] = 1;
	strcpy(command_string,"I2C_M_SETUP_Type_tx_length");
        robo_host->robocaller(command_string,args,2);
	
	args[1] = (unsigned int)rx_data6;
	strcpy(command_string,"I2C_M_SETUP_Type_rx_data");
	robo_host->robocaller(command_string,args,2);

	args[1] = 6;
	strcpy(command_string,"I2C_M_SETUP_Type_rx_length");
	robo_host->robocaller(command_string,args,2);

	args[0] = (unsigned int)LPC_I2C0;
	args[1] = (unsigned int)config;
	args[2] = I2C_TRANSFER_POLLING; 
	strcpy(command_string,"I2C_MasterTransferData");
	ret = robo_host->robocaller(command_string,1,&ret_value,args,3);

	if (ret == ERROR)
    {
        sprintf(command_string,"I2C Read Error -3 [%d]",errno);
        perror(command_string);
    }
//		perror("I2C Read Error - 3 [%d]",errno);

// 	ret = robo_host->read(rx_data6,2);
// 	values[0] = ret;
// 	ret = robo_host->read((void *)((int)rx_data6+2),2);
// 	values[1] = ret;
// 	ret = robo_host->read((void *)((int)rx_data6+4),2);
// 	values[2] = ret;
    
    ret = robo_host->readS(rx_data6);
	values[0] = ret;
	ret = robo_host->readS((void *)((int)rx_data6+2));
	values[1] = ret;
	ret = robo_host->readS((void *)((int)rx_data6+4));
	values[2] = ret;    
}

// read 6 sequential bytes from starting address given by reg
// don't cast the bytes into shorts, leave as chars
void I2CDevice::read6Reg(void* reg, unsigned char* values)
{
	unsigned int ret, ret_value;
	char command_string[128];
	unsigned int args[5];

	//robo_host->store(tx_data,1,(unsigned int)reg | 0x80); //0b10000000); most significant bit needs to be 1 for sequential reads
    robo_host->store(tx_data,(unsigned int)reg | 0x80); //0b10000000); most significant bit needs to be 1 for sequential reads
    
	args[0] = (unsigned int)config;
        args[1] = 1;
	strcpy(command_string,"I2C_M_SETUP_Type_tx_length");
        robo_host->robocaller(command_string,args,2);
	
	args[1] = (unsigned int)rx_data6;
	strcpy(command_string,"I2C_M_SETUP_Type_rx_data");
	robo_host->robocaller(command_string,args,2);

	args[1] = 6;
	strcpy(command_string,"I2C_M_SETUP_Type_rx_length");
	robo_host->robocaller(command_string,args,2);

	args[0] = (unsigned int)LPC_I2C0;
	args[1] = (unsigned int)config;
	args[2] = I2C_TRANSFER_POLLING; 
	strcpy(command_string,"I2C_MasterTransferData");
	ret = robo_host->robocaller(command_string,1,&ret_value,args,3);

	if (ret == ERROR)
    {
        sprintf(command_string,"I2C Read Error -3 [%d]",errno);
        perror(command_string);
    }
//		perror("I2C Read Error - 3 [%d]",errno);

// 	ret = robo_host->read(rx_data6,1);
// 	values[0] = ret;
// 	ret = robo_host->read((void *)((int)rx_data6+2),2);
// 	values[1] = ret;
// 	ret = robo_host->read((void *)((int)rx_data6+4),2);
// 	values[2] = ret;
 	values[0] = robo_host->readC(rx_data6);
    values[1] = robo_host->readC((void *)((int)rx_data6+1));
	values[2] = robo_host->readC((void *)((int)rx_data6+2));
    values[3] = robo_host->readC((void *)((int)rx_data6+3));
	values[4] = robo_host->readC((void *)((int)rx_data6+4));
    values[5] = robo_host->readC((void *)((int)rx_data6+5));	
}


// read 6 sequential bytes from starting address
void I2CDevice::read6(unsigned char* values)
{
	unsigned int ret, ret_value;
	char command_string[128];
	unsigned int args[5];

	//robo_host->store(tx_data,1,(unsigned int)reg | 0x80); //0b10000000); most significant bit needs to be 1 for sequential reads
	args[0] = (unsigned int)config;
    args[1] = 0;
	strcpy(command_string,"I2C_M_SETUP_Type_tx_length");
    robo_host->robocaller(command_string,args,2);
	
	args[1] = (unsigned int)rx_data6;
	strcpy(command_string,"I2C_M_SETUP_Type_rx_data");
	robo_host->robocaller(command_string,args,2);

	args[1] = 6;
	strcpy(command_string,"I2C_M_SETUP_Type_rx_length");
	robo_host->robocaller(command_string,args,2);

	args[0] = (unsigned int)LPC_I2C0;
	args[1] = (unsigned int)config;
	args[2] = I2C_TRANSFER_POLLING; 
	strcpy(command_string,"I2C_MasterTransferData");
	ret = robo_host->robocaller(command_string,1,&ret_value,args,3);

	if (ret == ERROR)
    {
        sprintf(command_string,"I2C Read Error -4 [%d]",errno);
        perror(command_string);
    }
//		perror("I2C Read Error - 3 [%d]",errno);

// 	ret = robo_host->read(rx_data6,1);
// 	values[0] = ret;
// 	ret = robo_host->read((void *)((int)rx_data6+1),1);
// 	values[1] = ret;
//     ret = robo_host->read((void *)((int)rx_data6+2),1);
// 	values[2] = ret;
// 	ret = robo_host->read((void *)((int)rx_data6+3),1);
// 	values[3] = ret;
// 	ret = robo_host->read((void *)((int)rx_data6+4),1);
// 	values[4] = ret;
// 	ret = robo_host->read((void *)((int)rx_data6+5),1);
// 	values[5] = ret;
    
    values[0] = robo_host->readC(rx_data6);
    values[1] = robo_host->readC((void *)((int)rx_data6+1));
    values[2] = robo_host->readC((void *)((int)rx_data6+2));
    values[3] = robo_host->readC((void *)((int)rx_data6+3));
    values[4] = robo_host->readC((void *)((int)rx_data6+4));
    values[5] = robo_host->readC((void *)((int)rx_data6+5));
}

// write value to address given by reg
void I2CDevice::writeReg(unsigned char reg, unsigned char value)
{
	int ret;
    unsigned int ret_value;
	unsigned int args[5];
	char command_string[128];

// 	robo_host->store(tx_data,1,(unsigned char)reg);
// 	robo_host->store((void*)((int)tx_data+1),1,value);
    robo_host->store(tx_data,reg);
	robo_host->store((void*)((int)tx_data+1),value);
	
	args[0] = (unsigned int)config;
	args[1] = 2;
	strcpy(command_string,"I2C_M_SETUP_Type_tx_length");
	robo_host->robocaller(command_string,args,2);

	args[1] = 0;
	strcpy(command_string,"I2C_M_SETUP_Type_rx_data");
	robo_host->robocaller(command_string,args,2);

	strcpy(command_string,"I2C_M_SETUP_Type_rx_length");
	robo_host->robocaller(command_string,args,2);

	args[0] = (unsigned int)LPC_I2C0;
	args[1] = (unsigned int)config;
	args[2] = I2C_TRANSFER_POLLING; 
	strcpy(command_string,"I2C_MasterTransferData");
	ret = robo_host->robocaller(command_string,1,&ret_value,args,3);

	if (ret == ERROR)
        //printf("I2C Write Error");
		perror("I2C Write Error");

	if (readReg((void *)reg) != value)
        //printf("I2C Verification Error");
		perror("I2C Verification Error");
}

// write value to I2CDevice
void I2CDevice::writeReg(unsigned char value)
{
	int ret;
	unsigned int args[5];
	char command_string[128];

	//robo_host->store(tx_data,1,value);
    robo_host->store(tx_data,value);

    
	args[0] = (unsigned int)config;
	args[1] = 1;
	strcpy(command_string,"I2C_M_SETUP_Type_tx_length");
	robo_host->robocaller(command_string,args,2);

	args[1] = 0;
	strcpy(command_string,"I2C_M_SETUP_Type_rx_data");
	robo_host->robocaller(command_string,args,2);

	strcpy(command_string,"I2C_M_SETUP_Type_rx_length");
	robo_host->robocaller(command_string,args,2);

	args[0] = (unsigned int)LPC_I2C0;
	args[1] = (unsigned int)config;
	args[2] = I2C_TRANSFER_POLLING; 
	strcpy(command_string,"I2C_MasterTransferData");
	//ret = robo_host->robocaller(command_string,true,args,3);
    robo_host->robocaller(command_string,args,3);

// 	if (ret == ERROR)
//         //printf("I2C Write Error");
// 		perror("I2C Write Error");
// 
// 	if (readReg(reg) != value)
//         //printf("I2C Verification Error");
// 		perror("I2C Verification Error");
}


// create I2CDevice connected to Robovero host at address
I2CDevice::I2CDevice(Robovero* host, unsigned int address)
{
	unsigned int args[5];
	int ret;
    unsigned int ret_value;
	char command_string[128];        

	robo_host = host;
    
    /* try setting clock speed to 400khz 
    strcpy(command_string,"I2C_Init");
    args[0] = (unsigned int)LPC_I2C0;
    args[1] = (unsigned int)400000;
    robo_host->robocaller(command_string,false,args,2);*/
    
	strcpy(command_string,"I2C_M_SETUP_Type_malloc");
	ret = robo_host->robocaller(command_string,1,(unsigned int*)&config,NULL,0);
	tx_data = robo_host->malloc(2);
	rx_data = robo_host->malloc(1);
	rx_data6 = robo_host->malloc(6);

	args[0] = (unsigned int)config;
	args[1] = address;
	strcpy(command_string,"I2C_M_SETUP_Type_sl_addr7bit");
	robo_host->robocaller(command_string,args,2);
	
	args[1] = (unsigned int)tx_data;
	strcpy(command_string,"I2C_M_SETUP_Type_tx_data");
	robo_host->robocaller(command_string,args,2);
	
	args[1] = 3;
	strcpy(command_string,"I2C_M_SETUP_Type_retransmissions_max");
	robo_host->robocaller(command_string,args,2);
}

// free up memory locations on Robovero
I2CDevice::~I2CDevice()
{
	robo_host->free(config);
	robo_host->free(tx_data);
	robo_host->free(rx_data);
	robo_host->free(rx_data6);
}
