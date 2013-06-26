/*
 *  Robovero.cpp
 *
 *  Robovero class to communicate with the Gumstix Robovero expansion board
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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <pthread.h>
#include <time.h>
#include <string>
#include <map>
#include "Robovero.h"
#include "lpc_types.h"

#define LPC_APB0_BASE         (0x40000000UL)
#define LPC_I2C0_BASE         (LPC_APB0_BASE + 0x1C000)
#define LPC_I2C0              (LPC_I2C0_BASE)


using namespace std;

void *reader_function(void *ptr);

// return the response from the Robovero
int Robovero::getReturn(int num_return, unsigned int* return_values)
{
	char returnMessage[128];
    char *str_tok;
	int nBytes, actual_return;
    actual_return = 0;

	nBytes = getMessage(returnMessage);
	if (nBytes > 0)
    {
        str_tok = strtok(returnMessage," ");
        while (str_tok != NULL && actual_return < num_return)
        {
            return_values[actual_return] = (unsigned int)strtoul(str_tok,NULL,16);
            actual_return++;
            str_tok = strtok(NULL," ");
        }
        return actual_return;
    }
	else
		return -1;
}

// call a function with no return values
// function is the string command, 
// args is a pointer to additional arguments with num_args indicating the number of arguments
// robocaller returns 0 or -1 on error and and something else otherwise (not super helpful)
int Robovero::robocaller(char* function, unsigned int* args, int num_args)
{
	char command_string[256];
	char log_string[256];
	unsigned int ret;
	int i, nBytes;

	// make sure we're connected
	if (serialPort <= -1)
	{
		perror("Robovero not connected, cannot use robocaller\n");
		return -1;
	}					

	// check to see if function is in dictionary
	if (functionDict[function] <= 0)
	{			
		// search and add to dictionary
		sprintf(command_string, "search %s\r\n", function);

		if (debug)
		{
			sprintf(log_string, "[%f] ADD TO DICTIONARY: %s\r\n", (double)clock()/CLOCKS_PER_SEC, function);
			write(log, log_string, strlen(log_string));
		}
		// don't allow 0 as command in dictionary
		ret = 0;
		while (ret <= 0)
		{
			write(serialPort, command_string, strlen(command_string));
			getReturn(1,&ret);
		}
		if (debug)
		{
			sprintf(log_string, "[%f] INDEX: %X\r\n", (double)clock()/CLOCKS_PER_SEC, ret);
			write(log, log_string, strlen(log_string));
		}

		if (ret < 0)
			perror("Error retrieving function index\n");

		functionDict[function] = ret;
	}

	// prepare the command
	// add the command args to the end of the command if they exist
	sprintf(command_string, "%X", functionDict[function]);
	for (i=0;i<num_args;i++)
	{
		sprintf(command_string,"%s %X",command_string,args[i]);
	}
	strcat(command_string,"\r\n");

	if (debug)
	{
		sprintf(log_string, "[%f] REQUEST: %s", (double)clock()/CLOCKS_PER_SEC, command_string);
		write(log, log_string, strlen(log_string));
	}
	write(serialPort, command_string, strlen(command_string));
			
    return ret;
}

// call a function and get back a response into return values
// function is the string command, num_return is the number of expected return values
// ret_values is a pointer to the returnd values
// args is a pointer to additional arguments with num_args indicating the number of arguments
// robocaller returns 0 or -1 on error and and something else otherwise (not super helpful)
int Robovero::robocaller(char* function, int num_return, unsigned int* return_values, unsigned int* args, int num_args)
{
	char command_string[256];
	char log_string[256];
	unsigned int ret;
	int i, nBytes;

	// make sure we're connected
	if (serialPort <= -1)
	{
		perror("Robovero not connected, cannot use robocaller\n");
		return -1;
	}					

	// check to see if function is in dictionary
	if (functionDict[function] <= 0)
	{			
		// search and add to dictionary
		sprintf(command_string, "search %s\r\n", function);

		if (debug)
		{
			sprintf(log_string, "[%f] ADD TO DICTIONARY: %s\r\n", (double)clock()/CLOCKS_PER_SEC, function);
			write(log, log_string, strlen(log_string));
		}
		// don't allow 0 as command in dictionary
		ret = 0;
		while (ret <= 0)
		{
			write(serialPort, command_string, strlen(command_string));
			getReturn(1,&ret);
		}
		if (debug)
		{
			sprintf(log_string, "[%f] INDEX: %X\r\n", (double)clock()/CLOCKS_PER_SEC, ret);
			write(log, log_string, strlen(log_string));
		}

		if (ret < 0)
			perror("Error retrieving function index\n");

		functionDict[function] = ret;
	}

	// prepare the command
	// add the command args to the end of the command if they exist
	sprintf(command_string, "%X", functionDict[function]);
	for (i=0;i<num_args;i++)
	{
		sprintf(command_string,"%s %X",command_string,args[i]);
	}
	strcat(command_string,"\r\n");

	if (debug)
	{
		sprintf(log_string, "[%f] REQUEST: %s", (double)clock()/CLOCKS_PER_SEC, command_string);
		write(log, log_string, strlen(log_string));
	}
	write(serialPort, command_string, strlen(command_string));
			
	// if we are requesting a return value
	if (num_return > 0)
	{
        // getReturn will return the number of values actually received
		ret = getReturn(num_return, return_values);
		
		if (debug)
		{
            //sprintf(log_string, "[%f] RESPONSE: %X\r\n", (double)clock()/CLOCKS_PER_SEC, ret);
            sprintf(log_string, "[%f] RESPONSE:", (double)clock()/CLOCKS_PER_SEC);
            // loop through and output the return values
            for(i=0;i<ret;i++)
            {
                sprintf(log_string, "%s %X", log_string, return_values[i]);
            }
            sprintf(log_string, "%s\r\n", log_string);
			write(log, log_string, strlen(log_string));
		}
				
		return ret;
	}	
	else
		return 0;
}

void* Robovero::malloc(unsigned int size)
{
	// need to alloc some memory on the robovero and pass the pointer back
	char fcn [] = "malloc";
    unsigned int ret_add;
	//return (void *)robocaller(fcn, true, &size, 1);
    robocaller(fcn,1,&ret_add,&size,1);
    return (void *)ret_add;
}

void Robovero::free(void* ptr)
{
	// need to free up memory pointed to by ptr
	char fcn [] = "free";
	unsigned int args[1];
	args[0] = (unsigned int)ptr;
	robocaller(fcn,args,1);
}

unsigned char Robovero::readC(void* ptr)
{
	// read data at the memory address
    // return single char (byte)
	char fcn [] = "deref";
	unsigned int args[2];
    unsigned int ret_value;
	args[0] = (unsigned int)ptr;
	args[1] = 1; // how many bytes
	//return (unsigned char)robocaller(fcn,true,args,2);
    robocaller(fcn,1,&ret_value,args,2);
    return (unsigned char)ret_value;
}

unsigned short Robovero::readS(void* ptr)
{
	// read data at the memory address
    // return an unsigned short (2 bytes)
	char fcn [] = "deref";
	unsigned int args[2];
    unsigned int ret_value;
	args[0] = (unsigned int)ptr;
	args[1] = 2; // how many bytes
	//return (unsigned short)robocaller(fcn,true,args,2);
    robocaller(fcn,1,&ret_value,args,2);
    return (unsigned short)ret_value;
}

// void Robovero::store(void* ptr, unsigned int width, unsigned char data)
// {
// 	// store data at the memory location
// 	char fcn [] = "deref";
// 	unsigned int args[3];
// 	args[0] = (unsigned int)ptr;
// 	args[1] = width;
// 	args[2] = data;
// 	robocaller(fcn,false,args,3);
// }

void Robovero::store(void* ptr, unsigned char data)
{
	// store data at the memory location
    // data is always a single byte
	char fcn [] = "deref";
	unsigned int args[3];
	args[0] = (unsigned int)ptr;
	args[1] = 1;
	args[2] = data;
	robocaller(fcn,args,3);
}

// create a Robovero object without the debug log
Robovero::Robovero (const char* port)
{			
	debug = false;
	init(port);
}

// create a Robovero object with a debug log stored at logFile location
Robovero::Robovero (const char* port, const char* logFile)
{
	debug = true;
	log = open(logFile, O_WRONLY | O_APPEND | O_CREAT, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH );
	if (log == -1)
	{
		printf("Could not open logging file\n");
		debug = false;
	}
	init(port);
}

// delete the Robovero object
Robovero::~Robovero ()
{
	pthread_cancel(threadReader);
	pthread_join(threadReader, NULL);
	close(serialPort);
	if (debug)
		close(log);
}

// wait for a message to come back from the Robovero
int Robovero::getMessage(char* buffer)
{
	// counter to avoid infinite wait
	int waitCounter = 0;
    
	if (serialPort > -1)
	{
		int messageLength;
		while (strlen(message)<1)
		{
			// waiting too long
			if (waitCounter > 100)
			{
				printf("Waited too long for message\n");
				return -1;
			}
			usleep(1);
			waitCounter++;
		}

		messageLength = strlen(message);
		memcpy(buffer, message, messageLength);
		buffer[messageLength] = '\0';
		message[0] = '\0';
		return messageLength;
	}
	else
		return -1;
}

// initialize the new Robovero object
void Robovero::init(const char* port)
{
	int nBytes;
	/*int thread_args[2];*/
	char fcn [] = "roboveroConfig";
	struct termios options;
    
    unsigned int args[5];

    //serialPort = open(port, O_RDWR | O_NDELAY);
    serialPort = open(port, O_RDWR | O_NONBLOCK);
	tcgetattr(serialPort, &options);
    cfmakeraw(&options);
    cfsetispeed(&options, B115200);
    cfsetospeed(&options, B115200);
    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~PARENB; // no parity
    options.c_cflag &= ~CSTOPB; // one stop bit
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8; // 8 data bits

    tcsetattr(serialPort, TCSAFLUSH, &options);
    tcflush(serialPort, TCIOFLUSH);
	

	if (serialPort == -1)
	{
		/* couldn't open port */
		perror("open_port: Unable to open port /dev/ttyACM0\n");
	}
	else
	{
		fcntl(serialPort, F_SETFL, FNDELAY);

		// now I'm going to send the initialize characters
		nBytes = write(serialPort, "\r\n", 2);
		nBytes = write(serialPort, "promptOff\r\n", 11);

		// pause to clear the buffer
		sleep(1);
		if (tcflush(serialPort, TCIFLUSH) != 0)
		{
			perror("Error flushing buffer\n");
		}

		// I'm going to open up a thread to read the data
		readerID = pthread_create( &threadReader, NULL, reader_function, (void*) this);
	}
	// record the start time
	start_time = time (NULL);

	// call the config routine
	robocaller(fcn,NULL,0);
    
    /* change the i2c bus speed */
    /* try setting clock speed to 400khz */
    strcpy(fcn,"I2C_Cmd");
    args[0] = (unsigned int)LPC_I2C0;
    args[1] = (unsigned int)DISABLE;
    robocaller(fcn,args,2);
    
    strcpy(fcn,"I2C_Init");
    args[0] = (unsigned int)LPC_I2C0;
    args[1] = (unsigned int)400000;
    robocaller(fcn,args,2);

    strcpy(fcn,"I2C_Cmd");
    args[0] = (unsigned int)LPC_I2C0;
    args[1] = (unsigned int)ENABLE;
    robocaller(fcn,args,2);

}

// thread function which reads the data from the serial port
void *reader_function(void *ptr)
{
	struct timespec ts;

	Robovero *caller = (Robovero *)ptr;
	int fd = caller->serialPort;

	char buffer[256];
	char temp_buffer[256];
	int nBytes, newBytes;

	char *CRLFptr;

	ts.tv_sec = 0;
	ts.tv_nsec = 500000; //500 usec

	buffer[0] = '\0';
	nBytes = 0;
	while (ptr)
	{
		CRLFptr = strstr(buffer, "\r\n");
		while (CRLFptr == NULL)
		{
			temp_buffer[0] = '\0';
			newBytes = read(fd, temp_buffer, 256);
			if (newBytes > 0)
			{
				memcpy(buffer+nBytes,temp_buffer,newBytes);
				nBytes+=newBytes;
                //printf("buffer is now %s\n", buffer);
			}

			// check the buffer for a \r\n
			CRLFptr = strstr(buffer, "\r\n");
		}
		// grab the received message without the \r\n
		memcpy(caller->message,buffer,CRLFptr-buffer);
		caller->message[CRLFptr-buffer] = '\0';
       // printf("received message = %s\n",caller->message);
		
		// move the remainder of the buffer to the head
		nBytes -= (CRLFptr-buffer)+2;
		memcpy(buffer,CRLFptr+2,nBytes);
		buffer[nBytes] = '\0';

		nanosleep(&ts,NULL);
	}
}
