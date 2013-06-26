/*
 *  Robovero.h
 *
 *  Robovero class header to communicate with the Gumstix Robovero expansion board
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
#define Robovero_H

#include <pthread.h>
#include <time.h>
#include <string>
#include <map>

using namespace std;

class Robovero
{
	int log;
	bool debug;
	pthread_t threadReader;
	int readerID;
	time_t start_time;
	map <string,int> functionDict;

	public:
		// serialPort and message are public so thread can access them
		int serialPort;
		char message[256];
		
		// returns up to num_return integers in the response into return_values buffer and returns the actual amount returned
		int getReturn(int num_return, unsigned int* return_values);
		// makes calls to the robovero functions, expects no return
		int robocaller(char* function, unsigned int* args, int num_args);
        // makes calls to the robovero functions, return values are placed in return_values
        int robocaller(char* function, int num_return, unsigned int* return_values, unsigned int* args, int num_args);
		// allocate memory and return address
		void* malloc(unsigned int size);
		// free memory
		void free(void* ptr);
		// retrieve value at address (deref)
		//unsigned char read(void* ptr, unsigned int width);
        // return single byte or two bytes
        unsigned char readC(void* ptr);
        unsigned short readS(void *ptr);
		// store data at address
		//void store(void* ptr, unsigned int width, unsigned char data);
		void store(void* ptr, unsigned char data);

		Robovero (const char* port);
		Robovero (const char* port, const char* logFile);
		~Robovero ();

	private:		
		int getMessage(char* buffer);
		void init(const char* port);
};	

#endif
