/*
 *  Robovero_SF.cpp
 *
 *  S-Function Driver to use the Gumstix Robovero
 *  Andrew C. Smith
 *  Aerospace Robotics Lab, Stanford University
 *  acsmith@stanford.edu
 *  
 *  March 22 2013
 *
 *  v1.1 Initial driver
 *  v1.2 Added ADCDevice to output 8 (7 working) analog channels
 *      September 25 2013
 *
 */


#define S_FUNCTION_NAME Robovero_SF
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"

#if defined(MATLAB_MEX_FILE)
    // Put simulation code here
#else
    // Put general real-time code here (applicable for every target)
    #if defined(__INTIME__)
        // Put InTime code here
    #elif defined(UNDER_RTSS)
        // Put RTX code here
    #elif defined(_WIN32)
        // Put Windows 32-bit code here
    #elif defined(_WIN64)
        // Put Windows 64-bit code here
    #elif defined(__QNX__)
        // Put general QNX code here (any QNX target)
        #if defined(__X86__)
            // Put QNX x86 specific code here
        #elif defined(__PPC__)
            // Put QNX PPC specific code here
        #elif defined(__MIPS__)
            // Put QNX MIPS specific code here
        #elif defined(__SH__)
            // Put QNX SuperH specific code here
        #elif defined(__ARM__)
            // Put QNX ARM specific code here
        #else
            #error QNX target not recognized!
        #endif
    #elif defined(__linux)
        // Put general Linux code here (gumstix or x86)
        #include <stdio.h>
        #include <stdlib.h>
        #include "Robovero.h"
        #include "I2CDevice.h"
        #include "PWMDevice.h"

        #if defined(__x86__)
            // Put Linux x86 specific code here
        #elif defined(__arm__)
            // Put Linux ARM (gumstix) specific code here
        #else
            #error Unrecognized Linux target!
        #endif
    #else
        #error Unrecognized target!
    #endif
#endif

#define NPARAMS 1
        
#define SAMPLE_TIME(S)  (real_T)mxGetScalar(ssGetSFcnParam(S, 0))


/*====================*
 * S-function methods *
 *====================*/
#define MDL_CHECK_PARAMETERS   /* Change to #undef to remove function */
#if defined(MDL_CHECK_PARAMETERS) && defined(MATLAB_MEX_FILE)
static void mdlCheckParameters(SimStruct *S)
{
  // code to check if parameters are ok, only works in simulation
  // not valid for real time
}
#endif /* MDL_CHECK_PARAMETERS */

/* Function: mdlInitializeSizes ===============================================
 * Abstract:
 *    The sizes information is used by Simulink to determine the S-function
 *    block's characteristics (number of inputs, outputs, states, etc.).
 */
static void mdlInitializeSizes(SimStruct *S)
{
    int_T nInputPorts  = 1;               /* number of input ports  */
    int_T dimInputPorts[1] = {6};                 /* dimension array of input port sizes */
    int_T needsInput[1] = {0};                    /* array of direct feedthrough */
    
    int_T nOutputPorts = 4;               /* number of output ports */
    int_T dimOutputPorts[4] = {3,3,3,8};                /* dimension array of output port sizes */
                                          /* outputs are accel, magnetometer, gyro, ADC (channels 0-3,5-7), channel 4 will always be equal to 0 */
    
    int_T nContStates = 0;   /* number of continuous states */
    int_T nDiscStates = 0;   /* number of discrete states */  /* past pwm outputs */
    
    int_T nDWork = 0;        /* size of D (data) work vector */
    int_T nRWork = 0;        /* size of R (real) work vector */
    int_T nIWork = 0;        /* size of I (integer) work vector */
    int_T nPWork = 7;        /* size of P (pointer) work vector */
    int_T nMWork = 0;        /* size of M (mode) work vector */
    int_T nZCWork = 0;       /* size of ZC (zero crossings) work vector */    
    
    int_T i = 0;             /* for loop counter */

    ssSetNumSFcnParams(S, NPARAMS);  /* Number of expected parameters */
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S))
    {
        /*
         * If the the number of expected input parameters is not
         * equal to the number of parameters entered in the 
         * dialog box, return. Simulink will generate an error 
         * indicating that there is aparameter mismatch.
         */
        return;
    }
    else
    {
        /* check the parameters to make sure they're valid */
        /* only works for simulation */
        #if defined(MATLAB_MEX_FILE)
            mdlCheckParameters(S);
            if (ssGetErrorStatus(S) != NULL)
                return;
        #endif
    }
    
    /* Set the number of continuous and discrete states */
    ssSetNumContStates(S, nContStates);
    ssSetNumDiscStates(S, nDiscStates);

    /*
     * Configure the input ports. First set the number of input
     * ports. 
     */
    if (!ssSetNumInputPorts(S, nInputPorts)) return;    
    
    /*
     * Set input port dimensions for each input port index 
     * starting at 0.
    */
    for (i=0; i < nInputPorts; i++)
    {
        if(ssSetInputPortVectorDimension(S, i, dimInputPorts[i])==0) return;
       /*
        * Set direct feedthrough flag (1=yes, 0=no).
       */
        ssSetInputPortDirectFeedThrough(S, i, needsInput[i]);
    }    

    /*
     * Configure the output ports. First set the number of 
     * output ports.
     */
    if (!ssSetNumOutputPorts(S, nOutputPorts)) return;

    /*
     * Set output port dimensions for each output port index 
     * starting at 0.
     */
    for (i=0; i < nOutputPorts; i++)
    {
        if(ssSetOutputPortVectorDimension(S, i, dimOutputPorts[i])==0) return;
    }
    
    ssSetOutputPortDataType(S,3,SS_UINT16);
    
    /*
     * Set the number of sample times.     */
    ssSetNumSampleTimes(S, 1);   

    /*
     * Set size of the work vectors.
     */
    ssSetNumDWork(S, nDWork);   /* data vector */
    ssSetNumRWork(S, nRWork);   /* real vector    */
    ssSetNumIWork(S, nIWork);   /* integer vector */
    ssSetNumPWork(S, nPWork);   /* pointer vector */
    ssSetNumModes(S, nMWork);   /* mode vector    */
    ssSetNumNonsampledZCs(S, nZCWork);   /* zero crossings */

    ssSetOptions(S, 0);

} /* end mdlInitializeSizes */

/* Function: mdlInitializeSampleTimes =========================================
 * Abstract:
 *    Specifiy that we have a continuous sample time.
 */
static void mdlInitializeSampleTimes(SimStruct *S)
{
    real_T time;
    
    // get sample time from parameters
    time = SAMPLE_TIME(S);

    if (time == -1)
    {
        ssSetSampleTime(S, 0, INHERITED_SAMPLE_TIME);
        ssSetModelReferenceSampleTimeDefaultInheritance(S);
    }
    else
        ssSetSampleTime(S, 0, time);
    
    ssSetOffsetTime(S, 0, 0.0);
}

#define MDL_START
#if defined(MDL_START)
/* Function: mdlStart ===========================================================
 * Abstract:
 *    This function is called once at start of model execution. If you
 *    have states that should be initialized once, this is the place
 *    to do it.
 */
static void mdlStart(SimStruct *S)
{
    #if defined(__linux)

        Robovero *robo1;
        I2CDevice *accel_p, *magneto_p, *gyro_p;
        PWMDevice *servo1_p, *servo4_p;
        ACDDevice *adc_p;
        
        // create Robovero object
        //robo1 = new Robovero("/dev/ttyACM0","SF_logging.log");
        robo1 = new Robovero("/dev/ttyACM0");

        // save the Robovero pointer to the PWork vector
        ssSetPWorkValue(S, 0, robo1);
        
        // create the accelerometer I2C device
        accel_p = new I2CDevice(robo1,0x18);
        // save it to the PWork vector
        ssSetPWorkValue(S, 1, accel_p);
        // initialize the accelerometer
        accel_p->writeReg((void *)0x20,0x27);
        accel_p->writeReg((void *)0x23,0x00);
    
        // create the magnetometer I2C device
        magneto_p = new I2CDevice(robo1,0x1E);
        // save it to the PWork vector
        ssSetPWorkValue(S, 2, magneto_p);
        // initialize the magnetometer
        magneto_p->writeReg((void *)0x00,0x18);
        magneto_p->writeReg((void *)0x01,0x20);
        magneto_p->writeReg((void *)0x02,0x00);

        // create the gyro I2C device
        gyro_p = new I2CDevice(robo1, 0x68);
        // save it to the PWork vector
        ssSetPWorkValue(S, 3, gyro_p);
        // initalize the gyro
        gyro_p->writeReg((void *)0x22,0x08);
        gyro_p->writeReg((void *)0x23,0x80);
        gyro_p->writeReg((void *)0x20,0x0F);
        
        // initalize the PWM devices and save their pointers to PWork
        servo1_p = new PWMDevice(robo1,1);
        ssSetPWorkValue(S, 4, servo1_p);        
        servo4_p = new PWMDevice(robo1,4);
        ssSetPWorkValue(S, 5, servo4_p);
        
        // initalize the ADC and save pointer to PWork
        adc_p = new ADCDevice(robo1);
        ssSetPWorkValue(S, 6, adc_p);
    
    #endif    
}
#endif // MDL_START

#define MDL_INITIALIZE_CONDITIONS
/* Function: mdlInitializeConditions ========================================
 * Abstract:
 *    Initialize all the states with the parameters
 */
static void mdlInitializeConditions(SimStruct *S)
{
}

/* Function: mdlOutputs =======================================================
 * Abstract:
 *      y = Cx + Du 
 */
static void mdlOutputs(SimStruct *S, int_T tid)
{
    // output vectors
    real_T *accel_out = ssGetOutputPortRealSignal(S,0);
    real_T *compass_out = ssGetOutputPortRealSignal(S,1);
    real_T *gyro_out = ssGetOutputPortRealSignal(S,2);    
    uint16_T *ADC_out = (uint16_T *)ssGetOutputPortSignal(S,3);
    
    // counter
    int i = 0;
    
    #if defined(__linux)

        // vector of return values
        unsigned short ret_values[3];
        I2CDevice *accel_p, *magneto_p, *gyro_p;
        ADCDevice adc_p;

        // get the accelerometer pointer from PWork
        accel_p = (I2CDevice*)ssGetPWorkValue(S, 1);
        // read the data register, 3 axis x 2 bytes/axis = 6 bytes
		accel_p->read6Reg((void *)0x28, ret_values);
        // output the accel data, properly scaled in +/- 2g maximum
        accel_out[0] = ((double)((short)ret_values[0]))/16384.0;
        accel_out[1] = ((double)((short)ret_values[1]))/16384.0;
        accel_out[2] = ((double)((short)ret_values[2]))*(2.0/32767.0);
  
        // get the magnetometer pointer from PWork
        magneto_p = (I2CDevice*)ssGetPWorkValue(S, 2);
        // read the data register, 3 axis x 2 bytes/axis = 6 bytes
		magneto_p->read6Reg((void *)0x03, ret_values);	
        // output the magnetometer data, properly scaled in +/- Gauss (1.3 maximum)
        compass_out[0] = ((double)((short)ret_values[0]))/1055.0*1.3;
        compass_out[1] = ((double)((short)ret_values[1]))/1055.0*1.3;
        compass_out[2] = ((double)((short)ret_values[2]))/950.0*1.3;
        
        // get the gyro pointer from PWork
        gyro_p = (I2CDevice*)ssGetPWorkValue(S, 3);
        // read the data register, 3 axis x 2 bytes/axis = 6 bytes
		gyro_p->read6Reg((void*)0x28, ret_values);
        // output the gyro data, properly scaled in degrees/s (maximum 250)
        gyro_out[0] = ((double)((short)ret_values[0]))*(250.0/32767.0);
        gyro_out[1] = ((double)((short)ret_values[1]))*(250.0/32767.0);
        gyro_out[2] = ((double)((short)ret_values[2]))*(250.0/32767.0);
        
        // get the ADC pointer from PWork
        adc_p = (ADCDevice*)ssGetPWorkValue(S, 6);
        // output the ADC values
        for (i=0;i<8;i++)
        {
            // don't do channel 4
            if (i != 4)
            {
                ADC_out[i] = adc_p->readADC(i);
            }
        }        
		
    #endif
                
}

#define MDL_DERIVATIVES
/* Function: mdlDerivatives =================================================
 * Abstract:
 *      xdot = Ax + Bu
 */
static void mdlDerivatives(SimStruct *S)
{
}

#define MDL_UPDATE
/* Function: mdlUpdate ========================================================
* Abstract:
* This function is called once for every major integration time step.
* Discrete states are typically updated here, but this function is useful
* for performing any tasks that should only take place once per integration
* step.
*/    
static void mdlUpdate(SimStruct *S, int_T tid)
{
    // get PWM motor input commands
    InputRealPtrsType servo_cmd = ssGetInputPortRealSignalPtrs(S,0);
    
    #if defined(__linux)
    
        // get the PWM device pointers from PWork vector
        PWMDevice *servo1_p, *servo4_p;
        servo1_p = (PWMDevice*)ssGetPWorkValue(S, 4);
        servo4_p = (PWMDevice*)ssGetPWorkValue(S, 5);

        // apply the command to the motors, casting it as uint
        // have to test out servo/motor range to find possible input range
        servo1_p->move((unsigned int)*servo_cmd[0]);
        servo4_p->move((unsigned int)*servo_cmd[3]);

    #endif

}

/* Function: mdlTerminate =====================================================
 * Abstract:
 *    No termination needed, but we are required to have this routine.
 */
static void mdlTerminate(SimStruct *S)
{
    #if defined(__linux)
    
        Robovero *robo1;
        I2CDevice *accel_p, *magneto_p, *gyro_p;
        PWMDevice *servo1_p, *servo4_p;
        ADCDevice *adc_p;

        // get all the pointers from PWork
        robo1 = (Robovero*)ssGetPWorkValue(S, 0);
        accel_p = (I2CDevice*)ssGetPWorkValue(S, 1);
        magneto_p = (I2CDevice*)ssGetPWorkValue(S, 2);
        gyro_p = (I2CDevice*)ssGetPWorkValue(S, 3);
        servo1_p = (PWMDevice*)ssGetPWorkValue(S, 4);
        servo1_p = (PWMDevice*)ssGetPWorkValue(S, 5);
        adc_p = (ADCDevice*)ssGetPWorkValue(S, 6);

        // remove from memory and shutdown
        delete adc_p;
        delete servo4_p;
        delete servo1_p;
        delete gyro_p;
        delete magneto_p;
        delete accel_p;
        delete robo1;
    #endif
    
    UNUSED_ARG(S); /* unused input argument */
}

#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
    #include "simulink.c"      /* MEX-file interface mechanism */
#else
    #include "cg_sfun.h"       /* Code generation registration function */
#endif  