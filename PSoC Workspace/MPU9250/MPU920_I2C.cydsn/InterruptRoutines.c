
#include "InterruptRoutines.h" 
#include "project.h" 
#include "MPU9250.h"
#include "MPU9250_RegMap.h"
#include "MPU9250_I2C.h"
#include "stdio.h"
#include "Fusion.h"
#include "FusionAhrs.h"
#include "FusionBias.h"
#include "FusionCalibration.h"
#include "FusionCompass.h"
#include "FusionTypes.h"
#include "math.h"
#include "inttypes.h"
#include "stdlib.h"
#include "stdio.h"
#include "functions.h"

#define TIME_CALIBRATION 10 //seconds

int32 value_digit; 
int32 value_mv; 
uint8 ch_receveid; 
uint8 SendBytesFlag=0;

extern int16 state;
extern bool flag_calibration_todo;

uint16_t count_10ms_time = 0;
uint16_t count_10ms_time_thr = 0;
uint16_t count_10ms_time_2s = 0;
uint16_t count_MPU9250;
uint8_t SAMPLING_FREQ = 100;
char message[90];


CY_ISR(MPU9250_DR_ISR) 
{   
    if (flag_calibration_todo == true){
        state = 0;
    }
    else{
        state = 1;  // to perform AHRS_filtering and Algorithm computation
    }
    Timer_2_ReadStatusRegister();
}


CY_ISR(Custom_ISR)
{      
    Timer_ReadStatusRegister();

    if ((count_10ms_time > SAMPLING_FREQ*TIME_CALIBRATION) && (flag_calibration_todo == true))
    {       
        // stop interrupt 
        count_10ms_time = 0;     
        MagCalibration_unbias(); // call magnetometer unbiasing method
        flag_calibration_todo = false;     
    }

    else
    {
        count_10ms_time++; //increment each 10 ms second
        count_10ms_time_thr++; //increment each 10 ms second
        count_10ms_time_2s++; //increment each 10 ms second
    }
}

CY_ISR(Wake_isr_interrupt)
{
    Pin_Wake_ClearInterrupt();
}

