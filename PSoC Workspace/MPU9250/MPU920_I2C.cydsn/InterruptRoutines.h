#ifndef __INTERRUPT_ROUTINES_H 
    #define __INTERRUPT_ROUTINES_H
    #include "cytypes.h" 
    #include "stdio.h" 
    #include "MPU9250.h"
    #include "MPU9250_RegMap.h"
    #include "MPU9250_I2C.h"
    
 
    #define TRANSMIT_BUFFER_SIZE 16
    CY_ISR_PROTO (MPU9250_DR_ISR);
    CY_ISR_PROTO (Custom_ISR);
    CY_ISR_PROTO(Wake_isr_interrupt);
 
    char DataBuffer[TRANSMIT_BUFFER_SIZE]; 
    volatile uint8 PacketReadyFlag;
#endif
 