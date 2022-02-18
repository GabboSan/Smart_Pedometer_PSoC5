/* ========================================
 *
 * Copyright YOUR COMPANY, THE YEAR
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF your company.
 *
 * ========================================
*/

#include "MPU9250.h"
#include "MPU9250_RegMap.h"
#include "MPU9250_I2C.h"
#include "math.h"
#include "UART_Debug.h"
#include "stdio.h"
#include "project.h"
#include <EEPROM/EEPROM.h>
#include "InterruptRoutines.h"

#define EEPROM_ADDRESS 0x50

 void EEPROM_Write(uint16_t reg_add, uint8_t* data, uint8_t dim){    
    int jump=0;
    uint8_t low_add = reg_add & 0xFF;
    uint8_t high_add = (reg_add>>8) & 0xFF;
    I2C_MPU9250_Master_MasterSendStart(EEPROM_ADDRESS,0);
    I2C_MPU9250_Master_MasterWriteByte(high_add);
    I2C_MPU9250_Master_MasterWriteByte(low_add);
    for(int i=0; i<dim; i++){
        if(reg_add+i==0||reg_add+i==1||reg_add+i==2||reg_add+i==3||reg_add+i==4){
            jump=i;
            break;}
        I2C_MPU9250_Master_MasterWriteByte(data[i]);
    }
    I2C_MPU9250_Master_MasterSendStop();
    if(jump!=0){
        uint8_t new_dim= dim-jump+1;
        uint8_t remain[new_dim];
        for(int k=0; k<new_dim; k++)
            remain[k]=data[k+jump];
        EEPROM_Write(5,remain,new_dim);
    }
}

void EEPROM_Write123(uint16_t reg_add, uint8_t* data, uint8_t dim){
    uint8_t low_add = reg_add & 0xFF;
    uint8_t high_add = (reg_add>>8) & 0xFF;
    I2C_MPU9250_Master_MasterSendStart(EEPROM_ADDRESS,0);
    I2C_MPU9250_Master_MasterWriteByte(high_add);
    I2C_MPU9250_Master_MasterWriteByte(low_add);
    for(int i=0; i<dim; i++){
        I2C_MPU9250_Master_MasterWriteByte(data[i]);
    }
    I2C_MPU9250_Master_MasterSendStop();
    
}


void EEPROM_Read(uint16_t reg_add, uint8_t *data, uint8_t dim){
    int jump=0;
    uint8_t low_add = reg_add & 0xFF;
    uint8_t high_add = (reg_add>>8) & 0xFF;
    I2C_MPU9250_Master_MasterSendStart(EEPROM_ADDRESS,0);
    I2C_MPU9250_Master_MasterWriteByte(high_add);
    I2C_MPU9250_Master_MasterWriteByte(low_add);
    I2C_MPU9250_Master_MasterSendRestart(EEPROM_ADDRESS,1);
    
    for(int i=0; i<dim; i++){
        if(reg_add+i==0||reg_add+i==1||reg_add+i==2||reg_add+i==3||reg_add+i==4){
            jump=i--;
            break;}
        if (i==dim-1)
        data[i] = I2C_MPU9250_Master_MasterReadByte(0); //last read need NOACK
        else
        data[i] = I2C_MPU9250_Master_MasterReadByte(1); //all the other need ACK
    }
    I2C_MPU9250_Master_MasterSendStop();
    if(jump!=0){
        uint8_t new_dim= dim-jump;
        uint8_t remain[new_dim];
        EEPROM_Read(5,remain,new_dim);
        //unisci vettori
        for(int i=0; i<dim; i++){
            data[dim-new_dim+1+i]=remain[i];
        }
    }   
}

void EEPROM_Read_exclude(uint16_t reg_add, uint8_t *data, uint8_t dim){
  
    uint8_t low_add = reg_add & 0xFF;
    uint8_t high_add = (reg_add>>8) & 0xFF;
    I2C_MPU9250_Master_MasterSendStart(EEPROM_ADDRESS,0);
    I2C_MPU9250_Master_MasterWriteByte(high_add);
    I2C_MPU9250_Master_MasterWriteByte(low_add);
    I2C_MPU9250_Master_MasterSendRestart(EEPROM_ADDRESS,1);
    
    for(int i=0; i<dim; i++){
        if(reg_add+i==0||reg_add+i==1||reg_add+i==2||reg_add+i==3||reg_add+i==4){
            continue;
        }
        if (i==dim-1)
            data[i] = I2C_MPU9250_Master_MasterReadByte(0); //last read need NOACK
        else
            data[i] = I2C_MPU9250_Master_MasterReadByte(1); //all the other need ACK
    }
    I2C_MPU9250_Master_MasterSendStop();   
}



uint8_t EEPROM_Read123(uint16_t reg_add){
    uint8_t low_add = reg_add & 0xFF;
    uint8_t high_add = (reg_add>>8) & 0xFF;
    I2C_MPU9250_Master_MasterSendStart(EEPROM_ADDRESS,0);
    I2C_MPU9250_Master_MasterWriteByte(high_add);
    I2C_MPU9250_Master_MasterWriteByte(low_add);
    I2C_MPU9250_Master_MasterSendRestart(EEPROM_ADDRESS,1);
    
    uint8_t data = I2C_MPU9250_Master_MasterReadByte(0); // NOACK

    I2C_MPU9250_Master_MasterSendStop();
    return data;
}

void add_update(uint16_t add, uint8_t session_n){
  
    uint8_t data[3];
    data[0]= session_n;
    data[1]= (add>>8) & 0xFF; //MSB
    data[2]= add & 0xFF;    //LSB
    EEPROM_Write123(2,data,3);

}
/* [] END OF FILE */
