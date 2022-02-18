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
#include "project.h"
#include "serial_worker.h"
#include "stdbool.h"
#include "EEPROM/EEPROM.h"
#include "functions.h"
#include "ssd1306.h"
#include "MPU9250.h"
#define EEPROM_DIM 512 // 2^16 / 128, need to know how many 128 byte package eeprom stores
#define EEPROM_BUFFER_DIM 128
#define WOMTHR 100
#define WOMFREQ 2

uint8_t return_message[] = {0x24,'\0',0x0A};
uint8_t package[5]; // header = 0x24 , tail = 0x0A
uint8_t profile_data[2];
uint8_t session_data[EEPROM_BUFFER_DIM];
uint8_t buffer_data[EEPROM_BUFFER_DIM];
int i=0;
extern int16 state;
extern bool bt_mode;
extern bool flag_calibration_todo;
extern uint16_t count_10ms_time;
extern uint16_t eeprom_last_address;
bool connected = false;

bool send_data = false; //true
bool data_incoming = false;

void serial_check_HT(void){ //check header and tail
    if (bt_mode && UART_Debug_GetRxBufferSize() > 0){
        data_incoming = true;
        package[i] =  UART_Debug_GetChar();
    }
    else if (UART_USB_GetRxBufferSize() > 0){
        data_incoming = true;
        package[i] =  UART_USB_GetChar();
    }
    
    if (data_incoming){
        if (i == 0 && package[0] != 0x24)
            i=0;
        else {
            if (i >= 2 && package[i] == 0x0A){
                serial_processing(package[1]);
                i=0;
            }
            else i++;
        }
        data_incoming = false;
    }
}

void serial_processing(char a){
    switch(a){
        case 'C':      // pc want to establish connection
            return_message[1] = 'A';
            connected = true;
            display_clear();display_update();
            gfx_setTextSize(1);gfx_setTextColor(WHITE);gfx_setCursor(2,2);
            gfx_println("PC Connection");
            display_update();    //NOTE4
            bt_mode == true? UART_Debug_PutArray(return_message,3) : UART_USB_PutArray(return_message,3);
            SaveSession();
            break;
            
        case 'c':      // pc want to close connection (ensure all activity with pc are closed)
            send_data = false;
            connected = false;
            MPU9250_StartWom(WOMTHR, WOMFREQ);
            break;
            
        case 'R':       // pc want real time data
            send_data = true;
            break;
        
        case 'r':       // pc close real time data flux
            send_data = false;
            break;
           
        case 'Z':       // pc ask for calibration 
            Connection_Led_Write(0);
            display_clear();display_update();
            gfx_setTextSize(1);gfx_setTextColor(WHITE);gfx_setCursor(2,2);
            gfx_println("CALIBRATION MODE");
            display_update();    //NOTE4
            flag_calibration_todo = true;
            count_10ms_time = 0;
            return_message[1] = 'A';
            bt_mode == true? UART_Debug_PutArray(return_message,3) : UART_USB_PutArray(return_message,3);
            break;
            
        case 'P':   //pc ask for current profile
            return_message[1] = EEPROM_Read123(1);  
            bt_mode == true? UART_Debug_PutArray(return_message,3) : UART_USB_PutArray(return_message,3);
            break;
            
        case 'p':   //pc set current profile 
            profile_data[0]=package[3]; profile_data[1]=package[2]; // package[2] is UserID and package[3] is user's height
            EEPROM_Write123(0,profile_data,2);
            break;
            
        case 'S': //send all stored session to PC 
            for (uint16_t idx=0; idx<EEPROM_DIM; idx++){
                EEPROM_Read_exclude(128*idx,buffer_data,EEPROM_BUFFER_DIM);
                if (idx==0)
                    bt_mode == true? UART_Debug_PutArray(buffer_data,EEPROM_BUFFER_DIM-5) : UART_USB_PutArray(buffer_data,EEPROM_BUFFER_DIM-5);
                else
                    bt_mode == true? UART_Debug_PutArray(buffer_data,EEPROM_BUFFER_DIM) : UART_USB_PutArray(buffer_data,EEPROM_BUFFER_DIM);
            }
            break;
            
    }
    
}


/* [] END OF FILE */
