/* ============================================
 *
 * @brief Main file for MPU9250 group1 project.
 * 
 * This is the main file to be used with the
 * MPU9250 group1 project. It sets up all the 
 * components required for the project.
 * 
 * @author group1
 * @year 2021-2022
 * ============================================
*/

// Include required header files
#include "project.h"
#include "MPU9250.h"
#include "MPU9250_RegMap.h"
#include "MPU9250_I2C.h"
#include "stdio.h"
#include "FusionCalibration.h"
#include "FusionCompass.h"
#include "FusionTypes.h"
#include "math.h"
#include "InterruptRoutines.h"
#include "functions.h"
#include "Threshold.h"
#include "stdbool.h"
#include "serial_worker.h"
#include "EEPROM/EEPROM.h"
//display
#include <stdlib.h>
#include "ssd1306.h"
#define DISPLAY_ADDRESS 0x3D 
#define WOMTHR 100
#define WOMFREQ 2
#define EEPROM_DIM 128

int16 state = -1; 
bool flag_calibration_todo = false;
bool bt_mode = false;

// *************** AHRS variables ****************
float32 acc[3], gyro[3], magn[3]; 
uint16_t n_samples = 0;
uint16_t n_samples_thr = 0;
uint16_t n_samples_2s = 0; 
extern uint16_t count_10ms_time;
extern uint16_t count_10ms_time_thr;
extern uint16_t count_10ms_time_2s;
extern uint16_t stride_total;
extern int16_t roll_curr;
extern int16_t pitch_curr;
extern int16_t yaw_curr;
MPU9250Setting setting;
extern uint8_t wom;
extern uint16_t eeprom_last_address;
extern uint8_t eeprom_data[EEPROM_DIM];
extern uint8_t idx_eeprom_data;

int main(void)
{   
            
    CyGlobalIntEnable; 
    
    // Start UART component
    UART_Debug_Start();
    UART_USB_Start();
    
    // Start I2C component
    I2C_MPU9250_Master_Start();
    CyDelay(1000);
    
    // *************** SETTING MPU9250 **************************
    
    setting.accel_fs_sel = A4G;
    setting.gyro_fs_sel = G500DPS;
    setting.mag_output_bits = M16BITS;
    setting.fifo_sample_rate = SMPL_100HZ;
    setting.gyro_fchoice = 0x03;
    setting.gyro_dlpf_cfg = DLPF_41HZ;
    setting.accel_fchoice = 0x01;
    setting.accel_dlpf_cfg = DLPFa_45HZ;

    //init MPU
    init_MPU();
    //init fusion
    Fusion_init();
    //start MPU ISR
    MPU9250_ISR_StartEx(MPU9250_DR_ISR);
    //timer ISR
    Timer_Start(); 
    Timer_Int_StartEx(Custom_ISR); 
    Timer_2_Start();
    //init display 
    display_init(DISPLAY_ADDRESS); 

    //reset all memory
    /*uint8_t zeros[128]={0};
    EEPROM_Write(5,zeros,123);
    CyDelay(100);
    for (uint16 i=1;i<512;i++){
        EEPROM_Write(i*128,zeros,128);
        CyDelay(100);
    }*/
    
    // get last written EEPROM address 
    uint8_t address[2];
    address[0]=EEPROM_Read123(3);   //MSB
    address[1]=EEPROM_Read123(4);   //LSB
    eeprom_last_address = (address[0] << 8) | (address[1] & 0xFF);
    

    MPU9250_StartWom(WOMTHR, WOMFREQ);
    
    /*========================================
    STATE -1: idle, possible operations:
        - read incoming serial messages
        - enable power saving features
    STATE 0: calibration
    STATE 1: data_processing
    ========================================*/
    
    for(;;)
    {
        switch(state)
        {
            case -1: //idle
                serial_check_HT();  //check for new serial messages
                if( wom==1){
                    SaveSession();
                    wom=0;
                }
                break;
            
            case 0:  // magnetometer calibration
                MagCalibration_acquisition(magn);
                state = -1;  // set idle mode
                break;
                
            case 1:  //data processing
                Connection_Led_Write(1);
                AHRS_filtering(acc, gyro, magn);
                Threshold_algo(acc, &count_10ms_time, &count_10ms_time_thr, &count_10ms_time_2s);
                state = -1;  // set idle mode
                break;
        }       
    }
}

/* [] END OF FILE */
