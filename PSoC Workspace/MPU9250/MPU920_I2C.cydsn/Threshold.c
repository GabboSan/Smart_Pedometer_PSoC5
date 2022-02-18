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

/* [] END OF FILE */

#include "math.h"
#include "stdio.h"
#include "project.h" 
#include "Threshold.h"
#include "EEPROM/EEPROM.h"
#include "stdbool.h"
#include <stdlib.h>
#include "ssd1306.h"
#define DISPLAY_ADDRESS 0x3D 
extern int16 state;

// ****************************************************************
// Zhao Algorithm: Thresolding the 3-axis accelerometer measurement 
//#define SAMPLING_FREQ 100 //Hz. Better to update this referring to the register 19 of MPU9250
#define TIME_WINDOW_MIN SAMPLING_FREQ*0.2   //0.2 sec
#define TIME_WINDOW_MAX SAMPLING_FREQ*2     //2.0 sec
#define ACC_PRECISION 0.000065f 
#define HEIGHT 180 
#define TIMEWOM 5
#define EEPROM_DIM 128
#define EEPROM_ADDRESS_RANGE 65535
#define DATA_PATH_DIM 15

//volatile uint8_t SAMPLING_FREQ = 50;
static int16_t sample_old[3] = {0,0,0};
static int16_t sample_new[3] = {0,0,0}; 
static int16_t min_thr = 32767;               //updates each 50 samples 
static int16_t max_thr = -32767;              //updates each 50 samples 
static int16_t dynamic_thr = 0;               //updates each 50 samples 
static uint8_t max_acc_index_old;             //index of the acc axis with max change value
static uint8_t max_acc_index_new; 
static uint8_t max_acc_index;
static uint8_t i; 

static uint8_t n_step_in_2s = 0; 
static uint8_t valid_step = 1; 
 uint16_t step_counter = 0; 
static float stride; 
uint16_t stride_total; 
static uint8_t current_speed; 
extern uint8_t SAMPLING_FREQ;

extern uint16_t count_10ms_time;
extern uint16_t count_10ms_time_thr;
extern uint16_t count_10ms_time_2s;
// ********************** step direction and detection variables *********************
extern int16_t roll_curr; // as the device will be held vertical, this will be the angle to be considered
extern int16_t pitch_curr;
extern int16_t yaw_curr;
float cos_dir_x;
float sin_dir_y;
int16_t dist_x_2s;
int16_t dist_y_2s;
uint16_t time_tot;

uint8_t Nostep=0;
uint8_t wom=0;
extern bool connected;
extern bool send_data;
extern bool bt_mode;

char data[DATA_PATH_DIM];

/************ EEPROM VARIABLES ************/
uint8_t eeprom_data[EEPROM_DIM];
uint8_t idx_eeprom_data = 0;
uint16_t eeprom_last_address = EEPROM_ADDRESS_RANGE;


// TABLE 1: Stride as a Function of Speed (steps per 2 s) and Height

/*
Steps per 2 s | Stride (m/s) 
0~2             Height/5
2~3             Height/4
3~4             Height/3
4~5             Height/2
5~6             Height/1.2 
6~8             Height
>=8             1.2 × Height
*/
// ****************************************************************
       


void Threshold_algo(float32* acc, uint16_t* count_10ms_time, uint16_t* count_10ms_time_thr, uint16_t* count_10ms_time_2s)
{
    // ****************************************************************
    // G: per evitare quei problemi di 100 Hz - 200 Hz che capitano occasionalmente, utilizzo counter dell ISR_10ms
    // Zhao Algorithm: Thresolding the 3-axis accelerometer measurement         

    if (acc[0] > acc[1] && acc[0] > acc[2]) max_acc_index = 0; 
    if (acc[1] > acc[0] && acc[1] > acc[2]) max_acc_index = 1; 
    if (acc[2] > acc[0] && acc[2] > acc[1]) max_acc_index = 2;
    
    for (i=0; i<3; i++)
        { 
        if (acc[i] > max_thr) 
        {   
            max_thr = acc[i];
        }
        
        if (acc[i] < min_thr) 
            {   
            min_thr = acc[i];
            }
        }
    //update threshold if n_samples = 50 or 100, or each second. 
    //if (*n_samples_thr%SAMPLING_FREQ == 0) //SAMPLING_FREQ = 100
    
    //if(*count_10ms_time_thr%100 == 0) //each 1 second -> update the threshold. This condition miss some 100 counter.. 
    if(*count_10ms_time_thr >= 100) //each 1 second -> update the threshold
        {dynamic_thr = (min_thr+max_thr)/2;

        *count_10ms_time_thr = 0;
        min_thr = 32767;                    //updates each SAMPLING_FREQ samples 
        max_thr = -32767;                   //updates each SAMPLING_FREQ samples
        }
    //replace new samples
    for (i=0; i<3; i++)
    { 
        sample_old[i] = sample_new[i];
        max_acc_index_old = max_acc_index_new;
        //if (abs(sample_new[i] - sample_old[i]) > ACC_PRECISION) //to remove high frequency oscillation 
        {
            sample_new[i] = acc[i]; 
            max_acc_index_new = max_acc_index;
        }
    }
    
    // set valid_step to 1 if time window 
    //if (*n_samples > SAMPLING_FREQ*0.2) //&& n_samples < SAMPLING_FREQ*2.0
    if (*count_10ms_time > 100*0.2)       // if more than 200 ms has passed from the last one -> step is valid 
        {
            valid_step = 1;
        }
    else valid_step = 0; 
        
    // detect step: as happening if there is a negative slope of the
    // acceleration plot (sample_new < sample_old) when the acceleration 
    // curve crosses below the dynamic threshold.
    if (dynamic_thr < sample_old[max_acc_index_old] && dynamic_thr > sample_new[max_acc_index_new]) //if it is in the middle
    { if (valid_step == 1) 
        { 
            step_counter++;
            n_step_in_2s++;
            //*n_samples = 0;        //reset counter
            *count_10ms_time = 0;
            valid_step = 0;
            cos_dir_x += cos(roll_curr); //cumulate the cosine of the Roll during the 2s 
            sin_dir_y += sin(roll_curr); //cumulate the sine of the Roll during the 2s
        } 
    }
    
    //compute the stride, according to the TABLE 1 
    if (*count_10ms_time_2s >= 100*2.0){ //2 seconds
        if (n_step_in_2s == 0){
            stride = 0;
            Nostep++;
        }
        else{
            if (n_step_in_2s > 0 && n_step_in_2s <= 2) stride = HEIGHT/5;  //cm
            if (n_step_in_2s > 2 && n_step_in_2s <= 3) stride = HEIGHT/4; 
            if (n_step_in_2s > 3 && n_step_in_2s <= 4) stride = HEIGHT/3; 
            if (n_step_in_2s > 4 && n_step_in_2s <= 5) stride = HEIGHT/2; 
            if (n_step_in_2s > 5 && n_step_in_2s <= 6) stride = HEIGHT/1.2; 
            if (n_step_in_2s > 6 && n_step_in_2s <= 8) stride = HEIGHT/1.0; 
            if (n_step_in_2s >= 8) stride = HEIGHT*1.2; 
            Nostep=0;
        }
        stride_total += stride;                    // cm 
        current_speed = stride/2.0; //from stride in 2 seconds -> cm/s
        *count_10ms_time_2s = 0; 
        n_step_in_2s = 0;
        
        if(Nostep==TIMEWOM && !connected){
            wom=1;  
        }
        //show numbers on display
        double appoggio=step_counter;
        uint8_t cifra;

        int ncifre=5;
        if(step_counter<10000)
        ncifre=4;
        if(step_counter<1000)
        ncifre=3;
        if(step_counter<100)
        ncifre=2;
        if(step_counter<10)
        ncifre=1;

        uint8_t number[ncifre];

        if(ncifre>1){
            for(int i=1; i<ncifre; i++){
                appoggio/=10.0;
            }
        }
        for(int i=0; i<ncifre; i++){
            cifra=appoggio;
            number[i]=cifra + '0';
            appoggio = (appoggio-cifra)*10.0;
        }
                   
        double appoggio1=current_speed;
        uint8_t cifra1;

        int ncifre1=5;
        if(step_counter<10000)
        ncifre1=4;
        if(step_counter<1000)
        ncifre1=3;
        if(current_speed<100)
        ncifre1=2;
        if(current_speed<10)
        ncifre1=1;

        uint8_t number1[ncifre];

        if(ncifre1>1){
            for(int i=1; i<ncifre1; i++){
                appoggio1/=10.0;
            }
        }
        for(int i=0; i<ncifre1; i++){
            cifra1=appoggio1;
            number1[i]=cifra1 + '0';
            appoggio1 = (appoggio1-cifra1)*10.0;
        }
            
                
        display_clear();    
        display_update();
        gfx_setTextSize(2);
        gfx_setTextColor(WHITE);
        gfx_setCursor(2,2);
        gfx_println("Step:");            
        for(int i=0; i<ncifre; i++){
            gfx_setCursor(75+i*13,2);
            gfx_write(number[i]);
        }
        gfx_setCursor(2,40);
        gfx_println("Speed:");
        for(int i=0; i<ncifre1; i++){
            gfx_setCursor(75+i*13,40);
            gfx_write(number1[i]);
        }
        display_update();  
        // keep track of the direction of the stride
        
        dist_x_2s = stride * cos_dir_x; // multiply the length of stride_vector by step_dir_x (in cm)
        dist_y_2s = stride * sin_dir_y; // multiply the length of stride_vector by step_dir_y (in cm)
        
        eeprom_data[idx_eeprom_data] = dist_x_2s & 0xFF;
        eeprom_data[idx_eeprom_data+1] = (dist_x_2s>>8) & 0xFF;
        eeprom_data[idx_eeprom_data+2] = dist_y_2s & 0xFF;
        eeprom_data[idx_eeprom_data+3] = (dist_y_2s>>8) & 0xFF;
        
        if (send_data){  // send new data if sending mode is active
            //char data[DATA_PATH_DIM];
            for(uint8_t i=0;i<DATA_PATH_DIM;i++)
                data[i]=' ';
            // header
            data[0] = 0x23;
            sprintf(&data[1],"%d %d",dist_x_2s,dist_y_2s);
            // footer
            data[DATA_PATH_DIM-1] = 0x0A;
            
            bt_mode == true? UART_Debug_PutArray(data,DATA_PATH_DIM) : UART_USB_PutArray(data,DATA_PATH_DIM);
        }
        
        idx_eeprom_data +=4;    // +4 -> 4 cells are written in previous steps 
        
        if(idx_eeprom_data==EEPROM_DIM){      //write partial session to eeprom w/ tail
            EEPROM_Write(eeprom_last_address, eeprom_data ,idx_eeprom_data);
            eeprom_last_address+=idx_eeprom_data;
            idx_eeprom_data=0;
        }

        //reset direction counter
        cos_dir_x = 0;
        sin_dir_y = 0;
        }   

}