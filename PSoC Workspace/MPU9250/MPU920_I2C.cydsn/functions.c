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

#include "math.h"
#include "stdio.h"
#include "project.h" 
#include "MPU9250.h"
#include "Fusion.h"
#include "FusionAhrs.h"
#include "FusionBias.h"
#include "functions.h"
#include "EEPROM/EEPROM.h"
#include "ssd1306.h"

#define TRANSMIT_BUFFER_SIZE 16
#define GYRO_SENSITIVITY 0.0076f    //s_gyro = 3.8*10^-3 with 250 DPS FS, 7.6*10^-3 with 500 DPS FS 
#define ACC_SENSITIVITY 0.000130f   //s_acc = 6.1*10^-5 with +-2g FS, 12.2*10^-5 with +-2g FS 
#define MAGN_RESOLUTION 1.5;        // 4900*2/65535 = 1.5 uT/LSB 
#define MAG_DECLINATION +3.17;      // Milan 
#define DATA_DIM 26 //26 
#define EARTH_MAGNETIC_FIELD_MAX 47 //uT in Milan
#define AK8963_MAGN_FIELD_MAX 4912 //from datasheet
#define GAIN_AHRS_FILTER 5.0f //5.0 works well, actually 0.5f is recommended.. but really slow 
#define WOMTHR 100
#define WOMFREQ 2
#define EEPROM_DIM 128
#define EEPROM_ADDRESS_RANGE 65535
 
// *******************Magnetometer variables **********************
uint8_t mag_bias_factory[3];
uint8_t raw_data_calib_magn[3];

static int16_t max_magn_calibration[3]= {-32767, -32767, -32767};
static int16_t min_magn_calibration[3] = {32767, 32767, 32767};
extern int16_t mag_bias[3];  // offset to be calibrated during init_magCalibration()
extern int16_t mag_scale[3]; // scale to be calibrated during init_magCalibration()
extern float magCalibration[3];
static uint8_t mag_uncalib[6];
extern uint8_t session[];
extern uint8_t session_dim;

// *******************Global variables **********************
extern int16 state;
extern bool send_data;
extern bool bt_mode;
static uint8_t i;
extern uint8_t count_MPU9250;
extern uint16_t count_10ms_time;
extern uint16_t count_10ms_time_thr;
extern uint16_t count_10ms_time_2s;
char message[90];   
int16_t roll;
int16_t pitch;
int16_t yaw;

extern uint16_t stride_total;
extern uint16_t step_counter;
extern int16_t dist_y_2s;
extern int16_t dist_x_2s;
extern uint16_t time_tot;



// *******************AHRS variables **********************
FusionBias fusionBias;
FusionAhrs fusionAhrs;
float samplePeriod = 0.01f; // replace this value with actual sample period in seconds

/*************** EEPROM VARIABLES ****************/
extern uint8_t idx_eeprom_data;
extern uint8_t eeprom_data[EEPROM_DIM];
extern uint16_t eeprom_last_address;
extern bool connected;

void Fusion_init(void)
{
    FusionBiasInitialise(&fusionBias, 0.1f, samplePeriod); // stationary threshold = 0.5 degrees per second

    // Initialise AHRS algorithm
    FusionAhrsInitialise(&fusionAhrs, GAIN_AHRS_FILTER); // gain = 0.5

    // Set optional magnetic field limits
    FusionAhrsSetMagneticField(&fusionAhrs, 20.0f, 70.0f); // valid magnetic field range = 20 uT to 70 uT   
}



void MagCalibration_acquisition(float32* magn)
{
        
    MPU9250_ReadMagRaw(&mag_uncalib[0]);   

    //CALIBRAZIONE
    magn[0] = ((int16_t)mag_uncalib[0] << 8) | mag_uncalib[1];  // Turn the MSB and LSB into a signed 16-bit value
    magn[1] = ((int16_t)mag_uncalib[2] << 8) | mag_uncalib[3];  // Data stored as little Endian
    magn[2] = ((int16_t)mag_uncalib[4] << 8) | mag_uncalib[5];
    
    for(i=0;i<3;i++)
    {
        if(magn[i]>max_magn_calibration[i]) max_magn_calibration[i] = magn[i];
        if(magn[i]<min_magn_calibration[i]) min_magn_calibration[i] = magn[i];
    }

}

void MagCalibration_unbias(void)
{
    static int16_t correct_value;
    for(i=0;i<3;i++)
    {   //remove the offset 
        mag_bias[i] = (max_magn_calibration[i]+min_magn_calibration[i])/2;
        //scale the value to +47uT (Milan) if necessary 
        correct_value = EARTH_MAGNETIC_FIELD_MAX * 32767/AK8963_MAGN_FIELD_MAX;
        max_magn_calibration[i] = max_magn_calibration[i] - mag_bias[i]; 
        mag_scale[i] = 100*correct_value/max_magn_calibration[i];
         //80 instead of 100 just to be sure that magn field is not amplified more than the 50uT because of a bad calibration
    }
    
    state = -1;
}


void AHRS_filtering(float32* acc, float32* gyro, float32* magn)
{
    
    // prepare packet
    static uint8_t data[DATA_DIM];
    // header
    data[0] = 0x24;
    // footer
    data[DATA_DIM-1] = 0x0A; 
    // read data
    
    MPU9250_ReadAccGyroRaw(&data[1]);       //fill data with 3*3*2bit = 12 bytes   
    MPU9250_ReadMagRaw(&data[13]);          //fill data with 3*2bit = 6 bytes
    MPU9250_ReadAccGyro(data, &acc[0], &gyro[0], &magn[0]); //fill acc, gyro, magn with bytes from data
    AHRS_filter(&data[19],acc,gyro,magn); //fill last data cells with Yaw, Pitch, Roll computed by AHRS 
    
    // data[DATA_DIM-2] = count_MPU9250;
    // send packet over uart
    if (send_data) 
        bt_mode == true? UART_Debug_PutArray(data,DATA_DIM) : UART_USB_PutArray(data,DATA_DIM);
}
void SaveSession(void){
    
    // alla fine della session salva: restante parte array + nsteps + tail
    // poi salva indirizzo ultima cella eeprom scritta 

    if (idx_eeprom_data<=124){   //if below limit of 128 byte page
        eeprom_data[idx_eeprom_data] =  (step_counter>>8) & 0xFF;    //n steps, MSB
        eeprom_data[idx_eeprom_data+1] = step_counter & 0xFF; //LSB
        eeprom_data[idx_eeprom_data+2] = 0xFF;  //tail
        eeprom_data[idx_eeprom_data+3] = 0xFF;
        idx_eeprom_data+=4;
    }
    else{
        eeprom_data[EEPROM_DIM-4] = (step_counter>>8) & 0xFF;     //n steps, MSB
        eeprom_data[EEPROM_DIM-3] = step_counter & 0xFF;    //LSB
        eeprom_data[EEPROM_DIM-2] = 0xFF;  //tail
        eeprom_data[EEPROM_DIM-1] = 0xFF;
        idx_eeprom_data=128;
    }
    
    
    //write 128 byte END session 
    EEPROM_Write(eeprom_last_address, eeprom_data ,idx_eeprom_data);
    CyDelay(100);
    //write eeprom last address into eeprom (keep track of last session position)
    eeprom_last_address += idx_eeprom_data;
    if (EEPROM_ADDRESS_RANGE-128 < eeprom_last_address) //check if eeprom_last_address is going to hit the upper size of eeprom
        eeprom_last_address=128;
    if (eeprom_last_address%EEPROM_DIM!=0){     //check if eeprom_last_address is multiple of 128
        eeprom_last_address += EEPROM_DIM - eeprom_last_address%EEPROM_DIM;
    }
    add_update(eeprom_last_address, EEPROM_Read123(2)+1);        // write eeprom_last_address to EEPROM cell 0x03-0x04 
  
    
    idx_eeprom_data = 0;
    step_counter=0;

    Connection_Led_Write(0);  
    if (!connected)
        MPU9250_StartWom(WOMTHR, WOMFREQ);
    
}

/* [] END OF FILE */