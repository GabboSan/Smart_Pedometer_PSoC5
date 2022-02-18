/* ========================================
 * This source file contains the functions
 * used to interface with the MPU9250.
 * 
 * Functions declarations are present in the
 * MPU9250.h header file.
 *
 * ========================================
*/



/* ========= Includes ========= */
#include "MPU9250.h"
#include "MPU9250_RegMap.h"
#include "MPU9250_I2C.h"
#include "math.h"
#include "UART_Debug.h"
#include "stdio.h"
#include "project.h"
#include "Fusion.h"
#include "FusionAhrs.h"
#include "FusionBias.h"
#include "FusionCalibration.h"
#include "FusionCompass.h"
#include "FusionTypes.h"
#include "ssd1306.h"
#include "InterruptRoutines.h"
#include "EEPROM/EEPROM.h"

/* ========= MACROS ========= */
#ifndef MPU9250_ACC_FS_MASK 
    #define MPU9250_ACC_FS_MASK 0x18 // This mask is used for acc full scale bits
#endif

#ifndef MPU9250_GYRO_FS_MASK 
    #define MPU9250_GYRO_FS_MASK 0x18 // This mask is used for gyro full scale bits
#endif

#ifndef MPU9250_SLEEP_MASK 
    #define MPU9250_SLEEP_MASK 0x40
#endif

#ifndef MPU9250_G
    #define MPU9250_G 9.807f
#endif

#define EEPROM_DIM 128

/* ========= VARIABLES ========= */
float acc_scale = 0;    // Accelerometer scaling factor
float gyro_scale = 0;   // Gyroscope scaling factor

volatile int16_t mag_bias[3] = {0.0, 0.0, 0.0};  // offset to be calibrated during init_magCalibration()
volatile int16_t mag_scale[3] = {100,100,100}; // scale to be calibrated during init_magCalibration()
extern int16_t roll;
extern int16_t pitch;
extern int16_t yaw;
float magCalibration[3] = {1.0, 1.0, 1.0};

 
#define MAGN_RESOLUTION 1.5  // 4900*2/65535 = 1.5 uT/LSB 
extern uint8_t raw_data_calib_magn[3];  //values from AK8963 datasheet
extern uint8_t mag_bias_factory[3];     // sensitivity adjustment is computed by the raw_data_calib_magn;
extern MPU9250Setting setting;
//Magnetometer calibration: replace these values with the one observed during testing
    // put the magn in direction of the N (50uT): record the max value in bit and 
    // 9800uT/(2^16)*(MAGN_SENSITIVITY*mag_bias_factory*MAG_X_HYPER_ADJ) = 50uT
#define MAG_X_HYPER_ADJ 1.00
#define MAG_Y_HYPER_ADJ  1.00
#define MAG_Z_HYPER_ADJ  1.00


#define GYRO_SENSITIVITY 0.0038f  //s_gyro = 3.8*10^-3 with 250 DPS FS 
#define ACC_SENSITIVITY 0.000065f   //s_acc = 6.1*10^-5 with +-2g FS 
#define DEG_TO_RAD 0.01745f
#define RAD_TO_DEG 57.296f

float acc_resolution = {0.f};                // scale resolutions per LSB for the sensors
float gyro_resolution = {0.f};               // scale resolutions per LSB for the sensors
float mag_resolution = {0.f};                // scale resolutions per LSB for the sensors
int16_t roll_curr;
int16_t pitch_curr;
int16_t yaw_curr;


// G: DA METTERE DOPO AVER RIEMPITO I 3 VETTORI RESOLUTION!!!
FusionVector3 gyroscopeSensitivity = {
    .axis.x = GYRO_SENSITIVITY, 
    .axis.y = GYRO_SENSITIVITY,
    .axis.z = GYRO_SENSITIVITY,
}; // replace these values with actual sensitivity in degrees per second per lsb as specified in gyroscope datasheet

FusionVector3 accelerometerSensitivity = {
    .axis.x = ACC_SENSITIVITY, 
    .axis.y = ACC_SENSITIVITY,
    .axis.z = ACC_SENSITIVITY,
}; // replace these values with actual sensitivity in g per lsb as specified in accelerometer datasheet

FusionVector3 hardIronBias = {
    .axis.x = 0.0f,
    .axis.y = 0.0f,
    .axis.z = 0.0f,
}; // replace these values with actual hard-iron bias in uT if known

extern FusionBias fusionBias;
extern FusionAhrs fusionAhrs;
extern float samplePeriod;
extern uint16_t eeprom_last_address;
extern uint8_t eeprom_data[EEPROM_DIM];
extern uint8_t idx_eeprom_data;

void MPU9250_StartWom(uint8_t threshold, uint8_t wakeFrequency){

    display_clear();    
    display_update();
    /*
    frequency of wke unp depending on the following table
    Lposc   Output Frequency (Hz)
    0        0.24
    1        0.49
    2        0.98
    3        1.95
    4        3.91
    5        7.81
    6        15.63
    7        31.25
    8        62.50
    9        125
    10       250
    11       500
    12-15  RESERVED
    */
    if (wakeFrequency>11)
        wakeFrequency=11;

    //save registers status before wom
    uint8_t pwr1 = MPU9250_I2C_Read(MPU9250_I2C_ADDRESS, MPU9250_PWR_MGMT_1_REG);
    uint8_t pwr2 = MPU9250_I2C_Read(MPU9250_I2C_ADDRESS, MPU9250_PWR_MGMT_2_REG);
    uint8_t acc2 = MPU9250_I2C_Read(MPU9250_I2C_ADDRESS, MPU9250_ACCEL_CONFIG_2_REG);
    uint8_t detect = MPU9250_I2C_Read(MPU9250_I2C_ADDRESS,MPU9250_MOT_DETECT_REG);
    uint8_t inten = MPU9250_I2C_Read(MPU9250_I2C_ADDRESS, MPU9250_INT_ENABLE_REG);

    //activate wom
    MPU9250_I2C_Write(MPU9250_I2C_ADDRESS, MPU9250_PWR_MGMT_1_REG, pwr1 & 0x8F); //1000 1111 In PWR_MGMT_1 (0x6B) make CYCLE =0, SLEEP = 0 and STANDBY = 0
    MPU9250_I2C_Write(MPU9250_I2C_ADDRESS, MPU9250_PWR_MGMT_2_REG, pwr2 & 0xC7); //1100 0111 In PWR_MGMT_2 (0x6C) set DIS_XA, DIS_YA, DIS_ZA = 0
    MPU9250_I2C_Write(MPU9250_I2C_ADDRESS, MPU9250_PWR_MGMT_2_REG, MPU9250_I2C_Read(MPU9250_I2C_ADDRESS, MPU9250_PWR_MGMT_2_REG) | 0x07); //0000 0111 and DIS_XG, DIS_YG, DIS_ZG = 1
    MPU9250_I2C_Write(MPU9250_I2C_ADDRESS, MPU9250_ACCEL_CONFIG_2_REG, acc2 & 0xF9); //1111 1001
    MPU9250_I2C_Write(MPU9250_I2C_ADDRESS, MPU9250_ACCEL_CONFIG_2_REG, MPU9250_I2C_Read(MPU9250_I2C_ADDRESS, MPU9250_ACCEL_CONFIG_2_REG) | 0x09); //0000 1001 ACCEL_CONFIG 2  (0x1D) set ACCEL_FCHOICE_B = 1 and A_DLPFCFG[2:0]=1(b001)
    MPU9250_I2C_Write(MPU9250_I2C_ADDRESS, MPU9250_INT_ENABLE_REG, 0x40); //INT_ENABLE (0x38), set the whole register to 0x40 to enable motion interrupt only
    MPU9250_I2C_Write(MPU9250_I2C_ADDRESS,MPU9250_MOT_DETECT_REG, detect | 0xC0); //1100 0000 MOT_DETECT_CTRL (0x69), set ACCEL_INTEL_EN = 1 and ACCEL_INTEL_MODE = 1
    MPU9250_I2C_Write(MPU9250_I2C_ADDRESS, MPU9250_WOM_THR_REG, threshold); //n WOM_THR (0x1F), set the WOM_Threshold[7:0] to 1~255 LSBs (0~1020mg) This register holds the threshold value for the Wake on Motion Interrupt for 
    //accel x/y/z axes. LSB = 4mg. Range is 0mg to 1020mg
    MPU9250_I2C_Write(MPU9250_I2C_ADDRESS,MPU9250_LP_ACCEL_ODR_REG, MPU9250_I2C_Read(MPU9250_I2C_ADDRESS,MPU9250_LP_ACCEL_ODR_REG) & (wakeFrequency | 0xF0)); //in LP_ACCEL_ODR (0x1E), set Lposc_clksel[3:0] = 0.24Hz ~ 500Hz prendo 4LSB di wakefrequency con AND salvo gli zeri
    MPU9250_I2C_Write(MPU9250_I2C_ADDRESS,MPU9250_LP_ACCEL_ODR_REG, MPU9250_I2C_Read(MPU9250_I2C_ADDRESS,MPU9250_LP_ACCEL_ODR_REG) | (wakeFrequency & 0x0F) ); //prendo 4LSB di wakefrequency con OR salvo gli uno
    MPU9250_I2C_Write(MPU9250_I2C_ADDRESS, MPU9250_PWR_MGMT_1_REG,  MPU9250_I2C_Read(MPU9250_I2C_ADDRESS, MPU9250_PWR_MGMT_1_REG) | 0x20); //0010 0000 In PWR_MGMT_1 (0x6B) make CYCLE =1

    //wom activated, now AltActive mode Psoc
    MPU9250_ISR_Stop();
    Timer_Int_Stop(); 
    
    Wake_isr_StartEx(Wake_isr_interrupt);
    //Pin_Wake_ClearInterrupt(); 

    CyPmSaveClocks();
    CyPmAltAct(PM_SLEEP_TIME_NONE,PM_SLEEP_SRC_PICU);
    CyPmRestoreClocks();
    
    Wake_isr_Stop();

    MPU9250_ISR_StartEx(MPU9250_DR_ISR);
    Timer_Int_StartEx(Custom_ISR); 


    //exiting from wom, restore registers
     MPU9250_I2C_Write(MPU9250_I2C_ADDRESS, MPU9250_PWR_MGMT_1_REG, pwr1 );
     MPU9250_I2C_Write(MPU9250_I2C_ADDRESS, MPU9250_PWR_MGMT_2_REG, pwr2 );
     MPU9250_I2C_Write(MPU9250_I2C_ADDRESS, MPU9250_ACCEL_CONFIG_2_REG, acc2 );
     MPU9250_I2C_Write(MPU9250_I2C_ADDRESS,MPU9250_MOT_DETECT_REG, detect );
     MPU9250_I2C_Write(MPU9250_I2C_ADDRESS, MPU9250_INT_ENABLE_REG, inten);

    MPU9250_ClearInterruptAny();
    
    //write header package into EEPROM
    uint8_t read_dato[2];
    read_dato[0]= EEPROM_Read123(1); //user id 
    read_dato[1]= EEPROM_Read123(2);   // session number 
    eeprom_data[0]=0xff;
    eeprom_data[1]=0xfe; 
    eeprom_data[2]=read_dato[1]+1; // increment session n. by 1
    eeprom_data[3]=read_dato[0];
    idx_eeprom_data+=4;
}


void init_MPU(void){
    uint8_t connection = 0; // Variable to store connection status
    
    // Scan I2C bus and find devices
    for (int address = 0; address < 128; address++) {
        if (I2C_MPU9250_Master_MasterSendStart(address, 0) == I2C_MPU9250_Master_MSTR_NO_ERROR) {
        }
        I2C_MPU9250_Master_MasterSendStop();
    }
    
    // Wait until MPU9250 is connected
    do {
        connection = MPU9250_IsConnected();
    } while (connection == 0);
    
    // Show connection status feedback
    Connection_Led_Write(1);
    
    // Start the MPU9250
    MPU9250_Start();
}


void MPU9250_Start(void) {
    // This function starts the MPU9250.
    
    // Check if the I2C component has already been started,
    // otherwise start it.
    if (!I2C_MPU9250_Master_initVar) {
        I2C_MPU9250_Master_Start();
        CyDelay(10);
    }
    
    // Wake up MPU9250
    MPU9250_WakeUp();
    
    acc_resolution = get_acc_resolution(setting.accel_fs_sel);
    gyro_resolution = get_gyro_resolution(setting.gyro_fs_sel);
    mag_resolution = get_mag_resolution(setting.mag_output_bits);
    
    //update the AHRS structures
    accelerometerSensitivity.axis.x = acc_resolution;
    accelerometerSensitivity.axis.y = acc_resolution;
    accelerometerSensitivity.axis.z = acc_resolution;
    gyroscopeSensitivity.axis.x = gyro_resolution;
    gyroscopeSensitivity.axis.y = gyro_resolution;
    gyroscopeSensitivity.axis.z = gyro_resolution;
    
    // reset device
    MPU9250_I2C_Write(MPU9250_I2C_ADDRESS, MPU9250_PWR_MGMT_1_REG, 0x80);  // Write a one to bit 7 reset bit; toggle reset device
    CyDelay(100);

    // wake up device
    MPU9250_I2C_Write(MPU9250_I2C_ADDRESS, MPU9250_PWR_MGMT_1_REG, 0x00);  // Clear sleep mode bit (6), enable all sensors
    CyDelay(100);                                  // Wait for all registers to reset

    // get stable time source
    MPU9250_I2C_Write(MPU9250_I2C_ADDRESS, MPU9250_PWR_MGMT_1_REG, 0x01);  // Auto select clock source to be PLL gyroscope reference if ready else
    CyDelay(200);
    
    // Configure Gyro and Thermometer
    // Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz, respectively;
    // minimum delay time for this setting is 5.9 ms, which means sensor fusion update rates cannot
    // be higher than 1 / 0.0059 = 170 Hz
    // GYRO_DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
    // With the MPU9250, it is possible to get gyro sample rates of 32 kHz (!), 8 kHz, or 1 kHz
    uint8_t mpu_config = (uint8_t)setting.gyro_dlpf_cfg;
    MPU9250_I2C_Write(MPU9250_I2C_ADDRESS, MPU9250_CONFIG_REG, mpu_config);

    // Set sample rate 
    uint8_t sample_rate = (uint8_t)setting.fifo_sample_rate;
    MPU9250_I2C_Write(MPU9250_I2C_ADDRESS, MPU9250_SMPLRT_DIV_REG, sample_rate);  // Use a 200 Hz rate; a rate consistent with the filter update rate
                                                                                // determined inset in CONFIG above

    // Set gyroscope full scale range
    // Range selects FS_SEL and GFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
    uint8_t c = MPU9250_I2C_Read(MPU9250_I2C_ADDRESS, MPU9250_GYRO_CONFIG_REG);  // get current MPU9250_GYRO_CONFIG_REG register value
    c = c & ~0xE0;                                     // Clear self-test bits [7:5]
    c = c & ~0x03;                                     // Clear Fchoice bits [1:0]
    c = c & ~0x18;                                     // Clear GYRO_FS_SEL bits [4:3]
    c = c | ((uint8_t)(setting.gyro_fs_sel) << 3);       // Set full scale range for the gyro
    c = c | ((uint8_t)(~setting.gyro_fchoice) & 0x03);   // Set Fchoice for the gyro
    MPU9250_I2C_Write(MPU9250_I2C_ADDRESS, MPU9250_GYRO_CONFIG_REG, c);          // Write new MPU9250_GYRO_CONFIG_REG value to register

    // Set accelerometer full-scale range configuration
    c = MPU9250_I2C_Read(MPU9250_I2C_ADDRESS, MPU9250_ACCEL_CONFIG_REG);     // get current MPU9250_ACCEL_CONFIG_REG register value
    c = c & ~0xE0;                                 // Clear self-test bits [7:5]
    c = c & ~0x18;                                 // Clear ACCEL_FS_SEL bits [4:3]
    c = c | ((uint8_t)(setting.accel_fs_sel) << 3);  // Set full scale range for the accelerometer
    MPU9250_I2C_Write(MPU9250_I2C_ADDRESS, MPU9250_ACCEL_CONFIG_REG, c);     // Write new MPU9250_ACCEL_CONFIG_REG register value

    // Set accelerometer sample rate configuration
    // It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
    // accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
    c = MPU9250_I2C_Read(MPU9250_I2C_ADDRESS, MPU9250_ACCEL_CONFIG_2_REG);        // get current MPU9250_ACCEL_CONFIG_2_REG register value
    c = c & ~0x0F;                                     // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
    c = c | (~(setting.accel_fchoice << 3) & 0x08);    // Set accel_fchoice_b to 1
    c = c | ((uint8_t)(setting.accel_dlpf_cfg) & 0x07);  // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
    MPU9250_I2C_Write(MPU9250_I2C_ADDRESS, MPU9250_ACCEL_CONFIG_2_REG, c);        // Write new MPU9250_ACCEL_CONFIG_2_REG register value

    // The accelerometer, gyro, and thermometer are set to 1 kHz sample rates,
    // but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting

    // Configure Interrupts and Bypass Enable
    // Set interrupt pin active high, push-pull, hold interrupt pin level HIGH until interrupt cleared,
    // clear on read of INT_STATUS, and enable I2C_BYPASS_EN so additional chips
    // can join the I2C bus and all can be controlled by the PSOC as master
    MPU9250_I2C_Write(MPU9250_I2C_ADDRESS, MPU9250_INT_PIN_CFG_REG, 0x22); //func1: set it to 0010 0010. If 0110 0010 -> set 0x62
    MPU9250_I2C_Write(MPU9250_I2C_ADDRESS, MPU9250_INT_ENABLE_REG, 0x00);  //func2: set it to 0000 0001. Enable data ready (bit 0) interrupt
    CyDelay(100);
    
     //G: Set up magnetometer (enable it)
    MPU9250_MAG_Enable(); 
    
}

void MPU9250_Sleep(void) {
    // This function sleeps the MPU9250 by entering sleep mode.
    
    // Set sleep bit in power management 1 register.
    MPU9250_I2C_Write(MPU9250_I2C_ADDRESS, MPU9250_PWR_MGMT_1_REG,  
        ( MPU9250_I2C_Read(MPU9250_I2C_ADDRESS, MPU9250_PWR_MGMT_1_REG) | MPU9250_SLEEP_MASK));
}

void MPU9250_WakeUp(void) {
    // This function wakes up the MPU9250 exiting sleep mode.
    
    // Clear sleep bit in power management 1 register.
    MPU9250_I2C_Write(MPU9250_I2C_ADDRESS, MPU9250_PWR_MGMT_1_REG,  
        ( MPU9250_I2C_Read(MPU9250_I2C_ADDRESS, MPU9250_PWR_MGMT_1_REG) & ~MPU9250_SLEEP_MASK));
}

uint8_t MPU9250_IsConnected(void) {
    // Checks if the MPU9250 is present on the I2C bus
    uint8_t err = I2C_MPU9250_Master_MasterSendStart(MPU9250_I2C_ADDRESS, 0);
    I2C_MPU9250_Master_MasterSendStop();
    if (err > 0)
        return 0;
    // Then also check if the value contained in the who am i register is the expected one
    return MPU9250_I2C_Read(MPU9250_I2C_ADDRESS, MPU9250_WHO_AM_I_REG) == MPU9250_WHO_AM_I;
}

uint8_t MPU9250_ReadWhoAmI(void) {
    // Reads the who am i register and return the value
    return MPU9250_I2C_Read(MPU9250_I2C_ADDRESS, MPU9250_WHO_AM_I_REG);   
}

uint8_t MPU9250_ReadMagWhoAmI(void) {
    // Reads the who am i register of the magnetometer
    return MPU9250_I2C_Read(AK8963_I2C_ADDRESS, 0x00);
}

void MPU9250_ReadAcc(int16_t* acc) {
    // We can read 6 consecutive bytes since the accelerometer registers are in order
    
    uint8_t temp[6];  // Temp variable to store the data
    
    // Read data via I2C
    MPU9250_I2C_ReadMulti(MPU9250_I2C_ADDRESS, MPU9250_ACCEL_XOUT_H_REG, temp, 6);
    acc[0] = (temp[0] << 8) | (temp[1] & 0xFF);
    acc[1] = (temp[2] << 8) | (temp[3] & 0xFF);
    acc[2] = (temp[4] << 8) | (temp[5] & 0xFF);
}

void MPU9250_ReadAccRaw(uint8_t* acc) {
    // We can read 6 consecutive bytes since the accelerometer registers are in order
    
    // Read data via I2C
    MPU9250_I2C_ReadMulti(MPU9250_I2C_ADDRESS, MPU9250_ACCEL_XOUT_H_REG, acc, 6);
}

void MPU9250_ReadGyro(int16_t* gyro) {
    // We can read 6 consecutive bytes since the gyroscope registers are in order
    
    uint8_t temp[6];  // Temp variable to store the data
    
    // Read data via I2C
    MPU9250_I2C_ReadMulti(MPU9250_I2C_ADDRESS, MPU9250_GYRO_XOUT_H_REG, temp, 6);
    gyro[0] = (temp[0] << 8) | (temp[1] & 0xFF);
    gyro[1] = (temp[2] << 8) | (temp[3] & 0xFF);
    gyro[2] = (temp[4] << 8) | (temp[5] & 0xFF);
}

void MPU9250_ReadGyroRaw(uint8_t* gyro) {
    // We can read 6 consecutive bytes since the gyroscope registers are in order
    
    // Read data via I2C
    MPU9250_I2C_ReadMulti(MPU9250_I2C_ADDRESS, MPU9250_GYRO_XOUT_H_REG, gyro, 6);
}

void MPU9250_ReadAccGyro(uint8_t* temp, float32* acc,float32* gyro,float32* magn) {
    // put 'temp' bytes into the 3 vectors

    
    //adjust axis of accelerometer and gyroscope to the magnetometer ones 
    //mag_16bit[2] = -mag_16bit[2]; //-z
    //temp_magn = mag_16bit[0];
    //mag_16bit[0] = mag_16bit[1];
    //mag_16bit[1] = temp_magn; //invert 'x' and 'y' axis
    
    *acc++  = (int16_t)(temp[1] << 8) | (temp[2] & 0xFF);
    *acc++  = (int16_t)(temp[3] << 8) | (temp[4] & 0xFF);
    *acc  = (int16_t)(temp[5] << 8) | (temp[6] & 0xFF);
    *gyro++ = (int16_t)(temp[7] << 8) | (temp[8] & 0xFF);    //before: *gyro++ = (temp[8] << 8) | (temp[9] & 0xFF);
    *gyro++ = (int16_t)(temp[9] << 8) | (temp[10] & 0xFF);
    *gyro = (int16_t)(temp[11] << 8) | (temp[12] & 0xFF);
    *magn++ = (int16_t)(temp[13] << 8) | (temp[14] & 0xFF);
    *magn++ = (int16_t)(temp[15] << 8) | (temp[16] & 0xFF);
    *magn = (int16_t)(temp[17] << 8) | (temp[18] & 0xFF);
    
}

void MPU9250_ReadAccGyroRaw(uint8_t* data) {
    uint8_t end_trasmission;
    // Read data via I2C
    MPU9250_I2C_ReadMulti(MPU9250_I2C_ADDRESS, MPU9250_ACCEL_XOUT_H_REG, data, 6);
    MPU9250_I2C_ReadMulti(MPU9250_I2C_ADDRESS, MPU9250_GYRO_XOUT_H_REG, data + 6, 6);
    MPU9250_I2C_ReadMulti(AK8963_I2C_ADDRESS, MPU9250_MAG_XOUT_H_REG, data + 12, 6);
    end_trasmission = MPU9250_I2C_Read(AK8963_I2C_ADDRESS, 0x09); //ST2 register (role as data reading end register)
}

void AHRS_filter(uint8_t* data, float32* acc,float32* gyro,float32* magn) {
    
    // Madgwick function needs to be fed North, East, and Down direction like
    // (AN, AE, AD, GN, GE, GD, MN, ME, MD)
    // Accel and Gyro direction is Right-Hand, X-Forward, Z-Up
    // Magneto direction is Right-Hand, Y-Forward, Z-Down
    // So to adopt to the general Aircraft coordinate system (Right-Hand, X-Forward, Z-Down),
    // we need to feed (ax, -ay, -az, gx, -gy, -gz, my, -mx, mz)
    // but we pass (-ax, ay, az, gx, -gy, -gz, my, -mx, mz)
    // because gravity is by convention positive down, we need to ivnert the accel data

    // get quaternion based on aircraft coordinate (Right-Hand, X-Forward, Z-Down)
    // acc[mg], gyro[deg/s], mag [mG]
    
    /*
     * @brief Initialises the gyroscope bias correction algorithm.
     * @param fusionBias FusionBias structure.
     * @param threshold Gyroscope threshold (in degrees per second) below which the
     * gyroscope is detected stationary.
     * @param samplePeriod Nominal sample period (in seconds) corresponding the rate
     * at which the application will update the algorithm.
     * 
     */
    
    acc[0]  = -acc[0];
    acc[1]  = +acc[1];
    acc[2]  = +acc[2];
    gyro[0] = +gyro[0];  
    gyro[1] = -gyro[1]; 
    gyro[2] = -gyro[2]; 
    magn[1] = +magn[1];
    magn[0] = -magn[0];
    magn[2] = +magn[2]; 
    
    // Calibrate gyroscope
    FusionVector3 uncalibratedGyroscope = {
        .axis.x = gyro[0], // replace this value with actual gyroscope x axis measurement in lsb //
        .axis.y = gyro[1], // replace this value with actual gyroscope y axis measurement in lsb //
        .axis.z = gyro[2], // replace this value with actual gyroscope z axis measurement in lsb //
    };
    FusionVector3 calibratedGyroscope = FusionCalibrationInertial(uncalibratedGyroscope, FUSION_ROTATION_MATRIX_IDENTITY, gyroscopeSensitivity, FUSION_VECTOR3_ZERO);

    // Calibrate accelerometer
    FusionVector3 uncalibratedAccelerometer = {
        .axis.x = acc[0], // replace this value with actual accelerometer x axis measurement in lsb //
        .axis.y = acc[1], // replace this value with actual accelerometer y axis measurement in lsb //
        .axis.z = acc[2], // replace this value with actual accelerometer z axis measurement in lsb //
    };
    FusionVector3 calibratedAccelerometer = FusionCalibrationInertial(uncalibratedAccelerometer, FUSION_ROTATION_MATRIX_IDENTITY, accelerometerSensitivity, FUSION_VECTOR3_ZERO);

    // Calibrate magnetometer
    FusionVector3 uncalibratedMagnetometer = {
        .axis.x = magn[0], // replace this value with actual magnetometer x axis measurement in uT //
        .axis.y = magn[1], // replace this value with actual magnetometer y axis measurement in uT //
        .axis.z = magn[2], // replace this value with actual magnetometer z axis measurement in uT //
    };
    FusionVector3 calibratedMagnetometer = FusionCalibrationMagnetic(uncalibratedMagnetometer, FUSION_ROTATION_MATRIX_IDENTITY, hardIronBias);

    // Update gyroscope bias correction algorithm
    calibratedGyroscope = FusionBiasUpdate(&fusionBias, calibratedGyroscope);

    // Update AHRS algorithm
    FusionAhrsUpdate(&fusionAhrs, calibratedGyroscope, calibratedAccelerometer, calibratedMagnetometer, samplePeriod);

    // Print Euler angles
    FusionEulerAngles eulerAngles = FusionQuaternionToEulerAngles(FusionAhrsGetQuaternion(&fusionAhrs));    
   
    
    *data++ = (int)eulerAngles.angle.roll >> 8; 
    *data++ = (int)eulerAngles.angle.roll & 0xFF;
    *data++ = (int)eulerAngles.angle.pitch >> 8;
    *data++ = (int)eulerAngles.angle.pitch & 0xFF;
    *data++ = (int)eulerAngles.angle.yaw >> 8;
    *data++ = (int)eulerAngles.angle.yaw & 0xFF;
    
    roll_curr = (eulerAngles.angle.roll & 0xFF) | (eulerAngles.angle.roll >> 8);
    pitch_curr = (eulerAngles.angle.pitch & 0xFF) | (eulerAngles.angle.pitch >> 8);
    yaw_curr = (eulerAngles.angle.yaw & 0xFF) | (eulerAngles.angle.yaw >> 8);
}

void MPU9250_ReadMag(int16_t* mag_uT) {
    
   uint8_t temp[6];
    static int16_t mag_16bit[3];
    // Get RAW data
    MPU9250_ReadMagRaw(temp); //temp contains mag_raw in two complements format, as MSB[0]-LSB[1].
    
    
    mag_16bit[0] = ((int16_t)temp[0] << 8) | temp[1];  // Turn the MSB and LSB into a signed 16-bit value
    mag_16bit[1] = ((int16_t)temp[2] << 8) | temp[3];  // Data stored as little Endian
    mag_16bit[2] = ((int16_t)temp[4] << 8) | temp[5];
    
    //correct each with the mag_bias factory alogn x,y,z to get value in uT!
                                //mag[0] = ((float)(mag[0] * MAGN_RESOLUTION * mag_bias_factory[0] * MAG_X_HYPER_ADJ - mag_bias[0]) * mag_scale[0]);  // get actual magnetometer value, this depends on scale being set
                                //mag[1] = ((float)(mag[1] * MAGN_RESOLUTION * mag_bias_factory[1] * MAG_Y_HYPER_ADJ - mag_bias[1]) * mag_scale[1]);
                                //mag[2] = ((float)(mag[2] * MAGN_RESOLUTION * mag_bias_factory[2] * MAG_Z_HYPER_ADJ - mag_bias[2]) * mag_scale[2]);
    mag_uT[0] = ((float)(mag_16bit[0] * MAGN_RESOLUTION * mag_bias_factory[0]));  // get actual magnetometer value, this depends on scale being set
    mag_uT[1] = ((float)(mag_16bit[1] * MAGN_RESOLUTION * mag_bias_factory[1]));
    mag_uT[2] = ((float)(mag_16bit[2] * MAGN_RESOLUTION * mag_bias_factory[2]));
    
    /*
    uint8_t temp[6];
    // Get RAW data
   MPU9250_ReadMagRaw(temp);
    
    mag[0] = (temp[0] << 8) | (temp[1] & 0xFF);
    mag[1] = (temp[2] << 8) | (temp[3] & 0xFF);
    mag[2] = (temp[4] << 8) | (temp[5] & 0xFF);
    */
}

void MPU9250_ReadMagRaw(uint8_t* mag_raw) {
    uint8_t end_trasmission;
    // Read data via I2C
    uint8_t temp[6];
    static int16_t mag_16bit[3];
    // Measurement data is stored in two’s complement and Little Endian format. 
         
    // Measurement range of each axis is -8190 ~ +8190 in decimal in 14-bit output, and -32760 ~ 32760 in 16-bit output.
    MPU9250_I2C_ReadMulti(AK8963_I2C_ADDRESS, MPU9250_MAG_XOUT_H_REG, temp, 6); //LSB[1],MSB[0]!!!
    end_trasmission = MPU9250_I2C_Read(AK8963_I2C_ADDRESS, 0x09); //ST2 register (role as data reading end register)
    
    
    //ricontrollare
    mag_16bit[0] = ((int16_t)temp[1] << 8) | temp[0];  // Turn the MSB and LSB into a signed 16-bit value
    mag_16bit[1] = ((int16_t)temp[3] << 8) | temp[2];  // Data stored as little Endian
    mag_16bit[2] = ((int16_t)temp[5] << 8) | temp[4];
    
    //calibrate mag values
    mag_16bit[0] = (mag_16bit[0] - mag_bias[0])* mag_scale[0]/100;
    mag_16bit[1] = (mag_16bit[1] - mag_bias[1])* mag_scale[1]/100;
    mag_16bit[2] = (mag_16bit[2] - mag_bias[2])* mag_scale[2]/100;
    
    
    //adjust axis to the one of accelerometer and gyroscope
    //mag_16bit[2] = -mag_16bit[2]; //-z
    //temp_magn = mag_16bit[0];
    //mag_16bit[0] = mag_16bit[1];
    //mag_16bit[1] = temp_magn; //invert 'x' and 'y' axis
    
    
    // convert the temp [2's complement] into MSB and LSB bytes 
    *mag_raw++ = mag_16bit[0] >> 8  ; //MSB
    *mag_raw++ = mag_16bit[0] & 0xFF; //LSB
    *mag_raw++ = mag_16bit[1] >> 8;
    *mag_raw++ = mag_16bit[1] & 0xFF;
    *mag_raw++ = mag_16bit[2] >> 8;
    *mag_raw++ = mag_16bit[2] & 0xFF;
    
    
    
    
    //uint8_t end_trasmission;
    // Read data via I2C
    //MPU9250_I2C_ReadMulti(AK8963_I2C_ADDRESS, MPU9250_MAG_XOUT_H_REG, mag, 6);
    //end_trasmission = MPU9250_I2C_Read(AK8963_I2C_ADDRESS, 0x09); //ST2 register (role as data reading end register)
}

void MPU9250_ReadSelfTestGyro(int16_t* self_test_gyro) {
    uint8_t temp[6];
    
    MPU9250_I2C_ReadMulti(MPU9250_I2C_ADDRESS, MPU9250_SELF_TEST_X_GYRO_REG, temp, 6);
    self_test_gyro[0] = ( temp[0] << 8) | ( temp[1] & 0xFF);
    self_test_gyro[1] = ( temp[1] << 8) | ( temp[3] & 0xFF);
    self_test_gyro[2] = ( temp[2] << 8) | ( temp[5] & 0xFF);
}

void MPU9250_ReadSelfTestAcc(int16_t* self_test_acc) {
    uint8_t temp[6];
    
    MPU9250_I2C_ReadMulti(MPU9250_I2C_ADDRESS, MPU9250_SELF_TEST_X_ACCEL_REG, temp, 6);
    self_test_acc[0] = ( temp[0] << 8) | ( temp[1] & 0xFF);
    self_test_acc[1] = ( temp[1] << 8) | ( temp[3] & 0xFF);
    self_test_acc[2] = ( temp[2] << 8) | ( temp[5] & 0xFF);
}

void MPU9250_SelfTest(float* deviation) {
    // Perform self test of accelerometer and gyroscope according to the
    // procedure described in the application note MPU-9250 Accelerometer, Gyroscope and
    // Compass Self-Test Implementation.
    
    ACCEL_FS_SEL Old_Acc_FS;    // Old value of accelerometer full scale range
    GYRO_FS_SEL Old_Gyro_FS;  // Old value of accelerometer full scale range
    int16_t ST_Acc[3] = {0,0,0};  // Accelerometer values with self test enabled
    int16_t Acc[3] = {0,0,0};     // Accelerometer average valus without self test enabled
    int16_t ST_Gyro[3] = {0,0,0}; // Gyroscope values with self test enabled
    int16_t Gyro[3] = {0,0,0};    // Gyroscope values without self test enabled
    int16_t ST_Response[6];       // Self test response on acc and gyro 3 axis
    
    // Get current accelerometer full scale range
    Old_Acc_FS = MPU9250_GetAccFS();
    // Get current gyroscope full scale range
    Old_Gyro_FS = MPU9250_GetGyroFS();
    
    // Set gyroscope full scale range to 250dps
    MPU9250_SetGyroFS(G250DPS);
    // Set accelerometer full scale range to 2g
    MPU9250_SetAccFS(A2G);
    
    // Write 0 to sample rate divider register -- no additional divider
    MPU9250_SetSampleRateDivider(0x00);
    
    // Set gyroscope and accelerometer DLPF configuration to 1kHz Fs, 92 Hz bandwidth
    // First set up configuration register so that DLPF CFG is set to 2
    MPU9250_I2C_Write(MPU9250_I2C_ADDRESS, MPU9250_CONFIG_REG, 0x02);
    // Then write 00 in FChoice_b of GYRO config register
    uint8_t temp = MPU9250_I2C_Read(MPU9250_I2C_ADDRESS, MPU9250_GYRO_CONFIG_REG);
    temp &= ~ 0x02; // Clear bits [1:0]
    MPU9250_I2C_Write(MPU9250_I2C_ADDRESS, MPU9250_GYRO_CONFIG_REG, temp);
    // Set accelerometer DLPF configuration
    MPU9250_I2C_Write(MPU9250_I2C_ADDRESS, MPU9250_ACCEL_CONFIG_REG, 0x02);
    // Get 200 readings 
    int16_t Acc_Temp[3];
    int16_t Gyro_Temp[3];
    
    for (int reading = 0; reading < 200; reading++) {
        MPU9250_ReadAcc(Acc_Temp);
        Acc[0] += Acc_Temp[0];
        Acc[1] += Acc_Temp[1];
        Acc[2] += Acc_Temp[2];
        MPU9250_ReadGyro(Gyro_Temp);
        Gyro[0] += Gyro_Temp[0];
        Gyro[1] += Gyro_Temp[1];
        Gyro[2] += Gyro_Temp[2];
    }
    // .. and average them
    for (int i = 0; i < 3; i++) {
        Acc[i]  /= 200;
        Gyro[i] /= 200;
    }
    char message[50];
    sprintf(message, "Avg: %5d %5d %5d -- %5d %5d %5d\r\n", Acc[0]*100, Acc[1]*100, Acc[2]*100, Gyro[0]*100, Gyro[1]*100, Gyro[2]*100);
    UART_Debug_PutString(message);
    // Enable self test gyroscope
    temp = MPU9250_I2C_Read(MPU9250_I2C_ADDRESS, MPU9250_GYRO_CONFIG_REG);
    // Then, set bits [7,6,5]
    temp |= 0b11100000;
    MPU9250_I2C_Write(MPU9250_I2C_ADDRESS, MPU9250_GYRO_CONFIG_REG, temp);
    // Enable self test accelerometer
    temp = MPU9250_I2C_Read(MPU9250_I2C_ADDRESS, MPU9250_ACCEL_CONFIG_REG);
    // Then, set bits [7,6,5]
    temp |= 0b11100000;
    MPU9250_I2C_Write(MPU9250_I2C_ADDRESS, MPU9250_ACCEL_CONFIG_REG, temp);
    
    // Wait 20 ms so that everything is stable
    CyDelay(20);
    
    for (int reading = 0; reading < 200; reading++) {
        MPU9250_ReadAcc(Acc_Temp);
        ST_Acc[0] += Acc_Temp[0];
        ST_Acc[1] += Acc_Temp[1];
        ST_Acc[2] += Acc_Temp[2];
        MPU9250_ReadGyro(Gyro_Temp);
        ST_Gyro[0] += Gyro_Temp[0];
        ST_Gyro[1] += Gyro_Temp[1];
        ST_Gyro[2] += Gyro_Temp[2];
    }
    // .. and average them
    for (int i = 0; i < 3; i++) {
        ST_Acc[i]  /= 200;
        ST_Gyro[i] /= 200;
    }
    sprintf(message, "STg: %5d %5d %5d -- %5d %5d %5d\r\n", ST_Acc[0]*100, ST_Acc[1]*100, ST_Acc[2]*100, ST_Gyro[0]*100, ST_Gyro[1]*100, ST_Gyro[2]*100);
    UART_Debug_PutString(message);
    // Disable self test gyroscope -- Read config register
    temp = MPU9250_I2C_Read(MPU9250_I2C_ADDRESS, MPU9250_GYRO_CONFIG_REG);
    // Clear bits [7,6,5]
    temp &= ~0b11100000;
    // Write new value to the register
    MPU9250_I2C_Write(MPU9250_I2C_ADDRESS, MPU9250_GYRO_CONFIG_REG, temp);
    // Disbale self test accelerometer -- Read config register
    temp = MPU9250_I2C_Read(MPU9250_I2C_ADDRESS, MPU9250_ACCEL_CONFIG_REG);
    // Clear bits [7,6,5]
    temp &= ~0b11100000;
    // Write new vale to the register
    MPU9250_I2C_Write(MPU9250_I2C_ADDRESS, MPU9250_ACCEL_CONFIG_REG, temp);
    
    // Wait 20 ms so that everything is stable
    CyDelay(20);
    
    // Restore full scale range values
    MPU9250_SetAccFS(Old_Acc_FS);
    MPU9250_SetGyroFS(Old_Gyro_FS);
    
    // Compute self test responses
    ST_Response[0] = ST_Acc[0] - Acc[0];
    ST_Response[1] = ST_Acc[1] - Acc[1];
    ST_Response[2] = ST_Acc[2] - Acc[2];
    ST_Response[3] = ST_Gyro[0] - Gyro[0];
    ST_Response[4] = ST_Gyro[1] - Gyro[1];
    ST_Response[5] = ST_Gyro[2] - Gyro[2];
    
    // Get values stored in the device for self test
    int16_t ST_AccStored[3];
    int16_t ST_GyroStored[3];
    
    MPU9250_ReadSelfTestAcc(ST_AccStored);
    MPU9250_ReadSelfTestGyro(ST_GyroStored);
    
    // Compute factory trim
    float Factory_Trim[6];
    Factory_Trim[0] = (float) ( ( 2620 / 1 << A2G) * pow( 1.01, ST_AccStored[0] - 1.0) );
    Factory_Trim[1] = (float) ( ( 2620 / 1 << A2G) * pow( 1.01, ST_AccStored[1] - 1.0) );
    Factory_Trim[2] = (float) ( ( 2620 / 1 << A2G) * pow( 1.01, ST_AccStored[2] - 1.0) );
    Factory_Trim[3] = (float) ( ( 2620 / 1 << A2G) * pow( 1.01, ST_GyroStored[0] - 1.0) );
    Factory_Trim[4] = (float) ( ( 2620 / 1 << A2G) * pow( 1.01, ST_GyroStored[1] - 1.0) );
    Factory_Trim[5] = (float) ( ( 2620 / 1 << A2G) * pow( 1.01, ST_GyroStored[2] - 1.0) );
    
    // Compute deviation
    for (int i = 0; i < 3; i++) {
        deviation[i] = 100.0 * ((float) (ST_Response[i] - Acc[i])) / Factory_Trim[i] - 100;
        deviation[i+3] = 100.0 * ((float) (ST_Response[i] - Gyro[i+3])) / Factory_Trim[i] - 100;
    }
}

void MPU9250_SetAccFS(ACCEL_FS_SEL fs) {
    // Write the new full scale value in the acc conf register
   
    // We need to first read the current bits of the register
    uint8_t temp = MPU9250_I2C_Read(MPU9250_I2C_ADDRESS, MPU9250_ACCEL_CONFIG_REG);
    // Then, we clear bits [4:3]
    temp &= ~MPU9250_ACC_FS_MASK;
    // Lastly, we write the new byte to the register
    MPU9250_I2C_Write(MPU9250_I2C_ADDRESS, MPU9250_ACCEL_CONFIG_REG, temp | ( fs << 3));
    
    // We also need to update the scaling factor
    switch(fs) {
    case A2G:
        acc_scale = MPU9250_G * 2.0f/pow(2,16)/2;
        break;
    case A4G:
        acc_scale = MPU9250_G * 4.0f/pow(2,16)/2;
        break;
    case A8G:
        acc_scale = MPU9250_G * 8.0f/pow(2,16)/2;
        break;
    case A16G:
        acc_scale = MPU9250_G * 16.0f/pow(2,16)/2;
        break;
        
    }
}

ACCEL_FS_SEL MPU9250_GetAccFS(void) {
    // Get the current full scale range of the accelerometer
    
    // First, get all the register bits
    uint8_t temp = MPU9250_I2C_Read(MPU9250_I2C_ADDRESS, MPU9250_ACCEL_CONFIG_REG);
    // Mask all bits expect [4:3]
    temp &= MPU9250_ACC_FS_MASK;
    // Shift them by 3
    temp = temp >> 3;
    return temp;
}

void MPU9250_SetGyroFS(GYRO_FS_SEL fs) {
    // Write the new full scale value in the gyro conf register
    
    // We need to first read the current bits of the register
    uint8_t temp = MPU9250_I2C_Read(MPU9250_I2C_ADDRESS, MPU9250_GYRO_CONFIG_REG);
    // Then, we clear bits [4:3]
    temp &= ~MPU9250_GYRO_FS_MASK;
    // Lastly, we write the new byte to the register
    MPU9250_I2C_Write(MPU9250_I2C_ADDRESS, MPU9250_GYRO_CONFIG_REG, temp | ( fs << 3));
}

GYRO_FS_SEL MPU9250_GetGyroFS(void) {
    // Get the current full scale range of the accelerometer
    
    // First, get all the register bits
    uint8_t temp = MPU9250_I2C_Read(MPU9250_I2C_ADDRESS, MPU9250_GYRO_CONFIG_REG);
    // Mask all bits expect [4:3]
    temp &= MPU9250_GYRO_FS_MASK;
    // Shift them by 3
    temp = temp >> 3;
    return temp;
}

// ********************************** G: ****************
float get_acc_resolution(const ACCEL_FS_SEL accel_af_sel) {
        switch (accel_af_sel) {
            // Possible accelerometer scales (and their register bit settings) are:
            // 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11).
            // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
            case A2G:
                return 2.0 / 32768.0;
            case A4G:
                return 4.0 / 32768.0;
            case A8G:
                return 8.0 / 32768.0;
            case A16G:
                return 16.0 / 32768.0;
            default:
                return 0.;
        }
    }

    float get_gyro_resolution(const GYRO_FS_SEL gyro_fs_sel) {
        switch (gyro_fs_sel) {
            // Possible gyro scales (and their register bit settings) are:
            // 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11).
            // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
            case G250DPS:
                return 250.0 / 32768.0;
            case G500DPS:
                return 500.0 / 32768.0;
            case G1000DPS:
                return 1000.0 / 32768.0;
            case G2000DPS:
                return 2000.0 / 32768.0;
            default:
                return 0.;
        }
    }

    float get_mag_resolution(const MAG_OUTPUT_BITS mag_output_bits) {
        switch (mag_output_bits) {
            // Possible magnetometer scales (and their register bit settings) are:
            // 14 bit resolution (0) and 16 bit resolution (1)
            // Proper scale to return milliGauss
            case M14BITS:
                return 10. * 4912. / 8190.0;
            case M16BITS:
                return 10. * 4912. / 32760.0;
            default:
                return 0.;
        }
    }
// ********************************************************************
    
void MPU9250_SetSampleRateDivider(uint8_t smplrt) {
    MPU9250_I2C_Write(MPU9250_I2C_ADDRESS, MPU9250_SMPLRT_DIV_REG, smplrt);
}

void MPU9250_ReadAccelerometerOffset(int16_t *acc_offset) {
    // Get the accelerometer offset values
    uint8_t temp[6] = {'\0'};
    MPU9250_I2C_ReadMulti(MPU9250_I2C_ADDRESS, MPU9250_XA_OFFSET_H_REG, temp, 6);
    acc_offset[0] = (temp[0] << 8) | (temp[1] & 0xFF);
    acc_offset[0] = (temp[2] << 8) | (temp[3] & 0xFF);
    acc_offset[0] = (temp[4] << 8) | (temp[5] & 0xFF);
}

void MPU9250_EnableRawDataInterrupt(void) {
    // Set bit [0] of MPU9250_INT_EN_REG
    // Read current value
    uint8_t temp = MPU9250_I2C_Read(MPU9250_I2C_ADDRESS, MPU9250_INT_ENABLE_REG);
    // Set bit[0]
    temp |= 0x01;
    // Write new value
    MPU9250_I2C_Write(MPU9250_I2C_ADDRESS, MPU9250_INT_ENABLE_REG, temp);
}

void MPU9250_DisableRawDataInterrupt(void) {
    // Clear bit [0] of MPU9250_INT_EN_REG
    // Read current value
    uint8_t temp = MPU9250_I2C_Read(MPU9250_I2C_ADDRESS, MPU9250_INT_ENABLE_REG);
    // Clear bit[0]
    temp &= ~0x01;
    // Write new value
    MPU9250_I2C_Write(MPU9250_I2C_ADDRESS, MPU9250_INT_ENABLE_REG, temp);
}

void MPU9250_EnableFsyncInterrupt(void) {
    // Set bit [3] of MPU9250_INT_EN_REG
    // Read current value
    uint8_t temp = MPU9250_I2C_Read(MPU9250_I2C_ADDRESS, MPU9250_INT_ENABLE_REG);
    // Set bit[3]
    temp |= 0x08;
    // Write new value
    MPU9250_I2C_Write(MPU9250_I2C_ADDRESS, MPU9250_INT_ENABLE_REG, temp);
}

void MPU9250_DisableFsyncInterrupt(void) {
    // Clear bit [3] of MPU9250_INT_EN_REG
    // Read current value
    uint8_t temp = MPU9250_I2C_Read(MPU9250_I2C_ADDRESS, MPU9250_INT_ENABLE_REG);
    // Clear bit[3]
    temp &= ~0x08;
    // Write new value
    MPU9250_I2C_Write(MPU9250_I2C_ADDRESS, MPU9250_INT_ENABLE_REG, temp);
}

void MPU9250_EnableFifoOverflowInterrupt(void) {
    // Set bit [4] of MPU9250_INT_EN_REG
    // Read current value
    uint8_t temp = MPU9250_I2C_Read(MPU9250_I2C_ADDRESS, MPU9250_INT_ENABLE_REG);
    // Set bit[4]
    temp |= 0x10;
    // Write new value
    MPU9250_I2C_Write(MPU9250_I2C_ADDRESS, MPU9250_INT_ENABLE_REG, temp);
}

void MPU9250_DisableFifoOverflowInterrupt(void) {
    // Clear bit [4] of MPU9250_INT_EN_REG
    // Read current value
    uint8_t temp = MPU9250_I2C_Read(MPU9250_I2C_ADDRESS, MPU9250_INT_ENABLE_REG);
    // Clear bit[4]
    temp &= ~0x10;
    // Write new value
    MPU9250_I2C_Write(MPU9250_I2C_ADDRESS, MPU9250_INT_ENABLE_REG, temp);
}

void MPU9250_EnableWomInterrupt(void) {
    // Set bit [6] of MPU9250_INT_EN_REG
    // Read current value
    uint8_t temp = MPU9250_I2C_Read(MPU9250_I2C_ADDRESS, MPU9250_INT_ENABLE_REG);
    // Set bit[6]
    temp |= 0x40;
    // Write new value
    MPU9250_I2C_Write(MPU9250_I2C_ADDRESS, MPU9250_INT_ENABLE_REG, temp);
}

void MPU9250_DisableWomInterrupt(void) {
    // Clear bit [6] of MPU9250_INT_EN_REG
    // Read current value
    uint8_t temp = MPU9250_I2C_Read(MPU9250_I2C_ADDRESS, MPU9250_INT_ENABLE_REG);
    // Clear bit[6]
    temp &= ~0x40;
    // Write new value
    MPU9250_I2C_Write(MPU9250_I2C_ADDRESS, MPU9250_INT_ENABLE_REG, temp);
}

uint8_t MPU9250_ReadInterruptStatus(void) {
    return MPU9250_I2C_Read(MPU9250_I2C_ADDRESS, MPU9250_INT_STATUS_REG);
}

void MPU9250_SetInterruptActiveHigh(void) {
    // Clear bit [7] of MPU9250_INT_PIN_CFG_REG
    // Read current value
    uint8_t temp = MPU9250_I2C_Read(MPU9250_I2C_ADDRESS, MPU9250_INT_PIN_CFG_REG); //G: before it was setting the MPU9250_INT_ENABLE_REG
    // Clear bit[7]
    temp &= ~0x80;
    // Write new value
    MPU9250_I2C_Write(MPU9250_I2C_ADDRESS, MPU9250_INT_PIN_CFG_REG, temp);
}

void MPU9250_SetInterruptActiveLow(void) {
    // Set bit [7] of MPU9250_INT_PIN_CFG_REG
    // Read current value
    uint8_t temp = MPU9250_I2C_Read(MPU9250_I2C_ADDRESS, MPU9250_INT_PIN_CFG_REG);
    // Set bit[7]
    temp |= 0x40;
    // Write new value
    MPU9250_I2C_Write(MPU9250_I2C_ADDRESS, MPU9250_INT_PIN_CFG_REG, temp);
}

void MPU9250_SetInterruptOpenDrain(void) {
    // Clear bit [6] of MPU9250_INT_PIN_CFG_REG
    // Read current value
    uint8_t temp = MPU9250_I2C_Read(MPU9250_I2C_ADDRESS, MPU9250_INT_ENABLE_REG);
    // Clear bit[6]
    temp &= ~0x40;
    // Write new value
    MPU9250_I2C_Write(MPU9250_I2C_ADDRESS, MPU9250_INT_ENABLE_REG, temp);
}

void MPU9250_SetInterruptPushPull(void) {
    // Set bit [6] of MPU9250_INT_PIN_CFG_REG
    // Read current value
    uint8_t temp = MPU9250_I2C_Read(MPU9250_I2C_ADDRESS, MPU9250_INT_PIN_CFG_REG);
    // Set bit[6]
    temp |= 0x40;
    // Write new value
    MPU9250_I2C_Write(MPU9250_I2C_ADDRESS, MPU9250_INT_PIN_CFG_REG, temp);
}

void MPU9250_HeldInterruptPin(void) {
    // Set bit[5] of MPU9250_INT_PIN_CFG_REG
    // Read current value
    uint8_t temp = MPU9250_I2C_Read(MPU9250_I2C_ADDRESS, MPU9250_INT_PIN_CFG_REG);
    // Set bit[5]
    temp |= 0x20;
    // Write new value
    MPU9250_I2C_Write(MPU9250_I2C_ADDRESS, MPU9250_INT_PIN_CFG_REG, temp);
}
    
void MPU9250_InterruptPinPulse(void) {
    // Clear bit [5] of MPU9250_INT_PIN_CFG_REG
    // Read current value
    uint8_t temp = MPU9250_I2C_Read(MPU9250_I2C_ADDRESS, MPU9250_INT_ENABLE_REG);
    // Clear bit[5]
    temp &= ~0x20;
    // Write new value
    MPU9250_I2C_Write(MPU9250_I2C_ADDRESS, MPU9250_INT_ENABLE_REG, temp);   
}

void MPU9250_ClearInterruptAny(void) {
    // Set bit[4] of MPU9250_INT_PIN_CFG_REG
    // Read current value
    uint8_t temp = MPU9250_I2C_Read(MPU9250_I2C_ADDRESS, MPU9250_INT_PIN_CFG_REG);
    // Set bit[4]
    temp |= 0x10;
    // Write new value
    MPU9250_I2C_Write(MPU9250_I2C_ADDRESS, MPU9250_INT_PIN_CFG_REG, temp);   
}

void MPU9250_ClearInterruptStatusReg(void) {
    // Clear bit [4] of MPU9250_INT_PIN_CFG_REG
    // Read current value
    uint8_t temp = MPU9250_I2C_Read(MPU9250_I2C_ADDRESS, MPU9250_INT_PIN_CFG_REG); //G: before it was setting the MPU9250_INT_ENABLE_REG
    // Clear bit[4]
    temp &= ~0x10;
    // Write new value
    MPU9250_I2C_Write(MPU9250_I2C_ADDRESS, MPU9250_INT_PIN_CFG_REG, temp);      
}

void MPU9250_EnableI2CBypass(void) {
    // Clear bit [5] of MPU9250_USER_CTRL_REG
    // Read current value
    uint8_t temp = MPU9250_I2C_Read(MPU9250_I2C_ADDRESS, MPU9250_USER_CTRL_REG);
    // Clear bit [5]
    temp &= ~0x20;
    // Write new value
    MPU9250_I2C_Write(MPU9250_I2C_ADDRESS, MPU9250_USER_CTRL_REG, temp);   
    
    
    // Set bit [1] of MPU9250_INT_PIN_CFG_REG
    // Read current value
    temp = MPU9250_I2C_Read(MPU9250_I2C_ADDRESS, MPU9250_INT_PIN_CFG_REG);
    // Set bit[1]
    temp |= 0x02;
    // Write new value
    MPU9250_I2C_Write(MPU9250_I2C_ADDRESS, MPU9250_INT_PIN_CFG_REG, temp);
}

void MPU9250_DisableI2CBypass(void) {
    // Set bit [5] of MPU9250_INT_ENABLE_REG
    // Read current value
    uint8_t temp = MPU9250_I2C_Read(MPU9250_I2C_ADDRESS, MPU9250_USER_CTRL_REG);
    // Set bit [5]
    temp |= 0x20;
    // Write new value
    MPU9250_I2C_Write(MPU9250_I2C_ADDRESS, MPU9250_USER_CTRL_REG, temp); 
    
    // Clear bit [1] of MPU9250_INT_PIN_CFG_REG
    // Read current value
    temp = MPU9250_I2C_Read(MPU9250_I2C_ADDRESS, MPU9250_INT_ENABLE_REG);
    // Clear bit[1]
    temp &= ~0x02;
    // Write new value
    MPU9250_I2C_Write(MPU9250_I2C_ADDRESS, MPU9250_INT_ENABLE_REG, temp);
}

void MPU9250_MAG_Enable(void) {
    
    MPU9250_I2C_Write(AK8963_I2C_ADDRESS, MPU9250_MAG_CNTL1_REG, 0x00);  // Power down magnetometer
    CyDelay(10); 
    
    MPU9250_I2C_Write(AK8963_I2C_ADDRESS, 0x0B,
                    (MPU9250_I2C_Read(AK8963_I2C_ADDRESS, 0x0B) | 0x01));  // (reset CNTL2 0x0B --> all reg reset
        // For self-test:
        //MPU9250_I2C_Write(AK8963_I2C_ADDRESS, 0x0C,
        //                (MPU9250_I2C_Read(AK8963_I2C_ADDRESS, 0x0C) | 0x40));  // (set ASTC bit6 high)   --> selftest
    CyDelay(10);
    MPU9250_I2C_Write(AK8963_I2C_ADDRESS, MPU9250_MAG_CNTL1_REG, 0x0F);  // Enter Fuse ROM access mode
    CyDelay(10); 
    MPU9250_I2C_ReadMulti(AK8963_I2C_ADDRESS, 0x10, &raw_data_calib_magn[0], 3);      // Read the x-, y-, and z-axis calibration values
        mag_bias_factory[0] = (float)(raw_data_calib_magn[0] - 128) / 256. + 1.;  // Return x-axis sensitivity adjustment values, etc.
        mag_bias_factory[1] = (float)(raw_data_calib_magn[1] - 128) / 256. + 1.;
        mag_bias_factory[2] = (float)(raw_data_calib_magn[2] - 128) / 256. + 1.; 
    MPU9250_I2C_Write(AK8963_I2C_ADDRESS, MPU9250_MAG_CNTL1_REG, 0x00);  // Power down magnetometer before final setting
    CyDelay(10);
    // setting the MPU9250_MAG_CNTL1_REG: 
        // 0x00 = MAG off (default)
        // 0x01 = MAG on (14-bit output)
        // 0x02 = Continuous mode 1
        // 0x11 = MAG on AND 16-bit output with SNGL_MODE(0x11-> 00010001) 
        // 0x12 = MAG on AND 16-bit output with CNT_MODE1, 8Hz (0x12-> 00010010) 
        // 0x16 = MAG on AND 16-bit output with CNT_MODE2, 100Hz (0x16-> 00010110) 
    MPU9250_I2C_Write(AK8963_I2C_ADDRESS, MPU9250_MAG_CNTL1_REG, 0x16);  // MAG on AND 16-bit output
                                //0x16 -> (uint8_t)setting.mag_output_bits << 4 | MAG_MODE
    
}

void MPU9250_MAG_Disable(void) {
    
    MPU9250_I2C_Write(AK8963_I2C_ADDRESS, MPU9250_MAG_CNTL1_REG, 0x00);
    
    CyDelay(10);
    
}
/* [] END OF FILE */
