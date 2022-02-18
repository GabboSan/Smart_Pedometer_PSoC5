# AY2022 I Project-1

Smart Pedometer project by Group 1:

1. d'Arenzo Daniele
2. Santicchi Gabriele
3. Santini Matteo

In this work a PSoC CY8C5888LTI-LP097 communicates by I2C protocol with MPU9250, SSD1306, 24LC512 and by UART protocol with HC-06 Bluetooth.\
A PyQt5 GUI has been developed to upload sessions and set user credential.

## Aim of this Work

1. Implement a pedometer algorithm capable to identify and count steps online;
2. Exploit the over-threshold events interrupt of the MPU9250 to automatically wake up the device from sleep; go to sleep mode when no motion is detected for 1 minute;
3. Display in the GUI some statistics: total number of steps, average cadence, total activity time and the session path with respect to the origin (i.e. first 'step' detected);
4. Log the statistics of each session (i.e., of every walk detected) in the internal EEPROM of the PSoC;
5. Minimization of power consumption, by using powersaving functionalities of the PSoC (e.g., Alternate mode) as well as turining off external peripherals when not in use;
6. When connected via USB to the computer, the device can accept a data request and display/send the data stored into the EEPROM.

## MPU9250 and AHRS filtering

To keep track of the direction of the walk, the AHRS filter has been implemented to get a Roll-Pitch-Yaw estimate. As the filter needs the magnetometer measurements, the AK8963 is enabled and set as Slave on the I2C bus.
Each 2 seconds the pedometer algorithm compute the X and Y direction of the walk, by considering the RPY orientation of the device.
The two directions are saved as 16 bit and then saved into the cell of the EEPROM . Path is reconstructed by the PC through this relative coordinates system.

## EEPROM Session Data Structure

When no motion has been detected for 1 minute, the Wake-On-Motion (WOM) interrupt is enabled and session data are saved in the EEPROM with the following structure.\
**RESERVED CELL REGISTERS:**
* 0: Height;
* 1: UserID;
* 2: SessionID;
* 3-4: lastAddressEEPROM: to avoid data overwriting;

**CELL REGISTERS:**

From 24LC512 Datasheet, Read and Write max. operation number is limited to a Page size, which consist of 128 bytes. The Packets to be sent to the PC are structured as:

* UserID, SessionID, HEADER (0xFE,0xFF);
* dataXY: sequence of X and Y relative coordinates (in 16 bits) computed each 2 seconds;
* Number of Steps, TAIL (0xFF,0xFF);

## SSD1306 Display

The SSD1306 communicates by I2C with the Master. It displays the total step counter and the current speed (in cm/s) each 2 seconds. When the device enters in Sleep mode, the display is turned off.


## Final PCB ad Case
![alt text](https://github.com/GabboSan/Smart_Pedometer_PSoC5/blob/main/schematic_image.png)
# Guide

## Device Guide

Device must be put parallel to the calf fibers in order to get meaningful information about the orientation. During a Session, the Step counter and the Current speed are updated each 2 seconds on the SSD1306 display.\
A session starts when the device is powered, and ends when no motion has been detected for more than 1 minute. Then, the session is saved into the memory and the device enter in sleep mode until detection of futher N steps.

## GUI Guide

While running the GUI, the Device must be connected either by USB or Bluetooth. Even if an automatic scan of ports is implemented, sometimes a manual selection of the COM is required.
To get meaningful information about the orientation, the calibration of the magnetometer is recommended (move the device along an eigth figure until the calibration is not finished).\
The GUI is organized in tabs:

* Sessions: see the statistics related to each session downloaded from the device;
* Live Viewer: see the Session path and the current 3D orientation of the device;
* Edit Profile: to create, delete and modify a user;
* Advanced Tool: monitor Accelerometer, Gyroscope and Magnetometer values in real-time.  
