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
#include "cytypes.h"

void Fusion_init(void);
void MagCalibration_acquisition(float32 *magn);
void MagCalibration_unbias(void);
void AHRS_filtering(float32* acc, float32* gyro, float32* magn);
void SaveSession(void);

/* [] END OF FILE */