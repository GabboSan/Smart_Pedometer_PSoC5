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
#pragma once
#ifndef QUATERNIONFILTER_H
#include "project.h"


// Madgwick Quaternion Update
void madgwick2(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float* q);
#endif  // QUATERNIONFILTER_H
