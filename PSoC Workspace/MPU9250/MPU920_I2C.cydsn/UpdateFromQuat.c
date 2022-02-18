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
#define UPDATEFROMQUAT_H
#define PI_ 3.14 
#include "project.h"
#include "math.h"
extern int16_t roll;
extern int16_t pitch;
extern int16_t yaw;
    
    
void update_rpy(float qw, float qx, float qy, float qz) {
        // Define output variables from updated quaternion---these are Tait-Bryan angles, commonly used in aircraft orientation.
        // In this coordinate system, the positive z-axis is down toward Earth.
        // Yaw is the angle between Sensor x-axis and Earth magnetic North (or true North if corrected for local declination, looking down on the sensor positive yaw is counterclockwise.
        // Pitch is angle between sensor x-axis and Earth ground plane, toward the Earth is positive, up toward the sky is negative.
        // Roll is angle between sensor y-axis and Earth ground plane, y-axis up is positive roll.
        // These arise from the definition of the homogeneous rotation matrix constructed from quaternions.
        // Tait-Bryan angles as well as Euler angles are non-commutative; that is, the get the correct orientation the rotations must be
        // applied in the correct order which for this configuration is yaw, pitch, and then roll.
        // For more see http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles which has additional links.
        float a12, a22, a31, a32, a33;  // rotation matrix coefficients for Euler angles and gravity components
        a12 = 2.0f * (qx * qy + qw * qz);
        a22 = qw * qw + qx * qx - qy * qy - qz * qz;
        a31 = 2.0f * (qw * qx + qy * qz);
        a32 = 2.0f * (qx * qz - qw * qy);
        a33 = qw * qw - qx * qx - qy * qy + qz * qz;
        yaw = (int16_t)atan2f(a31, a33);
        pitch = (int16_t)-asinf(a32);
        roll = atan2f(a12, a22);
        yaw *= (int16_t)180.0f / PI_;
        pitch *= (int16_t)180.0f / PI_;
        roll *= (int16_t)180.0f / PI_; //yaw, pitch, and then roll.
        roll += (int16_t)3; //magnetic declination in Milan
        if (roll >= +180.f)
            roll -= (int16_t)360.f;
        else if (roll < -180.f)
            roll += (int16_t)360.f;

        //lin_acc[0] = a[0] + a31;
        //lin_acc[1] = a[1] + a32;
        //lin_acc[2] = a[2] - a33;
    }
/* [] END OF FILE */
