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



#ifndef __EEPROM__H
    #define __EEPROM__H
    
    #include <cytypes.h>
    #include <I2C_MPU9250_Master.h>
    
    void EEPROM_Write(uint16_t reg_add, uint8_t data[], uint8_t dim);

    void EEPROM_Write123(uint16_t reg_add, uint8_t data[], uint8_t dim);
    
    void EEPROM_Read(uint16_t reg_add, uint8_t data[], uint8_t dim);
    
    void EEPROM_Read_exclude(uint16_t reg_add, uint8_t *data, uint8_t dim);
    
    uint8_t EEPROM_Read123(uint16_t reg_add);
    
    void add_update(uint16_t add, uint8_t session_n);

#endif
/* [] END OF FILE */
