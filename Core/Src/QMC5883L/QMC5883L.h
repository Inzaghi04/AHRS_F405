#ifndef QMC5883L_H
#define QMC5883L_H

#include "stm32f4xx_hal.h"  

#define QMC5883L_ADDR           0x0D << 1 

// Các thanh ghi chính xác c?a QMC5883L
#define QMC5883L_REG_DATA_X_LSB 0x00
#define QMC5883L_REG_DATA_X_MSB 0x01
#define QMC5883L_REG_DATA_Y_LSB 0x02
#define QMC5883L_REG_DATA_Y_MSB 0x03
#define QMC5883L_REG_DATA_Z_LSB 0x04
#define QMC5883L_REG_DATA_Z_MSB 0x05
#define QMC5883L_REG_STATUS     0x06
#define QMC5883L_REG_TEMP_LSB   0x07
#define QMC5883L_REG_TEMP_MSB    0x08
#define QMC5883L_REG_CONF_1     0x09
#define QMC5883L_REG_CONF_2     0x0A
#define QMC5883L_REG_SET_RESET  0x0B
#define QMC5883L_REG_CHIP_ID    0x0D        

// C?u hình cho CONF1
#define QMC5883L_MODE_CONTINUOUS 0x01
#define QMC5883L_ODR_10HZ        0x00
#define QMC5883L_ODR_50HZ        0x04
#define QMC5883L_ODR_100HZ       0x08
#define QMC5883L_ODR_200HZ       0x0C
#define QMC5883L_RNG_2G          0x00
#define QMC5883L_RNG_8G          0x10
#define QMC5883L_OSR_512         0x00
#define QMC5883L_OSR_256         0x40
#define QMC5883L_OSR_128         0x80
#define QMC588L_OSR_64          0xC0

typedef struct {
    I2C_HandleTypeDef *i2c_handle;
    
    int16_t mag_x_raw, mag_y_raw, mag_z_raw;
    float   mag_x, mag_y, mag_z;
    float   heading;
} QMC5883L_t;

void QMC5883L_Init(QMC5883L_t *dev, I2C_HandleTypeDef *i2c_handle);
uint8_t QMC5883L_ReadMag(QMC5883L_t *dev);
float   QMC5883L_CalculateHeading(QMC5883L_t *dev);
void    QMC5883L_SoftReset(QMC5883L_t *dev);

#endif