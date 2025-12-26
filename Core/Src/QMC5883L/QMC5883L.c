#include "QMC5883L.h"
#include <math.h>

// Ð? nh?y v?i range 8G (r?t ph? bi?n trên GEP-M1025Q)
static const float sensitivity_8G = 0.00033333f;  // = 1 / 3000

void QMC5883L_Init(QMC5883L_t *dev, I2C_HandleTypeDef *i2c_handle)
{
    dev->i2c_handle = i2c_handle;

    uint8_t chip_id = 0;

    /* Read Chip ID */
    if (HAL_I2C_Mem_Read(i2c_handle,
                         QMC5883L_ADDR,
                         QMC5883L_REG_CHIP_ID,
                         I2C_MEMADD_SIZE_8BIT,
                         &chip_id,
                         1,
                         100) != HAL_OK)
    {
        /* I2C error */
        // dev->status = QMC_ERR_I2C;
        return;
    }

    /* QMC5883L thu?ng tr? v? 0xFF */
    if (chip_id != 0xFF)
    {
        /* Not QMC5883L */
        // dev->status = QMC_ERR_ID;
        return;
    }

    /* Soft reset */
    uint8_t data = 0x80;
    HAL_I2C_Mem_Write(i2c_handle,
                      QMC5883L_ADDR,
                      QMC5883L_REG_CONF_2,
                      I2C_MEMADD_SIZE_8BIT,
                      &data,
                      1,
                      100);
    HAL_Delay(20);

    /* Set/Reset Period (datasheet recommend = 0x01) */
    data = 0x01;
    HAL_I2C_Mem_Write(i2c_handle,
                      QMC5883L_ADDR,
                      QMC5883L_REG_SET_RESET,
                      I2C_MEMADD_SIZE_8BIT,
                      &data,
                      1,
                      100);
    HAL_Delay(10);

    /* Continuous mode, ODR 100Hz, Range ±8G, OSR 512 */
    data = QMC5883L_MODE_CONTINUOUS |
           QMC5883L_ODR_100HZ |
           QMC5883L_RNG_8G |
           QMC5883L_OSR_512;

    HAL_I2C_Mem_Write(i2c_handle,
                      QMC5883L_ADDR,
                      QMC5883L_REG_CONF_1,
                      I2C_MEMADD_SIZE_8BIT,
                      &data,
                      1,
                      100);

    HAL_Delay(50);

    // dev->status = QMC_OK;
}


uint8_t QMC5883L_ReadMag(QMC5883L_t *dev) {
    uint8_t status;
    uint8_t raw[6];


    if (HAL_I2C_Mem_Read(dev->i2c_handle, QMC5883L_ADDR, QMC5883L_REG_DATA_X_LSB, 1, raw, 6, 100) != HAL_OK) {
        return 0;
    }

    dev->mag_x_raw = (int16_t)(raw[0] | (raw[1] << 8));
    dev->mag_y_raw = (int16_t)(raw[2] | (raw[3] << 8));
    dev->mag_z_raw = (int16_t)(raw[4] | (raw[5] << 8));

    dev->mag_x = dev->mag_x_raw * sensitivity_8G;
    dev->mag_y = dev->mag_y_raw * sensitivity_8G;
    dev->mag_z = dev->mag_z_raw * sensitivity_8G;

    return 1;
}

float QMC5883L_CalculateHeading(QMC5883L_t *dev) {
    // Tilt-compensated heading (ch? c?n X và Y n?u module n?m ngang – dúng v?i GEP-M1025Q)
    float heading = atan2f(dev->mag_y, dev->mag_x) * 180.0f / 3.14159265359f;

    // Chu?n hóa v? 0–360°
    if (heading < 0) heading += 360.0f;

    dev->heading = heading;
    return heading;
}

// Dùng khi compass b? l?ch lâu ngày
void QMC5883L_SoftReset(QMC5883L_t *dev) {
    uint8_t data = 0x80;
    HAL_I2C_Mem_Write(dev->i2c_handle, QMC5883L_ADDR, QMC5883L_REG_CONF_2, 1, &data, 1, 100);
    HAL_Delay(100);
}