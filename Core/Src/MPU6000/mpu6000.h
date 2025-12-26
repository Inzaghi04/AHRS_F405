#ifndef __MPU6000_H_
#define __MPU6000_H_

#include "stm32f4xx_hal.h"
#include <stdbool.h>

#define MPU6000_CS_PIN GPIO_PIN_4
#define MPU6000_CS_PORT GPIOA

// Power Management Registers
#define MPU6000_PWR_MGMT_1        0x6B  // Power management 1 register
#define MPU6000_PWR_MGMT_2        0x6C  // Power management 2 register

// Who Am I Register (to verify communication with MPU-6000)
#define MPU6000_WHO_AM_I          0x75  // WHO_AM_I register (should return 0x68)

// Accelerometer Registers
#define ACCEL_SCALE 		      16384.0f
#define MPU6000_ACCEL_XOUT_H      0x3B  // Accelerometer X-axis high byte
#define MPU6000_ACCEL_XOUT_L      0x3C  // Accelerometer X-axis low byte
#define MPU6000_ACCEL_YOUT_H      0x3D  // Accelerometer Y-axis high byte
#define MPU6000_ACCEL_YOUT_L      0x3E  // Accelerometer Y-axis low byte
#define MPU6000_ACCEL_ZOUT_H      0x3F  // Accelerometer Z-axis high byte
#define MPU6000_ACCEL_ZOUT_L      0x40  // Accelerometer Z-axis low byte

// Gyroscope Registers
#define GYRO_SCALE				  131.0f
#define MPU6000_GYRO_XOUT_H       0x43  // Gyroscope X-axis high byte
#define MPU6000_GYRO_XOUT_L       0x44  // Gyroscope X-axis low byte
#define MPU6000_GYRO_YOUT_H       0x45  // Gyroscope Y-axis high byte
#define MPU6000_GYRO_YOUT_L       0x46  // Gyroscope Y-axis low byte
#define MPU6000_GYRO_ZOUT_H       0x47  // Gyroscope Z-axis high byte
#define MPU6000_GYRO_ZOUT_L       0x48  // Gyroscope Z-axis low byte

// Temperature Register
#define MPU6000_TEMP_OUT_H        0x41  // Temperature high byte
#define MPU6000_TEMP_OUT_L        0x42  // Temperature low byte

// Configuration Registers
#define MPU6000_CONFIG            0x1A  // Configuration register (for setting low-pass filter)
#define MPU6000_GYRO_CONFIG       0x1B  // Gyroscope configuration register
#define MPU6000_ACCEL_CONFIG      0x1C  // Accelerometer configuration register
#define MPU6000_ACCEL_CONFIG_2    0x1D  // Accelerometer configuration 2 register

// Interrupt Registers
#define MPU6000_INT_PIN_CFG       0x37  // Interrupt pin configuration register
#define MPU6000_INT_ENABLE        0x38  // Interrupt enable register
#define MPU6000_INT_STATUS        0x3A  // Interrupt status register

// FIFO Registers
#define MPU6000_FIFO_EN           0x23  // FIFO enable register
#define MPU6000_FIFO_COUNT_H      0x72  // FIFO count high byte
#define MPU6000_FIFO_COUNT_L      0x73  // FIFO count low byte
#define MPU6000_FIFO_R_W          0x74  // FIFO read/write register

// User Control Registers
#define MPU6000_USER_CTRL         0x6A  // User control register
#define MPU6000_FIFO_RST          0x68  // FIFO reset bit in User Control register
#define MPU6000_I2C_MST_RST       0x06  // I2C master reset bit in User Control register
#define MPU6000_SIG_COND          0x69  // Signal path reset register

typedef struct{
	SPI_HandleTypeDef *hspi;
	float acc[3];
	float gyro[3];

    float acc_offset[3];
    float gyro_offset[3];

	float temp;
	uint8_t tx_buffer[16];
	uint8_t dma_buffer[16];
	bool calibrated;
	volatile uint8_t state;
	volatile bool data_ready;
	volatile bool spi_transfer_done;
} MPU6000;

void MPU6000_Start_DMA(MPU6000 *dev);
void MPU6000_Read_Blocking(MPU6000 *dev, uint8_t *spi_rx_buffer);
void MPU6000_Process_DMA(MPU6000 *dev);
void MPU6000_Process(MPU6000 *dev, uint8_t* spi_rx_buffer);

void MPU6000_Calibrate(MPU6000 *dev);
uint16_t MPU6000_Read(MPU6000 *dev,uint8_t reg);
void MPU6000_Write(MPU6000 *dev,uint8_t reg,uint8_t data);
void MPU6000_Init(MPU6000 *dev,SPI_HandleTypeDef *hspi);

#endif
