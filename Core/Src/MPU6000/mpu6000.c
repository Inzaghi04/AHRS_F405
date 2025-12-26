#include "mpu6000.h"
uint16_t MPU6000_Read(MPU6000 *dev, uint8_t reg) {
    uint8_t tx[2] = {reg | 0x80, 0x00}; // reg addr + dummy
    uint8_t rx[2] = {0};

    HAL_GPIO_WritePin(MPU6000_CS_PORT, MPU6000_CS_PIN, GPIO_PIN_RESET);

    HAL_SPI_TransmitReceive(dev->hspi, tx, rx, 2, HAL_MAX_DELAY);

    HAL_GPIO_WritePin(MPU6000_CS_PORT, MPU6000_CS_PIN, GPIO_PIN_SET);

    return rx[1]; // the second byte is the register value
}

void MPU6000_Write(MPU6000 *dev, uint8_t reg, uint8_t data) {
    uint8_t tx[2] = {reg & 0x7F, data};

    HAL_GPIO_WritePin(MPU6000_CS_PORT, MPU6000_CS_PIN, GPIO_PIN_RESET);

    HAL_SPI_Transmit(dev->hspi, tx, 2, HAL_MAX_DELAY);

    HAL_GPIO_WritePin(MPU6000_CS_PORT, MPU6000_CS_PIN, GPIO_PIN_SET);
}

void MPU6000_Init(MPU6000 *dev, SPI_HandleTypeDef *hspi) {
    dev->hspi = hspi;

    HAL_GPIO_WritePin(MPU6000_CS_PORT, MPU6000_CS_PIN, GPIO_PIN_SET);
    HAL_Delay(100);

    // Reset device
    MPU6000_Write(dev, MPU6000_PWR_MGMT_1, 0x80);
    HAL_Delay(100);

    // Wake up with PLL
    MPU6000_Write(dev, MPU6000_PWR_MGMT_1, 0x01);
    HAL_Delay(10);

    uint8_t reg = MPU6000_Read(dev, MPU6000_GYRO_CONFIG);
    reg &= ~0x18; // clear FS_SEL
    reg |= (0 << 3);
    MPU6000_Write(dev, MPU6000_GYRO_CONFIG, reg);

    reg = MPU6000_Read(dev, MPU6000_ACCEL_CONFIG);
    reg &= ~0x18; // clear AFS_SEL
    reg |= (0 << 3);
    MPU6000_Write(dev, MPU6000_ACCEL_CONFIG, reg);

    // Verify WHO_AM_I
    uint8_t whoami = MPU6000_Read(dev, MPU6000_WHO_AM_I);
    if (whoami != 0x68) {
        // handle error
    }
}



void MPU6000_Start_DMA(MPU6000 *dev) {
    // Prepare TX buffer: [reg|0x80, dummy...]
    dev->tx_buffer[0] = 0x3B | 0x80; // ACCEL_XOUT_H
    for (int i = 1; i < 15; i++) {
        dev->tx_buffer[i] = 0xFF; // dummy bytes
    }

    // Pull CS low before starting transfer
		HAL_GPIO_WritePin(MPU6000_CS_PORT, MPU6000_CS_PIN, GPIO_PIN_RESET);

    if (HAL_SPI_TransmitReceive_DMA(dev->hspi, dev->tx_buffer, dev->dma_buffer, 15) != HAL_OK) {
        HAL_GPIO_WritePin(MPU6000_CS_PORT, MPU6000_CS_PIN, GPIO_PIN_SET);
    }
}

void MPU6000_Read_Blocking(MPU6000 *dev, uint8_t *spi_rx_buffer) {
    // 1. Prepare TX buffer: [register address, dummy bytes...]
    dev->tx_buffer[0] = 0x3B | 0x80; // ACCEL_XOUT_H | Read bit (0x80)
    for (int i = 1; i < 15; i++) {
        dev->tx_buffer[i] = 0xFF; // dummy bytes for clocking out data
    }

    // 2. Pull CS low before starting transfer
    HAL_GPIO_WritePin(MPU6000_CS_PORT, MPU6000_CS_PIN, GPIO_PIN_RESET);

    // 3. Blocking SPI Transfer (Replaces DMA)
    // The task will block (halt) here until the transfer of 15 bytes is complete.
    if (HAL_SPI_TransmitReceive(dev->hspi,
                                dev->tx_buffer,
                                spi_rx_buffer,
                                15,
                                HAL_MAX_DELAY) != HAL_OK)
    {
        // Handle SPI error here (e.g., logging, infinite loop, or return)
    }

    // 4. Pull CS high after transfer is complete
    HAL_GPIO_WritePin(MPU6000_CS_PORT, MPU6000_CS_PIN, GPIO_PIN_SET);
}


void MPU6000_Process_DMA(MPU6000 *dev) {
    int16_t raw_acc_x = (dev->dma_buffer[1] << 8) | dev->dma_buffer[2];
    int16_t raw_acc_y = (dev->dma_buffer[3] << 8) | dev->dma_buffer[4];
    int16_t raw_acc_z = (dev->dma_buffer[5] << 8) | dev->dma_buffer[6];

    int16_t raw_temp  = (dev->dma_buffer[7] << 8) | dev->dma_buffer[8];

    int16_t raw_gyro_x = (dev->dma_buffer[9] << 8) | dev->dma_buffer[10];
    int16_t raw_gyro_y = (dev->dma_buffer[11] << 8) | dev->dma_buffer[12];
    int16_t raw_gyro_z = (dev->dma_buffer[13] << 8) | dev->dma_buffer[14];

    dev->acc[0] = (float)raw_acc_x / ACCEL_SCALE;
    dev->acc[1] = -(float)raw_acc_y / ACCEL_SCALE;
    dev->acc[2] = (float)raw_acc_z / ACCEL_SCALE;

    dev->temp = ((float)raw_temp) / 340.0f + 36.53f;

    dev->gyro[0] = -(float)raw_gyro_x / GYRO_SCALE;
    dev->gyro[1] = (float)raw_gyro_y / GYRO_SCALE;
    dev->gyro[2] = -(float)raw_gyro_z / GYRO_SCALE;

    if (dev->calibrated){
        for (uint8_t i = 0; i < 3; i++){
        	dev->acc[i] -= dev->acc_offset[i];
        	dev->gyro[i] -= dev->gyro_offset[i];
        }
    }
}

void MPU6000_Process(MPU6000 *dev, uint8_t* spi_rx_buffer) {
    int16_t raw_acc_x = (spi_rx_buffer[1] << 8) | spi_rx_buffer[2];
    int16_t raw_acc_y = (spi_rx_buffer[3] << 8) | spi_rx_buffer[4];
    int16_t raw_acc_z = (spi_rx_buffer[5] << 8) | spi_rx_buffer[6];

    int16_t raw_temp  = (spi_rx_buffer[7] << 8) | spi_rx_buffer[8];

    int16_t raw_gyro_x = (spi_rx_buffer[9] << 8) | spi_rx_buffer[10];
    int16_t raw_gyro_y = (spi_rx_buffer[11] << 8) | spi_rx_buffer[12];
    int16_t raw_gyro_z = (spi_rx_buffer[13] << 8) | spi_rx_buffer[14];

    dev->acc[0] = (float)raw_acc_x / ACCEL_SCALE;
    dev->acc[1] = -(float)raw_acc_y / ACCEL_SCALE;
    dev->acc[2] = (float)raw_acc_z / ACCEL_SCALE;

    dev->temp = ((float)raw_temp) / 340.0f + 36.53f;

    dev->gyro[0] = -(float)raw_gyro_x / GYRO_SCALE;
    dev->gyro[1] = (float)raw_gyro_y / GYRO_SCALE;
    dev->gyro[2] = -(float)raw_gyro_z / GYRO_SCALE;

}

void MPU6000_Calibrate(MPU6000 *dev) {
    const uint16_t samples = 1000;
    float acc_sum[3] = {0}, gyro_sum[3] = {0};

    dev->calibrated = false;

    uint8_t spi_rx_buffer[8];

    for (uint16_t i = 0; i < samples; i++) {
        // Start a DMA read
        MPU6000_Read_Blocking(dev, spi_rx_buffer);
        // Process latest sample
        MPU6000_Process(dev, spi_rx_buffer);

        for (uint8_t axis = 0; axis < 3; axis++) {
            acc_sum[axis]  += dev->acc[axis];
            gyro_sum[axis] += dev->gyro[axis];

        }

        HAL_Delay(1); // ~1 kHz sample rate
    }

    for (uint8_t axis = 0; axis < 3; axis++) {
        dev->acc_offset[axis]  = acc_sum[axis] / samples;
        dev->gyro_offset[axis] = gyro_sum[axis] / samples;
    }

    // Subtract gravity on Z axis
    dev->acc_offset[2] -= 1;
    dev->state = 0;
    dev->calibrated = true;
}



