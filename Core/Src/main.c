/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mpu6000.h"
#include <stdio.h>
#include <stdint.h>
#include "LowPassFilter.h"
#include "NotchFilter.h"
#include "QMC5883L.h"
#include "matrix3.h"
#include "EKF.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define IMU_SAMPLE_RATE_HZ 200.0f  // Tuong ?ng v?i chu k? 5ms
#define ACC_CUTOFF_HZ      20.0f   // L?c rung d?ng khung
#define GYRO_CUTOFF_HZ     80.0f   // L?c nhi?u cao t?n
#define NOTCH_CENTER_HZ    180.0f      
#define NOTCH_BANDWIDTH    40.0f       
#define NOTCH_ATTENUATION  40.0f     

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;

TIM_HandleTypeDef htim5;

/* USER CODE BEGIN PV */
MPU6000 mpu;
LowPassFilter2pFloat acc_filter[3];
LowPassFilter2pFloat gyro_filter[3];
LowPassFilter2pFloat mag_filter[3];

NotchFilter_t acc_notch[3];
NotchFilter_t gyro_notch[3];
QMC5883L_t mag;
EKF_Handle_t ekf;

uint8_t found;
float ax, ay, az;
float gx, gy, gz;
float mx, my, mz;
float dt = 0.0f;
float N,E,D;
float roll, pitch, yaw;
float ekf_roll = 0.0f;
float ekf_pitch = 0.0f;
float ekf_yaw = 0.0f;
float mag_offset_x = 0.219831139f;
float mag_offset_y = 0.0419995785f;
float mag_offset_z = -0.261330724f;
float vn, ve, vd;
float pn, pe, pd;
float dt_real;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM5_Init(void);
/* USER CODE BEGIN PFP */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
	if (hspi->Instance == SPI1)
	{
		HAL_GPIO_WritePin(MPU6000_CS_PORT, MPU6000_CS_PIN, GPIO_PIN_SET);
		MPU6000_Process_DMA(&mpu);
		mpu.data_ready = true;
	}
}

uint32_t micros(void)
{
	return __HAL_TIM_GET_COUNTER(&htim5);
}


/* USER CODE END PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_I2C1_Init();
  MX_TIM5_Init();
	HAL_TIM_Base_Start(&htim5);
  /* USER CODE BEGIN 2 */
	MPU6000_Init(&mpu, &hspi1);
	HAL_Delay(100);
	MPU6000_Calibrate(&mpu);
	HAL_Delay(20);
	QMC5883L_Init(&mag, &hi2c1);
	for (int i = 0; i < 3; i++) {
        LowPassFilter2pFloat_init(&acc_filter[i],  IMU_SAMPLE_RATE_HZ, ACC_CUTOFF_HZ);
        LowPassFilter2pFloat_init(&gyro_filter[i], IMU_SAMPLE_RATE_HZ, GYRO_CUTOFF_HZ);
  }
	for(int i = 0; i < 3; i++)
	{
			NotchFilter_init(&acc_notch[i],
											 IMU_SAMPLE_RATE_HZ,
											 NOTCH_CENTER_HZ,
											 NOTCH_BANDWIDTH,
											 NOTCH_ATTENUATION);

			NotchFilter_init(&gyro_notch[i],
											 IMU_SAMPLE_RATE_HZ,
											 NOTCH_CENTER_HZ,
											 NOTCH_BANDWIDTH,
											 NOTCH_ATTENUATION);
	}
	EKF_Init(&ekf, 0.005f);
	Vector3f mag_sum = {0, 0, 0};
	int sample_count = 0;
	for (int i = 0; i < 100; i++)
	{
		if (QMC5883L_ReadMag(&mag)) {
        float mx_cal = mag.mag_x - mag_offset_x;
        float my_cal = mag.mag_y - mag_offset_y;
        float mz_cal = mag.mag_z - mag_offset_z;
        
        mag_sum.x += mx_cal;
        mag_sum.y += my_cal;
        mag_sum.z += mz_cal;
      }
		HAL_Delay(5);
	}
	if (sample_count > 0)
	{
		mag_sum.x /= sample_count;
		mag_sum.y /= sample_count;
		mag_sum.z /= sample_count;
		float norm = sqrtf(mag_sum.x*mag_sum.x + mag_sum.y*mag_sum.y + mag_sum.z*mag_sum.z);
      if (norm > 1e-4f) {
          Vector3f mag_ref_vec = {mag_sum.x/norm, mag_sum.y/norm, mag_sum.z/norm};
          EKF_SetMagReference(&ekf, mag_ref_vec);
      }
	}
	
	uint32_t last_mag_time_us = micros();
  uint32_t last_imu_trigger_us = micros();
	uint32_t last_ekf_process_us = micros();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		uint32_t now_us = micros();

    if (now_us - last_mag_time_us >= 10000)
    {
        if (QMC5883L_ReadMag(&mag)) 
        {
            mx = mag.mag_x - mag_offset_x;
            my = mag.mag_y - mag_offset_y;
            mz = mag.mag_z - mag_offset_z;
            float m_norm = sqrtf(mx*mx + my*my + mz*mz);
            if (m_norm > 1e-4f) {
                float inv_norm = 1.0f / m_norm;
                EKF_FuseMag(&ekf, (Vector3f){mx * inv_norm, my * inv_norm, mz * inv_norm});
            }
        }
        last_mag_time_us = now_us;
    }

    if (now_us - last_imu_trigger_us >= 5000)
    {
        MPU6000_Start_DMA(&mpu);
        last_imu_trigger_us = now_us;
    }

    if (mpu.data_ready)
    {
        mpu.data_ready = false;

        uint32_t current_ekf_us = micros();
        
        dt_real = (float)(current_ekf_us - last_ekf_process_us) * 0.000001f;
        
        last_ekf_process_us = current_ekf_us;

        if (dt_real <= 0.0f) dt_real = 0.005f;     
        if (dt_real > 0.02f) dt_real = 0.02f;       

        ekf.dt = dt_real;

        ax = NotchFilter_apply(&acc_notch[0], LowPassFilter2pFloat_apply(&acc_filter[0], mpu.acc[1]));
        ay = NotchFilter_apply(&acc_notch[1], LowPassFilter2pFloat_apply(&acc_filter[1], mpu.acc[0]));
        az = NotchFilter_apply(&acc_notch[2], LowPassFilter2pFloat_apply(&acc_filter[2], mpu.acc[2]));

        gx = NotchFilter_apply(&gyro_notch[0], LowPassFilter2pFloat_apply(&gyro_filter[0], mpu.gyro[1]));
        gy = NotchFilter_apply(&gyro_notch[1], LowPassFilter2pFloat_apply(&gyro_filter[1], mpu.gyro[0]));
        gz = NotchFilter_apply(&gyro_notch[2], LowPassFilter2pFloat_apply(&gyro_filter[2], mpu.gyro[2]));
			
				float ax_mss = ax * GRAVITY_MSS;
        float ay_mss = ay * GRAVITY_MSS;
        float az_mss = az * GRAVITY_MSS;

        float gx_rad = gx * DEG_TO_RAD;
        float gy_rad = gy * DEG_TO_RAD;
        float gz_rad = gz * DEG_TO_RAD;

				EKF_Predict(&ekf,(Vector3f){gx_rad, gy_rad, gz_rad}, (Vector3f){ax_mss, ay_mss, az_mss});
				EKF_FuseAccel(&ekf, (Vector3f){ax_mss, ay_mss, az_mss}); 
        
        EKF_GetEuler(&ekf, &roll, &pitch, &yaw);

				vn = ekf.vn;
				ve = ekf.ve;
				vd = ekf.vd;
				
				pn = ekf.pn;
				pe = ekf.pe;
				pd = ekf.pd;
				
				
    }	
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 83;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 4294967295;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
