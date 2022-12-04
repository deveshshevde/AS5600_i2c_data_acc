/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************/

#include "main.h"

#include "string.h"
#include "stdio.h"


#define I2C_TIMEOUT_BASE 10
#define I2C_TIMEOUT_BYTE 1

#define AS5600_RAW_ADDR 0X36

#define AS5600_ADDR (AS5600_RAW_ADDR << 1)

#define ZMCO		0x00
#define ZPOS_H		0x01
#define ZPOS_L		0x02
#define MPOS_H		0x03
#define MPOS_L		0x04
#define MANG_H		0x05
#define MANG_L		0x06
#define CONF_L		0x07
#define CONF_H		0x08
#define RAWANG_L	0x0D
#define ANGLE_H		0x0E
#define ANGLE_L		0x0F

#define BURN		0xFF


#define AS5600_RESOLUTION 4096//12bit/lines resolution
#define RAWANG_H 0X0C

#define abs(x) ((x)>0?(x):(-x))
#define _2PI 6.28318530718
int z=0;
float y=0;

I2C_HandleTypeDef hi2c1;


static double angle_data_prev;

static double full_rotation_offset;

void SystemClock_Config(void);

static void MX_GPIO_Init(void);

static void MX_I2C1_Init(void);


extern void initialise_monitor_handles();

void as5600_init(void);

uint16_t as5600GetRawAngle(void);

double as5600GetAngle(void);

void as5600_init(void) {
    full_rotation_offset = 0;
    angle_data_prev = as5600GetAngle();
}

static int i2cWrite(uint8_t dev_addr, uint8_t *pData, uint32_t count) {
    int status;
    int i2c_timeout = I2C_TIMEOUT_BASE + count * I2C_TIMEOUT_BYTE;

    status = HAL_I2C_Master_Transmit(&hi2c1, dev_addr, pData, count, i2c_timeout);
    return status;
}
static int i2cRead(uint8_t dev_addr, uint8_t *pData, uint32_t count) {
    int status;
    int i2c_timeout = I2C_TIMEOUT_BASE + count * I2C_TIMEOUT_BYTE;
    status = HAL_I2C_Master_Receive(&hi2c1, (dev_addr | 1), pData, count, i2c_timeout);
    return status;
}

uint16_t as5600GetRawAngle(void) {
    uint16_t raw_angle;
    uint8_t buffer[2] = {0};
    uint8_t raw_angle_reg = RAWANG_H;

    i2cWrite(AS5600_ADDR, &raw_angle_reg, 1);
    i2cRead(AS5600_ADDR, buffer, 2);
    raw_angle = ((uint16_t) buffer[0] << 8) | (uint16_t) buffer[1];
    return raw_angle;

}

double as5600GetAngle(void) {
    double angle_data = as5600GetRawAngle();

    double d_angle = angle_data - angle_data_prev;
    if (abs(d_angle) > (0.8 * AS5600_RESOLUTION)) {
        full_rotation_offset += (double) (d_angle > 0 ? -_2PI : _2PI);
    }
    angle_data_prev = angle_data;
    return (full_rotation_offset + (angle_data / AS5600_RESOLUTION) * _2PI);
}



int main(void)
{

	    initialise_monitor_handles();
	    HAL_Init();
	    SystemClock_Config();
	    MX_GPIO_Init();
	    MX_I2C1_Init();
	    as5600_init();
	    while (1) {
	        uint16_t angle_raw = as5600GetRawAngle();
	        double angle = as5600GetAngle();
	        HAL_Delay(100);

	         z = (int) angle;


	    }



}


void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

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

#ifdef  USE_FULL_ASSERT
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
