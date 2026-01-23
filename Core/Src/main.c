/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

COM_InitTypeDef BspCOMInit;
I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

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
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Initialize led */
  BSP_LED_Init(LED_GREEN);

  /* Initialize USER push-button, will be used to trigger an interrupt each time it's pressed.*/
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

  /* Initialize COM1 port (115200, 8 bits (7-bit data + 1 stop bit), no parity */
  BspCOMInit.BaudRate   = 115200;
  BspCOMInit.WordLength = COM_WORDLENGTH_8B;
  BspCOMInit.StopBits   = COM_STOPBITS_1;
  BspCOMInit.Parity     = COM_PARITY_NONE;
  BspCOMInit.HwFlowCtl  = COM_HWCONTROL_NONE;
  if (BSP_COM_Init(COM1, &BspCOMInit) != BSP_ERROR_NONE)
  {
    Error_Handler();
  }

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint16_t adrRead = 0b11010101;
  uint16_t adrWrite = 0b11010100;
  uint8_t regTest = 0x1E;
  uint16_t regGyro[] = {0x22, 0x23, 0x24, 0x25, 0x26, 0x27};
  uint16_t regAccel[] = {0x28, 0x29, 0x2A, 0x2B, 0x2C, 0x2D};
  uint16_t regEnable[] = {0x10, 0x11, 0x15, 0x16, 0x17, 0x02, 0x12};
  uint8_t readreg;
  uint8_t readregAccel;
  uint8_t readregGyro;
  uint8_t accelValues[6];
  uint8_t gyroValues[6];

  //Register values
  uint8_t value0 = 0b00000000;
  uint8_t value1 = 0b00000001;
  uint8_t value3 = 0b00000011;
  uint8_t value4 = 0b00000100;
  uint8_t value7 = 0b00000111;
  uint8_t value9 = 0b00001001;


  //Enable relevant sensors
  HAL_I2C_Mem_Write(&hi2c1, adrWrite, regEnable[7], 2, &value1, 1, 500);
  HAL_I2C_Mem_Write(&hi2c1, adrWrite, regEnable[0], 2, &value7, 1, 500);
  HAL_I2C_Mem_Write(&hi2c1, adrWrite, regEnable[1], 2, &value7, 1, 500);
  HAL_I2C_Mem_Write(&hi2c1, adrWrite, regEnable[3], 2, &value9, 1, 500);
  HAL_I2C_Mem_Write(&hi2c1, adrWrite, regEnable[6], 2, &value3, 1, 500);
  HAL_I2C_Mem_Write(&hi2c1, adrWrite, regEnable[7], 2, &value4, 1, 500);

  //  //Enable filters
  HAL_I2C_Mem_Write(&hi2c1, adrWrite, regEnable[4], 2, &value1, 1, 500);
  HAL_I2C_Mem_Write(&hi2c1, adrWrite, regEnable[5], 2, &value0, 1, 500);


  while (1)
  {

	  //Read accelerometers
	 /* for (uint8_t k = 0; k <= 6; k++) {
		  HAL_I2C_Mem_Read(&hi2c1, adrRead, regAccel[k], 1, &readregAccel, 1, 1000);
		  accelValues[k] = readregAccel;
	  } */
	  HAL_I2C_Mem_Read(&hi2c1, adrRead, regGyro[0], 2, &readregGyro, 1, 1000);
	  //Read gyro
	  /*for (uint8_t l = 0; l <= 4; l=l+2) {
		  HAL_I2C_Mem_Read(&hi2c1, adrRead, regGyro[l], 1, &readregGyro, 1, 1000);
		  printf("this bit is %i\n\r", readregGyro);
		  gyroValues[l] = readregGyro;
		  HAL_I2C_Mem_Read(&hi2c1, adrRead, regGyro[l+1], 1, &readregGyro, 1, 1000);
		  gyroValues[l+1] = readregGyro;
		  int16_t gyro_x = (int16_t)(gyroValues[1] << 8) | gyroValues[0];
		  printf("the gyro values are %i\n\r", gyro_x);
	  }*/


	  /*HAL_I2C_Mem_Read(&hi2c1, adrRead, 0x22, 1, &testbuffer, 12, 1000);
	  int16_t gyro_x = (int16_t)(testbuffer[1] << 8) | testbuffer[0];
	  gyro_x = gyro_x * 0.061f / 1000.0f;
	  gyro_x = gyro_x * 3.1415f / 180.0f;
	  printf("the gyro x is %i\n\r", gyro_x);*/

	  /*float accel_ms2_per_lsb = accel_g_per_lsb * 9.80665f;
	  float gyro_rads_per_lsb = gyro_dps_per_lsb * 0.01745329252f;

	  float ax_ms2 = accel_x * accel_ms2_per_lsb;
	  float gx_rads = gyro_x  * gyro_rads_per_lsb;*/

	  HAL_I2C_Mem_Read(&hi2c1, adrRead, regTest, 2, &readreg, 1, 1000);

	  printf("The value is %x\n\r", readreg);
	  printf("the gyro is %x\n\r", readregGyro);

	  readreg = 0;

	  HAL_Delay(500);

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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
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
  hi2c1.Init.Timing = 0x40B285C2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
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
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

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
