/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

typedef enum MCP23017_RegistersBank0 {
  MCP23017_IODIRA = 0x00,
  MCP23017_IODIRB = 0x01,
  MCP23017_IPOLA = 0x02,
  MCP23017_IPOLB = 0x03,
  MCP23017_GPINTENA = 0x04,
  MCP23017_GPINTENB = 0x05,
  MCP23017_DEFVALA = 0x06,
  MCP23017_DEFVALB = 0x07,
  MCP23017_INTCONA = 0x08,
  MCP23017_INTCONB = 0x09,
  MCP23017_IOCON_AB = 0x0A,
  MCP23017_IOCON_BA = 0x0B,
  MCP23017_GPPUA = 0x0C,
  MCP23017_GPPUB = 0x0D,
  MCP23017_INTFA = 0x0E,
  MCP23017_INTFB = 0x0F,
  MCP23017_INTCAPA = 0x10,
  MCP23017_INTCAPB = 0x11,
  MCP23017_GPIOA = 0x12,
  MCP23017_GPIOB = 0x13,
  MCP23017_OLATA = 0x14,
  MCP23017_OLATB = 0x15
} MCP23017_RegistersBank0;

typedef enum MCP23017_IOCONSetup {
  MCP23017_CON_BANK =    0b10000000,
  MCP23017_CON_MIRROR =  0b01000000,
  MCP23017_CON_SEQOP =   0b00100000,
  MCP23017_CON_DISSLW =  0b00010000,
  MCP23017_CON_HAEN =    0b00001000,
  MCP23017_CON_ODR =     0b00000100,
  MCP23017_CON_INTPOL =  0b00000010,
} MCP23017_IOCONSetup;

uint8_t MCP23017_AddrConv(uint8_t addr){
  if (addr > 7){
    addr = 7;
  }
  uint8_t fixedPart = 0b0100;
  uint8_t filteredAddr = 0b0000111 & addr;
  uint8_t slaveAddr = fixedPart << 3 | filteredAddr;
  return slaveAddr;
}

uint8_t MCP23017_I2CRead(uint16_t addr, uint16_t reg){
	HAL_StatusTypeDef status;
	uint8_t data;
	status = HAL_I2C_Mem_Read(&hi2c1, addr << 1 | 0b00000001, reg, 1, &data, 1, 10);
 // Wire.beginTransmission(addr);
 // Wire.write(reg);
 // Wire.requestFrom(0x20, 1, true);
 // uint8_t data = Wire.read();
 // Wire.endTransmission();
 // sleep(5);
  //return data;
	//HAL_Delay(5);
	return data;
}

void MCP23017_I2CWrite(uint16_t addr, uint16_t reg, uint8_t data){
	HAL_StatusTypeDef status;
	status = HAL_I2C_Mem_Write(&hi2c1, addr << 1, reg, 1, &data, 1, 10);
	//HAL_I2C_Master_Transmit (&hi2c1, addr, &data, 1, 1000);
 // Wire.beginTransmission(addr);
 // Wire.write(reg);
 // Wire.write(data);
 // Wire.endTransmission();
 // sleep(50);
	HAL_Delay(50);
}

uint16_t MCP23017_readNotes(uint8_t addr){
  uint8_t AData = MCP23017_I2CRead(addr, MCP23017_GPIOA);
  uint8_t BData = MCP23017_I2CRead(addr, MCP23017_GPIOB);
  return BData << 8 | AData;
}

void MCP23017_init(uint8_t addr){
  MCP23017_I2CWrite(addr, MCP23017_IOCON_AB, MCP23017_CON_MIRROR | MCP23017_CON_ODR);
  //uint8_t config = MCP23017_I2CRead(_devAddr, MCP23017_IOCON_AB);

  MCP23017_I2CWrite(addr, MCP23017_IODIRA, 0b11111111); //I/O DIRECTION REGISTER (ADDR 0x00)
  MCP23017_I2CWrite(addr, MCP23017_IODIRB, 0b11111111); //I/O DIRECTION REGISTER (ADDR 0x00)
  MCP23017_I2CWrite(addr, MCP23017_GPPUA, 0b11111111); //GPIO PULL-UP RESISTOR REGISTER (ADDR 0x06)
  MCP23017_I2CWrite(addr, MCP23017_GPPUB, 0b11111111); //GPIO PULL-UP RESISTOR REGISTER (ADDR 0x06)
  MCP23017_I2CWrite(addr, MCP23017_GPINTENA, 0b11111111); //INTERRUPT-ON-CHANGE PINS (ADDR 0x02)
  MCP23017_I2CWrite(addr, MCP23017_GPINTENB, 0b11111111); //INTERRUPT-ON-CHANGE PINS (ADDR 0x02)
  MCP23017_I2CWrite(addr, MCP23017_INTCONA, 0b00000000); //INTERRUPT-ON-CHANGE CONTROL REGISTER (ADDR 0x04)
  MCP23017_I2CWrite(addr, MCP23017_INTCONB, 0b00000000); //INTERRUPT-ON-CHANGE CONTROL REGISTER (ADDR 0x04)
  MCP23017_I2CWrite(addr, MCP23017_DEFVALA, 0b11111111); //DEFAULT VALUE REGISTER (ADDR 0x03)
  MCP23017_I2CWrite(addr, MCP23017_DEFVALB, 0b11111111); //DEFAULT VALUE REGISTER (ADDR 0x03)

  //Serial.println("MCP23017 Initiated");
}

uint8_t octaveIntTriggered = 0;

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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  uint8_t devAddr = MCP23017_AddrConv(0);
  MCP23017_init(devAddr);

  while (1)
  {

	if (octaveIntTriggered == 1){
		octaveIntTriggered = 0;
		uint16_t notes = MCP23017_readNotes(devAddr);
		if (notes == 0xFFFF){
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
		}else{
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
		}
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

  if(GPIO_Pin == GPIO_PIN_13) {
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
  } else if (GPIO_Pin == GPIO_PIN_10) {
	octaveIntTriggered = 1;
	//Then, must read MCP23017 values, otherwise, no more interrupt triggered.
  } else {
    __NOP();
  }
}

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
