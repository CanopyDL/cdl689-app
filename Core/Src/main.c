/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <string.h>
#include "Modbus.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

typedef enum
{
	/* State machine's initial state. */
	IMU_STATE_INIT=0,
    IMU_STATE_IDLE,
	IMU_STATE_WAITING,
	IMU_STATE_READ_REG,
	IMU_STATE_READ_REG_COMPLETE,
	IMU_STATE_WRITE_REG,
	IMU_STATE_WRITE_REG_COMPLETE
} IMU_STATES;

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for imuDataTask */
osThreadId_t imuDataTaskHandle;
const osThreadAttr_t imuDataTask_attributes = {
  .name = "imuDataTask",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 128 * 4
};
/* Definitions for streamTimer */
osTimerId_t streamTimerHandle;
const osTimerAttr_t streamTimer_attributes = {
  .name = "streamTimer"
};
/* Definitions for utilTimer */
osTimerId_t utilTimerHandle;
const osTimerAttr_t utilTimer_attributes = {
  .name = "utilTimer"
};
/* USER CODE BEGIN PV */
bool spiBusy = false;
volatile bool sixPacketReady = false;
volatile int bytesToRead;
volatile int packetsStreamed = 0;
uint16_t streamTimerPeriod_uS = 1000;
uint8_t spiTxBuffer[32];
uint8_t spiRxBuffer[32];
modbusHandler_t ModbusH;
uint16_t ModbusData[128];
uint8_t uartData[16];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
void StartDefaultTask(void *argument);
void startImuDataTask(void *argument);
void streamTimerCallback(void *argument);
void utilTimerCallback(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * Function to perform jump to system memory boot from user application
 *
 * Call function when you want to jump to system memory
 */
void JumpToBootloader(void) {
	void (*SysMemBootJump)(void);

	/**
	 * Step: Set system memory address.
	 *
	 *       For STM32F401C, system memory is on 0x1FFF 0000
	 *       For other families, check AN2606 document
	 */
	volatile uint32_t addr = 0x1FFF0000;

	/**
	 * Step: Disable RCC, set it to default (after reset) settings
	 *       Internal clock, no PLL, etc.
	 */
#if defined(USE_HAL_DRIVER)
	HAL_RCC_DeInit();
#endif /* defined(USE_HAL_DRIVER) */
#if defined(USE_STDPERIPH_DRIVER)
	RCC_DeInit();
#endif /* defined(USE_STDPERIPH_DRIVER) */

	/**
	 * Step: Disable systick timer and reset it to default values
	 */
	SysTick->CTRL = 0;
	SysTick->LOAD = 0;
	SysTick->VAL = 0;

	/**
	 * Step: Disable all interrupts
	 */
	__disable_irq();

	/**
	 * Step: Remap system memory to address 0x0000 0000 in address space
	 *       For each family registers may be different.
	 *       Check reference manual for each family.
	 *
	 *       For STM32F4xx, MEMRMP register in SYSCFG is used (bits[1:0])
	 *       For STM32F0xx, CFGR1 register in SYSCFG is used (bits[1:0])
	 *       For others, check family reference manual
	 */
	//Remap by hand... {
	  NVIC_SystemReset();

#if defined(STM32F4)
	SYSCFG->MEMRMP = 0x01;
#endif
#if defined(STM32F0)
	SYSCFG->CFGR1 = 0x01;
#endif
	//} ...or if you use HAL drivers
	//__HAL_SYSCFG_REMAPMEMORY_SYSTEMFLASH();	//Call HAL macro to do this for you

	/**
	 * Step: Set jump memory location for system memory
	 *       Use address with 4 bytes offset which specifies jump location where program starts
	 */
	SysMemBootJump = (void (*)(void)) (*((uint32_t *)(addr + 4)));

	/**
	 * Step: Set main stack pointer.
	 *       This step must be done last otherwise local variables in this function
	 *       don't have proper value since stack pointer is located on different position
	 *
	 *       Set direct address location which specifies stack pointer in SRAM location
	 */
	__set_MSP(*(uint32_t *)addr);

	/**
	 * Step: Actually call our function to jump to set location
	 *       This will start system memory execution
	 */
	SysMemBootJump();

	/**
	 * Step: Connect USB<->UART converter to dedicated USART pins and test
	 *       and test with bootloader works with STM32 Flash Loader Demonstrator software
	 */
}

#define TRIGGER_PIN (10)


static void VectorBase_Config(void)
{
  /* The constant array with vectors of the vector table is declared externally in the
   * c-startup code.
   */
  extern const unsigned long g_pfnVectors[];

  /* Remap the vector table to where the vector table is located for this program. */
  SCB->VTOR = (unsigned long)&g_pfnVectors[0];
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
	VectorBase_Config();

  /* USER CODE BEGIN 1 */
	//look at the AUX pin.  If low for a certain number of samples, then enter boot loader mode.
	// Enable the GPIOA peripheral in 'RCC_AHBENR'.
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	GPIOA->PUPDR  |=  (0x1 << (TRIGGER_PIN*2));
	  long int bootCount = 0;
	  unsigned char bootTrigger = true;
	  for(bootCount = 0; bootCount < 100000; bootCount++){
		  uint32_t idr_val = GPIOA->IDR;
		  if (idr_val & (1 << TRIGGER_PIN)) {
			  bootTrigger = false;
		  }
	  }
	  if(bootTrigger){
		  JumpToBootloader();
	  }

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
  MX_SPI2_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* creation of streamTimer */
  streamTimerHandle = osTimerNew(streamTimerCallback, osTimerPeriodic, NULL, &streamTimer_attributes);

  /* creation of utilTimer */
  utilTimerHandle = osTimerNew(utilTimerCallback, osTimerPeriodic, NULL, &utilTimer_attributes);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of imuDataTask */
  imuDataTaskHandle = osThreadNew(startImuDataTask, NULL, &imuDataTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 84;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 10000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  huart2.Init.BaudRate = 460800;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_8;
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA1 PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT_TRIG_Pin */
  GPIO_InitStruct.Pin = BOOT_TRIG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BOOT_TRIG_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi){
	//Reset CS
	spiBusy = false;
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);	//CS back high
	if(bytesToRead > 0){
		bytesToRead -= 2;
		if(bytesToRead == 0){
			sixPacketReady = true;	//this is the last pair of bytes to read.  Time to send it out from uart.
			if(packetsStreamed < 256){
				packetsStreamed++;
			}
			else{
				packetsStreamed = 0;
				HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_9);	//every 256 packets we toggle the LED
			}
		}
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);	//set CS low.  It will be reset in the callback for TX complete.
		HAL_SPI_TransmitReceive_IT(&hspi2, spiTxBuffer + 24 - bytesToRead, spiRxBuffer + 24 - bytesToRead, 2);
		spiBusy = true;
	}
	else{
		//we've read all the bytes, now send them out from UART if in streaming mode
		if(sixPacketReady){
			//we got a streaming packet of data containing 6 axis values over SPI.  CRC
			//Send it out via UART.
			uartData[0] = spiRxBuffer[1];
			uartData[1] = spiRxBuffer[3];

			uartData[2] = spiRxBuffer[5];
			uartData[3] = spiRxBuffer[7];

			uartData[4] = spiRxBuffer[9];
			uartData[5] = spiRxBuffer[11];

			uartData[6] = spiRxBuffer[13];
			uartData[7] = spiRxBuffer[15];

			uartData[8] = spiRxBuffer[17];
			uartData[9] = spiRxBuffer[19];

			uartData[10] = spiRxBuffer[21];
			uartData[11] = spiRxBuffer[23];

			uartData[12] = 0x55;
			uartData[13] = 0xAA;

			// set RS485 transceiver to transmit mode
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
			HAL_UART_Transmit_IT(&huart2, uartData,  14);
			sixPacketReady = false;
		}

	}
}

////void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim){
//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
//	//if(!spiBusy){
//		//HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_6);
//	//}
//}


/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10	, GPIO_PIN_SET);	//CS back high
	osTimerStart(utilTimerHandle, pdMS_TO_TICKS(1000));
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_startImuDataTask */
/**
* @brief Function implementing the imuDataTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_startImuDataTask */
void startImuDataTask(void *argument)
{
  /* USER CODE BEGIN startImuDataTask */
	HAL_StatusTypeDef halStatus;
	IMU_STATES state = IMU_STATE_INIT;
	IMU_STATES nextState = IMU_STATE_IDLE;

  /* Infinite loop */
	for(;;){
		switch(state){
		case IMU_STATE_INIT:
			memset(ModbusData,0,sizeof(ModbusData));
			ModbusH.uiModbusType = SLAVE_RTU;
			ModbusH.port =  &huart2;
			ModbusH.u8id = 1; //slave ID
			ModbusH.u16timeOut = 1000;
			//ModbusH.EN_Port = NULL; // No RS485
			ModbusH.EN_Port = GPIOA; // RS485 Enable
			ModbusH.EN_Pin = GPIO_PIN_1; // RS485 Enable
			ModbusH.u32overTime = 0;
			ModbusH.au16regs = ModbusData;
			ModbusH.u8regsize= sizeof(ModbusData)/sizeof(ModbusData[0]);
			 //Initialize Modbus library
			ModbusInit(&ModbusH);
			//Start capturing traffic on serial Port
			ModbusStart(&ModbusH);
			state = IMU_STATE_IDLE;
			break;

		case IMU_STATE_IDLE:
			osSemaphoreAcquire(ModbusH.ModBusSphrHandle , portMAX_DELAY);
			if (ModbusData[1] != 0){
				//this is a request to read a register
				state = IMU_STATE_READ_REG;
			}
			else if(ModbusData[2] > 0){
				//this is a request to write a register
				state = IMU_STATE_WRITE_REG;

			}
			else if(ModbusData[10] == 0x69){
				ModbusData[10] = 0;
				spiTxBuffer[0] = 0x80 | 0x22;	//OUTX_L_G
				spiTxBuffer[1] = 0x0;
				spiTxBuffer[2] = 0x80 | 0x23;	//OUTX_H_G
				spiTxBuffer[3] = 0x0;
				spiTxBuffer[4] = 0x80 | 0x24;	//OUTY_L_G
				spiTxBuffer[5] = 0x0;
				spiTxBuffer[6] = 0x80 | 0x25;	//OUTY_H_G
				spiTxBuffer[7] = 0x0;
				spiTxBuffer[8] = 0x80 | 0x26;	//OUTZ_L_G
				spiTxBuffer[9] = 0x0;
				spiTxBuffer[10] = 0x80 | 0x27;	//OUTZ_H_G
				spiTxBuffer[11] = 0x0;

				spiTxBuffer[12] = 0x80 | 0x28;	//OUTX_L_A
				spiTxBuffer[13] = 0x0;
				spiTxBuffer[14] = 0x80 | 0x29;	//OUTX_H_A
				spiTxBuffer[15] = 0x0;
				spiTxBuffer[16] = 0x80 | 0x2A;	//OUTY_L_A
				spiTxBuffer[17] = 0x0;
				spiTxBuffer[18] = 0x80 | 0x2B;	//OUTY_H_A
				spiTxBuffer[19] = 0x0;
				spiTxBuffer[20] = 0x80 | 0x2C;	//OUTZ_L_A
				spiTxBuffer[21] = 0x0;
				spiTxBuffer[22] = 0x80 | 0x2D;	//OUTZ_H_A
				spiTxBuffer[23] = 0x0;

				//start streaming
				//ModbusEnd(&ModbusH);
				HAL_TIM_Base_Start_IT(&htim2);
			}
			else if(ModbusData[10] == 0x96){
				ModbusData[10] = 0;
				//stop streaming
				//HAL_TIM_PWM_Stop_IT(&htim2, TIM_CHANNEL_1);
				HAL_TIM_Base_Stop_IT(&htim2);
				//osTimerStop(streamTimerHandle);
			}
			else if(ModbusData[11] > 0){	//set the timer update rate
				htim2.Instance->ARR = ModbusData[11];
				//htim2.Instance->ARR = 400;
				htim2.Instance->CNT = 0;
				//htim2.Instance->CCR2 = ModbusData[11];
				ModbusData[11] = 0;
			}
			osSemaphoreRelease(ModbusH.ModBusSphrHandle);

			break;

		case IMU_STATE_WAITING:
			if(!spiBusy){
				state = nextState;
			}
			break;

		case IMU_STATE_READ_REG:
			spiTxBuffer[0] = 0x80 | ModbusData[1];	//read register
			spiTxBuffer[1] = 00;
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);	//set CS low.  It will be reset in the callback for TX complete.
			halStatus = HAL_SPI_TransmitReceive_IT(&hspi2, spiTxBuffer, spiRxBuffer, 2);
			spiBusy = true;
			state = IMU_STATE_WAITING;
			nextState = IMU_STATE_READ_REG_COMPLETE;
			break;

		case IMU_STATE_READ_REG_COMPLETE:
			osSemaphoreAcquire(ModbusH.ModBusSphrHandle , portMAX_DELAY);
			ModbusData[1] = 0;	//clear out the request
			ModbusData[3] = spiRxBuffer[1];
			state = nextState = IMU_STATE_IDLE;
			osSemaphoreRelease(ModbusH.ModBusSphrHandle);
			break;

		case IMU_STATE_WRITE_REG:
			spiTxBuffer[0] = ModbusData[2];	//write register
			spiTxBuffer[1] = ModbusData[3];
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);	//set CS low.  It will be reset in the callback for TX complete.
			halStatus = HAL_SPI_TransmitReceive_IT(&hspi2, spiTxBuffer, spiRxBuffer, 2);
			spiBusy = true;
			state = IMU_STATE_WAITING;
			nextState = IMU_STATE_WRITE_REG_COMPLETE;
			break;

		case IMU_STATE_WRITE_REG_COMPLETE:
			osSemaphoreAcquire(ModbusH.ModBusSphrHandle , portMAX_DELAY);
			ModbusData[2] = 0;	//clear out the request
			ModbusData[3] = spiRxBuffer[1];
			state = nextState = IMU_STATE_IDLE;
			osSemaphoreRelease(ModbusH.ModBusSphrHandle);
			break;

		default:
			break;
		}

		osDelay(1);
	}
  /* USER CODE END startImuDataTask */
}

/* streamTimerCallback function */
void streamTimerCallback(void *argument)
{
  /* USER CODE BEGIN streamTimerCallback */

  /* USER CODE END streamTimerCallback */
}

/* utilTimerCallback function */
void utilTimerCallback(void *argument)
{
  /* USER CODE BEGIN utilTimerCallback */
	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_8);
//	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
//	uint8_t tb[8] = "test\r\n";
//	HAL_UART_Transmit_IT(&huart2, tb, 6);
  /* USER CODE END utilTimerCallback */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  else if(htim->Instance == TIM2){	//streaming timer
	bytesToRead = 24;
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET); //set CS low.  It will be reset in the callback for TX complete.
	HAL_SPI_TransmitReceive_IT(&hspi2, spiTxBuffer + 24 - bytesToRead, spiRxBuffer + 24 - bytesToRead, 2);	//kick off two bytes of SPI RX/TX.
	spiBusy = true;
  }
  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
