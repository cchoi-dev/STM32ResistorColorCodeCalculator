/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include "st7735.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// Joystick direction definitions
#define jsNeutral 0
#define jsPress 1
#define jsUp 2
#define jsDown 3
#define jsRight 4
#define jsLeft 5
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;

osThreadId defaultTaskHandle;
osThreadId joystickTaskHandle;
osThreadId displayTaskHandle;
osMessageQId joystickDisplayQueueHandle;
osMutexId uart2MutexHandle;
osMutexId spiMutexHandle;
/* USER CODE BEGIN PV */
// LED Blinking PVs
int pinDelay[4] = {50, 100, 300, 600};
volatile uint8_t pinDelayIndex = 0;
// Joystick PVs
uint8_t jsHeld = 0; // 0 - false, 1 - true

// Logic PVs
uint8_t pointerIndex = 0; // inddex for pointer for resistor color selection
uint8_t bandIndex = 0; // index for resistor band selection
enum logicState {IDLE, SEL_FIRST, SEL_SECOND, SEL_THIRD, SEL_FOURTH, CHECK_CALC, CALC};
enum logicState currState = IDLE; // FSM state for resistor color code selection

// Display PVs
const uint16_t backgroundColor = 0x9BCE;
const uint16_t pointerColor = 0x001F;
const uint16_t resistorColor = 0xCF9F;
const uint16_t resistorBandColors[] = {0x0000, 0x3374, 0x001F, 0x1CDF, 0x1F9F,
                                       0x1FEC, 0xFAE3, 0xFC75, 0x9C92, 0xFFDF};
const uint8_t bandIndices[] = {46, 63, 80, 106};
uint8_t selectedValues[] = {0xFF, 0xFF, 0xFF, 0xFF}; // Values defaulted to 0xFF
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI1_Init(void);
void StartDefaultTask(void const * argument);
void startJoystickTask(void const * argument);
void startDisplayTask(void const * argument);

/* USER CODE BEGIN PFP */
uint8_t checkJoystick();
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
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  // Keep slave select high for now
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
  //ST7735Initialize();
  /* USER CODE END 2 */

  /* Create the mutex(es) */
  /* definition and creation of uart2Mutex */
  osMutexDef(uart2Mutex);
  uart2MutexHandle = osMutexCreate(osMutex(uart2Mutex));

  /* definition and creation of spiMutex */
  osMutexDef(spiMutex);
  spiMutexHandle = osMutexCreate(osMutex(spiMutex));

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of joystickDisplayQueue */
  osMessageQDef(joystickDisplayQueue, 16, uint16_t);
  joystickDisplayQueueHandle = osMessageCreate(osMessageQ(joystickDisplayQueue), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of joystickTask */
  osThreadDef(joystickTask, startJoystickTask, osPriorityNormal, 0, 128);
  joystickTaskHandle = osThreadCreate(osThread(joystickTask), NULL);

  /* definition and creation of displayTask */
  osThreadDef(displayTask, startDisplayTask, osPriorityNormal, 0, 128);
  displayTaskHandle = osThreadCreate(osThread(displayTask), NULL);

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */
  
  /* USER CODE END ADC1_Init 2 */

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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  if (pinDelayIndex < sizeof(pinDelay)/sizeof(pinDelay[0]) - 1) {
    pinDelayIndex++;
  } else {
    pinDelayIndex = 0;
  }
}

uint8_t checkJoystick() {
  HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
  int val = HAL_ADC_GetValue(&hadc1);
  val = val >> 2; // Reduce from 12-bit res to 10-bit
  if (val < 50) {
    return jsLeft;
  } else if (val < 200) {
    return jsDown;
  } else if (val < 400) {
    return jsPress;
  } else if (val < 600) {
    return jsRight;
  } else if (val < 950) {
    return jsUp;
  }
  return jsNeutral;
}

void drawPointer(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint16_t color, uint16_t bgColor) {
  uint8_t sendDataColor[] = {color >> 8, color & 0xFF};
  uint8_t sendDataBgColor[] = {bgColor >> 8, bgColor & 0xFF};
  ST7735Select();
  setAddrWindow(x, y, w, h);
  for (int j = 0; j < h; j++) {
    for (int i = 0; i < w; i++) {
      if ( i <= ((w / 2) + j) && i >= ((w / 2) - j)) {
        ST7735WriteData(sendDataColor, sizeof(sendDataColor));
      } else {
        ST7735WriteData(sendDataBgColor, sizeof(sendDataBgColor));
      }
    }
  }  
  ST7735Deselect();
}

void drawResistor() {
  drawRectangle(0, 50, 20, 10, 0xF79E);
  drawRectangle(ST7735_TFTWIDTH - 20, 50, 20, 10, 0xF79E);
  drawRectangle(20, 45, ST7735_TFTWIDTH - 40, 20, resistorColor);
  drawRectangle(22, 43, 40, 2, resistorColor);
  drawRectangle(98, 43, 40, 2, resistorColor);
  drawRectangle(22, 65, 40, 2, resistorColor);
  drawRectangle(98, 65, 40, 2, resistorColor);
  drawRectangle(28, 41, 24, 2, resistorColor);
  drawRectangle(108, 41, 24, 2, resistorColor);
  drawRectangle(28, 67, 24, 2, resistorColor);
  drawRectangle(108, 67, 24, 2, resistorColor);
}

void drawFirstBand(uint16_t color) {
  drawRectangle(46, 41, 6, 28, color);
  drawRectangle(52, 43, 4, 24, color);
}

void drawSecondBand(uint16_t color) {
  drawRectangle(63, 45, 10, 20, color);
}

void drawThirdBand(uint16_t color) {
  drawRectangle(80, 45, 10, 20, color);
}

void drawFourthBand(uint16_t color) {
  drawRectangle(106, 43, 2, 24, color);
  drawRectangle(108, 41, 8, 28, color);
}
  
void resistorGuiInit() {
  //{0x0000, 0x3374, 0x001F, 0x1CDF, 0x1F9F, 0x1FEC, 0xFAE3, 0xFC75, 0x9C92, 0xFFDF}
  // Draw resistor color codes
  drawRectangle(0, 0, 16, 10, resistorBandColors[0]); // Black
  drawRectangle(16, 0, 16, 10, resistorBandColors[1]); // Brown
  drawRectangle(32, 0, 16, 10, resistorBandColors[2]); // Red
  drawRectangle(48, 0, 16, 10, resistorBandColors[3]); // Orange
  drawRectangle(64, 0, 16, 10, resistorBandColors[4]); // Yellow
  drawRectangle(80, 0, 16, 10, resistorBandColors[5]); // Green
  drawRectangle(96, 0, 16, 10, resistorBandColors[6]); // Blue
  drawRectangle(112, 0, 16, 10, resistorBandColors[7]); // Violet
  drawRectangle(128, 0, 16, 10, resistorBandColors[8]); // Grey
  drawRectangle(144, 0, 16, 10, resistorBandColors[9]); // White
  drawPointer(0, 15, 15, 10, pointerColor, backgroundColor); // Color pointer
  drawPointer(46, 80, 10, 5, pointerColor, backgroundColor); // Band pointer
  // Draw resistor
  drawResistor();
}

// Note: char* buff must point to an array of at least size 11
void int2str(uint64_t num, char* buff) {
  char values[11];
  uint8_t valuesIndex = 0;
  uint8_t digit = num % 10;
  values[valuesIndex++] = "0123456789"[digit];
  num /= 10;
  while (num != 0) {
    digit = num % 10;
    values[valuesIndex++] = "0123456789"[digit];
    num /= 10;
  }
  
  while (valuesIndex > 0) {
    *buff = values[--valuesIndex];
    buff++;
  }
  return;
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{

  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */ 
}

/* USER CODE BEGIN Header_startJoystickTask */
/**
* @brief Function implementing the joystickTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_startJoystickTask */
void startJoystickTask(void const * argument)
{
  /* USER CODE BEGIN startJoystickTask */
  HAL_ADC_Start(&hadc1);
  /* Infinite loop */
  for(;;)
  {
    uint8_t jsCommand = checkJoystick();
    if (jsCommand != jsNeutral) {
      if (jsHeld == 0) {
        osMutexWait(uart2MutexHandle, 50);
        osMutexWait(spiMutexHandle, 50);
        switch(jsCommand) {
          case jsUp:
            HAL_UART_Transmit(&huart2, (uint8_t*)"Up\r\n", strlen("Up\r\n"), HAL_MAX_DELAY);
            if (bandIndex > 0) {
              drawRectangle(bandIndices[bandIndex--], 80, 10, 10, backgroundColor);
            } else {
              drawRectangle(bandIndices[bandIndex], 80, 10, 10, backgroundColor);
              bandIndex = 3;
            }
            drawPointer(bandIndices[bandIndex], 80, 10, 5, pointerColor, backgroundColor);
            break;
          case jsDown:
            HAL_UART_Transmit(&huart2, (uint8_t*)"Down\r\n", strlen("Down\r\n"), HAL_MAX_DELAY);
            if (bandIndex < 3) {
              drawRectangle(bandIndices[bandIndex++], 80, 10, 10, backgroundColor);
            } else {
              drawRectangle(bandIndices[bandIndex], 80, 10, 10, backgroundColor);
              bandIndex = 0;
            }
            drawPointer(bandIndices[bandIndex], 80, 10, 5, pointerColor, backgroundColor);
            break;
          case jsLeft:
            HAL_UART_Transmit(&huart2, (uint8_t*)"Left\r\n", strlen("Left\r\n"), HAL_MAX_DELAY);
            if (pointerIndex > 0) {
              drawRectangle(16 * pointerIndex--, 15, 15, 10, backgroundColor);
            } else {
              drawRectangle(16 * pointerIndex, 15, 15, 10, backgroundColor);
              pointerIndex = 9;
            }
            drawPointer(16 * pointerIndex, 15, 15, 10, pointerColor, backgroundColor);
            break;
          case jsRight:
            HAL_UART_Transmit(&huart2, (uint8_t*)"Right\r\n", strlen("Right\r\n"), HAL_MAX_DELAY);
            if (pointerIndex < 9) {
              drawRectangle(16 * pointerIndex++, 15, 15, 10, backgroundColor);
            } else {
              drawRectangle(16 * pointerIndex, 15, 15, 10, backgroundColor);
              pointerIndex = 0;
            }
            drawPointer(16 * pointerIndex, 15, 15, 10, pointerColor, backgroundColor);
            break;
          case jsPress:
            HAL_UART_Transmit(&huart2, (uint8_t*)"Press\r\n", strlen("Press\r\n"), HAL_MAX_DELAY);
            osMessagePut(joystickDisplayQueueHandle, 0xABCD, 100);
            break;
        }
        osMutexRelease(uart2MutexHandle);
        osMutexRelease(spiMutexHandle);
      }
      jsHeld = 1;
    } else {
      jsHeld = 0;
    }
    osDelay(10);
  }
  /* USER CODE END startJoystickTask */
}

/* USER CODE BEGIN Header_startDisplayTask */
/**
* @brief Function implementing the displayTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_startDisplayTask */
void startDisplayTask(void const * argument)
{
  /* USER CODE BEGIN startDisplayTask */
  const uint8_t txData[26] = "Hello from startSendUart\r\n";
  osMutexWait(uart2MutexHandle, 50);
  HAL_UART_Transmit(&huart2, (uint8_t*)txData, 26, HAL_MAX_DELAY);
  osMutexRelease(uart2MutexHandle);
  ST7735Initialize();
  drawCustomBackground(backgroundColor);
  resistorGuiInit();
  int ohms = 0;
  /* Infinite loop */
  for(;;)
  {
    switch(currState) {
      case IDLE:
        osMessageGet(joystickDisplayQueueHandle, osWaitForever);
        if (bandIndex == 0) {
          currState = SEL_FIRST;
        } else if (bandIndex == 1) {
          currState = SEL_SECOND;
        } else if (bandIndex == 2) {
          currState = SEL_THIRD;
        } else if (bandIndex == 3) {
          currState = SEL_FOURTH;
        }
        break;
      case SEL_FIRST:
        drawFirstBand(resistorBandColors[pointerIndex]);
        selectedValues[bandIndex] = pointerIndex;
        drawRectangle(bandIndices[bandIndex++], 80, 10, 10, backgroundColor);
        drawPointer(bandIndices[bandIndex], 80, 10, 5, pointerColor, backgroundColor);
        currState = CHECK_CALC;
        break;
      case SEL_SECOND:
        drawSecondBand(resistorBandColors[pointerIndex]);
        selectedValues[bandIndex] = pointerIndex;
        drawRectangle(bandIndices[bandIndex++], 80, 10, 10, backgroundColor);
        drawPointer(bandIndices[bandIndex], 80, 10, 5, pointerColor, backgroundColor);
        currState = CHECK_CALC;
        break;
      case SEL_THIRD:
        drawThirdBand(resistorBandColors[pointerIndex]);
        selectedValues[bandIndex] = pointerIndex;
        drawRectangle(bandIndices[bandIndex++], 80, 10, 10, backgroundColor);
        drawPointer(bandIndices[bandIndex], 80, 10, 5, pointerColor, backgroundColor);
        currState = CHECK_CALC;
        break;
      case SEL_FOURTH:
        drawFourthBand(resistorBandColors[pointerIndex]);
        selectedValues[bandIndex] = pointerIndex;
        drawRectangle(bandIndices[bandIndex], 80, 10, 10, backgroundColor);
        bandIndex = 0;
        drawPointer(bandIndices[bandIndex], 80, 10, 5, pointerColor, backgroundColor);
        currState = CHECK_CALC;
        break;
      case CHECK_CALC:
        if ((selectedValues[0]!= 0xFF) && (selectedValues[1] != 0xFF) && (selectedValues[2] != 0xFF)) {
          currState = CALC;
        } else {
          currState = IDLE;
        }
        break;
      case CALC:
        ohms = (selectedValues[0] * 10 + selectedValues[1]) * 10^(selectedValues[2]);
        if ((ohms / 10^9) > 0) { // Display G before ohms
					// Check if 2nd digit in 10^8's spot
					if ((ohms / 10^8) % 10 > 0) {
					} else {
					}
        } else if ((ohms / 10^6) > 0) { // Display M before Ohms
					if ((ohms /10^5) % 10 > 0) {
					} else {
					}
        } else if ((ohms / 10^3) > 0) { // Display k before Ohms
					if ((ohms /10^2) % 10 > 0) {
					} else {
					}
        } else {
        }
        currState = IDLE;
        break;
    }
    osDelay(50);
  }
  /* USER CODE END startDisplayTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM3 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM3) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

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
