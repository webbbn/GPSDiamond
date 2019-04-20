/* USER CODE BEGIN Header */
/**
******************************************************************************
* @file           : main.c
* @brief          : Main program body
******************************************************************************
** This notice applies to any and all portions of this file
* that are not between comment pairs USER CODE BEGIN and
* USER CODE END. Other portions of this file, whether 
* inserted by the user or by software development tools
* are owned by their respective copyright owners.
*
* COPYRIGHT(c) 2019 STMicroelectronics
*
* Redistribution and use in source and binary forms, with or without modification,
* are permitted provided that the following conditions are met:
*   1. Redistributions of source code must retain the above copyright notice,
*      this list of conditions and the following disclaimer.
*   2. Redistributions in binary form must reproduce the above copyright notice,
*      this list of conditions and the following disclaimer in the documentation
*      and/or other materials provided with the distribution.
*   3. Neither the name of STMicroelectronics nor the names of its contributors
*      may be used to endorse or promote products derived from this software
*      without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
******************************************************************************
*/
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
	      NMEA_COMPLETE,
	      UBX_COMPLETE,
	      MSG_IN_PROCESS,
	      MSG_ERROR
} MSGStatus;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ERROR_MS56			(3)

#define MS5637_I2C_ADDR 	(0xEC)
#define MS56_CMD_RST		(0x1E)
#define MS56_CMD_PROM_READ	(0xA0)
#define MS56_CMD_CONV_P_256	(0x40)
#define MS56_CMD_CONV_T_256	(0x50)
#define MS56_CMD_ADC_READ	(0x00)
#define MS56_PROM_SIZE		(7)

// Oversample rates
#define MS5637_OSR_256  0x00
#define MS5637_OSR_512  0x02
#define MS5637_OSR_1024 0x04
#define MS5637_OSR_2048 0x06
#define MS5637_OSR_4096 0x08

#define SEA_LEVEL_PRESSURE 1013.25f

#define RX_BUFFER_SIZE 128

#define SENSOR_SAMPLE_RATE 400
#define RESET_THRESHOLD 2000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint16_t MS56_PROMData[MS56_PROM_SIZE];
uint8_t RxWriteIndex1;
uint8_t RxReadIndex1;
uint8_t Rx_data1[2];
uint8_t Rx_Buffer1[RX_BUFFER_SIZE];
uint8_t Transfer_cplt1;
uint8_t RxWriteIndex2;
uint8_t RxReadIndex2;
uint8_t Rx_data2[2];
uint8_t Rx_Buffer2[RX_BUFFER_SIZE];
uint8_t Transfer_cplt2;
uint8_t nmea_buffer[20];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
int8_t MS5637_ReadData(float *temperature, float *pressure, float *altitude);
int8_t MS5637_Setup(void);
static void errorHandler(uint8_t source, uint8_t errorId);
uint8_t itoa(int32_t x, uint8_t *str, uint8_t n, uint8_t d);
uint8_t ftoa(float x, uint8_t *str, uint8_t n, uint8_t afterpoint);
void ubx_checksum(uint8_t *msg);
int nmea_checksum(const char *s);
MSGStatus parse_msg(uint8_t val);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#ifdef __GNUC__
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
/**
 * @brief  Retargets the C library printf function to the USART.
 * @param  None
 * @retval None
 */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the EVAL_COM1 and Loop until the end of transmission */
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);

  return ch;
}
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
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  int8_t retval = MS5637_Setup();
  if (retval < 0) {
    errorHandler(ERROR_MS56, -retval);
  }

  // Configure the UART receive interrupts
  HAL_UART_Receive_IT(&huart1, Rx_data1, 1);
  HAL_UART_Receive_IT(&huart2, Rx_data2, 1);
  RxReadIndex1 = 0;
  RxWriteIndex1 = 0;
  RxReadIndex2 = 0;
  RxWriteIndex2 = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  char buf[RX_BUFFER_SIZE];
  uint32_t loop_counter = 0;
  uint32_t last_valid_message = 0;
  while (1) {

    // Write data from the USART1 to USART2
    uint8_t i;
    for (i = 0; RxReadIndex1 != RxWriteIndex1; ++i) {
      buf[i] = Rx_Buffer1[RxReadIndex1];
      RxReadIndex1 = (RxReadIndex1 + 1) % RX_BUFFER_SIZE;
    }
    if (i > 0) {
      HAL_UART_Transmit(&huart2, (uint8_t *)buf, i, 0xFFFF);
    }

    // Write data from the USART2 to USART1
    // Insert sensor messages when appropriate
    for (i = 0; RxReadIndex2 != RxWriteIndex2; ++i) {
      uint8_t val = Rx_Buffer2[RxReadIndex2];
      RxReadIndex2 = (RxReadIndex2 + 1) % RX_BUFFER_SIZE;

      // Write the byte to the output channel.
      HAL_UART_Transmit(&huart1, &val, 1, 0xFFFF);

      MSGStatus status = parse_msg(val);
      switch (status) {
      case NMEA_COMPLETE:
      case UBX_COMPLETE:
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
	last_valid_message = 0;
	break;
      case MSG_IN_PROCESS:
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
	break;
      case MSG_ERROR:
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
	break;
      }

      if (loop_counter > SENSOR_SAMPLE_RATE) {
	if (status == NMEA_COMPLETE) {
	  // Read the pressure/temperature sensor.
	  float temperature;
	  float pressure;
	  float altitude;
	  MS5637_ReadData(&temperature, &pressure, &altitude);
	  uint8_t flen;
	  HAL_UART_Transmit(&huart1, (uint8_t *)"$GPBAR,1,", 9, 0xFFFF);
	  flen = ftoa(temperature, nmea_buffer, 20, 3);
	  HAL_UART_Transmit(&huart1, nmea_buffer, flen, 0xFFFF);
	  HAL_UART_Transmit(&huart1, (uint8_t*)",", 1, 0xFFFF);
	  flen = ftoa(pressure, nmea_buffer, 10, 3);
	  HAL_UART_Transmit(&huart1, nmea_buffer, flen, 0xFFFF);
	  HAL_UART_Transmit(&huart1, (uint8_t*)",", 1, 0xFFFF);
	  flen = ftoa(altitude, nmea_buffer, 20, 3);
	  HAL_UART_Transmit(&huart1, nmea_buffer, flen, 0xFFFF);
	  HAL_UART_Transmit(&huart1, (uint8_t*)"\r\n", 2, 0xFFFF);
	  loop_counter = 0;
	} else if (status == UBX_COMPLETE) {
	  loop_counter = 0;
	}
      }
    }
    ++loop_counter;

    // Reset the UART if we don't recieve a message for a while.
    if (++last_valid_message > RESET_THRESHOLD) {
      HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
      HAL_UART_Init(&huart2);
      MX_USART2_UART_Init();
      HAL_UART_Receive_IT(&huart2, Rx_data2, 1);
      last_valid_message = 0;
    }
    HAL_Delay(1);
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
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  hi2c1.Init.Timing = 0x2000090E;
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
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  //huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_AUTOBAUDRATE_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */
  //huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_AUTOBAUDRATE_INIT;

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : IMU_INT1_Pin IMU_INT2_Pin */
  GPIO_InitStruct.Pin = IMU_INT1_Pin|IMU_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

// Interrupt/DMA callback routine
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);

  if (huart->Instance == USART1) {
    uint8_t next = (RxWriteIndex1 + 1) % RX_BUFFER_SIZE;

    // Don't overflow the buffer
    if (next != RxReadIndex1) {

      // Add data to Rx_Buffer
      Rx_Buffer1[RxWriteIndex1] = Rx_data1[0];
      RxWriteIndex1 = next;
    }

    // Activate UART receive interrupt every time
    HAL_UART_Receive_IT(&huart1, Rx_data1, 1);

  } else {
    uint8_t next = (RxWriteIndex2 + 1) % RX_BUFFER_SIZE;

    // Don't overflow the buffer
    if (next != RxReadIndex2) {

      // Add data to Rx_Buffer
      Rx_Buffer2[RxWriteIndex2] = Rx_data2[0];
      RxWriteIndex2 = next;
    }

    // Activate UART receive interrupt every time
    HAL_UART_Receive_IT(&huart2, Rx_data2, 1);

  }
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
}

static void errorHandler(uint8_t source, uint8_t errorId) {
#ifdef DEBUG
  uint8_t i;

  while(1) {

    for (i=0; i<source; i++) {
      HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
      doSleepStop_WakeRTC_RTCCLK(500);
      HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
      doSleepStop_WakeRTC_RTCCLK(300);
    }

    doSleepStop_WakeRTC_RTCCLK(1500);

    for (i=0; i<errorId; i++) {
      HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
      doSleepStop_WakeRTC_RTCCLK(50);
      HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
      doSleepStop_WakeRTC_RTCCLK(450);
    }

    doSleepStop_WakeRTC_CKSPRE(5);
  }
#else
  /* Reset MCU */
  HAL_NVIC_SystemReset();
#endif
}

int8_t MS5637_Setup(void) {
  HAL_StatusTypeDef status;
  uint8_t data[2];
  uint8_t i;

  // sensor needs at most 15ms after power-up
  HAL_Delay(100);

  /* Check if device ready to accept commands */
  status = HAL_I2C_IsDeviceReady(&hi2c1, MS5637_I2C_ADDR, 10, 1000);
  if (status ) {
    return -1;
  }

  /* Send reset sequence */
  data[0] = 0x1E;
  status = HAL_I2C_Master_Transmit(&hi2c1, MS5637_I2C_ADDR, data, 1, 10);
  if (status ) {
    return -1;
  }
  HAL_Delay(100);

  /* Read PROM containing calibration data */
  for (i=0; i<MS56_PROM_SIZE; i++) {

    /* Send PROM read command */
    data[0] = MS56_CMD_PROM_READ + (i<<1);
    status = HAL_I2C_Master_Transmit(&hi2c1, MS5637_I2C_ADDR, data, 1, 10);
    if (status ) {
      return -1;
    }

    /* Read 16-bit data */
    status = HAL_I2C_Master_Receive(&hi2c1, MS5637_I2C_ADDR, data, 2, 10);
    if (status ) {
      return -1;
    }
    MS56_PROMData[i] = ((uint16_t)data[0] << 8) + data[1];
  }

  return 0;
}

int8_t MS5637_ReadData(float *temperature, float *pressure, float *altitude) {
  static double smoothed_temperature = 1000;
  HAL_StatusTypeDef status;
  uint8_t data[3];

  // Send temperature conversion command
  data[0] = MS56_CMD_CONV_T_256 + 6;
  status = HAL_I2C_Master_Transmit(&hi2c1, MS5637_I2C_ADDR, data, 1, 10);
  if (status ) {
    return -1;
  }

  // Wait for conversion - depends on oversampling ratio
  HAL_Delay(6);

  /* Read ADC result */

  data[0] = MS56_CMD_ADC_READ;
  status = HAL_I2C_Master_Transmit(&hi2c1, MS5637_I2C_ADDR, data, 1, 10);
  if (status ) {
    return -3;
  }

  status = HAL_I2C_Master_Receive(&hi2c1, MS5637_I2C_ADDR, data, 3, 10);
  if (status ) {
    return -4;
  }

  uint32_t d2 = ((uint32_t)data[0] << 16) + (data[1] << 8) + data[2];

  /* Send pressure conversion command */
  data[0] = MS56_CMD_CONV_P_256 + 6;
  status = HAL_I2C_Master_Transmit(&hi2c1, MS5637_I2C_ADDR, data, 1, 10);
  if (status ) {
    return -5;
  }

  /* Wait for conversion - depends on oversampling ratio */
  HAL_Delay(6);

  /* Read ADC result */
  data[0] = MS56_CMD_ADC_READ;
  status = HAL_I2C_Master_Transmit(&hi2c1, MS5637_I2C_ADDR, data, 1, 10);
  if (status ) {
    return -7;
  }

  status = HAL_I2C_Master_Receive(&hi2c1, MS5637_I2C_ADDR, data, 3, 10);
  if (status ) {
    return -8;
  }

  uint32_t d1 = ((uint32_t)data[0] << 16) + (data[1] << 8) + data[2];

  // Calculate the final temperature, pressure and altitude values
  int64_t dT = d2 - (((uint64_t)MS56_PROMData[5]) << 8);
  int32_t t = 2000 + ((dT * (int64_t)MS56_PROMData[6]) >> 23);

  int64_t off  = ((int64_t)MS56_PROMData[2] << 17) + ((dT * (int64_t)MS56_PROMData[4]) >> 6);
  int64_t sens = (((int64_t)MS56_PROMData[1]) << 16) + ((dT * (int64_t)MS56_PROMData[3]) >> 7);

  // Second order temperature compensation
  if (t < 2000) {
    double square = pow (dT,2);
    double t2 = square / (1L << 31);
    square = pow (t-2000,2);
    double off2  = square * 5 / 2;
    double sens2 = square * 5 / 4;
    if (t < 15) {
      square = pow(t+1500,2);
      off2  += square * 7;
      sens2 += square * 11 / 2;
    }

    t    -= t2;
    off  -= off2;
    sens -= sens2;
  }

  int64_t p0 = ((sens * (int64_t)d1) >> 21);
  int32_t p = (int32_t)((p0 - off) >> 15);

  *temperature = (float)t / 100.0f;
  if (smoothed_temperature == 1000) {
    smoothed_temperature = *temperature;
  } else {
    smoothed_temperature = (smoothed_temperature * 9.0 + *temperature) / 10.0;
  }
  *pressure = (float)p / 100.0f;
  *altitude = ((pow((SEA_LEVEL_PRESSURE / *pressure), 1 / 5.257) - 1.0) *
	       (*temperature + 273.15)) / 0.0065;

  return 0;
}


// Converts a given integer x to string str[].  d is the number
// of digits required in output. If d is more than the number
// of digits in x, then 0s are added at the beginning.
// The length of the string is returned, and it is not null terminated.
uint8_t itoa(int32_t x, uint8_t *str, uint8_t n, uint8_t d) {
  uint8_t len = 0;
  uint8_t i;
  uint8_t j;

  // Don't create an empty string
  if (n == 0) {
    return 0;
  }

  // Insert the negative sign if necessary.
  if (x < 0) {
    x = -x;
    str[0] = '-';
    ++str;
    --n;
    len = 1;
  }

  // The number length can't be longer than the buffer.
  if (d > n) {
    d = n;
  }

  // Create the string backward in the buffer
  for (i = 0; (i < n) && x; ++i, x = x / 10) {
    str[i] = x % 10;
  }

  // Pad with zeros if requested.
  for ( ; i < d; ++i) {
    str[i] = 0;
  }

  // Add the current number of digets to the length;
  len += i;

  // Reverse the string and convert to ASCII
  for (j = 0; i > j; ++j, --i) {
    uint8_t tmp = str[i - 1];
    str[i - 1] = '0' + str[j];
    str[j] = '0' + tmp;
  }

  // Return the length of the string created.
  return len;
}

// Converts a floating point number to string.
uint8_t ftoa(float x, uint8_t *str, uint8_t n, uint8_t afterpoint) {
  uint8_t len = 0;
  uint8_t i;
  uint8_t j;

  // Don't create an empty string
  if (n == 0) {
    return 0;
  }

  // Insert the negative sign if necessary.
  if (x < 0) {
    x = -x;
    str[0] = '-';
    ++str;
    --n;
    len = 1;
  }

  // Append the "0." if the number is < 1.0.
  if (x < 1.0) {
    if (n < 2) {
      return 0;
    }
    str[0] = '0';
    str[1] = '.';
    str += 2;
    n -= 2;
    len += 2;
  }

  // Turn the floating point value into fixed point.
  for (i = 0; i < afterpoint; ++i) {
    x *= 10.0;
  }

  // Create the string backward in the buffer
  for (i = 0; (i < n) && (x > 1); ++i) {
    // Add the decimal point when necessary.
    if ((afterpoint != 0) && (i == afterpoint)) {
      str[i] = '.';
      continue;
    }
    str[i] = '0' + ((uint32_t)(x) % 10);
    x = x / 10;
  }

  // Add the current number of digets to the length;
  len += i;

  // Reverse the string;
  for (j = 0; i > j; ++j, --i) {
    uint8_t tmp = str[i - 1];
    str[i - 1] = str[j];
    str[j] = tmp;
  }

  // Return the length of the string created.
  return len;
}

void ubx_checksum(uint8_t *msg) {
  uint16_t length = ((uint16_t*)msg)[2];
  uint8_t ck_a = 0;
  uint8_t ck_b = 0;
  uint16_t i;
  for (i = 2; i < length + 6; ++i) {
    ck_a = ck_a + msg[i];
    ck_b = ck_b + ck_a;
  }
  msg[length + 6] = ck_a;
  msg[length + 7] = ck_b;
}

int nmea_checksum(const char *s) {
  int c = 0;
  while(*s)
    c ^= *s++;
  return c;
}

MSGStatus parse_msg(uint8_t val) {
  static uint8_t NMEA_msg_len = 0;
  static uint8_t UBX_msg_len = 0;
  static uint8_t UBX_payload_len = 0;
  static uint8_t parsing_UBX = 0;

  // Are we parsing an NMEA message?
  if (NMEA_msg_len) {
    ++NMEA_msg_len;
    parsing_UBX = 0;
    // Is this message too long, or are we missing the start delminter?
    if ((NMEA_msg_len > 82) || ((NMEA_msg_len == 7) && (val != ','))) {
      NMEA_msg_len = 0;
      return MSG_IN_PROCESS;
    } else if (val == '\n') {
      // We recieved the end of a frame.
      NMEA_msg_len = 0;
      return NMEA_COMPLETE;
    }
  } else if (UBX_msg_len) {
    // Are we parsing a UBX message?
    parsing_UBX = 1;
    ++UBX_msg_len;
    if (UBX_msg_len == 2) {
      // Is this the second sync char?
      if (val != 0x62) {
	UBX_msg_len = 0;
	return MSG_ERROR;
      }
    } else if (UBX_msg_len == 5) {
      // This should be the high byte of the length field.
      // It's little-endian, so this is the least significant byte.
      UBX_payload_len = val;
    } else if (UBX_msg_len == 6) {
      // None of the messages that we are interested in are longer than 255 bytes.
      // If this value is non-zero, we must have a parsing error.
      if (val) {
	UBX_msg_len = 0;
	return MSG_ERROR;
      }
    } else if (UBX_msg_len == (UBX_payload_len + 8)) {
      UBX_msg_len = 0;
      return UBX_COMPLETE;
    }
  } else if (val == '$') {
    // Start of NMEA message.
    NMEA_msg_len = 1;
  } if (val == 0xB5) {
    // Start of UBX message.
    UBX_msg_len = 1;
  }
  return MSG_IN_PROCESS;
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
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
