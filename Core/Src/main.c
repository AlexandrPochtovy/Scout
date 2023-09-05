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
#include "adc.h"
#include "crc.h"
#include "i2c.h"
#include "iwdg.h"
#include "rng.h"
#include "rtc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

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

/* USER CODE BEGIN PV */
//i2c1 bus for sensors
uint8_t i2c1Buff[32] = {0};
fifo_t i2c1Data = {.buffer = i2c1Buff, .buffer_size = sizeof(i2c1Buff), .bytes_avail = 0,
                   .head = 0, .tail = 0, .lockState = BUFFER_FREE};
I2C_IRQ_Conn_t I2CSensors = {.i2c = I2C1,
                          .status = PORT_FREE,
                          .step = 0,
                          .addr = 0x00,
                          .len = 0,
                          .mode = I2C_MODE_WRITE,
                          .buffer = &i2c1Data};
//i2c3 bus for VL53L01 sensors
uint8_t i2c3Buff[32] = {0};
fifo_t i2c3Data = {.buffer = i2c3Buff, .buffer_size = sizeof(i2c3Buff), .bytes_avail = 0,
                   .head = 0, .tail = 0, .lockState = BUFFER_FREE};
I2C_IRQ_Conn_t I2CLasers= {.i2c = I2C3,
                          .status = PORT_FREE,
                          .step = 0,
                          .addr = 0x00,
                          .len = 0,
                          .mode = I2C_MODE_WRITE,
                          .buffer = &i2c3Data};
//SPI1 for LoRa radio
uint8_t spi1b1[255] = {0};
uint8_t spi1b2[255] = {0};
fifo_t spi1_TX = {.buffer = spi1b1, .buffer_size = sizeof(spi1b1), .bytes_avail = 0,
                    .head = 0, .tail = 0, .lockState = BUFFER_FREE};
fifo_t spi1_RX = {.buffer = spi1b2, .buffer_size = sizeof(spi1b2), .bytes_avail = 0,
                    .head = 0, .tail = 0, .lockState = BUFFER_FREE};
SPI_Conn_TWO_t SPI_LoRa = {	.SPIbus = SPI1,
							.status = PORT_FREE,
							.mode = SPI_MODE_DUPLEX,
							.txlen = 0,
							.txbuffer = &spi1_TX,
							.txlen = 0,
							.rxbuffer = &spi1_RX,
							.rxlen = 0
};
//SPI3 for flash
uint8_t spi3b1[1024] = {0};
uint8_t spi3b2[1024] = {0};
fifo_t spi3_TX = {.buffer = spi3b1, .buffer_size = sizeof(spi3b1), .bytes_avail = 0,
                    .head = 0, .tail = 0, .lockState = BUFFER_FREE};
fifo_t spi3_RX = {.buffer = spi3b2, .buffer_size = sizeof(spi3b2), .bytes_avail = 0,
                    .head = 0, .tail = 0, .lockState = BUFFER_FREE};
SPI_Conn_TWO_t SPI_Flash = {.SPIbus = SPI3,
							.status = PORT_FREE,
							.mode = SPI_MODE_DUPLEX,
							.txlen = 0,
							.txbuffer = &spi3_TX,
							.txlen = 0,
							.rxbuffer = &spi3_RX,
							.rxlen = 0
};
//usart1 bus for Orange Pi Zero2 connection
uint8_t usart1b1[255] = {0};
uint8_t usart1b2[255] = {0};
fifo_t usart1_TX = {.buffer = usart1b1, .buffer_size = sizeof(usart1b1), .bytes_avail = 0,
                    .head = 0, .tail = 0, .lockState = BUFFER_FREE};
fifo_t usart1_RX = {.buffer = usart1b2, .buffer_size = sizeof(usart1b2), .bytes_avail = 0,
                    .head = 0, .tail = 0, .lockState = BUFFER_FREE};
USART_Conn_t USART_Orange = {.USART = USART1,
                             .txStatus = PORT_FREE,
                             .rxStatus= PORT_FREE,
                             .txbuffer = &usart1_TX,
                             .txlen = 0,
                             .rxbuffer = &usart1_RX,
                             .rxlen = 0};
//usart2 bus for external connection CLI
uint8_t usart2b1[255] = {0};
uint8_t usart2b2[255] = {0};
fifo_t usart2_TX = {.buffer = usart2b1, .buffer_size = sizeof(usart2b1), .bytes_avail = 0,
                    .head = 0, .tail = 0, .lockState = BUFFER_FREE};
fifo_t usart2_RX = {.buffer = usart2b2, .buffer_size = sizeof(usart2b2), .bytes_avail = 0,
                    .head = 0, .tail = 0, .lockState = BUFFER_FREE};
USART_Conn_t USART_CLI = {.USART = USART2,
                             .txStatus = PORT_FREE,
                             .rxStatus= PORT_FREE,
                             .txbuffer = &usart2_TX,
                             .txlen = 0,
                             .rxbuffer = &usart2_RX,
                             .rxlen = 0};
//usart3 bus for external connection Arduino
uint8_t usart3b1[255] = {0};
uint8_t usart3b2[255] = {0};
fifo_t usart3_TX = {.buffer = usart3b1, .buffer_size = sizeof(usart3b1), .bytes_avail = 0,
                    .head = 0, .tail = 0, .lockState = BUFFER_FREE};
fifo_t usart3_RX = {.buffer = usart3b2, .buffer_size = sizeof(usart3b2), .bytes_avail = 0,
                    .head = 0, .tail = 0, .lockState = BUFFER_FREE};
USART_Conn_t USART_UNO = {.USART = USART3,
                          .txStatus = PORT_FREE,
                          .rxStatus= PORT_FREE,
                          .txbuffer = &usart3_TX,
                          .txlen = 0,
                          .rxbuffer = &usart3_RX,
                          .rxlen = 0};
//uart5 bus for GPS connection
uint8_t uart5b1[255] = {0};
uint8_t uart5b2[255] = {0};
fifo_t uart5_TX = {.buffer = uart5b1, .buffer_size = sizeof(uart5b1), .bytes_avail = 0,
                    .head = 0, .tail = 0, .lockState = BUFFER_FREE};
fifo_t uart5_RX = {.buffer = uart5b2, .buffer_size = sizeof(uart5b2), .bytes_avail = 0,
                    .head = 0, .tail = 0, .lockState = BUFFER_FREE};
USART_Conn_t UART_GPS = {.USART = UART5,
                         .txStatus = PORT_FREE,
                         .rxStatus= PORT_FREE,
                         .txbuffer = &uart5_TX,
                         .txlen = 0,
                         .rxbuffer = &uart5_RX,
                         .rxlen = 0};

//	Objects and Sensors
TCA9548A_t tca9548 = {.addr=TCA9548A_ADDR, .status=DEVICE_NOT_INIT, .step=0, .port=0x00};						//i2c multiplexer

ADXL345_t adxl345 = {.addr=ADXL345_ADDR, .status=DEVICE_NOT_INIT, .step=0, .data={0}, .raw={0}};		//accelerometer
ITG3205_t itg3205 = {.addr=ITG3205_ADDR, .status=DEVICE_NOT_INIT, .step=0, .data={0}, .raw={0}};		//gyroscope
QMC5883L_t qmc5883 = {.addr=QMC5883L_ADDR, .status=DEVICE_NOT_INIT, .step=0, .data={0}, .raw={0}}; 	//magnetometer
BME280_t bme280 = {.addr=BME280_ADDR1, .status=DEVICE_NOT_INIT, .step=0,
                   .calib_data={0}, .uncomp_data={0}, .data_int={0}, .data_float={0}};							//ambient sensor
INA219_t ina219 = {.addr=INA219_ADDR, .status=DEVICE_NOT_INIT, .step=0, .raw={0}};									//current voltage power sensor

Camera_t camera;		//camera's servo drives
HC_SR04_t USMrange = {.distance_mm = 0, .start = 0, .stop = 0};

uint16_t headlightsLevel = 0;
uint16_t ambientLightLevel = 0;
uint16_t mcuTemp = 0;
uint16_t mcuVoltage = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
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
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  /* System interrupt init*/
  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  /* SysTick_IRQn interrupt configuration */
  NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),15, 0));

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CRC_Init();
  MX_RNG_Init();
  MX_RTC_Init();
  MX_SPI1_Init();
  MX_SPI3_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_I2C3_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_UART5_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM9_Init();
  MX_TIM10_Init();
  MX_TIM11_Init();
  MX_TIM13_Init();
  MX_TIM12_Init();
  MX_TIM8_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_ADC1_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */
	camera.posH = Servo_Init(&camera.srvLR, SG90_MIN, SG90_MAX, 10);
	camera.posV = Servo_Init(&camera.srvUD, SG90_MIN, SG90_MAX, 10);
	HardwareInit();

	camera.posH = SG90_MIN;
/*	camera.posV = SG90_MIN;
	LL_mDelay(2000);
	camera.posH = SG90_MAX;
	LL_mDelay(2000);
	camera.posV = SG90_MAX;
	LL_mDelay(2000);
	camera.posH = SG90_MIN;
	LL_mDelay(2000);
	camera.posV = SG90_MIN;
	LL_mDelay(2000);
	camera.posH = (SG90_MIN + SG90_MAX) / 2;
	camera.posV = (SG90_MIN + SG90_MAX) / 2;*/
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  TIM8->CCR3 = headlightsLevel;
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
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_5);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_5)
  {
  }
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
  LL_RCC_HSE_Enable();

   /* Wait till HSE is ready */
  while(LL_RCC_HSE_IsReady() != 1)
  {

  }
  LL_RCC_LSI_Enable();

   /* Wait till LSI is ready */
  while(LL_RCC_LSI_IsReady() != 1)
  {

  }
  LL_PWR_EnableBkUpAccess();
  LL_RCC_LSE_Enable();

   /* Wait till LSE is ready */
  while(LL_RCC_LSE_IsReady() != 1)
  {

  }
  LL_RCC_HSE_EnableCSS();
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_8, 320, LL_RCC_PLLP_DIV_2);
  LL_RCC_PLL_ConfigDomain_48M(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_8, 320, LL_RCC_PLLQ_DIV_8);
  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {

  }
  while (LL_PWR_IsActiveFlag_VOS() == 0)
  {
  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_4);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_2);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {

  }
  LL_Init1msTick(160000000);
  LL_SetSystemCoreClock(160000000);
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
