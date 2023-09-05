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
#include "Objects.h"
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
//variables for time based
volatile uint32_t mainTimeTick = 0;
volatile uint32_t sensorReqMask = 0;
volatile uint32_t laserReqMask = 0;
uint16_t headlightsLevel = 0;
uint16_t ambientLightLevel = 0;
uint16_t mcuTemp = 0;
uint16_t mcuVoltage = 0;

extern I2C_IRQ_Conn_t I2CSensors;		//i2c1 bus for sensors
extern I2C_IRQ_Conn_t I2CLasers;		//i2c3 bus for VL53L01 sensors
extern SPI_Conn_TWO_t SPI_LoRa;			//SPI1 for LoRa radio
extern SPI_Conn_TWO_t SPI_Flash;		//SPI3 for flash
extern USART_Conn_t USART_Orange;		//usart1 bus for Orange Pi Zero2 connection
extern USART_Conn_t USART_CLI;			//usart2 bus for external connection CLI
extern USART_Conn_t USART_UNO;			//usart3 bus for external connection Arduino
extern USART_Conn_t UART_GPS;				//uart5 bus for GPS connection
extern TCA9548A_t tca9548;					//i2c multiplexer
extern VL53L0x_t LaserFrontLeft;		//laser sensor VL53L01 front left place
extern VL53L0x_t LaserFrontRight;		//laser sensor VL53L01 front left place
extern VL53L0x_t LaserMidLeft;			//laser sensor VL53L01 front left place
extern VL53L0x_t LaserMidRight;			//laser sensor VL53L01 front left place
extern VL53L0x_t LaserBackLeft;			//laser sensor VL53L01 front left place
extern VL53L0x_t LaserBackRight;		//laser sensor VL53L01 front left place
extern ADXL345_t adxl345;						//accelerometer
extern ITG3205_t itg3205;						//gyroscope
extern QMC5883L_t qmc5883; 					//magnetometer
extern BME280_t bme280;							//ambient sensor
extern INA219_t ina219;							//current voltage power sensor
extern HC_SR04_t USMrange;

Camera_t camera;		//camera's servo drives
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
  //MX_IWDG_Init();
  /* USER CODE BEGIN 2 */
	camera.posH = Servo_Init(&camera.srvLR, SG90_MIN, SG90_MAX, 10);
	camera.posV = Servo_Init(&camera.srvUD, SG90_MIN, SG90_MAX, 10);
	HardwareInit();
	camera.posH = SG90_MIN;
	camera.posV = SG90_MIN;
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
	camera.posV = (SG90_MIN + SG90_MAX) / 2;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if (sensorReqMask && ADXL_REQ_MASK) {
	  	uint8_t st;
	  	st = ADXL345_GetData(&I2CSensors, &adxl345);
	  	if (st) {
	  		BusRequestOff(sensorReqMask, ADXL_REQ_MASK);
	  	}
	  }
	  if (sensorReqMask && ITG_REQ_MASK) {
	  	uint8_t st;
	  	st = ITG3205_GetData(&I2CSensors, &itg3205);
	  	if (st) { BusRequestOff(sensorReqMask, ITG_REQ_MASK); }
	  }
	  if (sensorReqMask && QMC_REQ_MASK) {
	  	uint8_t st;
	  	st = QMC5883L_GetData(&I2CSensors, &qmc5883);
	  	if (st) {
	  		BusRequestOff(sensorReqMask, QMC_REQ_MASK);
	  		///quaternions calculate
	  	}
	  }
		if (sensorReqMask && BME_REQ_MASK) {
			uint8_t st;
			st = BME280_GetData(&I2CSensors, &bme280);
			if (st) {
				BusRequestOff(sensorReqMask,BME_REQ_MASK);
			}
		}
		if (sensorReqMask && INA_REQ_MASK) {
			uint8_t st;
			st = INA219_GetRawData(&I2CSensors, &ina219);
			if (st) {
				BusRequestOff(sensorReqMask, INA_REQ_MASK);
			}
		}
		camera.posH = SG90_MAX;
		LL_mDelay(2000);
		camera.posV = SG90_MAX;
		LL_mDelay(2000);
		camera.posH = SG90_MIN;
		LL_mDelay(2000);
		camera.posV = SG90_MIN;
		LL_mDelay(2000);

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
