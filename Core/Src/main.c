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
uint16_t headLightsLevel = 0;
uint16_t ambientLightLevel = 0;
uint16_t mcuTemp = 0;
uint16_t mcuVoltage = 0;

extern I2C_IRQ_Conn_t I2CSensors;		//i2c1 bus for sensors
extern I2C_IRQ_Conn_t I2CLasers;		//i2c3 bus for VL53L01 sensors
extern SPI_Conn_TWO_t SPI_LoRa;			//SPI1 for LoRa radio
extern SPI_Conn_TWO_t SPI_Flash;		//SPI3 for flash
extern USART_Conn_t USART_Orange;	//usart1 bus for Orange Pi Zero2 connection
extern USART_Conn_t USART_CLI;			//usart2 bus for external connection CLI
extern USART_Conn_t USART_UNO;		//usart3 bus for external connection Arduino
extern USART_Conn_t UART_GPS;				//uart5 bus for GPS connection
extern TCA9548A_t tca9548;					//i2c multiplexer
extern VL53L0x_t LaserFrontLeft;		//laser sensor VL53L01 front left place
extern VL53L0x_t LaserFrontRight;		//laser sensor VL53L01 front left place
extern VL53L0x_t LaserMidLeft;			//laser sensor VL53L01 front left place
extern VL53L0x_t LaserMidRight;			//laser sensor VL53L01 front left place
extern VL53L0x_t LaserBackLeft;			//laser sensor VL53L01 front left place
extern VL53L0x_t LaserBackRight;		//laser sensor VL53L01 front left place
extern ADXL345_t adxl345;				//accelerometer
extern ITG3205_t itg3205;				//gyroscope
extern QMC5883L_t qmc5883; 				//magnetometer
extern BME280_t bme280;					//ambient sensor
extern INA219_t ina219;					//current voltage power sensor
extern HC_SR04_t USMrange;
extern Drive_t Drive;
extern PID_MF_t pidWL;
extern PID_MF_t pidWR;
Camera_t camera;		//camera's servo drives
uint16_t leftWheelPWM = 0;
uint16_t rightWheelPWM = 0;
//extern Quaternion_t quattro;
//extern EulerAngles_t mainAngles;
extern struct commonQ basQuat1;
extern struct commonQ basQuat2;
extern struct commonQ basQuat3;
extern struct commonQ basQuat4;
extern struct commonQ basQuat5;
extern struct commonQ basQuat6;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void DriveStop(void);
void DriveForward(void);
void DriveBackward(void);
void DriveRight(void);
void DriveLeft(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

	/* System interrupt init*/
	NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

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
	MX_TIM12_Init();
	MX_TIM8_Init();
	MX_TIM6_Init();
	MX_TIM7_Init();
	MX_TIM13_Init();
	MX_ADC1_Init();
	//MX_IWDG_Init();
	/* USER CODE BEGIN 2 */
	SysTick_Config(SystemCoreClock / 1000);	//1ms tick
	TIM10->CCR1 = camera.posH = Servo_Init(&camera.srvLR, SG90_MIN, SG90_MAX,
	    10);
	TIM11->CCR1 = camera.posV = Servo_Init(&camera.srvUD, SG90_MIN, SG90_MAX,
	    10);

	HardwareInit();
	PID_MotoFilteredInit(10, 5, 2.5, 5, 100, &pidWL);
	PID_MotoFilteredInit(10, 5, 2.5, 5, 100, &pidWR);
	uint8_t stbus;
	LL_GPIO_ResetOutputPin(Laser1_SHUT_GPIO_Port, Laser1_SHUT_Pin);
	LL_mDelay(5);
	LL_GPIO_SetOutputPin(Laser1_SHUT_GPIO_Port, Laser1_SHUT_Pin);
	LL_mDelay(5);
	LL_GPIO_ResetOutputPin(Laser2_SHUT_GPIO_Port, Laser2_SHUT_Pin);
	LL_mDelay(5);
	LL_GPIO_SetOutputPin(Laser2_SHUT_GPIO_Port, Laser2_SHUT_Pin);
	LL_mDelay(5);
	LL_GPIO_ResetOutputPin(Laser3_SHUT_GPIO_Port, Laser3_SHUT_Pin);
	LL_mDelay(5);
	LL_GPIO_SetOutputPin(Laser3_SHUT_GPIO_Port, Laser3_SHUT_Pin);
	LL_mDelay(5);
	LL_GPIO_ResetOutputPin(Laser4_SHUT_GPIO_Port, Laser4_SHUT_Pin);
	LL_mDelay(5);
	LL_GPIO_SetOutputPin(Laser4_SHUT_GPIO_Port, Laser4_SHUT_Pin);
	LL_mDelay(5);
	LL_GPIO_ResetOutputPin(Laser5_SHUT_GPIO_Port, Laser5_SHUT_Pin);
	LL_mDelay(5);
	LL_GPIO_SetOutputPin(Laser5_SHUT_GPIO_Port, Laser5_SHUT_Pin);
	LL_mDelay(5);
	LL_GPIO_ResetOutputPin(Laser_6_SHUT_GPIO_Port, Laser_6_SHUT_Pin);
	LL_mDelay(5);
	LL_GPIO_SetOutputPin(Laser_6_SHUT_GPIO_Port, Laser_6_SHUT_Pin);
	LL_mDelay(5);
	do {
		stbus = TCA9548A_SetChannels(&I2CLasers, &tca9548, TCA9548A_CH_OFF);
	} while (!stbus);			// init i2c multiplexor
	do {
		stbus = TCA9548A_SetChannels(&I2CLasers, &tca9548, TCA9548A_CH0);
	} while (!stbus);	// enable channel 0

	do {
		stbus = VL53L0x_Init(&I2CLasers, &LaserBackLeft);
	} while (!stbus);				// init laser back left
	do {
		stbus = TCA9548A_SetChannels(&I2CLasers, &tca9548, TCA9548A_CH1);
	} while (!stbus);	// enable channel 1
	do {
		stbus = VL53L0x_Init(&I2CLasers, &LaserBackRight);
	} while (!stbus);				// init laser back right
	do {
		stbus = TCA9548A_SetChannels(&I2CLasers, &tca9548, TCA9548A_CH2);
	} while (!stbus);	// enable channel 2*/
	do {
		stbus = VL53L0x_Init(&I2CLasers, &LaserMidRight);
	} while (!stbus);				// init laser mid right
	do {
		stbus = TCA9548A_SetChannels(&I2CLasers, &tca9548, TCA9548A_CH3);
	} while (!stbus);	// enable channel 3*/
	do {
		stbus = VL53L0x_Init(&I2CLasers, &LaserFrontRight);
	} while (!stbus);				// init laser front right
	do {
		stbus = TCA9548A_SetChannels(&I2CLasers, &tca9548, TCA9548A_CH4);
	} while (!stbus);	// enable channel 4*/
	do {
		stbus = VL53L0x_Init(&I2CLasers, &LaserFrontLeft);
	} while (!stbus);				// init laser front left
	do {
		stbus = TCA9548A_SetChannels(&I2CLasers, &tca9548, TCA9548A_CH5);
	} while (!stbus);	// enable channel 5*/
	do {
		stbus = VL53L0x_Init(&I2CLasers, &LaserMidLeft);
	} while (!stbus);				// init laser mid left
	do {
		stbus = ADXL345_Init(&I2CSensors, &adxl345);
	} while (!stbus);			// init accelerometer pass
	do {
		stbus = QMC5883L_Init(&I2CSensors, &qmc5883);
	} while (!stbus);		// init magnetometer pass
	do {
		stbus = ITG3205_Init(&I2CSensors, &itg3205);
	} while (!stbus);			// init gyroscope
	do {
		stbus = INA219_Init(&I2CSensors, &ina219);
	} while (!stbus);				// init power sensor
	do {
		stbus = BME280_Init(&I2CSensors, &bme280);
	} while (!stbus);				// init ambient sensor pass
	__NOP();

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {

		if (laserReqMask) {
			switch (laserReqMask) {
			case 1:	// enable channel 0
				if (TCA9548A_SetChannels(&I2CLasers, &tca9548, TCA9548A_CH0)) {
					laserReqMask = 2;
				}
				break;
			case 2:
				if (VL53_ReadRangeSingle(&I2CLasers, &LaserBackLeft)) {
					laserReqMask = 6;
				}
				break;
			case 3:	//
				if (VL53_StartContinuous(&I2CLasers, &LaserBackLeft, 30)) {
					laserReqMask = 4;
				}
				break;
			case 4:
				if (VL53_readRangeContinuousMillimeters(&I2CLasers, &LaserBackLeft)) {
					laserReqMask = 5;
				}
				break;
			case 5:
				if (VL53_StopContinuous(&I2CLasers, &LaserBackLeft)) {
					laserReqMask = 6;
				}
				break;
			case 6:	// enable channel 1
				if (TCA9548A_SetChannels(&I2CLasers, &tca9548, TCA9548A_CH1)) {
					laserReqMask = 7;
				}
				break;
			case 7:
				if (VL53_ReadRangeSingle(&I2CLasers, &LaserBackRight)) {
					laserReqMask = 11;
				}
				break;
			case 8:
				if (VL53_StartContinuous(&I2CLasers, &LaserBackRight, 30)) {
					laserReqMask = 9;
				}
				break;
			case 9:
				if (VL53_readRangeContinuousMillimeters(&I2CLasers, &LaserBackRight)) {
					laserReqMask = 10;
				}
				break;
			case 10:
				if (VL53_StopContinuous(&I2CLasers, &LaserBackRight)) {
					laserReqMask = 11;
				}
				break;
			case 11:	// enable channel 2
				if (TCA9548A_SetChannels(&I2CLasers, &tca9548, TCA9548A_CH2)) {
					laserReqMask = 12;
				}
				break;
			case 12:
				if (VL53_ReadRangeSingle(&I2CLasers, &LaserMidRight)) {
					laserReqMask = 16;
				}
				break;
			case 13:	// init laser mid right
				if (VL53_StartContinuous(&I2CLasers, &LaserMidRight, 30)) {
					laserReqMask = 14;
				}
				break;
			case 14:
				if (VL53_readRangeContinuousMillimeters(&I2CLasers, &LaserMidRight)) {
					laserReqMask = 15;
				}
				break;
			case 15:
				if (VL53_StopContinuous(&I2CLasers, &LaserMidRight)) {
					laserReqMask = 16;
				}
				break;
			case 16:	// enable channel 3
				if (TCA9548A_SetChannels(&I2CLasers, &tca9548, TCA9548A_CH3)) {
					laserReqMask = 17;
				}
				break;
			case 17:
				if (VL53_ReadRangeSingle(&I2CLasers, &LaserFrontRight)) {
					laserReqMask = 21;
				}
				break;
			case 18:	// init laser front right
				if (VL53_StartContinuous(&I2CLasers, &LaserFrontRight, 30)) {
					laserReqMask = 19;
				}
				break;
			case 19:
				if (VL53_readRangeContinuousMillimeters(&I2CLasers, &LaserFrontRight)) {
					laserReqMask = 20;
				}
				break;
			case 20:
				if (VL53_StopContinuous(&I2CLasers, &LaserFrontRight)) {
					laserReqMask = 21;
				}
				break;
			case 21:	// enable channel 4
				if (TCA9548A_SetChannels(&I2CLasers, &tca9548, TCA9548A_CH4)) {
					laserReqMask = 22;
				}
				break;
			case 22:
				if (VL53_ReadRangeSingle(&I2CLasers, &LaserFrontLeft)) {
					laserReqMask = 26;
				}
				break;
			case 23:	// init laser front left
				if (VL53_StartContinuous(&I2CLasers, &LaserFrontLeft, 30)) {
					laserReqMask = 24;
				}
				break;
			case 24:
				if (VL53_readRangeContinuousMillimeters(&I2CLasers, &LaserFrontLeft)) {
					laserReqMask = 25;
				}
				break;
			case 25:
				if (VL53_StopContinuous(&I2CLasers, &LaserFrontLeft)) {
					laserReqMask = 26;
				}
				break;
			case 26:	// enable channel 5
				if (TCA9548A_SetChannels(&I2CLasers, &tca9548, TCA9548A_CH5)) {
					laserReqMask = 27;
				}
				break;
			case 27:
				if (VL53_ReadRangeSingle(&I2CLasers, &LaserMidLeft)) {
					laserReqMask = 0;
				}
				break;
			case 28:	// init laser front left
				if (VL53_StartContinuous(&I2CLasers, &LaserMidLeft, 30)) {
					laserReqMask = 29;
				}
				break;
			case 29:
				if (VL53_readRangeContinuousMillimeters(&I2CLasers, &LaserMidLeft)) {
					laserReqMask = 30;
				}
				break;
			case 30:
				if (VL53_StopContinuous(&I2CLasers, &LaserMidLeft)) {
					laserReqMask = 0;
				}
				break;
			default:
				laserReqMask = 0;
				break;
			}
		}
		if (sensorReqMask && ADXL_REQ_MASK) {
			uint8_t st;
			st = ADXL345_GetData(&I2CSensors, &adxl345);
			if (st) {
				sensorReqMask &= ~ADXL_REQ_MASK;
				sensorReqMask |= QMC_REQ_MASK;
			}
		}
		if (sensorReqMask && ITG_REQ_MASK) {
			uint8_t st;
			st = ITG3205_GetData(&I2CSensors, &itg3205);
			if (st) {
				sensorReqMask &= ~ITG_REQ_MASK;
				sensorReqMask |= QMC_REQ_MASK;
			}
		}
		if (sensorReqMask && QMC_REQ_MASK) {
			uint8_t st;
			st = QMC5883L_GetData(&I2CSensors, &qmc5883);
			if (st) {
				sensorReqMask &= ~QMC_REQ_MASK;
				(void) MadgwickAHRS_9(adxl345.data.X,
				    adxl345.data.Y,
				    adxl345.data.Z,
				    itg3205.data.X,
				    itg3205.data.Y,
				    itg3205.data.X,
				    qmc5883.data.X,
				    qmc5883.data.Y,
				    qmc5883.data.Z,
				    100,
				    &basQuat1.quattro);
				(void) MahonyAHRS_9(adxl345.data.X,
				    adxl345.data.Y,
				    adxl345.data.Z,
				    itg3205.data.X,
				    itg3205.data.Y,
				    itg3205.data.X,
				    qmc5883.data.X,
				    qmc5883.data.Y,
				    qmc5883.data.Z,
				    1,
				    1,
				    100,
				    &basQuat2.quattro);
				(void) QuaternionCalcHabr_9(adxl345.data.X,
				    adxl345.data.Y,
				    adxl345.data.Z,
				    itg3205.data.X,
				    itg3205.data.Y,
				    itg3205.data.X,
				    qmc5883.data.X,
				    qmc5883.data.Y,
				    qmc5883.data.Z,
				    100,
				    &basQuat3.quattro);
			}
		}
		if (sensorReqMask && BME_REQ_MASK) {
			uint8_t st;
			st = BME280_GetData(&I2CSensors, &bme280);
			if (st) {
				sensorReqMask &= ~BME_REQ_MASK;
			}
		}
		if (sensorReqMask && INA_REQ_MASK) {
			uint8_t st;
			st = INA219_GetData(&I2CSensors, &ina219);
			if (st) {
				BusRequestOff(sensorReqMask, INA_REQ_MASK);
			}
		}
		DriveForward();
		//int32_t tmp = 1000 - (ambientLightLevel - 100) / 2;
		//headlightsLevel = Min(tmp, 0);
		//headlightsLevel = Max(tmp, 1000);
		//TIM8->CCR3 = headlightsLevel;

		//CCR1 - right
		//CCR2 - left
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		//LL_IWDG_ReloadCounter(IWDG);
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	LL_FLASH_SetLatency(LL_FLASH_LATENCY_5);
	while (LL_FLASH_GetLatency() != LL_FLASH_LATENCY_5) {
	}
	LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
	LL_RCC_HSE_Enable();

	/* Wait till HSE is ready */
	while (LL_RCC_HSE_IsReady() != 1) {

	}
	LL_RCC_LSI_Enable();

	/* Wait till LSI is ready */
	while (LL_RCC_LSI_IsReady() != 1) {

	}
	LL_RCC_HSE_EnableCSS();
	LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_8, 320,
	LL_RCC_PLLP_DIV_2);
	LL_RCC_PLL_ConfigDomain_48M(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_8, 320,
	LL_RCC_PLLQ_DIV_8);
	LL_RCC_PLL_Enable();

	/* Wait till PLL is ready */
	while (LL_RCC_PLL_IsReady() != 1) {

	}
	while (LL_PWR_IsActiveFlag_VOS() == 0) {
	}
	LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
	LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_4);
	LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_2);
	LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

	/* Wait till System clock is ready */
	while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL) {

	}
	LL_Init1msTick(160000000);
	LL_SetSystemCoreClock(160000000);
}

/* USER CODE BEGIN 4 */
void DriveStop(void) {
	LL_GPIO_ResetOutputPin(Drive_A1_GPIO_Port,
	Drive_A1_Pin | Drive_A2_Pin | Drive_B1_Pin | Drive_B2_Pin);
}

void DriveForward(void) {
	LL_GPIO_ResetOutputPin(Drive_A1_GPIO_Port, Drive_A2_Pin | Drive_B2_Pin);
	LL_GPIO_SetOutputPin(Drive_A1_GPIO_Port, Drive_A1_Pin | Drive_B1_Pin);
}
void GoBackward(void) {
	LL_GPIO_ResetOutputPin(Drive_A1_GPIO_Port, Drive_A1_Pin | Drive_B1_Pin);
	LL_GPIO_SetOutputPin(Drive_A1_GPIO_Port, Drive_A2_Pin | Drive_B2_Pin);
}
void GoRight(void) {
	LL_GPIO_ResetOutputPin(Drive_A1_GPIO_Port, Drive_A1_Pin | Drive_B2_Pin);
	LL_GPIO_SetOutputPin(Drive_A1_GPIO_Port, Drive_A2_Pin | Drive_B1_Pin);
}
void DriveLeft(void) {
	LL_GPIO_ResetOutputPin(Drive_A1_GPIO_Port, Drive_A2_Pin | Drive_B1_Pin);
	LL_GPIO_SetOutputPin(Drive_A1_GPIO_Port, Drive_A1_Pin | Drive_B2_Pin);
}

void HardwareInit(void) {

	TIM3->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E;
	TIM3->CR1 |= TIM_CR1_CEN;

	TIM4->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E;
	TIM4->CR1 |= TIM_CR1_CEN;

	TIM7->DIER |= TIM_DIER_UIE;
	TIM7->CR1 |= TIM_CR1_CEN;

	TIM10->CCER |= TIM_CCER_CC1E;
	TIM10->DIER |= TIM_DIER_UIE;
	TIM10->CR1 |= TIM_CR1_CEN;

	TIM11->CCER |= TIM_CCER_CC1E;
	TIM11->DIER |= TIM_DIER_UIE;
	TIM11->CR1 |= TIM_CR1_CEN;

	TIM12->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E;
	TIM12->DIER |= TIM_DIER_UIE | TIM_DIER_CC2IE;

	TIM8->CCER |= TIM_CCER_CC3E;
	TIM8->BDTR |= TIM_BDTR_MOE;
	TIM8->CR1 |= TIM_CR1_CEN;

	TIM9->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E;
	TIM9->DIER |= TIM_DIER_UIE;
	TIM9->CR1 |= TIM_CR1_CEN;

	TIM13->CCER |= TIM_CCER_CC1E;
	TIM13->DIER |= TIM_DIER_UIE | TIM_DIER_CC1IE;
	TIM13->CR1 |= TIM_CR1_CEN;

	I2C1->CR2 |= I2C_CR2_ITERREN | I2C_CR2_ITEVTEN;
	I2C1->CR1 |= I2C_CR1_PE;

	/*I2C2->CR2 |= I2C_CR2_ITERREN | I2C_CR2_ITEVTEN;
	 I2C2->CR1 |= I2C_CR1_PE;*/

	I2C3->CR2 |= I2C_CR2_ITERREN | I2C_CR2_ITEVTEN;
	I2C3->CR1 |= I2C_CR1_PE;

	ADC1->CR1 |= ADC_CR1_EOCIE | ADC_CR1_OVRIE;
	ADC1->CR2 |= ADC_CR2_ADON;
	LL_mDelay(5);
	ADC1->CR2 |= ADC_CR2_SWSTART;
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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
