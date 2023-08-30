/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stddef.h"
#include "stm32f4xx_ll_adc.h"
#include "stm32f4xx_ll_crc.h"
#include "stm32f4xx_ll_i2c.h"
#include "stm32f4xx_ll_iwdg.h"
#include "stm32f4xx_ll_rcc.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_system.h"
#include "stm32f4xx_ll_exti.h"
#include "stm32f4xx_ll_cortex.h"
#include "stm32f4xx_ll_utils.h"
#include "stm32f4xx_ll_pwr.h"
#include "stm32f4xx_ll_dma.h"
#include "stm32f4xx_ll_rng.h"
#include "stm32f4xx_ll_rtc.h"
#include "stm32f4xx_ll_spi.h"
#include "stm32f4xx_ll_tim.h"
#include "stm32f4xx_ll_usart.h"
#include "stm32f4xx_ll_gpio.h"

#if defined(USE_FULL_ASSERT)
#include "stm32_assert.h"
#endif /* USE_FULL_ASSERT */

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "HW_Init.h"
#include "Peripherals/I2C/MyI2C.h"
#include "Peripherals/SPI/MySPI.h"
#include "Peripherals/USART/MyUSART.h"
#include "ADXL345/ADXL345.h"
#include "ITG3205/ITG3205.h"
#include "QMC5883L/QMC5883L.h"
#include "BME280/BME280.h"
#include "INA219/INA219.h"
#include "TCA9548A/TCA9548A.h"
#include "SG90/SG90.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
	typedef struct Camera {
		uint16_t posH;
		uint16_t posV;
		Servo_t srvLR;
		Servo_t srvUD;
	} Camera_t;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Laser4_SHUT_Pin LL_GPIO_PIN_2
#define Laser4_SHUT_GPIO_Port GPIOE
#define Laser3_SHUT_Pin LL_GPIO_PIN_3
#define Laser3_SHUT_GPIO_Port GPIOE
#define Laser5_SHUT_Pin LL_GPIO_PIN_4
#define Laser5_SHUT_GPIO_Port GPIOE
#define DRIVE1_PWM_Pin LL_GPIO_PIN_5
#define DRIVE1_PWM_GPIO_Port GPIOE
#define DRIVE2_PWM_Pin LL_GPIO_PIN_6
#define DRIVE2_PWM_GPIO_Port GPIOE
#define Laser_6_SHUT_Pin LL_GPIO_PIN_13
#define Laser_6_SHUT_GPIO_Port GPIOC
#define Drive_A1_Pin LL_GPIO_PIN_0
#define Drive_A1_GPIO_Port GPIOC
#define Drive_A2_Pin LL_GPIO_PIN_1
#define Drive_A2_GPIO_Port GPIOC
#define Drive_B1_Pin LL_GPIO_PIN_2
#define Drive_B1_GPIO_Port GPIOC
#define Drive_B2_Pin LL_GPIO_PIN_3
#define Drive_B2_GPIO_Port GPIOC
#define KEY_1_Pin LL_GPIO_PIN_0
#define KEY_1_GPIO_Port GPIOA
#define KEY_1_EXTI_IRQn EXTI0_IRQn
#define LED_Pin LL_GPIO_PIN_1
#define LED_GPIO_Port GPIOA
#define SPI1_NSS_LORA_Pin LL_GPIO_PIN_4
#define SPI1_NSS_LORA_GPIO_Port GPIOA
#define SPI1_SCK_LORA_Pin LL_GPIO_PIN_5
#define SPI1_SCK_LORA_GPIO_Port GPIOA
#define SPI1_MISO_LORA_Pin LL_GPIO_PIN_6
#define SPI1_MISO_LORA_GPIO_Port GPIOA
#define SPI1_MOSI_LORA_Pin LL_GPIO_PIN_7
#define SPI1_MOSI_LORA_GPIO_Port GPIOA
#define LIGHT_Pin LL_GPIO_PIN_4
#define LIGHT_GPIO_Port GPIOC
#define LoRa_RST_Pin LL_GPIO_PIN_5
#define LoRa_RST_GPIO_Port GPIOC
#define PCA_RST_Pin LL_GPIO_PIN_0
#define PCA_RST_GPIO_Port GPIOB
#define Laser_6_INT_Pin LL_GPIO_PIN_10
#define Laser_6_INT_GPIO_Port GPIOE
#define Laser_6_INT_EXTI_IRQn EXTI15_10_IRQn
#define Laser_5_INT_Pin LL_GPIO_PIN_11
#define Laser_5_INT_GPIO_Port GPIOE
#define Laser_5_INT_EXTI_IRQn EXTI15_10_IRQn
#define Laser_2_INT_Pin LL_GPIO_PIN_12
#define Laser_2_INT_GPIO_Port GPIOE
#define Laser_2_INT_EXTI_IRQn EXTI15_10_IRQn
#define Laser_4_INT_Pin LL_GPIO_PIN_13
#define Laser_4_INT_GPIO_Port GPIOE
#define Laser_4_INT_EXTI_IRQn EXTI15_10_IRQn
#define Laser_1_INT_Pin LL_GPIO_PIN_14
#define Laser_1_INT_GPIO_Port GPIOE
#define Laser_1_INT_EXTI_IRQn EXTI15_10_IRQn
#define Laser_3_INT_Pin LL_GPIO_PIN_15
#define Laser_3_INT_GPIO_Port GPIOE
#define Laser_3_INT_EXTI_IRQn EXTI15_10_IRQn
#define OLED_SCL2_Pin LL_GPIO_PIN_10
#define OLED_SCL2_GPIO_Port GPIOB
#define OLED_SDA2_Pin LL_GPIO_PIN_11
#define OLED_SDA2_GPIO_Port GPIOB
#define USM_TRIG_Pin LL_GPIO_PIN_14
#define USM_TRIG_GPIO_Port GPIOB
#define USM_ECHO_Pin LL_GPIO_PIN_15
#define USM_ECHO_GPIO_Port GPIOB
#define LoRa_DI0_Pin LL_GPIO_PIN_8
#define LoRa_DI0_GPIO_Port GPIOD
#define LoRa_DI0_EXTI_IRQn EXTI9_5_IRQn
#define LoRa_DI1_Pin LL_GPIO_PIN_9
#define LoRa_DI1_GPIO_Port GPIOD
#define LoRa_DI1_EXTI_IRQn EXTI9_5_IRQn
#define ENC2_A_Pin LL_GPIO_PIN_12
#define ENC2_A_GPIO_Port GPIOD
#define ENC2_B_Pin LL_GPIO_PIN_13
#define ENC2_B_GPIO_Port GPIOD
#define ENC1_A_Pin LL_GPIO_PIN_6
#define ENC1_A_GPIO_Port GPIOC
#define ENC1_B_Pin LL_GPIO_PIN_7
#define ENC1_B_GPIO_Port GPIOC
#define PWM_LED_Pin LL_GPIO_PIN_8
#define PWM_LED_GPIO_Port GPIOC
#define VL53_SDA3_Pin LL_GPIO_PIN_9
#define VL53_SDA3_GPIO_Port GPIOC
#define VL53_SCL3_Pin LL_GPIO_PIN_8
#define VL53_SCL3_GPIO_Port GPIOA
#define SPI3_NSS_FLASH_Pin LL_GPIO_PIN_15
#define SPI3_NSS_FLASH_GPIO_Port GPIOA
#define GPS_EN_Pin LL_GPIO_PIN_3
#define GPS_EN_GPIO_Port GPIOD
#define GPS_PPT_Pin LL_GPIO_PIN_4
#define GPS_PPT_GPIO_Port GPIOD
#define LoRa_DI2_Pin LL_GPIO_PIN_5
#define LoRa_DI2_GPIO_Port GPIOD
#define LoRa_DI2_EXTI_IRQn EXTI9_5_IRQn
#define LoRa_DI4_Pin LL_GPIO_PIN_6
#define LoRa_DI4_GPIO_Port GPIOD
#define LoRa_DI4_EXTI_IRQn EXTI9_5_IRQn
#define LoRa_DI3_Pin LL_GPIO_PIN_7
#define LoRa_DI3_GPIO_Port GPIOD
#define LoRa_DI3_EXTI_IRQn EXTI9_5_IRQn
#define SPI3_SCK_FLASH_Pin LL_GPIO_PIN_3
#define SPI3_SCK_FLASH_GPIO_Port GPIOB
#define SPI3_MISO_FLASH_Pin LL_GPIO_PIN_4
#define SPI3_MISO_FLASH_GPIO_Port GPIOB
#define SPI3_MOSI_FLASH_Pin LL_GPIO_PIN_5
#define SPI3_MOSI_FLASH_GPIO_Port GPIOB
#define Sensor_SCL_Pin LL_GPIO_PIN_6
#define Sensor_SCL_GPIO_Port GPIOB
#define Sensor_SDA_Pin LL_GPIO_PIN_7
#define Sensor_SDA_GPIO_Port GPIOB
#define TIM10_CamServoLR_Pin LL_GPIO_PIN_8
#define TIM10_CamServoLR_GPIO_Port GPIOB
#define TIM11_CamServoUD_Pin LL_GPIO_PIN_9
#define TIM11_CamServoUD_GPIO_Port GPIOB
#define Laser2_SHUT_Pin LL_GPIO_PIN_0
#define Laser2_SHUT_GPIO_Port GPIOE
#define Laser1_SHUT_Pin LL_GPIO_PIN_1
#define Laser1_SHUT_GPIO_Port GPIOE
#ifndef NVIC_PRIORITYGROUP_0
#define NVIC_PRIORITYGROUP_0         ((uint32_t)0x00000007) /*!< 0 bit  for pre-emption priority,
                                                                 4 bits for subpriority */
#define NVIC_PRIORITYGROUP_1         ((uint32_t)0x00000006) /*!< 1 bit  for pre-emption priority,
                                                                 3 bits for subpriority */
#define NVIC_PRIORITYGROUP_2         ((uint32_t)0x00000005) /*!< 2 bits for pre-emption priority,
                                                                 2 bits for subpriority */
#define NVIC_PRIORITYGROUP_3         ((uint32_t)0x00000004) /*!< 3 bits for pre-emption priority,
                                                                 1 bit  for subpriority */
#define NVIC_PRIORITYGROUP_4         ((uint32_t)0x00000003) /*!< 4 bits for pre-emption priority,
                                                                 0 bit  for subpriority */
#endif

/* USER CODE BEGIN Private defines */
#define SG90_MIN	400
#define SG90_MAX	2600

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
