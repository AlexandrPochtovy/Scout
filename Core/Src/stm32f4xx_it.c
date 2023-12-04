/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    stm32f4xx_it.c
 * @brief   Interrupt Service Routines.
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
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
uint8_t pidSelect = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/

/* USER CODE BEGIN EV */
extern volatile uint32_t mainTimeTick;
extern volatile uint32_t DevicesEnableMask;
extern volatile uint32_t laserReqMask;
extern I2C_IRQ_Conn_t I2CSensors;
extern I2C_IRQ_Conn_t I2CLasers;
extern USART_FullDuplex_t USART_Orange;
extern USART_FullDuplex_t USART_CLI;
extern USART_FullDuplex_t USART_UNO;
extern USART_FullDuplex_t UART_GPS;
extern SPI_Conn_TWO_t SPI_LoRa;
extern SPI_Conn_TWO_t SPI_Flash;
extern Camera_t camera;
extern HC_SR04_t USMrange;
extern uint16_t headLightsLevel;
extern uint16_t ambientLightLevel;
extern uint16_t mcuTemp;
extern uint16_t mcuVoltage;
extern Drive_t Drive;
extern Drive_t Drive;
extern pidData_t PidAVR_L;
extern pidData_t PidAVR_R;
extern pidS_t PidSimple_L;
extern pidS_t PidSimple_R;
extern pidF_t PidFilter_L;
extern pidF_t PidFilter_R;
extern PID_M_t PidMoto_L;
extern PID_M_t PidMoto_R;
extern PID_MF_t PidMotoFilter_L;
extern PID_MF_t PidMotoFilter_R;
extern uint16_t leftWheelPWM;
extern uint16_t rightWheelPWM;
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
 * @brief This function handles Non maskable interrupt.
 */
void NMI_Handler(void)
{
	/* USER CODE BEGIN NonMaskableInt_IRQn 0 */

	/* USER CODE END NonMaskableInt_IRQn 0 */
	/* USER CODE BEGIN NonMaskableInt_IRQn 1 */
	while (1)
	{
	}
	/* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
 * @brief This function handles Hard fault interrupt.
 */
void HardFault_Handler(void)
{
	/* USER CODE BEGIN HardFault_IRQn 0 */

	/* USER CODE END HardFault_IRQn 0 */
	while (1)
	{
		/* USER CODE BEGIN W1_HardFault_IRQn 0 */
		/* USER CODE END W1_HardFault_IRQn 0 */
	}
}

/**
 * @brief This function handles Memory management fault.
 */
void MemManage_Handler(void)
{
	/* USER CODE BEGIN MemoryManagement_IRQn 0 */

	/* USER CODE END MemoryManagement_IRQn 0 */
	while (1)
	{
		/* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
		/* USER CODE END W1_MemoryManagement_IRQn 0 */
	}
}

/**
 * @brief This function handles Pre-fetch fault, memory access fault.
 */
void BusFault_Handler(void)
{
	/* USER CODE BEGIN BusFault_IRQn 0 */

	/* USER CODE END BusFault_IRQn 0 */
	while (1)
	{
		/* USER CODE BEGIN W1_BusFault_IRQn 0 */
		/* USER CODE END W1_BusFault_IRQn 0 */
	}
}

/**
 * @brief This function handles Undefined instruction or illegal state.
 */
void UsageFault_Handler(void)
{
	/* USER CODE BEGIN UsageFault_IRQn 0 */

	/* USER CODE END UsageFault_IRQn 0 */
	while (1)
	{
		/* USER CODE BEGIN W1_UsageFault_IRQn 0 */
		/* USER CODE END W1_UsageFault_IRQn 0 */
	}
}

/**
 * @brief This function handles System service call via SWI instruction.
 */
void SVC_Handler(void)
{
	/* USER CODE BEGIN SVCall_IRQn 0 */

	/* USER CODE END SVCall_IRQn 0 */
	/* USER CODE BEGIN SVCall_IRQn 1 */

	/* USER CODE END SVCall_IRQn 1 */
}

/**
 * @brief This function handles Debug monitor.
 */
void DebugMon_Handler(void)
{
	/* USER CODE BEGIN DebugMonitor_IRQn 0 */

	/* USER CODE END DebugMonitor_IRQn 0 */
	/* USER CODE BEGIN DebugMonitor_IRQn 1 */

	/* USER CODE END DebugMonitor_IRQn 1 */
}

/**
 * @brief This function handles Pendable request for system service.
 */
void PendSV_Handler(void)
{
	/* USER CODE BEGIN PendSV_IRQn 0 */

	/* USER CODE END PendSV_IRQn 0 */
	/* USER CODE BEGIN PendSV_IRQn 1 */

	/* USER CODE END PendSV_IRQn 1 */
}

/**
 * @brief This function handles System tick timer.
 */
void SysTick_Handler(void)
{
	/* USER CODE BEGIN SysTick_IRQn 0 */

	/* USER CODE END SysTick_IRQn 0 */

	/* USER CODE BEGIN SysTick_IRQn 1 */
	++mainTimeTick;
	if (mainTimeTick % IMU_POOL_PERIOD == 0) {
		DevicesEnableMask |= ADXL_REQ_MASK | ITG_REQ_MASK | QMC_REQ_MASK;
	}
	if ((mainTimeTick % BME_POOL_PERIOD) == 0) {
		DevicesEnableMask |= BME_REQ_MASK;
	}
	if ((mainTimeTick % INA_POOL_PERIOD) == 0) {
		DevicesEnableMask |= INA_REQ_MASK;
	}
	if ((mainTimeTick % VL53_POOL_PERIOD) == 0) {
		laserReqMask = 1;
	}
	/* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
 * @brief This function handles EXTI line0 interrupt.
 */
void EXTI0_IRQHandler(void)
{
	/* USER CODE BEGIN EXTI0_IRQn 0 */

	/* USER CODE END EXTI0_IRQn 0 */
	if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_0) != RESET)
	  {
		LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_0);
		/* USER CODE BEGIN LL_EXTI_LINE_0 */

		/* USER CODE END LL_EXTI_LINE_0 */
	}
	/* USER CODE BEGIN EXTI0_IRQn 1 */

	/* USER CODE END EXTI0_IRQn 1 */
}

/**
 * @brief This function handles ADC1, ADC2 and ADC3 global interrupts.
 */
void ADC_IRQHandler(void)
{
	/* USER CODE BEGIN ADC_IRQn 0 */

	/* USER CODE END ADC_IRQn 0 */

	/* USER CODE BEGIN ADC_IRQn 1 */
	(void) ADC->CSR;
	volatile uint32_t sr = ADC1->SR;
	if (sr && ADC_SR_OVR) { //clear overrun
		ADC1->SR &= ~ADC_SR_OVR;
	}
	if (sr && ADC_SR_EOC) { //clear end conversion
		ambientLightLevel = (uint16_t) ADC1->DR;
		ADC1->SR &= ~ADC_SR_EOC;
	}
	/* USER CODE END ADC_IRQn 1 */
}

/**
 * @brief This function handles EXTI line[9:5] interrupts.
 */
void EXTI9_5_IRQHandler(void)
{
	/* USER CODE BEGIN EXTI9_5_IRQn 0 */

	/* USER CODE END EXTI9_5_IRQn 0 */
	if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_5) != RESET)
	  {
		LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_5);
		/* USER CODE BEGIN LL_EXTI_LINE_5 */

		/* USER CODE END LL_EXTI_LINE_5 */
	}
	if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_6) != RESET)
	  {
		LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_6);
		/* USER CODE BEGIN LL_EXTI_LINE_6 */

		/* USER CODE END LL_EXTI_LINE_6 */
	}
	if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_7) != RESET)
	  {
		LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_7);
		/* USER CODE BEGIN LL_EXTI_LINE_7 */

		/* USER CODE END LL_EXTI_LINE_7 */
	}
	if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_8) != RESET)
	  {
		LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_8);
		/* USER CODE BEGIN LL_EXTI_LINE_8 */

		/* USER CODE END LL_EXTI_LINE_8 */
	}
	if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_9) != RESET)
	  {
		LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_9);
		/* USER CODE BEGIN LL_EXTI_LINE_9 */

		/* USER CODE END LL_EXTI_LINE_9 */
	}
	/* USER CODE BEGIN EXTI9_5_IRQn 1 */

	/* USER CODE END EXTI9_5_IRQn 1 */
}

/**
 * @brief This function handles TIM1 break interrupt and TIM9 global interrupt.
 */
void TIM1_BRK_TIM9_IRQHandler(void)
{
	/* USER CODE BEGIN TIM1_BRK_TIM9_IRQn 0 */

	/* USER CODE END TIM1_BRK_TIM9_IRQn 0 */

	/* USER CODE BEGIN TIM1_BRK_TIM9_IRQn 1 */
	volatile uint32_t sr = TIM9->SR;
	if (sr & TIM_SR_UIF) {
		TIM9->CCR1 = SimpleRamp_IT(TIM9->CCR1, rightWheelPWM, 0, 3999, 1);
		TIM9->CCR2 = SimpleRamp_IT(TIM9->CCR2, leftWheelPWM, 0, 3999, 1);
		TIM9->SR &= ~TIM_SR_UIF;
	}
	/* USER CODE END TIM1_BRK_TIM9_IRQn 1 */
}

/**
 * @brief This function handles TIM1 update interrupt and TIM10 global interrupt.
 */
void TIM1_UP_TIM10_IRQHandler(void)
{
	/* USER CODE BEGIN TIM1_UP_TIM10_IRQn 0 */

	/* USER CODE END TIM1_UP_TIM10_IRQn 0 */

	/* USER CODE BEGIN TIM1_UP_TIM10_IRQn 1 */
	volatile uint32_t sr = TIM10->SR;
	if (sr && TIM_SR_UIF) {
		TIM10->CCR1 = SimpleRamp_IT(TIM10->CCR1, camera.posH, camera.srvLR.min, camera.srvLR.max, camera.srvLR.step);
		TIM10->SR &= ~TIM_SR_UIF;
	}

	/* USER CODE END TIM1_UP_TIM10_IRQn 1 */
}

/**
 * @brief This function handles TIM1 trigger and commutation interrupts and TIM11 global interrupt.
 */
void TIM1_TRG_COM_TIM11_IRQHandler(void)
{
	/* USER CODE BEGIN TIM1_TRG_COM_TIM11_IRQn 0 */

	/* USER CODE END TIM1_TRG_COM_TIM11_IRQn 0 */

	/* USER CODE BEGIN TIM1_TRG_COM_TIM11_IRQn 1 */
	volatile uint32_t sr = TIM11->SR;
	if (sr && TIM_SR_UIF) {
		TIM11->CCR1 = SimpleRamp_IT(TIM11->CCR1, camera.posV, camera.srvUD.min, camera.srvUD.max, camera.srvUD.step);
		TIM11->SR &= ~TIM_SR_UIF;
	}
	/* USER CODE END TIM1_TRG_COM_TIM11_IRQn 1 */
}

/**
 * @brief This function handles I2C1 event interrupt.
 */
void I2C1_EV_IRQHandler(void)
{
	/* USER CODE BEGIN I2C1_EV_IRQn 0 */

	/* USER CODE END I2C1_EV_IRQn 0 */

	/* USER CODE BEGIN I2C1_EV_IRQn 1 */
	I2C_Alt_IRQ_CallBack(&I2CSensors);
	//I2C_Raw_IRQ_CallBack(&I2CSensors);
	/* USER CODE END I2C1_EV_IRQn 1 */
}

/**
 * @brief This function handles I2C1 error interrupt.
 */
void I2C1_ER_IRQHandler(void)
{
	/* USER CODE BEGIN I2C1_ER_IRQn 0 */

	/* USER CODE END I2C1_ER_IRQn 0 */

	/* USER CODE BEGIN I2C1_ER_IRQn 1 */
	I2C_ERR_IRQ_CallBack(&I2CSensors);
	/* USER CODE END I2C1_ER_IRQn 1 */
}

/**
 * @brief This function handles I2C2 event interrupt.
 */
void I2C2_EV_IRQHandler(void)
{
	/* USER CODE BEGIN I2C2_EV_IRQn 0 */

	/* USER CODE END I2C2_EV_IRQn 0 */

	/* USER CODE BEGIN I2C2_EV_IRQn 1 */

	/* USER CODE END I2C2_EV_IRQn 1 */
}

/**
 * @brief This function handles I2C2 error interrupt.
 */
void I2C2_ER_IRQHandler(void)
{
	/* USER CODE BEGIN I2C2_ER_IRQn 0 */

	/* USER CODE END I2C2_ER_IRQn 0 */

	/* USER CODE BEGIN I2C2_ER_IRQn 1 */

	/* USER CODE END I2C2_ER_IRQn 1 */
}

/**
 * @brief This function handles SPI1 global interrupt.
 */
void SPI1_IRQHandler(void)
{
	/* USER CODE BEGIN SPI1_IRQn 0 */

	/* USER CODE END SPI1_IRQn 0 */
	/* USER CODE BEGIN SPI1_IRQn 1 */
	SPI_IRQ_TWO_CallBack(&SPI_LoRa);
	/* USER CODE END SPI1_IRQn 1 */
}

/**
 * @brief This function handles USART1 global interrupt.
 */
void USART1_IRQHandler(void)
{
	/* USER CODE BEGIN USART1_IRQn 0 */

	/* USER CODE END USART1_IRQn 0 */
	/* USER CODE BEGIN USART1_IRQn 1 */
	USART_EV_IRQ_CallBack(&USART_Orange);
	/* USER CODE END USART1_IRQn 1 */
}

/**
 * @brief This function handles USART2 global interrupt.
 */
void USART2_IRQHandler(void)
{
	/* USER CODE BEGIN USART2_IRQn 0 */

	/* USER CODE END USART2_IRQn 0 */
	/* USER CODE BEGIN USART2_IRQn 1 */
	USART_EV_IRQ_CallBack(&USART_CLI);
	/* USER CODE END USART2_IRQn 1 */
}

/**
 * @brief This function handles USART3 global interrupt.
 */
void USART3_IRQHandler(void)
{
	/* USER CODE BEGIN USART3_IRQn 0 */

	/* USER CODE END USART3_IRQn 0 */
	/* USER CODE BEGIN USART3_IRQn 1 */
	USART_EV_IRQ_CallBack(&USART_UNO);
	/* USER CODE END USART3_IRQn 1 */
}

/**
 * @brief This function handles EXTI line[15:10] interrupts.
 */
void EXTI15_10_IRQHandler(void)
{
	/* USER CODE BEGIN EXTI15_10_IRQn 0 */

	/* USER CODE END EXTI15_10_IRQn 0 */
	if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_10) != RESET)
	  {
		LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_10);
		/* USER CODE BEGIN LL_EXTI_LINE_10 */

		/* USER CODE END LL_EXTI_LINE_10 */
	}
	if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_11) != RESET)
	  {
		LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_11);
		/* USER CODE BEGIN LL_EXTI_LINE_11 */

		/* USER CODE END LL_EXTI_LINE_11 */
	}
	if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_12) != RESET)
	  {
		LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_12);
		/* USER CODE BEGIN LL_EXTI_LINE_12 */

		/* USER CODE END LL_EXTI_LINE_12 */
	}
	if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_13) != RESET)
	  {
		LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_13);
		/* USER CODE BEGIN LL_EXTI_LINE_13 */

		/* USER CODE END LL_EXTI_LINE_13 */
	}
	if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_14) != RESET)
	  {
		LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_14);
		/* USER CODE BEGIN LL_EXTI_LINE_14 */

		/* USER CODE END LL_EXTI_LINE_14 */
	}
	if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_15) != RESET)
	  {
		LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_15);
		/* USER CODE BEGIN LL_EXTI_LINE_15 */

		/* USER CODE END LL_EXTI_LINE_15 */
	}
	/* USER CODE BEGIN EXTI15_10_IRQn 1 */

	/* USER CODE END EXTI15_10_IRQn 1 */
}

/**
 * @brief This function handles TIM8 break interrupt and TIM12 global interrupt.
 */
void TIM8_BRK_TIM12_IRQHandler(void)
{
	/* USER CODE BEGIN TIM8_BRK_TIM12_IRQn 0 */

	/* USER CODE END TIM8_BRK_TIM12_IRQn 0 */

	/* USER CODE BEGIN TIM8_BRK_TIM12_IRQn 1 */
	uint32_t sr = TIM12->SR;
	if (sr & TIM_SR_CC2IF) {
		if (TIM12->CCER & TIM_CCER_CC2P) { //полярность 1
			USMrange.stop = TIM12->CCR2;
			HC_SR04DistanceSimpleCalc(&USMrange, 65535);
			TIM12->CCER &= ~TIM_CCER_CC2P;
		}
		else { //полярность 1
			USMrange.start = TIM12->CCR2;
			TIM12->CCER |= TIM_CCER_CC2P;
		}
	}
	TIM12->SR = 0;

	/* USER CODE END TIM8_BRK_TIM12_IRQn 1 */
}

/**
 * @brief This function handles TIM8 update interrupt and TIM13 global interrupt.
 */
void TIM8_UP_TIM13_IRQHandler(void)
{
	/* USER CODE BEGIN TIM8_UP_TIM13_IRQn 0 */

	/* USER CODE END TIM8_UP_TIM13_IRQn 0 */

	/* USER CODE BEGIN TIM8_UP_TIM13_IRQn 1 */
	volatile uint32_t sr;
	sr = TIM8->SR;
	if (sr & TIM_SR_UIF) {
		TIM8->CCR3 = SimpleRamp_IT(TIM8->CCR3, headLightsLevel, 0, 1999, 1);
		TIM8->SR &= ~TIM_SR_UIF;
	}
	sr = TIM13->SR;
	if (sr & TIM_SR_UIF) {
		switch (pidSelect) {
		case 0:
			rightWheelPWM = leftWheelPWM = 0;
			break;
		case 1:
			leftWheelPWM = Max(Min(pid_Controller(Drive.WL.speedSP, Drive.WL.speedAct, &PidAVR_L), 0), 3999);
			rightWheelPWM = Max(Min(pid_Controller(Drive.WR.speedSP, Drive.WR.speedAct, &PidAVR_R), 0), 3999);
			break;
		case 2:
			leftWheelPWM = PidSimpleProcessing(Drive.WL.speedSP, Drive.WL.speedAct, 0, 3999, &PidSimple_L);
			rightWheelPWM = PidSimpleProcessing(Drive.WR.speedSP, Drive.WR.speedAct, 0, 3999, &PidSimple_R);
			break;
		case 3:
			leftWheelPWM = PidFilteredProcessing(Drive.WL.speedSP, Drive.WL.speedAct, 100, 0, 3999, &PidFilter_L);
			rightWheelPWM = PidFilteredProcessing(Drive.WR.speedSP, Drive.WR.speedAct, 100, 0, 3999, &PidFilter_R);
			break;
		case 4:
			leftWheelPWM = PID_MotoProcessing(Drive.WL.speedSP, Drive.WL.speedAct, 0, 3999, 100, &PidMoto_L);
			rightWheelPWM = PID_MotoProcessing(Drive.WR.speedSP, Drive.WR.speedAct, 0, 3999, 100, &PidMoto_R);
			break;
		case 5:
			leftWheelPWM = PID_MotoFilteredProcessing(Drive.WL.speedSP, Drive.WL.speedAct, 100, 0, 3999,  &PidMotoFilter_L);
			rightWheelPWM = PID_MotoFilteredProcessing(Drive.WR.speedSP, Drive.WR.speedAct, 100, 0, 3999, &PidMotoFilter_R);
			break;
		default:
			rightWheelPWM = leftWheelPWM = 0;
			break;
		}

		TIM13->SR &= ~TIM_SR_UIF;
	}
	if (sr & TIM_SR_CC1IF) {
		//TIM12->CR1 |= TIM_CR1_CEN;
		TIM13->SR &= ~TIM_SR_CC1IF;
	}
	/* USER CODE END TIM8_UP_TIM13_IRQn 1 */
}

/**
 * @brief This function handles TIM8 trigger and commutation interrupts and TIM14 global interrupt.
 */
void TIM8_TRG_COM_TIM14_IRQHandler(void)
{
	/* USER CODE BEGIN TIM8_TRG_COM_TIM14_IRQn 0 */

	/* USER CODE END TIM8_TRG_COM_TIM14_IRQn 0 */

	/* USER CODE BEGIN TIM8_TRG_COM_TIM14_IRQn 1 */
	/*TODO ADD clear interrupt code*/
	/* USER CODE END TIM8_TRG_COM_TIM14_IRQn 1 */
}

/**
 * @brief This function handles SPI3 global interrupt.
 */
void SPI3_IRQHandler(void)
{
	/* USER CODE BEGIN SPI3_IRQn 0 */

	/* USER CODE END SPI3_IRQn 0 */
	/* USER CODE BEGIN SPI3_IRQn 1 */
	SPI_IRQ_TWO_CallBack(&SPI_Flash);
	/* USER CODE END SPI3_IRQn 1 */
}

/**
 * @brief This function handles UART5 global interrupt.
 */
void UART5_IRQHandler(void)
{
	/* USER CODE BEGIN UART5_IRQn 0 */

	/* USER CODE END UART5_IRQn 0 */
	/* USER CODE BEGIN UART5_IRQn 1 */
	USART_EV_IRQ_CallBack(&UART_GPS);
	/* USER CODE END UART5_IRQn 1 */
}

/**
 * @brief This function handles TIM6 global interrupt, DAC1 and DAC2 underrun error interrupts.
 */
void TIM6_DAC_IRQHandler(void)
{
	/* USER CODE BEGIN TIM6_DAC_IRQn 0 */

	/* USER CODE END TIM6_DAC_IRQn 0 */

	/* USER CODE BEGIN TIM6_DAC_IRQn 1 */
	volatile uint32_t sr;
	sr = TIM6->SR;
	if (sr & TIM_SR_UIF) {
		TIM6->SR &= ~TIM_SR_UIF;
	}
	/* USER CODE END TIM6_DAC_IRQn 1 */
}

/**
 * @brief This function handles TIM7 global interrupt.
 */
void TIM7_IRQHandler(void)
{
	/* USER CODE BEGIN TIM7_IRQn 0 */

	/* USER CODE END TIM7_IRQn 0 */
	/* USER CODE BEGIN TIM7_IRQn 1 */
	uint32_t sr = TIM7->SR;
	if (sr & TIM_SR_UIF) {
		uint32_t dtR = TIM3->CNT;
		TIM3->CNT = 0;
		uint32_t dtL = TIM4->CNT;
		TIM4->CNT = 0;
		uint32_t dt = (TIM7->ARR + 1) / 100;
		Drive.WL.speedAct = WheelSpeedMeasure(dtL, dt);
		Drive.WR.speedAct = WheelSpeedMeasure(dtR, dt);
		/*TODO set new ARR value here*/
		TIM12->CR1 |= TIM_CR1_CEN;
		TIM7->SR &= ~TIM_SR_UIF;
	}
	/* USER CODE END TIM7_IRQn 1 */
}

/**
 * @brief This function handles I2C3 event interrupt.
 */
void I2C3_EV_IRQHandler(void)
{
	/* USER CODE BEGIN I2C3_EV_IRQn 0 */

	/* USER CODE END I2C3_EV_IRQn 0 */

	/* USER CODE BEGIN I2C3_EV_IRQn 1 */
	I2C_Alt_IRQ_CallBack(&I2CLasers);
	//I2C_Raw_IRQ_CallBack(&I2CLasers);
	/* USER CODE END I2C3_EV_IRQn 1 */
}

/**
 * @brief This function handles I2C3 error interrupt.
 */
void I2C3_ER_IRQHandler(void)
{
	/* USER CODE BEGIN I2C3_ER_IRQn 0 */

	/* USER CODE END I2C3_ER_IRQn 0 */

	/* USER CODE BEGIN I2C3_ER_IRQn 1 */
	I2C_ERR_IRQ_CallBack(&I2CLasers);
	/* USER CODE END I2C3_ER_IRQn 1 */
}

/**
 * @brief This function handles HASH and RNG global interrupts.
 */
void HASH_RNG_IRQHandler(void)
{
	/* USER CODE BEGIN HASH_RNG_IRQn 0 */

	/* USER CODE END HASH_RNG_IRQn 0 */

	/* USER CODE BEGIN HASH_RNG_IRQn 1 */

	/* USER CODE END HASH_RNG_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
