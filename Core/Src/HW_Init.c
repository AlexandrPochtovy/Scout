/*
 * HW_Init.c
 *
 *  Created on: Aug 29, 2023
 *      Author: alexm
 */
#include "HW_Init.h"

void HardwareInit(void) {

	TIM10->CCER |= TIM_CCER_CC1E;
	TIM10->DIER |= TIM_DIER_UIE;
	TIM10->CR1 |= TIM_CR1_CEN;

	TIM11->CCER |= TIM_CCER_CC1E;
	TIM11->DIER |= TIM_DIER_UIE;
	TIM11->CR1 |= TIM_CR1_CEN;

}
