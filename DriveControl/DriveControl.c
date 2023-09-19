/*
 * DriveControl.c
 *
 *  Created on: Sep 11, 2023
 *      Author: alexm
 */


#include "DriveControl.h"

uint16_t SetPWM_IT(uint16_t actual, uint16_t SP, const uint16_t max, const uint16_t min, const uint16_t step) {
	if (SP > actual) {
		if (actual < (max - step)) {
			return actual + step;
		} else {
			return max;
		}
	}
	else if (SP < actual) {
		if (actual > (min + step)) {
			return actual - step;
		} else {
			return min;
		}
	} else {
		return actual;
	}
}
