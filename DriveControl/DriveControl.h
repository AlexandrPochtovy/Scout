/*
 * DriveControl.h
 *
 *  Created on: Sep 11, 2023
 *      Author: alexm
 */

#ifndef DRIVECONTROL_H_
#define DRIVECONTROL_H_

#include "stdint.h"

uint16_t SetPWM_IT(uint16_t actual, uint16_t SP, const uint16_t max, const uint16_t min, const uint16_t step);



#endif /* DRIVECONTROL_H_ */
