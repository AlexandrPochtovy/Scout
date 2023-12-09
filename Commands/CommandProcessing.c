/*********************************************************************************
	Original author:  Aliaksandr Pachtovy<alex.mail.prime@gmail.com>
										https://github.com/AlexandrPochtovy

	Licensed under the Apache License, Version 2.0 (the "License");
	you may not use this file except in compliance with the License.
	You may obtain a copy of the License at

			http://www.apache.org/licenses/LICENSE-2.0

	Unless required by applicable law or agreed to in writing, software
	distributed under the License is distributed on an "AS IS" BASIS,
	WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
	See the License for the specific language governing permissions and
	limitations under the License.

 * CommandProcessing.c
 * Created on: Nov 30, 2023
 ********************************************************************************/

#include "CommandProcessing.h"

static ConnectionMode_t step = BEGIN;
static uint8_t buffer[255];

uint8_t CheckCRC32(uint8_t* data, uint8_t len) {
	uint32_t crc32Calc = F4xx_HW_CRC32(CRC, ( uint32_t* )&data, len - 4);
	uint32_t *pcrc;
	pcrc = ( uint32_t* )&buffer[len - 4];
	uint32_t crcShift = *pcrc;
	return crc32Calc == crcShift;
	}

uint8_t AddCRC32(uint8_t* data, uint8_t len) {
	uint32_t crc32Calc = F4xx_HW_CRC32(CRC, ( uint32_t* )&data, len - 4);
	data[len - 4] = crc32Calc;
	return 1;
	}

void RequestProcessing(USART_FullDuplex_t *usart) {
	uint8_t len;
	if (step == BEGIN) { //start communications
		USART_ProcessingEnable(usart);
		step = LISTENING;
		}
	if (step == LISTENING) { //usart port listening
		len = USART_Receive(usart, buffer);
		if (len) {
			step = REQUEST_PROCESSING;
			}
		}
	if (step == REQUEST_PROCESSING) {
		if ((buffer[0] == COMMAND_START_BYTE) && (CheckCRC32(buffer, len))) {
			switch (buffer[1]) {//select device
				case SYSTEM_SELECT://request for system, MCU, etc.
					switch (buffer[2]) {
						case SYSTEM_GET_STATUS:

							break;
						case SYSTEM_GET_TICK:

							break;
						default:
							break;
						}
					break;
				case DRIVE_SELECT://retuest for drive
					switch (buffer[2]) {
						case DRIVE_GET_STATUS:

							break;
						case DRIVE_GET_WLEEHS:

							break;
						case DRIVE_GET_ORIENTATION:
							break;
						case DRIVE_GET_POWER:

							break;
						case DRIVE_SET_SPEED:

							break;
						case DRIVE_SET_DIRECTION:

							break;
						case DRIVE_SET_TARGET:
							break;
						default:
							break;
						}
					break;
				case CAMERA_SELECT://request for camera (servo)
					switch (buffer[2]) {
						case CAMERA_GET_STATUS:
							/* code */
							break;
						case CAMERA_SET_AUTO_ON:
							/* code */
							break;
						case CAMERA_SET_AUTO_OFF:
							/* code */
							break;
						case CAMERA_SET_TARGET:
							/* code */
							break;
						default:
							break;
						}
					break;
				case SENSORS_SELECT://request for sensors
					switch (buffer[2]) {
						case SENSOR_GET_STATUS:
							/* code */
							break;
						case SENSOR_GET_ALL_VALUE:
							/* code */
							break;
						case SENSOR_GET_AMBIENT:
							/* code */
							break;
						case SENSOR_GET_POWER:
							/* code */
							break;
						default:
							break;
						}
					break;
				case GPS_SELECT://request for gps module
					switch (buffer[2]) {
						case GPS_GET_STATUS:
							/* code */
							break;
						case GPS_GET_VALUE:
							/* code */
							break;
						default:
							break;
						}
					break;
				default://unknow request but crc32 valud, send NOOP
					buffer[1] = COMMAND_NOOP;
					buffer[2] = COMMAND_NOOP;
					len = 4;
					break;
				}
			}
		else {//send NOOP if wrong start byte or bad crc32
			buffer[1] = COMMAND_NOOP;
			buffer[2] = COMMAND_NOOP;
			len = 4;
			}
		buffer[0] = COMMAND_ACCEPT_BYTE;
		buffer[3] = len;
		AddCRC32(buffer, len);
		step = SENDING;
		}
	if (step == SENDING) {
		if (USART_Transmit(usart, buffer, len)) {
			step = LISTENING;
			}
		}
	}
