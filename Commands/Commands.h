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

 * Commands.h
 * Created on: Nov 29, 2023
 ********************************************************************************/
#include <stdint.h>

 /*packet data format:
 byte[0]     synchro-byte
 byte[1]     device select
 byte[2]     device command
 byte[3]     packet length
 byte[4..n]  valid data, no bytes if HELLO command use
 byte[n-4]   CRC32
 */

 /*	SELECT DEVICE
		HELLO		0x00
		DRIVE		0x01
		CAMERA	0x02
		SENSORS	0x03
		GPS			0x04
		SYSTEM	0x0F
 */


#define COMMAND_START_BYTE	(uint8_t)0xAA
#define COMMAND_ACCEPT_BYTE	(uint8_t)0x55
#define COMMAND_NOOP				(uint8_t)0xFF

enum DeviceSelect {
	SYSTEM_SELECT = ( uint8_t )0x00,
	DRIVE_SELECT = ( uint8_t )0x10,
	CAMERA_SELECT = ( uint8_t )0x20,
	SENSORS_SELECT = ( uint8_t )0x30,
	GPS_SELECT = ( uint8_t )0x40,
	};

enum SystemCommand {
	SYSTEM_GET_STATUS = ( uint8_t )0x00,
	SYSTEM_GET_TICK = ( uint8_t )0x01,
	SENSOR_GET_LIGHT = ( uint8_t )0x02,
	};

enum DriveCommand {
	DRIVE_GET_STATUS = ( uint8_t )0x00,
	DRIVE_GET_WLEEHS = ( uint8_t )0x02,
	DRIVE_GET_ORIENTATION = ( uint8_t )0x03,
	DRIVE_GET_POWER = ( uint8_t )0x04,
	DRIVE_SET_SPEED = ( uint8_t )0x05,
	DRIVE_SET_DIRECTION = ( uint8_t )0x06,
	DRIVE_SET_TARGET = ( uint8_t )0x07,
	};

enum CameraCommand {
	CAMERA_GET_STATUS = ( uint8_t )0x00,
	CAMERA_SET_AUTO_ON = ( uint8_t )0x02,
	CAMERA_SET_AUTO_OFF = ( uint8_t )0x03,
	CAMERA_SET_TARGET = ( uint8_t )0x04
	};

enum SensorsCommand {
	SENSOR_GET_STATUS = ( uint8_t )0x00,
	SENSOR_GET_ALL_VALUE = ( uint8_t )0x01,
	SENSOR_GET_AMBIENT = ( uint8_t )0x02,
	SENSOR_GET_POWER = ( uint8_t )0x03,
	};

enum GpsCommand {
	GPS_GET_STATUS = ( uint8_t )0x00,
	GPS_GET_VALUE = ( uint8_t )0x01,
	};

