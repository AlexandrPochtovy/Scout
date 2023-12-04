/*********************************************************************************
 Original author: user

 Licensed under the Apache License, Version 2.0 (the "License");
 you may not use this file except in compliance with the License.
 You may obtain a copy of the License at

 http://www.apache.org/licenses/LICENSE-2.0

 Unless required by applicable law or agreed to in writing, software
 distributed under the License is distributed on an "AS IS" BASIS,
 WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 See the License for the specific language governing permissions and
 limitations under the License.

 * Objects.h
 * Created on: Sep 5, 2023
 ********************************************************************************/

#ifndef INC_OBJECTS_H_
#define INC_OBJECTS_H_

#ifdef __cplusplus
extern "C" {
#endif

	/*	i2c1 bus for sensors	***********************************/
	uint8_t i2c1Buff[32] = { 0 };
	fifo_t i2c1Data = { .buffer = i2c1Buff, .buffer_size = sizeof(i2c1Buff), .bytes_avail = 0, .head = 0, .tail = 0 };
	I2C_IRQ_Conn_t I2CSensors = { .i2c = I2C1,
	  .status = PORT_FREE,
	  .addr = 0x00,
	  .len = 0,
	  .mode = I2C_MODE_WRITE,
	  .buffer = &i2c1Data };

	/*	i2c3 bus for VL53L01 sensors	****************************/
	uint8_t i2c3Buff[32] = { 0 };
	fifo_t i2c3Data = { .buffer = i2c3Buff, .buffer_size = sizeof(i2c3Buff), .bytes_avail = 0, .head = 0, .tail = 0 };
	I2C_IRQ_Conn_t I2CLasers = { .i2c = I2C3,
	  .status = PORT_FREE,
	  .addr = 0x00,
	  .len = 0,
	  .mode = I2C_MODE_WRITE,
	  .buffer = &i2c3Data };
//SPI1 for LoRa radio
	uint8_t spi1b1[255] = { 0 };
	uint8_t spi1b2[255] = { 0 };
	fifo_t spi1_TX = { .buffer = spi1b1, .buffer_size = sizeof(spi1b1), .bytes_avail = 0, .head = 0, .tail = 0 };
	fifo_t spi1_RX = { .buffer = spi1b2, .buffer_size = sizeof(spi1b2), .bytes_avail = 0, .head = 0, .tail = 0 };
	SPI_Conn_TWO_t SPI_LoRa = { .SPIbus = SPI1, .status = PORT_FREE, .mode = SPI_FULLDUPLEX_RW, .txlen = 0, .txbuffer =
	  &spi1_TX, .txlen = 0, .rxbuffer = &spi1_RX, .rxlen = 0 };
//SPI3 for flash
	uint8_t spi3b1[1024] = { 0 };
	uint8_t spi3b2[1024] = { 0 };
	fifo_t spi3_TX = { .buffer = spi3b1, .buffer_size = sizeof(spi3b1), .bytes_avail = 0, .head = 0, .tail = 0 };
	fifo_t spi3_RX = { .buffer = spi3b2, .buffer_size = sizeof(spi3b2), .bytes_avail = 0, .head = 0, .tail = 0 };
	SPI_Conn_TWO_t SPI_Flash = { .SPIbus = SPI3, .status = PORT_FREE, .mode = SPI_FULLDUPLEX_RW, .txlen = 0, .txbuffer =
	  &spi3_TX, .txlen = 0, .rxbuffer = &spi3_RX, .rxlen = 0 };
//usart1 bus for Orange Pi Zero2 connection
	uint8_t usart1b1[255] = { 0 };
	uint8_t usart1b2[255] = { 0 };
	fifo_t usart1_TX = { .buffer = usart1b1, .buffer_size = sizeof(usart1b1), .bytes_avail = 0, .head = 0, .tail = 0 };
	fifo_t usart1_RX = { .buffer = usart1b2, .buffer_size = sizeof(usart1b2), .bytes_avail = 0, .head = 0, .tail = 0 };
	USART_FullDuplex_t USART_Orange = { .port = USART1,
	  .transmitStatus = PORT_FREE,
	  .receiveStatus = PORT_FREE,
	  .transmitData = &usart1_TX,
	  .transmitLen = 0,
	  .receiveData = &usart1_RX,
	  .receiveLen = 0 };
//usart2 bus for external connection CLI
	uint8_t usart2b1[255] = { 0 };
	uint8_t usart2b2[255] = { 0 };
	fifo_t usart2_TX = { .buffer = usart2b1, .buffer_size = sizeof(usart2b1), .bytes_avail = 0, .head = 0, .tail = 0 };
	fifo_t usart2_RX = { .buffer = usart2b2, .buffer_size = sizeof(usart2b2), .bytes_avail = 0, .head = 0, .tail = 0 };
	USART_FullDuplex_t USART_CLI = { .port = USART2,
	  .transmitStatus = PORT_FREE,
	  .receiveStatus = PORT_FREE,
	  .transmitData = &usart2_TX,
	  .transmitLen = 0,
	  .receiveData = &usart2_RX,
	  .receiveLen = 0 };
//usart3 bus for external connection Arduino
	uint8_t usart3b1[255] = { 0 };
	uint8_t usart3b2[255] = { 0 };
	fifo_t usart3_TX = { .buffer = usart3b1, .buffer_size = sizeof(usart3b1), .bytes_avail = 0, .head = 0, .tail = 0 };
	fifo_t usart3_RX = { .buffer = usart3b2, .buffer_size = sizeof(usart3b2), .bytes_avail = 0, .head = 0, .tail = 0 };
	USART_FullDuplex_t USART_UNO = { .port = USART3,
	  .transmitStatus = PORT_FREE,
	  .receiveStatus = PORT_FREE,
	  .transmitData = &usart3_TX,
	  .transmitLen = 0,
	  .receiveData = &usart3_RX,
	  .receiveLen = 0 };
//uart5 bus for GPS connection
	uint8_t uart5b1[255] = { 0 };
	uint8_t uart5b2[255] = { 0 };
	fifo_t uart5_TX = { .buffer = uart5b1, .buffer_size = sizeof(uart5b1), .bytes_avail = 0, .head = 0, .tail = 0 };
	fifo_t uart5_RX = { .buffer = uart5b2, .buffer_size = sizeof(uart5b2), .bytes_avail = 0, .head = 0, .tail = 0 };
	USART_FullDuplex_t UART_GPS = { .port = UART5,
	  .transmitStatus = PORT_FREE,
	  .receiveStatus = PORT_FREE,
	  .transmitData = &uart5_TX,
	  .transmitLen = 0,
	  .receiveData = &uart5_RX,
	  .receiveLen = 0 };

//	Objects and Sensors
	TCA9548A_t tca9548 = { .addr = TCA9548A_ADDR, .status = DEVICE_READY, .errLimit = 10, .errCount = 0, .port = 0x00 }; //i2c multiplexer
	VL53L0x_t LaserFrontLeft = { .addr = VL53L0x_ADDR_DEFAULT,
	  .status = DEVICE_READY,
	  .stepL1 = 0,
	  .stepL2 = 0,
	  .stepL3 = 0,
	  .modelID = 0,
	  .revisionID = 0,
	  .count_timeout = 0,
	  .limit_timeout = 250,
	  .timeoutFlag = 0,
	  .stop_variable = 0,
	  .sp_budget_us = 0,
	  .measurement_timing_budget_us = 0,
	  .spad_count = 0,
	  .spad_type_is_aperture = 0,
	  .enables = { 0 },
	  .timeouts = { 0 },
	  .sequence_config = 0,
	  .vcsel_period_reg = 0,
	  .new_pre_range_timeout_mclks = 0,
	  .new_msrc_timeout_mclks = 0,
	  .new_final_range_timeout_mclks = 0,
	  .vcselPeriodValue = 0,
	  .ref_spad_map = { 0 },
	  .tmp8 = 0,
	  .tmp16 = 0,
	  .range = 0,
	  .smoothRange = 0,
	  .errLimit = 10 };
	VL53L0x_t LaserFrontRight = { .addr = VL53L0x_ADDR_DEFAULT,
	  .status = DEVICE_READY,
	  .stepL1 = 0,
	  .stepL2 = 0,
	  .stepL3 = 0,
	  .modelID = 0,
	  .revisionID = 0,
	  .count_timeout = 0,
	  .limit_timeout = 250,
	  .timeoutFlag = 0,
	  .stop_variable = 0,
	  .sp_budget_us = 0,
	  .measurement_timing_budget_us = 0,
	  .spad_count = 0,
	  .spad_type_is_aperture = 0,
	  .enables = { 0 },
	  .timeouts = { 0 },
	  .sequence_config = 0,
	  .vcsel_period_reg = 0,
	  .new_pre_range_timeout_mclks = 0,
	  .new_msrc_timeout_mclks = 0,
	  .new_final_range_timeout_mclks = 0,
	  .vcselPeriodValue = 0,
	  .ref_spad_map = { 0 },
	  .tmp8 = 0,
	  .tmp16 = 0,
	  .range = 0,
	  .smoothRange = 0,
	  .errLimit = 10 };
	VL53L0x_t LaserMidLeft = { .addr = VL53L0x_ADDR_DEFAULT,
	  .status = DEVICE_READY,
	  .stepL1 = 0,
	  .stepL2 = 0,
	  .stepL3 = 0,
	  .modelID = 0,
	  .revisionID = 0,
	  .count_timeout = 0,
	  .limit_timeout = 250,
	  .timeoutFlag = 0,
	  .stop_variable = 0,
	  .sp_budget_us = 0,
	  .measurement_timing_budget_us = 0,
	  .spad_count = 0,
	  .spad_type_is_aperture = 0,
	  .enables = { 0 },
	  .timeouts = { 0 },
	  .sequence_config = 0,
	  .vcsel_period_reg = 0,
	  .new_pre_range_timeout_mclks = 0,
	  .new_msrc_timeout_mclks = 0,
	  .new_final_range_timeout_mclks = 0,
	  .vcselPeriodValue = 0,
	  .ref_spad_map = { 0 },
	  .tmp8 = 0,
	  .tmp16 = 0,
	  .range = 0,
	  .smoothRange = 0,
	  .errLimit = 10 };
	VL53L0x_t LaserMidRight = { .addr = VL53L0x_ADDR_DEFAULT,
	  .status = DEVICE_READY,
	  .stepL1 = 0,
	  .stepL2 = 0,
	  .stepL3 = 0,
	  .modelID = 0,
	  .revisionID = 0,
	  .count_timeout = 0,
	  .limit_timeout = 250,
	  .timeoutFlag = 0,
	  .stop_variable = 0,
	  .sp_budget_us = 0,
	  .measurement_timing_budget_us = 0,
	  .spad_count = 0,
	  .spad_type_is_aperture = 0,
	  .enables = { 0 },
	  .timeouts = { 0 },
	  .sequence_config = 0,
	  .vcsel_period_reg = 0,
	  .new_pre_range_timeout_mclks = 0,
	  .new_msrc_timeout_mclks = 0,
	  .new_final_range_timeout_mclks = 0,
	  .vcselPeriodValue = 0,
	  .ref_spad_map = { 0 },
	  .tmp8 = 0,
	  .tmp16 = 0,
	  .range = 0,
	  .smoothRange = 0,
	  .errLimit = 10 };
	VL53L0x_t LaserBackLeft = { .addr = VL53L0x_ADDR_DEFAULT,
	  .status = DEVICE_READY,
	  .stepL1 = 0,
	  .stepL2 = 0,
	  .stepL3 = 0,
	  .modelID = 0,
	  .revisionID = 0,
	  .count_timeout = 0,
	  .limit_timeout = 250,
	  .timeoutFlag = 0,
	  .stop_variable = 0,
	  .sp_budget_us = 0,
	  .measurement_timing_budget_us = 0,
	  .spad_count = 0,
	  .spad_type_is_aperture = 0,
	  .enables = { 0 },
	  .timeouts = { 0 },
	  .sequence_config = 0,
	  .vcsel_period_reg = 0,
	  .new_pre_range_timeout_mclks = 0,
	  .new_msrc_timeout_mclks = 0,
	  .new_final_range_timeout_mclks = 0,
	  .vcselPeriodValue = 0,
	  .ref_spad_map = { 0 },
	  .tmp8 = 0,
	  .tmp16 = 0,
	  .range = 0,
	  .smoothRange = 0,
	  .errLimit = 10 };
	VL53L0x_t LaserBackRight = { .addr = VL53L0x_ADDR_DEFAULT,
	  .status = DEVICE_READY,
	  .stepL1 = 0,
	  .stepL2 = 0,
	  .stepL3 = 0,
	  .modelID = 0,
	  .revisionID = 0,
	  .count_timeout = 0,
	  .limit_timeout = 250,
	  .timeoutFlag = 0,
	  .stop_variable = 0,
	  .sp_budget_us = 0,
	  .measurement_timing_budget_us = 0,
	  .spad_count = 0,
	  .spad_type_is_aperture = 0,
	  .enables = { 0 },
	  .timeouts = { 0 },
	  .sequence_config = 0,
	  .vcsel_period_reg = 0,
	  .new_pre_range_timeout_mclks = 0,
	  .new_msrc_timeout_mclks = 0,
	  .new_final_range_timeout_mclks = 0,
	  .vcselPeriodValue = 0,
	  .ref_spad_map = { 0 },
	  .tmp8 = 0,
	  .tmp16 = 0,
	  .range = 0,
	  .smoothRange = 0,
	  .errLimit = 10 };

	ADXL345_t adxl345 = { .addr = ADXL345_ADDR, .status = DEVICE_READY, .errLimit = 10, .step = 0, .data = { 0 }, .raw =
	  { 0 } };		//accelerometer
	ITG3205_t itg3205 = { .addr = ITG3205_ADDR, .status = DEVICE_READY, .errLimit = 10, .step = 0, .data = { 0 }, .raw =
	  { 0 } };		//gyroscope
	QMC5883L_t qmc5883 = { .addr = QMC5883L_ADDR, .status = DEVICE_READY, .errLimit = 10, .step = 0, .data = { 0 }, .raw =
	  { 0 } }; 	//magnetometer
	BME280_t bme280 = { .addr = BME280_ADDR1,
	  .status = DEVICE_READY,
	  .errLimit = 10,
	  .step = 0,
	  .calib_data = { 0 },
	  .uncomp_data = { 0 },
	  .data_int = { 0 },
	  .data_float = { 0 } };				//ambient sensor
	INA219_t ina219 = { .addr = INA219_ADDR, .status = DEVICE_READY, .errLimit = 10, .step = 0, .raw = { 0 } };	//current voltage power sensor

	HC_SR04_t USMrange = { .distance_mm = 0, .start = 0, .stop = 0 };

	Drive_t Drive = { .mode = WAITING,
	  .step = 0,
	  .position.x = 0,
	  .position.y = 0,
	  .speed = 0,
	  .distance = 0,
	  .fullPath = 0,
	  .intLength = 0,
	  .bearing = 0,
	  .course = 0,
	  .angle = 0,
	  .iAngle = 0,
	  .gyroSpeed = 0,
	  .SP.target.x = 0,
	  .SP.target.y = 0,
	  .SP.pwmLeft = 0,
	  .SP.pwmRight = 0 };
	pidData_t PidAVR_L = {0};
	pidData_t PidAVR_R = {0};
	pidS_t PidSimple_L = {0};
	pidS_t PidSimple_R = {0};
	pidF_t PidFilter_L = {0};
	pidF_t PidFilter_R = {0};
	PID_M_t PidMoto_L = {0};
	PID_M_t PidMoto_R = {0};
	PID_MF_t PidMotoFilter_L = {0};
	PID_MF_t PidMotoFilter_R = {0};

#ifdef __cplusplus
}
#endif

#endif /* INC_OBJECTS_H_ */
