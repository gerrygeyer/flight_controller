/*
 * communication.h
 *
 *  Created on: Nov 2, 2024
 *      Author: Gerry Geyer
 */

#ifndef INC_COMMUNICATION_H_
#define INC_COMMUNICATION_H_

#include <stdint.h>
#include <parameter.h>
#include <communication.h>
#include <main.h>

void init_motor(void);
uint8_t run_motors(motor_t * pHandle);
uint8_t send_speed_status_M1(int16_t speed, uint8_t status);
uint8_t send_speed_status_M2(int16_t speed, uint8_t status);
uint8_t send_speed_status_M3(int16_t speed, uint8_t status);
uint8_t send_speed_status_M4(int16_t speed, uint8_t status);


// debug fnctions
uint8_t scan_i2c_addresses(I2C_HandleTypeDef *hi2c);

#endif /* INC_COMMUNICATION_H_ */
