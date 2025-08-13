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

void Recive_motor_status1(void);
void service_recive_motor_information(void);

// debug fnctions
uint8_t scan_i2c_addresses(I2C_HandleTypeDef *hi2c);


/**
 * @brief Service counter for all 4 Motors
 * @note  used only in commincaton.c for counts the missing motor data
 * @see   ReferenzOderModulname
 */
typedef struct
{
	uint8_t m1; /**< counter for M1  */
	uint8_t m2; /**< counter for M2 */
	uint8_t m3; /**< counter for M3 */
	uint8_t m4; /**< counter for M4 */
} counter_service_motor;

#endif /* INC_COMMUNICATION_H_ */
