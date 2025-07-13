/*
 * communicaton.c
 *
 *  Created on: Nov 2, 2024
 *      Author: Gerry Geyer
 */

#include <communication.h>
#include <parameter.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <main.h>
#include "bno085.h"       // für BNO085_UART_DMA_RXCallback
#include <settings.h>

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart6;
extern UART_HandleTypeDef huart5;


uint8_t TxData_M1[4], TxData_M2[4], TxData_M3[4], TxData_M4[4];

// debug:
uint8_t debug_receive[50];

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == UART5) {
        // Datenverarbeitung
        // z. B. erneut Empfang starten
        HAL_UART_Receive_IT(&huart5, debug_receive, sizeof(debug_receive));

//        HAL_Delay(100);
    }
//    if (huart == &BNO085_UART_HANDLE) { // Triggering IMU processing
//        BNO085_UART_DMA_RXCallback();
//    }
}



void init_motor(void){
	HAL_Delay(2000);
	int16_t null = 0;
	send_speed_status_M1(null, MOTOR_INIT);
	send_speed_status_M2(null, MOTOR_INIT);
	send_speed_status_M3(null, MOTOR_INIT);
	send_speed_status_M4(null, MOTOR_INIT);
	HAL_Delay(2000);
}


uint8_t run_motors(motor_t * pHandle){
	uint8_t counter_fails = 0;
	counter_fails += send_speed_status_M1(pHandle->m_1, pHandle->state_m_1);
	counter_fails += send_speed_status_M2(pHandle->m_2, pHandle->state_m_2);
	counter_fails += send_speed_status_M3(pHandle->m_3, pHandle->state_m_3);
	counter_fails += send_speed_status_M4(pHandle->m_4, pHandle->state_m_4);

	switch (counter_fails){
	case 0:
		return OK;
	default:
		return NOT_OK;
	}

}


uint8_t send_speed_status_M1(int16_t speed, uint8_t status) {

	if(COMMUNICATION_MOTOR){

		TxData_M1[0] = 0xFF;
		TxData_M1[1] = status;
		TxData_M1[2] = (uint8_t)(speed >> 8);
		TxData_M1[3] = (uint8_t)(speed & 0xFF);

		if (HAL_UART_Transmit_DMA(&huart1, TxData_M1, sizeof(TxData_M1)) != HAL_OK) {
			return NOT_OK;
		}
	}
    return OK;
}

uint8_t send_speed_status_M2(int16_t speed, uint8_t status) {

	if(COMMUNICATION_MOTOR){
		TxData_M2[0] = 0xFF;
		TxData_M2[1] = status;
		TxData_M2[2] = (uint8_t)(speed >> 8);
		TxData_M2[3] = (uint8_t)(speed & 0xFF);

		if (HAL_UART_Transmit_DMA(&huart2, TxData_M2, sizeof(TxData_M2)) != HAL_OK) {
			return NOT_OK;
		}
	}
    return OK;
}

uint8_t send_speed_status_M3(int16_t speed, uint8_t status) {

	if(COMMUNICATION_MOTOR){

		TxData_M3[0] = 0xFF;
		TxData_M3[1] = status;
		TxData_M3[2] = (uint8_t)(speed >> 8);
		TxData_M3[3] = (uint8_t)(speed & 0xFF);

		if (HAL_UART_Transmit_DMA(&huart3, TxData_M3, sizeof(TxData_M3)) != HAL_OK) {
			return NOT_OK;
		}
	}
    return OK;

}

uint8_t send_speed_status_M4(int16_t speed, uint8_t status) {

	if(COMMUNICATION_MOTOR){
		TxData_M4[0] = 0xFF;
		TxData_M4[1] = status;
		TxData_M4[2] = (uint8_t)(speed >> 8);
		TxData_M4[3] = (uint8_t)(speed & 0xFF);

		if (HAL_UART_Transmit_DMA(&huart6, TxData_M4, sizeof(TxData_M4)) != HAL_OK) {
			return NOT_OK;
		}
	}
    return OK;
}







// DEBUGGING FUNCTIONS


// scan_i2c_addresses(&hi2c): scan for i2c adress; return (uint8_t) adress.
uint8_t scan_i2c_addresses(I2C_HandleTypeDef *hi2c) {
    uint8_t dummy = 0;
    HAL_StatusTypeDef result;

    for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
        result = HAL_I2C_Master_Transmit(hi2c, addr << 1, &dummy, 1, 5);
        if (result == HAL_OK) {
            return (addr << 1);
        }
    }
    return 0;
}

