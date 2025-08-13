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

counter_service_motor motor_service;
recive_motor rec_m1, rec_m2, rec_m3, rec_m4;

// --- Globale Variablen ---
int16_t Iq_meas_q15_rx = 0;
int16_t speed_rx = 0;
uint16_t battery_voltage_rx = 0;
uint8_t system_state_rx = 0;

// Empfangsbuffer (8 Byte Frame)
uint8_t rx_buffer_motor1[8];
uint8_t rx_buffer_motor2[8];
uint8_t rx_buffer_motor3[8];
uint8_t rx_buffer_motor4[8];


// debug:
uint8_t debug_receive[50];

static void Start_recive_motor_data(uint8_t motor);



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
	HAL_Delay(200);
	int16_t null = 0;
	memset(&motor_service, 0, sizeof(motor_service));
	memset(&rec_m1, 0, sizeof(rec_m1));
	memset(&rec_m2, 0, sizeof(rec_m2));
	memset(&rec_m3, 0, sizeof(rec_m3));
	memset(&rec_m4, 0, sizeof(rec_m4));

	send_speed_status_M1(null, MOTOR_INIT);
	send_speed_status_M2(null, MOTOR_INIT);
	send_speed_status_M3(null, MOTOR_INIT);
	send_speed_status_M4(null, MOTOR_INIT);

	Start_recive_motor_data(0);
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
recive_motor get_data_from_buffer(const uint8_t *buffer){
	recive_motor Output;
	Output.Iq 				= (int16_t)(buffer[1] | (buffer[2] << 8));
	Output.speed_rpm 		= (int16_t)(buffer[3] | (buffer[4] << 8));
	Output.battery_voltage	= (uint16_t)(buffer[5] | (buffer[6] << 8));
	Output.system_state		= buffer[7];
	Output.service 			= 0;

	return (Output);
}

void Recive_motor_status(uint8_t motor){

	switch (motor){
	case 1:
		if (rx_buffer_motor1[0] == 0xFF) {
			rec_m1 = get_data_from_buffer(rx_buffer_motor1);
		}
		// Neustarten des Empfangs
		HAL_UART_Receive_IT(&huart1, rx_buffer_motor1, 8);
		break;
	case 2:
		if (rx_buffer_motor2[0] == 0xFF) {
			rec_m2 = get_data_from_buffer(rx_buffer_motor2);
		}
		// Neustarten des Empfangs
		HAL_UART_Receive_IT(&huart2, rx_buffer_motor2, 8);
		break;

	case 3:
		if (rx_buffer_motor3[0] == 0xFF) {
			rec_m3 = get_data_from_buffer(rx_buffer_motor3);
		}
		// Neustarten des Empfangs
		HAL_UART_Receive_IT(&huart3, rx_buffer_motor3, 8);
		break;

	case 4:
		if (rx_buffer_motor4[0] == 0xFF) {
			rec_m4 = get_data_from_buffer(rx_buffer_motor4);
		}
		// Neustarten des Empfangs
		HAL_UART_Receive_IT(&huart6, rx_buffer_motor4, 8);
		break;
	default:
		// do nothing
		break;
		}

	}



void service_recive_motor_information(void){

	rec_m1.service++;
	rec_m2.service++;
	rec_m3.service++;
	rec_m4.service++;

	if (rec_m1.service > 100){
		Start_recive_motor_data(1);
		rec_m1.service = 0;
	}
	if (rec_m2.service > 100){
		Start_recive_motor_data(2);
		rec_m2.service = 0;
	}
	if (rec_m3.service > 100){
		Start_recive_motor_data(3);
		rec_m3.service = 0;
	}
	if (rec_m4.service > 100){
		Start_recive_motor_data(4);
		rec_m4.service = 0;
	}

}




// Startet den ersten DMA/UART-Empfang
static void Start_recive_motor_data(uint8_t motor) {

	switch(motor){
	case 1:
		if (HAL_UART_GetState(&huart1) == HAL_UART_STATE_READY) {
			if (HAL_UART_Receive_IT(&huart1, rx_buffer_motor1, 8) != HAL_OK) {
				// Fehlerbehandlung falls Empfang nicht starten konnte
	//            Error_Handler();
			}
		}
		break;
	case 2:
		if (HAL_UART_GetState(&huart2) == HAL_UART_STATE_READY) {
			if (HAL_UART_Receive_IT(&huart2, rx_buffer_motor2, 8) != HAL_OK) {
				// Fehlerbehandlung falls Empfang nicht starten konnte
	//            Error_Handler();
			}
		}
		break;
	case 3:
		if (HAL_UART_GetState(&huart3) == HAL_UART_STATE_READY) {
			if (HAL_UART_Receive_IT(&huart3, rx_buffer_motor3, 8) != HAL_OK) {
				// Fehlerbehandlung falls Empfang nicht starten konnte
	//            Error_Handler();
			}
		}
		break;
	case 4:
		if (HAL_UART_GetState(&huart6) == HAL_UART_STATE_READY) {
			if (HAL_UART_Receive_IT(&huart6, rx_buffer_motor4, 8) != HAL_OK) {
				// Fehlerbehandlung falls Empfang nicht starten konnte
	//            Error_Handler();
			}
		}
		break;
	default: // start all. used in init process
		if (HAL_UART_GetState(&huart1) == HAL_UART_STATE_READY) {
			if (HAL_UART_Receive_IT(&huart1, rx_buffer_motor1, 8) != HAL_OK) {
				// Fehlerbehandlung falls Empfang nicht starten konnte
	//            Error_Handler();
			}
		}
		if (HAL_UART_GetState(&huart2) == HAL_UART_STATE_READY) {
			if (HAL_UART_Receive_IT(&huart2, rx_buffer_motor2, 8) != HAL_OK) {
				// Fehlerbehandlung falls Empfang nicht starten konnte
	//            Error_Handler();
			}
		}
		if (HAL_UART_GetState(&huart3) == HAL_UART_STATE_READY) {
			if (HAL_UART_Receive_IT(&huart3, rx_buffer_motor3, 8) != HAL_OK) {
				// Fehlerbehandlung falls Empfang nicht starten konnte
	//            Error_Handler();
			}
		}
		if (HAL_UART_GetState(&huart6) == HAL_UART_STATE_READY) {
			if (HAL_UART_Receive_IT(&huart6, rx_buffer_motor4, 8) != HAL_OK) {
				// Fehlerbehandlung falls Empfang nicht starten konnte
	//            Error_Handler();
			}
		}
	break;
}
}


// DEBUGGING FUNCTIONS
void send_test_value_motor1(void){

}

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

