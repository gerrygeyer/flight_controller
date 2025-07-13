/*
 * log_data.h
 *
 *  Created on: Jul 11, 2025
 *      Author: gerrygeyer
 */

#ifndef INC_LOG_DATA_H_
#define INC_LOG_DATA_H_


#include "fatfs.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <main.h>
#include <parameter.h>

#define LOG_BUFFER_SIZE 512
#define LOG_RING_SIZE 4096
#define LOG_WRITE_THRESHOLD (LOG_RING_SIZE * 3 / 4)  // ab 75 % wird geschrieben
#define LOG_SYNC_INTERVAL_MS 200                     // alle 200 ms f_sync()

void Log_WriteBuffered(const char* data);
void Log_ProcessBuffered(void);

bool Log_Init(void);
bool Log_Write(const char* data);
void Log_Process(void);  // regelmäßig im main loop aufrufen

void Log_GyroCSV(const xyz_16t *gyro);

void set_log_data_flag(void);
void log_data_if_ready(void);
#endif /* INC_LOG_DATA_H_ */
