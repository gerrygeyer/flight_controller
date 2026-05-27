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

//#define LOG_BUFFER_SIZE 512
#define LOG_RING_SIZE 16384
#define LOG_WRITE_THRESHOLD (LOG_RING_SIZE * 3 / 4)  // ab 75 % wird geschrieben
#define LOG_SYNC_INTERVAL_MS 200                     // alle 200 ms f_sync()

/**
 * @brief     Writes a string to the log ring buffer.
 * @details   Adds characters from the input string into a circular buffer for later writing to the SD card.
 *            Prevents blocking during runtime by deferring file I/O.
 * @param     data Null-terminated string to log.
 */
//void Log_WriteBuffered(const char* data);
bool Log_WriteBuffered(const char* s);

/**
 * @brief     Writes buffered log data to the SD card.
 * @details   Flushes data from the ring buffer to the open log file using `f_write()` and `f_sync()`.
 *            Skips writes if the buffer isn't full enough or too little time has passed.
 */
void Log_ProcessBuffered(void);

/**
 * @brief     Initializes the logging system.
 * @details   Mounts the filesystem, finds a new unique log file name, and opens the file for writing.
 *            Writes the CSV header on success.
 * @retval    true if initialization succeeded, false otherwise.
 */
bool Log_Init(void);


bool Log_Write(const char* data);

/**
 * @brief     Logs a single gyroscope (or magnetometer) data sample to CSV.
 * @details   Appends a formatted CSV line with index and 3-axis data to the ring buffer.
 * @param     gyro Pointer to struct containing x, y, z values.
 */
void Log_GyroCSV(const xyz_16t *gyro);
//void Log_IMUCSV(const xyz_16t *gyro, const xyz_16t *acc, const xyz_16t *mag, bool mag_valid);
void Log_IMUCSV(const xyz_16t *gyro, const xyz_16t *acc, const xyz_16t *mag, bool mag_valid, wxyz_16t *q);
/**
 * @brief     Sets the logging trigger flag.
 * @details   Marks that a new data sample is ready to be logged.
 */
void set_log_data_flag(void);

/**
 * @brief     Logs data if the logging flag is set.
 * @details   Checks if logging is due. If so, resets the flag and writes the current sample to the log.
 */
void log_data_if_ready(void);


#endif /* INC_LOG_DATA_H_ */
