/*
 * gps.h
 *
 *  Created on: May 20, 2025
 *      Author: gerrygeyer
 */

#ifndef GPS_GPS_H_
#define GPS_GPS_H_

#ifndef GPS_H
#define GPS_H

#include "main.h"
#include <stdint.h>

#define GPS_DMA_BUFFER_SIZE 128



void GPS_DMA_Init(UART_HandleTypeDef *huart);
uint8_t GPS_DMA_ReadLine(char *out_line);

typedef struct {
    char nmea_sentence[GPS_DMA_BUFFER_SIZE];
    uint8_t index;
    uint8_t ready;
} GPS_t;

void GPS_Init(UART_HandleTypeDef *huart);
void GPS_ProcessByte(uint8_t byte);
uint8_t GPS_ReadLine(char *out_line);

#endif


#endif /* GPS_GPS_H_ */
