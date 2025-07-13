/*
 * gps.c
 *
 *  Created on: May 20, 2025
 *      Author: gerrygeyer
 */


#include "gps.h"
#include <string.h>

static UART_HandleTypeDef *gps_huart;
static uint8_t dma_buffer[GPS_DMA_BUFFER_SIZE];
static uint8_t line_buffer[GPS_DMA_BUFFER_SIZE];
static uint16_t old_pos = 0;

void GPS_DMA_Init(UART_HandleTypeDef *huart) {
    gps_huart = huart;
    HAL_UART_Receive_DMA(gps_huart, dma_buffer, GPS_DMA_BUFFER_SIZE);
}

uint8_t GPS_DMA_ReadLine(char *out_line) {
    uint16_t new_pos = GPS_DMA_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(gps_huart->hdmarx);
    while (old_pos != new_pos) {
        char c = dma_buffer[old_pos++];
        if (old_pos >= GPS_DMA_BUFFER_SIZE) old_pos = 0;

        static uint16_t idx = 0;
        if (c == '\n') {
            line_buffer[idx] = '\0';
            idx = 0;
            strcpy(out_line, (char*)line_buffer);
            return 1;
        } else if (idx < GPS_DMA_BUFFER_SIZE - 1) {
            line_buffer[idx++] = c;
        } else {
            idx = 0; // Overflow vermeiden
        }
    }
    return 0;
}
