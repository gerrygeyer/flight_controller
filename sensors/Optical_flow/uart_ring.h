/*
 * uart_ring.h
 *
 *  Created on: Aug 30, 2025
 *      Author: gerrygeyer
 */

#ifndef OPTICAL_FLOW_UART_RING_H_
#define OPTICAL_FLOW_UART_RING_H_

#include "main.h"
#include <stdint.h>
#include <stddef.h>

#ifndef UART8_DMA_RX_BUF_SZ
#define UART8_DMA_RX_BUF_SZ   (512)
#endif
#ifndef UART8_RING_SZ
#define UART8_RING_SZ         (1024)
#endif

extern uint8_t  uart8_dma_rx_buf[UART8_DMA_RX_BUF_SZ];
extern uint8_t  uart8_ring[UART8_RING_SZ];
extern volatile size_t uart8_ring_head;
extern volatile size_t uart8_ring_tail;

void UART8_Ring_OnIdleDMA(UART_HandleTypeDef *huart, size_t rxed);
size_t UART8_Ring_Read(uint8_t *dst, size_t maxlen);
size_t UART8_Ring_Avail(void);

void init_optical_flow_communication(void);
void uart8Interrupt(void);
void OF_Parser_Tick_1kHz(void);

#endif /* OPTICAL_FLOW_UART_RING_H_ */
