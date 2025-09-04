/*
 * uart_ring.c
 *
 *  Created on: Aug 30, 2025
 *      Author: gerrygeyer
 */


#include "uart_ring.h"
#include <string.h>

extern UART_HandleTypeDef huart8;

uint8_t  uart8_dma_rx_buf[UART8_DMA_RX_BUF_SZ];
uint8_t  uart8_ring[UART8_RING_SZ];
volatile size_t uart8_ring_head = 0;
volatile size_t uart8_ring_tail = 0;

static inline void ring_write_byte(uint8_t b){
    size_t nxt = (uart8_ring_head + 1) % UART8_RING_SZ; /* wenn zeit ist: "%" ersetzen durch "&" da schneller bei n^2 */
    if(nxt != uart8_ring_tail){
        uart8_ring[uart8_ring_head] = b;
        uart8_ring_head = nxt;
    } // overflow -> drop
}

void UART8_Ring_OnIdleDMA(UART_HandleTypeDef *huart, size_t rxed){
    for(size_t i=0;i<rxed;i++){
        ring_write_byte(uart8_dma_rx_buf[i]);
    }
}

size_t UART8_Ring_Avail(void){
    if(uart8_ring_head >= uart8_ring_tail) return uart8_ring_head - uart8_ring_tail;
    return UART8_RING_SZ - uart8_ring_tail + uart8_ring_head;
}

size_t UART8_Ring_Read(uint8_t *dst, size_t maxlen){
    size_t n = UART8_Ring_Avail();
    if(n > maxlen) n = maxlen;
    for(size_t i=0;i<n;i++){
        dst[i] = uart8_ring[uart8_ring_tail];
        uart8_ring_tail = (uart8_ring_tail + 1) % UART8_RING_SZ;
    }
    return n;
}


void init_optical_flow_communication(void){
	  HAL_UARTEx_ReceiveToIdle_DMA(&huart8, uart8_dma_rx_buf, UART8_DMA_RX_BUF_SZ);
	  __HAL_DMA_DISABLE_IT(huart8.hdmarx, DMA_IT_HT); // Half-Transfer-IRQ aus
}

void uart8Interrupt(void){
	 if(__HAL_UART_GET_FLAG(&huart8, UART_FLAG_IDLE) != RESET){
	        __HAL_UART_CLEAR_IDLEFLAG(&huart8);
	        size_t pos = UART8_DMA_RX_BUF_SZ - __HAL_DMA_GET_COUNTER(huart8.hdmarx);
	        UART8_Ring_OnIdleDMA(&huart8, pos);
	        // DMA neu starten
	        HAL_UARTEx_ReceiveToIdle_DMA(&huart8, uart8_dma_rx_buf, UART8_DMA_RX_BUF_SZ);
	        __HAL_DMA_DISABLE_IT(huart8.hdmarx, DMA_IT_HT);
	    }
}

void OF_Parser_Tick_1kHz(void)
{
    const size_t MAX_BUDGET = 64;  // 64 ist ein guter Startwert
    size_t avail = UART8_Ring_Avail();
    size_t budget = (avail < MAX_BUDGET) ? avail : MAX_BUDGET;

    uint8_t b;
    while (budget-- && UART8_Ring_Read(&b, 1) == 1) {
        micolink_decode(b);
    }
}
