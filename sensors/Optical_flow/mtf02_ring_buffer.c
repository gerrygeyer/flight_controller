/*
 * mtf02_ring_buffer.c
 *
 *  Created on: Sep 8, 2025
 *      Author: gerrygeyer
 */


// uart8_rx.c
#include <main.h>
#include "stm32h7xx.h"
#include <mtf02_ring_buffer.h>
#include <parameter.h>


#include <string.h>
#include <stdbool.h>

extern UART_HandleTypeDef huart8;

ring_t s_rb;
__attribute__((aligned(32))) static uint8_t s_dma_rx[256];
static volatile uint16_t s_last = 0;   // letzter kopierter Index

/* --- nur für H7: D-Cache-Inval auf Byte-Range --- */
static inline void dcache_inv_range(uint16_t from, uint16_t to_excl){
    uintptr_t a = (uintptr_t)&s_dma_rx[from];
    uintptr_t b = (uintptr_t)&s_dma_rx[to_excl];
    a &= ~((uintptr_t)32 - 1);
    b  = (b + 31) & ~((uintptr_t)32 - 1);
    SCB_InvalidateDCache_by_Addr((uint32_t*)a, (int32_t)(b - a));
}

/* --- aus DMA-Puffer in Ring kopieren --- */
void mtf_drain_dma_to_ring(void){
    const uint16_t len = sizeof(s_dma_rx);

    if (huart8.hdmarx == NULL) return;   // Safety
    uint16_t ndtr = __HAL_DMA_GET_COUNTER(huart8.hdmarx);
    if (ndtr > len) return;              // Safety (sollte nie passieren)

    uint16_t cur = (uint16_t)(len - ndtr);
    if (cur > len) cur = len;            // Safety
    if (s_last >= len) s_last = 0;       // Safety

    if (cur == s_last) return;

    if (cur > s_last){
        for (uint16_t i=s_last; i<cur && i<len; i++)
            rb_write_byte(&s_rb, s_dma_rx[i]);
    } else {
        for (uint16_t i=s_last; i<len; i++)
            rb_write_byte(&s_rb, s_dma_rx[i]);
        for (uint16_t i=0; i<cur && i<len; i++)
            rb_write_byte(&s_rb, s_dma_rx[i]);
    }
    s_last = cur;
}

/* --- Start: NUR CIRCULAR-DMA, kein ReceiveToIdle --- */
static HAL_StatusTypeDef UART8_RX_Start_Circular(UART_HandleTypeDef *huart, uint8_t *buf, uint16_t len){
    // OVR ignorieren bei DMA (verhindert ORE-Fehler)
    SET_BIT(huart->Instance->CR3, USART_CR3_OVRDIS);

    HAL_UART_AbortReceive(huart);
    if (huart->hdmarx) HAL_DMA_Abort(huart->hdmarx);

    __HAL_UART_CLEAR_PEFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
    __HAL_UART_CLEAR_OREFLAG(huart);
    __HAL_UART_CLEAR_IDLEFLAG(huart);

    s_last = 0;
    HAL_StatusTypeDef st = HAL_UART_Receive_DMA(huart, buf, len);
    if (st != HAL_OK) return st;

    __HAL_DMA_ENABLE_IT(huart->hdmarx, DMA_IT_HT);  // Half-Transfer an
    __HAL_DMA_ENABLE_IT(huart->hdmarx, DMA_IT_TC);  // Transfer-Complete an
    __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);      // optional: IDLE zum schnelleren Nachziehen

    return HAL_OK;
}

HAL_StatusTypeDef UART8_RX_Start(void)
{
    // Retry-Konstanten (lokal definiert)
    const uint32_t retries = 5;
    const uint32_t wait_ms = 5;

    s_rb.head = s_rb.tail = 0;
    s_last = 0;

    if (huart8.hdmarx == NULL) {
        return HAL_ERROR; // __HAL_LINKDMA fehlt
    }

    for (uint32_t i = 0; i < retries; i++)
    {
        // --- Cleanup ---
        HAL_UART_AbortReceive(&huart8);
        if (huart8.hdmarx) HAL_DMA_Abort(huart8.hdmarx);

        // OVR ignorieren (DMA-Usecase)
        SET_BIT(huart8.Instance->CR3, USART_CR3_OVRDIS);

        // Flags + FIFO leeren
        __HAL_UART_CLEAR_PEFLAG(&huart8);
        __HAL_UART_CLEAR_FEFLAG(&huart8);
        __HAL_UART_CLEAR_NEFLAG(&huart8);
        __HAL_UART_CLEAR_OREFLAG(&huart8);
        __HAL_UART_CLEAR_IDLEFLAG(&huart8);
    #if defined(UART_RXDATA_FLUSH_REQUEST)
        __HAL_UART_SEND_REQ(&huart8, UART_RXDATA_FLUSH_REQUEST);
    #else
        while (__HAL_UART_GET_FLAG(&huart8, UART_FLAG_RXNE)) {
            volatile uint32_t d = huart8.Instance->RDR;
            (void)d;
        }
    #endif

        // --- Start: Circular DMA ---
        HAL_StatusTypeDef st = HAL_UART_Receive_DMA(&huart8, s_dma_rx, sizeof(s_dma_rx));
        if (st == HAL_OK) {
            __HAL_DMA_ENABLE_IT(huart8.hdmarx, DMA_IT_HT);
            __HAL_DMA_ENABLE_IT(huart8.hdmarx, DMA_IT_TC);
            __HAL_UART_ENABLE_IT(&huart8, UART_IT_IDLE); // optional
            return HAL_OK;
        }

        // kurze Pause vor erneutem Versuch (nur gültig im Thread-Kontext)
        if (wait_ms) HAL_Delay(wait_ms);
    }

    return HAL_ERROR; // nach allen Versuchen gescheitert
}

///* --- ISRs: sauber trennen --- */
//void DMA1_Stream7_IRQHandler(void){
//    mtf_drain_dma_to_ring();                 // bei HT/TC aus DMA -> Ring
//    HAL_DMA_IRQHandler(&hdma_uart8_rx);
//}
//void UART8_IRQHandler(void){
//    if (__HAL_UART_GET_FLAG(&huart8, UART_FLAG_IDLE) &&
//        __HAL_UART_GET_IT_SOURCE(&huart8, UART_IT_IDLE)) {
//        __HAL_UART_CLEAR_IDLEFLAG(&huart8);
//        mtf_drain_dma_to_ring();             // bei Pause sofort nachziehen
//    }
//    HAL_UART_IRQHandler(&huart8);
//}

/* --- Parser bleibt wie bei dir: liest aus s_rb --- */
bool MTF01_GetFrame(mtf01_payload_t *out){
    static uint8_t buf[MTF01_FRAME_LEN];
    if (rb_avail(&s_rb) < MTF01_FRAME_LEN) return false;

    while (rb_avail(&s_rb) >= MTF01_FRAME_LEN) {
        rb_read(&s_rb, &buf[0], 1);
        if (buf[0] != MTF01_STX) continue;

        for (uint16_t j=1; j<MTF01_FRAME_LEN; j++) {
            if (rb_read(&s_rb, &buf[j], 1) == 0) return false;
        }

        uint8_t cs = 0;
        for (uint16_t j=0; j<MTF01_FRAME_LEN-1; j++) cs += buf[j];
        if (cs != buf[MTF01_FRAME_LEN-1]) continue;

        memcpy(out, &buf[MTF01_HDR_SZ], sizeof(mtf01_payload_t));
        return true;
    }
    return false;
}




