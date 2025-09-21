/*
 * mtf02_ring_buffer.h
 *
 *  Created on: Sep 8, 2025
 *      Author: gerrygeyer
 */

#ifndef OPTICAL_FLOW_MTF02_RING_BUFFER_H_
#define OPTICAL_FLOW_MTF02_RING_BUFFER_H_

// ring.h
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <parameter.h>


/* ========================= Ringbuffer ========================= */
#ifndef RB_SIZE
#define RB_SIZE 1024   // Potenz von 2 (z.B. 256/512/1024)
#endif

typedef struct {
    uint8_t  buf[RB_SIZE];
    volatile uint16_t head; // schreibt Producer (ISR)
    volatile uint16_t tail; // liest Consumer (Main/Task)
} ring_t;

// verfügbare Bytes im Ring
static inline uint16_t rb_avail(const ring_t *r){
    return (uint16_t)((r->head - r->tail) & (RB_SIZE - 1));
}

// freie Bytes im Ring
static inline uint16_t rb_free(const ring_t *r){
    return (uint16_t)(RB_SIZE - ((r->head - r->tail) & (RB_SIZE - 1)) - 1);
}

// 1 Byte schreiben (droppt, wenn voll)
static inline void rb_write_byte(ring_t *r, uint8_t b){
    uint16_t h = r->head;
    uint16_t n = (uint16_t)((h + 1) & (RB_SIZE - 1));
    if (n != r->tail) { r->buf[h] = b; r->head = n; }
}

// bis zu max Bytes lesen, Rückgabe = Anzahl gelesen
static inline uint16_t rb_read(ring_t *r, uint8_t *dst, uint16_t max){
    uint16_t n = rb_avail(r); if (n > max) n = max;
    for (uint16_t i = 0; i < n; i++) {
        dst[i] = r->buf[r->tail];
        r->tail = (uint16_t)((r->tail + 1) & (RB_SIZE - 1));
    }
    return n;
}

/* Globaler Ring (einmal definieren!) */
extern ring_t s_rb;

/* ========================= MTF02 / Micolink ========================= */
#define MTF01_STX        0xEF
#define MTF01_MSG_ID     0x51
#define MTF01_PAYLOAD_SZ 0x14
#define MTF01_HDR_SZ     6
#define MTF01_CS_SZ      1
#define MTF01_FRAME_LEN  (MTF01_HDR_SZ + MTF01_PAYLOAD_SZ + MTF01_CS_SZ)

///* Payload-Struct (packed) */
//typedef struct __attribute__((packed)) {
//    uint32_t system_time_ms;
//    uint32_t distance_mm;
//    uint8_t  distance_strength;
//    uint8_t  distance_precision;
//    uint8_t  distance_status;
//    uint8_t  reserved1;
//    int16_t  flow_vel_x;
//    int16_t  flow_vel_y;
//    uint8_t  flow_quality;
//    uint8_t  flow_status;
//    uint16_t reserved2;
//} mtf01_payload_t;

/* API */
#ifdef __cplusplus
extern "C" {
#endif

// Startet UART8 RX via ReceiveToIdle DMA (legt DMA-Chunk in .c fest)
HAL_StatusTypeDef UART8_RX_Start(void);
void mtf_drain_dma_to_ring(void);

// Liest genau ein vollständiges Frame aus s_rb und füllt 'out' (true bei Erfolg)
bool MTF01_GetFrame(mtf01_payload_t *out);

#ifdef __cplusplus
}
#endif


#endif /* OPTICAL_FLOW_MTF02_RING_BUFFER_H_ */
