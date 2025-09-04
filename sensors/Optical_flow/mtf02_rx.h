/*
 * mtf02_rx.h
 *
 *  Created on: Aug 30, 2025
 *      Author: gerrygeyer
 */

// Optical_flow/mtf02_rx.h

#ifndef OPTICAL_FLOW_MTF02_RX_H
#define OPTICAL_FLOW_MTF02_RX_H


#include "stm32h7xx_hal.h"
#include <stdbool.h>
#include <stdint.h>
#include "mtf02.h"   // <-- Dein unveränderter Vendor-Header (MICOLINK_* Typen)

#include <parameter.h>

#ifdef __cplusplus
extern "C" {
#endif

/* App-eigene, kompakte Messstruktur (aus Vendor-Payload abgeleitet) */
typedef struct {
    uint32_t time_ms;     /* ms */
    uint32_t distance;    /* mm (0 = nicht verfügbar) */
    uint8_t  strength;
    uint8_t  precision;
    uint8_t  dis_status;
    int16_t  flow_vel_x;  /* cm/s @ 1 m */
    int16_t  flow_vel_y;  /* cm/s @ 1 m */
    uint8_t  flow_quality;
    uint8_t  flow_status;
} mtf02_meas_t;

/* Einmalig nach MX_Init()/UART8 Init aufrufen */
void MTF02_RX_Init(UART_HandleTypeDef *huart8);

/* In UART8_IRQHandler ganz oben aufrufen (IDLE-Handling + Re-Arm) */
void MTF02_RX_OnUartIRQ(void);

/* Im 1 kHz Tick aufrufen (Ring abbauen, Parser füttern) */
void MTF02_RX_Tick1kHz(void);

/* Letzte vollständige Messung abholen; true = neu seit letztem Abruf */
bool MTF02_RX_GetLatest(mtf02_meas_t *out);

void Tick_1kHz_Handler(void);

#ifdef __cplusplus
}
#endif

#endif /* OPTICAL_FLOW_MTF02_RX_H */

