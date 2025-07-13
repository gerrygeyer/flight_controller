/*
 * bno085.h
 *
 *  Created on: May 20, 2025
 *      Author: gerrygeyer
 */

/* bno085.h  – Header für BNO085 I²C + Interrupt
 * Author: gerrygeyer
 * Stand: 29 Mai 2025
 */

#ifndef BNO085_H
#define BNO085_H

#include "stm32h7xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

// === I2C-Adresse (7-bit left-shifted) ===
#define BNO085_I2C_ADDR         (0x4A << 1)   // Standard-Adresse bei AD0 = GND

// === Konfigurierbare Pins ===
#define BNO085_ADDR_PORT		GPIOE
#define BNO085_ADDR_PIN			GPIO_PIN_15

#define BNO085_INT_PORT         GPIOB
#define BNO085_INT_PIN          GPIO_PIN_2
#define BNO085_RST_PORT         GPIOE
#define BNO085_RST_PIN          GPIO_PIN_14

// === DMA Puffergröße ===
#define MAX_PAYLOAD 256  // erhöht, damit große Pakete durchkommen

#define INIT_TIMEOUT_MS 200
#define INIT_RETRIES     5
#define RX_BUFFER_SIZE   64

// === DMA-Zustände ===
typedef enum {
    DMA_STATE_IDLE,
    DMA_STATE_PREFIX,
    DMA_STATE_HEADER,
    DMA_STATE_PAYLOAD
} DMA_State_t;

typedef struct {
    float q_i;
    float q_j;
    float q_k;
    float q_real;
    float accuracy;
} RotationVector_t;

typedef struct {
    float real;
    float i;
    float j;
    float k;
} Quaternion;

// === Globale Funktionen ===
uint8_t BNO085_Init(void);
void BNO085_Reset(void);
void BNO085_StartFeature(uint8_t reportID, uint32_t interval_us);
void BNO085_ProcessPacket(void);
void BNO085_EXTI_Callback(void);  // von EXTI ISR aufrufen
void BNO085_DMA_StateMachine(void);
uint8_t BNO085_GetQuat(float* x, float* y, float* z, float* w);
uint8_t BNO085_Init_interrupt(void);

#endif // BNO085_H
