/*
 * bno085.c
 *
 *  Created on: May 20, 2025
 *      Author: gerrygeyer
 */


// bno085.c
#include "bno085.h"
#include <string.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>



volatile bool i2c_dma_done = false;
bool init_flag = false;
bool wait_for_interrupt = false;
extern I2C_HandleTypeDef hi2c2;

static DMA_State_t dma_state = DMA_STATE_IDLE;
//static uint8_t prefix[2];
static uint8_t header[4];
static uint8_t payload[MAX_PAYLOAD];
static uint16_t total_length;

static uint8_t prefix[2] __attribute__((aligned(4)));


static float q_from_fixed(int16_t val)
{
    // fixed-point Q14-Format → float [-1,1]
    return (float)val / (1 << 14);
}

Quaternion BNO085_ParseRotationVector(const uint8_t* packet, uint16_t offset)
{
    Quaternion q;

    int16_t qi = (int16_t)((packet[offset + 4] << 8) | packet[offset + 5]);
    int16_t qj = (int16_t)((packet[offset + 6] << 8) | packet[offset + 7]);
    int16_t qk = (int16_t)((packet[offset + 8] << 8) | packet[offset + 9]);
    int16_t qr = (int16_t)((packet[offset +10] << 8) | packet[offset +11]);

    q.i = q_from_fixed(qi);
    q.j = q_from_fixed(qj);
    q.k = q_from_fixed(qk);
    q.real = q_from_fixed(qr);

    return q;
}


RotationVector_t q;

RotationVector_t BNO085_ParsePacket(const uint8_t* packet, uint16_t length)
{
    RotationVector_t quat = {0};

    if (length < 20)
        return quat;

    uint8_t report_id = packet[4];
    if (report_id != 0x05)  // Rotation Vector Report ID
        return quat;

    // Paketlayout ab Byte 5:
    // Little Endian: je 16-bit signed (außer Accuracy)
    int16_t q_i_raw    = (int16_t)(packet[5] | (packet[6] << 8));
    int16_t q_j_raw    = (int16_t)(packet[7] | (packet[8] << 8));
    int16_t q_k_raw    = (int16_t)(packet[9] | (packet[10] << 8));
    int16_t q_real_raw = (int16_t)(packet[11] | (packet[12] << 8));
    uint16_t acc_raw   = (uint16_t)(packet[13] | (packet[14] << 8));

    const float scale = 1.0f / 16384.0f;  // laut Datenblatt

    quat.q_i    = q_i_raw * scale;
    quat.q_j    = q_j_raw * scale;
    quat.q_k    = q_k_raw * scale;
    quat.q_real = q_real_raw * scale;
    quat.accuracy = acc_raw * 0.0001f;  // in radians

    return quat;
}


// Funktion wird über _it.c getriggert.
uint8_t BNO085_Init_interrupt(void)
{
	wait_for_interrupt = true;
	return 0;
}

uint8_t BNO085_Init(void)
{
    // Optional: Adresse festlegen über Pin
    HAL_GPIO_WritePin(BNO085_ADDR_PORT, BNO085_ADDR_PIN, GPIO_PIN_RESET);

    // 1. Hardware-Reset
    HAL_GPIO_WritePin(BNO085_RST_PORT, BNO085_RST_PIN, GPIO_PIN_RESET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(BNO085_RST_PORT, BNO085_RST_PIN, GPIO_PIN_SET);
    HAL_Delay(100);

    // Auf ersten Interrupt warten
    uint32_t timeout = HAL_GetTick() + 200;
    while (!wait_for_interrupt)
    {
        if (HAL_GetTick() > timeout) return 0;
        HAL_Delay(1);
    }
    wait_for_interrupt = false;

    // Optional: check I2C
    if (HAL_I2C_IsDeviceReady(&hi2c2, BNO085_I2C_ADDR, 3, 100) != HAL_OK)
        return 0;

    // Feature aktivieren (z. B. Rotation Vector)
    BNO085_StartFeature(0x05, 5000);  // 200 Hz
    HAL_Delay(100);
    BNO085_EXTI_Callback();

    return 1;
    }


void BNO085_StartFeature(uint8_t reportID, uint32_t interval_us)
{
    uint8_t feature_cmd[17] = {0};

    feature_cmd[0] = 0xFD;
    feature_cmd[1] = reportID;
    feature_cmd[2] = 0x00;
    feature_cmd[3] = 0x00; feature_cmd[4] = 0x00;
    feature_cmd[5] = (uint8_t)(interval_us & 0xFF);
    feature_cmd[6] = (uint8_t)(interval_us >> 8);
    feature_cmd[7] = (uint8_t)(interval_us >> 16);
    feature_cmd[8] = (uint8_t)(interval_us >> 24);
    memset(&feature_cmd[9], 0, 8);

    uint8_t packet[21];
    packet[0] = 0x15;
    packet[1] = 0x00;
    packet[2] = 0x02;  // channel 2 (SensorHub Control)
    packet[3] = 0x00;
    memcpy(&packet[4], feature_cmd, 17);

//    HAL_Delay(2);
    if(HAL_I2C_Master_Transmit(&hi2c2, BNO085_I2C_ADDR, packet, 21, HAL_MAX_DELAY) != HAL_OK){
    	uint8_t test = 0; // debug

    }
}

void BNO085_EXTI_Callback(void)
{
//    uint8_t prefix[2] = {0};
    if (HAL_I2C_Master_Receive(&hi2c2, BNO085_I2C_ADDR, prefix, 2, HAL_MAX_DELAY) != HAL_OK)
        return;

    uint16_t total_length = prefix[0] | ((prefix[1] & 0x3F) << 8);
    if (total_length < 4 || total_length > 512)
        return;

    uint8_t packet[512] = {0};
    if (HAL_I2C_Master_Receive(&hi2c2, BNO085_I2C_ADDR, packet, total_length, HAL_MAX_DELAY) != HAL_OK)
        return;


    for (uint16_t i = 0; i < total_length - 11; i++) {
        if (packet[i] == 0x05) {
            Quaternion q = BNO085_ParseRotationVector(packet, i);

            break; // oder verarbeite mehrere
        }
    }
    // Jetzt ist packet[] das komplette empfangene Paket
    // → hier kannst du Report ID, Quaternion etc. auswerten
}

void BNO085_DMA_StateMachine(void)
{
    switch (dma_state)
    {
        case DMA_STATE_PREFIX:
            total_length = prefix[0] | (prefix[1] << 8);
            if (total_length < 4 || total_length > MAX_PAYLOAD + 4)
            {
                dma_state = DMA_STATE_IDLE;
                break;
            }
            dma_state = DMA_STATE_HEADER;
            if (HAL_I2C_Master_Receive_DMA(&hi2c2, BNO085_I2C_ADDR, header, 4) != HAL_OK)
            {
                dma_state = DMA_STATE_IDLE;
            }
            break;

        case DMA_STATE_HEADER:
            dma_state = DMA_STATE_PAYLOAD;
            if (HAL_I2C_Master_Receive_DMA(&hi2c2, BNO085_I2C_ADDR, payload, total_length - 4) != HAL_OK)
            {
                dma_state = DMA_STATE_IDLE;
            }
            break;

        case DMA_STATE_PAYLOAD:
            // hier: Daten interpretieren (payload[])
            // z.B. Quaternion extrahieren oder Status prüfen
            dma_state = DMA_STATE_IDLE;
            break;

        default:
            dma_state = DMA_STATE_IDLE;
            break;
    }
}




