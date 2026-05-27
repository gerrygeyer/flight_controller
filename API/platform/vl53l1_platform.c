/**
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

#include "vl53l1_platform.h"
#include "stm32h7xx_hal.h"

// Externes I2C-Handle (I2C1)
extern I2C_HandleTypeDef hi2c1;

// Sensoradresse (Standard 0x29 << 1)
#define VL53L1X_I2C_ADDR   (0x29 << 1)

// Ignoriere den dev Parameter, da nur ein Sensor verwendet wird

int8_t VL53L1_WriteMulti(uint16_t dev, uint16_t index, uint8_t *pdata, uint32_t count) {
    uint8_t buffer[258]; // 2 für Index + bis zu 256 Datenbytes
    buffer[0] = (index >> 8) & 0xFF;  // High Byte
    buffer[1] = index & 0xFF;         // Low Byte
    memcpy(&buffer[2], pdata, count);

    if (HAL_I2C_Master_Transmit(&hi2c1, VL53L1X_I2C_ADDR, buffer, count + 2, HAL_MAX_DELAY) != HAL_OK)
        return -1;
    return 0;
}





int8_t VL53L1_ReadMulti(uint16_t dev, uint16_t index, uint8_t *pdata, uint32_t count) {
    uint8_t index_bytes[2];
    index_bytes[0] = (index >> 8) & 0xFF;
    index_bytes[1] = index & 0xFF;

    // 1. Registeradresse senden
    if (HAL_I2C_Master_Transmit(&hi2c1, VL53L1X_I2C_ADDR, index_bytes, 2, HAL_MAX_DELAY) != HAL_OK)
        return -1;

    // 2. Daten empfangen
    if (HAL_I2C_Master_Receive(&hi2c1, VL53L1X_I2C_ADDR, pdata, count, HAL_MAX_DELAY) != HAL_OK)
        return -1;

    return 0;
}


int8_t VL53L1_WrByte(uint16_t dev, uint16_t index, uint8_t data) {
    return VL53L1_WriteMulti(dev, index, &data, 1);
}

int8_t VL53L1_WrWord(uint16_t dev, uint16_t index, uint16_t data) {
    uint8_t buffer[2];
    buffer[0] = (data >> 8) & 0xFF;
    buffer[1] = data & 0xFF;
    return VL53L1_WriteMulti(dev, index, buffer, 2);
}

int8_t VL53L1_WrDWord(uint16_t dev, uint16_t index, uint32_t data) {
    uint8_t buffer[4];
    buffer[0] = (data >> 24) & 0xFF;
    buffer[1] = (data >> 16) & 0xFF;
    buffer[2] = (data >> 8) & 0xFF;
    buffer[3] = data & 0xFF;
    return VL53L1_WriteMulti(dev, index, buffer, 4);
}

int8_t VL53L1_RdByte(uint16_t dev, uint16_t index, uint8_t *data) {
    return VL53L1_ReadMulti(dev, index, data, 1);
}

int8_t VL53L1_RdWord(uint16_t dev, uint16_t index, uint16_t *data) {
    uint8_t buffer[2];
    int8_t status = VL53L1_ReadMulti(dev, index, buffer, 2);
    if (status != 0) return status;
    *data = (buffer[0] << 8) | buffer[1];
    return 0;
}

int8_t VL53L1_RdDWord(uint16_t dev, uint16_t index, uint32_t *data) {
    uint8_t buffer[4];
    int8_t status = VL53L1_ReadMulti(dev, index, buffer, 4);
    if (status != 0) return status;
    *data = (buffer[0] << 24) | (buffer[1] << 16) | (buffer[2] << 8) | buffer[3];
    return 0;
}

int8_t VL53L1_WaitMs(uint16_t dev, int32_t wait_ms) {
    HAL_Delay(wait_ms);
    return 0;
}
