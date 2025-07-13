
#include "vl53l0x.h"
#include "stm32h7xx_hal.h" // Change to your MCU family
#include <string.h>

extern I2C_HandleTypeDef hi2c1;

#define VL53L0X_I2C_ADDR  (0x29 << 1)

VL53L0X_StatusTypeDef VL53L0X_WriteByte(uint8_t reg, uint8_t data) {
    return (HAL_I2C_Mem_Write(&hi2c1, VL53L0X_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, 100) == HAL_OK) ? VL53L0X_OK : VL53L0X_ERROR;
}

VL53L0X_StatusTypeDef VL53L0X_ReadByte(uint8_t reg, uint8_t *data) {
    return (HAL_I2C_Mem_Read(&hi2c1, VL53L0X_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, 1, 100) == HAL_OK) ? VL53L0X_OK : VL53L0X_ERROR;
}

VL53L0X_StatusTypeDef VL53L0X_Init(void) {
    // Magic "start window" fix (ST empfohlen)
    VL53L0X_WriteByte(0x80, 0x01);
    VL53L0X_WriteByte(0xFF, 0x01);
    VL53L0X_WriteByte(0x00, 0x00);
    VL53L0X_WriteByte(0x91, 0x3C);
    VL53L0X_WriteByte(0x00, 0x01);
    VL53L0X_WriteByte(0xFF, 0x00);
    VL53L0X_WriteByte(0x80, 0x00);

    // Ranging-Modus: Continuous
    VL53L0X_WriteByte(0x00, 0x02); // SYSTEM__MODE_START

    // Interrupt auf "new sample ready"
    VL53L0X_WriteByte(0x0A, 0x01); // SYSTEM_INTERRUPT_CONFIG_GPIO
    VL53L0X_WriteByte(0x0C, 0x00); // Active High
    VL53L0X_WriteByte(0x0B, 0x01); // Interrupt clear

    return VL53L0X_OK;
}



VL53L0X_StatusTypeDef VL53L0X_StartRanging(void) {
    VL53L0X_WriteByte(0x00, 0x01); // Start ranging
    return VL53L0X_OK;
}

VL53L0X_StatusTypeDef VL53L0X_ReadDistance(uint16_t *distance) {
    uint8_t dist[2];
    if (HAL_I2C_Mem_Read(&hi2c1, VL53L0X_I2C_ADDR, 0x1E, I2C_MEMADD_SIZE_8BIT, dist, 2, 100) != HAL_OK)
        return VL53L0X_ERROR;
    *distance = (dist[0] << 8) | dist[1];
    return VL53L0X_OK;
}
