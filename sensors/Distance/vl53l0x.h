
#ifndef VL53L0X_H
#define VL53L0X_H

#include <stdint.h>

typedef enum {
    VL53L0X_OK = 0,
    VL53L0X_ERROR = 1
} VL53L0X_StatusTypeDef;

VL53L0X_StatusTypeDef VL53L0X_Init(void);
VL53L0X_StatusTypeDef VL53L0X_StartRanging(void);
VL53L0X_StatusTypeDef VL53L0X_ReadDistance(uint16_t *distance);

VL53L0X_StatusTypeDef VL53L0X_WriteByte(uint8_t reg, uint8_t data);
VL53L0X_StatusTypeDef VL53L0X_ReadByte(uint8_t reg, uint8_t *data);


#endif
