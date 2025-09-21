/*
 * mpu600.h
 *
 *  Created on: Jun 23, 2025
 *      Author: gerrygeyer
 */


#ifndef MPU6000_H_
#define MPU6000_H_

#include "stm32h7xx_hal.h"
#include <parameter.h>

// I2C Handle
#define MPU6000_I2C         hi2c2

// MPU6000 I2C Address (AD0 = 0 → 0x68, AD0 = 1 → 0x69)
//#define MPU6000_I2C_ADDR    (0x68 << 1)  // HAL expects 8-bit addr
#define MPU6000_I2C_ADDR    (0x68 << 1)  // HAL expects 8-bit addr

// MPU6000 Register Map
#define MPU6000_REG_PWR_MGMT_1     0x6B
#define MPU6000_REG_SMPLRT_DIV     0x19
#define MPU6000_REG_CONFIG         0x1A
#define MPU6000_REG_GYRO_CONFIG    0x1B
#define MPU6000_REG_ACCEL_CONFIG   0x1C
#define MPU6000_REG_INT_ENABLE     0x38
#define MPU6000_REG_ACCEL_XOUT_H   0x3B
#define MPU6000_REG_SIGNAL_PATH_RESET 0x68
#define MPU6000_REG_WHO_AM_I       0x75

// GPIO Pins (extern definiert)
#define IMU_ADDR_PORT      GPIOE
#define IMU_ADDR_PIN       GPIO_PIN_15
#define IMU_INT_PORT       GPIOB
#define IMU_INT_PIN        GPIO_PIN_2
#define IMU_RST_PORT       GPIOE
#define IMU_RST_PIN        GPIO_PIN_14

extern I2C_HandleTypeDef MPU6000_I2C;

typedef struct{
	int16_t x;
	int16_t y;
	int16_t z;
}int16_xyz;

typedef struct{
	xyz_32t acc;
	xyz_32t gyro;
}OFFSET_ACC_GYRO;

typedef enum {
    MPU6000_IDLE,
    MPU6000_WRITE_REG,
    MPU6000_READ_DATA
} MPU6000_State_t;



HAL_StatusTypeDef MPU6000_Init(void);
void Read_MPU6000_over_DMA(void);
HAL_StatusTypeDef MPU6000_ReadAccelGyro(sensor_fusion * pHandler_sf);
//HAL_StatusTypeDef MPU6000_ReadAccelGyro(void);
uint8_t MPU6000_ReadID(void);

void get_data_from_imu(void);

void MPU6000_EXTI_Callback(void);

void MPU6000_ReadAccelGyro_start_IT(void);
void MPU6000_Get_data_IT(sensor_fusion * pHandler_sf);

void MPU6000_Service(void);

uint8_t calculate_offset_values_imu(void);
OFFSET_ACC_GYRO MPU6000_ReadAccelGyro_offset(void);

#endif /* IMU_MPU600_H_ */
