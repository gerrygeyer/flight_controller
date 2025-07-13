/*
 * vl53l1x_stm.h
 *
 *  Created on: May 19, 2025
 *      Author: gerrygeyer
 */

#ifndef DISTANCE_DISTANCE_SENSOR_H_
#define DISTANCE_DISTANCE_SENSOR_H_
#include <main.h>

extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c3;
extern I2C_HandleTypeDef hi2c4;

#define VL53L1X_DEFAULT_ADDR 0x52
#define VL53L1X_I2C_ADDR 0x52


void init_lidar_sensors(void);
void init_lidar_minimal(I2C_HandleTypeDef* i2c, GPIO_TypeDef* xshut_port, uint16_t xshut_pin);
void init_lidar(I2C_HandleTypeDef* i2c, GPIO_TypeDef* xshut_port, uint16_t xshut_pin);
void read_distance_lidar1(void);
void read_distance_information(void);

#endif /* DISTANCE_DISTANCE_SENSOR_H_ */
