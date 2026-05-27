/*
 * LIS3MDL.h
 *
 *  Created on: Jun 23, 2025
 *      Author: gerrygeyer
 */

#ifndef MAGNETOMETER_LIS3MDL_H_
#define MAGNETOMETER_LIS3MDL_H_

#include "stm32h7xx_hal.h"
#include <stdint.h>
#include <parameter.h>

// Wähle Adresse je nach SDOG-Verbindung:
// SDOG = GND → 0x1C (Standard)
// SDOG = VCC → 0x1E
#define LIS3MDL_I2C_ADDR_LOW   (0x1C << 1)  // 0x38
#define LIS3MDL_I2C_ADDR_HIGH  (0x1E << 1)  // 0x3C
#define LIS3MDL_I2C_ADDR       LIS3MDL_I2C_ADDR_LOW  // <== Hier ggf. Adresse ändern

extern I2C_HandleTypeDef hi2c1;

/**
 * @brief Initialisiert den LIS3MDL Magnetometer (I2C, DRDY auf INT)
 */
HAL_StatusTypeDef LIS3MDL_Init(void);

/**
 * @brief Liest die aktuellen Magnetfeldwerte (in gauss, roh) aus
 *
 * @param mx Zeiger auf X-Achse Ergebnis
 * @param my Zeiger auf Y-Achse Ergebnis
 * @param mz Zeiger auf Z-Achse Ergebnis
 */
void LIS3MDL_ReadMagnetometer(int16_t* mx, int16_t* my, int16_t* mz);

/**
 * @brief Callback-Funktion, die vom EXTI ausgelöst wird
 *        Ruft intern LIS3MDL_ReadMagnetometer() auf.
 */
void LIS3MDL_EXTI_Callback(void);

void LIS3MDL_Service(void);

/**
 * @brief     Copies the latest magnetometer data into the sensor fusion handle.
 *
 * @details   Transfers the most recent magnetic field measurements stored in the global variables
 *            (`global_mx`, `global_my`, `global_mz`) into the `mag_t` field of the given `sensor_fusion` struct.
 *            This function is typically called by the sensor fusion algorithm once new data is marked as ready.
 *
 * @param     pHandle Pointer to the sensor fusion structure that will receive the updated magnetometer values.
 *
 * @note      The global variables must be updated (e.g., via I²C callback) before calling this function.
 *
 * @see       HAL_I2C_MemRxCpltCallback(), sensor_fusion
 */
void get_data_mag(void);
void read_data_mag(sensor_fusion *pHandle);


void softiron_apply_q15(const int16_t *mag_raw, int16_t *mag_out);


void hardiron_apply_q15(int16_t *mag_raw);

#endif /* MAGNETOMETER_LIS3MDL_H_ */
