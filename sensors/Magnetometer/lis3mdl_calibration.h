/*
 * lis3mdl_calibration.h
 *
 *  Created on: Jul 11, 2025
 *      Author: gerrygeyer
 */

#ifndef MAGNETOMETER_LIS3MDL_CALIBRATION_H_
#define MAGNETOMETER_LIS3MDL_CALIBRATION_H_


#ifndef LIS3MDL_CALIBRATION_H_
#define LIS3MDL_CALIBRATION_H_

#include <stdint.h>

typedef struct {
    int16_t offset_x;
    int16_t offset_y;
    int16_t offset_z;
    int16_t scale_x;
    int16_t scale_y;
    int16_t scale_z;
} MagCalibration_t;

void LIS3MDL_Calibration_Init(void);
void LIS3MDL_Calibration_Update(int16_t mx, int16_t my, int16_t mz);
void LIS3MDL_Calibration_Compute(MagCalibration_t *result);

#endif
#endif /* MAGNETOMETER_LIS3MDL_CALIBRATION_H_ */
