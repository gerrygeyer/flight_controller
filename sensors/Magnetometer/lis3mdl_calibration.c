/*
 * lis3mdl_calibration.c
 *
 *  Created on: Jul 11, 2025
 *      Author: gerrygeyer
 */


#include "lis3mdl_calibration.h"

static int16_t mag_min[3];
static int16_t mag_max[3];
static uint8_t first_sample = 1;

void LIS3MDL_Calibration_Init(void)
{
    first_sample = 1;
}

void LIS3MDL_Calibration_Update(int16_t mx, int16_t my, int16_t mz)
{
    if (first_sample)
    {
        mag_min[0] = mag_max[0] = mx;
        mag_min[1] = mag_max[1] = my;
        mag_min[2] = mag_max[2] = mz;
        first_sample = 0;
    }
    else
    {
        if (mx < mag_min[0]) mag_min[0] = mx;
        if (my < mag_min[1]) mag_min[1] = my;
        if (mz < mag_min[2]) mag_min[2] = mz;

        if (mx > mag_max[0]) mag_max[0] = mx;
        if (my > mag_max[1]) mag_max[1] = my;
        if (mz > mag_max[2]) mag_max[2] = mz;
    }
}

void LIS3MDL_Calibration_Compute(MagCalibration_t *result)
{
    result->offset_x = (mag_max[0] + mag_min[0]) / 2;
    result->offset_y = (mag_max[1] + mag_min[1]) / 2;
    result->offset_z = (mag_max[2] + mag_min[2]) / 2;

    result->scale_x = (mag_max[0] - mag_min[0]) / 2;
    result->scale_y = (mag_max[1] - mag_min[1]) / 2;
    result->scale_z = (mag_max[2] - mag_min[2]) / 2;
}
