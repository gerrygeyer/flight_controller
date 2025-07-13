/*
 * orientation.h
 *
 *  Created on: Oct 25, 2024
 *      Author: Gerry Geyer
 */



#ifndef INC_ORIENTATION_H_
#define INC_ORIENTATION_H_

#include<parameter.h>
void init_orientation(void);
void get_orientation(at_angl_f *pHandle);
void read_orientation(void);
void Sensor_Init(uint8_t calib);
void get_angular_rate(at_angl_f *pHandle);

void read_imu_values(void);
#endif /* INC_ORIENTATION_H_ */
