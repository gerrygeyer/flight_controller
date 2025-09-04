/*
 * EKF.h
 *
 *  Created on: Aug 24, 2025
 *      Author: gerrygeyer
 */

#ifndef INC_EKF_H_
#define INC_EKF_H_


#include <control_functions.h>
#include <orientation.h>
#include <parameter.h>
#include <sys_math.h>
#include <settings.h>
#include <task.h>

#include <string.h>

#define EKF_FRQ	1000

#define FS_GYRO_RAD_S   (34.906585f)        // 2000 dps full-scale
#define FS_ACC_MPS2     (16.0f * 9.80665f)  // 16 g full-scale



void init_EKF(void);




#endif /* INC_EKF_H_ */
