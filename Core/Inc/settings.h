/*
 * settings.h
 *
 *  Created on: Dec 18, 2024
 *      Author: Gerry Geyer
 */

#ifndef INC_SETTINGS_H_
#define INC_SETTINGS_H_

#include <parameter.h>

#define DEBUG_MODE ON
//#define DEBUG_MODE OFF

#define ATTITUDE_CONTROL LQR
//#define ATTITUDE_CONTROL PID


#define LOG_DATA				OFF

#define COMMUNICATION_MOTOR		OFF
#define COMMUNICATION_IMU_MAG	ON




#define STEP_FUNCTION		0
#define RAMP_FUNCTION		1
#define UP_DOWN_FUNCTION	2

//#define TEST_FUNCTION STEP_FUNCTION
//#define TEST_FUNCTION RAMP_FUNCTION
#define TEST_FUNCTION UP_DOWN_FUNCTION



#endif /* INC_SETTINGS_H_ */
