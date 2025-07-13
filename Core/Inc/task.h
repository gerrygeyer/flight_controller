/*
 * task.h
 *
 *  Created on: Oct 25, 2024
 *      Author: Gerry Geyer
 */

#ifndef INC_TASK_H_
#define INC_TASK_H_
#include <parameter.h>

void task_init(void);
void time_management(void);
at_control_f create_attitude_control_signals(void);
void system_stop_function(void);

#define SYSTEM_STOP		0
#define SYSTEM_START 	1

#define TIME_PERIOD_SHORT 	500	// 5s
#define TIME_PERIOD 		1000	// 10s
#define TIME_1				0
#define TIME_2				1
#define TIME_3				2
#define TIME_4				3
#define TIME_5				4
#define TIME_6				5
#define TIME_7				6
#define TIME_8				7
#define TIME_9				8
#define TIME_10				9
#define TIME_11				10
#define TIME_12				11
#define TIME_13				12
#define TIME_14				13

#endif /* INC_TASK_H_ */
