/*
 * LIS3MDL.c
 *
 *  Created on: Jun 23, 2025
 *      Author: gerrygeyer
 */


#include "main.h"
#include "lis3mdl.h"
#include <sys_math.h>
#include <sensor_fusion.h>
#include <stdbool.h>

//#define DMA_NOT_USED 1

uint8_t mag_stuck_counter;

uint8_t I2C_buf[6];  // statt DMA
int16_t global_mx, global_my, global_mz, global_mx_buffer,global_my_buffer, global_mz_buffer;
volatile bool lis3mdl_write_data;
xyz_16t debug_mag;
bool mag_init = 0;

/**
 * @brief		function to write register in polling mode
 *
 * @details 	only used in init-process
 */
static void LIS3MDL_WriteReg(uint8_t reg, uint8_t value)
{
    HAL_I2C_Mem_Write(&hi2c1, LIS3MDL_I2C_ADDR,
                      reg, I2C_MEMADD_SIZE_8BIT, &value, 1, HAL_MAX_DELAY);
}


/**
 * @brief     Initializes the LIS3MDL magnetometer via I²C.
 *
 * @details   Performs the basic initialization of the LIS3MDL sensor. This includes setting the CS pin (PB3) high
 *            to activate I²C mode, performing an optional I²C scan to detect connected devices, and configuring
 *            key control registers of the sensor. It also resets the internal stuck counter and performs an initial
 *            dummy read to unlock the DRDY (data ready) interrupt.
 *
 * @note      The CS pin (PB3) must be configured as a GPIO output before calling this function.
 *            This function uses HAL_Delay() and is therefore not suitable for hard real-time contexts.
 *
 * @warning   Must be called before accessing the LIS3MDL sensor. Reinitializing while DRDY is already active
 *            may lead to communication errors.
 *
 * @see       LIS3MDL_WriteReg(), LIS3MDL_ReadMagnetometer()
 */
HAL_StatusTypeDef LIS3MDL_Init(void)
{
	mag_stuck_counter = 0;

	lis3mdl_write_data = false;
	global_mx_buffer = 0;
	global_my_buffer = 0;
	global_mz_buffer = 0;
    // Set PB3 (CS Pin) auf High um I2C zu aktivieren
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
    HAL_Delay(10);

    // I2C Scan (optional)
//    for (uint8_t address = 0x08; address <= 0x77; address++)
//    {
//        if (HAL_I2C_IsDeviceReady(&hi2c1, address << 1, 2, 10) == HAL_OK)
//        {
//            HAL_Delay(10);
//        }
//    }
//
//    if (HAL_I2C_IsDeviceReady(&hi2c1, LIS3MDL_I2C_ADDR, 3, HAL_MAX_DELAY) != HAL_OK)
//    {
//        HAL_Delay(1);
//    }

    // Konfiguration
    LIS3MDL_WriteReg(0x20, 0b01111100); // CTRL_REG1: 80Hz, UHP
    LIS3MDL_WriteReg(0x21, 0x00);       // CTRL_REG2: ±4 gauss
    LIS3MDL_WriteReg(0x22, 0x00);       // CTRL_REG3: Continuous mode
    LIS3MDL_WriteReg(0x23, 0b00001100); // CTRL_REG4: UHP Z
    LIS3MDL_WriteReg(0x22, 0x04);       // CTRL_REG3: DRDY enable

    HAL_Delay(10);

    // Initialer Lesezugriff zum Freischalten von DRDY
    uint8_t dummy[6];
    HAL_StatusTypeDef return_value = HAL_I2C_Mem_Read(&hi2c1, LIS3MDL_I2C_ADDR,
                     0x28 | 0x80, I2C_MEMADD_SIZE_8BIT, dummy, 6, HAL_MAX_DELAY);
    mag_init = 1;
    return return_value;
}

/**
 * @brief     Handles EXTI interrupt triggered by the LIS3MDL DRDY signal.
 *
 * @details   This function is intended to be called when the LIS3MDL asserts its data-ready (DRDY) interrupt pin.
 *            It checks whether the I²C peripheral is ready and, if so, initiates a non-blocking (interrupt-based)
 *            read of the latest magnetometer data registers (6 bytes: X, Y, Z).
 *
 * @note      The function assumes that `I2C_buf` points to a valid 6-byte buffer.
 *            This is a lightweight callback and should return quickly.
 *
 * @warning   The I²C bus (hi2c1) must not be busy when the interrupt occurs.
 *            Make sure no other I²C operations are running concurrently.
 *
 * @see       HAL_I2C_Mem_Read_IT(), LIS3MDL_Init()
 */

void LIS3MDL_EXTI_Callback(void)
{
    if (HAL_I2C_GetState(&hi2c1) == HAL_I2C_STATE_READY)
    {
        HAL_I2C_Mem_Read_IT(&hi2c1, LIS3MDL_I2C_ADDR,
                            0x28 | 0x80, I2C_MEMADD_SIZE_8BIT,
                            I2C_buf, 6);
    }
}

static xyz_16t hard_iron_measurement(int16_t mx, int16_t my, int16_t mz){
	xyz_16t mean_val, zero_output = {0};

	static bool finish_flag = 0;
	static uint8_t counter = 0;
	static xyz_32t store_data = {0};

	const durations = 100;

	if(counter < durations){
		store_data.x += mx;
		store_data.y += my;
		store_data.z += mz;
	}else{
		finish_flag = 1;
		mean_val.x = CLAMP_INT32_TO_INT16(store_data.x/durations);
		mean_val.y = CLAMP_INT32_TO_INT16(store_data.y/durations);
		mean_val.z = CLAMP_INT32_TO_INT16(store_data.z/durations);
	}
	counter++;

	if (finish_flag) return mean_val;
	return zero_output;
}

/**
 * @brief     Callback executed after I²C memory read is complete (non-blocking).
 *
 * @details   This function is automatically called by the HAL when a memory read operation using
 *            `HAL_I2C_Mem_Read_IT()` completes. If the transfer was triggered by I2C1, the received magnetometer
 *            data from the LIS3MDL is parsed and stored in global variables (`global_mx`, `global_my`, `global_mz`).
 *            It also resets the `mag_stuck_counter` and signals to the sensor fusion algorithm that new data is ready.
 *
 * @param     hi2c Pointer to the I2C handle structure (typically &hi2c1).
 *
 * @note      The function assumes that 6 bytes of data have been received into `I2C_buf`
 *            in little-endian format (LSB first) as output by the LIS3MDL.
 *
 * @see       HAL_I2C_Mem_Read_IT(), mag_ready()
 */
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C1)
    {
    	mag_stuck_counter = 0;
    	lis3mdl_write_data = true;
        global_mx = (int16_t)(I2C_buf[1] << 8 | I2C_buf[0]);
        global_my = (int16_t)(I2C_buf[3] << 8 | I2C_buf[2]);
        global_mz = (int16_t)(I2C_buf[5] << 8 | I2C_buf[4]);
        lis3mdl_write_data = false;


    	global_mx_buffer = global_mx;
    	global_my_buffer = global_my;
    	global_mz_buffer = global_mz;


        mag_ready();  // Signals that magnetometer data is ready for use (defined in sensor_fusion.c)


    }
}


/**
 * @brief     Periodic service routine for LIS3MDL magnetometer health monitoring.
 *
 * @details   This function should be called regularly (e.g., every 1 ms) to monitor the I²C communication state
 *            with the LIS3MDL magnetometer. It uses an internal frequency divider (`freq_counter`) to execute
 *            the actual logic roughly every 13 ms. If the I²C bus is busy for more than one cycle, the function
 *            assumes a stuck state and attempts to restart a non-blocking read of the magnetometer data.
 *
 * @note      This is a fallback mechanism in case the EXTI or DRDY-triggered read fails. It helps recover
 *            from I²C communication hangs. The function relies on `HAL_I2C_GetState()` to detect bus readiness.
 *
 * @warning   Make sure this function is called from a context that allows I²C operations (e.g., not from inside an ISR).
 *
 * @see       HAL_I2C_Mem_Read_IT(), HAL_I2C_GetState(), LIS3MDL_EXTI_Callback()
 */
void LIS3MDL_Service(void)
{
	static uint8_t freq_counter = 0;
	if(freq_counter < 13){
		freq_counter ++;
	}else{

		freq_counter = 0;


		if (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY)
		{
			mag_stuck_counter++;

			if (mag_stuck_counter > 1)
			{
				// Neuversuch
				HAL_StatusTypeDef test_report = HAL_I2C_Mem_Read_IT(&hi2c1, LIS3MDL_I2C_ADDR,
									0x28 | 0x80, I2C_MEMADD_SIZE_8BIT,
									I2C_buf, 6);

				// optional: Zähler inkrementieren für Debug-Zwecke
				// lis3mdl_restart_counter++;
				mag_stuck_counter = 0;
			}
		}
		else
		{

//			mag_stuck_counter = 0;
		}


	}

}


void read_data_mag(sensor_fusion *pHandle)
{
	if (!lis3mdl_write_data){
		pHandle->mag_t.x = global_mx;
		pHandle->mag_t.y = global_my;
		pHandle->mag_t.z = global_mz;
	}else{
		pHandle->mag_t.x = global_mx_buffer;
		pHandle->mag_t.y = global_my_buffer;
		pHandle->mag_t.z = global_mz_buffer;
	}

}




/**
 * @brief Soft-iron correction matrix
 * @note  Calculated offline in MATLAB and scaled to Q15
 * @see   softiron_apply_q15()
 */

#if (SYSTEM == DRONE)

// Hard-iron offset vector (scaled to Q15-compatible range)
// DEFAULT
//const int16_t hardiron_q15[3] = { -573, -2171, -4990};
//
//const int32_t softiron_q15[3][3] = {
//    {  32768,   4378,   1525 },
//    {   4378,  29344,    623 },
//    {   1525,    623,  28004 }
//};

//const int16_t hardiron_q15[3] = { -538, -1032, -6373 };
//
//// === Soft-Iron Matrix (Q15-normalized, max element = 32768) ===
//const int32_t softiron_q15[3][3] = {
//    {  30357,  -1672,  15855 },
//    {  -1672,  29440,   9463 },
//    {  15855,   9463,  32768 },
//};

//// === Hard-Iron Offset (Q15, ±4 Gauss) ===
//const int16_t hardiron_q15[3] = { -1095, -1901, -6017 };
//
//// === Soft-Iron Matrix (Q15, max=32768) ===
//const int32_t softiron_q15[3][3] = {
//    {  19515,   -234,    753 },
//    {   -234,  20698,    677 },
//    {    753,    677,  32767 },
//};

const int16_t hardiron_q15[3] = { -650, -847, -2794 };

// === Soft-Iron Matrix (Q15, max=32768) ===
const int32_t softiron_q15[3][3] = {
    {  22225,  -8339,  25151 },
    {  11005,  32767,   1139 },
    { -24104,   7272,  23711 },
};


//// === Hard-Iron Offset ===
//const int16_t hardiron_q15[3] = { -1122, 657, -6604 };
//
//// === Soft-Iron Matrix (Q15-normalized, max element = 32768) ===
//const int32_t softiron_q15[3][3] = {
//    {  13008,  -1379,   1523 },
//    {  -1379,  32768, -14661 },
//    {   1523, -14661,  18440 }
//};
//// === Hard-Iron Offset 2. only half ball ===
//const int16_t hardiron_q15[3] = { -2390, -1740, -6546 };
//
//// === Soft-Iron Matrix (Q15-normalized, max element = 32768) ===
//const int32_t softiron_q15[3][3] = {
//    {  14829,   1864,    846 },
//    {   1864,  14879,   1902 },
//    {    846,   1902,  32768 }
//};

//// === Hard-Iron Offset Only important positions===
//const int16_t hardiron_q15[3] = { 22, -1283, -5855 };
//
//// === Soft-Iron Matrix (Q15-normalized, max element = 32768) ===
//const int32_t softiron_q15[3][3] = {
//    {  31220,    855,  -6114 },
//    {    855,  32286,  -2814 },
//    {  -6114,  -2814,  32768 }
//};
//// === Hard-Iron Offset 3.0 ===
//const int16_t hardiron_q15[3] = { -1130, -779, -6371};
//
//// === Soft-Iron Matrix (Q15-normalized, max element = 32768) ===
//const int32_t softiron_q15[3][3] = {
//    {  27533,    452,    822 },
//    {    452,  17690,  -4410 },
//    {    822,  -4410,  32768 }
//};

#endif

#if (SYSTEM == IMU)
//// === Hard-Iron Offset ===
//const int16_t hardiron_q15[3] = { -1859, -2111, -3630 };
//
//// === Soft-Iron Matrix (Q15-normalized, max element = 32768) ===
//const int32_t softiron_q15[3][3] = {
//    {  30071,    374,    988 },
//    {    374,  22206,   -890 },
//    {    988,   -890,  32768 }
//};
//// === Hard-Iron Offset 2. only half ball ===
//const int16_t hardiron_q15[3] = { -2390, -1740, -6546 };
//
//// === Soft-Iron Matrix (Q15-normalized, max element = 32768) ===
//const int32_t softiron_q15[3][3] = {
//    {  14829,   1864,    846 },
//    {   1864,  14879,   1902 },
//    {    846,   1902,  32768 }
//};

//// === Hard-Iron Offset 3.0 ===
//const int16_t hardiron_q15[3] = { -1130, -779, -6371};
//
//// === Soft-Iron Matrix (Q15-normalized, max element = 32768) ===
//const int32_t softiron_q15[3][3] = {
//    {  27533,    452,    822 },
//    {    452,  17690,  -4410 },
//    {    822,  -4410,  32768 }
//};

// DEFAULT WERT
//// === Hard-Iron Offset ===
//const int16_t hardiron_q15[3] = { 22, -1284, -5855 };
//
//// === Soft-Iron Matrix (Q15-normalized, max element = 32768) ===
//const int32_t softiron_q15[3][3] = {
//    {  31220,    855,  -6114 },
//    {    855,  32286,  -2814 },
//    {  -6114,  -2814,  32768 }
//};


//// === Hard-Iron Offset ===
//const int16_t hardiron_q15[3] = { -346, -1780, -2503 };
//
//// === Soft-Iron Matrix (Q15, max=32768) ===
//const int32_t softiron_q15[3][3] = {
//    {  23257,   2090,  -3427 },
//    {   2090,  30635,  -2531 },
//    {  -3427,  -2531,  32767 },
//};


// DEFAULT
// === Hard-Iron Offset ===
const int16_t hardiron_q15[3] = { -563, -1020, -818 };

// === Soft-Iron Matrix (Q15, max=32768) ===
const int32_t softiron_q15[3][3] = {
    { -31729,  -9495,  -4229 },
    {   6838, -32767,   4980 },
    {   3279,  -3730, -32223 },
};
// NUR YAW
//// === Hard-Iron Offset ===
//const int16_t hardiron_q15[3] = { -1.310694e+02, -2.044060e+00, -8.290478e+01 };
//
//// === Soft-Iron Matrix (Q15-normalized, max element = 32768) ===
//const int32_t softiron_q15[3][3] = {
//    {  32768,   1981,  -1438 },
//    {   1981,  13592,   -836 },
//    {  -1438,   -836,   1473 }
//};
#endif

#if (SYSTEM == FRAME)

	// === Hard-Iron Offset ===
	const int16_t hardiron_q15[3] = { -724, 155, -3213 };

	// === Soft-Iron Matrix (Q15, max=32768) ===
	const int32_t softiron_q15[3][3] = {
	    {   4612,  26202,  19721 },
	    {  32767,  -4497,  -1687 },
	    {   1343,  19748, -26551 },
	};
#endif


//const int32_t softiron_q15[3][3] = {
//    {  30588,   3674,  -2487 },
//    {   3674,  29479,  -3683 },
//    {  -2487,  -3683 ,  32768 }
//};


//offset =
//
//   1.0e+03 *
//
//    0.2063   -0.7926   -8.4992;



//const int32_t hardiron_q15[3] = {
//		206, -792, -8499
//};

//2.963923737494823e+03	-2.119397637366230e+03	-5.369925197794294e+03


/**
 * @brief     Applies hard- and soft-iron correction to magnetometer data.
 *
 * @details   Expects Q15-compatible input values. The hard-iron offset is subtracted per axis.
 *            Then, the normalized soft-iron matrix is applied. The matrix is scaled such that
 *            the largest element equals 32768 (i.e., 1.0 in Q15). The computation is performed
 *            in Q30 and scaled back to Q15 by >>2 and >>13.
 *
 * @param     mag_raw   Pointer to input vector (x/y/z), scaled to Q15
 * @param     mag_out   Pointer to output vector (x/y/z), corrected in Q15
 *
 * @note      No preprocessing required – hard-iron compensation is done inside this function.
 */
void softiron_apply_q15(const int16_t *mag_raw, int16_t *mag_out){

    for (int i = 0; i < 3; i++) {
        int32_t acc = 0;
        for (int j = 0; j < 3; j++) {
//            acc += (((int32_t)softiron_q15[i][j] * (mag_raw[j] - hardiron_q15[j])) >> 2);
            acc += (((int32_t)softiron_q15[i][j] * (mag_raw[j])) >> 2);
        }
        acc = (acc >> 13);
        mag_out[i] = CLAMP_INT32_TO_INT16(acc);
    }

}


void hardiron_apply_q15(int16_t *mag_raw){
	mag_raw[0] = CLAMP_INT32_TO_INT16((int32_t)mag_raw[0] - hardiron_q15[0]);
	mag_raw[1] = CLAMP_INT32_TO_INT16((int32_t)mag_raw[1] - hardiron_q15[1]);
	mag_raw[2] = CLAMP_INT32_TO_INT16((int32_t)mag_raw[2] - hardiron_q15[2]);
}


