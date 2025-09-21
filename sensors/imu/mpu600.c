/*
 * mpu600.c
 *
 *  Created on: Jun 23, 2025
 *      Author: gerrygeyer
 */


#include "main.h"
#include <mpu600.h>
#include <sys_math.h>
#include <stdbool.h>
#include <sensor_fusion.h>
#include <stdlib.h>
#include <string.h>
#include <encoder.h>
#include <parameter.h>

#define MPU6000_READ_LEN 14  // accel+gyro+temp

bool offset_calc_sucess = 0;

uint8_t mpu6000_dma_buf[MPU6000_READ_LEN];
volatile bool mpu6000_dma_ready = false;

int16_xyz accel, gyro;
volatile bool init_flag_mpu;
OFFSET_ACC_GYRO imu_offset;
xyz_16t gyro_scale_K_Q14;


volatile MPU6000_State_t mpu_state = MPU6000_IDLE;
uint8_t mpu_reg_addr;
uint8_t* mpu_rx_buffer;
uint16_t mpu_rx_len;


uint8_t imu_data[14];
static bool read_pending;
uint8_t mpu_i2c_busy;

uint32_t neustart_counter = 0;

volatile uint32_t tx_callback_counter = 0;
bool mpu6000_data_ready_flag = true;  // gesetzt im EXTI-IRQ
uint8_t stuck_counter;
//static uint8_t mpu6000_retry_counter = 0;
//debug
int16_xyz gyro_raw;


#define MPU6000_RETRY_LIMIT 3

static HAL_StatusTypeDef MPU6000_Read_command_IT(uint8_t reg, uint8_t* data, uint16_t len);
static uint8_t calculate_offset_values(OFFSET_ACC_GYRO *pHandle);


// triggert by interrupt in stm32h7xx_it.c
void MPU6000_EXTI_Callback(void)
{
	mpu6000_data_ready_flag = true;
        if (mpu_state == MPU6000_IDLE && !read_pending)
        {

            HAL_StatusTypeDef status = MPU6000_Read_command_IT(MPU6000_REG_ACCEL_XOUT_H, imu_data, 14);

            if (status == HAL_OK)
            {
                read_pending = true;
            }
            else
            {
                // Optional: Fehlerzählung bei HAL_BUSY etc.
                mpu_i2c_busy++;
            }
        }

}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	tx_callback_counter++;
    if (hi2c == &MPU6000_I2C && mpu_state == MPU6000_WRITE_REG)
    {
        mpu_state = MPU6000_READ_DATA;
        HAL_I2C_Master_Receive_IT(&MPU6000_I2C, MPU6000_I2C_ADDR, imu_data, 14);
    }
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	static uint8_t offset_value_calc = 0;
    if (hi2c == &MPU6000_I2C && mpu_state == MPU6000_READ_DATA)
    {
    	mpu6000_data_ready_flag = false;
        mpu_state = MPU6000_IDLE;
        read_pending = false;  // ✅ Lesevorgang abgeschlossen
        stuck_counter = 0;
        if(offset_value_calc == 0){
        	offset_value_calc = calculate_offset_values(&imu_offset);
        }
        if(init_flag_mpu && (offset_value_calc == 1)){
        	task_imu_sensor_fusion(); // hier wird auf die daten zugegriffen
        }
        // Optional: Callback setzen, z. B.
        // MPU6000_ReadCompleteCallback();
    }
}

static void LED_heartbeat_fast(void){
	static uint16_t counter = 0;
		if(counter > 256){
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET); // LED AN
		}else{
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);   // LED AUS
		}
	counter++;
	if(counter > 512) counter = 0;


}

uint8_t calculate_offset_values_imu(void){
	uint32_t dummy_counter = 0;
	if (calculate_offset_values(&imu_offset)){
		return 1;
	}else{
		// do nothing

		dummy_counter++;
		if(dummy_counter > 100000UL) return 1;
	}
	return 0;
}



// Triggert by task.c wit 500Hz
void MPU6000_Service(void)
{

    if (mpu_state != MPU6000_IDLE)
    {
        stuck_counter++;

        if (stuck_counter > 1)
        {
            mpu_state = MPU6000_IDLE;
            read_pending = false;
            stuck_counter = 0;

            // optional: Fehlerzähler
//            mpu_i2c_stuck++;
            neustart_counter++;
            // versuche Neustart
            MPU6000_ReadAccelGyro_start_IT();
        }
    }
    else
    {
        stuck_counter = 0;
    }
}

static HAL_StatusTypeDef MPU6000_Write(uint8_t reg, uint8_t data) {
    return HAL_I2C_Mem_Write(&MPU6000_I2C, MPU6000_I2C_ADDR, reg, 1, &data, 1, HAL_MAX_DELAY);

}

static HAL_StatusTypeDef MPU6000_Read_command_IT(uint8_t reg, uint8_t* data, uint16_t len)
{
    if (mpu_state != MPU6000_IDLE)
    {

    	// HIER FLAG VON TIMER INTERRUPT !
        // Kommunikation noch nicht abgeschlossen → kein neuer Start erlaubt
        return HAL_BUSY;
    }

    mpu_state = MPU6000_WRITE_REG;
    mpu_reg_addr = reg;
    mpu_rx_buffer = data;
    mpu_rx_len = len;

    // Versuche, die Registeradresse über I2C zu senden
    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit_IT(&MPU6000_I2C, MPU6000_I2C_ADDR, &mpu_reg_addr, 1);

    if (status != HAL_OK)
    {
        // Übertragung fehlgeschlagen → Zustand zurücksetzen
        mpu_state = MPU6000_IDLE;
        return status;
    }

    return HAL_OK;
}


void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c == &MPU6000_I2C)
    {
        mpu_state = MPU6000_IDLE;
        read_pending = false;

        // Vollständiger Reset:
        __HAL_I2C_DISABLE(hi2c);
        __HAL_I2C_ENABLE(hi2c);


    }
}


void MPU6000_ReadAccelGyro_start_IT(void){
	MPU6000_Read_command_IT(MPU6000_REG_ACCEL_XOUT_H, imu_data, 14);
}

void MPU6000_Get_data_IT(sensor_fusion * pHandler_sf){
	int16_t x;
	__disable_irq();
	accel.x  = pHandler_sf->acc_t.x = (int16_t)(imu_data[0] << 8 | imu_data[1]) - imu_offset.acc.x;
	accel.y  = pHandler_sf->acc_t.y = (int16_t)(imu_data[2] << 8 | imu_data[3]) - imu_offset.acc.y;
	accel.z  = pHandler_sf->acc_t.z = (int16_t)(imu_data[4] << 8 | imu_data[5]) - imu_offset.acc.z;

//	gyro.x  = pHandler_sf->gyro_t.x = (int16_t)(imu_data[8] << 8 | imu_data[9]) - imu_offset.gyro.x;
	x = (int16_t)(imu_data[8] << 8 | imu_data[9]) - imu_offset.gyro.x;
	gyro.x  = pHandler_sf->gyro_t.x = CLAMP_INT32_TO_INT16(Q14_SHIFT_ROUND(((int32_t)x * (int32_t)gyro_scale_K_Q14.x)));
//	gyro.y	= pHandler_sf->gyro_t.y = (int16_t)(imu_data[10] << 8 | imu_data[11]) - imu_offset.gyro.y;
	x = (int16_t)(imu_data[10] << 8 | imu_data[11]) - imu_offset.gyro.y;
	gyro.y  = pHandler_sf->gyro_t.y = CLAMP_INT32_TO_INT16(Q14_SHIFT_ROUND(((int32_t)x * (int32_t)gyro_scale_K_Q14.y)));
//	gyro.z	= pHandler_sf->gyro_t.z = (int16_t)(imu_data[12] << 8 | imu_data[13]) - imu_offset.gyro.z;
	x = (int16_t)(imu_data[12] << 8 | imu_data[13]) - imu_offset.gyro.z;
	gyro.z  = pHandler_sf->gyro_t.z = CLAMP_INT32_TO_INT16(Q14_SHIFT_ROUND(((int32_t)x * (int32_t)gyro_scale_K_Q14.z)));

	gyro_raw.x = (int16_t)(imu_data[8] << 8 | imu_data[9]);
	gyro_raw.y = (int16_t)(imu_data[10] << 8 | imu_data[11]);
	gyro_raw.z = (int16_t)(imu_data[12] << 8 | imu_data[13]);
	__enable_irq();
}




static HAL_StatusTypeDef MPU6000_Read(uint8_t reg, uint8_t *data, uint16_t len) {
    return HAL_I2C_Mem_Read(&MPU6000_I2C, MPU6000_I2C_ADDR, reg, 1, data, len, HAL_MAX_DELAY);
    // schreibe mir um in

//    return HAL_I2C_Master_Receive_DMA(&MPU6000_I2C, MPU6000_I2C_ADDR, data, len);
}


HAL_StatusTypeDef MPU6000_Init(void) {
    // Software Reset
	init_flag_mpu = RESET;
	read_pending = false;
	mpu_i2c_busy = 0;
	stuck_counter = 0;

	gyro_scale_K_Q14.x = CLAMP_INT32_TO_INT16((int32_t)(IMU_GYRO_SCAL_X * (float)Q14));
	gyro_scale_K_Q14.y = CLAMP_INT32_TO_INT16((int32_t)(IMU_GYRO_SCAL_Y * (float)Q14));
	gyro_scale_K_Q14.z = CLAMP_INT32_TO_INT16((int32_t)(IMU_GYRO_SCAL_Z * (float)Q14));

	HAL_GPIO_WritePin(IMU_ADDR_PORT, IMU_ADDR_PIN, RESET);
	HAL_Delay(100);

    MPU6000_Write(MPU6000_REG_PWR_MGMT_1, 0x80);
    HAL_Delay(100);

//    if(HAL_I2C_IsDeviceReady(&MPU6000_I2C, MPU6000_I2C_ADDR, 3, HAL_MAX_DELAY)!= HAL_OK){
//    	HAL_Delay(1);
//    }

    // Wake-Up & Clock Source = Gyro Z
    MPU6000_Write(MPU6000_REG_PWR_MGMT_1, 0x01);
//    HAL_Delay(1);
    // DLPF: 44Hz, 1kHz ODR
    MPU6000_Write(MPU6000_REG_CONFIG, 0x03);
//    HAL_Delay(1);
    // Sample Rate = 1kHz
    MPU6000_Write(MPU6000_REG_SMPLRT_DIV, 0x00);
//    HAL_Delay(1);
    // Gyro ±2000°/s
    MPU6000_Write(MPU6000_REG_GYRO_CONFIG, 0x18);
//    HAL_Delay(1);
    // Accel ±16g
    MPU6000_Write(MPU6000_REG_ACCEL_CONFIG, 0x18);
//    HAL_Delay(1);
    // Enable Data Ready Interrup
    MPU6000_Write(MPU6000_REG_INT_ENABLE, 0x01);
//    HAL_Delay(1);
    memset(&imu_offset, 0, sizeof(imu_offset));
    init_flag_mpu = SET;

    HAL_Delay(10);
//	HAL_I2C_Mem_Read_DMA(&MPU6000_I2C, MPU6000_I2C_ADDR,MPU6000_REG_ACCEL_XOUT_H, 1, mpu6000_dma_buf, MPU6000_READ_LEN);
    MPU6000_ReadAccelGyro_start_IT();
    return HAL_OK;
}


static uint8_t calculate_offset_values(OFFSET_ACC_GYRO *pHandle){
	static uint16_t imu_data_counter = 0;
	static uint8_t data_ready = 0;
	static xyz_32t acc_offset = {0};
	static xyz_32t gyro_offset = {0};
	static uint8_t fail_counter = 0;
	xyz_16t result_values_gyro;
	OFFSET_ACC_GYRO imu_data;

	LED_heartbeat_fast();

	imu_data = MPU6000_ReadAccelGyro_offset();

	if(data_ready == 1){
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);   // LED AUS
		return 1;
	}

if(init_flag_mpu){
	if(imu_data_counter < 1000){

		imu_data_counter++;
		acc_offset.x += imu_data.acc.x;
		acc_offset.y += imu_data.acc.y;
		acc_offset.z += imu_data.acc.z;
		gyro_offset.x += imu_data.gyro.x;
		gyro_offset.y += imu_data.gyro.y;
		gyro_offset.z += imu_data.gyro.z;
	}else{
		pHandle->acc.x = acc_offset.x/1000;
		pHandle->acc.y = acc_offset.y/1000;
		pHandle->acc.z = acc_offset.z/1000;
		pHandle->acc.z = pHandle->acc.z - 2048;
		pHandle->gyro.x = gyro_offset.x/1000;
		pHandle->gyro.y = gyro_offset.y/1000;
		pHandle->gyro.z = gyro_offset.z/1000;

		result_values_gyro.x = imu_data.gyro.x - pHandle->gyro.x;
		result_values_gyro.y = imu_data.gyro.y - pHandle->gyro.y;
		result_values_gyro.z = imu_data.gyro.z - pHandle->gyro.z;

		if((abs(result_values_gyro.x) + abs(result_values_gyro.y) + abs(result_values_gyro.z)) > 5){
			imu_data_counter = 0;
			fail_counter++;

			acc_offset.x = 0;
			acc_offset.y = 0;
			acc_offset.z = 0;
			gyro_offset.x = 0;
			gyro_offset.y = 0;
			gyro_offset.z = 0;
		}else{
			fail_counter = 0;
			data_ready = 1;
			offset_calc_sucess = 1;
			if(TEST_STATION_ENCODER == ON) set_encoder_to_zero();
		}
		if(fail_counter > 200){
			if(TEST_STATION_ENCODER == ON) set_encoder_to_zero();
			// error
			return 1;
		}
	}
}
	return 0;
}

uint8_t MPU6000_ReadID(void) {
    uint8_t id = 0;
    MPU6000_Read(MPU6000_REG_WHO_AM_I, &id, 1);
    return id;
}





// Ohne DMA (funktioniert

HAL_StatusTypeDef MPU6000_ReadAccelGyro(sensor_fusion * pHandler_sf) {
    uint8_t data[14];


    if (MPU6000_Read(MPU6000_REG_ACCEL_XOUT_H, data, 14) != HAL_OK) return HAL_ERROR;



    accel.x  = pHandler_sf->acc_t.x = ((int16_t)(data[0] << 8 | data[1]) - imu_offset.acc.x);
    accel.y  = pHandler_sf->acc_t.y = ((int16_t)(data[2] << 8 | data[3]) - imu_offset.acc.y);
    accel.z  = pHandler_sf->acc_t.z = ((int16_t)(data[4] << 8 | data[5]) - imu_offset.acc.z);

    gyro.x  = pHandler_sf->gyro_t.x = CLAMP_INT32_TO_INT16(Q14_SHIFT_ROUND((((int32_t)(data[8] << 8 | data[9]) - imu_offset.gyro.x)) * gyro_scale_K_Q14.x));
    gyro.y	= pHandler_sf->gyro_t.y = ((int16_t)(data[10] << 8 | data[11]) - imu_offset.gyro.y);
    gyro.z	= pHandler_sf->gyro_t.z = ((int16_t)(data[12] << 8 | data[13]) - imu_offset.gyro.z);


    return HAL_OK;
}

OFFSET_ACC_GYRO MPU6000_ReadAccelGyro_offset(void){
	OFFSET_ACC_GYRO Output;
    uint8_t data[14];
    MPU6000_Read(MPU6000_REG_ACCEL_XOUT_H, data, 14);

    Output.acc.x = (int16_t)(data[0] << 8 | data[1]);
    Output.acc.y = (int16_t)(data[2] << 8 | data[3]);
    Output.acc.z = (int16_t)(data[4] << 8 | data[5]);

    Output.gyro.x = (int16_t)(data[8] << 8 | data[9]);
    Output.gyro.y = (int16_t)(data[10] << 8 | data[11]);
    Output.gyro.z = (int16_t)(data[12] << 8 | data[13]);

    return (Output);
}



