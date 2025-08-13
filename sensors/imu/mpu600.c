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

#define MPU6000_READ_LEN 14  // accel+gyro+temp

uint8_t mpu6000_dma_buf[MPU6000_READ_LEN];
volatile bool mpu6000_dma_ready = false;

int16_xyz accel, gyro;
volatile bool init_flag_mpu;
OFFSET_ACC_GYRO imu_offset;

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

#define MPU6000_RETRY_LIMIT 3

static HAL_StatusTypeDef MPU6000_Read_command_IT(uint8_t reg, uint8_t* data, uint16_t len);
static uint8_t calculate_offset_values(OFFSET_ACC_GYRO *pHandle);
static OFFSET_ACC_GYRO MPU6000_ReadAccelGyro_offset(void);

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
    if (hi2c == &MPU6000_I2C && mpu_state == MPU6000_READ_DATA)
    {
    	mpu6000_data_ready_flag = false;
        mpu_state = MPU6000_IDLE;
        read_pending = false;  // ✅ Lesevorgang abgeschlossen
        stuck_counter = 0;
        if(init_flag_mpu && calculate_offset_values(&imu_offset)){
        	task_imu_sensor_fusion(); // hier wird auf die daten zugegriffen
        }
        // Optional: Callback setzen, z. B.
        // MPU6000_ReadCompleteCallback();
    }
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
	accel.x  = pHandler_sf->acc_t.x = (int16_t)(imu_data[0] << 8 | imu_data[1]) - imu_offset.acc.x;
	accel.y  = pHandler_sf->acc_t.y = (int16_t)(imu_data[2] << 8 | imu_data[3]) - imu_offset.acc.y;
	accel.z  = pHandler_sf->acc_t.z = (int16_t)(imu_data[4] << 8 | imu_data[5]) - imu_offset.acc.z;

	gyro.x  = pHandler_sf->gyro_t.x = (int16_t)(imu_data[8] << 8 | imu_data[9]) - imu_offset.gyro.x;
	gyro.y	= pHandler_sf->gyro_t.y = (int16_t)(imu_data[10] << 8 | imu_data[11]) - imu_offset.gyro.y;
	gyro.z	= pHandler_sf->gyro_t.z = (int16_t)(imu_data[12] << 8 | imu_data[13]) - imu_offset.gyro.z;
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

	HAL_GPIO_WritePin(IMU_ADDR_PORT, IMU_ADDR_PIN, RESET);
	HAL_Delay(100);

    MPU6000_Write(MPU6000_REG_PWR_MGMT_1, 0x80);
    HAL_Delay(100);

    if(HAL_I2C_IsDeviceReady(&MPU6000_I2C, MPU6000_I2C_ADDR, 3, HAL_MAX_DELAY)!= HAL_OK){
    	HAL_Delay(1);
    }

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
	static uint8_t imu_data_counter = 0;
	static uint8_t data_ready = 0;
	if(data_ready == 1) return 1;

	if(imu_data_counter < 100){
		OFFSET_ACC_GYRO imu_data;
		imu_data_counter++;
		imu_data = MPU6000_ReadAccelGyro_offset();
		pHandle->acc.x += imu_data.acc.x;
		pHandle->acc.y += imu_data.acc.y;
		pHandle->acc.z += imu_data.acc.z;
		pHandle->gyro.x += imu_data.gyro.x;
		pHandle->gyro.y += imu_data.gyro.y;
		pHandle->gyro.z += imu_data.gyro.z;
	}else{
		pHandle->acc.x /= 100;
		pHandle->acc.y /= 100;
		pHandle->acc.z /= 100;
		pHandle->acc.z = pHandle->acc.z - 2048;
		pHandle->gyro.x /= 100;
		pHandle->gyro.y /= 100;
		pHandle->gyro.z /= 100;
		data_ready = 1;
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

    gyro.x  = pHandler_sf->gyro_t.x = ((int16_t)(data[8] << 8 | data[9]) - imu_offset.gyro.x);
    gyro.y	= pHandler_sf->gyro_t.y = ((int16_t)(data[10] << 8 | data[11]) - imu_offset.gyro.y);
    gyro.z	= pHandler_sf->gyro_t.z = ((int16_t)(data[12] << 8 | data[13]) - imu_offset.gyro.z);

    return HAL_OK;
}

static OFFSET_ACC_GYRO MPU6000_ReadAccelGyro_offset(void){
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



