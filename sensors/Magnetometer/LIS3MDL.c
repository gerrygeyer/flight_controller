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

#define DMA_NOT_USED 1

uint8_t mag_stuck_counter;

uint8_t I2C_buf[6];  // statt DMA
int16_t global_mx, global_my, global_mz;
xyz_16t debug_mag;

static void LIS3MDL_WriteReg(uint8_t reg, uint8_t value)
{
    HAL_I2C_Mem_Write(&hi2c1, LIS3MDL_I2C_ADDR,
                      reg, I2C_MEMADD_SIZE_8BIT, &value, 1, HAL_MAX_DELAY);
}

void LIS3MDL_Init(void)
{
	mag_stuck_counter = 0;

    // Set PB3 (CS Pin) auf High um I2C zu aktivieren
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
    HAL_Delay(10);

    // I2C Scan (optional)
    for (uint8_t address = 0x08; address <= 0x77; address++)
    {
        if (HAL_I2C_IsDeviceReady(&hi2c1, address << 1, 2, 10) == HAL_OK)
        {
            HAL_Delay(10);
        }
    }

    if (HAL_I2C_IsDeviceReady(&hi2c1, LIS3MDL_I2C_ADDR, 3, HAL_MAX_DELAY) != HAL_OK)
    {
        HAL_Delay(1);
    }

    // Konfiguration
    LIS3MDL_WriteReg(0x20, 0b01111100); // CTRL_REG1: 80Hz, UHP
    LIS3MDL_WriteReg(0x21, 0x00);       // CTRL_REG2: ±4 gauss
    LIS3MDL_WriteReg(0x22, 0x00);       // CTRL_REG3: Continuous mode
    LIS3MDL_WriteReg(0x23, 0b00001100); // CTRL_REG4: UHP Z
    LIS3MDL_WriteReg(0x22, 0x04);       // CTRL_REG3: DRDY enable

    HAL_Delay(10);

    // Initialer Lesezugriff zum Freischalten von DRDY
    uint8_t dummy[6];
    HAL_I2C_Mem_Read(&hi2c1, LIS3MDL_I2C_ADDR,
                     0x28 | 0x80, I2C_MEMADD_SIZE_8BIT, dummy, 6, HAL_MAX_DELAY);
}

//// Diese Funktion muss in main.c aktiv bleiben!
//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
//{
//    if (GPIO_Pin == GPIO_PIN_5) // PB5 = INT/DRDY vom LIS3MDL
//    {
//        LIS3MDL_EXTI_Callback();  // Trigger I2C read via Interrupt
//    }
//}

void LIS3MDL_EXTI_Callback(void)
{
    if (HAL_I2C_GetState(&hi2c1) == HAL_I2C_STATE_READY)
    {
        HAL_I2C_Mem_Read_IT(&hi2c1, LIS3MDL_I2C_ADDR,
                            0x28 | 0x80, I2C_MEMADD_SIZE_8BIT,
                            I2C_buf, 6);
    }
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C1)
    {
    	mag_stuck_counter = 0;
        global_mx = (int16_t)(I2C_buf[1] << 8 | I2C_buf[0]);
        global_my = (int16_t)(I2C_buf[3] << 8 | I2C_buf[2]);
        global_mz = (int16_t)(I2C_buf[5] << 8 | I2C_buf[4]);



        mag_ready();  // Hier kommt dein Sensorfusion-Aufruf rein
    }
}

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


void read_data_mag(sensor_fusion *pHandle){

	pHandle->mag_t.x = global_mx;
	pHandle->mag_t.y = global_my;
	pHandle->mag_t.z = global_mz;

}


// ############################################################
//
//static void LIS3MDL_WriteReg(uint8_t reg, uint8_t value)
//{
//    HAL_I2C_Mem_Write(&hi2c1, LIS3MDL_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &value, 1, HAL_MAX_DELAY);
//}
//void LIS3MDL_Init(void)
//{
//	// Set PB3 (CS Pin) auf High um I2C zu aktivieren
//	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
//	HAL_Delay(10);
//
//	if(1){
//	  for (uint8_t address = 0x08; address <= 0x77; address++)
//	    {
//	        if (HAL_I2C_IsDeviceReady(&hi2c1, address << 1, 2, 10) == HAL_OK)
//	        {
//	            HAL_Delay(10);
////	        	printf("I2C device found at address 0x%02X\r\n", address);
//	        }
//	    }
//	}
//
//    if(HAL_I2C_IsDeviceReady(&hi2c1, LIS3MDL_I2C_ADDR, 3, HAL_MAX_DELAY)!= HAL_OK){
//    	HAL_Delay(1);
//    }
//
//    // CTRL_REG1 (0x20): Temperature disable, Ultra-High-Performance XY, ODR = 80 Hz
//    // Bits: TEMP_EN=0, OM=11 (UHP), DO=111 (80Hz), FAST_ODR=0, ST=0
//    LIS3MDL_WriteReg(0x20, 0b01111100); // 0x7C
//
//    // CTRL_REG2 (0x21): Full scale selection ±4 gauss
//    LIS3MDL_WriteReg(0x21, 0x00); // FS = 00 → ±4 gauss
//
//    // CTRL_REG3 (0x22): Continuous-conversion mode (MD = 00)
//    LIS3MDL_WriteReg(0x22, 0x00);
//
//    // CTRL_REG4 (0x23): Ultra-High-Performance Z-axis (OMZ = 11)
//    LIS3MDL_WriteReg(0x23, 0b00001100); // OMZ = 11 (UHP)
//
//    // Optional: Interrupt enable on INT pin (DRDY enable)
//    // Bit 2 (I2_DRDY) of CTRL_REG3 (0x22) → setzen
//    // Kombination von Continuous Mode + DRDY auf INT möglich
//    LIS3MDL_WriteReg(0x22, 0x04);  // Bit 2 = 1 → DRDY signal auf INT pin
//
//    // Hinweis: Die letzte Schreiboperation überschreibt den vorherigen Wert von CTRL_REG3
//    // → Falls du Continuous Mode UND DRDY willst, musst du beide Bits setzen:
//    // 0x00 | 0x04 = 0x04 (das reicht in diesem Fall)
//
//    HAL_Delay(10); // kurze Verzögerung zur Stabilisierung
//
//
////    if(DMA_NOT_USED){
////
//    uint8_t buf[6];
//
//        // OUT_X_L = 0x28, auto-increment aktivieren mit Bit 7 (0x80)
//        HAL_I2C_Mem_Read(&hi2c1, LIS3MDL_I2C_ADDR, 0x28 | 0x80, I2C_MEMADD_SIZE_8BIT, buf, 6, HAL_MAX_DELAY);
////    }else{
////    	HAL_I2C_Mem_Read_DMA(&hi2c1, LIS3MDL_I2C_ADDR, 0x28 | 0x80, I2C_MEMADD_SIZE_8BIT, DMA_buf, 6);
////    }
//}
//
////void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
////{
////    if (GPIO_Pin == GPIO_PIN_5) // PB5 = INT/DRDY vom LIS3MDL
////    {
////        LIS3MDL_EXTI_Callback();  // Magnetfeldwerte auslesen und weiterverarbeiten
////    }
////}
//void LIS3MDL_EXTI_Callback(void)
//{
//    if (HAL_I2C_GetState(&hi2c1) == HAL_I2C_STATE_READY)
//    {
//        HAL_I2C_Mem_Read_DMA(&hi2c1, LIS3MDL_I2C_ADDR,
//                             0x28 | 0x80, I2C_MEMADD_SIZE_8BIT,
//                             DMA_buf, 6);
//    }
//}
//
//void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
//{
//    if (hi2c->Instance == I2C1)
//    {
//        // Konvertiere die DMA-Daten
//        global_mx = (int16_t)(DMA_buf[1] << 8 | DMA_buf[0]);
//        global_my = (int16_t)(DMA_buf[3] << 8 | DMA_buf[2]);
//        global_mz = (int16_t)(DMA_buf[5] << 8 | DMA_buf[4]);
//        mag_ready();
//
//        // Jetzt Sensorfusion oder Weiterverarbeitung
//        // z. B.: madgwick_update(..., acc, gyro, mag, true, beta, dt);
//    }
//}
//void LIS3MDL_EXTI_Callback(void)
//{
//
//    // Rohdaten aus den Magnetometer-Registern auslesen
//    LIS3MDL_ReadMagnetometer(&global_mx, &global_my, &global_mz);
//
//    // Hier kannst du die Werte in deine Sensorfusion übergeben:
//    // z.B.: Madgwick_Update(mx, my, mz);
//    //       oder Werte in eine Queue schreiben
//
//    // Debug: Werte über UART oder LED anzeigen
//    // printf("Mag: X=%d, Y=%d, Z=%d\r\n", mx, my, mz);
//}
//
//void LIS3MDL_ReadMagnetometer(int16_t* mx, int16_t* my, int16_t* mz)
//{
////	if(DMA_NOT_USED){
////    uint8_t buf[6];
////
////    // OUT_X_L = 0x28, auto-increment aktivieren mit Bit 7 (0x80)
////    HAL_I2C_Mem_Read(&hi2c1, LIS3MDL_I2C_ADDR, 0x28 | 0x80, I2C_MEMADD_SIZE_8BIT, buf, 6, HAL_MAX_DELAY);
////    // gib mir die DMA funktion davon:
////
////    *mx = debug_mag.x = (int16_t)(buf[1] << 8 | buf[0]);
////    *my = debug_mag.y = (int16_t)(buf[3] << 8 | buf[2]);
////    *mz = debug_mag.z = (int16_t)(buf[5] << 8 | buf[4]);
////	}else{
//    if(HAL_I2C_Mem_Read_DMA(&hi2c1, LIS3MDL_I2C_ADDR, 0x28 | 0x80, I2C_MEMADD_SIZE_8BIT, DMA_buf, 6)!= HAL_OK){
//    	uint8_t test;
//    	}
////	}
//}

//void get_data_mag(void){
//
//    global_mx = debug_mag.x = (int16_t)(DMA_buf[1] << 8 | DMA_buf[0]);
//    global_my = debug_mag.y = (int16_t)(DMA_buf[3] << 8 | DMA_buf[2]);
//    global_mz = debug_mag.z = (int16_t)(DMA_buf[5] << 8 | DMA_buf[4]);
//    mag_ready();
//}


