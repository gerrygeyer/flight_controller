/*
 * vl53l1x_stm.c
 *
 *  Created on: May 19, 2025
 *      Author: gerrygeyer
 */

//#include "VL53L1X_api.h"
#include <stdbool.h>
#include <main.h>

#include <communication.h>
#include <Distance/distance_sensor.h>
#include "platform/vl53l1_platform.h"
#include "stm32h7xx_hal.h"
#include "vl53l0x.h"


//VL53L1X_ERROR status;
VL53L0X_StatusTypeDef status;
bool sensor1_init = false;
bool sensor1_init_finish = false;

//
//void VL53L1X_DumpKeyRegisters_Separated(uint16_t dev)
//{
//	// Globale Variablen (z. B. in .c-Datei)
//	uint8_t reg_vhv_loop     = 0;
//	uint8_t reg_phasecal_ovr = 0;
//	uint8_t reg_fastosc      = 0;
//	uint8_t reg_phasecal_to  = 0;
//	uint16_t reg_macrop_a    = 0;
//	uint16_t reg_macrop_b    = 0;
//	uint32_t reg_intermeas   = 0;
//	uint8_t reg_mode_start   = 0;
//	uint8_t reg_gpio_status  = 0;
//	uint8_t reg_range_status = 0;
//	uint8_t reg_model_id     = 0;
//	uint8_t reg_module_type  = 0;
//    VL53L1_RdByte(dev, 0x0006, &reg_vhv_loop);      // VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND
//    VL53L1_RdByte(dev, 0x000B, &reg_phasecal_ovr);  // PHASECAL_CONFIG__OVERRIDE
//    VL53L1_RdByte(dev, 0x002D, &reg_fastosc);       // FAST_OSC_FREQ
//    VL53L1_RdByte(dev, PHASECAL_CONFIG__TIMEOUT_MACROP, &reg_phasecal_to);  // 0x0E
//    VL53L1_RdWord(dev, RANGE_CONFIG__TIMEOUT_MACROP_A_HI, &reg_macrop_a);   // 0x0040
//    VL53L1_RdWord(dev, RANGE_CONFIG__TIMEOUT_MACROP_B_HI, &reg_macrop_b);   // 0x0042
//    VL53L1_RdDWord(dev, VL53L1_SYSTEM__INTERMEASUREMENT_PERIOD, &reg_intermeas); // 0x004C
//    VL53L1_RdByte(dev, SYSTEM__MODE_START, &reg_mode_start);                // 0x0087
//    VL53L1_RdByte(dev, GPIO__TIO_HV_STATUS, &reg_gpio_status);              // 0x0031
//    VL53L1_RdByte(dev, VL53L1_RESULT__RANGE_STATUS, &reg_range_status);     // 0x0089
//    VL53L1_RdByte(dev, 0x010F, &reg_model_id);                              // MODEL_ID
//    VL53L1_RdByte(dev, 0x0110, &reg_module_type);    // MODULE_TYPE
//    uint16_t pll_calib = 0;
//    VL53L1_RdWord(dev, VL53L1_RESULT__OSC_CALIBRATE_VAL, &pll_calib); // 0x00DE
////    HAL_Delay(1);
////    status |= VL53L1X_ClearInterrupt(VL53L1X_DEFAULT_ADDR);
//}
//
//
//void VL53L1X_CheckConfiguration(uint16_t dev)
//{
//    uint8_t phasecal = 0;
//    uint16_t macrop_a = 0, macrop_b = 0;
//    uint32_t inter_measure = 0;
//    uint8_t interrupt_config = 0;
//    uint8_t range_status = 0;
//    uint16_t signal = 0;
//    VL53L1_RdWord(VL53L1X_DEFAULT_ADDR, 0x0098, &signal);
//    VL53L1_RdByte(dev, PHASECAL_CONFIG__TIMEOUT_MACROP, &phasecal);
//    VL53L1_RdWord(dev, RANGE_CONFIG__TIMEOUT_MACROP_A_HI, &macrop_a);
//    VL53L1_RdWord(dev, RANGE_CONFIG__TIMEOUT_MACROP_B_HI, &macrop_b);
//    VL53L1_RdDWord(dev, VL53L1_SYSTEM__INTERMEASUREMENT_PERIOD, &inter_measure);
//    VL53L1_RdByte(dev, SYSTEM__INTERRUPT_CONFIG_GPIO, &interrupt_config);
//    VL53L1_RdByte(dev, VL53L1_RESULT__RANGE_STATUS, &range_status);
//
//}


void init_lidar_sensors(void){
//	init_lidar(&hi2c1, GPIOA, GPIO_PIN_5); // Sensor 1
//	init_lidar_minimal(&hi2c1, GPIOA, GPIO_PIN_7);
    VL53L0X_StatusTypeDef status;
    uint16_t distance_mm = 0;
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
    HAL_Delay(200);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
    HAL_Delay(200);
    // Sensor initialisieren
    status = VL53L0X_Init();
    if (status != VL53L0X_OK) {
        // Fehlerbehandlung (LED blinken etc.)
    	HAL_Delay(1);
        return;
    }

    // Ranging starten
    status = VL53L0X_StartRanging();
    if (status != VL53L0X_OK) {
    	HAL_Delay(1);
        return;
    }

    HAL_Delay(50); // kurze Wartezeit für erstes Ergebnis
    sensor1_init_finish = true;

}

void read_distance_information(void){
	uint8_t int_flag = 0;
	VL53L0X_ReadByte(0x13, &int_flag);
	// Erwartet: 0x01 wenn neuer Wert bereit
	HAL_Delay(1);
}
//
//void init_lidar(I2C_HandleTypeDef* i2c, GPIO_TypeDef* xshut_port, uint16_t xshut_pin)
//{
//#define VL53L1X_DEFAULT_ADDR 0x52 // (0x29 << 1)
//	uint16_t dev = VL53L1X_DEFAULT_ADDR;
//extern I2C_HandleTypeDef hi2c1; // bereits vorhanden
//
//    VL53L1X_ERROR status;
//    uint8_t booted = 0;
//
//    // Sensor über XSHUT resetten (PA7)
//    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
//    HAL_Delay(200);
//    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
//    HAL_Delay(200);
//
//    // Warten bis Sensor gebootet ist
//    while (!booted) {
//        status = VL53L1X_BootState(VL53L1X_DEFAULT_ADDR, &booted);
//        HAL_Delay(2);
//    }
//
//    // Initialisierung
//    status = VL53L1X_SensorInit(VL53L1X_DEFAULT_ADDR);
//    // Konfiguration: Distanzmodus = Long, TimingBudget = 50ms, Messrate = alle 70ms
//    status = VL53L1X_SetDistanceMode(VL53L1X_DEFAULT_ADDR, 2);                // 2 = Long
//    status = VL53L1X_SetTimingBudgetInMs(VL53L1X_DEFAULT_ADDR, 50);           // zulässig: 15,20,33,50,100,200,500
//    status = VL53L1X_SetInterMeasurementInMs(VL53L1X_DEFAULT_ADDR, 70);       // > TimingBudget!
//
//    // Interrupt aktivieren (Low-Aktiv)
//    status = VL53L1X_SetInterruptPolarity(VL53L1X_DEFAULT_ADDR, 0);           // 0 = Low
//
//    // Ranging starten
//    status = VL53L1X_StartRanging(VL53L1X_DEFAULT_ADDR);
//
//
//
//
//
//    sensor1_init = true;
//}


void read_distance_lidar1(void){

//	if(sensor1_init){
//		uint16_t dev = VL53L1X_DEFAULT_ADDR;
//		uint16_t signal = 0, dist = 0;
//		uint8_t status = 0;
//		uint8_t mode = 0;
//		VL53L1_RdByte(dev, SYSTEM__MODE_START, &mode);
//		VL53L1X_GetSignalRate(dev, &signal);
//		VL53L1X_GetDistance(dev, &dist);
//		VL53L1X_GetRangeStatus(dev, &status);
//
////	printf("Signal = %d, Status = %d, Dist = %d\n", signal, status, distance);
//	status |= VL53L1X_ClearInterrupt(VL53L1X_DEFAULT_ADDR);
//
//	}

if(sensor1_init_finish){
    VL53L0X_StatusTypeDef status;
    uint16_t distance_mm = 0;
    status = VL53L0X_ReadDistance(&distance_mm);
    if (status == VL53L0X_OK) {
        // distance_mm enthält Entfernung in mm
        // z. B. über UART oder Debugger ausgeben
    	HAL_Delay(1); // hier ist ein break (zum auslesen von distance_mm)
    } else {
        // Fehlerbehandlung
    	HAL_Delay(1); // hier ist ein break
    }
}
    VL53L0X_WriteByte(0x0B, 0x01); // Interrupt zurücksetzen
}



void read_distance_lidar2(void){

}
void read_distance_lidar3(void){
}
//void init_lidar_minimal(I2C_HandleTypeDef* i2c, GPIO_TypeDef* xshut_port, uint16_t xshut_pin)
//{
//    VL53L1X_ERROR status;
//    uint8_t booted = 0;
//
//    VL53L1X_SetI2CHandle(i2c); // direkt ganz oben
//
//    HAL_GPIO_WritePin(xshut_port, xshut_pin, GPIO_PIN_RESET);
//    HAL_Delay(200);
//    HAL_GPIO_WritePin(xshut_port, xshut_pin, GPIO_PIN_SET);
//    HAL_Delay(200);
//
//    while (!booted) {
//        status = VL53L1X_BootState(VL53L1X_I2C_ADDR, &booted);
//        HAL_Delay(2);
//    }
//
//    status = VL53L1X_SensorInit(VL53L1X_I2C_ADDR);
//    status |= VL53L1X_StartRanging(VL53L1X_I2C_ADDR);
//}

