/**
 * @file    log_data.c
 * @brief   Log data to SD-Card
 * @details DetailedDescription
 *
 * @author  gerrygeyer
 * @date    Jul 13, 2025
 * @version 1.0
 *
 * @copyright MIT License
 */

#include "log_data.h"
#include <string.h>
#include "bsp_driver_sd.h"
#include <parameter.h>
#include <sensor_fusion.h>
#include <sys_math.h>
#include "fatfs.h"  // enthält SDFatFS & SDPath
#include <settings.h>
#include <inttypes.h>   // für PRIu32

/**
 * @brief     Dummy function for SD card detection.
 * @details   Required by FatFS, but the SD card detect pin is not connected on this board.
 * @retval    SD_PRESENT Always returns present.
 */
uint8_t BSP_SD_IsDetected(void)
{
    return SD_PRESENT;
}


FATFS FatFs;
volatile bool log_data_flag;

static FIL file;


//static UINT bytesWritten;

//static uint8_t logBuffer[LOG_BUFFER_SIZE];
//static uint16_t logIndex = 0;
//static bool logReady = false;

static char ring_buffer[LOG_RING_SIZE];
static volatile uint16_t ring_head = 0;
static volatile uint16_t ring_tail = 0;
static uint32_t sample_counter = 0;



static bool find_next_log_filename(char* filename, size_t len);




static inline uint16_t ring_free(void) {
    uint16_t head = ring_head, tail = ring_tail;
    return (tail > head) ? (tail - head - 1)
                         : (LOG_RING_SIZE - (head - tail) - 1);
}

bool Log_WriteBuffered(const char* s) {
    uint16_t len = (uint16_t)strlen(s);
    if (len >= LOG_RING_SIZE) return false;  // Zeile passt gar nicht

    // Blockierend warten, bis genug Platz im Ringpuffer ist
    while (ring_free() < len) {
        Log_ProcessBuffered();  // schafft Platz durch Schreiben
    }

    for (uint16_t i = 0; i < len; i++) {
        ring_buffer[ring_head] = s[i];
        ring_head = (ring_head + 1) % LOG_RING_SIZE;
    }
    return true;
}
//void Log_WriteBuffered(const char* data)
//{
//    uint16_t len = strlen(data);
//    for (uint16_t i = 0; i < len; i++) {
//        uint16_t next = (ring_head + 1) % LOG_RING_SIZE;
//        if (next == ring_tail) break;
//        ring_buffer[ring_head] = data[i];
//        ring_head = next;
//    }
//}




void Log_ProcessBuffered(void)
{
    static uint32_t last_sync_tick = 0;

    if (file.obj.fs == NULL) {
        return; // Dateiobjekt ungültig
    }

    // Berechnen, wie viele Bytes im Puffer liegen
    uint16_t fill = (ring_head >= ring_tail)
                  ? (ring_head - ring_tail)
                  : (LOG_RING_SIZE - ring_tail + ring_head);

    // Solange mindestens ein 512-Byte-Block im Ringpuffer ist, wegschreiben
    while (fill >= 512) {
        uint16_t chunk_len = (ring_head >= ring_tail)
                           ? (ring_head - ring_tail)
                           : (LOG_RING_SIZE - ring_tail);

        // maximal 512 Bytes pro Schreibzugriff
        if (chunk_len > 512) {
            chunk_len = 512;
        }

        UINT written = 0;
        FRESULT res = f_write(&file, &ring_buffer[ring_tail], chunk_len, &written);

        if (res != FR_OK || written == 0) {
            f_close(&file);
            return;
        }

        ring_tail = (ring_tail + written) % LOG_RING_SIZE;
        fill -= written;
    }

    // Alle LOG_SYNC_INTERVAL_MS (z.B. 200 ms) flushen
    if ((HAL_GetTick() - last_sync_tick) >= LOG_SYNC_INTERVAL_MS) {
        f_sync(&file);
        last_sync_tick = HAL_GetTick();
    }
}

//void Log_ProcessBuffered(void)
//{
//    static uint32_t tick_counter = 0;
//
//    if (file.obj.fs == NULL) {
//        // Dateiobjekt ist ungültig
////        const char* msg = "Invalid file handle\r\n";
////        HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
//        return;
//    }
//
//    uint16_t fill = (ring_head >= ring_tail)
//                  ? (ring_head - ring_tail)
//                  : (LOG_RING_SIZE - ring_tail + ring_head);
//
//    if (fill < LOG_WRITE_THRESHOLD && tick_counter < 50) {
//        tick_counter++;
//        return;
//    }
//
//    tick_counter = 0;
//
//    while (ring_tail != ring_head) {
//        uint16_t chunk_len = (ring_head >= ring_tail)
//                           ? (ring_head - ring_tail)
//                           : (LOG_RING_SIZE - ring_tail);
//
//        if (chunk_len == 0) break;
//
//        UINT written = 0;
//        FRESULT res = f_write(&file, &ring_buffer[ring_tail], chunk_len, &written);
//
//        if (res != FR_OK || written == 0) {
////            const char* msg = "f_write failed\r\n";
////            HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
//            f_close(&file);
//            return;
//        }
//
//        ring_tail = (ring_tail + written) % LOG_RING_SIZE;
//    }
//
//    f_sync(&file);
//}

//void Log_TickExample(void)
//{
//    char msg[64];
//    snprintf(msg, sizeof(msg), "Tick: %lu\r\n", HAL_GetTick());
//    Log_WriteBuffered(msg);
//}

bool Log_Init(void)
{
	if (BSP_SD_IsDetected() != SD_PRESENT){
//		HAL_Delay(1);
		return false;
	}

    log_data_flag = false;

    FRESULT fres = f_mount(&FatFs, "", 1);
    if (fres != FR_OK){
//    	HAL_Delay(1);
    	fres = f_mount(&FatFs, "", 1);
    	if (fres != FR_OK) return false;
    }

    char filename[32];
    if (!find_next_log_filename(filename, sizeof(filename)))
        return false;

    fres = f_open(&file, filename, FA_WRITE | FA_CREATE_ALWAYS);
    if (fres != FR_OK)
        return false;

//    f_write(&file, "index,mag_x,mag_y,mag_z\r\n", strlen(header), &written);

    return true;
}

static bool find_next_log_filename(char* filename, size_t len)
{
    for (int i = 1; i < 1000; i++) {
        snprintf(filename, len, "log_%03d.txt", i);
        FILINFO fno;
        if (f_stat(filename, &fno) != FR_OK) {
            return true;
        }
    }
    return false;
}

//void Log_GyroCSV(const xyz_16t *gyro)
//{
//    char msg[64];
//    snprintf(msg, sizeof(msg), "%lu,%d,%d,%d\r\n",
//             sample_counter, gyro->x, gyro->y, gyro->z);
//    Log_WriteBuffered(msg);
//    sample_counter++;
//}

void set_log_data_flag(void)
{
    log_data_flag = true;
}

//void log_data_if_ready(void)
//{
//    if (log_data_flag && LOG_DATA) {
//
//        log_data_flag = false;
//
//        sensor_fusion *sf = read_sensorfusion_data();
//        Log_GyroCSV(&sf->mag_t);  // ggf. später durch gyro ersetzen
//    }
//}
void Log_IMUCSV(const xyz_16t *gyro, const xyz_16t *acc, const xyz_16t *mag, bool mag_valid)
{
    char msg[128];   // größer, da jetzt 10 Werte
    snprintf(msg, sizeof(msg),
             "%" PRIu32 ",%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\r\n",
             sample_counter,
             gyro->x, gyro->y, gyro->z,
             acc->x,  acc->y,  acc->z,
             mag->x,  mag->y,  mag->z,
             mag_valid ? 1 : 0);
    Log_WriteBuffered(msg);
    sample_counter++;
}

void log_data_if_ready(void)
{
    if (log_data_flag && LOG_DATA) {
        log_data_flag = false;

        sensor_fusion *sf = read_sensorfusion_data();
        Log_IMUCSV(&sf->gyro_t, &sf->acc_t, &sf->mag_t, sf->mag_updated);
    }
}




