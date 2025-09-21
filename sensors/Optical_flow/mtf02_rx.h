/*
 * mtf02_rx.h
 *
 *  Created on: Aug 30, 2025
 *      Author: gerrygeyer
 */

// Optical_flow/mtf02_rx.h

#ifndef OPTICAL_FLOW_MTF02_RX_H
#define OPTICAL_FLOW_MTF02_RX_H


#include "stm32h7xx_hal.h"
#include <stdbool.h>
#include <stdint.h>


#include <parameter.h>

#ifdef __cplusplus
extern "C" {
#endif
//
///* App-eigene, kompakte Messstruktur (aus Vendor-Payload abgeleitet) */
typedef struct {
    uint32_t time_ms;     /* ms */
    uint32_t distance;    /* mm (0 = nicht verfügbar) */
    uint8_t  strength;
    uint8_t  precision;
    uint8_t  dis_status;
    int16_t  flow_vel_x;  /* cm/s @ 1 m */
    int16_t  flow_vel_y;  /* cm/s @ 1 m */
    uint8_t  flow_quality;
    uint8_t  flow_status;
} mtf02_meas_t;

//#include <stdint.h>
//#include <stdbool.h>
//#include <string.h>
//
#define MICOLINK_MSG_HEAD            0xEF
#define MICOLINK_MAX_PAYLOAD_LEN     64
#define MICOLINK_MAX_LEN             MICOLINK_MAX_PAYLOAD_LEN + 7

/*
    Message ID
*/
enum
{
    MICOLINK_MSG_ID_RANGE_SENSOR = 0x51,     // Range Sensor
};

/*
    Message Structure Definition
*/
typedef struct
{
    uint8_t head;
    uint8_t dev_id;
    uint8_t sys_id;
    uint8_t msg_id;
    uint8_t seq;
    uint8_t len;
    uint8_t payload[MICOLINK_MAX_PAYLOAD_LEN];
    uint8_t checksum;

    uint8_t status;
    uint8_t payload_cnt;
} MICOLINK_MSG_t;
//
///*
//    Payload Definition
//*/
//#pragma pack (1)
//// Range Sensor
typedef struct
{
    uint32_t  time_ms;		    // System time in ms
    uint32_t  distance;		    // distance(mm), 0 Indicates unavailable
    uint8_t   strength;	            // signal strength
    uint8_t   precision;	    // distance precision
    uint8_t   dis_status;	    // distance status
    uint8_t  reserved1;	            // reserved
    int16_t   flow_vel_x;	    // optical flow velocity in x
    int16_t   flow_vel_y;	    // optical flow velocity in y
    uint8_t   flow_quality;	    // optical flow quality
    uint8_t   flow_status;	    // optical flow status
    uint16_t  reserved2;	    // reserved
} MICOLINK_PAYLOAD_RANGE_SENSOR_t;
//#pragma pack ()



//void start_mtf02_dma(void);


//const MICOLINK_PAYLOAD_RANGE_SENSOR_t* MTF01_GetLatestData(void);
//void Triggert_UART_RxCpltCallback(void);
//bool mtf02_parse_frame(uint8_t *frame, mtf02_meas_t *out);
//void MTF02_ReadLatest(mtf02_meas_t *out);

/* Einmalig nach MX_Init()/UART8 Init aufrufen */
void MTF02_RX_Init(UART_HandleTypeDef *huart8);

/* In UART8_IRQHandler ganz oben aufrufen (IDLE-Handling + Re-Arm) */
void MTF02_RX_OnUartIRQ(void);

/* Im 1 kHz Tick aufrufen (Ring abbauen, Parser füttern) */
void MTF02_RX_Tick1kHz(void);

/* Letzte vollständige Messung abholen; true = neu seit letztem Abruf */
bool MTF02_RX_GetLatest(mtf02_meas_t *out);

void Tick_500Hz_Handler(mtf02_meas_t *data);

void force_s_dma_rx_visibility(void);

#ifdef __cplusplus
}
#endif

#endif /* OPTICAL_FLOW_MTF02_RX_H */

