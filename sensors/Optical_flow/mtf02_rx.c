
/**
 * @file    mtf02_rx.c
 * @brief   UART8 (DMA+IDLE) Empfang + Ringbuffer + MicroLink-Parser für MTF-02
 *
 * Datenfluss:
 *   UART8 RX -> DMA (circular) -> s_dma_rx[]  --(IDLE IRQ)-->  s_ring[]  --(1kHz)--> Parser -> s_last
 */


#include "Optical_flow/mtf02_rx.h"
//#include "mtf02.h"             // Enthält MICOLINK_MSG_t, Payload etc.
#include <string.h>
#include "main.h"

#define MTF01_FRAME_LEN  28

static uint8_t mtf01_rx_buffer[MTF01_FRAME_LEN];

extern UART_HandleTypeDef huart8;
uint8_t debug_counter_optical_flow = 0;

//bool micolink_check_sum(MICOLINK_MSG_t* msg)
//{
//    uint8_t length = msg->len + 6;
//    uint8_t temp[MICOLINK_MAX_LEN];
//    uint8_t checksum = 0;
//
//    memcpy(temp, msg, length);
//
//    for(uint8_t i=0; i<length; i++)
//    {
//        checksum += temp[i];
//    }
//
//    if(checksum == msg->checksum)
//        return true;
//    else
//        return false;
//}
//
//bool micolink_parse_char(MICOLINK_MSG_t* msg, uint8_t data)
//{
//    switch(msg->status)
//    {
//    case 0:
//        if(data == MICOLINK_MSG_HEAD)
//        {
//            msg->head = data;
//            msg->status++;
//        }
//        break;
//
//    case 1:     // device id
//        msg->dev_id = data;
//        msg->status++;
//        break;
//
//    case 2:     // system id
//        msg->sys_id = data;
//        msg->status++;
//        break;
//
//    case 3:     // message id
//        msg->msg_id = data;
//        msg->status++;
//        break;
//
//    case 4:     //
//        msg->seq = data;
//        msg->status++;
//        break;
//
//    case 5:     // payload length
//        msg->len = data;
//        if(msg->len == 0)
//            msg->status += 2;
//        else if(msg->len > MICOLINK_MAX_PAYLOAD_LEN)
//            msg->status = 0;
//        else
//            msg->status++;
//        break;
//
//    case 6:     // payload receive
//        msg->payload[msg->payload_cnt++] = data;
//        if(msg->payload_cnt == msg->len)
//        {
//            msg->payload_cnt = 0;
//            msg->status++;
//        }
//        break;
//
//    case 7:     // check sum
//        msg->checksum = data;
//        msg->status = 0;
//        if(micolink_check_sum(msg))
//        {
//            return true;
//        }
//
//    default:
//        msg->status = 0;
//        msg->payload_cnt = 0;
//        break;
//    }
//
//    return false;
//}
//
//void micolink_decode(uint8_t data)
//{
//    static MICOLINK_MSG_t msg;
//
//    if(micolink_parse_char(&msg, data) == false)
//        return;
//
//    switch(msg.msg_id)
//    {
//        case MICOLINK_MSG_ID_RANGE_SENSOR:
//        {
//            MICOLINK_PAYLOAD_RANGE_SENSOR_t payload;
//            memcpy(&payload, msg.payload, msg.len);
//
//            if (msg.len == sizeof(MICOLINK_PAYLOAD_RANGE_SENSOR_t))  // Sicherstellen, dass Länge stimmt
//                  {
//                __disable_irq();  // 🔐 Kritischen Bereich sichern
//                memcpy(&mtf01_rx_buffer, msg.payload, sizeof(MICOLINK_PAYLOAD_RANGE_SENSOR_t));
//                debug_counter_optical_flow++;
//                __enable_irq();   // 🔓 Wieder freigeben
//                  }
//            /*
//                You can get the sensor data here:
//
//                distance           = payload.distance;
//                distance strength  = payload.strength;
//                distance precision = payload.precision;
//                distance status    = payload.tof_status;
//                flow velocity x    = payload.flow_vel_x;
//                flow velocity y    = payload.flow_vel_y;
//                flow quality       = payload.flow_quality;
//                flow status        = payload.flow_status;
//            */
//            break;
//        }
//
//        default:
//            break;
//        }
//}
//// Letzter Messwert (global für Zugriff)
//
//const MICOLINK_PAYLOAD_RANGE_SENSOR_t* MTF01_GetLatestData(void)
//{
//    static MICOLINK_PAYLOAD_RANGE_SENSOR_t data_copy;
//
//    __disable_irq();  // 🔐 Blockiere kurz alle Interrupts
//    memcpy(&data_copy, &mtf01_rx_buffer, sizeof(MICOLINK_PAYLOAD_RANGE_SENSOR_t));
//    __enable_irq();   // 🔓 Freigeben
//
//    return &data_copy;
//}
//
//void start_mtf02_dma(void)
//{
//    if( HAL_UART_Receive_IT(&huart8, mtf01_rx_buffer, MTF01_FRAME_LEN) != HAL_OK){
//    	uint8_t test = 0;
//    }
//}
//
//void Triggert_UART_RxCpltCallback(void){
//	for (uint8_t i = 0; i < MTF01_FRAME_LEN; i++)
//	        {
//	            micolink_decode(mtf01_rx_buffer[i]);  // Zeichenweise weiterreichen
//	        }
//	 HAL_UART_IRQHandler(&huart8);        // Wichtig: Flags löschen!
//	        // Empfang erneut starten
//	        if(HAL_UART_Receive_IT(&huart8, mtf01_rx_buffer, MTF01_FRAME_LEN) != HAL_OK){
//	        	uint8_t test = 0;
//	        }
//
//}
//
//
////void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
////{
////    if (huart->Instance == UART8)
////    {
////        for (uint8_t i = 0; i < MTF01_FRAME_LEN; i++)
////        {
////            micolink_decode(mtf01_rx_buffer[i]);
////        }
////
////        HAL_UART_Receive_IT(&huart8, mtf01_rx_buffer, MTF01_FRAME_LEN);
////    }
////}
////const MICOLINK_PAYLOAD_RANGE_SENSOR_t *MTF02_GetData(void)
////{
////    return &mtf02_latest;
////}



/* ============================= Konfiguration ============================= */

#ifndef MTF02_DMA_RX_BUF_SZ
#define MTF02_DMA_RX_BUF_SZ     (512u)
#endif

#ifndef MTF02_RING_SZ
#define MTF02_RING_SZ           (2048u)
#endif

#ifndef MTF02_TICK_MAX_BUDGET
#define MTF02_TICK_MAX_BUDGET   (256u)   /* max. Bytes pro 1 kHz Tick */
#endif

/* ============================= Private Daten ============================= */

static UART_HandleTypeDef *s_huart = NULL;

/* DMA-Zielpuffer (wird von HAL/DMA gefüllt) */
__attribute__((used)) __attribute__((section(".dtcmram"))) __attribute__((aligned(32)))
static uint8_t s_dma_rx[MTF02_DMA_RX_BUF_SZ];

/* Software-Ringpuffer */
static uint8_t s_ring[MTF02_RING_SZ];
static volatile size_t s_ring_head = 0;
static volatile size_t s_ring_tail = 0;

/* Parser-State (Vendor-Container) */
static MICOLINK_MSG_t s_msg;

/* Letzte Messung + Flag */
static mtf02_meas_t s_last;
static volatile uint8_t s_has_new = 0u;

mtf02_meas_t m = {0};
optical_flow_data optical_flow_meas;
volatile static bool init_of = 0;

/* ============================= Ring-Helfer =============================== */

static inline size_t ring_avail(void)
{
    uint32_t pm = __get_PRIMASK(); __disable_irq();
    size_t h = s_ring_head, t = s_ring_tail;
    if (!pm) __enable_irq();
    return (h >= t) ? (h - t) : (MTF02_RING_SZ - (t - h));
}

static inline int ring_read_byte(uint8_t *out)
{
    uint32_t pm = __get_PRIMASK(); __disable_irq();
    if (s_ring_head == s_ring_tail) { if (!pm) __enable_irq(); return 0; }
    *out = s_ring[s_ring_tail];
    s_ring_tail = (s_ring_tail + 1u) % MTF02_RING_SZ;
    if (!pm) __enable_irq();
    return 1;
}

static inline void ring_write_byte(uint8_t b)
{
    size_t nxt = (s_ring_head + 1u) % MTF02_RING_SZ;
    if (nxt != s_ring_tail) {
        s_ring[s_ring_head] = b;
        s_ring_head = nxt;
    }
}

//static inline int ring_read_byte(uint8_t *out)
//{
//    if (s_ring_head == s_ring_tail) return 0;
//    *out = s_ring[s_ring_tail];
//    s_ring_tail = (s_ring_tail + 1u) % MTF02_RING_SZ;
//    return 1;
//}

/* ============================= Parser (lokal) ============================ */

static bool rx_checksum_ok(const MICOLINK_MSG_t *msg)
{
    /* Vendor: checksum = Summe über (head..len + payload[len]) */
    uint8_t length = (uint8_t)(msg->len + 6u);        /* 6 Felder: head..len */
    const uint8_t *p = (const uint8_t *)msg;
    uint8_t cs = 0u;
    for (uint8_t i = 0; i < length; i++) cs += p[i];
    return (cs == msg->checksum);
}

static bool rx_parse_byte(MICOLINK_MSG_t *msg, uint8_t data)
{
    switch (msg->status)
    {
        case 0: if (data == MICOLINK_MSG_HEAD) { msg->head = data; msg->status = 1; } break;
        case 1: msg->dev_id = data; msg->status = 2; break;
        case 2: msg->sys_id = data; msg->status = 3; break;
        case 3: msg->msg_id = data; msg->status = 4; break;
        case 4: msg->seq    = data; msg->status = 5; break;
        case 5:
            msg->len = data;
            if (msg->len == 0u)       { msg->status = 7; }
            else if (msg->len > MICOLINK_MAX_PAYLOAD_LEN) { msg->status = 0; }
            else { msg->payload_cnt = 0; msg->status = 6; }
            break;
        case 6:
            msg->payload[msg->payload_cnt++] = data;
            if (msg->payload_cnt == msg->len) { msg->payload_cnt = 0; msg->status = 7; }
            break;
        case 7:
            msg->checksum = data;
            msg->status = 0;
            if (rx_checksum_ok(msg)) return true;
            /* else: fallthrough -> Reset unten */
        default:
            msg->status = 0;
            msg->payload_cnt = 0;
            break;
    }
    return false;
}

static void rx_on_full_frame(const MICOLINK_MSG_t *msg)
{
    if (msg->msg_id != MICOLINK_MSG_ID_RANGE_SENSOR) return;

    if (msg->len != sizeof(MICOLINK_PAYLOAD_RANGE_SENSOR_t)) return;

    MICOLINK_PAYLOAD_RANGE_SENSOR_t p;
    memcpy(&p, msg->payload, sizeof(p));

    s_last.time_ms      = p.time_ms;
    s_last.distance     = p.distance;
    s_last.strength     = p.strength;
    s_last.precision    = p.precision;
    s_last.dis_status   = p.dis_status;
    s_last.flow_vel_x   = p.flow_vel_x;
    s_last.flow_vel_y   = p.flow_vel_y;
    s_last.flow_quality = p.flow_quality;
    s_last.flow_status  = p.flow_status;

    s_has_new = 1u;
}

/* ============================= Öffentliche API =========================== */

void MTF02_RX_Init(UART_HandleTypeDef *huart8)
{
    /* 1. Handle setzen */
    s_huart = huart8;

    /* 2. IDLE-IRQ deaktivieren (jetzt ist s_huart gültig) */
    __HAL_UART_DISABLE_IT(s_huart, UART_IT_IDLE);

    // 3. Interne Zustände zurücksetzen
    memset(&s_msg, 0, sizeof(s_msg));
    memset(&s_last, 0, sizeof(s_last));
    s_has_new = 0u;
    s_ring_head = 0u;
    s_ring_tail = 0u;

    // 4. Alle UART Interrupts abschalten
    __HAL_UART_DISABLE_IT(s_huart, UART_IT_RXNE);
    __HAL_UART_DISABLE_IT(s_huart, UART_IT_PE);
    __HAL_UART_DISABLE_IT(s_huart, UART_IT_ERR);

    // 5. Laufende Transfers stoppen
    HAL_UART_DMAStop(s_huart);
    HAL_UART_AbortReceive(s_huart);

    // 6. Error-Flags löschen
    __HAL_UART_CLEAR_OREFLAG(s_huart);
    __HAL_UART_CLEAR_NEFLAG(s_huart);
    __HAL_UART_CLEAR_FEFLAG(s_huart);
    __HAL_UART_CLEAR_PEFLAG(s_huart);
    __HAL_UART_CLEAR_IDLEFLAG(s_huart);

    s_huart->Instance->CR3 |= USART_CR3_OVRDIS;
    s_huart->ErrorCode = HAL_UART_ERROR_NONE;

    // 7. DMA-Start mit Fehlerprüfung
    HAL_StatusTypeDef st = HAL_UARTEx_ReceiveToIdle_DMA(s_huart, s_dma_rx, MTF02_DMA_RX_BUF_SZ);
    if (st != HAL_OK) {
        Error_Handler();  // oder while(1);
    }

    // 8. Half-Transfer IRQ abschalten (optional, weil wir nur IDLE nutzen)
    if (s_huart->hdmarx) {
        __HAL_DMA_DISABLE_IT(s_huart->hdmarx, DMA_IT_HT);
    }

    // 9. IDLE IRQ aktivieren (jetzt ist alles bereit)
    __HAL_UART_CLEAR_IDLEFLAG(s_huart);
    __HAL_UART_ENABLE_IT(s_huart, UART_IT_IDLE);

    // 10. Init-Flag setzen
    init_of = 1;
}

//void MTF02_RX_OnUartIRQ(void)
//{
//    if (s_huart == NULL) return;
//
//    if (__HAL_UART_GET_FLAG(s_huart, UART_FLAG_IDLE) != RESET) {
//        __HAL_UART_CLEAR_IDLEFLAG(s_huart);
//
//        /* Anzahl empfangener Bytes im DMA-Puffer */
//        size_t pos = (size_t)MTF02_DMA_RX_BUF_SZ - (size_t)__HAL_DMA_GET_COUNTER(s_huart->hdmarx);
//
//        /* In Software-Ring kopieren */
//        for (size_t i = 0; i < pos; i++) {
//            ring_write_byte(s_dma_rx[i]);
//        }
//
//        /* DMA neu armeren */
//        HAL_UARTEx_ReceiveToIdle_DMA(s_huart, s_dma_rx, MTF02_DMA_RX_BUF_SZ);
//        __HAL_DMA_DISABLE_IT(s_huart->hdmarx, DMA_IT_HT);
//    }
//}

void MTF02_RX_Tick1kHz(void)
{
	if (s_huart && s_huart->ErrorCode != HAL_UART_ERROR_NONE) {
	    __HAL_UART_CLEAR_OREFLAG(s_huart);
	    __HAL_UART_CLEAR_NEFLAG(s_huart);
	    __HAL_UART_CLEAR_FEFLAG(s_huart);
	    __HAL_UART_CLEAR_PEFLAG(s_huart);
	    __HAL_UART_CLEAR_IDLEFLAG(s_huart);
	    s_huart->ErrorCode = HAL_UART_ERROR_NONE;
	    HAL_UART_DMAStop(s_huart);
	    HAL_UARTEx_ReceiveToIdle_DMA(s_huart, s_dma_rx, MTF02_DMA_RX_BUF_SZ);
	    if (s_huart->hdmarx) __HAL_DMA_DISABLE_IT(s_huart->hdmarx, DMA_IT_HT);
	}
    size_t avail  = ring_avail();
    size_t budget = (avail < (size_t)MTF02_TICK_MAX_BUDGET) ? avail : (size_t)MTF02_TICK_MAX_BUDGET;

    uint8_t b;
    while (budget-- && ring_read_byte(&b)) {
        if (rx_parse_byte(&s_msg, b)) {
            rx_on_full_frame(&s_msg);
        }
    }
}

bool MTF02_RX_GetLatest(mtf02_meas_t *out)
{
    if (!out) return false;
    if (!s_has_new) return false;
    *out = s_last;
    s_has_new = 0u;
    return true;
}

void Tick_500Hz_Handler(mtf02_meas_t *data)
{
	if(!init_of) return;
    // Bytes aus Ring abarbeiten, Parser füttern
    MTF02_RX_Tick1kHz();

    // gleich danach Messdaten abholen
//    mtf02_meas_t m;
    if (MTF02_RX_GetLatest(&m)) {
    	static uint32_t last_time = 0;
    	if (m.time_ms != last_time){

    		memcpy(data,&m, sizeof(mtf02_meas_t));
			// wir haben jetzt ein neues Paket
//			float h_m  = (m.distance > 0U) ? (0.001f * (float)m.distance) : 0.0f;
//			float vx   = (float)m.flow_vel_x * h_m * 0.01f; // cm/s@1m -> m/s
//			float vy   = (float)m.flow_vel_y * h_m * 0.01f;


    	}
    	last_time = m.time_ms;
        // -> hier direkt weiterverarbeiten oder in FIFO/Queue legen
    }

}

// ganz oben in mtf02_rx.c hast du bereits s_huart, s_dma_rx[], ring_write_byte(), etc.

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if(!init_of) return;
    if (huart != s_huart) return;

    if (Size == 0 || Size > MTF02_DMA_RX_BUF_SZ) goto rearm;  // Guard

    // H7: Cache-Linien INVALIDIEREN vor CPU-Lesen (32-Byte ausrichten)
    uintptr_t a = (uintptr_t)s_dma_rx;
    uintptr_t start = a & ~((uintptr_t)32 - 1);
    uintptr_t end   = (a + Size + 31) & ~((uintptr_t)32 - 1);
    SCB_InvalidateDCache_by_Addr((uint32_t*)start, (int32_t)(end - start));

    for (uint16_t i = 0; i < Size; i++) {
        ring_write_byte(s_dma_rx[i]);   // schreibt nur, wenn Platz; sonst drop
    }

rearm:
    HAL_UARTEx_ReceiveToIdle_DMA(s_huart, s_dma_rx, MTF02_DMA_RX_BUF_SZ);
    if (s_huart->hdmarx) __HAL_DMA_DISABLE_IT(s_huart->hdmarx, DMA_IT_HT);
}


void get_optical_flow_data(optical_flow_data *data){

}


void force_s_dma_rx_visibility(void)
{
    volatile void *p = (void*)s_dma_rx;
    (void)p;
}
