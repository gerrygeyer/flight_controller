/*
 * dwt_delay.c
 *
 *  Created on: Sep 4, 2025
 *      Author: gerrygeyer
 */


//#include "dwt_delay.h"
//
//// Manche M7-Varianten haben einen Lock Access Register (LAR).
//// Das hier schadet nicht, wenn er nicht vorhanden ist.
//static inline void DWT_UnlockIfPresent(void) {
//#if defined(DWT_LAR)
//    DWT->LAR = 0xC5ACCE55; // Unlock
//#endif
//}
//
//bool DWT_TryInit(void)
//{
//    // Trace einschalten
//    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
//
//    DWT_UnlockIfPresent();
//
//    // CYCCNT einschalten
//    DWT->CYCCNT = 0;
//    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
//
//    // Kurz prüfen, ob der Zähler wirklich läuft
//    uint32_t before = DWT->CYCCNT;
//    for (volatile int i = 0; i < 100; ++i) { __asm volatile("nop"); }
//    uint32_t after = DWT->CYCCNT;
//
//    return (after != before);
//}
//
//void DWT_EnableIfNeeded(void)
//{
//    if ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) == 0U) {
//        (void)DWT_TryInit();
//    }
//}
//
//// -------- HAL_Delay-Override --------
///*
// * WICHTIG:
// *  - Diese Funktion ersetzt die __weak HAL_Delay() aus der HAL.
// *  - Sie nutzt DWT->CYCCNT. Falls dieser nicht läuft, gibt es
// *    eine Fallback-Busy-Wait, damit es NIE hängen bleibt.
// *  - Sehr lange Delays werden gekappt, damit 32-bit-CYCCNT-Vergleich stabil bleibt.
// */
//void HAL_Delay(uint32_t Delay_ms)
//{
//    // Maximaldauer so wählen, dass 'cycles' in 32 Bit passt.
//    // cycles = (SystemCoreClock/1000) * ms  <= 0xFFFFFFFF
//    uint32_t cycles_per_ms = (SystemCoreClock / 1000U);
//    if (cycles_per_ms == 0U) { cycles_per_ms = 1U; } // Sicherheitsnetz
//    uint32_t max_ms = 0xFFFFFFFFu / cycles_per_ms;
//    if (Delay_ms > max_ms) {
//        // Kappe sehr lange Delays in Blöcke
//        uint32_t full_chunks = Delay_ms / max_ms;
//        uint32_t rest_ms     = Delay_ms % max_ms;
//        for (uint32_t i = 0; i < full_chunks; ++i) {
//            HAL_Delay(max_ms);
//        }
//        Delay_ms = rest_ms;
//    }
//
//    // Versuche, DWT sicher zu aktivieren (idempotent)
//    DWT_EnableIfNeeded();
//
//    // Prüfen, ob CYCCNT wirklich zählt
//    uint32_t probe0 = DWT->CYCCNT;
//    for (volatile int i = 0; i < 32; ++i) { __asm volatile("nop"); }
//    uint32_t probe1 = DWT->CYCCNT;
//    bool dwt_ok = (probe1 != probe0);
//
//    if (dwt_ok) {
//        uint32_t start  = DWT->CYCCNT;
//        uint32_t target = cycles_per_ms * Delay_ms;
//        while ((uint32_t)(DWT->CYCCNT - start) < target) {
//            // busy wait
//        }
//        return;
//    }
//
//    // ---- Fallback: einfache Busy-Wait-Schleife (grobe Genauigkeit) ----
//    // Kalibrierung: 12 NOPs ~ grobe "Einheit". Passe bei Bedarf an.
//    uint32_t loops = Delay_ms * (SystemCoreClock / 12000U);
//    while (loops--) {
//        __asm volatile("nop");
//    }
//}
