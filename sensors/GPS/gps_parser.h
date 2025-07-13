/*
 * gps_parser.h
 *
 *  Created on: May 20, 2025
 *      Author: gerrygeyer
 */


#ifndef GPS_PARSER_H
#define GPS_PARSER_H

#include <stdint.h>

typedef struct {
    float latitude;     // in Dezimalgrad
    float longitude;    // in Dezimalgrad
    float altitude;     // in Meter
    uint8_t fix;        // 0 = kein Fix, 1 = GPS-Fix
    uint8_t satellites;
} GGA_Data_t;

typedef struct {
    float latitude;     // in Dezimalgrad
    float longitude;    // in Dezimalgrad
    float speed_knots;  // Geschwindigkeit über Grund
    float course_deg;   // Richtung über Grund
} RMC_Data_t;

uint8_t parseGGA(const char *nmea, GGA_Data_t *out);
uint8_t parseRMC(const char *nmea, RMC_Data_t *out);

#endif


