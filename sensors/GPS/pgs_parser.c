/*
 * pgs_parser.c
 *
 *  Created on: May 20, 2025
 *      Author: gerrygeyer
 */


#include "gps_parser.h"
#include <stdlib.h>
#include <string.h>

static float convertToDecimal(const char *raw, const char *dir) {
    float deg, min;
    int len = strlen(raw);
    if (len < 4) return 0.0f;
    char deg_buff[3] = {0};
    strncpy(deg_buff, raw, (len > 7 ? 3 : 2));
    deg = atof(deg_buff);
    min = atof(raw + (len > 7 ? 3 : 2));
    float decimal = deg + min / 60.0f;
    if (dir[0] == 'S' || dir[0] == 'W') decimal *= -1.0f;
    return decimal;
}

uint8_t parseGGA(const char *nmea, GGA_Data_t *out) {
    if (!strstr(nmea, "$GPGGA")) return 0;

    char buf[100];
    strncpy(buf, nmea, sizeof(buf));
    buf[sizeof(buf)-1] = '\0';

    char *token = strtok(buf, ",");
    int field = 0;
    while (token) {
        switch (field) {
            case 2: out->latitude = convertToDecimal(token, strtok(NULL, ",")); field++; break;
            case 4: out->longitude = convertToDecimal(token, strtok(NULL, ",")); field++; break;
            case 6: out->fix = atoi(token); break;
            case 7: out->satellites = atoi(token); break;
            case 9: out->altitude = atof(token); break;
        }
        token = strtok(NULL, ",");
        field++;
    }
    return 1;
}

uint8_t parseRMC(const char *nmea, RMC_Data_t *out) {
    if (!strstr(nmea, "$GPRMC")) return 0;

    char buf[100];
    strncpy(buf, nmea, sizeof(buf));
    buf[sizeof(buf)-1] = '\0';

    char *token = strtok(buf, ",");
    int field = 0;
    while (token) {
        switch (field) {
            case 3: out->latitude = convertToDecimal(token, strtok(NULL, ",")); field++; break;
            case 5: out->longitude = convertToDecimal(token, strtok(NULL, ",")); field++; break;
            case 7: out->speed_knots = atof(token); break;
            case 8: out->course_deg = atof(token); break;
        }
        token = strtok(NULL, ",");
        field++;
    }
    return 1;
}
