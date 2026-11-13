/*
 * GPS.h
 *
 *  Created on: Sep 10, 2026
 *      Author: may1
 */

#ifndef INC_GPS_H_
#define INC_GPS_H_
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <stdint.h>
#include "lwip/udp.h"
#include "lwip/pbuf.h"
#include "lwip/ip_addr.h"
#define NMEA_BUF_SIZE 128
void Parse_GPS_Sentence(char *sentence);
typedef struct {
    float utc_time;
    double latitude;
    char lat_dir;
    double longitude;
    char lon_dir;
    uint8_t fix_quality;
    uint8_t num_satellites;
    float hdop;
    float altitude;
} GPS_GGA_Data;

extern struct udp_pcb *gps_udp_pcb;


#endif /* INC_GPS_H_ */
