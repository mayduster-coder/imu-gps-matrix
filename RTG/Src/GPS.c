#include "GPS.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

GPS_GGA_Data g_gps_data;

static uint8_t GPS_Validate_Checksum(const char *sentence)// function to Check Buffer Validation
{
    if (sentence[0] != '$') return 0;

    uint8_t crc = 0;
    int i = 1;

    while (sentence[i] != '*' && sentence[i] != '\0')
    {
        crc ^= (uint8_t)sentence[i]; // XOR
        i++;
    }

    if (sentence[i] != '*') return 0;

    uint8_t expected_crc = (uint8_t)strtol(&sentence[i + 1], NULL, 16);
    return (crc == expected_crc);
}

void Parse_GPS_Sentence(char *sentence)
{
    if (sentence == NULL) return;

    // valid Checksum
    if (!GPS_Validate_Checksum(sentence))
    {
        return;
    }

    if (strncmp(sentence, "$GPGGA", 6) != 0 && strncmp(sentence, "$GNGGA", 6) != 0)
    {
        return;
    }

    GPS_GGA_Data temp_data = {0};
    char *str_ptr = sentence;
    char *token;
    uint8_t field_index = 0;

    double raw_lat = 0.0;
    double raw_lon = 0.0;

    while ((token = strsep(&str_ptr, ",")) != NULL)
    {
        if (token[0] != '\0')
        {
            switch (field_index)
            {
                case 1:  temp_data.utc_time      = atof(token); break;
                case 2:  raw_lat                = atof(token); break;
                case 3:  temp_data.lat_dir      = token[0];    break;
                case 4:  raw_lon                = atof(token); break;
                case 5:  temp_data.lon_dir      = token[0];    break;
                case 6:  temp_data.fix_quality  = atoi(token); break;
                case 7:  temp_data.num_satellites = atoi(token); break;
                case 8:  temp_data.hdop          = atof(token); break;
                case 9:  temp_data.altitude      = atof(token); break;
                default: break;
            }
        }
        field_index++;
    }

    // calculate Latitude
    if (raw_lat > 0.0)
    {
        int deg = (int)(raw_lat / 100.0);
        double min = raw_lat - (deg * 100.0);
        temp_data.latitude = deg + (min / 60.0);

        if (temp_data.lat_dir == 'S')
            temp_data.latitude = -temp_data.latitude;
    }

    // calculate Longitude
    if (raw_lon > 0.0)
    {
        int deg = (int)(raw_lon / 100.0);
        double min = raw_lon - (deg * 100.0);
        temp_data.longitude = deg + (min / 60.0);

        if (temp_data.lon_dir == 'W')
            temp_data.longitude = -temp_data.longitude;
    }

    // update data
    g_gps_data = temp_data;
}
