#include "gps.h"
#include "main.h"

extern gngga_t gps;

static float nmea_to_deg(float val);
int parse_gngga(char *line, gngga_t *out);

uint8_t GPS_Process_Data(uint8_t *data, uint16_t len){
    uint16_t i = 0;

    uint8_t wbuf[256];

    memset(wbuf, 0, sizeof(wbuf));

    if (data[0] != '$' || memcmp(data + 3, "GGA", 3) != 0) return GPS_PARSE_ERR;

    for (; i < sizeof(wbuf); i++) {
        if (data[i] == '\r' && data[i+1] == '\n') break;
    }

    memcpy(wbuf, data, i);

    // sscanf(wbuf, "$GNGGA,%f,%f,N,%f,E,%hhd,%hhd,%f,%f,M,%f,M,,*%hhd", &time, &lon, &lat, &fix, &sat, &HDOP, &alt, &geo, &crc);
    parse_gngga(wbuf, &gps);

    uint8_t calc_crc = 0;
    
    for (i = 1; i < len; i++) {
        if (data[i] == '*') break;
        calc_crc ^= data[i];
    }

    gps.valid = (calc_crc == gps.crc);
}

static float nmea_to_deg(float val)
{
    int deg = (int)(val / 100);
    float min = val - (deg * 100);
    return deg + (min / 60.0f);
}

int parse_gngga(char *line, gngga_t *out)
{
    char *p = line;
    char *fields[15] = {0};
    int field_count = 0;

    const char *asterisk = strchr(line, '*');
    if (!asterisk || strlen(asterisk) < 3) return -1;

    // Convert two hex chars after '*' to uint8_t
    out->crc = (uint8_t)strtol(asterisk + 1, NULL, 16);

    // split by ',' (in-place)
    while (*p && field_count < 15)
    {
        fields[field_count++] = p;

        while (*p && *p != ',' && *p != '*')
            p++;

        if (*p == ',' || *p == '*')
        {
            *p = '\0';
            p++;
        }
    }

    // minimal validation
    if (field_count < 10)
        return -3;

    // parse fields
    out->time = atof(fields[1]);

    float raw_lat = atof(fields[2]);
    char lat_dir = fields[3][0];

    float raw_lon = atof(fields[4]);
    char lon_dir = fields[5][0];

    out->fix = (uint8_t)atoi(fields[6]);
    out->sat = (uint8_t)atoi(fields[7]);
    out->hdop = atof(fields[8]);
    out->alt = atof(fields[9]);
    out->geo = atof(fields[11]);

    // convert coordinates
    out->lat = nmea_to_deg(raw_lat);
    out->lon = nmea_to_deg(raw_lon);

    if (lat_dir == 'S') out->lat = -out->lat;
    if (lon_dir == 'W') out->lon = -out->lon;

    return 0;
}

