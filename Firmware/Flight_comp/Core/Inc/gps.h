#pragma once
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

#define GPS_OK          0
#define GPS_PARSE_ERR   1

typedef struct {
    float time;
    float lat;
    float lon;
    uint8_t fix;
    uint8_t sat;
    float hdop;
    float alt;
    float geo;
    uint8_t crc;
    bool valid;
} gngga_t;

uint8_t GPS_Process_Data(uint8_t *data, uint16_t start);