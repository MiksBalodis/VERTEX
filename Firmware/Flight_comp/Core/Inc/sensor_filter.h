
#ifndef SENSOR_FILTER_H
#define SENSOR_FILTER_H

#include <stdint.h>
#include <stdbool.h>

typedef struct
{
    float value;
    float alpha;
    bool initialized;
} LowPassFilter_t;

typedef struct
{
    float lat;
    float lon;
    bool initialized;
} GpsFilter_t;

void LowPass_Init(LowPassFilter_t *filter, float alpha);
void LowPass_Reset(LowPassFilter_t *filter);
float LowPass_Update(LowPassFilter_t *filter, float input);

void GpsFilter_Init(GpsFilter_t *filter);
bool GpsFilter_Update(
    GpsFilter_t *filter,
    float input_lat,
    float input_lon,
    float alpha,
    float max_jump_meters,
    float *output_lat,
    float *output_lon
);

#endif
