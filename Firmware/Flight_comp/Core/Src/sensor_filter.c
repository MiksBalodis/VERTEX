
#include "sensor_filter.h"
#include <math.h>

void LowPass_Init(LowPassFilter_t *filter, float alpha)
{
    if (filter == 0) return;

    filter->value = 0.0f;
    filter->alpha = alpha;
    filter->initialized = false;
}

void LowPass_Reset(LowPassFilter_t *filter)
{
    if (filter == 0) return;

    filter->value = 0.0f;
    filter->initialized = false;
}

float LowPass_Update(LowPassFilter_t *filter, float input)
{
    if (filter == 0) return input;

    if (!filter->initialized)
    {
        filter->value = input;
        filter->initialized = true;
        return filter->value;
    }

    filter->value = filter->value + filter->alpha * (input - filter->value);
    return filter->value;
}

void GpsFilter_Init(GpsFilter_t *filter)
{
    if (filter == 0) return;

    filter->lat = 0.0f;
    filter->lon = 0.0f;
    filter->initialized = false;
}

static bool gps_valid(float lat, float lon)
{
    return (
        isfinite(lat) &&
        isfinite(lon) &&
        lat >= -90.0f &&
        lat <= 90.0f &&
        lon >= -180.0f &&
        lon <= 180.0f &&
        !(lat == 0.0f && lon == 0.0f)
    );
}

static float gps_distance_m(float lat1, float lon1, float lat2, float lon2)
{
    const float meters_per_deg_lat = 111320.0f;
    const float meters_per_deg_lon = 111320.0f * cosf(lat1 * 3.14159265f / 180.0f);

    float dx = (lon2 - lon1) * meters_per_deg_lon;
    float dy = (lat2 - lat1) * meters_per_deg_lat;

    return sqrtf(dx * dx + dy * dy);
}

bool GpsFilter_Update(
    GpsFilter_t *filter,
    float input_lat,
    float input_lon,
    float alpha,
    float max_jump_meters,
    float *output_lat,
    float *output_lon
)
{
    if (filter == 0 || output_lat == 0 || output_lon == 0) return false;

    if (!gps_valid(input_lat, input_lon)) return false;

    if (!filter->initialized)
    {
        filter->lat = input_lat;
        filter->lon = input_lon;
        filter->initialized = true;

        *output_lat = filter->lat;
        *output_lon = filter->lon;

        return true;
    }

    float jump = gps_distance_m(filter->lat, filter->lon, input_lat, input_lon);

    if (jump > max_jump_meters)
    {
        *output_lat = filter->lat;
        *output_lon = filter->lon;
        return false;
    }

    filter->lat = filter->lat + alpha * (input_lat - filter->lat);
    filter->lon = filter->lon + alpha * (input_lon - filter->lon);

    *output_lat = filter->lat;
    *output_lon = filter->lon;

    return true;
}
