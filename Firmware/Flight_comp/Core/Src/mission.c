#include "mission.h"
#include "imu_fusion.h"
#include "main.h"
#include "buzzer.h"
#include "neopixel.h"
#include "BMP388.h"
#include "fatfs.h"
#include "sensor_filter.h"
#include "gps.h"

#include <string.h>
#include <stdbool.h>

extern Buzzer_Handle hbuzz1;
extern TIM_HandleTypeDef htim6;

extern gngga_t gps;

/* -------------------------------------------------------------------------
   Pressure pre-filter.
   A gentle low-pass smooths ADC-level noise before the barometric altitude
   is computed.  Alpha = 0.10 gives a time constant of ~10 samples.
   ------------------------------------------------------------------------- */
static LowPassFilter_t pressure_filter;

/* -------------------------------------------------------------------------
   Multi-sensor vertical Kalman filter.

   Tuning rationale (BMP388 at 50 Hz, IIR coeff 3, LSM6DSO at 3333 Hz):

   q_alt  = 0.50  – altitude process noise (m²).
             Larger than typical baro-only KFs because we are now driven by
             IMU acceleration.  Accommodates small errors in the rotation
             projection and IMU bias drift.

   q_vel  = 2.00  – velocity process noise (m²/s²).
             Must be large enough that the filter can track rapid velocity
             changes (e.g. motor ignition, max-Q).  With IMU as the process
             input this converges quickly.

   r_baro = 0.50  – BMP388 altitude noise variance (m²).
             BMP388 with IIR-3 and OS-8x at 50 Hz: ~0.15 m RMS in still
             air, rising to ~0.5 m under motor vibration.  0.5 m² (~0.7 m
             sigma) is conservative but safe.

   r_gps  = 25.0  – GPS altitude noise variance (m²).
             Consumer GNSS vertical accuracy: 3-10 m (1-sigma typical).
             5 m sigma = 25 m² is a reasonable conservative value.
             Because r_gps >> r_baro the GPS correction is a gentle nudge,
             preventing baro drift over long flights without disrupting the
             fast baro tracking.
   ------------------------------------------------------------------------- */
static VertKF_t vert_kf;

/*
 * Ground-level MSL pressure, locked in during READY state and kept fixed
 * for the duration of the flight.  Used only for BMP388 AGL conversion.
 */
float ground_pressure;

/*
 * Timestamp of the previous Mission_Update call, used to derive dt.
 */
static uint32_t previous_update_ms = 0;

/* -------------------------------------------------------------------------
   GPS position filter (for lat/lon — unchanged)
   ------------------------------------------------------------------------- */
static GpsFilter_t gps_filter;

static float filtered_gps_lat = 0.0f;
static float filtered_gps_lon = 0.0f;

/* -------------------------------------------------------------------------
   Launch detection
   ------------------------------------------------------------------------- */
#define LAUNCH_ACCEL_THRESHOLD_MG      300.0f
#define LAUNCH_CONFIRMATION_SAMPLES    3

static uint8_t launch_confirm_counter = 0;

/* -------------------------------------------------------------------------
   IMU and integration state
   ------------------------------------------------------------------------- */
IMU_Data_t        imu;
IMU_Integration_t imu_integration;

extern BMP388_HandleTypeDef hbmp388;
extern FATFS FatFs;
extern volatile Quaternion quat;
extern float battery_v;

/* -------------------------------------------------------------------------
   Microsecond timer (overflows tracked by htim6 ISR)
   ------------------------------------------------------------------------- */
volatile uint32_t overflow;

/* -------------------------------------------------------------------------
   Mission state machine
   ------------------------------------------------------------------------- */
typedef enum
{
    MISSION_READY = 0,
    MISSION_ASCENT,
    MISSION_DECENT,        /* kept for binary compatibility */
    MISSION_LANDED,
    MISSION_IDLE,
    MISSION_POST_FAIL
} MissionState_t;

static volatile MissionState_t mission_state = MISSION_READY;

/* -------------------------------------------------------------------------
   Telemetry packets (layout unchanged from previous version)
   ------------------------------------------------------------------------- */
typedef struct __attribute__((packed)) {
    float    altitude;
    int16_t  accel;
    int16_t  speed;
    int16_t  pitch, roll, yaw;
    uint8_t  flight_state;
    uint32_t timestamp;
} TelemetryData_t;

TelemetryData_t telemetry;

/*
 * RAW telemetry layout (54 bytes, unchanged):
 *   Offset  Size  Field
 *    0       4    quat w
 *    4       4    quat x
 *    8       4    quat y
 *   12       4    quat z
 *   16       4    imu rx (mg)
 *   20       4    imu ry (mg)
 *   24       4    imu rz (mg)
 *   28       1    flight_state
 *   29       4    altitude AGL (m)     ← VertKF estimate
 *   33       4    timestamp (µs)
 *   37       4    latitude
 *   41       4    longitude
 *   45       4    vertical_speed (m/s) ← VertKF estimate
 *   49       4    battery_voltage (V)
 *   53       1    GPS sat count
 */
typedef struct __attribute__((packed)) {
    float    w, x, y, z;
    float    rx, ry, rz;
    uint8_t  flight_state;
    float    altitude;
    uint32_t timestamp;
    float    lat;
    float    lon;
    float    vertical_speed;
    float    battery_voltage;
    uint8_t  sat_count;
} RAW_TelemetryData_t;   /* 54 bytes */

RAW_TelemetryData_t raw_telemetry;

bool POST_fault_flags[FAULT_MAX] = {0};

/* -------------------------------------------------------------------------
   Helpers
   ------------------------------------------------------------------------- */
static int16_t clamp_float_to_int16(float value)
{
    if (value >  32767.0f) return  32767;
    if (value < -32768.0f) return -32768;
    return (int16_t)value;
}

/* =========================================================================
   Mission_Init
   ========================================================================= */
void Mission_Init(void)
{
    mission_state = MISSION_READY;

    for (uint8_t fault = 0; fault < FAULT_MAX; fault++)
    {
        if (POST_fault_flags[fault])
        {
            mission_state = MISSION_POST_FAIL;
            break;
        }
    }

    telemetry.flight_state     = mission_state;
    raw_telemetry.flight_state = mission_state;

    /* Pressure low-pass */
    LowPass_Init(&pressure_filter, 0.10f);

    /* Multi-sensor vertical Kalman filter */
    VertKF_Init(&vert_kf,
                /* q_alt  */ 0.50f,
                /* q_vel  */ 2.00f,
                /* r_baro */ 0.50f,
                /* r_gps  */ 25.0f);

    /* GPS lat/lon filter */
    GpsFilter_Init(&gps_filter);
    filtered_gps_lat = 0.0f;
    filtered_gps_lon = 0.0f;

    previous_update_ms    = 0;
    launch_confirm_counter = 0;
    ground_pressure       = 101325.0f;   /* standard atmosphere default */
}

/* =========================================================================
   Mission_Update
   ========================================================================= */
void Mission_Update(void)
{
    uint32_t rprs, rtemp, bmp_time;
    float    prs, temp;

    /* ------------------------------------------------------------------
       Read sensors
       ------------------------------------------------------------------ */
    IMU_Fusion_Update(&imu);

    BMP388_ReadRawPressTempTime(&hbmp388, &rprs, &rtemp, &bmp_time);
    BMP388_CompensateRawPressTemp(&hbmp388, rprs, rtemp, &prs, &temp);

    telemetry.flight_state     = mission_state;
    raw_telemetry.flight_state = mission_state;

    /* ------------------------------------------------------------------
       Compute dt (millisecond HAL tick, wraps every ~49 days — fine)
       ------------------------------------------------------------------ */
    uint32_t now_ms = HAL_GetTick();
    float    dt_s   = 0.020f;     /* safe default (50 Hz) */

    if (previous_update_ms != 0)
    {
        uint32_t elapsed_ms = now_ms - previous_update_ms;
        if (elapsed_ms > 0 && elapsed_ms < 2000)
            dt_s = (float)elapsed_ms * 0.001f;
    }
    previous_update_ms = now_ms;

    /* ------------------------------------------------------------------
       Pressure pre-filter → barometric AGL altitude
       ------------------------------------------------------------------ */
    float prs_filtered   = LowPass_Update(&pressure_filter, prs);

    /* During READY the ground pressure tracks the current filtered value
       so that AGL is 0 at the instant of launch detection.             */
    if (mission_state == MISSION_READY)
    {
        ground_pressure = prs_filtered;
    }

    /* BMP388_FindAltitude: returns AGL metres when called with the
       pressure recorded at ground level.                               */
    float baro_alt_agl = BMP388_FindAltitude(ground_pressure, prs_filtered);

    /* ------------------------------------------------------------------
       Ground GPS altitude accumulation (READY state only)
       ------------------------------------------------------------------ */
    if (mission_state == MISSION_READY)
    {
        /* Accumulate ground GPS MSL altitude only when fix is valid.
           We gate on fix>=1, valid, and a minimum of 4 satellites.
           HDOP is intentionally not gated here: we want to get a
           reference even with moderate geometry.                       */
        if (gps.valid && gps.fix >= 1 && gps.sat >= 4)
        {
            VertKF_UpdateGroundGpsAlt(&vert_kf, gps.alt);
        }
    }

    /* ------------------------------------------------------------------
       Run the Kalman filter
       ------------------------------------------------------------------ */
    if (mission_state == MISSION_READY)
    {
        /* While on the pad, keep re-seeding the filter so it starts
           flight from a settled, zero-altitude, zero-velocity state.  */
        VertKF_Reset(&vert_kf, 0.0f);
    }
    else
    {
        /* In flight: full multi-sensor update.
           Pass a pointer to the gps struct; VertKF_Update will check
           fix quality internally and only fuse when criteria are met. */
        VertKF_Update(&vert_kf,
                      baro_alt_agl,
                      imu.rx,       /* mg, rocket-right */
                      imu.ry,       /* mg, rocket-up    */
                      imu.rz,       /* mg, rocket-front */
                      quat.w,
                      quat.x,
                      quat.y,
                      quat.z,
                      &gps,
                      dt_s);
    }

    float est_altitude = vert_kf.alt;
    float est_vel      = vert_kf.vel;

    /* ------------------------------------------------------------------
       Populate telemetry
       ------------------------------------------------------------------ */
    telemetry.altitude     = est_altitude;
    raw_telemetry.altitude = est_altitude;

    telemetry.speed           = clamp_float_to_int16(est_vel * 100.0f);  /* cm/s */
    raw_telemetry.vertical_speed = est_vel;

    raw_telemetry.battery_voltage = battery_v;
    raw_telemetry.sat_count       = gps.sat;

    /* ------------------------------------------------------------------
       GPS lat/lon filtering (unchanged)
       ------------------------------------------------------------------ */
    float gps_lat, gps_lon;

    if (GpsFilter_Update(&gps_filter,
                         gps.lat, gps.lon,
                         0.25f, 75.0f,
                         &gps_lat, &gps_lon))
    {
        filtered_gps_lat = gps_lat;
        filtered_gps_lon = gps_lon;
    }

    /* ------------------------------------------------------------------
       Mission state machine (unchanged logic)
       ------------------------------------------------------------------ */
    switch (mission_state)
    {
        case MISSION_READY:
            if (imu.ry > LAUNCH_ACCEL_THRESHOLD_MG)
            {
                if (launch_confirm_counter < LAUNCH_CONFIRMATION_SAMPLES)
                    launch_confirm_counter++;
            }
            else
            {
                launch_confirm_counter = 0;
            }

            if (launch_confirm_counter >= LAUNCH_CONFIRMATION_SAMPLES)
            {
                mission_state          = MISSION_ASCENT;
                launch_confirm_counter = 0;

                HAL_TIM_Base_Start_IT(&htim6);

                IMU_Fusion_SetGyro(&imu_integration, 0, 0, 0, Mission_GetTick());

                f_mount(&FatFs, "", 1);
                BUZZ(&hbuzz1, 100);
            }

            ground_pressure = prs_filtered;
            break;

        case MISSION_ASCENT:
            break;

        case MISSION_DECENT:
            break;

        case MISSION_POST_FAIL:
            Mission_SafeMode();
            break;

        case MISSION_LANDED:
            HAL_TIM_Base_Stop(&htim6);
            f_mount(&FatFs, "", 0);
            break;

        default:
            mission_state = MISSION_READY;
            break;
    }
}

/* =========================================================================
   Remaining functions — unchanged from original
   ========================================================================= */

void Mission_SafeMode(void)
{
    while (1)
    {
        for (uint8_t fault = 0; fault < FAULT_MAX; fault++)
        {
            if (POST_fault_flags[fault])
            {
                for (uint8_t i = 0; i <= fault; i++)
                {
                    LED_Set_Color(64, 0, 0);
                    HAL_Delay(500);
                    LED_Set_Color(0, 0, 0);
                    HAL_Delay(500);
                }
                HAL_Delay(1500);
            }
        }
    }
}

void Mission_IncTick(void)
{
    overflow++;
}

uint32_t Mission_GetTick(void)
{
    uint32_t high;
    uint16_t low;

    do {
        high = overflow;
        low  = __HAL_TIM_GET_COUNTER(&htim6);
    } while (high != overflow);

    return (high << 16) | low;
}

void Mission_BuildTelemetryPacket(uint8_t *buf)
{
    telemetry.timestamp = Mission_GetTick();
    telemetry.pitch     = imu_integration.pitch * 10.0f;
    telemetry.roll      = imu_integration.roll  * 10.0f;
    telemetry.yaw       = imu_integration.yaw   * 10.0f;

    memcpy(buf, &telemetry, sizeof(TelemetryData_t));
}

void Mission_BuildRAWTelemetryPacket(uint8_t *buf)
{
    raw_telemetry.timestamp = Mission_GetTick();
    raw_telemetry.w         = quat.w;
    raw_telemetry.x         = quat.x;
    raw_telemetry.y         = quat.y;
    raw_telemetry.z         = quat.z;

    raw_telemetry.rx = imu.rx;
    raw_telemetry.ry = imu.ry;
    raw_telemetry.rz = imu.rz;

    raw_telemetry.flight_state    = mission_state;
    raw_telemetry.lat             = filtered_gps_lat;
    raw_telemetry.lon             = filtered_gps_lon;
    raw_telemetry.battery_voltage = battery_v;
    raw_telemetry.sat_count       = gps.sat;

    /* altitude and vertical_speed are kept current by Mission_Update */

    memcpy(buf, &raw_telemetry, sizeof(RAW_TelemetryData_t));
}

void Mission_IMU_DRDY(void)
{
    if (mission_state == MISSION_ASCENT)
        IMU_Fusion_IntegrateGyro(&imu_integration, Mission_GetTick());
}
