#include "mission.h"
#include "imu_fusion.h"
#include "main.h"
#include "buzzer.h"
#include "neopixel.h"
#include "BMP388.h"
#include "fatfs.h"
#include "sensor_filter.h"
#include "servo.h"
#include "tvc_servo.h"
#include "gps.h"

#include <string.h>
#include <stdbool.h>

extern Buzzer_Handle hbuzz1;
extern TIM_HandleTypeDef htim6;

extern gngga_t gps;

static LowPassFilter_t pressure_filter;

static VertKF_t vert_kf;

float ground_pressure;

static uint32_t previous_update_ms = 0;

static GpsFilter_t gps_filter;

static float filtered_gps_lat = 0.0f;
static float filtered_gps_lon = 0.0f;

/* ---------------------------------------------------------------------------
   Launch detection.

   The threshold is now NET of gravity. The old code compared imu.ry (which
   reads ~1 g at rest) against 900 mg, so it self-triggered on the bench.
   --------------------------------------------------------------------------- */
#define LAUNCH_NET_ACCEL_THRESHOLD_MG  2000.0f   /* 2 g net -- tune to your motor */
#define LAUNCH_CONFIRMATION_SAMPLES    3

/* ---------------------------------------------------------------------------
   Control loop.
   --------------------------------------------------------------------------- */
#define CTRL_RATE_HZ   400.0f
#define CTRL_DT_S      (1.0f / CTRL_RATE_HZ)

/* ---------------------------------------------------------------------------
   TVC gains.

   !!! THESE ARE PLACEHOLDERS. Derive them from a sim (mass, lateral moment of
   !!! inertia, thrust curve, gimbal moment arm, servo slew rate) before you fly.
   !!! Kd USED TO BE ZERO, which meant no damping at all on an inverted-pendulum
   !!! plant -- that alone guaranteed divergence.
   --------------------------------------------------------------------------- */
#define TVC_KP          0.5f    /* deg of nozzle per deg of tilt   */
#define TVC_KI          0.0f    /* leave at zero until P+D is tuned */
#define TVC_KD_DEG_DPS  0.05f   /* deg of nozzle per deg/s of rate  */

/* Cut TVC at burnout: past this point the nozzle has no thrust to vector, so
   the servos would just chase a diverging angle. Set to your motor's burn time. */
#define MOTOR_BURN_TIME_S  2.0f

static uint8_t launch_confirm_counter = 0;

IMU_Data_t        imu;
IMU_Integration_t imu_integration;

TVC_t tvc;

extern BMP388_HandleTypeDef hbmp388;
extern FATFS FatFs;
FIL telemetry_file;
extern uint32_t flight_rand_id;
extern volatile Quaternion quat;
extern float battery_v;

volatile uint32_t overflow;

extern servo_t hservo1, hservo2, hservo3;

float max_altitude = 0.0f;

uint32_t last_telemetry_tst = 0;

uint32_t led_tmr = 0;
uint32_t idle_tmr = 0;
uint32_t freefall_tmr = 0;

bool is_flash_ready = false;

/* Set once the 400 Hz control timer owns SPI2. Before this, the main loop reads
   the IMU directly (safe: no ISR is touching the bus yet). After it, the main
   loop MUST NOT touch the IMU. */
volatile bool ctrl_loop_active = false;

static uint32_t launch_tick_us = 0;
static float    g_ref_mg       = 1000.0f;
static bool     tvc_armed      = false;

typedef enum
{
    MISSION_READY = 0,
    MISSION_ASCENT,
    MISSION_DECENT,
    MISSION_LANDED,
    MISSION_IDLE,
    MISSION_POST_FAIL
} MissionState_t;

static volatile MissionState_t mission_state = MISSION_READY;

typedef struct __attribute__((packed)) {
    float    altitude;
    int16_t  accel;
    int16_t  speed;
    int16_t  pitch, roll, yaw;
    uint8_t  flight_state;
    uint32_t timestamp;
} TelemetryData_t;

TelemetryData_t telemetry;

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
} RAW_TelemetryData_t;

RAW_TelemetryData_t raw_telemetry;

bool POST_fault_flags[FAULT_MAX] = {0};

static int16_t clamp_float_to_int16(float value)
{
    if (value >  32767.0f) return  32767;
    if (value < -32768.0f) return -32768;
    return (int16_t)value;
}

void Mission_Init(void)
{
    mission_state = MISSION_IDLE;

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

    LowPass_Init(&pressure_filter, 0.10f);

    VertKF_Init(&vert_kf,
                             0.50f,
                             2.00f,
                             0.50f,
                             25.0f);

    GpsFilter_Init(&gps_filter);
    filtered_gps_lat = 0.0f;
    filtered_gps_lon = 0.0f;

    previous_update_ms    = 0;
    launch_confirm_counter = 0;
    ground_pressure       = 101325.0f;

    TVC_Init(&tvc,
              &hservo1,
              &hservo2,
              Servo1_Home,
              Servo2_Home,
              TVC_KP,
              TVC_KI,
              TVC_KD_DEG_DPS * 1e-3f,   /* pid_step takes the rate in mdps */
              TVC_MAX_DEFLECTION_DEG);

    TVC_Disable(&tvc);
    tvc_armed = false;

    /* Free-running microsecond timebase from power-on. It used to only start at
       launch, which meant Mission_GetTick() was garbage before that. */
    overflow = 0;
    __HAL_TIM_SET_COUNTER(&htim6, 0);
    HAL_TIM_Base_Start_IT(&htim6);

    idle_tmr = HAL_GetTick();
}

void Mission_Update(void)
{
    uint32_t rprs, rtemp, bmp_time;
    float    prs, temp;

    /* SPI2 has exactly one owner. Before the control loop starts, that's us.
       After it starts, it's Mission_ControlTick() and we only take snapshots. */
    if (!ctrl_loop_active)
        IMU_Fusion_Update(&imu, &imu_integration, Mission_GetTick());

    IMU_Data_t imu_s;
    Quaternion q_s;
    __disable_irq();
    imu_s = imu;
    q_s.w = quat.w; q_s.x = quat.x; q_s.y = quat.y; q_s.z = quat.z;
    __enable_irq();

    BMP388_ReadRawPressTempTime(&hbmp388, &rprs, &rtemp, &bmp_time);
    BMP388_CompensateRawPressTemp(&hbmp388, rprs, rtemp, &prs, &temp);

    telemetry.flight_state     = mission_state;
    raw_telemetry.flight_state = mission_state;

    /* HAL_GetTick() is 1 ms; when the loop ran faster than 1 kHz elapsed_ms was
       0, the guard failed, and dt silently fell back to a hard-coded 20 ms.
       Use the microsecond timer. */
    uint32_t now_us = Mission_GetTick();
    float    dt_s   = 0.020f;

    if (previous_update_ms != 0)
    {
        uint32_t elapsed_us = now_us - previous_update_ms;
        if (elapsed_us > 0 && elapsed_us < 2000000)
            dt_s = (float)elapsed_us * 1e-6f;
    }
    previous_update_ms = now_us;

    float prs_filtered   = LowPass_Update(&pressure_filter, prs);

    if (mission_state == MISSION_READY)
    {
        ground_pressure = prs_filtered;
    }

    float baro_alt_agl = BMP388_FindAltitude(ground_pressure, prs_filtered);

    if (mission_state == MISSION_READY)
    {

        if (gps.valid && gps.fix >= 1 && gps.sat >= 4)
        {
            VertKF_UpdateGroundGpsAlt(&vert_kf, gps.alt);
        }
    }

    if (mission_state == MISSION_READY)
    {

        VertKF_Reset(&vert_kf, 0.0f);
    }
    else
    {

        VertKF_Update(&vert_kf,
                      baro_alt_agl,
                      imu_s.rx,
                      imu_s.ry,
                      imu_s.rz,
                      q_s.w,
                      q_s.x,
                      q_s.y,
                      q_s.z,
                      &gps,
                      dt_s);
    }

    float est_altitude = vert_kf.alt;
    float est_vel      = vert_kf.vel;

    telemetry.altitude     = est_altitude;
    raw_telemetry.altitude = est_altitude;

    telemetry.speed           = clamp_float_to_int16(est_vel * 100.0f);
    raw_telemetry.vertical_speed = est_vel;

    raw_telemetry.battery_voltage = battery_v;
    raw_telemetry.sat_count       = gps.sat;

    float gps_lat, gps_lon;

    if (GpsFilter_Update(&gps_filter,
                         gps.lat, gps.lon,
                         0.25f, 75.0f,
                         &gps_lat, &gps_lon))
    {
        filtered_gps_lat = gps_lat;
        filtered_gps_lon = gps_lon;
    }

    switch (mission_state)
    {
        case MISSION_IDLE:
            if(HAL_GetTick()-idle_tmr > IDLE_TIMEOUT_S*1000){
                mission_state = MISSION_READY;

                /* Control loop is still stopped here, so the main loop has SPI2
                   to itself. Both of these block. */
                IMU_Fusion_CalibrateGyro(200);
                g_ref_mg = IMU_Fusion_CaptureGravityRef(100);
                VertKF_SetGravityRef(&vert_kf, g_ref_mg);

                /* From here on, Mission_ControlTick() owns the IMU bus. */
                Mission_StartControlLoop();

                FRESULT res;
                char filename[14];
                res = f_mount(&FatFs, "", 1);
                if (res != FR_OK){
                    LED_Set_Color(64, 0, 0);
                    break;
                }

                snprintf(filename, sizeof(filename), "LOG_%04lX.CSV", flight_rand_id & 0xFFFF);
                res = f_open(&telemetry_file, filename, FA_CREATE_ALWAYS | FA_WRITE);
                if(res != FR_OK){
                    LED_Set_Color(64, 0, 0);
                    break;
                }

                const char *header =
                    "w,x,y,z,rx,ry,rz,flight_state,altitude,timestamp,"
                    "lat,lon,vertical_speed,battery_voltage,sat_count\r\n";

                UINT bw;
                res = f_write(&telemetry_file, header, strlen(header), &bw);
                if(res != FR_OK){
                    LED_Set_Color(64, 0, 0);
                    break;
                }
                // f_close(&telemetry_file);

                is_flash_ready = true;
            }
            break;
        case MISSION_READY:
            /* NET of gravity. imu.ry reads ~+1 g sitting on the pad. */
            if ((imu_s.ry - g_ref_mg) > LAUNCH_NET_ACCEL_THRESHOLD_MG)
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

                launch_tick_us = Mission_GetTick();

                /* Zero the euler integrals AND seed the quaternion from the
                   measured gravity direction, instead of asserting identity and
                   hoping the rail was perfectly vertical. */
                IMU_Fusion_ResetAttitude(&imu_integration, launch_tick_us);

                TVC_Reset(&tvc);
                TVC_Enable(&tvc);
                tvc_armed = true;

                BUZZ(&hbuzz1, 100);
            }

            ground_pressure = prs_filtered;
            break;

        case MISSION_ASCENT:
            /* TVC no longer runs here -- it runs in Mission_ControlTick() at a
               fixed 400 Hz. The main loop is not a control loop: it shares time
               with I2C, GPS, LoRa and blocking SD writes. */

            /* Burnout: no thrust means no control authority. Park the gimbal. */
            if (tvc_armed &&
                (Mission_GetTick() - launch_tick_us) > (uint32_t)(MOTOR_BURN_TIME_S * 1000000.0f))
            {
                TVC_Disable(&tvc);
                tvc_armed = false;
            }

            // If PRS is not valid, use timeout for decent
            if((Mission_GetTick() - launch_tick_us) > (MAX_ASCENT_TIME_S * 1000000)){
                mission_state = MISSION_DECENT;
                freefall_tmr = Mission_GetTick();
            }

            if(est_altitude > max_altitude){
                max_altitude = est_altitude;
            }

            if(max_altitude - est_altitude > DECENT_DETECTION_ALTITUDE_M){
                mission_state = MISSION_DECENT;
                freefall_tmr = Mission_GetTick();
            }

            if(Mission_GetTick() - last_telemetry_tst > 100000 && is_flash_ready){
                Mission_SaveTelemetry();
                last_telemetry_tst = Mission_GetTick();
            }
            break;

        case MISSION_DECENT:
            TVC_Disable(&tvc);
            tvc_armed = false;

            if(est_altitude < LANDING_DETECTION_ALTITUDE_M){
                mission_state = MISSION_LANDED;
            }

            if((Mission_GetTick() - launch_tick_us) > (MAX_DECENT_TIME_S * 1000000)){
                mission_state = MISSION_LANDED;
            }

            if(Mission_GetTick() - freefall_tmr > (FEEFALL_TIME_S * 1000000)){
                Servo_SetAngle(&hservo3, Parachute_Servo_Deploy);
            }
            break;

        case MISSION_POST_FAIL:
            Mission_SafeMode();
            break;

        case MISSION_LANDED:
            Mission_StopControlLoop();
            TVC_Disable(&tvc);
            f_close(&telemetry_file);
            f_mount(&FatFs, "", 0);
            LED_Set_Color(0, 0, 64);
            break;

        default:
            mission_state = MISSION_READY;
            break;
    }
}

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

    memcpy(buf, &raw_telemetry, sizeof(RAW_TelemetryData_t));
}

/* ---------------------------------------------------------------------------
   Fixed-rate control loop. Runs from the TIM3 ISR at CTRL_RATE_HZ.

   This is the ONLY place SPI2 is touched once ctrl_loop_active is set. The old
   design read the IMU from a 3333 Hz DRDY EXTI handler AND from the main loop
   simultaneously, with no mutual exclusion -- that's what corrupted the accel
   (665 mg of "gravity") and drifted the quaternion 82 deg in 10 stationary
   seconds.
   --------------------------------------------------------------------------- */
void Mission_ControlTick(void)
{
    if (!ctrl_loop_active) return;

    IMU_Fusion_Update(&imu, &imu_integration, Mission_GetTick());

    if (mission_state == MISSION_ASCENT && tvc_armed)
    {
        TVC_Update(&tvc,
                   imu_integration.pitch,
                   imu.gx,
                   imu_integration.yaw,
                   imu.gz,
                   imu_integration.roll,
                   CTRL_DT_S);
    }
}

void Mission_StartControlLoop(void)
{
    imu_integration.time = Mission_GetTick();
    ctrl_loop_active = true;
    Ctrl_Timer_Start();
}

void Mission_StopControlLoop(void)
{
    ctrl_loop_active = false;
    Ctrl_Timer_Stop();
}

float Mission_GetGravityRefMg(void)
{
    return g_ref_mg;
}

void Mission_SaveTelemetry(void){
    UINT bw;
    static char line[256];
    uint8_t tmp[54];
    Mission_BuildRAWTelemetryPacket(tmp);

    int32_t w  = (int32_t)(raw_telemetry.w  * 1000000.0f);
    int32_t x  = (int32_t)(raw_telemetry.x  * 1000000.0f);
    int32_t y  = (int32_t)(raw_telemetry.y  * 1000000.0f);
    int32_t z  = (int32_t)(raw_telemetry.z  * 1000000.0f);

    /* mg * 1000 (i.e. micro-g). The old 1e6 scale overflowed int32 above ~2.1 g,
       so every real boost sample would have wrapped to garbage.
       Post-flight: divide by 1000 to get mg. */
    int32_t rx = (int32_t)(raw_telemetry.rx * 1000.0f);
    int32_t ry = (int32_t)(raw_telemetry.ry * 1000.0f);
    int32_t rz = (int32_t)(raw_telemetry.rz * 1000.0f);

    int32_t altitude = (int32_t)(raw_telemetry.altitude * 100.0f);

    int32_t lat = (int32_t)(raw_telemetry.lat * 10000000.0f);
    int32_t lon = (int32_t)(raw_telemetry.lon * 10000000.0f);

    int32_t vertical_speed = (int32_t)(raw_telemetry.vertical_speed * 100.0f);
    int32_t battery_voltage = (int32_t)(raw_telemetry.battery_voltage * 100.0f);


    int len = snprintf(
        line, sizeof(line),
        "%ld,%ld,%ld,%ld,"
        "%ld,%ld,%ld,"
        "%u,"
        "%ld,"
        "%lu,"
        "%ld,%ld,"
        "%ld,"
        "%ld,"
        "%u\r\n",

        w,
        x,
        y,
        z,
        rx,
        ry,
        rz,
        raw_telemetry.flight_state,
        altitude,
        (unsigned long)raw_telemetry.timestamp,
        lat,
        lon,
        vertical_speed,
        battery_voltage,
        raw_telemetry.sat_count
    );

    if(len < 0){return;}
    FRESULT res = f_write(&telemetry_file, line, len, &bw);
    if(res != FR_OK){
        LED_Set_Color(64, 0, 0);
        is_flash_ready = false;
    }
}
