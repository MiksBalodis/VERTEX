/*
 * servo.h
 *
 *		The MIT License.
 *  Created on: 01.02.2019
 *		Author: Mateusz Salamon
 *		www.msalamon.pl
 *		mateusz@msalamon.pl
 *
 *      Website: https://msalamon.pl/nigdy-wiecej-multipleksowania-na-gpio!-max7219-w-akcji-cz-3/
 *      GitHub:  https://github.com/lamik/MAX7219_matrix_STM32_HAL
 */

#ifndef SERVO_H_
#define SERVO_H_

#include "main.h"

//
//	Servo
//
#define SERVO_SG90

//
//	Defines
//

// For Turnigy TG9e
#ifdef	TURNIGY_TG9E
#define SERVO_MIN 550
#define SERVO_MAX 2430
#define ANGLE_MIN 0
#define ANGLE_MAX 180
#endif

#ifdef	SERVO_SG90
#define SERVO_MIN 61
#define SERVO_MAX 304
#define ANGLE_MIN 0
#define ANGLE_MAX 180
#endif

/* -------------------------------------------------------------------------
   PWM timing helper — lets each servo's travel be set in MICROSECONDS.

   The servo timer (htim5) input clock is the APB1 timer clock. With
   SYSCLK = 144 MHz (HSE 8 MHz, PLLM4/PLLN144/PLLP2) and APB1 divider /4,
   that clock is 72 MHz. One timer tick = 1 / (72e6 / (PSC+1)).

   SERVO_TIMER_PSC MUST match htim5.Init.Prescaler in main.c (591). If you
   change the servo frame rate later (in MX_TIM5_Init change only .Period,
   keep .Prescaler = 591), this stays correct and no value below changes.
   ------------------------------------------------------------------------- */
#define SERVO_TIMER_CLK_HZ   72000000UL
#define SERVO_TIMER_PSC      591UL          /* == htim5.Init.Prescaler */
#define SERVO_TICK_HZ        (SERVO_TIMER_CLK_HZ / (SERVO_TIMER_PSC + 1UL))
/* microseconds -> timer compare counts (rounded) */
#define US_TO_COUNT(us)      ((uint16_t)(((uint32_t)(us) * SERVO_TICK_HZ + 500000UL) / 1000000UL))

/* KST X10 Mini Pro-A (datasheet 2025-04): 900..2100 us = -60..+60 deg,
   1500 us = center, 7.4 V HV (works down to 4.8 V). Applied per-servo in
   Servo_Init_All() via Servo_SetLimitsUS() — NOT globally — so the parachute
   servo keeps its own calibration. */

typedef struct {
    TIM_HandleTypeDef* htim;
    uint32_t channel;

    /* Per-servo calibration. Defaults set from the global block above in
       Servo_Init(); override per servo with Servo_SetLimitsUS(). This is what
       lets the two TVC servos be KST while the parachute keeps its own map. */
    float    angle_min;   /* software angle at count_min [deg] */
    float    angle_max;   /* software angle at count_max [deg] */
    uint16_t count_min;   /* timer compare at angle_min */
    uint16_t count_max;   /* timer compare at angle_max */
} servo_t;

void Servo_Init(servo_t *servo, TIM_HandleTypeDef *_htim, uint32_t _channel);
/* Set this servo's travel: software angle_min..angle_max maps to min_us..max_us
   of pulse. Counts are derived from the timer clock via US_TO_COUNT. */
void Servo_SetLimitsUS(servo_t *servo, float angle_min, float angle_max,
                       uint16_t min_us, uint16_t max_us);
void Servo_SetAngle(servo_t *servo, uint16_t angle);
void Servo_SetAngleFine(servo_t *servo, float angle);

#endif /* SERVO_H_ */
