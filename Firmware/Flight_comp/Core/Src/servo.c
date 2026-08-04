
#include "main.h"

#include "servo.h"

//	Servo innitialization
void Servo_Init(servo_t *servo, TIM_HandleTypeDef *_htim, uint32_t _channel)
{
	servo->htim = _htim;
	servo->channel = _channel;

	/* Default calibration = the global block in servo.h (SG90). Override for a
	   specific servo with Servo_SetLimitsUS(). */
	servo->angle_min = (float)ANGLE_MIN;
	servo->angle_max = (float)ANGLE_MAX;
	servo->count_min = (uint16_t)SERVO_MIN;
	servo->count_max = (uint16_t)SERVO_MAX;

	HAL_TIM_PWM_Start(servo->htim, servo->channel);
}

//	Per-servo travel calibration (angle range <-> pulse range in microseconds)
void Servo_SetLimitsUS(servo_t *servo, float angle_min, float angle_max,
                       uint16_t min_us, uint16_t max_us)
{
	servo->angle_min = angle_min;
	servo->angle_max = angle_max;
	servo->count_min = US_TO_COUNT(min_us);
	servo->count_max = US_TO_COUNT(max_us);
}

//	map help function
long map(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

//	Servo set angle function
void Servo_SetAngle(servo_t *servo, uint16_t angle)
{
	if(angle > (uint16_t)servo->angle_max) angle = (uint16_t)servo->angle_max;
	if(angle < (uint16_t)servo->angle_min) angle = (uint16_t)servo->angle_min;

	  uint16_t tmp = map(angle, (long)servo->angle_min, (long)servo->angle_max,
	                            servo->count_min, servo->count_max);
	  __HAL_TIM_SET_COMPARE(servo->htim, servo->channel, tmp);
}

//	Servo set angle fine function
void Servo_SetAngleFine(servo_t *servo, float angle)
{
	if(angle < servo->angle_min) angle = servo->angle_min;
	if(angle > servo->angle_max) angle = servo->angle_max;

	  /* Float mapping + rounding. The integer map() truncates the fractional
	     degree (its argument is a long), which quantised this "fine" call down
	     to whole-degree steps -- a deadband the TVC loop cannot tune out. */
	  float pulse = (angle - servo->angle_min)
	              * (float)(servo->count_max - servo->count_min)
	              / (servo->angle_max - servo->angle_min)
	              + (float)servo->count_min;
	  __HAL_TIM_SET_COMPARE(servo->htim, servo->channel, (uint16_t)(pulse + 0.5f));
}
