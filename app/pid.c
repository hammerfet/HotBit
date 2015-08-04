#include "stm32l0xx_hal.h"
#include "prototypes.h"
#include "config.h"


static float P;
static float I;
static float D;
static float error;
static float result;
static float last_sample;


uint32_t PID_Controller(uint32_t setpoint, uint32_t sample)
{
	error = (float) setpoint - (float) sample;

	P =  error * Kp;

	I += error * Ki;

	D = (last_sample - sample) * Kd;

	last_sample = sample;

	if (I > iMax)		I = iMax;

	else if (I < iMin)	I = iMin;

	result = P + I + D;


	if (result > MAX_SAFE_PWM)
		return MAX_SAFE_PWM;

	else if (result < 0)
		return 0;

	else
		return (uint32_t) result;
}
