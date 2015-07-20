#include "stm32l0xx_hal.h"
#include "prototypes.h"
#include "config.h"

static float D_temp = 0;
static float I_temp = 0;


static float error;
static float P;
static float I;
static float D;
static float output;

uint32_t Controller(uint32_t setpoint, uint32_t adcvalue)
{

	error = setpoint - adcvalue;

	P = Kp * error;

	I_temp += error;

	if (I_temp > iMax) I_temp = iMax;
	if (I_temp < iMin) I_temp = iMin;

	I = Ki * I_temp;

	D = Kd * (D_temp - error);
	D_temp = error;

	output = output - (P + I + D);

	return (uint32_t) output;
}
