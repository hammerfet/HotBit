#include "stm32l0xx_hal.h"
#include "prototypes.h"
#include "config.h"

// Macro for determining if the current state is a TEMP state or not
#define _IS_TEMP_STATE (CurrentState >= TEMP1 && CurrentState <= TEMP9)

// Macro for setting the PWM output duty cycle, the blockPWM flag is used to force PWM duty to 0
uint8_t blockPWM = 0;
#define _SET_PWM(x) (__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_3, (blockPWM ? 0 : x)))

// Macro for resetting the Auto power off feature. The AUTOPOWEROFFTIME define is used to determine time
uint32_t autoPowerOffTimerTick = 0;
#define _RESET_AUTO_POWER_OFF_TIMER (autoPowerOffTimerTick = 0)

// Typedef handles for the STM32 HAL drivers
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim2;
ADC_HandleTypeDef hadc;

// Variables used in this file
state volatile CurrentState = TEMP1;
state LastTempState = TEMP1;
uint8_t readyToRunADC = 0;

// Control variables
uint32_t tipTempRawValue = 0;
uint32_t pwmDriveValue = 0;

uint32_t setPoint[9] = {
	500,
	800,
	1000,
	1500,
	2000,
	2500,
	3000,
	3500,
	4000
};

void stateMachine(void)
{
	switch (CurrentState)
	{
		case SLEEP:
			clearLEDs();
			_SET_PWM(0);
			//HAL_PWR_EnterSTANDBYMode();
			break;

		case IDLE:
			setLED(LED_1);
			setLED(LED_9);
			clearLEDs();
			_SET_PWM(0);
			break;

		case TEMP1:
			clearLEDs();
			LastTempState = TEMP1;
			setLED(LED_1);			
			basicController(setPoint[0]);
			break;

		case TEMP2:
			clearLEDs();
			LastTempState = TEMP2;
			setLED(LED_2);
			basicController(setPoint[1]);
			break;

		case TEMP3:
			clearLEDs();
			LastTempState = TEMP3;
			setLED(LED_3);
			basicController(setPoint[2]);
			break;

		case TEMP4:
			clearLEDs();
			LastTempState = TEMP4;
			setLED(LED_4);
			basicController(setPoint[3]);
			break;

		case TEMP5:
			clearLEDs();
			LastTempState = TEMP5;
			setLED(LED_5);
			basicController(setPoint[4]);
			break;

		case TEMP6:
			clearLEDs();
			LastTempState = TEMP6;
			setLED(LED_6);
			basicController(setPoint[5]);
			break;

		case TEMP7:
			clearLEDs();
			LastTempState = TEMP7;
			setLED(LED_7);
			basicController(setPoint[6]);
			break;

		case TEMP8:
			clearLEDs();
			LastTempState = TEMP8;
			setLED(LED_8);
			basicController(setPoint[7]);
			break;

		case TEMP9:
			clearLEDs();
			LastTempState = TEMP9;
			setLED(LED_9);
			basicController(setPoint[8]);
			break;

		default:
			CurrentState = IDLE;
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (CurrentState == IDLE || CurrentState == SLEEP)
		CurrentState = LastTempState;

	else if (HAL_GPIO_ReadPin(TEMPDN_PORT, TEMPDN_PIN) == GPIO_PIN_RESET && _IS_TEMP_STATE && CurrentState != TEMP1)
		CurrentState--;

	else if (HAL_GPIO_ReadPin(TEMPUP_PORT, TEMPUP_PIN) == GPIO_PIN_RESET && _IS_TEMP_STATE && CurrentState != TEMP9)
		CurrentState++;

	// Reset timeout
	_RESET_AUTO_POWER_OFF_TIMER;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM6)
	{
	// ------- Tip temp ADC conversion poll section 

		// We need to stop the PWM and create a delay 
		// before taking an ADC sample due to transients
		// A sample is taken every 10 TIM6 periods
		
		// We first inc this ADC counter
		readyToRunADC++;

		// If the counter is 9, we kill the PWM to prepare for a conversion
		if (readyToRunADC == 9)
			blockPWM = 1;

		// if counter = 10, do a measurement, clear timer and unblock PWM
		else if (readyToRunADC == 10)
		{
			startTipTempMeasurement();
			readyToRunADC = 0;
			blockPWM = 0;
		}
		
		
	// ------- Auto power off timer section 

		// Tick this variable and compare it to the AUTPPOWEROFFTIME defined in config.h
		autoPowerOffTimerTick++;

		// If equal, we start drop to IDLE if running or SLEEP if idle
		if (autoPowerOffTimerTick == AUTOPOWEROFFTIME)
		{
			if (_IS_TEMP_STATE)
				CurrentState = IDLE;

			else if (CurrentState == IDLE)
				CurrentState = SLEEP;
			
			autoPowerOffTimerTick = 0;
		}

	}
}

void startTipTempMeasurement(void)
{
	if (HAL_ADC_GetState(&hadc) == HAL_ADC_STATE_READY)
	{
		HAL_ADC_Start(&hadc);
		HAL_ADC_PollForConversion(&hadc, 100);

		if (HAL_ADC_GetState(&hadc) == HAL_ADC_STATE_EOC)
			tipTempRawValue = HAL_ADC_GetValue(&hadc);

		HAL_ADC_Stop(&hadc);
	}
}

void basicController(uint32_t setpoint)
{
	if (tipTempRawValue < setpoint)
	{
		pwmDriveValue++;
	}

	if (tipTempRawValue > setpoint)
	{
		if (pwmDriveValue > 0)
			pwmDriveValue--;
	}

	if (pwmDriveValue > 18000)
		pwmDriveValue = 18000;

	_SET_PWM(pwmDriveValue);
}