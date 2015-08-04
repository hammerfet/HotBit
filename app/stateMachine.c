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
static state volatile CurrentState = SLEEP;
static state LastTempState = TEMP1;
static uint8_t readyToRunADC = 0;
static uint32_t ledBlinkCounter = 0;

// Control variables
static uint32_t volatile tipTempRawValue = 0;
static uint32_t volatile pwmDriveValue = 0;



void stateMachine(void)
{
	switch (CurrentState)
	{
		case SLEEP:
			clearLEDs();
			_SET_PWM(0);
			stopIMU();
			//HAL_PWR_EnterSTOPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
			HAL_Delay(5); // Give the clocks a chance to start before enabling IMU
			startIMU();
			break;

		case IDLE:
			clearLEDs();

			if(ledBlinkCounter < LED_BLINK_TIME/2)
				setLED(LastTempState - 2); // This is a hacky way of getting the last LED lit

			if(ledBlinkCounter > LED_BLINK_TIME)
				ledBlinkCounter = 0;

			else
				ledBlinkCounter++;

			_SET_PWM(0);

			if (checkForMovement())
			{
				CurrentState = LastTempState;
				ledBlinkCounter = 0; // Reset the blink counter so we get a smooth blink next time we hit idle
			}

			checkForMovement(); // This is just for double buffering
			
			break;

		case TEMP1:
			clearLEDs();
			LastTempState = TEMP1;
			setLED(LED_1);
			pwmDriveValue = PID_Controller(SET_POINT_1, tipTempRawValue);
			_SET_PWM(pwmDriveValue);
			if (checkForMovement()) _RESET_AUTO_POWER_OFF_TIMER;
			break;

		case TEMP2:
			clearLEDs();
			LastTempState = TEMP2;
			setLED(LED_2);
			pwmDriveValue = PID_Controller(SET_POINT_2, tipTempRawValue);
			_SET_PWM(pwmDriveValue);
			if (checkForMovement()) _RESET_AUTO_POWER_OFF_TIMER;
			break;

		case TEMP3:
			clearLEDs();
			LastTempState = TEMP3;
			setLED(LED_3);
			pwmDriveValue = PID_Controller(SET_POINT_3, tipTempRawValue);
			_SET_PWM(pwmDriveValue);
			if (checkForMovement()) _RESET_AUTO_POWER_OFF_TIMER;
			break;

		case TEMP4:
			clearLEDs();
			LastTempState = TEMP4;
			setLED(LED_4);
			pwmDriveValue = PID_Controller(SET_POINT_4, tipTempRawValue);
			_SET_PWM(pwmDriveValue);
			if (checkForMovement()) _RESET_AUTO_POWER_OFF_TIMER;
			break;

		case TEMP5:
			clearLEDs();
			LastTempState = TEMP5;
			setLED(LED_5);
			pwmDriveValue = PID_Controller(SET_POINT_5, tipTempRawValue);
			_SET_PWM(pwmDriveValue);
			if (checkForMovement()) _RESET_AUTO_POWER_OFF_TIMER;
			break;

		case TEMP6:
			clearLEDs();
			LastTempState = TEMP6;
			setLED(LED_6);
			pwmDriveValue = PID_Controller(SET_POINT_6, tipTempRawValue);
			_SET_PWM(pwmDriveValue);
			if (checkForMovement()) _RESET_AUTO_POWER_OFF_TIMER;
			break;

		case TEMP7:
			clearLEDs();
			LastTempState = TEMP7;
			setLED(LED_7);
			pwmDriveValue = PID_Controller(SET_POINT_7, tipTempRawValue);
			_SET_PWM(pwmDriveValue);
			if (checkForMovement()) _RESET_AUTO_POWER_OFF_TIMER;
			break;

		case TEMP8:
			clearLEDs();
			LastTempState = TEMP8;
			setLED(LED_8);
			pwmDriveValue = PID_Controller(SET_POINT_8, tipTempRawValue);
			_SET_PWM(pwmDriveValue);
			if (checkForMovement()) _RESET_AUTO_POWER_OFF_TIMER;
			break;

		case TEMP9:
			clearLEDs();
			LastTempState = TEMP9;
			setLED(LED_9);
			pwmDriveValue = PID_Controller(SET_POINT_9, tipTempRawValue);
			_SET_PWM(pwmDriveValue);
			if (checkForMovement()) _RESET_AUTO_POWER_OFF_TIMER;
			break;

		default:
			CurrentState = IDLE;
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (CurrentState == SLEEP && (HAL_GPIO_ReadPin(TEMPUP_PORT, TEMPUP_PIN) == GPIO_PIN_SET))
		CurrentState = LastTempState; // This is a wakeup from stop mode call

	else if (CurrentState == IDLE)
		CurrentState = LastTempState;

	else if (HAL_GPIO_ReadPin(TEMPDN_PORT, TEMPDN_PIN) == GPIO_PIN_SET && _IS_TEMP_STATE && CurrentState != TEMP1)
		CurrentState--;

	else if (HAL_GPIO_ReadPin(TEMPUP_PORT, TEMPUP_PIN) == GPIO_PIN_SET && _IS_TEMP_STATE && CurrentState != TEMP9)
		CurrentState++;

	int i;
	for(i = 0; i < DEBOUNCE_VALUE; i++){} // Lazy button debouncing

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
		
		
	// ------- Auto idle and power off timer section 

		// Tick this variable and compare it to the AUTPPOWEROFFTIME defined in config.h
		autoPowerOffTimerTick++;

		// If equal to IDLE_TIME, we set the idle state
		if (autoPowerOffTimerTick == IDLE_TIME && _IS_TEMP_STATE)
		{
			CurrentState = IDLE;
			autoPowerOffTimerTick = 0;
		}

		//Or if sleep time, we sleep
		if (autoPowerOffTimerTick == SLEEP_TIME && (CurrentState == IDLE))
		{
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
