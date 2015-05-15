#include "stm32l0xx_hal.h"
#include "prototypes.h"
#include "config.h"

typedef enum state_t
{
	SLEEP,
	IDLE,
	TEMP1,
	TEMP2,
	TEMP3,
	TEMP4,
	TEMP5,
	TEMP6,
	TEMP7,
	TEMP8,
	TEMP9

} state;

//uint8_t CurrentState;

state CurrentState = SLEEP;

void stateMachine(void)
{
	switch (CurrentState)
	{
		case SLEEP:
			clearLEDs();
			break;

		case IDLE:
			clearLEDs();
			break;

		case TEMP1:
			setLED(LED_1);
			break;

		case TEMP2:
			setLED(LED_2);
			break;

		case TEMP3:
			setLED(LED_3);
			break;

		case TEMP4:
			setLED(LED_4);
			break;

		case TEMP5:
			setLED(LED_5);
			break;

		case TEMP6:
			setLED(LED_6);
			break;

		case TEMP7:
			setLED(LED_7);
			break;

		case TEMP8:
			setLED(LED_8);
			break;

		case TEMP9:
			setLED(LED_9);
			break;

		default:
			CurrentState = IDLE;
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

	if (GPIO_Pin == GPIO_PIN_0)
	{
		if (HAL_GPIO_ReadPin(TEMPDN_PORT, TEMPDN_PIN) == GPIO_PIN_RESET)
		{
			if (CurrentState > TEMP1)
				CurrentState--;
		}
	}

	if (GPIO_Pin == GPIO_PIN_8)
	{
		if (HAL_GPIO_ReadPin(TEMPUP_PORT, TEMPUP_PIN) == GPIO_PIN_RESET)
		{
			if (CurrentState == SLEEP || CurrentState == IDLE)
				CurrentState = TEMP1; // Load last temp from flash

			else if (CurrentState < TEMP9)
				CurrentState++;
		}
	}
}

void tempUpPressed(void)
{
	if (CurrentState <= TEMP9)
		CurrentState++;
}

void tempDnPressed(void)
{
	CurrentState--;
}