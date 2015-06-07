#include "stm32l0xx_hal.h"
#include "prototypes.h"
#include "config.h"

SPI_HandleTypeDef hspi1;
HAL_StatusTypeDef status;

uint8_t spiData[6];
IMUMotion volatile motionNew = { 0, 0, 0 };
IMUMotion volatile motionOld = { 0, 0, 0 };

// Private prototypes
void IMUReadBytes(uint8_t address, uint8_t *pdata, uint8_t count);
uint8_t IMUWriteByte(uint8_t address, uint8_t data);


void startIMU(void)
{
	// Check that the device exists
	IMUReadBytes(WHO_AM_I, &spiData, 1);

	if (spiData[0] == 0x49)
	{
		IMUWriteByte(CTRL1, 0x67); // Set up and start accel
		IMUWriteByte(CTRL2, 0xC0); // Set up and AA filter
		IMUExists = 1;
	}
	
	// Flag is cleared if we didnt detect a device
	else IMUExists = 0;
}

void stopIMU(void)
{
	// Check that the device exists
	if (IMUExists)
	{
		IMUWriteByte(CTRL1, 0x07); // Set up and start accel
		IMUExists = 1;
	}
}

uint8_t checkForMovement(void)
{
	uint8_t flag = 0;

	if (IMUExists)
	{
		IMUReadBytes(ACCEL_START_REG, &spiData, 6);

		motionNew.X = (uint16_t)((spiData[1] << 8) | spiData[0]);
		motionNew.Y = (uint16_t)((spiData[3] << 8) | spiData[2]);
		motionNew.Z = (uint16_t)((spiData[5] << 8) | spiData[4]);

		if (
			(motionNew.X > motionOld.X + MOTION_THRESHOLD) ||
			(motionNew.X < motionOld.X - MOTION_THRESHOLD) ||
			(motionNew.Y > motionOld.Y + MOTION_THRESHOLD) ||
			(motionNew.Y < motionOld.Y - MOTION_THRESHOLD) ||
			(motionNew.Z > motionOld.Z + MOTION_THRESHOLD) ||
			(motionNew.Z < motionOld.Z - MOTION_THRESHOLD)     )
		{
			flag = 1;
		}

		motionOld.X = motionNew.X;
		motionOld.Y = motionNew.Y;
		motionOld.Z = motionNew.Z;
	}

	return flag;
}


void IMUReadBytes(uint8_t address, uint8_t *pdata, uint8_t count)
{
	// This bit of the address enables read
	address |= (1 << 7);

	// This bit auto increments the register
	if (count > 1)
		address |= (1 << 6);

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

	status = HAL_SPI_Transmit(&hspi1, &address, 1, 0);
	status = HAL_SPI_Receive(&hspi1, pdata, count, 0);
	
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
}

uint8_t IMUWriteByte(uint8_t address, uint8_t data)
{
	uint8_t outData[2];
	outData[0] = address;
	outData[1] = data;

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

	status = HAL_SPI_Transmit(&hspi1, &outData, 2, 0);

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

	return 0;
}