#include "stm32l0xx_hal.h"
#include "prototypes.h"
#include "config.h"

#define WHO_AM_I	0x0F
#define CTRL0		0x1E
#define CTRL1		0x20
#define CTRL3		0x22
#define IG_CFG1		0x30
#define IG_THS1		0x32
#define IG_DUR1		0x33


uint8_t IMUExists;

SPI_HandleTypeDef hspi1;
HAL_StatusTypeDef status;

uint8_t dataOut[3];
uint8_t dataIn[3];

// Private prototypes
uint8_t IMUReadByte(uint8_t address);
uint8_t IMUWriteByte(uint8_t address, uint8_t data);



void configureIMU(void)
{
	// We need to put NSS high before we start using the SPI bus. It'll be low on powerup
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

	// Check that the device exists
	if (IMUReadByte(WHO_AM_I) == 0x49)
	{
		//IMUWriteByte(CTRL0, 0x02); // Set HPF on INT1 pin
		IMUWriteByte(CTRL1, 0x67); // Set up and start accel
		IMUWriteByte(CTRL3, 0x20); // Enable the interrupt generator
		IMUWriteByte(IG_CFG1, 0x3F); // Configure interrupt generator
		IMUWriteByte(IG_THS1, 0x0F); // Set int threshold
		IMUWriteByte(IG_DUR1, 0x01); // Set int duration

		IMUExists = 1;
	}
	
	// Flag is cleared if we didnt detect a device
	else IMUExists = 0;
}

uint8_t IMUReadByte(uint8_t address)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

	address |= (1 << 7);
	dataOut[0] = address;
	
	status = HAL_SPI_Transmit(&hspi1, &dataOut, 1, 0);
	status = HAL_SPI_Receive(&hspi1, &dataIn, 1, 0);
	
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

	return dataIn[0];
}

uint8_t IMUWriteByte(uint8_t address, uint8_t data)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

	dataOut[0] = address;
	dataOut[1] = data;

	status = HAL_SPI_Transmit(&hspi1, &dataOut, 2, 0);

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

	return 0;
}