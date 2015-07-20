 /*
 * Brief:       Main config, pin mappings and global variables.
 *
 *				From here we can see all the major variables shared	between the 
 *				soldering pen controller as well as all the peripherals. There are
 *				also many config values that can be tweaked for different setups.
 *
 * Copyright (C) 2015 Rajesh Nakarja
 * http://www.naklojik.com
 *
 * This software is licenced under the MIT Licence. For full terms visit:
 * http://opensource.org/licenses/MIT
 */

#ifndef __CONFIG__
#define __CONFIG__

/*------------------------*/
/* PIN MAPPINGS FOR BOARD */
/*------------------------*/

	/* This is just a reference, don't change these pins without 
	   checking where they are used. Each pin also have
	   interrupts tied to them, so they'd also need changing */

	/* LED Pins */
	#define LEDA_PIN		GPIO_PIN_8
	#define LEDA_PORT 		GPIOA
	
	#define LEDB_PIN		GPIO_PIN_9
	#define LEDB_PORT 		GPIOA
	
	#define LEDC_PIN		GPIO_PIN_10
	#define LEDC_PORT 		GPIOA
	
	#define LED1_PIN		GPIO_PIN_15
	#define LED1_PORT 		GPIOA
	
	#define LED2_PIN		GPIO_PIN_3
	#define LED2_PORT 		GPIOB
	
	#define LED3_PIN		GPIO_PIN_4
	#define LED3_PORT 		GPIOB

	/* Button inputs */
	#define TEMPDN_PIN		GPIO_PIN_8
	#define TEMPDN_PORT		GPIOB
	
	#define TEMPUP_PIN		GPIO_PIN_0
	#define TEMPUP_PORT 	GPIOA

	/* IO Signals */
	#define HEATER_PIN		GPIO_PIN_2
	#define HEATER_PORT		GPIOA

	#define THERM_PIN		GPIO_PIN_1
	#define THERM_PORT		GPIOA

	/* Accelerometer signals */
	#define ACCELINT_PIN	GPIO_PIN_1
	#define ACCELINT_PORT	GPIOB


/*------------------------*/
/*      IMU VARIABLES     */
/*------------------------*/

	#define MOTION_THRESHOLD 2000

	#define WHO_AM_I		 0x0F
	#define CTRL1			 0x20
	#define CTRL2			 0x21
	#define ACCEL_START_REG  0x28

	uint8_t IMUExists;

/*------------------------*/
/*      IO VARIABLES      */
/*------------------------*/

	#define LED_BLINK_TIME 1000
	#define DEBOUNCE_VALUE 5000

/*------------------------*/
/*  Controller Variables  */
/*------------------------*/

	#define Kp 				1.00
	#define Ki 				0.01
	#define Kd 				0.01

	#define iMax 			100
	#define iMin 			-100

	#define IDLE_TIME  		(60*5)
	#define SLEEP_TIME 		(180*10)

	#define SET_POINT_1		500
	#define SET_POINT_2		800
	#define SET_POINT_3		1000
	#define SET_POINT_4		1500
	#define SET_POINT_5		2000
	#define SET_POINT_6		2500
	#define SET_POINT_7		3000
	#define SET_POINT_8		3500
	#define SET_POINT_9		4000

/*------------------------*/
/*  USEFUL ENUMS/STRUCTS  */
/*------------------------*/

	typedef enum led_t
	{
		LED_1,
		LED_2,
		LED_3,
		LED_4,
		LED_5,
		LED_6,
		LED_7,
		LED_8,
		LED_9,

	} led;

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

	typedef struct
	{
		int16_t X, Y, Z;

	} IMUMotion;

#endif
