#ifndef __PROTOTYPES_H__
#define __PROTOTYPES_H__

// Statemachine related
void stateMachine(void);

// Temp regulation related
void startTipTempMeasurement(void);
void basicController(uint32_t setpoint);

// LED Related
void setLED(uint8_t LED);
void clearLEDs();

// IMU Related
void startIMU(void);
void stopIMU(void);
uint8_t checkForMovement(void);


#endif
