#ifndef __PROTOTYPES_H__
#define __PROTOTYPES_H__

// Statemachine related
void stateMachine(void);

// Temp regulation related
void startTipTempMeasurement(void);

// LED Related
void setLED(uint8_t LED);
void clearLEDs();

// IMU Related
void startIMU(void);
void stopIMU(void);
uint8_t checkForMovement(void);

// PID controller related
uint32_t PID_Controller(uint32_t setpoint, uint32_t sample);


#endif
