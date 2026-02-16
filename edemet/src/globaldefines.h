#ifndef _GLOBAL_DEFINES
#define _GLOBAL_DEFINES

#include <stdio.h>
#include <stdint.h>
#include "xparameters.h"

#define VERSION_MAYOR 0
#define VERSION_MINOR 1
#define VERSION_PATCH 2

#define device_internal_id 0x01


//Base clock for PWM is 1 MHz or 1000 kHz
#define PWM_BASE_CLK 1000.0
#define MIN_PWM_FREQ (PWM_BASE_CLK/4096.0)
#define PWM_TARGET_FREQ 1


extern const uint32_t PWM_ADDRESS[30];


void enableAllDutyCycle(void);
void disableAllDutyCycle(void);
void setPwmFrequency(uint8_t , float);
void setDutyCycle(uint8_t , float );
int16_t get13s(const uint8_t *buf, uint32_t index);

#endif