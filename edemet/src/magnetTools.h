#ifndef MAGNET_TOOL_H
#define MAGNET_TOOL_H
#include "xparameters.h"
#include "stdbool.h"
#include "FreeRTOS.h"
void updateTemperaturePL(uint8_t id, int32_t temperature);
float interpretMagneticField(int16_t rawValue);
float interpretTempearture(int16_t rawValue);
void setledPanelColor(uint8_t LED, uint8_t R, uint8_t G, uint8_t B);
void setPwmMode(uint8_t id, bool mode);

#endif