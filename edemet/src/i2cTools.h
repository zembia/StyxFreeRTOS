#ifndef I2CTOOLS_H
#define I2CTOOLS_H

#include "xparameters.h"
#include "stdbool.h"
#include "FreeRTOS.h"

#define PCA9535_INPUT_PORT_0      0x00
#define PCA9535_INPUT_PORT_1      0x01
#define PCA9535_OUTPUT_PORT_0     0x02
#define PCA9535_OUTPUT_PORT_1     0x03
#define PCA9535_CONFIG_PORT_0     0x06
#define PCA9535_CONFIG_PORT_1     0x07

#define ADS1115_ADDR 0x48


// ADS1115 Register Addresses
#define ADS1115_REG_CONVERSION  0x00
#define ADS1115_REG_CONFIG      0x01

// ADS1115 Configuration bits
#define ADS1115_OS_SINGLE       0x8000  // Start single conversion
#define ADS1115_MUX_AIN0_GND    0x4000  // AIN0 vs GND
#define ADS1115_MUX_AIN1_GND    0x5000  // AIN1 vs GND
#define ADS1115_MUX_AIN2_GND    0x6000  // AIN2 vs GND
#define ADS1115_MUX_AIN3_GND    0x7000  // AIN3 vs GND
#define ADS1115_PGA_6_144V      0x0000  // ±6.144V range
#define ADS1115_PGA_4_096V      0x0200  // ±4.096V range
#define ADS1115_PGA_2_048V      0x0400  // ±2.048V range (default)
#define ADS1115_PGA_1_024V      0x0600  // ±1.024V range
#define ADS1115_PGA_0_512V      0x0800  // ±0.512V range
#define ADS1115_PGA_0_256V      0x0A00  // ±0.256V range
#define ADS1115_MODE_SINGLE     0x0100  // Single-shot mode
#define ADS1115_MODE_CONTINUOUS 0x0000  // Continuous conversion
#define ADS1115_DR_128SPS       0x0080  // 128 samples per second
#define ADS1115_DR_250SPS       0x00A0  // 250 samples per second
#define ADS1115_DR_860SPS       0x00E0  // 860 samples per second
#define ADS1115_COMP_QUE_DISABLE 0x0003 // Disable comparator
#define ADS1115_MAGNETIC_FIELD  ADS1115_MUX_AIN0_GND
#define ADS1115_TEMP1           ADS1115_MUX_AIN1_GND
#define ADS1115_TEMP2           ADS1115_MUX_AIN2_GND

// ADS1115 function prototypes
bool ADS1115_ReadChannel(UINTPTR BaseAddress, uint8_t mux_channel,
                         uint8_t DevAddr, uint8_t channel, int16_t *result);
bool ADS1115_ReadAN0_AN1_AN2(UINTPTR BaseAddress, uint8_t mux_channel,
                             int16_t *an0, int16_t *an1, int16_t *an2);
int32_t ADS1115_RawToMillivolts(int16_t raw, uint16_t pga);

bool ADS1115_WriteReg(UINTPTR BaseAddress, uint8_t DevAddr, 
                      uint8_t Reg, uint16_t Data);
bool ADS1115_ReadReg(UINTPTR BaseAddress, uint8_t DevAddr,
                     uint8_t Reg, uint16_t *Data);

void checkAllI2CDevices(void);
void setIICmux(UINTPTR BaseAddress, uint8_t index);
bool checkPresence(UINTPTR BaseAddress, uint8_t deviceAddress);
void checkIICchannel(UINTPTR BaseAddress);
bool PCA9535_WriteReg(UINTPTR BaseAddress, uint8_t DevAddr, uint8_t Reg, uint8_t Data);
bool PCA9535_ReadReg(UINTPTR BaseAddress, uint8_t DevAddr,  uint8_t Reg, uint8_t *Data);
bool PCA9535_SetPins0to11_Output(UINTPTR BaseAddress, uint8_t DevAddr);
bool PCA9535_SetPin(UINTPTR BaseAddress, uint8_t DevAddr, uint8_t Pin, bool Value);
bool PCA9535_SetPins0to11(UINTPTR BaseAddress, uint8_t DevAddr, bool Value);


void enableOutputs(UINTPTR);
void disableOutputs(UINTPTR);
void configureOutputs(UINTPTR);

// I2C bus recovery functions
bool I2C_CheckBusIdle(UINTPTR BaseAddress);
void I2C_ResetBus(UINTPTR BaseAddress);
bool I2C_SafeSend(UINTPTR BaseAddress, uint8_t DevAddr, uint8_t *data, uint8_t len, uint8_t option);
bool I2C_SafeRecv(UINTPTR BaseAddress, uint8_t DevAddr, uint8_t *data, uint8_t len, uint8_t option);

#endif
