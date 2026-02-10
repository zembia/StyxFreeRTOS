#include "xparameters.h"
#include "stdbool.h"
#include "xiic_l.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "stdio.h"


#include "i2cTools.h"
#include <projdefs.h>

#define INTC_DEVICE_ID		XPAR_SCUGIC_SINGLE_DEVICE_ID
#define INTC			 	XScuGic
#define INTC_HANDLER		XScuGic_InterruptHandler

#define INA740_ADDR 0x40
#define TMP102_ADDR 0x4B
#define TCA9548ARGER_ADDR 0x70

#define IOEXP1_ADDR 0x20
#define IOEXP2_ADDR 0x21
#define IOEXP3_ADDR 0x22

uint8_t ioexp_addr[]={IOEXP1_ADDR,IOEXP2_ADDR,IOEXP3_ADDR};

uint8_t ADDRESS_TO_CHECK[] = {INA740_ADDR,TMP102_ADDR};

//Function prototypes
static uint8_t pca9535_out0_shadow = 0x00;
static uint8_t pca9535_out1_shadow = 0x00;


bool PCA9535_WriteReg(UINTPTR BaseAddress, uint8_t DevAddr,
                      uint8_t Reg, uint8_t Data)
{
    uint8_t buf[2] = { Reg, Data };

    if (XIic_Send(BaseAddress, DevAddr, buf, 2, XIIC_STOP) != 2)
        return false;

    return true;
}

bool PCA9535_ReadReg(UINTPTR BaseAddress, uint8_t DevAddr,
                     uint8_t Reg, uint8_t *Data)
{
    if (XIic_Send(BaseAddress, DevAddr, &Reg, 1, XIIC_REPEATED_START) != 1)
        return false;

    if (XIic_Recv(BaseAddress, DevAddr, Data, 1, XIIC_STOP) != 1)
        return false;

    return true;
}


void checkAllI2CDevices(void)
{


    bool present;
    int i;
    printf("Checking IOEXPANDER BUS 1:\r\n");
    
    
    for (i=0;i<(int)sizeof(ioexp_addr);i++)
    {
        present = checkPresence(XPAR_AXI_IIC_IOEXP_1_BASEADDR,ioexp_addr[i]);
        if (present){
            printf("\t%02X present\r\n",ioexp_addr[i]);
        }else {
            printf("\t%02X not present\r\n",ioexp_addr[i]);
        }
    }
    

    printf("Checking IOEXPANDER BUS 2:\r\n");
    
    for (i=0;i<(int)sizeof(ioexp_addr);i++)
    {
        present = checkPresence(XPAR_AXI_IIC_IOEXP_2_BASEADDR,ioexp_addr[i]);
        if (present){
            printf("\t%02X present\r\n",ioexp_addr[i]);
        }else {
            printf("\t%02X not present\r\n",ioexp_addr[i]);
        }
    }
    

    // Dome pulses on the ioexpander
    // Set pins as OUTPUTS
    printf("Init setting as outputs\r\n");
    bool res;
    for (i = 0; i < (int)sizeof(ioexp_addr); i++) {
        res = PCA9535_SetPins0to11_Output(XPAR_AXI_IIC_IOEXP_1_BASEADDR, ioexp_addr[i]);
        if (!res) {
            printf("Error setting pin as output\r\n");
            break;
        }
        res = PCA9535_SetPins0to11_Output(XPAR_AXI_IIC_IOEXP_2_BASEADDR, ioexp_addr[i]);
        if (!res) {
            printf("Error setting pin as output\r\n");
            break;
        }
    }
    printf("Done setting as outputs\r\n");
    if (!res) {
        printf("Error setting pin as output\r\n");
        return;
    }


    printf("Checking AXI_A:\r\n");
    checkIICchannel(XPAR_I2C_MAGNET_PORTS_AXI_IIC_A_BASEADDR);
    printf("Checking AXI_B:\r\n");
    checkIICchannel(XPAR_I2C_MAGNET_PORTS_AXI_IIC_B_BASEADDR);
    printf("Checking AXI_C:\r\n");
    checkIICchannel(XPAR_I2C_MAGNET_PORTS_AXI_IIC_C_BASEADDR);
    printf("Checking AXI_D:\r\n");
    checkIICchannel(XPAR_I2C_MAGNET_PORTS_AXI_IIC_D_BASEADDR);
    printf("Checking AXI_E:\r\n");
    checkIICchannel(XPAR_I2C_MAGNET_PORTS_AXI_IIC_E_BASEADDR);
    printf("Checking AXI_F:\r\n");
    checkIICchannel(XPAR_I2C_MAGNET_PORTS_AXI_IIC_F_BASEADDR);    

}


void checkIICchannel(UINTPTR BaseAddress)
{
    uint8_t i,j;    
    
    printf("\tIICMUX...");
    if (checkPresence(BaseAddress,TCA9548ARGER_ADDR))
    {
        printf("\tfound!\r\n");
    }
    else {
        printf("\tnot found\r\n");
    }
    for (i=0;i<5;i++)
    {
        
        setIICmux(BaseAddress,1<<i);
        printf("\tPort %d...\r\n",i);

        for (j=0;j<sizeof(ADDRESS_TO_CHECK);j++)
        {
            printf("\t\t%02X...",ADDRESS_TO_CHECK[j]);
            if (checkPresence(BaseAddress,ADDRESS_TO_CHECK[j]))
            {
                printf("\tfound!\r\n");
            }
            else{
                printf("\tnot found\r\n");
            }
        }
    }
}

void setIICmux(UINTPTR BaseAddress, uint8_t index)
{
    XIic_Send(BaseAddress, TCA9548ARGER_ADDR, &index, 1, XIIC_STOP);
}


bool checkPresence(UINTPTR BaseAddress, uint8_t deviceAddress)
{
    
    uint8_t dummy = 0x00;

    // Limpiar interrupciones pendientes
    XIic_WriteReg(BaseAddress, XIIC_IISR_OFFSET, 0xFFFFFFFF);

    //Enviamos una lectura
    //XIic_Send(BaseAddress, deviceAddress, &dummy, 1, XIIC_STOP);
    XIic_Recv(BaseAddress,deviceAddress,&dummy,1,XIIC_STOP);
    
    u16 StatusReg;
    int timeout = 100;
    while(timeout-- > 0)
    {
        
        //leemos el registro de estado
        StatusReg = XIic_ReadReg(BaseAddress, XIIC_SR_REG_OFFSET);
        //Esperamos a que la transacción esté completa
        if (StatusReg & XIIC_SR_BUS_BUSY_MASK)
        {
            vTaskDelay(1);
            continue;
        }
        break;
    }
    // Leer el registro de interrupciones
    StatusReg = XIic_ReadReg(BaseAddress, XIIC_IISR_OFFSET);
    
    // Verificar si hubo error de arbitraje o NACK
    if (StatusReg & (XIIC_INTR_ARB_LOST_MASK|XIIC_INTR_TX_ERROR_MASK)) {
        XIic_WriteReg(BaseAddress, XIIC_IISR_OFFSET, StatusReg);
        return false;  // Perdió el arbitraje
    }
    
    //En caso contrario tenemos exito
    return true;
    
}

bool PCA9535_SetPins0to11_Output(UINTPTR BaseAddress, uint8_t DevAddr)
{
    uint8_t cfg0, cfg1;

    // Read current configuration
    if (!PCA9535_ReadReg(BaseAddress, DevAddr, PCA9535_CONFIG_PORT_0, &cfg0))
    {
        xil_printf("Could not read configuration PCA9535");
        return false;
    }
    if (!PCA9535_ReadReg(BaseAddress, DevAddr, PCA9535_CONFIG_PORT_1, &cfg1))
    {
        xil_printf("Could not read configuration PCA9535");
        return false;
    }
    
    vTaskDelay(1);


    // Port 0: pins 0–7 all outputs
    cfg0 = 0x00;

    // Port 1: pins 8–11 outputs, 12–15 unchanged
    cfg1 &= 0xF0;   // clear bits 0–3

    // Write back
    if (!PCA9535_WriteReg(BaseAddress, DevAddr, PCA9535_CONFIG_PORT_0, cfg0))
    {
        xil_printf("Could not write configuration PCA9535");
        return false;
    }
    if (!PCA9535_WriteReg(BaseAddress, DevAddr, PCA9535_CONFIG_PORT_1, cfg1))
    {
        xil_printf("Could not write configuration PCA9535");
        return false;
    }
    vTaskDelay(1);

    return true;
}

bool PCA9535_SetPin(UINTPTR BaseAddress, uint8_t DevAddr,
                    uint8_t Pin, bool Value)
{
    uint8_t reg, data, bit;

    if (Pin > 11)
        return false;

    if (Pin < 8) {
        reg = PCA9535_OUTPUT_PORT_0;
        bit = Pin;
    } else {
        reg = PCA9535_OUTPUT_PORT_1;
        bit = Pin - 8;
    }

    // Read current output register
    if (!PCA9535_ReadReg(BaseAddress, DevAddr, reg, &data))
        return false;

    if (Value)
        data |=  (1 << bit);
    else
        data &= ~(1 << bit);

    // Write back
    if (!PCA9535_WriteReg(BaseAddress, DevAddr, reg, data))
        return false;

    return true;
}


void configureOutputs(UINTPTR BaseAddress) {
    for (int i = 0; i < (int)sizeof(ioexp_addr); i++){
        PCA9535_SetPins0to11_Output(BaseAddress, ioexp_addr[i]);        
    }
    
}



void enableOutputs(UINTPTR BaseAddress) {
    for (int i = 0; i < (int)sizeof(ioexp_addr); i++){
        PCA9535_SetPins0to11(BaseAddress, ioexp_addr[i], 0);
        // PCA9535_SetPins0to11(XPAR_AXI_IIC_IOEXP_1_BASEADDR, ioexp_addr[i], 0);
        // PCA9535_SetPins0to11(XPAR_AXI_IIC_IOEXP_2_BASEADDR, ioexp_addr[i], 0);
    }
    
}

void disableOutputs(UINTPTR BaseAddress) {
    for (int i = 0; i < (int)sizeof(ioexp_addr); i++){
        PCA9535_SetPins0to11(BaseAddress, ioexp_addr[i], 1);
        // PCA9535_SetPins0to11(XPAR_AXI_IIC_IOEXP_1_BASEADDR, ioexp_addr[i], 1);
        // PCA9535_SetPins0to11(XPAR_AXI_IIC_IOEXP_2_BASEADDR, ioexp_addr[i], 1);
    }
}

bool PCA9535_SetPins0to11(UINTPTR BaseAddress,
                          uint8_t DevAddr,
                          bool Value)
{
    if (Value) {
        // Set pins 0–7 HIGH
        pca9535_out0_shadow = 0xFF;

        // Set pins 8–11 HIGH, keep 12–15 unchanged
        pca9535_out1_shadow |= 0x0F;
    } else {
        // Set pins 0–7 LOW
        pca9535_out0_shadow = 0x00;

        // Set pins 8–11 LOW, keep 12–15 unchanged
        pca9535_out1_shadow &= 0xF0;
    }

    // Write both ports (no reads = fast)
    if (!PCA9535_WriteReg(BaseAddress, DevAddr,
                          PCA9535_OUTPUT_PORT_0,
                          pca9535_out0_shadow))
        return false;

    if (!PCA9535_WriteReg(BaseAddress, DevAddr,
                          PCA9535_OUTPUT_PORT_1,
                          pca9535_out1_shadow))
        return false;

    return true;
}
/**
 * @brief Write 16-bit value to ADS1115 register
 */
bool ADS1115_WriteReg(UINTPTR BaseAddress, uint8_t DevAddr, 
                      uint8_t Reg, uint16_t Data)
{
    uint8_t buf[3];
    buf[0] = Reg;
    buf[1] = (Data >> 8) & 0xFF;  // MSB first
    buf[2] = Data & 0xFF;          // LSB second

    // Clear pending interrupts
    XIic_WriteReg(BaseAddress, XIIC_IISR_OFFSET, 0xFFFFFFFF);

    if (XIic_Send(BaseAddress, DevAddr, buf, 3, XIIC_STOP) != 3)
        return false;

    // Wait for bus to be idle (critical!)
    u16 StatusReg;
    int timeout = 100;
    while(timeout-- > 0) {
        StatusReg = XIic_ReadReg(BaseAddress, XIIC_SR_REG_OFFSET);
        if (!(StatusReg & XIIC_SR_BUS_BUSY_MASK))
            break;
        vTaskDelay(1);
    }

    if (timeout <= 0)
        return false;

    // Check for errors
    StatusReg = XIic_ReadReg(BaseAddress, XIIC_IISR_OFFSET);
    if (StatusReg & (XIIC_INTR_ARB_LOST_MASK | XIIC_INTR_TX_ERROR_MASK)) {
        XIic_WriteReg(BaseAddress, XIIC_IISR_OFFSET, StatusReg);
        return false;
    }

    return true;
}

/**
 * @brief Read 16-bit value from ADS1115 register
 */
bool ADS1115_ReadReg(UINTPTR BaseAddress, uint8_t DevAddr,
                     uint8_t Reg, uint16_t *Data)
{
    uint8_t buf[2];
    u16 StatusReg;
    int timeout;

    // Clear pending interrupts
    XIic_WriteReg(BaseAddress, XIIC_IISR_OFFSET, 0xFFFFFFFF);

    // Write register pointer
    if (XIic_Send(BaseAddress, DevAddr, &Reg, 1, XIIC_REPEATED_START) != 1)
        return false;

    // Wait for write to complete
    timeout = 100;
    while(timeout-- > 0) {
        StatusReg = XIic_ReadReg(BaseAddress, XIIC_SR_REG_OFFSET);
        if (!(StatusReg & XIIC_SR_BUS_BUSY_MASK))
            break;
        vTaskDelay(1);
    }

    // Small delay between write and read
    vTaskDelay(1);

    // Clear interrupts again before read
    XIic_WriteReg(BaseAddress, XIIC_IISR_OFFSET, 0xFFFFFFFF);

    // Read 2 bytes
    if (XIic_Recv(BaseAddress, DevAddr, buf, 2, XIIC_STOP) != 2)
        return false;

    // Wait for read to complete
    timeout = 100;
    while(timeout-- > 0) {
        StatusReg = XIic_ReadReg(BaseAddress, XIIC_SR_REG_OFFSET);
        if (!(StatusReg & XIIC_SR_BUS_BUSY_MASK))
            break;
        vTaskDelay(1);
    }

    if (timeout <= 0)
        return false;

    // Check for errors
    StatusReg = XIic_ReadReg(BaseAddress, XIIC_IISR_OFFSET);
    if (StatusReg & (XIIC_INTR_ARB_LOST_MASK | XIIC_INTR_TX_ERROR_MASK)) {
        XIic_WriteReg(BaseAddress, XIIC_IISR_OFFSET, StatusReg);
        return false;
    }

    // ADS1115 sends MSB first, then LSB
    *Data = ((uint16_t)buf[0] << 8) | buf[1];
    return true;
}

/**
 * @brief Read a single channel from ADS1115
 * Per datasheet: Write config with OS=1, wait for conversion, poll OS bit, read result
 */
bool ADS1115_ReadChannel(UINTPTR BaseAddress, uint8_t mux_channel,
                         uint8_t DevAddr, uint8_t channel, int16_t *result)
{
    uint16_t config;
    uint16_t mux;

    if (channel > 3)
        return false;

    // Select I2C mux channel if specified
    if (mux_channel != 0xFF) {
        setIICmux(BaseAddress, 1 << mux_channel);
        vTaskDelay(pdMS_TO_TICKS(5));  // Wait after mux switch
    }

    // Select MUX configuration
    switch (channel) {
        case 0: mux = ADS1115_MUX_AIN0_GND; break;
        case 1: mux = ADS1115_MUX_AIN1_GND; break;
        case 2: mux = ADS1115_MUX_AIN2_GND; break;
        case 3: mux = ADS1115_MUX_AIN3_GND; break;
        default: return false;
    }

    // Build configuration (per datasheet Table 9)
    config = ADS1115_OS_SINGLE |           // Start single conversion
             mux |                          // Select input
             ADS1115_PGA_4_096V |          // ±4.096V range
             ADS1115_MODE_SINGLE |         // Single-shot mode
             ADS1115_DR_128SPS |           // 128 SPS
             ADS1115_COMP_QUE_DISABLE;     // Disable comparator

    xil_printf("Writing config 0x%04X to start conversion\r\n", config);
    
    // Write configuration to start conversion (per datasheet section 9.3.2)
    if (!ADS1115_WriteReg(BaseAddress, DevAddr, ADS1115_REG_CONFIG, config)) {
        xil_printf("Failed to write config\r\n");
        return false;
    }

    // Per datasheet: at 128 SPS, conversion takes 1/128 = 7.8ms
    // Add margin for safety
    vTaskDelay(pdMS_TO_TICKS(10));

    // Poll config register until OS bit = 1 (conversion complete)
    // Per datasheet section 9.3.3: "poll the OS bit"
    int timeout = 50;
    int poll_count = 0;
    
    while (timeout-- > 0) {
        if (!ADS1115_ReadReg(BaseAddress, DevAddr, ADS1115_REG_CONFIG, &config)) {
            xil_printf("Failed to read config at poll %d\r\n", poll_count);
            return false;
        }

        poll_count++;
        
        // Check if OS bit is set (bit 15 = 1 means conversion complete)
        if (config & 0x8000) {
            xil_printf("Conversion complete after %d polls\r\n", poll_count);
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(2));
    }

    if (timeout <= 0) {
        xil_printf("Timeout waiting for conversion\r\n");
        return false;
    }

    // Read conversion result from register 0x00
    uint16_t raw;
    if (!ADS1115_ReadReg(BaseAddress, DevAddr, ADS1115_REG_CONVERSION, &raw)) {
        xil_printf("Failed to read conversion result\r\n");
        return false;
    }
    
    xil_printf("Read raw value: 0x%04X (%d)\r\n", raw, (int16_t)raw);

    *result = (int16_t)raw;
    return true;
}

/**
 * @brief Read AN0, AN1, and AN2 from ADS1115 (with I2C mux support)
 * @param BaseAddress: I2C base address
 * @param mux_channel: I2C mux channel (0-7), or 0xFF to skip mux
 * @param an0: Pointer to store AN0 result
 * @param an1: Pointer to store AN1 result
 * @param an2: Pointer to store AN2 result
 * @return true if all reads successful, false otherwise
 */
bool ADS1115_ReadAN0_AN1_AN2(UINTPTR BaseAddress, uint8_t mux_channel,
                             int16_t *an0, int16_t *an1, int16_t *an2)
{
    // Select I2C mux channel once at the beginning
    if (mux_channel != 0xFF) {
        setIICmux(BaseAddress, 1 << mux_channel);
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    if (!ADS1115_ReadChannel(BaseAddress, 0xFF, ADS1115_ADDR, 0, an0)) {
        printf("ERROR: Failed to read AN0\r\n");
        return false;
    }

    if (!ADS1115_ReadChannel(BaseAddress, 0xFF, ADS1115_ADDR, 1, an1)) {
        printf("ERROR: Failed to read AN1\r\n");
        return false;
    }

    if (!ADS1115_ReadChannel(BaseAddress, 0xFF, ADS1115_ADDR, 2, an2)) {
        printf("ERROR: Failed to read AN2\r\n");
        return false;
    }

    return true;
}


/**
 * @brief Convert ADS1115 raw value to voltage (in millivolts)
 * @param raw: Raw 16-bit signed value from ADC
 * @param pga: PGA setting (use same as in ADS1115_ReadChannel)
 * @return Voltage in millivolts
 */
int32_t ADS1115_RawToMillivolts(int16_t raw, uint16_t pga)
{
    // LSB size in microvolts based on PGA setting
    int32_t lsb_uv;

    switch (pga) {
        case ADS1115_PGA_6_144V: lsb_uv = 187; break;  // 187.5 µV
        case ADS1115_PGA_4_096V: lsb_uv = 125; break;  // 125 µV
        case ADS1115_PGA_2_048V: lsb_uv = 62;  break;  // 62.5 µV
        case ADS1115_PGA_1_024V: lsb_uv = 31;  break;  // 31.25 µV
        case ADS1115_PGA_0_512V: lsb_uv = 15;  break;  // 15.625 µV
        case ADS1115_PGA_0_256V: lsb_uv = 7;   break;  // 7.8125 µV
        default: lsb_uv = 125; break;  // Default to ±4.096V
    }

    // Convert to millivolts
    return (raw * lsb_uv) / 1000;
}