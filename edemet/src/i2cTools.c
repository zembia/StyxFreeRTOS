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
static unsigned XIic_Send_custom(UINTPTR BaseAddress, u8 Address,
		   u8 *BufferPtr, unsigned ByteCount, u8 Option);
static unsigned XIic_Recv_custom(UINTPTR BaseAddress, u8 Address,
			u8 *BufferPtr, unsigned ByteCount, u8 Option);
// I2C bus recovery functions
/**
 * @brief Check if I2C bus is idle
 */
bool I2C_CheckBusIdle(UINTPTR BaseAddress) {
    u16 StatusReg = XIic_ReadReg(BaseAddress, XIIC_SR_REG_OFFSET);
    return !(StatusReg & XIIC_SR_BUS_BUSY_MASK);
}

/**
 * @brief Reset I2C bus - clears errors and reinitializes
 */
void I2C_ResetBus(UINTPTR BaseAddress) {
    // Clear all interrupt flags
    XIic_WriteReg(BaseAddress, XIIC_IISR_OFFSET, 0xFFFFFFFF);
    
    // Software reset
    XIic_WriteReg(BaseAddress, XIIC_RESETR_OFFSET, 0x0A);
    

    //Do timing adjustment
/*
    xil_printf("Timing TSUSTA reg: %08X\r\n",XIic_ReadReg(BaseAddress, 0x128));
    xil_printf("Timing TSUSTB  reg: %08X\r\n",XIic_ReadReg(BaseAddress, 0x12C));
    xil_printf("Timing THDSTA  reg: %08X\r\n",XIic_ReadReg(BaseAddress, 0x130));
    xil_printf("Timing TSUDAT  reg: %08X\r\n",XIic_ReadReg(BaseAddress, 0x134));
    xil_printf("Timing TBUF    reg: %08X\r\n",XIic_ReadReg(BaseAddress, 0x138));
    xil_printf("Timing THIGH   reg: %08X\r\n",XIic_ReadReg(BaseAddress, 0x13C));
    xil_printf("Timing TLOW    reg: %08X\r\n",XIic_ReadReg(BaseAddress, 0x140));
    xil_printf("Timing THDDAT     reg: %08X\r\n",XIic_ReadReg(BaseAddress, 0x144));
*/
    vTaskDelay(pdMS_TO_TICKS(10));
    uint32_t CntlReg = XIic_ReadReg(BaseAddress,  XIIC_CR_REG_OFFSET);
    CntlReg = XIIC_CR_MSMS_MASK | XIIC_CR_ENABLE_DEVICE_MASK;		
    //XIic_WriteReg(BaseAddress,  XIIC_CR_REG_OFFSET, CntlReg);


    /*XIic_WriteReg(BaseAddress, 0x13C, 58);
    XIic_WriteReg(BaseAddress, 0x140, 58);*/
    

    
    
    xil_printf("[I2C RESET] Bus at 0x%08x reset\r\n", BaseAddress);
}

/**
 * @brief Safe I2C send with timeout and error recovery
 */
bool I2C_SafeSend(UINTPTR BaseAddress, uint8_t DevAddr, uint8_t *data, uint8_t len, uint8_t option) {
    //return XIic_Send_custom(BaseAddress,DevAddr,data,len,option);

    int retries = 3;
    
    while (retries-- > 0) {
        // Check if bus is stuck
        int timeout = 50;
        while (!I2C_CheckBusIdle(BaseAddress) && timeout-- > 0) {
            vTaskDelay(pdMS_TO_TICKS(1));
        }
        
        if (timeout <= 0) {
            xil_printf("[I2C] Bus stuck before send, resetting...\r\n");
            I2C_ResetBus(BaseAddress);
            continue;
        }
        
        // Clear pending interrupts
        XIic_WriteReg(BaseAddress, XIIC_IISR_OFFSET, 0xFFFFFFFF);
        
        // Attempt send
        int sent = XIic_Send_custom(BaseAddress, DevAddr, data, len, option);
        
        if (sent == len) {
            // Wait for completion with timeout
            timeout = 100;
            while (timeout-- > 0) {
                u16 StatusReg = XIic_ReadReg(BaseAddress, XIIC_SR_REG_OFFSET);
                if (!(StatusReg & XIIC_SR_BUS_BUSY_MASK)) {
                    // Check for errors
                    u16 IrqReg = XIic_ReadReg(BaseAddress, XIIC_IISR_OFFSET);
                    if (IrqReg & (XIIC_INTR_ARB_LOST_MASK | XIIC_INTR_TX_ERROR_MASK)) {
                        XIic_WriteReg(BaseAddress, XIIC_IISR_OFFSET, IrqReg);
                        xil_printf("[I2C] Send error detected\r\n");
                        break;
                    }
                    return true; // Success!
                }
                vTaskDelay(pdMS_TO_TICKS(1));
            }
            
            xil_printf("[I2C] Send timeout\r\n");
        }
        
        // Failed - reset and retry
        xil_printf("[I2C] Send failed (sent %d/%d), retry %d\r\n", sent, len, retries);
        I2C_ResetBus(BaseAddress);
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    
    return false;
}

/**
 * @brief Safe I2C receive with timeout and error recovery
 */
bool I2C_SafeRecv(UINTPTR BaseAddress, uint8_t DevAddr, uint8_t *data, uint8_t len, uint8_t option) {
    int retries = 3;
    
    while (retries-- > 0) {
        // Check if bus is stuck
        int timeout = 50;
        while (!I2C_CheckBusIdle(BaseAddress) && timeout-- > 0) {
            vTaskDelay(pdMS_TO_TICKS(1));
        }
        
        if (timeout <= 0) {
            xil_printf("[I2C] Bus stuck before recv, resetting...\r\n");
            I2C_ResetBus(BaseAddress);
            continue;
        }
        
        // Clear pending interrupts
        XIic_WriteReg(BaseAddress, XIIC_IISR_OFFSET, 0xFFFFFFFF);
        
        // Attempt receive
        
        int received = XIic_Recv_custom(BaseAddress, DevAddr, data, len, option);
        
        if (received == len) {
            // Wait for completion with timeout
            timeout = 100;
            while (timeout-- > 0) {
                u16 StatusReg = XIic_ReadReg(BaseAddress, XIIC_SR_REG_OFFSET);
                if (!(StatusReg & XIIC_SR_BUS_BUSY_MASK)) {
                    // Check for errors
                    u16 IrqReg = XIic_ReadReg(BaseAddress, XIIC_IISR_OFFSET);
                    if (IrqReg & (XIIC_INTR_ARB_LOST_MASK | XIIC_INTR_TX_ERROR_MASK)) {
                        XIic_WriteReg(BaseAddress, XIIC_IISR_OFFSET, IrqReg);
                        xil_printf("[I2C] Recv error detected\r\n");
                        break;
                    }
                    return true; // Success!
                }
                vTaskDelay(pdMS_TO_TICKS(1));
            }
            
            xil_printf("[I2C] Recv timeout\r\n");
        }
        
        // Failed - reset and retry
        xil_printf("[I2C] Recv failed (got %d/%d), retry %d\r\n", received, len, retries);
        I2C_ResetBus(BaseAddress);
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    
    return false;
}

uint8_t ADDRESS_TO_CHECK[] = {INA740_ADDR,TMP102_ADDR,ADS1115_ADDR};
//uint8_t ADDRESS_TO_CHECK[] = {INA740_ADDR,TMP102_ADDR};

//Function prototypes
static uint8_t pca9535_out0_shadow = 0x00;
static uint8_t pca9535_out1_shadow = 0x00;


bool PCA9535_WriteReg(UINTPTR BaseAddress, uint8_t DevAddr,
                      uint8_t Reg, uint8_t Data)
{
    uint8_t buf[2] = { Reg, Data };

    if (XIic_Send_custom(BaseAddress, DevAddr, buf, 2, XIIC_STOP) != 2)
        return false;

    return true;
}

bool PCA9535_ReadReg(UINTPTR BaseAddress, uint8_t DevAddr,
                     uint8_t Reg, uint8_t *Data)
{
    if (XIic_Send_custom(BaseAddress, DevAddr, &Reg, 1, XIIC_REPEATED_START) != 1)
        return false;

    if (XIic_Recv_custom(BaseAddress, DevAddr, Data, 1, XIIC_STOP) != 1)
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
        present = checkPresence(XPAR_I2C_MAGNET_PORTS_AXI_IIC_IOEXP_1_BASEADDR,ioexp_addr[i]);
        if (present){
            printf("\t%02X present\r\n",ioexp_addr[i]);
        }else {
            printf("\t%02X not present\r\n",ioexp_addr[i]);
        }
    }
    

    printf("Checking IOEXPANDER BUS 2:\r\n");
    
    for (i=0;i<(int)sizeof(ioexp_addr);i++)
    {
        present = checkPresence(XPAR_I2C_MAGNET_PORTS_AXI_IIC_IOEXP_2_BASEADDR,ioexp_addr[i]);
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
        res = PCA9535_SetPins0to11_Output(XPAR_I2C_MAGNET_PORTS_AXI_IIC_IOEXP_1_BASEADDR, ioexp_addr[i]);
        if (!res) {
            printf("Error setting pin as output\r\n");
            break;
        }
        res = PCA9535_SetPins0to11_Output(XPAR_I2C_MAGNET_PORTS_AXI_IIC_IOEXP_2_BASEADDR, ioexp_addr[i]);
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

    bool magnetVector[5];
    bool pciVector[5];
    printf("Checking AXI_A:\r\n");
    checkIICchannel(XPAR_I2C_MAGNET_PORTS_AXI_IIC_A_BASEADDR,magnetVector,pciVector);
    printf("Checking AXI_B:\r\n");
    checkIICchannel(XPAR_I2C_MAGNET_PORTS_AXI_IIC_B_BASEADDR,magnetVector,pciVector);
    printf("Checking AXI_C:\r\n");
    checkIICchannel(XPAR_I2C_MAGNET_PORTS_AXI_IIC_C_BASEADDR,magnetVector,pciVector);
    printf("Checking AXI_D:\r\n");
    checkIICchannel(XPAR_I2C_MAGNET_PORTS_AXI_IIC_D_BASEADDR,magnetVector,pciVector);
    printf("Checking AXI_E:\r\n");
    checkIICchannel(XPAR_I2C_MAGNET_PORTS_AXI_IIC_E_BASEADDR,magnetVector,pciVector);
    printf("Checking AXI_F:\r\n");
    checkIICchannel(XPAR_I2C_MAGNET_PORTS_AXI_IIC_F_BASEADDR,magnetVector,pciVector);    

}


uint8_t checkIICchannel(UINTPTR BaseAddress, bool *em_ok, bool *pcie_status)
{
    uint8_t i,j;    
    bool everythingOK;
    bool pciOk;
    printf("\tIICMUX...");
    if (checkPresence(BaseAddress,TCA9548ARGER_ADDR))
    {
        printf("\tfound!\r\n");
    }
    else {
        printf("\tnot found\r\n");
        return 0;
    }
    for (i=0;i<5;i++)
    {
        
        setIICmux(BaseAddress,1<<i);
        printf("\tPort %d...\r\n",i);

        everythingOK = true;
        pciOk = true;
        for (j=0;j<sizeof(ADDRESS_TO_CHECK);j++)
        {
            printf("\t\t%02X...",ADDRESS_TO_CHECK[j]);
            if (checkPresence(BaseAddress,ADDRESS_TO_CHECK[j]))
            {
                printf("\tfound!\r\n");
            }
            else{
                printf("\tnot found\r\n");
                everythingOK = false;
                if (ADDRESS_TO_CHECK[j] != ADS1115_ADDR)
                {
                    pciOk = false;
                }
            }
        }
        em_ok[i] = everythingOK;
        pcie_status[i] = pciOk;
    } 

    return 1;
}

void setIICmux(UINTPTR BaseAddress, uint8_t index)
{
    if (!I2C_SafeSend(BaseAddress, TCA9548ARGER_ADDR, &index, 1, XIIC_STOP))
    {
        xil_printf("Error setting IIC mux\r\n");
    }
}


bool checkPresence(UINTPTR BaseAddress, uint8_t deviceAddress)
{
    
    uint8_t dummy = 0x00;

    // Limpiar interrupciones pendientes
    XIic_WriteReg(BaseAddress, XIIC_IISR_OFFSET, 0xFFFFFFFF);

    //Enviamos una lectura
    //XIic_Send_custom(BaseAddress, deviceAddress, &dummy, 1, XIIC_STOP);
    if (XIic_Recv_custom(BaseAddress,deviceAddress,&dummy,1,XIIC_STOP) != 1)
    {
        xil_printf("error on XIic_Recv to address 0x%02X\r\n", deviceAddress);
        return false;
    }   
    
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

    if (XIic_Send_custom(BaseAddress, DevAddr, buf, 3, XIIC_STOP) != 3)
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
    if (XIic_Send_custom(BaseAddress, DevAddr, &Reg, 1, XIIC_REPEATED_START) != 1)
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
    if (XIic_Recv_custom(BaseAddress, DevAddr, buf, 2, XIIC_STOP) != 2)
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





#define HANG_LIMIT 2
static unsigned SendData(UINTPTR BaseAddress, u8 *BufferPtr,
			 unsigned ByteCount, u8 Option)
{
	u32 IntrStatus;
    TickType_t lastTimer = xTaskGetTickCount();
    unsigned originalByteCount = ByteCount;
	/*
	 * Send the specified number of bytes in the specified buffer by polling
	 * the device registers and blocking until complete
	 */
	while (ByteCount > 0) {
		/*
		 * Wait for the transmit to be empty before sending any more
		 * data by polling the interrupt status register
		 */
		while (1) {
			IntrStatus = XIic_ReadIisr(BaseAddress);

			if (IntrStatus & (XIIC_INTR_TX_ERROR_MASK |
					  XIIC_INTR_ARB_LOST_MASK |
					  XIIC_INTR_BNB_MASK)) {
				return ByteCount;
			}

			if (IntrStatus & XIIC_INTR_TX_EMPTY_MASK) {
				break;
			}
		}
		/* If there is more than one byte to send then put the
		 * next byte to send into the transmit FIFO
		 */
		if (ByteCount > 1) {
			XIic_WriteReg(BaseAddress,  XIIC_DTR_REG_OFFSET,
				 *BufferPtr++);
		}
		else {
			if (Option == XIIC_STOP) {
				/*
				 * If the Option is to release the bus after
				 * the last data byte, Set the stop Option
				 * before sending the last byte of data so
				 * that the stop Option will be generated
				 * immediately following the data. This is
				 * done by clearing the MSMS bit in the
				 * control register.
				 */
				XIic_WriteReg(BaseAddress,  XIIC_CR_REG_OFFSET,
					 XIIC_CR_ENABLE_DEVICE_MASK |
					 XIIC_CR_DIR_IS_TX_MASK);
			}

			/*
			 * Put the last byte to send in the transmit FIFO
			 */
			XIic_WriteReg(BaseAddress,  XIIC_DTR_REG_OFFSET,
				 *BufferPtr++);

			if (Option == XIIC_REPEATED_START) {
				XIic_ClearIisr(BaseAddress,
						XIIC_INTR_TX_EMPTY_MASK);
				/*
				 * Wait for the transmit to be empty before
				 * setting RSTA bit.
				 */
				while (1) {
					IntrStatus =
						XIic_ReadIisr(BaseAddress);
					if (IntrStatus &
						XIIC_INTR_TX_EMPTY_MASK) {
						/*
						 * RSTA bit should be set only
						 * when the FIFO is completely
						 * Empty.
						 */
						XIic_WriteReg(BaseAddress,
							 XIIC_CR_REG_OFFSET,
						   XIIC_CR_REPEATED_START_MASK |
						   XIIC_CR_ENABLE_DEVICE_MASK |
						   XIIC_CR_DIR_IS_TX_MASK |
						   XIIC_CR_MSMS_MASK);
						break;
					}
				}
			}
		}

		/*
		 * Clear the latched interrupt status register and this must be
		 * done after the transmit FIFO has been written to or it won't
		 * clear
		 */
		XIic_ClearIisr(BaseAddress, XIIC_INTR_TX_EMPTY_MASK);

		/*
		 * Update the byte count to reflect the byte sent and clear
		 * the latched interrupt status so it will be updated for the
		 * new state
		 */
		ByteCount--;
	}

	if (Option == XIIC_STOP) {
		/*
		 * If the Option is to release the bus after transmission of
		 * data, Wait for the bus to transition to not busy before
		 * returning, the IIC device cannot be disabled until this
		 * occurs. Note that this is different from a receive operation
		 * because the stop Option causes the bus to go not busy.
		 */
        lastTimer = xTaskGetTickCount();
		while (xTaskGetTickCount()-lastTimer<HANG_LIMIT) {
			if (XIic_ReadIisr(BaseAddress) &
				XIIC_INTR_BNB_MASK) {
				break;
			}
		}

        if (xTaskGetTickCount()-lastTimer >= HANG_LIMIT)
        {
            return originalByteCount;
        }
	}

	return ByteCount;
}


unsigned XIic_Send_custom(UINTPTR BaseAddress, u8 Address,
		   u8 *BufferPtr, unsigned ByteCount, u8 Option)
{
	unsigned RemainingByteCount;
	u32 ControlReg;
	volatile u32 StatusReg;

    
    TickType_t lastTimer = xTaskGetTickCount();
	/* Wait until I2C bus is freed, exit if timed out. */
	if (XIic_WaitBusFree(BaseAddress) != XST_SUCCESS) {
		return 0;
	}

	/* Check to see if already Master on the Bus.
	 * If Repeated Start bit is not set send Start bit by setting
	 * MSMS bit else Send the address.
	 */
	ControlReg = XIic_ReadReg(BaseAddress,  XIIC_CR_REG_OFFSET);
	if ((ControlReg & XIIC_CR_REPEATED_START_MASK) == 0) {
		/*
		 * Put the address into the FIFO to be sent and indicate
		 * that the operation to be performed on the bus is a
		 * write operation
		 */
		XIic_Send7BitAddress(BaseAddress, Address,
					XIIC_WRITE_OPERATION);
		/* Clear the latched interrupt status so that it will
		 * be updated with the new state when it changes, this
		 * must be done after the address is put in the FIFO
		 */
		XIic_ClearIisr(BaseAddress, XIIC_INTR_TX_EMPTY_MASK |
				XIIC_INTR_TX_ERROR_MASK |
				XIIC_INTR_ARB_LOST_MASK);

		/*
		 * MSMS must be set after putting data into transmit FIFO,
		 * indicate the direction is transmit, this device is master
		 * and enable the IIC device
		 */
		XIic_WriteReg(BaseAddress,  XIIC_CR_REG_OFFSET,
			 XIIC_CR_MSMS_MASK | XIIC_CR_DIR_IS_TX_MASK |
			 XIIC_CR_ENABLE_DEVICE_MASK);

		/*
		 * Clear the latched interrupt
		 * status for the bus not busy bit which must be done while
		 * the bus is busy
		 */
		StatusReg = XIic_ReadReg(BaseAddress,  XIIC_SR_REG_OFFSET);
        lastTimer = xTaskGetTickCount();
		while (((StatusReg & XIIC_SR_BUS_BUSY_MASK) == 0)&& (xTaskGetTickCount()-lastTimer<HANG_LIMIT)) {
			StatusReg = XIic_ReadReg(BaseAddress,
						  XIIC_SR_REG_OFFSET);
		}

		XIic_ClearIisr(BaseAddress, XIIC_INTR_BNB_MASK);
         if (xTaskGetTickCount()-lastTimer >= HANG_LIMIT)
        {
            xil_printf("CRITICAL ERROR IIC BUS");
            return 0;
        }
	}
	else {
		/*
		 * Already owns the Bus indicating that its a Repeated Start
		 * call. 7 bit slave address, send the address for a write
		 * operation and set the state to indicate the address has
		 * been sent.
		 */
		XIic_Send7BitAddress(BaseAddress, Address,
					XIIC_WRITE_OPERATION);
	}

	/* Send the specified data to the device on the IIC bus specified by the
	 * the address
	 */
	RemainingByteCount = SendData(BaseAddress, BufferPtr,
					ByteCount, Option);

	ControlReg = XIic_ReadReg(BaseAddress,  XIIC_CR_REG_OFFSET);
	if ((ControlReg & XIIC_CR_REPEATED_START_MASK) == 0) {
		/*
		 * The Transmission is completed, disable the IIC device if
		 * the Option is to release the Bus after transmission of data
		 * and return the number of bytes that was received. Only wait
		 * if master, if addressed as slave just reset to release
		 * the bus.
		 */
		if ((ControlReg & XIIC_CR_MSMS_MASK) != 0) {
			XIic_WriteReg(BaseAddress,  XIIC_CR_REG_OFFSET,
				 (ControlReg & ~XIIC_CR_MSMS_MASK));
		}

		if ((XIic_ReadReg(BaseAddress, XIIC_SR_REG_OFFSET) &
		    XIIC_SR_ADDR_AS_SLAVE_MASK) != 0) {
			XIic_WriteReg(BaseAddress,  XIIC_CR_REG_OFFSET, 0);
		}
		else {
			StatusReg = XIic_ReadReg(BaseAddress,
					XIIC_SR_REG_OFFSET);
            lastTimer = xTaskGetTickCount();
		    while (((StatusReg & XIIC_SR_BUS_BUSY_MASK) != 0)&& (xTaskGetTickCount()-lastTimer<HANG_LIMIT)) {
			    StatusReg = XIic_ReadReg(BaseAddress,
						  XIIC_SR_REG_OFFSET);
		    }
            if (xTaskGetTickCount()-lastTimer >= HANG_LIMIT)
            {
                xil_printf("CRITICAL ERROR IIC BUS");
                return 0;
            }
		}
	}

	return ByteCount - RemainingByteCount;
}


static unsigned RecvData(UINTPTR BaseAddress, u8 *BufferPtr,
			 unsigned ByteCount, u8 Option)
{
	u32 CntlReg;
	u32 IntrStatusMask;
	u32 IntrStatus;
    TickType_t lastTimer = xTaskGetTickCount();
    unsigned original_bytes = ByteCount;
	/* Attempt to receive the specified number of bytes on the IIC bus */

	while (ByteCount > 0) {
		/* Setup the mask to use for checking errors because when
		 * receiving one byte OR the last byte of a multibyte message an
		 * error naturally occurs when the no ack is done to tell the
		 * slave the last byte
		 */
		if (ByteCount == 1) {
			IntrStatusMask =
				XIIC_INTR_ARB_LOST_MASK | XIIC_INTR_BNB_MASK;
		} else {
			IntrStatusMask =
				XIIC_INTR_ARB_LOST_MASK |
				XIIC_INTR_TX_ERROR_MASK | XIIC_INTR_BNB_MASK;
		}

		/* Wait for the previous transmit and the 1st receive to
		 * complete by checking the interrupt status register of the
		 * IPIF
		 */
		while (1) {
			IntrStatus = XIic_ReadIisr(BaseAddress);
			if (IntrStatus & XIIC_INTR_RX_FULL_MASK) {
				break;
			}
			/* Check the transmit error after the receive full
			 * because when sending only one byte transmit error
			 * will occur because of the no ack to indicate the end
			 * of the data
			 */
			if (IntrStatus & IntrStatusMask) {
				return ByteCount;
			}
		}

		CntlReg = XIic_ReadReg(BaseAddress,  XIIC_CR_REG_OFFSET);

		/* Special conditions exist for the last two bytes so check for
		 * them. Note that the control register must be setup for these
		 * conditions before the data byte which was already received is
		 * read from the receive FIFO (while the bus is throttled
		 */
		if (ByteCount == 1) {
			if (Option == XIIC_STOP) {

				/* If the Option is to release the bus after the
				 * last data byte, it has already been read and
				 * no ack has been done, so clear MSMS while
				 * leaving the device enabled so it can get off
				 * the IIC bus appropriately with a stop
				 */
				XIic_WriteReg(BaseAddress,  XIIC_CR_REG_OFFSET,
					 XIIC_CR_ENABLE_DEVICE_MASK);
			}
		}

		/* Before the last byte is received, set NOACK to tell the slave
		 * IIC device that it is the end, this must be done before
		 * reading the byte from the FIFO
		 */
		if (ByteCount == 2) {
			/* Write control reg with NO ACK allowing last byte to
			 * have the No ack set to indicate to slave last byte
			 * read
			 */
			XIic_WriteReg(BaseAddress,  XIIC_CR_REG_OFFSET,
				 CntlReg | XIIC_CR_NO_ACK_MASK);
		}

		/* Read in data from the FIFO and unthrottle the bus such that
		 * the next byte is read from the IIC bus
		 */
		*BufferPtr++ = (u8) XIic_ReadReg(BaseAddress,
						  XIIC_DRR_REG_OFFSET);

		if ((ByteCount == 1) && (Option == XIIC_REPEATED_START)) {

			/* RSTA bit should be set only when the FIFO is
			 * completely Empty.
			 */
			XIic_WriteReg(BaseAddress,  XIIC_CR_REG_OFFSET,
				 XIIC_CR_ENABLE_DEVICE_MASK | XIIC_CR_MSMS_MASK
				 | XIIC_CR_REPEATED_START_MASK);

		}

		/* Clear the latched interrupt status so that it will be updated
		 * with the new state when it changes, this must be done after
		 * the receive register is read
		 */
		XIic_ClearIisr(BaseAddress, XIIC_INTR_RX_FULL_MASK |
				XIIC_INTR_TX_ERROR_MASK |
				XIIC_INTR_ARB_LOST_MASK);
		ByteCount--;
	}

	if (Option == XIIC_STOP) {

		/* If the Option is to release the bus after Reception of data,
		 * wait for the bus to transition to not busy before returning,
		 * the IIC device cannot be disabled until this occurs. It
		 * should transition as the MSMS bit of the control register was
		 * cleared before the last byte was read from the FIFO
		 */
        lastTimer = xTaskGetTickCount();
		while (1) {
			if (XIic_ReadIisr(BaseAddress) & XIIC_INTR_BNB_MASK) {
				break;
			}

            if (xTaskGetTickCount()-lastTimer>=HANG_LIMIT)
            {
                return original_bytes;
            }
		}
	}

	return ByteCount;
}


unsigned XIic_Recv_custom(UINTPTR BaseAddress, u8 Address,
			u8 *BufferPtr, unsigned ByteCount, u8 Option)
{
	u32 CntlReg;
	unsigned RemainingByteCount;
	volatile u32 StatusReg;
    TickType_t lastTimer = xTaskGetTickCount();
	/* Tx error is enabled in case the address (7 or 10) has no device to
	 * answer with Ack. When only one byte of data, must set NO ACK before
	 * address goes out therefore Tx error must not be enabled as it will go
	 * off immediately and the Rx full interrupt will be checked.  If full,
	 * then the one byte was received and the Tx error will be disabled
	 * without sending an error callback msg
	 */
	XIic_ClearIisr(BaseAddress,
			XIIC_INTR_RX_FULL_MASK | XIIC_INTR_TX_ERROR_MASK |
			XIIC_INTR_ARB_LOST_MASK);

	/* Set receive FIFO occupancy depth for 1 byte (zero based) */
	XIic_WriteReg(BaseAddress,  XIIC_RFD_REG_OFFSET, 0);


	/* Check to see if already Master on the Bus.
	 * If Repeated Start bit is not set send Start bit by setting MSMS bit
	 * else Send the address
	 */
	CntlReg = XIic_ReadReg(BaseAddress,  XIIC_CR_REG_OFFSET);
	if ((CntlReg & XIIC_CR_REPEATED_START_MASK) == 0) {
		/* 7 bit slave address, send the address for a read operation
		 * and set the state to indicate the address has been sent
		 */
		XIic_Send7BitAddress(BaseAddress, Address,
					XIIC_READ_OPERATION);


		/* MSMS gets set after putting data in FIFO. Start the master
		 * receive operation by setting CR Bits MSMS to Master, if the
		 * buffer is only one byte, then it should not be acknowledged
		 * to indicate the end of data
		 */
		CntlReg = XIIC_CR_MSMS_MASK | XIIC_CR_ENABLE_DEVICE_MASK;
		if (ByteCount == 1) {
			CntlReg |= XIIC_CR_NO_ACK_MASK;
		}

		/* Write out the control register to start receiving data and
		 * call the function to receive each byte into the buffer
		 */
		XIic_WriteReg(BaseAddress,  XIIC_CR_REG_OFFSET, CntlReg);

		/* Clear the latched interrupt status for the bus not busy bit
		 * which must be done while the bus is busy
		 */
		StatusReg = XIic_ReadReg(BaseAddress,  XIIC_SR_REG_OFFSET);
        lastTimer = xTaskGetTickCount();
		while (((StatusReg & XIIC_SR_BUS_BUSY_MASK) == 0) && (xTaskGetTickCount()-lastTimer<HANG_LIMIT)) {
			StatusReg = XIic_ReadReg(BaseAddress,
						  XIIC_SR_REG_OFFSET);
		}

		XIic_ClearIisr(BaseAddress, XIIC_INTR_BNB_MASK);
        if (xTaskGetTickCount()-lastTimer >= HANG_LIMIT)
        {
            xil_printf("CRITICAL ERROR IIC BUS");
            return 0;
        }
	} else {
	        /* Before writing 7bit slave address the Direction of Tx bit
		 * must be disabled
		 */
		CntlReg &= ~XIIC_CR_DIR_IS_TX_MASK;
		if (ByteCount == 1) {
			CntlReg |= XIIC_CR_NO_ACK_MASK;
		}
		XIic_WriteReg(BaseAddress,  XIIC_CR_REG_OFFSET, CntlReg);
		/* Already owns the Bus indicating that its a Repeated Start
		 * call. 7 bit slave address, send the address for a read
		 * operation and set the state to indicate the address has been
		 * sent
		 */
		XIic_Send7BitAddress(BaseAddress, Address,
					XIIC_READ_OPERATION);
	}
	/* Try to receive the data from the IIC bus */

	RemainingByteCount = RecvData(BaseAddress, BufferPtr,
				      ByteCount, Option);

	CntlReg = XIic_ReadReg(BaseAddress,  XIIC_CR_REG_OFFSET);
	if ((CntlReg & XIIC_CR_REPEATED_START_MASK) == 0) {
		/* The receive is complete, disable the IIC device if the Option
		 * is to release the Bus after Reception of data and return the
		 * number of bytes that was received
		 */
		XIic_WriteReg(BaseAddress,  XIIC_CR_REG_OFFSET, 0);
	}

	/* Wait until I2C bus is freed, exit if timed out. */
	if (XIic_WaitBusFree(BaseAddress) != XST_SUCCESS) {
		return 0;
	}

	/* Return the number of bytes that was received */
	return ByteCount - RemainingByteCount;
}
