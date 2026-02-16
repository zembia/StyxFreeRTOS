#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
/* Xilinx includes. */
#include "xil_printf.h"
#include "xparameters.h"
#include "xgpio.h"

#include "i2cTools.h"
#include "W5100.h"
#include "baseStructures.h"
#include "globaldefines.h"
#include <projdefs.h>
#include "xiic_l.h"
#include "xil_types.h"

const uint32_t PWM_ADDRESS[30]={  XPAR_PWM_MAGNETPWMCONTROLLER_0_BASEADDR,XPAR_PWM_MAGNETPWMCONTROLLER_1_BASEADDR,
                            XPAR_PWM_MAGNETPWMCONTROLLER_2_BASEADDR,XPAR_PWM_MAGNETPWMCONTROLLER_3_BASEADDR,
                            XPAR_PWM_MAGNETPWMCONTROLLER_4_BASEADDR,XPAR_PWM_MAGNETPWMCONTROLLER_5_BASEADDR,
                            XPAR_PWM_MAGNETPWMCONTROLLER_6_BASEADDR,XPAR_PWM_MAGNETPWMCONTROLLER_7_BASEADDR,
                            XPAR_PWM_MAGNETPWMCONTROLLER_8_BASEADDR,XPAR_PWM_MAGNETPWMCONTROLLER_9_BASEADDR,
                            XPAR_PWM_MAGNETPWMCONTROLLER_10_BASEADDR,XPAR_PWM_MAGNETPWMCONTROLLER_11_BASEADDR,
                            XPAR_PWM_MAGNETPWMCONTROLLER_12_BASEADDR,XPAR_PWM_MAGNETPWMCONTROLLER_13_BASEADDR,
                            XPAR_PWM_MAGNETPWMCONTROLLER_14_BASEADDR,XPAR_PWM_MAGNETPWMCONTROLLER_15_BASEADDR,
                            XPAR_PWM_MAGNETPWMCONTROLLER_16_BASEADDR,XPAR_PWM_MAGNETPWMCONTROLLER_17_BASEADDR,
                            XPAR_PWM_MAGNETPWMCONTROLLER_18_BASEADDR,XPAR_PWM_MAGNETPWMCONTROLLER_19_BASEADDR,
                            XPAR_PWM_MAGNETPWMCONTROLLER_20_BASEADDR,XPAR_PWM_MAGNETPWMCONTROLLER_21_BASEADDR,
                            XPAR_PWM_MAGNETPWMCONTROLLER_22_BASEADDR,XPAR_PWM_MAGNETPWMCONTROLLER_23_BASEADDR,
                            XPAR_PWM_MAGNETPWMCONTROLLER_24_BASEADDR,XPAR_PWM_MAGNETPWMCONTROLLER_25_BASEADDR,
                            XPAR_PWM_MAGNETPWMCONTROLLER_26_BASEADDR,XPAR_PWM_MAGNETPWMCONTROLLER_27_BASEADDR,
                            XPAR_PWM_MAGNETPWMCONTROLLER_28_BASEADDR,XPAR_PWM_MAGNETPWMCONTROLLER_29_BASEADDR
                            };

void setledPanelColor(uint8_t LED, uint8_t R, uint8_t G, uint8_t B);
void initReadADC(UINTPTR baseAddr, int8_t i2c_index, uint16_t channel);




void vTaskPwm(void *pvParameters);
void processCommand(char *);
void setPwmMode(uint8_t id, bool mode);
int16_t get13s(const uint8_t *buf, uint32_t index);
int16_t readADC(UINTPTR baseAddr, int8_t i2c_index);
void read_all_em_analog_inputs(void);
XGpio pwmEnableGpio;

__attribute__((section(".wave_buffers")))
uint8_t em_ctrl[NUM_EM][EM_VECTOR_SIZE];

__attribute__((section(".wave_buffers")))
uint8_t em_measure[NUM_EM][EM_MEASURE_VECTOR_SIZE];

__attribute__((section(".wave_buffers")))
uint8_t em_pwr[NUM_EM][EM_POWER_VECTOR_SIZE];

__attribute__((section(".wave_buffers")))
uint8_t em_temp[NUM_EM][EM_TEMP_VECTOR_SIZE];

__attribute__((section(".wave_buffers")))
uint8_t cfle_pwr[CFL_POWER_VECTOR_SIZE];

bool outputsStatus = false;

typedef struct
{
    bool outputsStatus;
} state_t;

state_t state;

operation_control_t op = OPERATION_CONTROL_INIT;
static task_manager_t configA;
static task_manager_t configB;
static task_manager_t configC;
static task_manager_t configD;
static task_manager_t configE;
static task_manager_t configF;
static task_manager_t configG;
static task_manager_t configH;
static task_manager_t configI;
static task_manager_t configJ;
static task_manager_t configK;
static task_manager_t configL;
static task_manager_t configM;
static task_manager_t configN;
// we need 1 task per I2C group
// static TaskHandle_t i2c_A_task, i2c_B_task, i2c_C_task, i2c_D_task, i2c_E_task, i2c_F_task;

/*
// Función de tarea de ejemplo
void vTaskHello(void *pvParameters)
{

    operation_control_t *op = (operation_control_t *)pvParameters;
    bool prevStatus = op->outputsStatus;
    xil_printf("Starting i2c task\r\n");

    // checkAllI2CDevices();
    vTaskDelay(pdMS_TO_TICKS(1));
    // disableOutputs();
    op->i2cReady = true;

    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(500));

        setIICmux(XPAR_I2C_MAGNET_PORTS_AXI_IIC_A_BASEADDR, 1 << 4);
        vTaskDelay(pdMS_TO_TICKS(10));

        // Build configuration (per datasheet Table 9)
        uint16_t config = ADS1115_OS_SINGLE |       // Start single conversion
                          0x50 |                    // Select input
                          ADS1115_PGA_4_096V |      // ±4.096V range
                          ADS1115_MODE_SINGLE |     // Single-shot mode
                          ADS1115_DR_128SPS |       // 128 SPS
                          ADS1115_COMP_QUE_DISABLE; // Disable comparator
        uint16_t flip_config = config << 8 | config >> 8;
        config = flip_config;

        // xil_printf("Writing config 0x%04X to start conversion\r\n", config);

        //-- START
        // Write Config Register to start conversion
        uint8_t write_buf[3];
        write_buf[0] = 0x01; // Config register
        write_buf[1] = 0xC3; // MSB of config
        write_buf[2] = 0x83; // LSB of config

        XIic_Send(XPAR_I2C_MAGNET_PORTS_AXI_IIC_A_BASEADDR, 0x48, write_buf, 3, XIIC_STOP);

        vTaskDelay(pdMS_TO_TICKS(20)); // Wait for conversion

        // Read Conversion Register (register 0x00)
        uint8_t reg = 0x00;
        uint8_t buf[2];

        XIic_Send(XPAR_I2C_MAGNET_PORTS_AXI_IIC_A_BASEADDR, 0x48, &reg, 1, XIIC_STOP);
        XIic_Recv(XPAR_I2C_MAGNET_PORTS_AXI_IIC_A_BASEADDR, 0x48, buf, 2, XIIC_STOP);

        int16_t result = ((int16_t)buf[0] << 8) | buf[1];
        xil_printf("Result = %d\r\n", result);
        //-- END
    }
}
*/


// Read magnetic and temperature of each of the 5 EMs corresponding to the current I2C Port
void vTaskMagnet(void *pvParameters)
{
    
    // Extract the configuration
    task_manager_t *cfg = (task_manager_t *)pvParameters;
    operation_control_t *op = cfg->op;
    uint32_t baseAddr = cfg->baseAddr;

    // Get task name (as you requested)
    char *pcTaskName = pcTaskGetName(NULL);

    bool prevStatus = op->outputsStatus;
     // 1. Wait for the previous task to finish
    if (cfg->waitSem != NULL) {
        xSemaphoreTake(cfg->waitSem, portMAX_DELAY);
    }

    xil_printf("Starting %s at address 0x%08x\r\n", pcTaskName, baseAddr);    
    
    // checkAllI2CDevices();

    // for (int i = 0; i < 5; i++) {        
    for (int ii = 0; ii < 1; ii++) {        
        int i = 4;
        initReadADC(baseAddr, i, ADS1115_MAGNETIC_FIELD);        
    }

    // 3. Signal the next task in the chain
    if (cfg->signalSem != NULL) {
        xSemaphoreGive(cfg->signalSem);
    }

    TickType_t lastWake = xTaskGetTickCount();
    
    
    uint8_t group_index;
    if (baseAddr == XPAR_I2C_MAGNET_PORTS_AXI_IIC_A_BASEADDR) {
        group_index = 0;
    } else if(baseAddr == XPAR_I2C_MAGNET_PORTS_AXI_IIC_B_BASEADDR) {
        group_index = 1;
    } else if(baseAddr == XPAR_I2C_MAGNET_PORTS_AXI_IIC_C_BASEADDR) {
        group_index = 2;
    } else if(baseAddr == XPAR_I2C_MAGNET_PORTS_AXI_IIC_D_BASEADDR) {
        group_index = 3;
    } else if(baseAddr == XPAR_I2C_MAGNET_PORTS_AXI_IIC_E_BASEADDR) {
        group_index = 4;
    } else if(baseAddr == XPAR_I2C_MAGNET_PORTS_AXI_IIC_F_BASEADDR) {
        group_index = 5;
    }

    /*
    while(1) {
        int i = 4;
        baseAddr = XPAR_I2C_MAGNET_PORTS_AXI_IIC_A_BASEADDR;    
        initReadADC(baseAddr, i, ADS1115_MAGNETIC_FIELD);
        vTaskDelay(pdMS_TO_TICKS(100));

        op->em[group_index*5+i].last_em_measure = readADC(baseAddr, i);
        vTaskDelay(pdMS_TO_TICKS(100));
        
        initReadADC(baseAddr, i, ADS1115_TEMP1); 
        vTaskDelay(pdMS_TO_TICKS(100));
        
        op->em[group_index*5+i].last_em_temp = readADC(baseAddr, i);
        vTaskDelay(pdMS_TO_TICKS(100));

        // Read EM power (INA)
        // Select baseAddr i2c_expander port
        setIICmux(baseAddr, 1 << i);
        vTaskDelay(pdMS_TO_TICKS(100));

        uint8_t buf[3] = {0x08, 0x00, 0x00}; // Should be 0x08
        // Set the pointer to voltage register            
        XIic_Send(baseAddr, 0x40, buf, 1, XIIC_STOP);
        vTaskDelay(pdMS_TO_TICKS(100));
        
        // Read voltage
        XIic_Recv(baseAddr, 0x40, buf, 3, XIIC_STOP); // Should be 3 bytes        

        // Save EM power
        op->em[group_index*5+i].last_em_pwr = ((((uint32_t)buf[0] << 16) | ((uint32_t)buf[1] << 8)  | ((uint32_t)buf[2]))*240)/1000000;        
        
        xil_printf("EM %d: mag=%d, temp=%d, pwr=%d\r\n", i, op->em[group_index*5+i].last_em_measure, op->em[group_index*5+i].last_em_temp, op->em[group_index*5+i].last_em_pwr);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    */

    while (1)
    {        
        if (group_index != 0) {
            vTaskDelay(pdMS_TO_TICKS(2000));
            continue;
        }
        // ======== STEP 1 ========
        // Read EM magnetic field (ADC)
        // for (int ii = 0; ii < 5; ii++) {
        for (int ii = 0; ii < 1; ii++) {
            int i = 4;
            op->em[group_index*5+i].last_em_measure = readADC(baseAddr, i);
        }

        // Start EM conversion of temp (ADC)
        // for (int ii = 0; ii < 5; ii++) {
        for (int ii = 0; ii < 1; ii++) {
            int i = 4;
            initReadADC(baseAddr, i, ADS1115_TEMP1); 
        }


        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(2)); 

        // ======== STEP 2 ========
        // Just once
        if (group_index == 0) {
            // Read CFLE power (ADC)
            // TODO
            // op->last_cfle_pwr = XXXX;
        }

        // Read EM power (INA)
        // for (int ii = 0; ii < 5; ii++) {
        for (int ii = 0; ii < 1; ii++) {
            int i = 4;
            // Read EM power (INA)
            // Select baseAddr i2c_expander port
            setIICmux(baseAddr, 1 << i);

            uint8_t buf[3] = {0x08, 0x00, 0x00}; // Should be 0x08
            // Set the pointer to voltage register            
            XIic_Send(baseAddr, 0x40, buf, 1, XIIC_STOP);
            
            // Read voltage
            XIic_Recv(baseAddr, 0x40, buf, 3, XIIC_STOP); // Should be 3 bytes

            // Save EM power
            op->em[group_index*5+i].last_em_pwr = ((((uint32_t)buf[0] << 16) | ((uint32_t)buf[1] << 8)  | ((uint32_t)buf[2]))*240)/1000000;
        }

        // Read EM temperature (ADC)
        for (int ii = 0; ii < 1; ii++) {
            int i = 4;
            op->em[group_index*5+i].last_em_temp = readADC(baseAddr, i);
        }
        
        // Start EM conversion of magnetic field (ADC)
        for (int ii = 0; ii < 5; ii++) {        
            int i = 4;
            initReadADC(baseAddr, i, ADS1115_MAGNETIC_FIELD); 
        }

        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(500));

    }
}


// Control Disable/Enable pin and Alert pin
void vTaskIoExp(void *pvParameters)
{
    // Extract the configuration
    task_manager_t *cfg = (task_manager_t *)pvParameters;
    operation_control_t *op = cfg->op;
    uint32_t baseAddr = cfg->baseAddr;

    // Get task name (as you requested)
    char *pcTaskName = pcTaskGetName(NULL);

    bool prevStatus = op->outputsStatus;

    // 1. Wait for the previous task to finish
    if (cfg->waitSem != NULL) {
        xSemaphoreTake(cfg->waitSem, portMAX_DELAY);
    }

    xil_printf("Starting %s at address 0x%08x\r\n", pcTaskName, baseAddr);

    configureOutputs(baseAddr);
    disableOutputs(baseAddr);
    
    // 3. Signal the next task in the chain
    if (cfg->signalSem != NULL) {
        xSemaphoreGive(cfg->signalSem);
    }

    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(200));

        if (op->outputsStatus == prevStatus)
        {
            continue;
        }

        prevStatus = op->outputsStatus;
        if (prevStatus)
        {
            enableOutputs(baseAddr);
            xil_printf("[%s] Outputs Enabled\r\n", pcTaskName);
        }
        else
        {
            //disableOutputs(baseAddr);
            xil_printf("[%s] Outputs Disabled\r\n", pcTaskName);
        }
    }
}

// void read_analog_inputs(void)
// {
//     int16_t an0, an1, an2;

//     // Example: ADS1115 is on mux channel 0 of AXI_IIC_A
//     if (ADS1115_ReadAN0_AN1_AN2(XPAR_I2C_MAGNET_PORTS_AXI_IIC_A_BASEADDR,
//                                 0,  // Mux channel 0
//                                 &an0, &an1, &an2)) {
//         // Convert to millivolts
//         int32_t mv0 = ADS1115_RawToMillivolts(an0, ADS1115_PGA_4_096V);
//         int32_t mv1 = ADS1115_RawToMillivolts(an1, ADS1115_PGA_4_096V);
//         int32_t mv2 = ADS1115_RawToMillivolts(an2, ADS1115_PGA_4_096V);

//         printf("AN0: %d mV (raw: %d)\r\n", mv0, an0);
//         printf("AN1: %d mV (raw: %d)\r\n", mv1, an1);
//         printf("AN2: %d mV (raw: %d)\r\n", mv2, an2);
//     } else {
//         printf("Failed to read analog inputs\r\n");
//     }
// }

// Or if you want to read from multiple electromagnets:
void read_all_em_analog_inputs(void)
{
    UINTPTR i2c_buses[] = {
        XPAR_I2C_MAGNET_PORTS_AXI_IIC_A_BASEADDR,
        XPAR_I2C_MAGNET_PORTS_AXI_IIC_B_BASEADDR,
        XPAR_I2C_MAGNET_PORTS_AXI_IIC_C_BASEADDR,
        XPAR_I2C_MAGNET_PORTS_AXI_IIC_D_BASEADDR,
        XPAR_I2C_MAGNET_PORTS_AXI_IIC_E_BASEADDR,
        XPAR_I2C_MAGNET_PORTS_AXI_IIC_F_BASEADDR};

    for (int bus = 0; bus < 6; bus++)
    {
        for (int mux_ch = 0; mux_ch < 5; mux_ch++)
        {
            int16_t an0, an1, an2;
            xil_printf("Bus %d, Mux %d:\r\n", bus, mux_ch);
            if (ADS1115_ReadAN0_AN1_AN2(i2c_buses[bus], mux_ch,
                                        &an0, &an1, &an2))
            {
                xil_printf("Bus %d, Mux %d: AN0=%d AN1=%d AN2=%d\r\n",
                           bus, mux_ch, an0, an1, an2);
            }
        }
    }
}

void vTaskEthernet(void *pvParameters)
{
    task_manager_t *cfg = (task_manager_t *)pvParameters;
    operation_control_t *op = cfg->op;
    uint8_t prevState = op->currentState;    
    uint32_t baseAddr = cfg->baseAddr;

    // Get task name (as you requested)
    char *pcTaskName = pcTaskGetName(NULL);

    bool prevStatus = op->outputsStatus;

    // 1. Wait for the previous task to finish
    if (cfg->waitSem != NULL) {
        xSemaphoreTake(cfg->waitSem, portMAX_DELAY);
    }

    xil_printf("Starting %s at address 0x%08x\r\n", pcTaskName, baseAddr);
    ethernetInit(op);
    
    // 3. Signal the next task in the chain
    if (cfg->signalSem != NULL) {
        xSemaphoreGive(cfg->signalSem);
    }

    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void vTaskLed(void *pvParameters)
{
    task_manager_t *cfg = (task_manager_t *)pvParameters;
    operation_control_t *op = cfg->op;
    uint32_t baseAddr = cfg->baseAddr;
    
    // Pattern Index and State tracking
    uint8_t prevState = 255; // Force initial update
    uint32_t patternIdx = 0;
    uint32_t patternLen = 0;
    bool *currentPattern = NULL;

    // Pattern definitions (matching your arrays)
    static bool slowBlink[]  = {0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1}; // 20 elements
    static bool fastBlink[]  = {0,0,1,1};                                 // 4 elements (modified for symmetry)
    static bool noBlinkOff[] = {0};                                       // 1 element
    static bool noBlinkOn[]  = {1};                                       // 1 element

    char *pcTaskName = pcTaskGetName(NULL);

    // 1. Wait for the previous task to finish (Sequential Startup)
    if (cfg->waitSem != NULL) {
        xSemaphoreTake(cfg->waitSem, portMAX_DELAY);
    }

    xil_printf("Starting %s at address 0x%08x\r\n", pcTaskName, baseAddr);    
    
    // 2. Signal the next task in the chain
    if (cfg->signalSem != NULL) {
        xSemaphoreGive(cfg->signalSem);
    }

    while (1)
    {
        // Run at 100ms resolution
        vTaskDelay(pdMS_TO_TICKS(100));        

        // Detect state change to reset pattern index
        if (op->currentState != prevState) {
            patternIdx = 0;
            prevState = op->currentState;
        }

        // 3. Select Pattern based on State
        switch (op->currentState)
        {
            case PLAY_STATE:
                currentPattern = noBlinkOn;
                patternLen = 1;
                break;

            case PAUSE_STATE:
                currentPattern = fastBlink;
                patternLen = 4;
                break;

            case STOP_STATE:
                // Check if the operation is "Done" or just stopped
                if (op->currentStage == STAGE_NONE && op->relativeTimeTick > 0) {
                    currentPattern = slowBlink;
                    patternLen = 20;
                } else {
                    currentPattern = noBlinkOff;
                    patternLen = 1;
                }
                break;
            
            default:
                currentPattern = noBlinkOff;
                patternLen = 1;
                break;
        }

        op->operationLed = currentPattern[patternIdx];

        // 5. Increment and wrap the index
        patternIdx++;
        if (patternIdx >= patternLen) {
            patternIdx = 0;
        }
    }
}

void vTaskMain(void *pvParameters)
{
    task_manager_t *cfg = (task_manager_t *)pvParameters;
    operation_control_t *op = cfg->op;
    uint8_t prevState = op->currentState;    
    uint32_t baseAddr = cfg->baseAddr;

    // Get task name (as you requested)
    char *pcTaskName = pcTaskGetName(NULL);

    bool prevStatus = op->outputsStatus;

    // 1. Wait for the previous task to finish
    if (cfg->waitSem != NULL) {
        xSemaphoreTake(cfg->waitSem, portMAX_DELAY);
    }

    xil_printf("Starting %s at address 0x%08x\r\n", pcTaskName, baseAddr);    

    uint32_t timeTick = 0;
    uint32_t prevRelativeTimeTick = 0;
    TickType_t lastWake = xTaskGetTickCount();

    while (1)
    {
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(50)); // TIME_PATCH: 100
        timeTick++;

        if (op->relativeTimeTick % 10 == 0 && op->relativeTimeTick > 0 && op->relativeTimeTick != prevRelativeTimeTick)
        {
            xil_printf("Time: %d, Stage: %d, State: %d\r\n", op->relativeTimeTick, op->currentStage, op->currentState);
        }
        prevRelativeTimeTick = op->relativeTimeTick;

        switch (op->currentState)
        {
        case STOP_STATE:
            if (op->cmd == CMD_PLAY)
            {
                xil_printf("Testing PLAY\r\n");
                xSemaphoreTake(op->mutex, portMAX_DELAY);                
                memset(em_measure, 0, sizeof(em_measure));
                memset(em_pwr, 0, sizeof(em_pwr));
                memset(em_temp, 0, sizeof(em_temp));
                memset(cfle_pwr, 0, sizeof(cfle_pwr));
                
                if (op->delayTime > 0)
                {
                    op->currentStage = STAGE_DELAY;
                }
                else if (op->operationTime > 0)
                {
                    op->currentStage = STAGE_OPERATION;
                }
                else
                {
                    op->currentStage = STAGE_NONE;
                }

                if (op->currentStage != STAGE_NONE)
                {
                    op->currentState = PLAY_STATE;
                }
                op->cmd = CMD_NONE;
                op->relativeTimeTick = 0;
                xSemaphoreGive(op->mutex);
            }
            break;

        case PLAY_STATE:
            xSemaphoreTake(op->mutex, portMAX_DELAY);
            op->relativeTimeTick = op->relativeTimeTick + 1;

            if (op->cmd == CMD_PAUSE)
            {
                xil_printf("Testing PAUSE\r\n");
                op->currentState = PAUSE_STATE;
            }
            else if (op->cmd == CMD_STOP)
            {
                xil_printf("Testing STOP\r\n");
                op->currentState = STOP_STATE;
            }

            if (op->currentStage == STAGE_DELAY)
            {
                if (op->relativeTimeTick >= op->delayTime)
                {
                    op->currentStage = STAGE_OPERATION;
                    op->relativeTimeTick = 0;
                }
            }
            else if (op->currentStage == STAGE_OPERATION)
            {
                if (op->relativeTimeTick >= op->operationTime)
                {
                    op->currentStage = STAGE_NONE;
                    //op->relativeTimeTick = 0;
                    op->currentState = DONE_STATE;
                }
            }
            op->cmd = CMD_NONE;
            xSemaphoreGive(op->mutex);
            break;

        case PAUSE_STATE:
            xSemaphoreTake(op->mutex, portMAX_DELAY);
            if (op->cmd == CMD_PLAY)
            {
                xil_printf("Testing PLAY\r\n");
                op->currentState = PLAY_STATE;
            }
            else if (op->cmd == CMD_STOP)
            {
                xil_printf("Testing STOP\r\n");
                op->currentState = STOP_STATE;
            }
            op->cmd = CMD_NONE;
            xSemaphoreGive(op->mutex);
            break;
        
        case DONE_STATE:
            xSemaphoreTake(op->mutex, portMAX_DELAY);
            
            if (op->cmd == CMD_STOP)
            {
                xil_printf("Testing STOP\r\n");
                op->currentState = STOP_STATE;
            }
            op->cmd = CMD_NONE;

            xSemaphoreGive(op->mutex);
            break;
        }

    }
}

void initReadADC(UINTPTR baseAddr, int8_t i2c_index, uint16_t channel) {
    
    setIICmux(baseAddr, 1 << i2c_index);    

    // Build configuration (per datasheet Table 9)
    uint16_t config = ADS1115_OS_SINGLE |       // Start single conversion
                        channel |
                        ADS1115_PGA_4_096V |      // ±4.096V range
                        ADS1115_MODE_SINGLE |     // Single-shot mode
                        ADS1115_DR_860SPS |       // 860 SPS
                        ADS1115_COMP_QUE_DISABLE; // Disable comparator
    uint16_t flip_config = config << 8 | config >> 8;
    // config = flip_config;

    // Write Config Register to start conversion
    uint8_t write_buf[3];
    write_buf[0] = 0x01; // Config register    
    write_buf[1] = config >> 8; // 0xC3; // MSB of config
    write_buf[2] = config ; // 0x83; // LSB of config

    XIic_Send(baseAddr, 0x48, write_buf, 3, XIIC_STOP);
}

int16_t readADC(UINTPTR baseAddr, int8_t i2c_index) {
    
    setIICmux(baseAddr, 1 << i2c_index);

    // Read Conversion Register (register 0x00)
    uint8_t reg = 0x00;    
    XIic_Send(baseAddr, 0x48, &reg, 1, XIIC_STOP);
    uint8_t buf[2];
    XIic_Recv(baseAddr, 0x48, buf, 2, XIIC_STOP);

    // 1. Combine bytes into a temporary 16-bit integer
    int16_t rawValue = (int16_t)((buf[0] << 8) | buf[1]);
    // xil_printf("buf bytes: %d %d\r\n", buf[0], buf[1]);

    // 2. Convert to voltage
    return (float)rawValue * 0.125f;
}

void vTaskPwm(void *pvParameters)
{

     // Extract the configuration
    task_manager_t *cfg = (task_manager_t *)pvParameters;
    operation_control_t *op = cfg->op;
    function_state_t prevState = op->currentState;    
    uint32_t baseAddr = cfg->baseAddr;

    // Get task name (as you requested)
    char *pcTaskName = pcTaskGetName(NULL);

    // 1. Wait for the previous task to finish
    if (cfg->waitSem != NULL) {
        xSemaphoreTake(cfg->waitSem, portMAX_DELAY);
    }

    xil_printf("Starting %s at address 0x%08x\r\n", pcTaskName, baseAddr); 
    
    // 3. Signal the next task in the chain
    if (cfg->signalSem != NULL) {
        xSemaphoreGive(cfg->signalSem);
    }


    //Forced Initialization
    disableAllDutyCycle();
    TickType_t lastWake = xTaskGetTickCount();
    

    while (true)
    {
        // ======== STEP 3 ========
        // apply PWM values
        switch (op->currentState)
        {
            case PLAY_STATE:
                // Resets the index just if a new process was started
                if (prevState != op->currentState){
                    if (prevState == STOP_STATE || prevState == DONE_STATE){
                        op->generalPlaybackIndex = 0;
                    }
                }

                // Don't do nothing if it's not the right stage.
                if (op->currentStage == STAGE_NONE || op->currentStage == STAGE_DELAY)
                {
                    // Disable all the outputs
                    op->outputsStatus = false;
                    disableAllDutyCycle();
                    for (int id = 0; id < 30; id++)
                    {
                        setDutyCycle(id, 0);
                    }

                    // Reset index of each electromagnet
                    for (int i = 0; i < NUM_EM; i++)
                    {
                        op->em[i].playbackIndex = 0;
                    }
                    break;
                }

                // It's in the right stage: STAGE_OPERATION
                op->outputsStatus = true;
                enableAllDutyCycle();
                // Write indexed value for each electromagnet
                for (int i = 0; i < NUM_EM; i++)
                {
                    // Get current PWM value from em_ctrl vector. considering that the variables are 13 bits length
                    int16_t pwmValueRaw = get13s(op->em[i].em_ctrl, op->em[i].playbackIndex);
                    float pwmValue = (float)pwmValueRaw / MAX_MAGNETIC_FIELD * 100.0f;
                    // Calculate percentage × 100 (e.g., 75.25% → 7525)
                    int32_t percent_x100 = (pwmValueRaw * 10000) / MAX_MAGNETIC_FIELD;
                    int32_t decimal_part = percent_x100 % 100;
                    if (decimal_part < 0)
                        decimal_part = -decimal_part; // Make absolute

                    if (i >= 2 && i < 7)
                    {
                        
                        xil_printf("%3d PWM %d: %d (%d.%02d%%) %d %d\r\n",
                                op->em[i].playbackIndex,
                                i,
                                pwmValueRaw,
                                percent_x100 / 100, // Integer part (keeps sign)
                                decimal_part,
                                op->generalPlaybackIndex,
                                op->em[i].last_em_pwr
                            );
                    }

                    setDutyCycle(i, pwmValue);
                }

                // Increase index of each electromagnet, if last item reached reset to 0
                for (int i = 0; i < NUM_EM; i++)
                {
                    op->em[i].playbackIndex = (op->em[i].playbackIndex + 1) % op->em[i].vector_length;
                }

                // Save the last value measured to buffers
                for (int j = 0; j < 6; j++) {
                    for (int i = 0; i < 5; i++){
                        put13s(op->em[j*5+i].em_measure, op->generalPlaybackIndex,op->em[j*5+i].last_em_measure);
                        put11s(op->em[j*5+i].em_temp, op->generalPlaybackIndex,op->em[j*5+i].last_em_temp);
                        put11s(op->em[j*5+i].em_pwr, op->generalPlaybackIndex,op->em[j*5+i].last_em_pwr);                
                    }
                }
                
                put10s(op->cfle_pwr, op->generalPlaybackIndex,op->last_cfle_pwr);                
                
                op->generalPlaybackIndex++;
                break;

            case STOP_STATE:
            case DONE_STATE:
                // Disable all the outputs
                op->outputsStatus = false;
                disableAllDutyCycle();
                for (int id = 0; id < 30; id++)
                {
                    setDutyCycle(id, 0);
                }

                // Reset index of each electromagnet
                for (int i = 0; i < NUM_EM; i++)
                {
                    op->em[i].playbackIndex = 0;
                }

                break;        

            case PAUSE_STATE:
                // Disable all the outputs
                op->outputsStatus = false;
                disableAllDutyCycle();
                for (int id = 0; id < 30; id++)
                {
                    setDutyCycle(id, 0);
                }

                break;
            default:
                break;
        }

        // DBG
        if (op->currentState != PLAY_STATE){
            for (int i = 0; i < NUM_EM; i++)
                {
                    // Get current PWM value from em_ctrl vector. considering that the variables are 13 bits length
                    int16_t pwmValueRaw = get13s(op->em[i].em_ctrl, op->em[i].playbackIndex);
                    float pwmValue = (float)pwmValueRaw / MAX_MAGNETIC_FIELD * 100.0f;
                    // Calculate percentage × 100 (e.g., 75.25% → 7525)
                    int32_t percent_x100 = (pwmValueRaw * 10000) / MAX_MAGNETIC_FIELD;
                    int32_t decimal_part = percent_x100 % 100;
                    if (decimal_part < 0)
                        decimal_part = -decimal_part; // Make absolute

                    if (i >= 2 && i < 7)
                    {
                        
                        xil_printf("%3d PWM %d: %d (%d.%02d%%) %d %d\r\n",
                                op->em[i].playbackIndex,
                                i,
                                pwmValueRaw,
                                percent_x100 / 100, // Integer part (keeps sign)
                                decimal_part,
                                op->generalPlaybackIndex,
                                op->em[i].last_em_pwr
                            );
                    }

                    setDutyCycle(i, pwmValue);
                }
        }

        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(4 + op->signalSamplePeriodMs / 2)); // TIME_PATCH (op->signalSamplePeriodMs)

        prevState = op->currentState;
    }
}

int main(void)
{
    // Initialize memory and variables
    xil_printf("Initialization...");
    op.mutex = xSemaphoreCreateMutex();
    op.currentState = STOP_STATE;
    op.outputsStatus = false;
    op.operationLed = false;
    op.warningLed = false;
    op.generalPlaybackIndex = 0;
    memset(op.availableEms, 0, sizeof(op.availableEms));
    op.availableEms[2] = true;
    memset(em_ctrl, 0, sizeof(em_ctrl));
    memset(em_measure, 0, sizeof(em_measure));
    memset(em_pwr, 0, sizeof(em_pwr));
    memset(em_temp, 0, sizeof(em_temp));
    memset(cfle_pwr, 0, sizeof(cfle_pwr));

    // Assign memory to op variable
    for (int i = 0; i < NUM_EM; i++)
    {
        // Assumes the vector has just 1 sample
        op.em[i].vector_length = 1;
        // Assign memory
        op.em[i].em_ctrl = em_ctrl[i];
        op.em[i].em_measure = em_measure[i];
        op.em[i].em_pwr = em_pwr[i];
        op.em[i].em_temp = em_temp[i];
    }
    op.cfle_pwr = cfle_pwr;
    xil_printf("done\r\n");

    int status;
    status = XGpio_Initialize(&pwmEnableGpio, XPAR_AXI_GPIO_0_BASEADDR);
    if (status != XST_SUCCESS)
    {
        perror("Gpio Initialization Failed\r\n");
    }


    SemaphoreHandle_t semAtoB = xSemaphoreCreateBinary();
    SemaphoreHandle_t semBtoC = xSemaphoreCreateBinary();
    SemaphoreHandle_t semCtoD = xSemaphoreCreateBinary();
    SemaphoreHandle_t semDtoF = xSemaphoreCreateBinary();
    SemaphoreHandle_t semFtoG = xSemaphoreCreateBinary();
    SemaphoreHandle_t semGtoH = xSemaphoreCreateBinary();
    SemaphoreHandle_t semHtoI = xSemaphoreCreateBinary();
    SemaphoreHandle_t semItoJ = xSemaphoreCreateBinary();
    SemaphoreHandle_t semJtoK = xSemaphoreCreateBinary();
    SemaphoreHandle_t semKtoL = xSemaphoreCreateBinary();
    SemaphoreHandle_t semLtoM = xSemaphoreCreateBinary();
    SemaphoreHandle_t semMtoN = xSemaphoreCreateBinary();


    // Define configs (static or global so they persist in memory)
    // Handles disable outputs and alert intpus  (the second one is not used)      
    configA.op = &op; 
    configA.baseAddr = XPAR_AXI_IIC_IOEXP_1_BASEADDR;
    configA.waitSem = NULL; 
    configA.signalSem = semAtoB;

    configB.op = &op; 
    configB.baseAddr = XPAR_AXI_IIC_IOEXP_2_BASEADDR;
    configB.waitSem = semAtoB; 
    configB.signalSem = semBtoC;

    configC.op = &op; 
    configC.baseAddr = XPAR_I2C_MAGNET_PORTS_AXI_IIC_A_BASEADDR;
    configC.waitSem = semBtoC; 
    configC.signalSem = semItoJ;//semCtoD;

    configD.op = &op; 
    configD.baseAddr = XPAR_I2C_MAGNET_PORTS_AXI_IIC_B_BASEADDR;
    configD.waitSem = semCtoD; 
    configD.signalSem = semDtoF;

    configE.op = &op; 
    configE.baseAddr = XPAR_I2C_MAGNET_PORTS_AXI_IIC_C_BASEADDR;
    configE.waitSem = semDtoF; 
    configE.signalSem = semFtoG;

    configF.op = &op; 
    configF.baseAddr = XPAR_I2C_MAGNET_PORTS_AXI_IIC_D_BASEADDR;
    configF.waitSem = semFtoG; 
    configF.signalSem = semGtoH;

    configG.op = &op; 
    configG.baseAddr = XPAR_I2C_MAGNET_PORTS_AXI_IIC_E_BASEADDR;
    configG.waitSem = semGtoH; 
    configG.signalSem = semHtoI;

    configH.op = &op; 
    configH.baseAddr = XPAR_I2C_MAGNET_PORTS_AXI_IIC_F_BASEADDR;
    configH.waitSem = semHtoI; 
    configH.signalSem = semItoJ;

    configI.op = &op; 
    configI.baseAddr = NULL;
    configI.waitSem = semItoJ; 
    configI.signalSem = semJtoK;
    
    configJ.op = &op; 
    configJ.baseAddr = NULL;
    configJ.waitSem = semJtoK; 
    configJ.signalSem = semKtoL;
    
    configK.op = &op; 
    configK.baseAddr = NULL;
    configK.waitSem = semKtoL; 
    configK.signalSem = semLtoM;
    
    configL.op = &op; 
    configL.baseAddr = NULL;
    configL.waitSem = semLtoM; 
    configL.signalSem = semMtoN;
    BaseType_t taskStatus;

    taskStatus = xTaskCreate(vTaskIoExp      , "IoExp1"  , 128  , &configA, tskIDLE_PRIORITY + 1, NULL);// Needs at least more thatn 64 
    if (taskStatus != pdPASS) {
        xil_printf("Failed to create vTaskIoExp1\r\n");
    }
    taskStatus = xTaskCreate(vTaskIoExp      , "IoExp2"  , 128  , &configB, tskIDLE_PRIORITY + 1, NULL);// Needs at least more thatn 64
    if (taskStatus != pdPASS) {
        xil_printf("Failed to create vTaskIoExp2\r\n");
    }
    taskStatus = xTaskCreate(vTaskMagnet     , "MagnetA" , 192  , &configC, tskIDLE_PRIORITY + 1, NULL);
    if (taskStatus != pdPASS) {
        xil_printf("Failed to create vTaskMagnetA\r\n");
    }
    taskStatus = xTaskCreate(vTaskMagnet     , "MagnetB" , 192  , &configD, tskIDLE_PRIORITY + 1, NULL);
    if (taskStatus != pdPASS) {
        xil_printf("Failed to create vTaskMagnetB\r\n");
    }
    taskStatus = xTaskCreate(vTaskMagnet     , "MagnetC" , 192  , &configE, tskIDLE_PRIORITY + 1, NULL);
    if (taskStatus != pdPASS) {
        xil_printf("Failed to create vTaskMagnetC\r\n");
    }
    taskStatus = xTaskCreate(vTaskMagnet     , "MagnetD" , 192  , &configF, tskIDLE_PRIORITY + 1, NULL);
    if (taskStatus != pdPASS) {
        xil_printf("Failed to create vTaskMagnetD\r\n");
    }
    taskStatus = xTaskCreate(vTaskMagnet     , "MagnetE" , 192  , &configG, tskIDLE_PRIORITY + 1, NULL);
    if (taskStatus != pdPASS) {
        xil_printf("Failed to create vTaskMagnetE\r\n");
    }
    taskStatus = xTaskCreate(vTaskMagnet     , "MagnetF" , 192  , &configH, tskIDLE_PRIORITY + 1, NULL);
    if (taskStatus != pdPASS) {
        xil_printf("Failed to create vTaskMagnetF\r\n");
    }
    taskStatus = xTaskCreate(vTaskPwm        , "Pwm"     , 2048  , &configI, tskIDLE_PRIORITY + 1, NULL);
    if (taskStatus != pdPASS) {
        xil_printf("Failed to create vTaskPwm\r\n");
    }
    taskStatus = xTaskCreate(vTaskLed        , "Led"     , 128  , &configJ, tskIDLE_PRIORITY + 1, NULL);
    if (taskStatus != pdPASS) {
        xil_printf("Failed to create vTaskLed\r\n");
    }   
    taskStatus = xTaskCreate(vTaskEthernet   , "Eth"     , 16384 , &configK, tskIDLE_PRIORITY + 2, NULL);
    if (taskStatus != pdPASS) {
        xil_printf("Failed to create vTaskEthernet\r\n");
    }
    taskStatus = xTaskCreate(vTaskMain       , "Main"    , 512  , &configL, tskIDLE_PRIORITY + 1, NULL);
    if (taskStatus != pdPASS) {
        xil_printf("Failed to create vTaskMain\r\n");
    }   


    for (int i=0;i<30;i++){
        if (i<10){
            setledPanelColor(i, 10, 0, 0); // Red
        } else if (i<20){
            setledPanelColor(i, 0, 10, 0); // Green
        } else {
            setledPanelColor(i, 0, 0, 10); // Blue
        }
    }

    vTaskStartScheduler();
    // nada después de vTaskStartScheduler se ejecuta a menos que falle
    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void setPwmFrequency(uint8_t id, float frequency)
{
    if (id > 29)
    {
        perror("ID not valid\n");
        return;
    }
    if ((frequency > PWM_BASE_CLK / 2) || (frequency < MIN_PWM_FREQ))
    {
        perror("frequency not valid\n");
        return;
    }

    uint32_t TIMING_REG;

    TIMING_REG = (uint32_t)((PWM_BASE_CLK / frequency - 1) + 0.5); // 0.5 rounds up

    Xil_Out32(PWM_ADDRESS[id], TIMING_REG);
    // To-Do: Disable PWM first
    printf("Set PWM N %u at %.2f kHz. TIMING REG: %u\n", id, frequency, TIMING_REG);
}

void setDutyCycle(uint8_t id, float dutyCycle)
{
    if (id > 29)
    {
        perror("ID not valid\n");
        return;
    }
    int32_t TIMING_REG;
    TIMING_REG = Xil_In32(PWM_ADDRESS[id]);

    int32_t DC_REG;
    DC_REG = TIMING_REG * dutyCycle / 100;
    Xil_Out32(PWM_ADDRESS[id] + 4, DC_REG);
    // xil_printf("Set PWM N %u at %.2f %%. DC REG: %d\n",id,dutyCycle,DC_REG);
}

void enableAllDutyCycle(void)
{
    XGpio_DiscreteWrite(&pwmEnableGpio, 1, 1);
}

void disableAllDutyCycle(void)
{
    XGpio_DiscreteWrite(&pwmEnableGpio, 1, 0);
}

void processCommand(char *cmd)
{
    int id;
    float freq, duty;

    if (sscanf(cmd, "SET %d %f %f", &id, &freq, &duty) == 3)
    {
        setPwmFrequency(id, freq);
        setDutyCycle(id, duty);
        xil_printf("ACK: ID=%d FREQ=%.2f DUTY=%.2f\n", id, freq, duty);
    }
    else if (strncmp(cmd, "EN", 2) == 0)
    {
        enableAllDutyCycle();
        xil_printf("ACK: ENABLED\n");
    }
    else if (strncmp(cmd, "DIS", 3) == 0)
    {
        disableAllDutyCycle();
        xil_printf("ACK: DISABLED\n");
    }
    else
    {
        xil_printf("ERR: Unknown command\n");
    }
}

void setPwmMode(uint8_t id, bool mode)
{
    Xil_Out32(PWM_ADDRESS[id] + 8, mode);
}

/**
 * @brief Get signed 13-bit value from packed buffer
 * @param buf: Packed data buffer (MSB-first)
 * @param index: Index of the value to extract (0-based)
 * @return Signed 13-bit value extended to 16-bit (-4096 to +4095)
 */
int16_t get13s(const uint8_t *buf, uint32_t index)
{
    uint32_t bit_offset = index * 13;
    uint32_t byte_offset = bit_offset >> 3; // / 8
    uint32_t bit_shift = bit_offset & 0x7;  // % 8

    // Read 4 bytes in BIG ENDIAN (MSB-first)
    // Using 4 bytes ensures we capture all 13 bits even when bit_shift is large
    uint32_t raw = ((uint32_t)buf[byte_offset] << 24) |
                   ((uint32_t)buf[byte_offset + 1] << 16) |
                   ((uint32_t)buf[byte_offset + 2] << 8) |
                   ((uint32_t)buf[byte_offset + 3]);

    // The 13-bit value starts at bit position `bit_shift` from the MSB (bit 31)
    // Shift right to align the 13 bits to LSB: (32 - 13 - bit_shift) = (19 - bit_shift)
    raw >>= (19 - bit_shift);

    // Mask to 13 bits
    raw &= 0x1FFF;

    // Sign extension: if bit 12 is set, extend it to bits 13-15
    if (raw & 0x1000)
    {
        raw |= 0xE000;
    }

    return (int16_t)raw;
}

/**
 * @brief Get signed 11-bit value from packed buffer
 */
int16_t get11s(const uint8_t *buf, uint32_t index)
{
    uint32_t bit_offset = index * 11;
    uint32_t byte_offset = bit_offset >> 3;
    uint32_t bit_shift = bit_offset & 0x7;

    uint32_t raw = ((uint32_t)buf[byte_offset] << 24) |
                   ((uint32_t)buf[byte_offset + 1] << 16) |
                   ((uint32_t)buf[byte_offset + 2] << 8) |
                   ((uint32_t)buf[byte_offset + 3]);

    raw >>= (21 - bit_shift);
    raw &= 0x7FF;

    // Sign extension: if bit 10 is set, extend to bits 11-15
    if (raw & 0x400) {
        raw |= 0xF800;
    }
    return (int16_t)raw;
}

/**
 * @brief Put signed 11-bit value into packed buffer
 */
static inline void put11s(uint8_t *buf, uint32_t index, int16_t value)
{
    uint32_t bit_offset  = index * 11;
    uint32_t byte_offset = bit_offset >> 3;
    uint32_t bit_shift   = bit_offset & 0x7;

    uint32_t raw = (uint16_t)value & 0x7FF;
    raw <<= (21 - bit_shift);

    uint32_t mask = (uint32_t)0x7FF << (21 - bit_shift);

    uint32_t cur = ((uint32_t)buf[byte_offset]     << 24) |
                   ((uint32_t)buf[byte_offset + 1] << 16) |
                   ((uint32_t)buf[byte_offset + 2] << 8)  |
                   ((uint32_t)buf[byte_offset + 3]);

    cur = (cur & ~mask) | raw;

    buf[byte_offset]     = (cur >> 24) & 0xFF;
    buf[byte_offset + 1] = (cur >> 16) & 0xFF;
    buf[byte_offset + 2] = (cur >> 8)  & 0xFF;
    buf[byte_offset + 3] = cur & 0xFF;
}

/**
 * @brief Get signed 10-bit value from packed buffer
 */
int16_t get10s(const uint8_t *buf, uint32_t index)
{
    uint32_t bit_offset = index * 10;
    uint32_t byte_offset = bit_offset >> 3;
    uint32_t bit_shift = bit_offset & 0x7;

    uint32_t raw = ((uint32_t)buf[byte_offset] << 24) |
                   ((uint32_t)buf[byte_offset + 1] << 16) |
                   ((uint32_t)buf[byte_offset + 2] << 8) |
                   ((uint32_t)buf[byte_offset + 3]);

    raw >>= (22 - bit_shift);
    raw &= 0x3FF;

    // Sign extension: if bit 9 is set, extend to bits 10-15
    if (raw & 0x200) {
        raw |= 0xFC00;
    }
    return (int16_t)raw;
}

/**
 * @brief Put signed 10-bit value into packed buffer
 */
static inline void put10s(uint8_t *buf, uint32_t index, int16_t value)
{
    uint32_t bit_offset  = index * 10;
    uint32_t byte_offset = bit_offset >> 3;
    uint32_t bit_shift   = bit_offset & 0x7;

    uint32_t raw = (uint16_t)value & 0x3FF;
    raw <<= (22 - bit_shift);

    uint32_t mask = (uint32_t)0x3FF << (22 - bit_shift);

    uint32_t cur = ((uint32_t)buf[byte_offset]     << 24) |
                   ((uint32_t)buf[byte_offset + 1] << 16) |
                   ((uint32_t)buf[byte_offset + 2] << 8)  |
                   ((uint32_t)buf[byte_offset + 3]);

    cur = (cur & ~mask) | raw;

    buf[byte_offset]     = (cur >> 24) & 0xFF;
    buf[byte_offset + 1] = (cur >> 16) & 0xFF;
    buf[byte_offset + 2] = (cur >> 8)  & 0xFF;
    buf[byte_offset + 3] = cur & 0xFF;
}



void setledPanelColor(uint8_t LED, uint8_t R, uint8_t G, uint8_t B)
{
    uint32_t dataOut;
    dataOut = LED<<24 | R<<16 | G<<8 | B;
    Xil_Out32(XPAR_WS2812B_DRIVER_0_BASEADDR, dataOut);
}