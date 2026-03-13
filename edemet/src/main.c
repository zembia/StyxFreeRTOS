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
#include <stdint.h>
#include "xiic_l.h"
#include "xil_types.h"
#include "math.h"
#include "ledbuttontools.h"

#include "magnetTools.h"
extern uint8_t globalretries;

const uint32_t PWM_ADDRESS[30]={    XPAR_PWM_MAGNETPWMCONTROLLER_12_BASEADDR,XPAR_PWM_MAGNETPWMCONTROLLER_9_BASEADDR,
                                    XPAR_PWM_MAGNETPWMCONTROLLER_6_BASEADDR,XPAR_PWM_MAGNETPWMCONTROLLER_3_BASEADDR,
                                    XPAR_PWM_MAGNETPWMCONTROLLER_2_BASEADDR,XPAR_PWM_MAGNETPWMCONTROLLER_28_BASEADDR,
                                    XPAR_PWM_MAGNETPWMCONTROLLER_25_BASEADDR,XPAR_PWM_MAGNETPWMCONTROLLER_22_BASEADDR,
                                    XPAR_PWM_MAGNETPWMCONTROLLER_19_BASEADDR,XPAR_PWM_MAGNETPWMCONTROLLER_16_BASEADDR,
                                    XPAR_PWM_MAGNETPWMCONTROLLER_13_BASEADDR,XPAR_PWM_MAGNETPWMCONTROLLER_10_BASEADDR,
                                    XPAR_PWM_MAGNETPWMCONTROLLER_7_BASEADDR,XPAR_PWM_MAGNETPWMCONTROLLER_4_BASEADDR,
                                    XPAR_PWM_MAGNETPWMCONTROLLER_1_BASEADDR,XPAR_PWM_MAGNETPWMCONTROLLER_27_BASEADDR,
                                    XPAR_PWM_MAGNETPWMCONTROLLER_24_BASEADDR,XPAR_PWM_MAGNETPWMCONTROLLER_21_BASEADDR,
                                    XPAR_PWM_MAGNETPWMCONTROLLER_18_BASEADDR,XPAR_PWM_MAGNETPWMCONTROLLER_15_BASEADDR,
                                    XPAR_PWM_MAGNETPWMCONTROLLER_29_BASEADDR,XPAR_PWM_MAGNETPWMCONTROLLER_26_BASEADDR,
                                    XPAR_PWM_MAGNETPWMCONTROLLER_23_BASEADDR,XPAR_PWM_MAGNETPWMCONTROLLER_20_BASEADDR,
                                    XPAR_PWM_MAGNETPWMCONTROLLER_17_BASEADDR,XPAR_PWM_MAGNETPWMCONTROLLER_14_BASEADDR,
                                    XPAR_PWM_MAGNETPWMCONTROLLER_11_BASEADDR,XPAR_PWM_MAGNETPWMCONTROLLER_8_BASEADDR,
                                    XPAR_PWM_MAGNETPWMCONTROLLER_5_BASEADDR,XPAR_PWM_MAGNETPWMCONTROLLER_0_BASEADDR
                            };                            
bool initReadADC(UINTPTR baseAddr, uint16_t channel);


QueueHandle_t xQueue[6] = {NULL};


bool mangetStatus[30]={0};
bool pcieStatus[30]={0};

void vTaskPwm(void *pvParameters);
int16_t get13s(const uint8_t *buf, uint32_t index);
int16_t readADC(UINTPTR baseAddr, bool *valid);
void read_all_em_analog_inputs(void);
XGpio pwmEnableGpio;

__attribute__((section(".wave_buffers")))
uint8_t em_ctrl[NUM_EM][EM_VECTOR_SIZE];

__attribute__((section(".wave_buffers")))
uint8_t em_measure[NUM_EM][EM_MEASURE_VECTOR_SIZE];

__attribute__((section(".wave_buffers")))
uint32_t em_ctrl_playback_trace[MAX_SAMPLES_PER_EM];

__attribute__((section(".wave_buffers")))
uint8_t em_pwr[NUM_EM][EM_POWER_VECTOR_SIZE];

__attribute__((section(".wave_buffers")))
uint8_t em_temp[NUM_EM][EM_TEMP_VECTOR_SIZE];

__attribute__((section(".wave_buffers")))
uint8_t cfle_pwr[CFL_POWER_VECTOR_SIZE];


__attribute__((section(".wave_buffers")))
uint8_t pause_vector[PAUSE_VECTOR_SIZE];

bool outputsStatus = false;

typedef struct
{
    bool outputsStatus;
} state_t;

state_t state;

volatile operation_control_t op = OPERATION_CONTROL_INIT;
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


typedef struct
{
    float gain;
    float P;
    float Q;
    float R;
} gain_kalman_t;

gain_kalman_t gain_kalman[NUM_EM];  
float lastEmControl[NUM_EM]={0};
float newSetPoint[NUM_EM]={0};



void gain_kalman_update(gain_kalman_t *kf,
                        float setPoint,
                        float measuredField)
{   
    //return;
    static uint8_t counter = 0;
    float realsetpoint = setPoint/kf->gain;

    if (fabs(realsetpoint) < 20)
        return;

    float alpha = 0.001f;   // slow adaptation

    static float num = 0;
    static float den = 0;

    float predictedMeasurement = realsetpoint;
    float error = predictedMeasurement-measuredField;
    float tempGain = kf->gain + alpha * error/realsetpoint; 

    if (tempGain < 0.01) tempGain = 0.01; // prevent too low gain
    if (tempGain > 0.1) tempGain = 0.1; // prevent too high gain
    kf->gain = tempGain;

    /*if (counter++==32)
    {
        counter = 0;
        printf("gain for EM: %f, setpoint: %f, measured: %f\r\n", kf->gain, realsetpoint, measuredField);
    }*/

}



// Read magnetic and temperature of each of the 5 EMs corresponding to the current I2C Port
void vTaskMagnet(void *pvParameters)
{        
    int16_t tempAdcValue;
    bool valid1;
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

    TickType_t lastWake = xTaskGetTickCount();    

    uint8_t group_index = 0xFF;
    if (baseAddr == XPAR_I2C_MAGNET_PORTS_AXI_IIC_A_BASEADDR) {
        group_index = 0;
    } else if(baseAddr == XPAR_I2C_MAGNET_PORTS_AXI_IIC_B_BASEADDR) {
        //return;
        group_index = 1;
    } else if(baseAddr == XPAR_I2C_MAGNET_PORTS_AXI_IIC_C_BASEADDR ) {
        //return;
        group_index = 2;
    } else if(baseAddr == XPAR_I2C_MAGNET_PORTS_AXI_IIC_D_BASEADDR) {
        //return;
        group_index = 3;
    } else if(baseAddr == XPAR_I2C_MAGNET_PORTS_AXI_IIC_E_BASEADDR) {
        //return;
        group_index = 4;
    } else if(baseAddr == XPAR_I2C_MAGNET_PORTS_AXI_IIC_F_BASEADDR) {
        //return;
        group_index = 5;
    }

    if (group_index > 5) {
        xil_printf("[ERROR] %s invalid baseAddr 0x%08x\r\n", pcTaskName, baseAddr);
        vTaskDelete(NULL);
    }
    I2C_ResetBus(baseAddr);
    vTaskDelay(pdMS_TO_TICKS(10));
    //XIic_IntrGlobalDisable(baseAddr);

  
    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(2)); 
    TickType_t lastWake2 = xTaskGetTickCount();
    TickType_t lastADC_READ=xTaskGetTickCount();
    

    uint32_t startTime;
    uint32_t endTime;
    uint32_t difftime1 = 0, difftime2 = 0, difftime3 = 0;

    uint16_t counter=0;
    bool once;
    bool magnetReadStartedOk[5] = {true, true, true, true, true};
    bool TempReadStartedOk[5] = {true, true, true, true, true};
    //we check the presence of every magnet
    if (cfg->signalSem != NULL) {
        xSemaphoreGive(cfg->signalSem);
    }
    checkIICchannel(baseAddr,&mangetStatus[group_index*5],&pcieStatus[group_index*5]);
    // 3. Signal the next task in the chain

    for (int i=0;i<5;i++)
    {
        if (mangetStatus[group_index*5+i] == false)
        {
            if (pcieStatus[group_index*5+i] == true)
            {
                setledPanelColor(group_index*5+i, MAX_PANEL_BRIGHTNESS, MAX_PANEL_BRIGHTNESS*0.4, 0);
            }
            else
            {
                setledPanelColor(group_index*5+i, MAX_PANEL_BRIGHTNESS_RED, 0, 0);
            }
        }
        else
        {
            setledPanelColor(group_index*5+i, 0, MAX_PANEL_BRIGHTNESS, 0);
            setIICmux(baseAddr, 1 << i);
            initINA(baseAddr);
            vTaskDelay(pdMS_TO_TICKS(1000));
            uint16_t tempData;
            INA_readReg(baseAddr,0x05,&tempData,2);
            xil_printf("Read Voltaje: %X",tempData);

        }
    }

    while (1)
    {   
        vTaskDelayUntil(&lastWake2, pdMS_TO_TICKS(10)); 
        //xil_printf("[DBG] Loop start\r\n");
        startTime = xTaskGetTickCount();
        // Start EM conversion of magnetic field (ADC)
        //xil_printf("[DBG] Starting mag conversions\r\n");
        once = true;
        for (int ii=0; ii < 5; ii++) 
        {
            if (mangetStatus[group_index*5+ii])
            {                
                if (setIICmux(baseAddr, 1 << ii) == 0)
                {
                    magnetReadStartedOk[ii] = false;
                    continue;
                }
                //Reading temperature converted in last loop

                if (once && ((xTaskGetTickCount() - lastADC_READ) <= pdMS_TO_TICKS(2)))
                {
                    vTaskDelay(pdMS_TO_TICKS(2));
                }
                //only read if the previous conversion was started ok, otherwise we would be reading magnet data instead of temp.
                if ((TempReadStartedOk[ii] == true) && (magnetReadStartedOk[ii]))
                {
                    tempAdcValue= readADC(baseAddr, &valid1);
                    if (valid1)
                    {                    
                        op->em[group_index*5+ii].last_em_temp = interpretTempearture(tempAdcValue);
                        updateTemperaturePL(group_index*5+ii, op->em[group_index*5+ii].last_em_temp);//Update temp in hardware
                    }
                }
            

                //initing magnetic field conversion
                magnetReadStartedOk[ii] = initReadADC(baseAddr, ADS1115_MAGNETIC_FIELD);
                if (once)
                {
                    lastADC_READ = xTaskGetTickCount();
                    once = false;
                } 
                
               /* 
                //Reading INA
                uint8_t buf[3] = {0x08, 0x00, 0x00};


                if (!INA_readReg(baseAddr,0x08,buf,3))
                {
                    xil_printf("[ERROR] Failed to read INA data for EM %d, group %d\r\n", ii, group_index);
                    //op->em[group_index*5+ii].last_em_pwr = 0;
                    continue;
                }

                // Save EM power
                op->em[group_index*5+ii].last_em_pwr = ((((uint32_t)buf[0] << 16) | ((uint32_t)buf[1] << 8)  | ((uint32_t)buf[2]))*240)/250000;*/
                
                
            }/*
            else if (pcieStatus[group_index*5+ii])
            {
               //Reading INA
                uint8_t buf[3] = {0x08, 0x00, 0x00};


                if (!INA_readReg(baseAddr,0x08,buf,3))
                {
                    xil_printf("[ERROR] Failed to read INA data for EM %d, group %d\r\n", ii, group_index);
                    //op->em[group_index*5+ii].last_em_pwr = 0;
                    continue;
                }

                // Save EM power
                op->em[group_index*5+ii].last_em_pwr = ((((uint32_t)buf[0] << 16) | ((uint32_t)buf[1] << 8)  | ((uint32_t)buf[2]))*240)/250000;
                
                op->em[group_index*5+ii].last_em_temp = 0;
            }*/
            else
            {
                op->em[group_index*5+ii].last_em_pwr = 0;
                op->em[group_index*5+ii].last_em_temp = 0;
            }
        }
        
        
        // Read EM magnetic field (ADC) and setup temp read for next loop    
        once = true;
        for (int ii=0; ii < 5; ii++) {
            if (mangetStatus[group_index*5+ii])
            {
                if (magnetReadStartedOk[ii] == true)
                {                
                    if (setIICmux(baseAddr, 1 << ii) == 0)
                    {
                        TempReadStartedOk[ii] = false;
                        continue;
                    }

                    if (once && ((xTaskGetTickCount() - lastADC_READ) <= pdMS_TO_TICKS(2)))
                    {
                        vTaskDelay(pdMS_TO_TICKS(2));
                    }
                    tempAdcValue= readADC(baseAddr, &valid1);
                    if (valid1)
                    {
                                    
                        gain_kalman_update(&gain_kalman[group_index*5+ii], lastEmControl[group_index*5+ii], op->em[group_index*5+ii].last_em_measure);
                        //delayedlastEmControl[group_index*5+ii] = lastEmControl[group_index*5+ii];
                        //delayedlastEmControl2[group_index*5+ii] = delayedlastEmControl[group_index*5+ii];
                        op->em[group_index*5+ii].last_em_measure = interpretMagneticField(tempAdcValue);                                            
                        
                    }
                }

                TempReadStartedOk[ii] = initReadADC(baseAddr, ADS1115_TEMP1);
                if (once)
                {
                    lastADC_READ = xTaskGetTickCount();
                    once= false;
                }
            }
            else
            {
                op->em[group_index*5+ii].last_em_measure = 0;
            }
        }


        if (++counter >=1000)
        {
           if (group_index==0)
           {
                xil_printf("------\r\n");
                for (int i=0; i < 5; i++) {
                    
                    xil_printf("%d: %d // %d // %d\r\n", group_index, op->em[group_index*5+i].last_em_temp, op->em[group_index*5+i].last_em_pwr, op->em[group_index*5+i].last_em_measure);
                }
            }
            counter=0;
        }

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
    //disableOutputs(baseAddr);
    enableOutputs(baseAddr);
    
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
                currentPattern = noBlinkOff;
                patternLen = 1;
                break;
            case DONE_STATE:
                    currentPattern = slowBlink;
                    patternLen = 20;
                break;
            default:
                currentPattern = noBlinkOff;
                patternLen = 1;
                break;
        }

        op->operationLed = currentPattern[patternIdx];
        setLedInd1State(op->operationLed);
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
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(100)); // TIME_PATCH: 100
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

bool initReadADC(UINTPTR baseAddr, uint16_t channel) {    
    // Build configuration (per datasheet Table 9)
    uint16_t config = ADS1115_OS_SINGLE |       // Start single conversion
                        channel |
                        ADS1115_PGA_6_144V |      // ±4.096V range
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

    if (!I2C_SafeSend(baseAddr, 0x48, write_buf, 3, XIIC_STOP)) {
        xil_printf("[ERROR] Failed to send ADC config\r\n");
        return 0;
    }
    uint16_t recvBuf;
    if (!I2C_SafeRecv(baseAddr, 0x48, &recvBuf, 2, XIIC_STOP)) {
        xil_printf("[ERROR] Failed to read ADC config\r\n");
        return 0;
    }
    uint16_t flip_recvBuf = recvBuf << 8 | recvBuf >> 8;


    if (!I2C_SafeRecv(baseAddr, 0x48, &recvBuf, 2, XIIC_STOP)) {
        xil_printf("[ERROR] Failed to read ADC config\r\n");
        return 0;
    }

    uint16_t flip_recvBuf2 =  recvBuf << 8 | recvBuf >> 8;



    bool conversionStarted = (flip_recvBuf&0x8000) == 0;
    if (conversionStarted == false)
    {
        xil_printf("[ERROR] conversion did not start\r\n");
        return 0;
    }

    if ((flip_recvBuf & 0x7000) != (config & 0x7000))
    {
        xil_printf("[ERROR] ADC channel not changed succesfully\r\n");
        return 0;
    }

    if (flip_recvBuf != flip_recvBuf)
    {
        xil_printf("[ERROR] glitch read ADC config\r\n");
        return 0;
    }

    return 1;
}

int16_t readADC(UINTPTR baseAddr, bool *valid) {
    
    *valid = false;
    // Read Conversion Register (register 0x00)
    uint8_t reg = 0x00;    
    if (!I2C_SafeSend(baseAddr, 0x48, &reg, 1, XIIC_STOP)) {
        xil_printf("[ERROR] Failed to set ADC register pointer\r\n");
        return 0;
    }
    
    uint8_t buf1[2];
    uint8_t buf2[2];
    uint8_t buf3[2];
    if (!I2C_SafeRecv(baseAddr, 0x48, buf1, 2, XIIC_STOP)) {
        xil_printf("[ERROR] Failed to read ADC data #1\r\n");
        return 0;
    }

    if (!I2C_SafeRecv(baseAddr, 0x48, buf2, 2, XIIC_STOP)) {
        xil_printf("[ERROR] Failed to read ADC data #2\r\n");
        return 0;
    }
/*
    if (!I2C_SafeRecv(baseAddr, 0x48, buf3, 2, XIIC_STOP)) {
        xil_printf("[ERROR] Failed to read ADC data #2\r\n");
        return 0;
    }*/

    int16_t rawValue1 = (int16_t)((buf1[0] << 8) | buf1[1]);
    int16_t rawValue2 = (int16_t)((buf2[0] << 8) | buf2[1]);
   // int16_t rawValue3 = (int16_t)((buf3[0] << 8) | buf3[1]);

    if (rawValue1 != rawValue2) {
        xil_printf("[WARNING] ADC glitch filtered: %d vs %d\r\n", rawValue1,
                   rawValue2);
        return 0;
    }

   /* if (rawValue1 != rawValue3) {
        xil_printf("[WARNING] ADC glitch filtered: %d vs %d\r\n", rawValue1,
                   rawValue3);
        return 0;
    }*/

    *valid = true;
    // 2. Convert to voltage
    return (float)rawValue1 * 0.1875;
}

void vTaskPwm(void *pvParameters)
{

     // Extract the configuration
    task_manager_t *cfg = (task_manager_t *)pvParameters;
    operation_control_t *op = cfg->op;
    function_state_t prevState = op->currentState;    
    function_state_t currentState = op->currentState;    
    uint32_t baseAddr = cfg->baseAddr;
    uint32_t playbacksum;

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
    //enableAllDutyCycle();
    for (int i=0;i<30;i++)
    {
        //setPwmMode(i,0);
        setPwmFrequency(i,0.25); // 1 kHz
        //setDutyCycle(i,50);
    }
   // vTaskDelay(portMAX_DELAY);
    disableAllDutyCycle();

    TickType_t lastWake = xTaskGetTickCount();
    /*vTaskDelay(pdMS_TO_TICKS(5000));
    for (int i=0;i<30;i++)
    {
        if (mangetStatus[i])
        {
            calibrateMagnet(i,op);
        }
    }*/

    while (true)
    {
        // ======== STEP 3 ========
        // apply PWM values

        prevState = currentState;
        currentState = op->currentState;
        switch (currentState)
        {
            case PLAY_STATE:
                globalretries = 1;
                // Resets the index just if a new process was started
                if (prevState != currentState){
                    if (prevState == STOP_STATE || prevState == DONE_STATE){
                        op->generalPlaybackIndex = 0;
                        op->pausePlaybackIndex = 0;
                        op->initialTimeTick = xTaskGetTickCount();
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
                    //float pwmValue = (float)pwmValueRaw / MAX_MAGNETIC_FIELD * 100.0f;
                    
                    float pwmValue = (float)pwmValueRaw *gain_kalman[i].gain;                    
                    lastEmControl[i] = pwmValue;


                    // Calculate percentage × 100 (e.g., 75.25% → 7525)
                    int32_t percent_x100 = (pwmValueRaw * 10000) / MAX_MAGNETIC_FIELD;
                    int32_t decimal_part = percent_x100 % 100;
                    if (decimal_part < 0)
                        decimal_part = -decimal_part; // Make absolute

                    if (i >= 2 && i < 1)//7)
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
                    if (mangetStatus[i])
                    {
                        if (pwmValue>0)
                        {
                            setledPanelColor(i,0,0,pwmValue);
                        }
                        else
                        {
                            setledPanelColor(i,0,0,-pwmValue);
                        }
                    }
                }

                // Increase index of each electromagnet, if last item reached reset to 0
                for (int i = 0; i < NUM_EM; i++)
                {
                    op->em[i].playbackIndex = (op->em[i].playbackIndex + 1) % op->em[i].vector_length;
                }

                playbacksum = op->generalPlaybackIndex+op->pausePlaybackIndex;

                if (playbacksum < MAX_SAMPLES_PER_EM)
                {
                    em_ctrl_playback_trace[playbacksum] = op->generalPlaybackIndex;
                }

                // Save the last value measured to buffers. This could be recicled in the previous loop
                for (int j = 0; j < 6; j++) {
                    for (int i = 0; i < 5; i++){
                        put13s(op->em[j*5+i].em_measure, playbacksum,op->em[j*5+i].last_em_measure);
                        put11s(op->em[j*5+i].em_temp, playbacksum,op->em[j*5+i].last_em_temp);
                        put11s(op->em[j*5+i].em_pwr, playbacksum,op->em[j*5+i].last_em_pwr);                
                    }
                }
                pause_vector_write_bit(playbacksum,0);
                put10s(op->cfle_pwr, playbacksum,op->last_cfle_pwr);                
                
                op->generalPlaybackIndex++;
                break;

            case STOP_STATE:
            case DONE_STATE:
                globalretries = 3;
                // Disable all the outputs
                op->outputsStatus = false;
                disableAllDutyCycle();
                for (int id = 0; id < 30; id++)
                {
                    setDutyCycle(id, 0);
                    lastEmControl[id] = 0;
                    if (mangetStatus[id])
                    {
                        setledPanelColor(id,0,MAX_PANEL_BRIGHTNESS,0);
                    }
                    else
                    {
                        if (pcieStatus[id])
                        {
                            setledPanelColor(id,MAX_PANEL_BRIGHTNESS,MAX_PANEL_BRIGHTNESS*0.4,0);
                        }
                        else
                        {
                            setledPanelColor(id,MAX_PANEL_BRIGHTNESS_RED,0,0);
                        }
                    }
                }

                // Reset index of each electromagnet
                for (int i = 0; i < NUM_EM; i++)
                {
                    op->em[i].playbackIndex = 0;
                }

                break;        

            case PAUSE_STATE:
                globalretries = 3;
                // Disable all the outputs
                op->outputsStatus = false;
                disableAllDutyCycle();
                playbacksum = op->generalPlaybackIndex + op->pausePlaybackIndex;

                if (playbacksum < MAX_SAMPLES_PER_EM)
                {
                    em_ctrl_playback_trace[playbacksum] = op->generalPlaybackIndex;
                }

                for (int id = 0; id < 30; id++)
                {
                    lastEmControl[id] = 0;
                    setDutyCycle(id, 0);
                    if (mangetStatus[id])
                    {
                        setledPanelColor(id,0,MAX_PANEL_BRIGHTNESS,0);
                    }
                    else
                    {
                        if (pcieStatus[id])
                        {
                            setledPanelColor(id,MAX_PANEL_BRIGHTNESS,MAX_PANEL_BRIGHTNESS*0.4,0);
                        }
                        else
                        {
                            setledPanelColor(id,MAX_PANEL_BRIGHTNESS_RED,0,0);
                        }
                    }
                    //don't increase playback index of magnets when paused
                    //op->em[id].playbackIndex = (op->em[id].playbackIndex + 1) % op->em[id].vector_length;
                    put13s(op->em[id].em_measure, playbacksum,op->em[id].last_em_measure);
                    put11s(op->em[id].em_temp, playbacksum,op->em[id].last_em_temp);
                    put11s(op->em[id].em_pwr, playbacksum,op->em[id].last_em_pwr);              
                }                
                put10s(op->cfle_pwr, playbacksum,op->last_cfle_pwr);
                pause_vector_write_bit(playbacksum,1);
                
                op->pausePlaybackIndex++;
                break;
            default:
                break;
        }

        // DBG
        /*
        if (currentState != PLAY_STATE){
            for (int i = 0; i < NUM_EM; i++)
                {
                    // Get current PWM value from em_ctrl vector. considering that the variables are 13 bits length
                    int16_t pwmValueRaw = get13s(op->em[i].em_ctrl, op->em[i].playbackIndex);
                    //float pwmValue = (float)pwmValueRaw / MAX_MAGNETIC_FIELD * 100.0f;
                    float pwmValue = (float)pwmValueRaw *gain_kalman[i].gain;
                    // Calculate percentage × 100 (e.g., 75.25% → 7525)
                    int32_t percent_x100 = (pwmValueRaw * 10000) / MAX_MAGNETIC_FIELD;
                    int32_t decimal_part = percent_x100 % 100;
                    if (decimal_part < 0)
                        decimal_part = -decimal_part; // Make absolute

                    if (i >= 2 && i < 1)//7)
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
                        //xil_printf("tick count: %u\r\n",xTaskGetTickCount());       
                    }
                    lastEmControl[i] = pwmValueRaw;
                    setDutyCycle(i, pwmValue);
                }
        }
        */
        if (op->signalSamplePeriodMs<4)
        {
            vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(4));
        }
        else
        {
            vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(op->signalSamplePeriodMs)); 
        }

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
    memset(em_ctrl_playback_trace, 0, sizeof(em_ctrl_playback_trace));
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
        op.em[i].sample_period = 100;
        op.em[i].mode = 1;        
        op.em[i].duty_cycle = 50;
        op.em[i].frequency_factor = 1;
        op.em[i].magnetic_field_offset = 0;
        op.em[i].magnetic_field_amplitude = 1000;
        op.em[i].em_group = 1;
        op.em[i].coils_columns = 1;
        op.em[i].rotational_frequency_factor = 1;
    

        gain_kalman[i].gain = 0.033333;
        gain_kalman[i].P = 1.0f;
        gain_kalman[i].Q = 1e-12f;
        gain_kalman[i].R = 1e-2;
        
    }
    op.cfle_pwr = cfle_pwr;
    xil_printf("done\r\n");

    int status;
    status = XGpio_Initialize(&pwmEnableGpio, XPAR_AXI_GPIO_0_BASEADDR);
    if (status != XST_SUCCESS)
    {
        perror("Gpio Initialization Failed\r\n");
    }
    enableAllDutyCycle();

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

    for (int i=0;i<6;i++)
    {
        xQueue[i] = xQueueCreate( 	1,sizeof( uint8_t ) );	/* Each space in the queue is large enough to hold a uint8_t. */
        configASSERT(xQueue[i]);
    }

	



    // Define configs (static or global so they persist in memory)
    // Handles disable outputs and alert intpus  (the second one is not used)      
    configA.op = &op; 
    configA.baseAddr = XPAR_I2C_MAGNET_PORTS_AXI_IIC_IOEXP_1_BASEADDR;
    configA.waitSem = NULL; 
    configA.signalSem = semAtoB;

    configB.op = &op; 
    configB.baseAddr = XPAR_I2C_MAGNET_PORTS_AXI_IIC_IOEXP_2_BASEADDR;
    configB.waitSem = semAtoB; 
    configB.signalSem = semBtoC;

    configC.op = &op; 
    configC.baseAddr = XPAR_I2C_MAGNET_PORTS_AXI_IIC_A_BASEADDR;
    configC.waitSem = semBtoC; 
    //configC.signalSem = semItoJ;//semCtoD;
    configC.signalSem = semCtoD;

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

    configM.op = &op;
    configM.baseAddr = XPAR_AXI_GPIO_0_BASEADDR;
    configM.waitSem = NULL; 
    configM.signalSem = NULL;
    BaseType_t taskStatus;

    taskStatus = xTaskCreate(vTaskIoExp      , "IoExp1"  , 128  , &configA, tskIDLE_PRIORITY + 1, NULL);// Needs at least more thatn 64 
    if (taskStatus != pdPASS) {
        xil_printf("Failed to create vTaskIoExp1\r\n");
    }
    taskStatus = xTaskCreate(vTaskIoExp      , "IoExp2"  , 128  , &configB, tskIDLE_PRIORITY + 1, NULL);// Needs at least more thatn 64
    if (taskStatus != pdPASS) {
        xil_printf("Failed to create vTaskIoExp2\r\n");
    }
    taskStatus = xTaskCreate(vTaskMagnet     , "MagnetA" , 1024  , &configC, tskIDLE_PRIORITY + 3, NULL);
    if (taskStatus != pdPASS) {
        xil_printf("Failed to create vTaskMagnetA\r\n");
    }
    taskStatus = xTaskCreate(vTaskMagnet     , "MagnetB" , 1024  , &configD, tskIDLE_PRIORITY + 3, NULL);
    if (taskStatus != pdPASS) {
        xil_printf("Failed to create vTaskMagnetB\r\n");
    }
    taskStatus = xTaskCreate(vTaskMagnet     , "MagnetC" , 1024  , &configE, tskIDLE_PRIORITY + 3, NULL);
    if (taskStatus != pdPASS) {
        xil_printf("Failed to create vTaskMagnetC\r\n");
    }
    taskStatus = xTaskCreate(vTaskMagnet     , "MagnetD" , 1024  , &configF, tskIDLE_PRIORITY + 3, NULL);
    if (taskStatus != pdPASS) {
        xil_printf("Failed to create vTaskMagnetD\r\n");
    }
    taskStatus = xTaskCreate(vTaskMagnet     , "MagnetE" , 1024  , &configG, tskIDLE_PRIORITY + 3, NULL);
    if (taskStatus != pdPASS) {
        xil_printf("Failed to create vTaskMagnetE\r\n");
    }
    taskStatus = xTaskCreate(vTaskMagnet     , "MagnetF" , 1024  , &configH, tskIDLE_PRIORITY + 3, NULL);
    if (taskStatus != pdPASS) {
        xil_printf("Failed to create vTaskMagnetF\r\n");
    }
    taskStatus = xTaskCreate(vTaskPwm        , "Pwm"     , 2048  , &configI, tskIDLE_PRIORITY + 2, NULL);
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
    taskStatus = xTaskCreate(vTaskButtons       , "Buttons"    , 512  , &configM, tskIDLE_PRIORITY + 1, NULL);
    if (taskStatus != pdPASS) {
        xil_printf("Failed to create vTaskButtons\r\n");
    }   

    for (int i=0;i<30;i++){
        setledPanelColor(i, 0, 0, 0); // Blue
        
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

    TIMING_REG = Xil_In32(PWM_ADDRESS[id]);
    //printf("\tPREV TIMING REG: %u\t", TIMING_REG);


    TIMING_REG = (uint32_t)((PWM_BASE_CLK / frequency - 1) + 0.5); // 0.5 rounds up

    Xil_Out32(PWM_ADDRESS[id], TIMING_REG);
    // To-Do: Disable PWM first
    //printf("Set PWM N %u at %.2f kHz. TIMING REG: %u", id, frequency, TIMING_REG);

    TIMING_REG = Xil_In32(PWM_ADDRESS[id]);
    //printf("\tVERIFIED TIMING REG: %u\n", TIMING_REG);

    
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



void pause_vector_write_bit(uint32_t bit_pos, bool value)
{
    uint32_t byte_index = bit_pos >> 3;      // divide by 8
    uint8_t  bit_index  = bit_pos & 0x07;    // mod 8

    uint8_t mask = (1U << bit_index);

    if (value)
    {
        pause_vector[byte_index] |= mask;    // Set bit
    }
    else
    {
        pause_vector[byte_index] &= ~mask;   // Clear bit
    }
}


bool pause_vector_read_bit(uint32_t bit_pos)
{
    uint32_t byte_index = bit_pos >> 3;
    uint8_t  bit_index  = bit_pos & 0x07;

    uint8_t mask = (1U << bit_index);

    return (pause_vector[byte_index] & mask) != 0;
}