#include "ledbuttontools.h"


#define PWM_FAN_CTRL 0
#define GREEN_BTN 1
#define RED_BTN 2
#define LED_IND1 3
#define LED_IND2 4

#define RED_BTN_DEFAULT_STATE 0
#define GREEN_BTN_DEFAULT_STATE 0

static XGpio gpio_inst;
void vTaskButtons(void *pvParameters)
{

    /*
    task_manager_t *cfg = (task_manager_t *)pvParameters;
    uint32_t baseAddr = cfg->baseAddr;
    char *pcTaskName = pcTaskGetName(NULL);

     // 1. Wait for the previous task to finish
    if (cfg->waitSem != NULL) {
        xSemaphoreTake(cfg->waitSem, portMAX_DELAY);
    }
    xil_printf("Starting %s at address 0x%08x\r\n", pcTaskName, baseAddr);   

    int status;
    status = XGpio_Initialize(&gpio_inst, XPAR_AXI_GPIO_0_BASEADDR);
    if (status != XST_SUCCESS)
    {
        perror("GPIO Initialization Failed\r\n");
    }

    


    //0 is for output, 1 is for input
    uint32_t direction_mask =   (0 << PWM_FAN_CTRL) | 
                                (1 << GREEN_BTN) | 
                                (1 << RED_BTN) | 
                                (0 << LED_IND1) | 
                                (0 << LED_IND2);
                            
    XGpio_SetDataDirection(&gpio_inst, PWM_FAN_CTRL, direction_mask); // Set PWM_FAN_CTRL as output
    TickType_t lastWake = xTaskGetTickCount();


    uint32_t gpio_state = XGpio_DiscreteRead(&gpio_inst, 1);
    uint8_t green_sample  = (gpio_state >> GREEN_BTN) & 0x1;
    uint8_t red_sample    = (gpio_state >> RED_BTN) & 0x1;

    uint8_t green_last_sample = green_sample;
    uint8_t red_last_sample = red_sample;

    while(1)
    {
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(10));

        green_last_sample = green_sample;
        red_last_sample = red_sample;

        gpio_state = XGpio_DiscreteRead(&gpio_inst, 1);
        green_sample = (gpio_state >> GREEN_BTN) & 0x1;
        red_sample   = (gpio_state >> RED_BTN) & 0x1;

        if (red_sample != RED_BTN_DEFAULT_STATE && red_last_sample == RED_BTN_DEFAULT_STATE)
        {
            xil_printf("Red button pressed\r\n");
            //do something
        
        }

        if (green_sample != GREEN_BTN_DEFAULT_STATE && green_last_sample == GREEN_BTN_DEFAULT_STATE)
        {
            xil_printf("Green button pressed\r\n");
            //do something
        }        
    }
    */
}


void setLedInd1State(bool state)
{
    uint32_t gpio_state = XGpio_DiscreteRead(&gpio_inst, 1);

    //we clear the bit corresponding to LED_IND1 and then set it to the desired state
    gpio_state = (gpio_state & ~(1 << LED_IND1)) | (state << LED_IND1);
    XGpio_DiscreteWrite(&gpio_inst, 1, gpio_state);
}
void setLedInd2State(bool state)
{
    uint32_t gpio_state = XGpio_DiscreteRead(&gpio_inst, 1);
    //we clear the bit corresponding to LED_IND2 and then set it to the desired state
    gpio_state = (gpio_state & ~(1 << LED_IND2)) | (state << LED_IND2);
    XGpio_DiscreteWrite(&gpio_inst, 1, gpio_state);
} 