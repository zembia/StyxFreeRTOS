#include "globaldefines.h"
#include "magnetTools.h"
#include "xgpio.h"
extern XGpio pwmEnableGpio;
extern const uint32_t PWM_ADDRESS[30];
void updateTemperaturePL(uint8_t id, int32_t temperature)
{
    Xil_Out32(PWM_ADDRESS[id] + 12, temperature);
}


float interpretMagneticField(int16_t rawValue)
{
    
    return (rawValue-2330)/0.73;
}

#define REF_V 4680.0
#define R1 24000
float interpretTempearture(int16_t rawValue) {

    //rawValue/(REF_V) = X/(X+R1);
    //(rawValue/REF_V)*X +R1*(rawValue/REF_V)* = X
    //X*(rawValue/REF_V - 1) = -R1*(rawValue/REF_V)
    //X = -R1*(rawValue/REF_V) / (rawValue/REF_V - 1)
    float X = -R1 * (rawValue / REF_V) / ((rawValue / REF_V) - 1);

    float a = 639.5, b = -0.1332, c = -162.5;
    float Temp = (a * pow(X, b) + c)*10;
    //float Temp = 0;
    // Assuming rawValue is a signed 11-bit integer representing temperature in °C with a scale factor of 0.125
    return Temp;
}

void setledPanelColor(uint8_t LED, uint8_t R, uint8_t G, uint8_t B)
{
    uint32_t dataOut;
    dataOut = LED<<24 | R<<16 | G<<8 | B;
    Xil_Out32(XPAR_WS2812B_DRIVER_0_BASEADDR, dataOut);
}


void enableAllDutyCycle(void)
{
    XGpio_DiscreteWrite(&pwmEnableGpio, 1, 1);
}

void disableAllDutyCycle(void)
{
    XGpio_DiscreteWrite(&pwmEnableGpio, 1, 0);
}

void setPwmMode(uint8_t id, bool mode)
{
    Xil_Out32(PWM_ADDRESS[id] + 8, mode);
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
    //printf("Set PWM N %u at %.2f %%. DC REG: %d. Timing REG: %d\n",id,dutyCycle,DC_REG, TIMING_REG);
}



void calibrateMagnet(uint8_t id, operation_control_t *op) 
{
    enableAllDutyCycle();
   int16_t value1, value2;
   xil_printf("Calibrating magnet %u\r\n", id);

   //ramp up
   for (int i=0;i<100;i=i+1)
   {
    setDutyCycle(id, i);
    vTaskDelay(pdMS_TO_TICKS(2));
   }
   vTaskDelay(pdMS_TO_TICKS(10));
   value1 = op->em[id].last_em_measure;

   //ramp to rest
   for (int i=100;i>0;i=i-1)
   {
    setDutyCycle(id, i);
    vTaskDelay(pdMS_TO_TICKS(2));
   }

   //ramp down
   for (int i=0;i>-100;i=i-1)
   {
    setDutyCycle(id, i);
    vTaskDelay(pdMS_TO_TICKS(2));
   }

   vTaskDelay(pdMS_TO_TICKS(10));
   value2 = op->em[id].last_em_measure;

   //ramp to rest
   for (int i=-100;i<0;i=i+1)
   {
    setDutyCycle(id, i);
    vTaskDelay(pdMS_TO_TICKS(2));
   }
   disableAllDutyCycle();
    float slope = (value2 - value1) / -200.0f; // 200 is the total change in duty cycle
    float intercept = value1 - slope * 100; // Using the point (100, value1) to find the intercept
    printf("Calibration result for magnet %u: slope = %.4f, intercept = %.2f\r\n", id, slope, intercept);
    return;

}