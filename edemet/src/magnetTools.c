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


const uint8_t translateVectorLED[30]={25,26,27,28,29, // Group A
                                14,13,12,11,10, // Group B 
                                19,18,17,16,15, //Group C
                                20,21,22,23,24, //Group D
                                0,1,2,3,4, //Group E
                                5,6,7,8,9 //group F
                            };


/*
//F is 25, group index is 5
translateVectorLED[0] = 25+4;
translateVectorLED[1] = 25+3;
translateVectorLED[2] = 25+2;
translateVectorLED[3] = 25+1;
translateVectorLED[4] = 25+0;

//E is 20
translateVectorLED[5] = 20+4;
translateVectorLED[6] = 20+3;
translateVectorLED[7] = 20+2;
translateVectorLED[8] = 20+1;
translateVectorLED[9] = 20+0;

//C is 15
translateVectorLED[10] = 15+4;
translateVectorLED[11] = 15+3;
translateVectorLED[12] = 15+2;
translateVectorLED[13] = 15+1;
translateVectorLED[14] = 15+0;

//B is 10
translateVectorLED[15] = 10+4;
translateVectorLED[16] = 10+3;
translateVectorLED[17] = 10+2;
translateVectorLED[18] = 10+1;
translateVectorLED[19] = 10+0;

//A is 5
translateVectorLED[20] = 5+4;
translateVectorLED[21] = 5+3;
translateVectorLED[22] = 5+2;
translateVectorLED[23] = 5+1;
translateVectorLED[24] = 5+0;

//D is 0
translateVectorLED[25] = 0+4;
translateVectorLED[26] = 0+3;
translateVectorLED[27] = 0+2;
translateVectorLED[28] = 0+1;
translateVectorLED[29] = 0+0;

*/
void setledPanelColor(uint8_t LED, uint8_t R, uint8_t G, uint8_t B)
{
    uint32_t dataOut;
    uint8_t realLED = translateVectorLED[LED];
    dataOut = realLED<<24 | R<<16 | G<<8 | B;
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
    float duty_cycle =dutyCycle;
    
    if (duty_cycle>100)
    {
        duty_cycle = 100;
    }
    if (duty_cycle<-100)
    {
        duty_cycle=-100;
    }

    int32_t TIMING_REG;
    TIMING_REG = Xil_In32(PWM_ADDRESS[id]);

    int32_t DC_REG;
    DC_REG = TIMING_REG * duty_cycle / 100;
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