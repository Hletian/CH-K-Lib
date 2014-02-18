#include "minishell.h"
#include <stdint.h>
#include "adc.h"





static int DoADC(int argc, char *argv[])
{
    
    uint32_t ADC_Value = 0;
    ADC_InitTypeDef ADC_InitStruct1;

    //???ADC
    ADC_InitStruct1.ADCxMap = ADC0_SE20_DM1;              //PC0 ????ADC0 14??
    ADC_InitStruct1.ADC_Precision = ADC_PRECISION_10BIT;  //10???
    ADC_InitStruct1.ADC_TriggerSelect = ADC_TRIGGER_SW;   //????(A ??????/???? B ??????????)
		ADC_InitStruct1.ADC_ClkDiv = ADC_CLKDIV_1;
    ADC_Init(&ADC_InitStruct1);
		MINISHELL_printf("ADC Test\r\n");
    while(1) 
    {
        ADC_Value = ADC_GetConversionValue(ADC0_SE20_DM1); //??AD???
        UART_printf("ADC0_SE20_DM1:%d\r\n",ADC_Value);
        DelayMs(300);
    }
    uint16_t i;
    return 0;
}





//用户函数注册结构
MINISHELL_CommandTableTypeDef Command_FunADC =
{
    .name = "ADC",            //命令名字
    .maxargs = 3,             //包含的最大参数
    .cmd = DoADC,       //实现函数接口
    .usage = "ADC Test",   //用途说明
};
