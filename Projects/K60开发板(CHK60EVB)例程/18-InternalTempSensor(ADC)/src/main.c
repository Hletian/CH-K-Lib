#include "sys.h"
#include "uart.h"
#include "gpio.h"
#include "adc.h"
#include "can.h"
#include "dac.h"
#include "delay.h"
#include "dma.h"
#include "enet.h"
#include "flash.h"
#include "ftm.h"
#include "i2c.h"
#include "lptm.h"
#include "pdb.h"
#include "pit.h"
#include "rtc.h"
#include "sd.h"
#include "spi.h"
#include "tsi.h"
#include "wdog.h"

//LED Devices
#include "led.h"
#include "led_chk60evb.h"
//KBI Devices
#include "kbi.h"
#include "kbi_chk60evb.h"



//���� ��˼���� Kinetis�̼�������
//�̼���汾 V2.4
//startup:        �����ļ�
//devices:        Ӳ��������Դ֧�ֿ�(����ʵ���õ�)
//utilities:      ���ģ��֧�ֿ�(����ʵ���õ�)
//drivers:        �̼���Դ��
//use:            �û����� ����mian������ �жϷ������ 

/*
* ����֪ʶ:
* 1- �ڲ��¶ȼ�ʵ���Ͼ��� оƬ�ڲ���һ�� ���������ӵ�ADC�ϡ�ʵ�ʻ���ʹ��ADC������
*/


char strBuffer[30];
int main(void)
{
    float temp;
    float vtemp;
    uint32_t ADC_Value = 0;
    ADC_InitTypeDef ADC_InitStruct1;
    //��ʼ��ϵͳʱ�� ʹ���ⲿ50M���� PLL��Ƶ��100M
    SystemClockSetup(ClockSource_EX50M,CoreClock_100M);
    DelayInit();
    LED_Init(LED_PinLookup_CHK60EVB, kNumOfLED);
    UART_DebugPortInit(UART4_RX_C14_TX_C15, 115200);
	  DisplayCPUInfo();
    //��ʼ��ADC
    ADC_InitStruct1.ADCxMap = ADC0_TEMP_SENSOR_DIFF;      //�ڲ��¶ȼ�
    ADC_InitStruct1.ADC_Precision = ADC_PRECISION_12BIT;  //12λ����
    ADC_InitStruct1.ADC_TriggerSelect = ADC_TRIGGER_SW;   //�������(A ͨ����ʹ����/Ӳ������ B ͨ��ֻ��ʹ��Ӳ������)
    ADC_Init(&ADC_InitStruct1);
    while(1) 
    {		
        ADC_Value = ADC_GetConversionValue(ADC0_TEMP_SENSOR_DIFF) & 0x0FFF; 
        //��ADֵת��Ϊ ���϶�  �ο��ٷ����� ��������Ǻ�׼ȷ
        vtemp=((float)ADC_Value/4096)*3.3;
        if(vtemp>=.7012)
        {
            temp=(vtemp-.7012)/.001646;
        }
        else
        {
            temp=(vtemp-.7012)/.001769;
        }
        temp=25-temp;
        sprintf((char *)strBuffer,"Tempature:%0.3f C\r\n",temp);	
        UART_printf(strBuffer);
        DelayMs(300);
    }
}

