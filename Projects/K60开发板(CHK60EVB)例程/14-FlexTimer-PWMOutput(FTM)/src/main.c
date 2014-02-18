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
*/


int main(void)
{
    uint32_t FTMDuty = 0;
    FTM_InitTypeDef FTM_InitStruct1;
    //��ʼ��ϵͳʱ�� ʹ���ⲿ50M���� PLL��Ƶ��100M
    SystemClockSetup(ClockSource_EX50M,CoreClock_100M);
    DelayInit();
    LED_Init(LED_PinLookup_CHK60EVB, kNumOfLED);
    UART_DebugPortInit(UART4_RX_C14_TX_C15, 115200);
    UART_printf("PWM Test\r\n");
	
    //��ʼ��FTM
    FTM_InitStruct1.Frequency = 1000;                // 1KHZ
    FTM_InitStruct1.FTMxMAP = FTM0_CH0_PC1;          //FTM0_CH0 PC1����
    FTM_InitStruct1.FTM_Mode = FTM_Mode_EdgeAligned; //���ض���ģʽ
    FTM_InitStruct1.InitalDuty = 4000;               //��ʼ�������40%��ռ�ձ�
    FTM_Init(&FTM_InitStruct1);
	
    FTM_InitStruct1.InitalDuty = 7000;               //��ʼ�������70%��ռ�ձ�
    FTM_InitStruct1.FTMxMAP = FTM0_CH1_PC2;          //FTM0 CH1 PC2 ����
    FTM_Init(&FTM_InitStruct1);
    while(1)
		{
        FTMDuty += 1000;
	    	FTMDuty %= 10000; //FTMռ�ձ����뷶Χ 0-10000 ��Ӧ %0- 100%
	    	FTM_PWM_ChangeDuty(FTM0_CH0_PC1, FTMDuty); //FTMDUTY�仯
	    	DelayMs(300);
		}
 }

