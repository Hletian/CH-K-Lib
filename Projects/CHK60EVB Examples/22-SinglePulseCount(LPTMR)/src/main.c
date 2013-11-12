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
//SPI Flash Devices;
#include "spiflash.h"
//LCD Devices
#include "lcd_chk60evb.h"  
#include "ads7843.h"

//CHGUI 
#include "chgui.h"         
#include "chgui_char.h"    
#include "chgui_bmp.h"     
#include "chgui_touch.h"


//���� ��˼���� Kinetis�̼�������
//�̼���汾 V2.4
//startup:        �����ļ�
//devices:        Ӳ��������Դ֧�ֿ�(����ʵ���õ�)
//utilities:      ���ģ��֧�ֿ�(����ʵ���õ�)
//drivers:        �̼���Դ��
//use:            �û����� ����mian������ �жϷ������ 

/*
* ����֪ʶ:
* 1- LPTMR �͹��Ķ�ʱ�� ������������ͨ�ڲ����������ⲿ�������
* 2- ��ʵ������FTMģ����PC4�ϲ���1KHZ���� ʹ��PC5�������弼�� �����ڴ�ӡ��ʾ
*    ʵ������ε�LCD �̽�C5C4
*/


int main(void)
{
    uint32_t LPTM_Value = 0;
    LPTM_InitTypeDef LPTM_InitStruct1;
    FTM_InitTypeDef FTM_InitStruct1;
    SystemClockSetup(ClockSource_EX50M,CoreClock_100M);
    DelayInit();
    LED_Init(LED_PinLookup_CHK60EVB, kNumOfLED);
    UART_DebugPortInit(UART4_RX_C14_TX_C15, 115200);
	  DisplayCPUInfo();
	  //��PC3�ϲ��� 1KHz ռ�ձ�50%�� ����
    FTM_InitStruct1.Frequency = 1000;
    FTM_InitStruct1.FTMxMAP = FTM0_CH3_PC4;
    FTM_InitStruct1.FTM_Mode = FTM_Mode_EdgeAligned;
    FTM_InitStruct1.InitalDuty = 5000;
    FTM_Init(&FTM_InitStruct1);
    //����PTC5�ϵ������������
    LPTM_InitStruct1.LPTMxMap = LPTM_CH2_PC5;
    LPTM_InitStruct1.LPTM_InitCompareValue = 200;          //���������ģʽ��������
    LPTM_InitStruct1.LPTM_Mode = LPTM_Mode_PC_FALLING;     //�½��ش�������
    LPTM_Init(&LPTM_InitStruct1);
    while(1) 
    {
        //��ȡ���弼��ֵ
        LPTM_Value = LPTM_GetTimerCounterValue(LPTMR0);
        //��ռ���ֵ
        LPTM_ResetTimeCounter(LPTMR0);
        UART_printf("LPTMR:%dHz\r\n", LPTM_Value);
        DelayMs(1000);
    }
}

