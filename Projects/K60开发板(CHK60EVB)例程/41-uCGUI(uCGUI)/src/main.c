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



//MiniShell
#include "minishell.h"

//���� ��˼���� Kinetis�̼�������
//�̼���汾 V2.4
//startup:        �����ļ�
//devices:        ����оƬ֧�ֿ�(����ʵ���õ�)
//utilities:      ���ģ��֧�ֿ�(����ʵ���õ�)
//drivers:        �̼���Դ��

/*
* ����֪ʶ:
* 1- MiniShell ���˿����ļ���Shellϵͳ
* 2- MiniShell�� ʹ�� ����ʵ��GetChar �� PutChar����Ȼ���尲װ�ṹ�� ����MINISHELL_Install����
* 3- �û���������SHELL_Register ע����� ����ע�����64���û�����
* 4- ע����ɺ����MINISHELL_Init()��ʼ��
* 5- ���º󻰵���LOOP������ʼSHELLѭ��
* 6- ����λ���ն���� ���������
*/
extern volatile int OS_TimeMS;
int main(void)
{
    OS_TimeMS=0; 
    PIT_InitTypeDef PIT_InitStruct1;
    SystemClockSetup(ClockSource_EX50M,CoreClock_100M);
    DelayInit();
    LED_Init(LED_PinLookup_CHK60EVB, kNumOfLED);
    UART_DebugPortInit(UART4_RX_C14_TX_C15, 115200);
	  DisplayCPUInfo();
	  //
    PIT_InitStruct1.PITx = PIT0;
    PIT_InitStruct1.PIT_Interval = 1;
	  PIT_Init(&PIT_InitStruct1);
	  PIT_ITConfig(PIT0, PIT_IT_TIF, ENABLE);
	  NVIC_EnableIRQ(PIT0_IRQn);
	  //
    PIT_InitStruct1.PITx = PIT1;
    PIT_InitStruct1.PIT_Interval = 10;
	  PIT_Init(&PIT_InitStruct1);
	  PIT_ITConfig(PIT1, PIT_IT_TIF, ENABLE);
	  NVIC_EnableIRQ(PIT1_IRQn);
	  ADS7843_Init();
		MainTask();			 //����UCGUIdemo
    while(1) 
    {
			
    }
}

