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

//MiniShell
#include "minishell.h"
//uCOS
#include "includes.h"


//���� ��˼���� Kinetis�̼�������
//�̼���汾 V2.4
//startup:        �����ļ�
//devices:        ����оƬ֧�ֿ�(����ʵ���õ�)
//utilities:      ���ģ��֧�ֿ�(����ʵ���õ�)
//drivers:        �̼���Դ��

/*
* ����֪ʶ:
* 1- uCOS����ϵͳ������ʵ��
* ������ ��uCOSִ��2��С���� �ֱ�ת2��С�Ƶĵ�ƽ uCOS��������
*/

//�����ջ��С
#define TASK_STK_SIZE              (128)
//�����������ȼ�
#define APP_LED1_TASK_PRIO         (5)
#define APP_LED0_TASK_PRIO         (7)
//���������ջ
OS_STK  APP_LED1_STK[TASK_STK_SIZE];
OS_STK  APP_LED0_STK[TASK_STK_SIZE];
//LEDС������
void AppLED1Task(void *pdata)
{
  pdata = pdata; //��ֹ���������� ��ʵ������
	while(1)
	{
		UART_printf("AppLED 1 Task:Process\r\n");
		LED_Toggle(kLED1);
		OSTimeDlyHMSM(0, 0, 0, 300);
	}
}

void AppLED0Task(void *pdata)
{
  pdata = pdata; //��ֹ���������� ��ʵ������
	while(1)
	{
		UART_printf("AppLED 2 Task:Process\r\n");
		LED_Toggle(kLED2);
		OSTimeDlyHMSM(0, 0, 0, 300);
	}
}

void uCOS_SysTickInit(void)
{
    SysTick->CTRL|=1<<1;   						//����SYSTICK�ж�
    SysTick->LOAD=CPUInfo.CoreClock/OS_TICKS_PER_SEC; 	//ÿ1/OS_TICKS_PER_SEC���ж�һ��	
    SysTick->CTRL|=1<<0;   														//����SYSTICK 
}

int main(void)
{
    SystemClockSetup(ClockSource_EX50M,CoreClock_100M);
    DelayInit();
    LED_Init(LED_PinLookup_CHK60EVB, kNumOfLED);
    UART_DebugPortInit(UART4_RX_C14_TX_C15, 115200);
	  DisplayCPUInfo();
	  uCOS_SysTickInit();
    OSInit();  //OS��ʼ��
    OSTaskCreate(AppLED1Task,(void *)0,
                &APP_LED1_STK[TASK_STK_SIZE-1],
                APP_LED1_TASK_PRIO); //����LED1����
    OSTaskCreate(AppLED0Task,(void *)0,
                &APP_LED0_STK[TASK_STK_SIZE-1],
                APP_LED0_TASK_PRIO); //����LED0����
    OSStart(); //����Ȩ��������ϵͳ
	//������Զ�������е���
  while(1);
}

void SysTick_Handler (void)
{
	OSIntEnter();		//�����ж�
	OSTimeTick();       //����ucos��ʱ�ӷ������               
	OSIntExit();        //���������л����ж�
}

