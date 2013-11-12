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
#include "led_chkatom.h"
//KBI Devices
#include "kbi.h"
#include "kbi_chkatom.h"
//SPI Flash Devices;
#include "spiflash.h"
//LCD Devices
#include "spilcd.h"  

//CHGUI 
#include "chgui.h"         
#include "chgui_char.h"    
#include "chgui_bmp.h"     

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
* 1- ��ʵ��ʹ��uCOS�����ʱ�� ��ʱˢ��LCD��ɫ  չ�������ʱ������
*/


#define TASK_STK_SIZE              (128)  //��������ͳһʹ��128�ֽڶ�ջ��ַ
//�����������ȼ�
#define APP_START_TASK_PRIO        (4)  //��ʼ����
#define APP_LED_TASK_PRIO          (6)  //LED��˸����
#define APP_WDOG_TASK_PRIO         (20) //���Ź� ����
//���������ջ
OS_STK  APP_START_STK[TASK_STK_SIZE];
OS_STK  APP_LED_STK[TASK_STK_SIZE];
OS_STK  APP_WDOG_STK[TASK_STK_SIZE];
//////////////////////////////////////////////////////////////////////////////
OS_TMR   * tmr1;			//�����ʱ��1
//���Ź�����
void AppWDOGTask(void *pdata)
{
	pdata=pdata; //��ֹ���������� ��ʵ������
	WDOG_Init(100); //�������Ź� 100MS��ʱ��λ
	while(1)
	{
		WDOG_Feed();
		DelayMs(50);
	}
}
//LEDС������
void AppLEDTask(void *pdata)
{
  pdata=pdata; //��ֹ���������� ��ʵ������
	while(1)
	{
		LED_Toggle(kLED1);
		OSTimeDlyHMSM(0, 0, 0, 300);
	}
}
//�����ʱ��1�Ļص�����				  	   
void tmr1_callback(OS_TMR *ptmr,void *p_arg) 
{	
    static uint32_t counter;
    UART_printf("Enter tmr1_callback:%d times\r\n", counter);
    counter++;	
}
//��ʼ����
void AppStartTask(void *pdata)
{
	uint8_t err;
  OS_CPU_SR cpu_sr=0;
	pdata = pdata; 		  
 	tmr1=OSTmrCreate(100,20,OS_TMR_OPT_PERIODIC,(OS_TMR_CALLBACK)tmr1_callback,0,"tmr1",&err);		//1S��ʼִ�У�ÿ200msִ��һ�� ���� OS_TMR_CFG_TICKS_PER_SEC ֵ����
	OSStatInit();					//��ʼ��ͳ������.�������ʱ1��������	
 	OS_ENTER_CRITICAL();			//�����ٽ���(�޷����жϴ��)    
    OSTaskCreate(AppLEDTask,(void *)0,
                &APP_LED_STK[TASK_STK_SIZE-1],
                APP_LED_TASK_PRIO); //����LED1����
    OSTaskCreate(AppWDOGTask,(void *)0,
                &APP_WDOG_STK[TASK_STK_SIZE-1],
                APP_WDOG_TASK_PRIO); //�������Ź�����
  UART_printf("uCOSII Software Timer\r\n");
	OSTmrStart(tmr1,&err);//���������ʱ��1				
 	OSTaskSuspend(APP_START_TASK_PRIO);	//������ʼ����.
	OS_EXIT_CRITICAL();				//�˳��ٽ���(���Ա��жϴ��)
}
static void uCOS_SysTickInit(void);

int main(void)
{
    SystemClockSetup(ClockSource_EX50M,CoreClock_100M);
    DelayInit();
    LED_Init(LED_PinLookup_CHKATOM, kNumOfLED);
    UART_DebugPortInit(UART4_RX_C14_TX_C15, 115200);
	  DisplayCPUInfo();
	  uCOS_SysTickInit();
    OSInit();  //OS��ʼ��
	OSTaskCreate(AppStartTask,(void *)0,
							&APP_START_STK[TASK_STK_SIZE-1],
							APP_START_TASK_PRIO); //����LED1����
	OSStart(); //����Ȩ��������ϵͳ
	//������Զ�������е���
}

static void uCOS_SysTickInit(void)
{
    SysTick->CTRL|=1<<1;   						//����SYSTICK�ж�
    SysTick->LOAD=CPUInfo.CoreClock/OS_TICKS_PER_SEC; 	//ÿ1/OS_TICKS_PER_SEC���ж�һ��	
    SysTick->CTRL|=1<<0;   														//����SYSTICK 
}

void SysTick_Handler (void)
{
	OSIntEnter();		//�����ж�
	OSTimeTick();       //����ucos��ʱ�ӷ������               
	OSIntExit();        //���������л����ж�
}

