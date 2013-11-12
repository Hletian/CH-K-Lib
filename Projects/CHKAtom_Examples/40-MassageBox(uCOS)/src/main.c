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
* 1- ����uCOS 3������ һ�������ź���������  ����һ�������ź�������ʾ ����һ��������Ϣ���䲢��ʾ�����Ϣ 
*/




#define TASK_STK_SIZE              (128)  //��������ͳһʹ��128�ֽڶ�ջ��ַ
//�����������ȼ�
#define APP_START_TASK_PRIO        (4)  //��ʼ����
#define APP_LED_TASK_PRIO          (6)  //LED��˸����
#define APP_MBOX_TASK_PRIO         (8)  //���������ʾ����
#define APP_SEM_TASK_PRIO          (9)  //�ź���������ʾ����
#define APP_POST_TASK_PRIO         (10) //���� �ź���Ͷ������
#define APP_WDOG_TASK_PRIO         (20) //���Ź� ����
//���������ջ
OS_STK  APP_START_STK[TASK_STK_SIZE];
OS_STK  APP_LED_STK[TASK_STK_SIZE];
OS_STK  APP_MBOX_STK[TASK_STK_SIZE];
OS_STK  APP_SEM_STK[TASK_STK_SIZE];
OS_STK  APP_POST_STK[TASK_STK_SIZE];
OS_STK  APP_WDOG_STK[TASK_STK_SIZE];
//////////////////////////////////////////////////////////////////////////////
OS_EVENT * msg_test;		//���������¼���ָ��
OS_EVENT * sem_test;		//�������ź���ָ��	 
//���Ź�����
void AppWDOGTask(void *pdata)
{
	pdata=pdata; //��ֹ���������� ��ʵ������
	WDOG_Init(100); //�������Ź� 100MS��ʱ��λ
	while(1)
	{
		WDOG_Feed();
		OSTimeDlyHMSM(0, 0, 0, 50);
	}
}
//����Ͷ�ݣ��ź���Ͷ������
void AppPostTask(void *pdata)
{
	uint8_t key='A';
	uint16_t TaskCtr=0;
	while(1)
	{
		key++;
		TaskCtr++;
		if(key > 'Z') key='A';            //�ı�����Ͷ�ݵ�����
		OSMboxPost(msg_test,(void*)key);  //������Ϣ
		OSSemPost(sem_test);              //�����ź���

		UART_printf("App Post Message&Sem:%d times\r\n", TaskCtr);
		OSTimeDlyHMSM(0, 0, 0, 300);
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
//������պ�������
void AppMBOXTask(void *pdata)
{
	uint8_t key;
	uint8_t err;
	uint16_t TaskCtr=0;
  pdata=pdata; //��ֹ���������� ��ʵ������
	while(1)
	{
		key=(uint32_t)OSMboxPend(msg_test,0,&err);  //�ȴ���Ϣ����
		TaskCtr++;
      UART_printf("Received MBox:%d \r\n", key);
	}
}
//�ź������ԣ���ʾ����
void AppSEMTask(void *pdata)
{
	uint8_t err;
	uint16_t TaskCtr=0;
  pdata=pdata; //��ֹ���������� ��ʵ������
	while(1)
	{
		OSSemPend(sem_test,0,&err); //�ȴ��ź���
		TaskCtr++;
		UART_printf("Received Sem:%d  \r\n", TaskCtr);
	}
}

//��ʼ����
void AppStartTask(void *pdata)
{
  OS_CPU_SR cpu_sr=0;
	pdata = pdata; 		  
	msg_test=OSMboxCreate((void*)0);	//������Ϣ����
	sem_test=OSSemCreate(0);		//�����ź���	
	OSStatInit();					//��ʼ��ͳ������.�������ʱ1��������	
 	OS_ENTER_CRITICAL();			//�����ٽ���(�޷����жϴ��)    
    OSTaskCreate(AppLEDTask,(void *)0,
                &APP_LED_STK[TASK_STK_SIZE-1],
                APP_LED_TASK_PRIO); //����LED1����
    OSTaskCreate(AppMBOXTask,(void *)0,
                &APP_MBOX_STK[TASK_STK_SIZE-1],
                APP_MBOX_TASK_PRIO); //�������������ʾ����
    OSTaskCreate(AppSEMTask,(void *)0,
                &APP_SEM_STK[TASK_STK_SIZE-1],
                APP_SEM_TASK_PRIO); //�����ź���������ʾ����
    OSTaskCreate(AppWDOGTask,(void *)0,
                &APP_WDOG_STK[TASK_STK_SIZE-1],
                APP_WDOG_TASK_PRIO); //�������Ź�����
    OSTaskCreate(AppPostTask,(void *)0,
                &APP_POST_STK[TASK_STK_SIZE-1],
                APP_POST_TASK_PRIO); //�������䣬�ź���Ͷ������
  UART_printf("uCOSII MBox&Sem DemoTest\r\n");
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

void SysTick_Handler (void)
{
	OSIntEnter();		//�����ж�
	OSTimeTick();       //����ucos��ʱ�ӷ������               
	OSIntExit();        //���������л����ж�
}

void uCOS_SysTickInit(void)
{
    SysTick->CTRL|=1<<1;   						//����SYSTICK�ж�
    SysTick->LOAD=CPUInfo.CoreClock/OS_TICKS_PER_SEC; 	//ÿ1/OS_TICKS_PER_SEC���ж�һ��	
    SysTick->CTRL|=1<<0;   														//����SYSTICK 
}
