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


//���� ��˼���� Kinetis�̼�������
//�̼���汾 V2.4
//startup:        �����ļ�
//devices:        Ӳ��������Դ֧�ֿ�(����ʵ���õ�)
//utilities:      ���ģ��֧�ֿ�(����ʵ���õ�)
//drivers:        �̼���Դ��
//user:           �û����� ����mian������ �жϷ������ 
 
/*
* ����֪ʶ:
* 1-CH-Kinetis�̼��� ģ�����(Bootloader�汾)
* 2-���ʹ��Bootloader ���ô�ģ�� ����Ҫ�����ģ��
*/

int main(void)
{
    //����GPIO��ʼ���ṹ
    GPIO_InitTypeDef GPIO_InitStruct1;
    SetVectorTable(0x05000UL); //Ϊ�����Bootlaoder���򣬸����ж���������ʼ��ַ	
    //��ʼ��ϵͳʱ�� ʹ���ڲ�RC���� FLL��Ƶ��96M
    SystemClockSetup(ClockSource_EX50M,CoreClock_96M);
    //��ʼ����ʱ
    DelayInit();
    GPIO_InitStruct1.GPIOx = PTD;                       //PTD(PORTD) �˿�
    GPIO_InitStruct1.GPIO_InitState = Bit_RESET;        //�������Ϊ��� ���Ϊ�͵�ƽ  ����Ϊ����ʱ������Ч
    GPIO_InitStruct1.GPIO_IRQMode = GPIO_IT_DISABLE;    //��ֹ�����ŵ��ж�����
    GPIO_InitStruct1.GPIO_Pin = GPIO_Pin_7;             //D7����
    GPIO_InitStruct1.GPIO_Mode = GPIO_Mode_OPP;         //����Ϊ�������ģʽ
    //ִ��GPIO��ʼ�� ����LED
    GPIO_Init(&GPIO_InitStruct1);                             
	
    while(1)
    {
        GPIO_ToggleBit(PTD, GPIO_Pin_7);//��ת��ƽ
        DelayMs(500);
    }
}

