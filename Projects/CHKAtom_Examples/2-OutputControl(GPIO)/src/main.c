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
//use:            �û����� ����mian������ �жϷ������ 

/*
* ����֪ʶ:
* 1-SystemClockSetup: ʹ��Ƭ��FLL �� PLL��Ƶ�ں�ʱ��: 
*   1:ClockSource_EX50M:�ⲿ50M����  ClockSource_IRC:�ڲ�RC����
*   2:CoreClock_100M: ��Ƶ�� �ں�Ƶ�� = 100M
* 2-Delay: ��ʱ���� ����SysTick ��ʱ��ʵ�� ���ȳ�ʼ��DelayInit(); Ȼ��Ϳ���ʹ��DelayMs ��  DelayUs��
* 3-GPIO_ToggleBit() ��תһ��IO�ڵĵ�ƽ ���ƵĻ���
*   1:GPIO_SetBits   ��һ��IO������Ϊ�ߵ�ƽ
*   2:GPIO_ResetBits ��һ��IO������Ϊ�͵�ƽ 
*   3:�ȶຯ���μ�gpio.h �� gpio.c
*/

int main(void)
{
	//����GPIO��ʼ���ṹ
	GPIO_InitTypeDef GPIO_InitStruct1;
	//��ʼ��ϵͳʱ�� ʹ���ڲ�RC���� FLL��Ƶ��96M
	SystemClockSetup(ClockSource_IRC,CoreClock_96M);
        DelayInit();
	GPIO_InitStruct1.GPIOx = PTC;                       //PTC(PORTC) �˿�
	GPIO_InitStruct1.GPIO_InitState = Bit_SET;          //�������Ϊ��� ������ߵ�ƽ �������Ϊ���� ������Ч
	GPIO_InitStruct1.GPIO_IRQMode = GPIO_IT_DISABLE;    //��ֹ�����ŵ��ж�����
	GPIO_InitStruct1.GPIO_Pin = GPIO_Pin_16;            //C16����
	GPIO_InitStruct1.GPIO_Mode = GPIO_Mode_OPP;         //����Ϊ�������ģʽ
	//ִ��GPIO��ʼ�� ����LED
	GPIO_Init(&GPIO_InitStruct1);                             
	
    while(1)
    {
        GPIO_ToggleBit(PTC, GPIO_Pin_16);
        DelayMs(500);
    }
}

