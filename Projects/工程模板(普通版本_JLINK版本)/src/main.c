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
* 1-K60��A B C D E 5��PORT��ÿ��PORT����0-31������(�����ͺŲ�ͬ����ɾ��)
* 2-ÿ��GPIO���� ������Ϊ ��� ����
* 3-�����ֿ�������Ϊ��ͨ���� �� �ⲿ�жϴ�������
* 4-Bit_RESET:�͵�ƽ  Bit_SET:�ߵ�ƽ
* 5-��ʼ��һ��GPIO:
*   1:�������ṹ��GPIO_InitStruct1
*   2:���ṹ�崫����Ӧ�����ò���
*   3:����GPIO_Init() ��������ʼ�����GPIO
* 6-���Ҫ��ʼ�����GPIO ֻ���������ó�ʼ���ṹ�� Ȼ�����ε���GPIO_Init
*/

int main(void)
{
    //����GPIO��ʼ���ṹ
    GPIO_InitTypeDef GPIO_InitStruct1;
    //��ʼ��ϵͳʱ�� ʹ���ڲ�RC���� FLL��Ƶ��96M
    SystemClockSetup(ClockSource_IRC,CoreClock_96M);
    //��ʼ����ʱ
	  DelayInit();
    GPIO_InitStruct1.GPIOx = PTD;                       //PTD(PORTD) �˿�
    GPIO_InitStruct1.GPIO_InitState = Bit_RESET;        //�������Ϊ��� ������͵�ƽ �������Ϊ���� ������Ч
    GPIO_InitStruct1.GPIO_IRQMode = GPIO_IT_DISABLE;    //��ֹ�����ŵ��ж�����
    GPIO_InitStruct1.GPIO_Pin = GPIO_Pin_7;             //D7����
    GPIO_InitStruct1.GPIO_Mode = GPIO_Mode_OPP;         //����Ϊ�������ģʽ
    //ִ��GPIO��ʼ�� ����LED
    GPIO_Init(&GPIO_InitStruct1);                             
	
    while(1)
    {
        GPIO_ToggleBit(PTD, GPIO_Pin_7); //��ת��ƽ
        DelayMs(500); //��ʱ500MS
    }
}

