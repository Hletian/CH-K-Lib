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
* 1- GPIO ģʽѡ��: ���gpio.h
*   1: GPIO_Mode_IN_FLOATING  ��������
*   2: GPIO_Mode_IPD          ��������
*   3: GPIO_Mode_IPU          ��������
*   4: GPIO_Mode_OOD          ��©��� ֻ������͵�ƽ
*   5: GPIO_Mode_OPP          �������
* 2- GPIO_ReadInputDataBit() ���ĳ��IO�ڵĵ�ƽ״̬ ǰ;�Ǹ�IO�Ѿ�������Ϊ���벢����ȷ�ĳ�ʼ��
*/

int main(void)
{
    uint8_t PinState = 0;
    //����GPIO��ʼ���ṹ
    GPIO_InitTypeDef GPIO_InitStruct1;
    //��ʼ��ϵͳʱ�� ʹ���ⲿ50M���� PLL��Ƶ��100M
    SystemClockSetup(ClockSource_EX50M,CoreClock_100M);
    DelayInit();
    //��ʼ��GPIO�������� (KEY1)
    GPIO_InitStruct1.GPIOx = PTB;              
    GPIO_InitStruct1.GPIO_InitState = Bit_RESET;   
    GPIO_InitStruct1.GPIO_IRQMode = GPIO_IT_DISABLE;
    GPIO_InitStruct1.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStruct1.GPIO_Mode = GPIO_Mode_IPD;         //��������
    GPIO_Init(&GPIO_InitStruct1);  
    //��ʼ��GPIO�������(LED1)
    GPIO_InitStruct1.GPIOx = PTC;  
    GPIO_InitStruct1.GPIO_Pin = GPIO_Pin_16;
    GPIO_InitStruct1.GPIO_Mode = GPIO_Mode_OPP;         //�������
    GPIO_Init(&GPIO_InitStruct1);    
	
    while(1)
    {
        //��KEY1�ĵ�ƽ״̬��ӳ��LED1�� 
        PinState = GPIO_ReadInputDataBit(PTB, GPIO_Pin_9);
        GPIO_WriteBit(PTC, GPIO_Pin_16, (BitAction)PinState); 
    }
}

