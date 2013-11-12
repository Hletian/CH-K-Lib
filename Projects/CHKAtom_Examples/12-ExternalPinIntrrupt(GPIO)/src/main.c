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


//���� ��˼���� Kinetis�̼�������
//�̼���汾 V2.4
//startup:        �����ļ�
//devices:        Ӳ��������Դ֧�ֿ�(����ʵ���õ�)
//utilities:      ���ģ��֧�ֿ�(����ʵ���õ�)
//drivers:        �̼���Դ��
//use:            �û����� ����mian������ �жϷ������ 

/*
* ����֪ʶ:
* 1- �ⲿ�ж� ���ⲿ���ŵ�ƽ�������������ж�
*/


int main(void)
{
    GPIO_InitTypeDef GPIO_InitStruct1;
    //��ʼ��ϵͳʱ�� ʹ���ⲿ50M���� PLL��Ƶ��100M
    SystemClockSetup(ClockSource_EX50M,CoreClock_100M);
    DelayInit();
    LED_Init(LED_PinLookup_CHKATOM, kNumOfLED);
    UART_DebugPortInit(UART4_RX_C14_TX_C15, 115200);

    GPIO_InitStruct1.GPIOx = PTB;
    GPIO_InitStruct1.GPIO_InitState = Bit_RESET;
    GPIO_InitStruct1.GPIO_IRQMode = GPIO_IT_RISING; //�����ش����ж�
    GPIO_InitStruct1.GPIO_Mode = GPIO_Mode_IPD;     //����
    GPIO_InitStruct1.GPIO_Pin = GPIO_Pin_9;
    GPIO_Init(&GPIO_InitStruct1);
	
    //����NVIC�ж�
    NVIC_EnableIRQ(PORTB_IRQn);
    while(1)
    {
    }
 }

