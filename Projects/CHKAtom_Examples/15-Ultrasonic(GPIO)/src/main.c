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
*/

#define US_TRIG_GPIO_PORT  PTD
#define US_TRIG_GPIO_PIN   GPIO_Pin_0

#define US_ECHO_GPIO_PORT  PTD
#define US_ECHO_GPIO_PIN   GPIO_Pin_1

int main(void)
{
    uint32_t counter = 0;
    GPIO_InitTypeDef GPIO_InitStruct1;
    //��ʼ��ϵͳʱ�� ʹ���ⲿ50M���� PLL��Ƶ��100M
    SystemClockSetup(ClockSource_EX50M,CoreClock_100M);
    DelayInit();
    LED_Init(LED_PinLookup_CHKATOM, kNumOfLED);
    UART_DebugPortInit(UART4_RX_C14_TX_C15, 115200);
    UART_printf("Ultrasonic Test\r\n");
    //Init US_TRIG_Pin
    GPIO_InitStruct1.GPIOx = US_TRIG_GPIO_PORT;
    GPIO_InitStruct1.GPIO_InitState = Bit_RESET;
    GPIO_InitStruct1.GPIO_IRQMode = GPIO_IT_DISABLE;
    GPIO_InitStruct1.GPIO_Mode = GPIO_Mode_OPP;
    GPIO_InitStruct1.GPIO_Pin = US_TRIG_GPIO_PIN;
    GPIO_Init(&GPIO_InitStruct1);
    //Init US_ECHO_PIN
    GPIO_InitStruct1.GPIOx = US_ECHO_GPIO_PORT;
    GPIO_InitStruct1.GPIO_InitState = Bit_RESET;
    GPIO_InitStruct1.GPIO_IRQMode = GPIO_IT_DISABLE;
    GPIO_InitStruct1.GPIO_Mode = GPIO_Mode_IPD;
    GPIO_InitStruct1.GPIO_Pin = US_ECHO_GPIO_PIN;
    GPIO_Init(&GPIO_InitStruct1);
		
		LED_Init(LED_PinLookup_CHKATOM, kNumOfLED);
    while(1)
    {
			  //����һ�� >10US�ߵ�ƽ���� TRIG
        GPIO_SetBits(US_TRIG_GPIO_PORT, US_TRIG_GPIO_PIN);
        DelayUs(30);
        GPIO_ResetBits(US_TRIG_GPIO_PORT, US_TRIG_GPIO_PIN);
        counter = 0;
			  //�ӹ��͵�ƽʱ��
        while(GPIO_ReadInputDataBit(US_ECHO_GPIO_PORT, US_ECHO_GPIO_PIN) == Bit_RESET) {};
			  //��ȡ�ߵ�ƽʱ��
        while(GPIO_ReadInputDataBit(US_ECHO_GPIO_PORT, US_ECHO_GPIO_PIN) == Bit_SET)
        {
          counter++;
					DelayUs(1);
        }
        UART_printf("Times %dUS\r\n", counter);
				//��ʱʱ������������
        DelayUs(counter*100);
				LED_Toggle(kLED1);
    }
 }

