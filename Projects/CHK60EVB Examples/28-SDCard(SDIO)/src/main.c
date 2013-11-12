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

//nrf24xx
#include "nrf2401.h"

//���� ��˼���� Kinetis�̼�������
//�̼���汾 V2.4
//startup:        �����ļ�
//devices:        ����оƬ֧�ֿ�(����ʵ���õ�)
//utilities:      ���ģ��֧�ֿ�(����ʵ���õ�)
//drivers:        �̼���Դ��

/*
* ����֪ʶ:
* 1- DMA ���ǰ��˹� ������CPU����Ԥ������°��˺������� �ر������ڴ�����������
* 2- �ڱ������� ��DMA�����ڴ��ڷ������ݴ��� �� ���ڴ����ݴ��͵� ���ڷ��ͼĴ��� ʵ�ִ��书��
*/



int main(void)
{
    SD_InitTypeDef SD_InitStruct1;
    SystemClockSetup(ClockSource_EX50M,CoreClock_100M);
    DelayInit();
    LED_Init(LED_PinLookup_CHK60EVB, kNumOfLED);
    UART_DebugPortInit(UART4_RX_C14_TX_C15, 115200);
	  DisplayCPUInfo();
	  UART_printf("Please Insert Card\r\n");
    SD_InitStruct1.SD_BaudRate = 2000000;
    //�ȴ�SD����ʼ���ɹ�
    while(SD_Init(&SD_InitStruct1) != ESDHC_OK);
    UART_printf("SD Size:%dMB\r\n", SD_InitStruct1.SD_Size);
    while(1)
    {
		
    }
}

