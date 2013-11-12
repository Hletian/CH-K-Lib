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
//devices:        Ӳ��������Դ֧�ֿ�(����ʵ���õ�)
//utilities:      ���ģ��֧�ֿ�(����ʵ���õ�)
//drivers:        �̼���Դ��
//use:            �û����� ����mian������ �жϷ������ 

/*
* ����֪ʶ:
* 1
*/

uint8_t NRF2401RXBuffer[32];

int main(void)
{
    uint32_t counter;
    NRF2401_InitTypeDef NRF2401_InitStruct1;
	  PIT_InitTypeDef PIT_InitStruct1;
    SystemClockSetup(ClockSource_EX50M,CoreClock_100M);
    DelayInit();
    LED_Init(LED_PinLookup_CHK60EVB, kNumOfLED);
    UART_DebugPortInit(UART4_RX_C14_TX_C15, 115200);
	  DisplayCPUInfo();
	  //����2401����
	  NRF2401_InitStruct1.CE_GPIO_Instance  = PTB_BASE;    //CE
	  NRF2401_InitStruct1.CE_GPIO_Pin       = GPIO_Pin_3;
	  NRF2401_InitStruct1.CS_GPIO_Instance  = PTB_BASE;    //CS
	  NRF2401_InitStruct1.CS_GPIO_Pin       = GPIO_Pin_9;
	  NRF2401_InitStruct1.IRQ_GPIO_Instance = PTB_BASE;    //IRQ
	  NRF2401_InitStruct1.IRQ_GPIO_Pin      = GPIO_Pin_2;
	  NRF2401_InitStruct1.DATA_SPI_CSMap = SPI1_PCS1_PB9; //SPI
	  NRF2401_InitStruct1.DATA_SPI_DataMap = SPI1_SCK_PB11_SOUT_PB16_SIN_PB17;
	  NRF2401_Init(&NRF2401_InitStruct1);
    if(NRF24L01_Check() != NRF_OK)
    {
        UART_printf("NRF2401 init failed\r\n");
        return 1;
    }
		
    //������Ϊ����ģʽ
    NRF2401_SetRXMode(); 
    while(1)
    {
			  if(counter > 20000)
				{
            NRF2401_SetTXMode(); //����Ϊ����ģʽ
            if(NRF2401_SendData("NRF2401 Test CHK60EVB!") == TX_OK)
            {
                UART_printf("Send Succ.\r\n");
            }
            else
            {
                UART_printf("Send Failed\r\n");
            }
						NRF2401_SetRXMode(); //����Ϊ����ģʽ
						counter = 0;
				}
				counter++;
        if(NRF2401_RecData(NRF2401RXBuffer) == NRF_OK) //���յ�������
        {
            UART_printf("DataRec:%s\r\n",NRF2401RXBuffer); //��ӡ����
        }
    }
}

