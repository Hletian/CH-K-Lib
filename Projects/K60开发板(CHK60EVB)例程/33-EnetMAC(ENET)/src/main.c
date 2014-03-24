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

//MiniShell
#include "minishell.h"

//���� ��˼���� Kinetis�̼�������
//�̼���汾 V2.4
//startup:        �����ļ�
//devices:        ����оƬ֧�ֿ�(����ʵ���õ�)
//utilities:      ���ģ��֧�ֿ�(����ʵ���õ�)
//drivers:        �̼���Դ��

/*
* ����֪ʶ:
* 1- ENET ��̫��ģ�� Internet��������� ����ĵ�λΪ��̫֡
* 2- ��ʵ����Ҫ������̫������  ���Խ��������ӵ�PC�� PC�ᷢ����̫֡ �����ڴ����ϻ��ӡ�յ���֡������Ϣ
*/

//����MAC�����ַ����
//�������Ͷ���
//������ַ
uint8_t  gCfgLoca_MAC[] = {0x22, 0x22, 0x22, 0x00, 0x00, 0x01};

//��̫֡���ͻ�����
uint8_t  gBuffer[1520]; 

extern uint8_t gEnetFlag ;

int main(void)
{
    uint32_t len;
    uint32_t i;
    ENET_InitTypeDef ENET_InitStruct1;
    SystemClockSetup(ClockSource_EX50M,CoreClock_100M);
    DelayInit();
    LED_Init(LED_PinLookup_CHK60EVB, kNumOfLED);
    UART_DebugPortInit(UART4_RX_C14_TX_C15, 115200);
    DisplayCPUInfo();

    ENET_InitStruct1.pMacAddress = gCfgLoca_MAC;
    ENET_Init(&ENET_InitStruct1);
    if(ENET_MiiLinkState() == TRUE)
    {
        UART_printf("ENET Phy Connected succ.\r\n");
    }
    else
    {
        UART_printf("ENET Phy Connected failed.\r\n");
    }
    while(1) 
    {
        len = ENET_MacRecData(gBuffer);
        if(len)
        {
            UART_printf("Enet frame received, len:%d\r\n", len);
            //��ӡ���յ�������
            for(i = 0; i < len; i++)
            {
                UART_printf("0x%x ", gBuffer[i]);
            }
            UART_printf("\r\n");
        }
    }
}

