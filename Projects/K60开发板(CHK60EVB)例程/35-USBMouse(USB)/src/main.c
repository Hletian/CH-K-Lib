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

//usb
#include "usb.h"
#include "usb_hid.h"

//���� ��˼���� Kinetis�̼�������
//�̼���汾 V2.4
//startup:        �����ļ�
//devices:        ����оƬ֧�ֿ�(����ʵ���õ�)
//utilities:      ���ģ��֧�ֿ�(����ʵ���õ�)
//drivers:        �̼���Դ��

/*
* ����֪ʶ:
* 1- ʹ��USB�е�HIDЭ�� ����ѧ�����豸 ģ��һ��USB��꣬��SHELL������mouse r ���������ƶ� ��������
* 2- ע��ʹ��USB BusCLock������48*nM  �������� ����48M
*/

//ʵ��MINISHELL����Ҫ��PutChar����
static void Putc(uint8_t data)
{
	UART_SendData(UART4, data);
}
//ʵ��MINISHELL����Ҫ��GetChar����
static uint8_t Getc(void)
{
	uint8_t ch;
  while(UART_ReceiveData(UART4, &ch) == FALSE);
	return ch;
}

//�û����� LED����
int MouseControl(int argc, char *argv[])
{
    //��������ֻ��2��(�ո�Ϊ�ָ���)   
    if(argc == 2)
		{
        if(!strcmp(argv[1], "r"))
				{
					USB_HID_SetMouse(20,0,0,0);
				}
        if(!strcmp(argv[1], "l"))
				{
					USB_HID_SetMouse(-20,0,0,0);
				}	  
        if(!strcmp(argv[1], "u"))
				{
					USB_HID_SetMouse(0,-20,0,0);
				}
        if(!strcmp(argv[1], "d"))
				{
					USB_HID_SetMouse(0,20,0,0);
				}	  
		}
		else
		{
        MINISHELL_printf("mouse r(l)(u)(d)\r\n");
        MINISHELL_printf("Ex:mouse r (Move Mouse Left)\r\n");
		}
		return 0;
}

//ע�� MiniShell������װ�ṹ
MINISHELL_InstallTypeDef MiniShell_InstallStruct1 = 
{
    .ctrl_putchar = Putc,
    .ctrl_getchar = Getc,
};

//�û������ṹ
MINISHELL_CommandTableTypeDef MyCommand1 =
{
    .name = "mouse",
    .maxargs = 2,
    .cmd = MouseControl,
    .usage = "mouse r(l)(u)(d)",
};



int main(void)
{
	  //ʹ��USB ����CoreClock = 96M
    SystemClockSetup(ClockSource_EX50M,CoreClock_96M);
    PIT_InitTypeDef PIT_InitStruct1;
    DelayInit();
    LED_Init(LED_PinLookup_CHK60EVB, kNumOfLED);
    UART_DebugPortInit(UART4_RX_C14_TX_C15, 115200);
	  DisplayCPUInfo();
	     
	  UART_printf("Waitting for USB connect ...\r\n");
    USB_Init(); //��ʼ��USB�豸 
    USB_WaitDeviceEnumed(); //�ȴ�USBö�ٳɹ�
	  UART_printf("USB connected!\r\n");
	
	  //������ʱ��
	  PIT_InitStruct1.PITx = PIT0;
	  PIT_InitStruct1.PIT_Interval = 10;
	  PIT_Init(&PIT_InitStruct1);
	  NVIC_EnableIRQ(PIT0_IRQn);
	  PIT_ITConfig(PIT0, PIT_IT_TIF, ENABLE);
	  //����SHELL
    MINISHELL_Install(&MiniShell_InstallStruct1);
		MINISHELL_Register(&MyCommand1, 1);
	  MINISHELL_Init();
	  //����SHELL
	  MINISHELL_CmdHandleLoop("SHELL>>");
    while(1) 
    {
    }
}

