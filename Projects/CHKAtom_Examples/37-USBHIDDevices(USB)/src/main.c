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
//SPI Flash Devices;
#include "spiflash.h"
//LCD Devices
#include "spilcd.h"  

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
* 1- ʵ��ͬ�� ����ʹ��USBЭ�鴫��HID���Զ������� ������ɺ���Ҫ��������ļ����е���λ���������
*/




int main(void)
{
	  //ʹ��USB ����CoreClock = 96M
    uint8_t usb_hid_send_buf[8] ={0,0,0,0,0,0};
    uint8_t usb_hid_rec_buf[8];
    uint8_t usb_hid_rec_cnt = 0; //����֡����
    SystemClockSetup(ClockSource_EX50M,CoreClock_96M);
    DelayInit();
    LED_Init(LED_PinLookup_CHKATOM, kNumOfLED);
    KBI_Init(KBI_PinLookup_CHKATOM, kNumOfKEY);
    UART_DebugPortInit(UART4_RX_C14_TX_C15, 115200);
    DisplayCPUInfo();
	     
    UART_printf("Waitting for USB connect ...\r\n");
    USB_Init(); //��ʼ��USB�豸 
    USB_WaitDeviceEnumed(); //�ȴ�USBö�ٳɹ�
	  UART_printf("USB connected!\r\n");
	
    while(1) 
    {
        KBI_Scan();
        usb_hid_send_buf[0] = KBI_GetKeyValue(kKEY1);
        usb_hid_send_buf[1] = KBI_GetKeyValue(kKEY1);	
        USB_HID_SendData(usb_hid_send_buf,8);  //��������
        if(USB_HID_RecData(usb_hid_rec_buf) != 0)  //���յ�������
        {
            LED_Ctrl(kLED1, (usb_hid_rec_buf[0]&0x01)>>0);
            LED_Ctrl(kLED2, (usb_hid_rec_buf[0]&0x02)>>1);		
            usb_hid_rec_cnt++;
					  UART_printf("USB HID Data Received\r\n");
        }
        HID_Proc(); //ִ��HID����
			  DelayMs(KBI_SCAN_PERIOD_IN_US/1000); //��ʱ
    }
}
