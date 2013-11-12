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

#include "bmp.h"


//���� ��˼���� Kinetis�̼�������
//�̼���汾 V2.4
//startup:        �����ļ�
//devices:        Ӳ��������Դ֧�ֿ�(����ʵ���õ�)
//utilities:      ���ģ��֧�ֿ�(����ʵ���õ�)
//drivers:        �̼���Դ��
//use:            �û����� ����mian������ �жϷ������ 

/*
* ����֪ʶ:
* 1- CHGUI֧��ͼƬ���뼰��ʾ
* 2- ����ͼƬ ��ͼƬ�û�ͼ�� ������С ������ 240*320 �� ���ΪBMPλͼ
*    ʹ�� BINtoC ���߽�ͼƬת��ΪC���� Ȼ�����GUI_BMP_Draw ����λͼ����ָ�뼴��
*/


CHGUI_CtrlOperation_TypeDef gILI9320_OperationStrcut1 = 
{
	LCD_Init,        
	NULL,
	LCD_DrawPoint,
	NULL,
	LCD_SetCursor,
	LCD_DrawHLine,
	LCD_DrawVLine,
	LCD_Fill,
	LCD_GetDeivceID,
};


//CHGUI��ʼ���ṹ
CHGUI_InitTypeDef CHGUI_InitStruct1 = 
{
    "CHK60", 
	   0,      
	   LCD_X_MAX, 
	   LCD_Y_MAX, 
    &gILI9320_OperationStrcut1,   //LCD�������������ӿ�������
};


int main(void)
{
    SystemClockSetup(ClockSource_EX50M,CoreClock_100M);
    DelayInit();
    LED_Init(LED_PinLookup_CHK60EVB, kNumOfLED);
    UART_DebugPortInit(UART4_RX_C14_TX_C15, 115200);
	  DisplayCPUInfo();
    GUI_Init(&CHGUI_InitStruct1);
    while(1) 
    {
			 GUI_BMP_Draw(0,0,BMP_DATA_TABLE);
    }
}

