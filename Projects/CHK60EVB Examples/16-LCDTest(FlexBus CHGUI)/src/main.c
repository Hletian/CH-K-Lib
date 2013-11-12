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
//LCD Devices
#include "lcd_chk60evb.h"  //����� LCD����
#include "chgui.h"         //CHGUI�������
#include "chgui_char.h"    //CHGUI�ַ���ʾ���
#include "chgui_bmp.h"     //CHGUI BMPͼƬ�������


//���� ��˼���� Kinetis�̼�������
//�̼���汾 V2.4
//startup:        �����ļ�
//devices:        Ӳ��������Դ֧�ֿ�(����ʵ���õ�)
//utilities:      ���ģ��֧�ֿ�(����ʵ���õ�)
//drivers:        �̼���Դ��
//use:            �û����� ����mian������ �жϷ������ 

/*
* ����֪ʶ:
* 1-LCD��GUI LCD:TFT��ɫҺ�� GUI:ͼ���û��ӿ� �������ʾһ��������
* 2-CHGUI: ���˿�����CHGUI��� Ŀǰ��ʵ�� ����2D�� �ַ�����ʾ�Լ�BMPͼƬ����
* 3-CHGUI:�÷� : CHGUI ͨ��һ��ص�����ָ�� ��ɳ�ʼ��
* ����ʵ�廯һ�� �����ṹCHGUI_CtrlOperation_TypeDef Ȼ������ײ�д�õ�LCD�������� ����Ҫ����LCD_DrawPoint
* ֻҪ��֤�ײ�ӿں�����ȷ CHGUI�Ϳ�������������ƽ̨�� ��Ӳ���޹�
* ��ɲ����ӿں������Ӻ� �ڶ���CHGUI_InitTypeDef ����
* LCD�豸�� - Ŀǰ����
* LCD�豸�� - Ŀǰ����
* LCDX�������ߴ�
* LCDY�������ߴ�
* �ײ㺯�����Ӳ����ṹ��ָ��
* Ȼ�����CHGUI_Init(&CHGUI_InitStruct1);����
* CHGUI����ĺ����ӿ���ch_gui.h chgui_char.h ��  ucgui_bmp.h���н��� ��Ȼ���������Ҳ����
* ������л��������� ��lcd_chk60evb.h�� ����� #define LCD_USE_HORIZONTAL   ����
*/

//�ײ�������ӽṹ Ŀǰ�İ汾ֻ��ʵ��LCD_DrawPoint�� LCD_Init�Ϳ��Թ�����������NULL�Ϳ���
CHGUI_CtrlOperation_TypeDef gILI9320_OperationStrcut1 = 
{
	LCD_Init,        //�ײ㺯����CHGUI�޹� LCD_Init��ʵ���� LCD_CHK60EVB.c�� ��ͬ
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
    "CHK60", //�豸����
	   0,      //�豸�� �粻��ȷ��0
	   LCD_X_MAX, //LCD X�������ֵ ����
	   LCD_Y_MAX, //LCD Y�������ֵ ����
    &gILI9320_OperationStrcut1,
};


int main(void)
{
    //��ʼ��ϵͳʱ�� ʹ���ⲿ50M���� PLL��Ƶ��100M
    SystemClockSetup(ClockSource_EX50M,CoreClock_100M);
    DelayInit();
    LED_Init(LED_PinLookup_CHK60EVB, kNumOfLED);
    UART_DebugPortInit(UART4_RX_C14_TX_C15, 115200);
	  DisplayCPUInfo();
    //��ʼ��GUI
    GUI_Init(&CHGUI_InitStruct1);
	  //���ñ���ɫΪ��ɫ
    GUI_SetBkColor(BLACK);
	  //��ͨ�ַ���ʾģʽ
	  GUI_SetTextMode(GUI_TEXTMODE_NORMAL);
	  //����ǰ��ɫΪ��ɫ
    GUI_SetColor(LGRAY);
	  //����ǰ��������ΪCourierNew
    GUI_SetFontFormName("FONT_CourierNew");
	  //��ӡ�ַ�
		GUI_printf("HelloWorld\r\n");
	  //��ӡCHGUI�汾��
		GUI_printf("CHGUI_Version:%0.2f\r\n", (float)(GUI_VERSION/100));
	  //��ӡLCDID
		GUI_printf("ID:%X\r\n", GUI_GetDeivceID());
    while(1) 
    {

    }
}

