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

//fat32
#include "znfat.h"

#include <math.h>


//���� ��˼���� Kinetis�̼�������
//�̼���汾 V2.4
//startup:        �����ļ�
//devices:        ����оƬ֧�ֿ�(����ʵ���õ�)
//utilities:      ���ģ��֧�ֿ�(����ʵ���õ�)
//drivers:        �̼���Դ��

/*
* ����֪ʶ:
* 1- FAT32�ļ�ϵͳ ������ʹ�ù�����Znfat�ļ�ϵͳ 
* 2- Znfat��Դ������znfatĿ¼�� �����û�����
* 3- �������FAT�ļ�ϵͳ����Ϣ ����� Znfatʹ��˵��������������ļ�ϵͳһ��
* 4- ����ʵ��һ������򵥵� ������� �����ֲ�SD���� PIC�ļ��������е�BMPͼƬ
* 5- ʹ��ʱ �뽫�����������ļ����ڵ�PIC�ļ��п�����SD����Ŀ¼��
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

struct znFAT_Init_Args initArgSD; //��ʼ����������
struct znFAT_Init_Args initArgFlash; //��ʼ����������
struct FileInfo fileInfo1,fileInfo2; //�ļ���Ϣ����
struct DateTime dtInfo1; //����ʱ��ṹ�����

static uint8_t gBMPBuffer[512];
//BMP ��ʾ���� �û��ص����� ���ڶ�ȡ����
//ʵ���Ͼ��Ƕ�SD���е�����
static uint32_t AppGetData(uint32_t ReqestedByte, uint32_t Offset, uint8_t **ppData)
{
    //����ʵ�ʿ��Զ�ȡ�����ݳ���
    if(ReqestedByte > sizeof(gBMPBuffer))
		{
			ReqestedByte = sizeof(gBMPBuffer);
		}
		//��ȡ����
    znFAT_ReadData(&fileInfo1 ,Offset, ReqestedByte, gBMPBuffer);
		//��������ָ��λ��
    *ppData = gBMPBuffer;
		//����ʵ�ʶ����ĳ���
    return ReqestedByte;
}


int main(void)
{
    uint8_t bmp_pic_cnt = 0;
    uint32_t i = 0;
    uint8_t res;
    SD_InitTypeDef SD_InitStruct1;
    SystemClockSetup(ClockSource_EX50M,CoreClock_100M);
    DelayInit();
    LED_Init(LED_PinLookup_CHK60EVB, kNumOfLED);
    UART_DebugPortInit(UART4_RX_C14_TX_C15, 115200);
    DisplayCPUInfo();
	
    GUI_Init(&CHGUI_InitStruct1);
    GUI_SetBkColor(BLACK);
    GUI_SetTextMode(GUI_TEXTMODE_NORMAL);
    GUI_SetColor(LGRAY);
    GUI_SetFontFormName("FONT_CourierNew");
    GUI_printf("HelloWorld\r\n");
    GUI_printf("CHGUI_Version:%0.2f\r\n", (float)(GUI_VERSION/100));
    GUI_printf("ID:%X\r\n", GUI_GetDeivceID());
    GUI_printf("Please Insert Card...\r\n");
    DelayMs(200);
    SD_InitStruct1.SD_BaudRate = 2000000;
    //�ȴ�SD����ʼ���ɹ�
    while(SD_Init(&SD_InitStruct1) != ESDHC_OK);
    //��ʼ��FAT32
    znFAT_Device_Init(); //�豸��ʼ��
    znFAT_Select_Device(0,&initArgSD); //ѡ��SD���豸
    res = znFAT_Init();
    if(res == ERR_SUCC)
    {
        GUI_printf("Suc. to init FS\r\n");
        GUI_printf("BPB_Sector_No:%d\r\n",initArgSD.BPB_Sector_No);   
        GUI_printf("Total_SizeKB:%d\r\n",initArgSD.Total_SizeKB); 
        GUI_printf("BytesPerSector:%d\r\n",initArgSD.BytesPerSector); 
        GUI_printf("FATsectors:%d\r\n",initArgSD.FATsectors);  
        GUI_printf("SectorsPerClust:%d\r\n",initArgSD.SectorsPerClust); 
        GUI_printf("FirstFATSector:%d\r\n",initArgSD.FirstFATSector); 
        GUI_printf("FirstDirSector:%d\r\n",initArgSD.FirstDirSector); 
        GUI_printf("FSsec:%d\r\n",initArgSD.FSINFO_Sec);
        GUI_printf("Next_Free_Cluster:%d\r\n",initArgSD.Next_Free_Cluster);
        GUI_printf("FreenCluster:%d\r\n",initArgSD.Free_nCluster); 
    }
    else
    {
        GUI_printf("FAT32 Init failed CODE:%d",res);
        while(1);
    }
    //ȷ���ж���BMP
    while(znFAT_Open_File(&fileInfo1, "/PIC/*.bmp", bmp_pic_cnt, 1) == ERR_SUCC)
    {
        bmp_pic_cnt++;
    }
    while(1)
    {
        //��SD���е�BMP�ļ�
        znFAT_Open_File(&fileInfo1, "/PIC/*.bmp", i, 1);
        //��ͼ
        GUI_BMP_DrawEx(0,0, AppGetData);
        //�ر��ļ�
        znFAT_Close_File(&fileInfo1);
        //ͣ��һ��
        DelayMs(1000);
        i++;
        if(i == bmp_pic_cnt) i = 0;
        LED_Toggle(kLED1);
    }
}

