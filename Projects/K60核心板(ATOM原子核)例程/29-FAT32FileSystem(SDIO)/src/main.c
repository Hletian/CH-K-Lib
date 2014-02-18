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

//fat32
#include "znfat.h"

//���� ��˼���� Kinetis�̼�������
//�̼���汾 V2.4
//startup:        �����ļ�
//devices:        ����оƬ֧�ֿ�(����ʵ���õ�)
//utilities:      ���ģ��֧�ֿ�(����ʵ���õ�)
//drivers:        �̼���Դ��

/*
* ����֪ʶ:
* 1- FAT32�ļ�ϵͳ ������ʹ�ù�����Znfat�ļ�ϵͳ �����ʼ���ɹ� ���ӡFAT����Ϣ
* 2- Znfat��Դ������znfatĿ¼�� �����û�����
* 3- �������FAT�ļ�ϵͳ����Ϣ ����� Znfatʹ��˵��������������ļ�ϵͳһ��
*/

struct znFAT_Init_Args initArgSD; //��ʼ����������
struct znFAT_Init_Args initArgFlash; //��ʼ����������
struct FileInfo fileInfo1,fileInfo2; //�ļ���Ϣ����
struct DateTime dtInfo1; //����ʱ��ṹ�����

int main(void)
{
    uint8_t res;
    SD_InitTypeDef SD_InitStruct1;
    SystemClockSetup(ClockSource_EX50M,CoreClock_100M);
    DelayInit();
    LED_Init(LED_PinLookup_CHKATOM, kNumOfLED);
    UART_DebugPortInit(UART4_RX_C14_TX_C15, 115200);
	  DisplayCPUInfo();
	  UART_printf("Please Insert Card\r\n");
    SD_InitStruct1.SD_BaudRate = 2000000; //SD��������
    //�ȴ�SD����ʼ���ɹ�
    while(SD_Init(&SD_InitStruct1) != ESDHC_OK);
    //��ʼ��FAT32
    znFAT_Device_Init(); //�豸��ʼ��
    znFAT_Select_Device(0,&initArgSD); //ѡ��SD���豸
    res = znFAT_Init(); //��ʼ��Znfat
    if(res == ERR_SUCC)
    {
        UART_printf("Suc. to init FS\r\n");
        UART_printf("BPB_Sector_No:%d\r\n",initArgSD.BPB_Sector_No);   
        UART_printf("Total_SizeKB:%d\r\n",initArgSD.Total_SizeKB); 
        UART_printf("BytesPerSector:%d\r\n",initArgSD.BytesPerSector); 
        UART_printf("FATsectors:%d\r\n",initArgSD.FATsectors);  
        UART_printf("SectorsPerClust:%d\r\n",initArgSD.SectorsPerClust); 
        UART_printf("FirstFATSector:%d\r\n",initArgSD.FirstFATSector); 
        UART_printf("FirstDirSector:%d\r\n",initArgSD.FirstDirSector); 
        UART_printf("FSsec:%d\r\n",initArgSD.FSINFO_Sec);
        UART_printf("Next_Free_Cluster:%d\r\n",initArgSD.Next_Free_Cluster);
        UART_printf("FreenCluster:%d\r\n",initArgSD.Free_nCluster); 
    }
    else
    {
        UART_printf("FAT32��ʼ��ʧ�� ����:%d",res);
        while(1);
    }
    while(1) {};
}

