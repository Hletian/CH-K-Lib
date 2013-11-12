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
* 1- FAT32�ļ�ϵͳ ������ʹ�ù�����Znfat�ļ�ϵͳ �����ʼ���ɹ� �ᴴ��һ��record.csv���ļ���¼�������� ��ȡ��ɺ��ö�������PC�ϲ鿴
* 2- Znfat��Դ������znfatĿ¼�� �����û�����
* 3- �������FAT�ļ�ϵͳ����Ϣ ����� Znfatʹ��˵��������������ļ�ϵͳһ��
* 4- CSV�ļ�����Execl��
*/

struct znFAT_Init_Args initArgSD; //��ʼ����������
struct znFAT_Init_Args initArgFlash; //��ʼ����������
struct FileInfo fileInfo1,fileInfo2; //�ļ���Ϣ����
struct DateTime dtInfo1; //����ʱ��ṹ�����
//��ʵ��궨��
#define PI  (3.1415)
//��ʵ��ROM������
const uint8_t TextTitle[] = "���,��������1,��������2\r\n";
uint8_t recordBuffer[20];
int main(void)
{
    float cnt = 0;
    uint32_t i;
    uint8_t res;
    SD_InitTypeDef SD_InitStruct1;
    SystemClockSetup(ClockSource_EX50M,CoreClock_100M);
    DelayInit();
    LED_Init(LED_PinLookup_CHKATOM, kNumOfLED);
    UART_DebugPortInit(UART4_RX_C14_TX_C15, 115200);
	  DisplayCPUInfo();
	  UART_printf("Please Insert Card\r\n");
    SD_InitStruct1.SD_BaudRate = 2000000;
    //�ȴ�SD����ʼ���ɹ�
    while(SD_Init(&SD_InitStruct1) != ESDHC_OK);
    //��ʼ��FAT32
    znFAT_Device_Init(); //�豸��ʼ��
    znFAT_Select_Device(0,&initArgSD); //ѡ��SD���豸
    res = znFAT_Init();
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
        UART_printf("FAT32 Init failed CODE:%d",res);
        while(1);
    }
    res = znFAT_Delete_File("/RECORD/record.csv");
    if(res != ERR_SUCC) 
    {
        UART_printf("File delate failed!, CODE:%d\r\n",res);
    }
    else
    {
        UART_printf("/RECORD/record.csv deleted succ.\r\n");
    }
    UART_printf("Creating Dir...");
    res = znFAT_Create_Dir("/RECORD/",&dtInfo1); //����Ŀ¼ 
    if((res != ERR_SUCC) && (res != ERR_DIR_ALREADY_EXISTING)) 
    {
        UART_printf("Fail CODE:%d\r\n",res);
        while(1);
    }
    UART_printf("Creating Files...");
    res = znFAT_Create_File(&fileInfo1,"/RECORD/record.csv",&dtInfo1); //�����ļ� �ļ�������ʹ��8.3���ļ���
    if(res != ERR_SUCC) 
    {
        UART_printf("Failed CODE:%d\r\n",res);
        while(1);
    }
    UART_printf("Writing data...\r\n");
    znFAT_WriteData(&fileInfo1,sizeof(TextTitle),(unsigned char *)TextTitle); //��ӡ��ͷ
	
    //д������
    for(cnt=0;cnt<(2*PI);cnt+=0.05)
    {
        i++;
        sprintf((char *)recordBuffer,"%d,%1.3f,%1.3f\r\n",i,sin(cnt),cos(cnt)); //д��sin cos
        UART_printf((char *)recordBuffer);
        znFAT_WriteData(&fileInfo1,strlen((char *)recordBuffer),(unsigned char *)recordBuffer);
    }
    UART_printf("Writing Completed please see CSV file on PC!\r\n");
    znFAT_Close_File(&fileInfo1); //��һ���ļ��󣬱���ر�
    znFAT_Flush_FS();  //����ˢ��ϵͳʹ�ļ���Ч
    while(1)
    {
		
    }
}

