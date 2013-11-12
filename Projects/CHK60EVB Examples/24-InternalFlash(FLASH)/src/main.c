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


//���� ��˼���� Kinetis�̼�������
//�̼���汾 V2.4
//startup:        �����ļ�
//devices:        Ӳ��������Դ֧�ֿ�(����ʵ���õ�)
//utilities:      ���ģ��֧�ֿ�(����ʵ���õ�)
//drivers:        �̼���Դ��
//use:            �û����� ����mian������ �жϷ������ 

/*
* ����֪ʶ:
* 1- �ڲ�Flash������ ���Կ���д��оƬ�ڲ�Flash ����ҪС��д�� д��ʱ���ܱ��жϴ�ϻ�д�� ������
* 2- д�������Sector Ϊ��λ д��һ�λ�д������Sector ��Ҫ��ҪFLASH_WriteByte����������
*/

static uint8_t flash_buffer[2048];	 //���ݻ�����
/***********************************************************************************************
 ���ܣ�д��Flash����
 �βΣ�FlashStartAdd:��ʼ��ַ
    NumByteToWrite  :��Ҫд������ݳ���
	        u8 *buffer: ����ָ��
 ���أ�0
 ��⣺�û��ӿں���
************************************************************************************************/
void FLASH_WriteByte(uint32_t FlashStartAdd,uint32_t NumByteToWrite,uint8_t *buffer)
{
	uint32_t i=0;
	uint32_t sectorNo = FlashStartAdd/(1<<11);        //�õ�ַ���ڵ�����
	uint32_t offset =   FlashStartAdd%(1<<11);        //�õ�ַ����������ƫ��
	uint32_t remain =   (1<<11)-offset;               //����ʣ���С
	if(NumByteToWrite<=remain)remain = NumByteToWrite;//������4096���ֽ�
	while(1)
	{
		FLASH_ReadByte(sectorNo*(1<<11),(1<<11),flash_buffer);  //ȡ����������������
		FLASH_EraseSector(sectorNo);             //��������
		for(i=0;i<remain;i++)
		{
			flash_buffer[offset+i]=buffer[i];	   //�޸�����
		}
		FLASH_WriteSector(sectorNo,(1<<11),flash_buffer);	//д����
		if(NumByteToWrite == remain) break;//д�������
		else
		{
			sectorNo++;              //������ַ��1
			offset=0;                //ƫ��λ��Ϊ0 	
		  buffer+=remain;       //ָ��ƫ��
			FlashStartAdd+=remain;    //д��ַƫ��	  
			NumByteToWrite-=remain;				//�ֽ����ݼ�
			if(NumByteToWrite > (1<<11)) remain = (1<<11);	//��һ����������д����
			else remain=NumByteToWrite;			//��һ����������д����
		}
	}
}

//���Բ���
#define FLASH_TEST_BUFFER_SIZE   (64)
#define FLASH_TEST_WRITE_ADDRESS (64)
//TEST����
void flashTest(void)
{
    uint32_t i;
    uint8_t test_buffer[FLASH_TEST_BUFFER_SIZE];
    FLASH_Init();
    memset(test_buffer, 'A', FLASH_TEST_BUFFER_SIZE);
    FLASH_WriteByte(CPUInfo.PFlashSize-FLASH_TEST_WRITE_ADDRESS, FLASH_TEST_BUFFER_SIZE, test_buffer);  //д��100���� 0ƫ�� д��256�ֽ�
    memset(test_buffer, 0, FLASH_TEST_BUFFER_SIZE);      //�������
    FLASH_ReadByte(CPUInfo.PFlashSize-FLASH_TEST_WRITE_ADDRESS, FLASH_TEST_BUFFER_SIZE, test_buffer);  //��ȡ����
    for(i = 0; i < FLASH_TEST_BUFFER_SIZE; i++)
    {
		    if(test_buffer[i] != 'A') //���ݳ���
        {
            UART_printf("FlashTestErr:[%d]:%d It should be A\r\n", i, test_buffer[i]);
		    }
    }
		UART_printf("Flash Test Completed\r\n");
}

int main(void)
{
	
    SystemClockSetup(ClockSource_EX50M,CoreClock_100M);
    DelayInit();
    LED_Init(LED_PinLookup_CHK60EVB, kNumOfLED);
    UART_DebugPortInit(UART4_RX_C14_TX_C15, 115200);
	  DisplayCPUInfo();
	  //��ʼ���ڲ�Flash������
    flashTest();
    
    while(1) 
    {
			  DelayMs(500);
    }
}

