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


//���� ��˼���� Kinetis�̼�������
//�̼���汾 V2.4
//startup:        �����ļ�
//devices:        Ӳ��������Դ֧�ֿ�(����ʵ���õ�)
//utilities:      ���ģ��֧�ֿ�(����ʵ���õ�)
//drivers:        �̼���Դ��
//use:            �û����� ����mian������ �жϷ������ 

/*
* ����֪ʶ:
* 1- SPI Flash ����SPIFLASH ʹ��SPI�ӿ�ͨѶ �ʺϴ洢һЩ�ֿ��ļ��ȵ� 
* 2- spiflash.c spiflash.h ���ù̼����SPI�����ṩ�˻���FLASH�������� ���� ��д ��ʼ�� �ȵ� 
*/

//SPIFLASH ���Ժ���
#define SPIFLASH_TEST_ADDR   (0x1000)
#define SPIFLASH_TEST_SIZE   (64)

//SPIFLASH �������Ժ���
static void SPIFLASH_Test(void)
{
    uint32_t i;
    uint8_t test_buffer[SPIFLASH_TEST_SIZE];
    SPI_FLASH_Read(test_buffer, SPIFLASH_TEST_ADDR, sizeof(test_buffer));
    UART_printf("Read data form 0x%x:\r\n", SPIFLASH_TEST_ADDR);
    for(i = 0; i < SPIFLASH_TEST_SIZE; i++)
    {
        UART_printf("[%x]:0x%x ", i, test_buffer[i]);
    }
		//д���������
		for(i = 0; i < SPIFLASH_TEST_SIZE; i++)
		{
        test_buffer[i] = i;
		}
		UART_printf("\r\n");
		//д������
		SPI_FLASH_Write(test_buffer, SPIFLASH_TEST_ADDR, sizeof(test_buffer));
		UART_printf("Write data completed\r\n");
		memset(test_buffer, 0, sizeof(test_buffer));
		SPI_FLASH_Read(test_buffer, SPIFLASH_TEST_ADDR, sizeof(test_buffer));
    UART_printf("Read data form 0x%x:\r\n", SPIFLASH_TEST_ADDR);
		//��ӡ����
    for(i = 0; i < SPIFLASH_TEST_SIZE; i++)
    {
        UART_printf("[%x]:0x%x ", i, test_buffer[i]);
    }
}

int main(void)
{
    SystemClockSetup(ClockSource_EX50M,CoreClock_100M);
    DelayInit();
    LED_Init(LED_PinLookup_CHK60EVB, kNumOfLED);
    UART_DebugPortInit(UART4_RX_C14_TX_C15, 115200);
	  DisplayCPUInfo();
	  //��ʼ��SPIFlash
    SPI_FLASH_Init();
	  UART_printf("SPI Flash Size:%dKB\r\n", SPI_FLASH_GetCapacity()/1024);
    //����SPIFLASH
	  SPIFLASH_Test();
    while(1) 
    {		
    }
}

