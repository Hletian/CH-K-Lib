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

//nrf24xx
#include "nrf2401.h"

//���� ��˼���� Kinetis�̼�������
//�̼���汾 V2.4
//startup:        �����ļ�
//devices:        Ӳ��������Դ֧�ֿ�(����ʵ���õ�)
//utilities:      ���ģ��֧�ֿ�(����ʵ���õ�)
//drivers:        �̼���Դ��
//use:            �û����� ����mian������ �жϷ������ 

/*
* ����֪ʶ:
* 1- DMA ���ǰ��˹� ������CPU����Ԥ������°��˺������� �ر������ڴ�����������
* 2- �ڱ������� ��DMA�����ڴ��ڷ������ݴ��� �� ���ڴ����ݴ��͵� ���ڷ��ͼĴ��� ʵ�ִ��书��
*/

uint8_t UARTSendBuffer[] = "This string is send through DMA\r\n";

int main(void)
{
    DMA_InitTypeDef DMA_InitStruct1;
    SystemClockSetup(ClockSource_EX50M,CoreClock_100M);
    DelayInit();
    LED_Init(LED_PinLookup_CHKATOM, kNumOfLED);
    UART_DebugPortInit(UART4_RX_C14_TX_C15, 115200);
	  DisplayCPUInfo();
    //����UART �����ж�/DMA����
    UART_ITConfig(UART4,UART_IT_TDRE,ENABLE);
    //ʹ��UART4 TX DMA����
    UART_DMACmd(UART4,UART_DMAReq_Tx,ENABLE);
    DMA_InitStruct1.Channelx = DMA_CH0;                         //ʹ��DMA0ͨ��  (0-15)
    DMA_InitStruct1.DMAAutoClose = ENABLE;                      //������Ϻ��Զ��ر�
    DMA_InitStruct1.EnableState = ENABLE;                       //��ʼ����������ʼ����
    DMA_InitStruct1.MinorLoopLength = sizeof(UARTSendBuffer);   //�������
    DMA_InitStruct1.PeripheralDMAReq  = UART4_TRAN_DMAREQ;      //UART ������ɴ���
    DMA_InitStruct1.TransferBytes = 1;                          //ÿ�δ���һ���ֽ�
    //����Ŀ�ĵ�ַ�������
    DMA_InitStruct1.DestBaseAddr = (uint32_t)&(UART4->D);       //ָ��Ŀ�ĵ�ַ
    DMA_InitStruct1.DestDataSize = DMA_DST_8BIT;                //����Ϊ1Byte
    DMA_InitStruct1.DestMajorInc = 0;                           //ִ��һ�δ�ѭ���� ��ַ������
    DMA_InitStruct1.DestMinorInc = 0;                           //ÿ�δ������ַ������
 
    //����Դ��ַ�������
    DMA_InitStruct1.SourceBaseAddr = (uint32_t)UARTSendBuffer;
    DMA_InitStruct1.SourceDataSize = DMA_SRC_8BIT;
    DMA_InitStruct1.SourceMajorInc = 0;
    DMA_InitStruct1.SourceMinorInc = 1;                         //ÿ�δ���1���ֽں��ַ�Զ���1 ����������ݼ�������
    DMA_Init(&DMA_InitStruct1);
    //�ȴ��������
    while(DMA_IsComplete(DMA_CH0) == FALSE);
    while(1)
    {
		
    }
}

