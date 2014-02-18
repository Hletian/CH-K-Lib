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
* 2- �ڱ������� ��DMA�����ڴ�֮��İ��� ��һƬ�ڴ������ ���Ƶ���һƬ�ڴ��� ��ҪΪ����ʾDMA����򵥹���
*/

uint8_t DMASrcBuffer[256]; //DMAԴ��ַ������
uint8_t DMADestBuffer[256];//DMA�����Ŀ�ĵ�ַ ������

int main(void)
{
    uint32_t i =0;
    DMA_InitTypeDef DMA_InitStruct1;
    SystemClockSetup(ClockSource_EX50M,CoreClock_100M);
    DelayInit();
    LED_Init(LED_PinLookup_CHKATOM, kNumOfLED);
    UART_DebugPortInit(UART4_RX_C14_TX_C15, 115200);
	  DisplayCPUInfo();
    //��������
    for(i=0;i<256;i++)
    {
        DMASrcBuffer[i] = i;
        DMADestBuffer[i] = 0;
    }
    DMA_InitStruct1.Channelx = DMA_CH0;                         //ʹ��DMA0ͨ��  (0-15)
    DMA_InitStruct1.DMAAutoClose = ENABLE;                      //������Ϻ��Զ��ر�
    DMA_InitStruct1.EnableState = ENABLE;                       //��ʼ����������ʼ����
    DMA_InitStruct1.MinorLoopLength = 256;                      //����256��
    DMA_InitStruct1.PeripheralDMAReq  = DMA_MUX2;               //����Ҫ����Դ �����Ŭ������
    DMA_InitStruct1.TransferBytes = 1;                          //ÿ�δ���һ���ֽ�
    //����Ŀ�ĵ�ַ�������
    DMA_InitStruct1.DestBaseAddr = (uint32_t)DMADestBuffer;     //ָ��Ŀ�ĵ�ַ
    DMA_InitStruct1.DestDataSize = DMA_DST_8BIT;                //����Ϊ1Byte
    DMA_InitStruct1.DestMajorInc = 0;                           //ִֻ��һ�δ�ѭ��
    DMA_InitStruct1.DestMinorInc = 1;                           //ÿ�δ������ַ+1
 
    //����Դ��ַ�������
    DMA_InitStruct1.SourceBaseAddr = (uint32_t)DMASrcBuffer;
    DMA_InitStruct1.SourceDataSize = DMA_SRC_8BIT;
    DMA_InitStruct1.SourceMajorInc = 0;
    DMA_InitStruct1.SourceMinorInc = 1;
    DMA_Init(&DMA_InitStruct1);
    NVIC_EnableIRQ(DMA0_IRQn); 
    DMA_ITConfig(DMA0, DMA_IT_MAJOR, DMA_CH0,ENABLE); //������������ж�
    //�ȴ��������
    while(DMA_IsComplete(DMA_CH0) == FALSE);
    UART_printf("DMADestBuffer:\r\n");
    for(i=0;i<256;i++)
    {
        UART_printf("%x ",DMADestBuffer[i]);
    }
	
    while(1)
    {
		
    }
}

