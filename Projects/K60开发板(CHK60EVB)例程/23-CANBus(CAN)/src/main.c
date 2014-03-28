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
* 1- CAN����ʵ�� ��Ҫһ��USB-CANת����
* 2- ����ID 10  ����ID 7
* 3- ��ʵ�������ѯ���ͽ���ģʽ ÿ��һ��ʱ�䷢��һ֡���� ͬ�½����յ�����Ϣ���ڴ�ӡ ��ҪUSB-CAN��λ�����
*/

CAN_TxMsgTypeDef CAN_TxMsg1;
CAN_RxMsgTypeDef CAN_RxMsg1;

int main(void)
{
    uint8_t i;
    uint32_t cnt = 0;
    CAN_InitTypeDef CAN_InitStruct1;
    //ʹ��CANʱ CoreClock ������96M
    SystemClockSetup(ClockSource_EX50M,CoreClock_96M);
    DelayInit();
    LED_Init(LED_PinLookup_CHK60EVB, kNumOfLED);
    UART_DebugPortInit(UART4_RX_C14_TX_C15, 115200);
    DisplayCPUInfo();
    //����CAN��ʼ����Ϣ
    CAN_InitStruct1.CANxMap = CAN1_TX_PE24_RX_PE25;      //PE24 PE25����
    CAN_InitStruct1.CAN_BaudRateSelect = CAN_SPEED_125K; //125K ������
    CAN_InitStruct1.FilterEnable = ENABLE;              //��ʹ�ù���ID����
    CAN_Init(&CAN_InitStruct1);
	  
    //���ý�����Ϣ
    CAN_RxMsg1.MBIndex = CAN_MB1;                  //��������ʹ��CAN_MB1
    CAN_RxMsg1.DLC = 8;                            //������8���ֽ� һ֡
    CAN_RxMsg1.Id = 7;                             //����ID
    CAN_EnableReceiveMB(CAN1, &CAN_RxMsg1);        //ʹ�ܽ�������
    //���÷�������
    CAN_TxMsg1.Id = 10;                          //����ID 10
    CAN_TxMsg1.DLC = 8;                          //ÿ֡8�ֽ�
    CAN_TxMsg1.IDE = CAN_IDE_Standard;           //��׼����֡
    CAN_TxMsg1.RTR = CAN_RTR_Data;               //����֡
    CAN_TxMsg1.MBIndex = CAN_MB2;                //ʹ������0
		
    //д���������  
    memcpy(CAN_TxMsg1.Data, "12345678", CAN_TxMsg1.DLC);
    while(1) 
    { 
        if (CAN_Receive(CAN1, &CAN_RxMsg1) == TRUE)
        {
					  //������ճɹ� ��ӡ��������
            UART_printf("ID:0x%x\r\n",CAN_RxMsg1.Id);
            UART_printf("Data: ");
            for(i = 0; i < CAN_RxMsg1.DLC; i++)
            {
                UART_printf("0x%x ", CAN_RxMsg1.Data[i]);
            }
            UART_printf("\r\n");
        }
        cnt++; 
        //����
        if(cnt == 1000000)
        {
            CAN_Transmit(CAN1, &CAN_TxMsg1);
            cnt = 0;
        }				
    }
}

