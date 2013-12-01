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

#include "chgui.h"

//���� ��˼���� Kinetis�̼�������
//�̼���汾 V2.4
//startup:        �����ļ�
//devices:        Ӳ��������Դ֧�ֿ�(����ʵ���õ�)
//utilities:      ���ģ��֧�ֿ�(����ʵ���õ�)
//drivers:        �̼���Դ��
//use:            �û����� ����mian������ �жϷ������ 
//LCD Devices
#include "spilcd.h"  //����� LCD����
#include "chgui.h"         //CHGUI�������
#include "chgui_char.h"    //CHGUI�ַ���ʾ���
#include "chgui_bmp.h"     //CHGUI BMPͼƬ�������


#include "ov7620.h"
CHGUI_CtrlOperation_TypeDef gILI9320_OperationStrcut1 = 
{
	SPILCD_Init,        //�ײ㺯����CHGUI�޹� LCD_Init��ʵ���� LCD_CHK60EVB.c�� ��ͬ
	NULL,
	SPILCD_DrawPoint,
	NULL,
	NULL,
	NULL,
	NULL,
	SPILCD_Fill,
	NULL,
};

//CHGUI��ʼ���ṹ
CHGUI_InitTypeDef CHGUI_InitStruct1 = 
{
    "CHK60", //�豸����
    0,      //�豸�� �粻��ȷ��0
    SPILCD_W, //LCD X�������ֵ ����
    SPILCD_H, //LCD Y�������ֵ ����
    &gILI9320_OperationStrcut1,
};
void I2C_ConfigOV7620(void);
void DisplayImage_WithSPILCD(uint16_t x,uint16_t y,uint8_t mode);
uint8_t StrBuffer[30];
int main(void)
{
    uint8_t i = 0;
    I2C_InitTypeDef I2C_InitStruct1;
    I2C_InitStruct1.I2CxMAP = I2C1_SCL_PC10_SDA_PC11;  //Use I2C1,PC10,PC11 to config SCCB
    I2C_InitStruct1.I2C_ClockSpeed = I2C_CLOCK_SPEED_200KHZ;
    //��ʼ��ϵͳʱ�� ʹ���ⲿ50M���� PLL��Ƶ��100M
    SystemClockSetup(ClockSource_EX50M,CoreClock_100M);
    DelayInit();
	   //��ʼ��GUI
    GUI_Init(&CHGUI_InitStruct1);
    //���ñ���ɫΪ��ɫ
    GUI_SetBkColor(BLACK);
    //��ͨ�ַ���ʾģʽ
    GUI_SetTextMode(GUI_TEXTMODE_NORMAL);
    //����ǰ��ɫΪ��ɫ
    GUI_SetColor(LGRAY);
    //LED_Init(LED_PinLookup_CHKATOM, kNumOfLED);
    //KBI_Init(KBI_PinLookup_CHKATOM, kNumOfKEY);
    UART_DebugPortInit(UART4_RX_C14_TX_C15, 115200);
    UART_printf("HelloWorld\r\n");
    DisplayCPUInfo();
    I2C_Init(&I2C_InitStruct1);
    I2C_ConfigOV7620();
    OV7620_Init();
    UART_printf("OV7620 TEST");
    EnableInterrupts();
    OV7620_StartTransfer();
    while(1)
    {
        if(OV7620Dev.TransferCompleteFlag == TRUE)
        {
            DisplayImage_WithSPILCD(5,15,1);        //��ʾͼ��
            OV7620_StartTransfer();                 //��ʼһ������
            OV7620Dev.TransferCompleteFlag = FALSE; //��Ҫ��������һ�����ݺ󽫱�־λ��0
        }
    }
}



//һ���ǽ�ͼ����ʾ��LCD�ϵĺ���

//RGB ��ɫ
uint16_t RGB2COLOR(uint8_t RR,uint8_t GG,uint8_t BB)
{
  return (((RR/8)<<11)+((GG/8)<<6)+BB/8); 
}

//��ʾͼ�� ͨ��SPILCD
void DisplayImage_WithSPILCD(uint16_t x,uint16_t y,uint8_t mode)
{
	uint16_t i,j; 
	//LCD���ô�����ʽ��㣬��׷������ٶ�
	SPILCD_SetWindow(x,x+CCD_IMAGE_W-1,y,y+CCD_IMAGE_H+1);									
	SPILCD_WriteRAM_Prepare();        //��ʼд��GRAM
		for(i=0;i<CCD_IMAGE_H;i++)
		{
		 	for(j=0;j<CCD_IMAGE_W;j++)
			{		
				//��ֵ��
				if(mode == 0)	
				{
				   if(CCD_Image[i][j] > 128) 
					 {
						 SPILCD_WriteRAM(WHITE);
					 }
					 else
					 {
						 SPILCD_WriteRAM(BLACK);
					 }
				} 
				//�Ҷ�
				else   	SPILCD_WriteRAM(RGB2COLOR(CCD_Image[i][j],CCD_Image[i][j],CCD_Image[i][j]));
			}
		}
}
 void I2C_ConfigOV7620(void)
{
	//���͵�ַ
	I2C_GenerateSTART(I2C1);
	I2C_Send7bitAddress(I2C1,0x42,I2C_MASTER_WRITE);
	I2C_WaitAck(I2C1);
	//������Ҫд��ļĴ�����ַ
	I2C_SendData(I2C1,0x11);
	I2C_WaitAck(I2C1);
	//��������
	I2C_SendData(I2C1,0x03);
	I2C_WaitAck(I2C1);
	//����
	I2C_GenerateSTOP(I2C1);
	while(I2C_IsLineBusy(I2C1) == TRUE);
	I2C_GenerateSTART(I2C1);
	I2C_Send7bitAddress(I2C1,0x42,I2C_MASTER_WRITE);
	I2C_WaitAck(I2C1);
	//������Ҫд��ļĴ�����ַ
	I2C_SendData(I2C1,0x14);
	I2C_WaitAck(I2C1);
	//��������
	I2C_SendData(I2C1,0x24);
	I2C_WaitAck(I2C1);
	//����
	I2C_GenerateSTOP(I2C1);
	while(I2C_IsLineBusy(I2C1) == TRUE);
	I2C_GenerateSTART(I2C1);
	I2C_Send7bitAddress(I2C1,0x42,I2C_MASTER_WRITE);
	I2C_WaitAck(I2C1);
	//������Ҫд��ļĴ�����ַ
	I2C_SendData(I2C1,0x28);
	I2C_WaitAck(I2C1);
	//��������
	I2C_SendData(I2C1,0x20);
	I2C_WaitAck(I2C1);
	//����
	I2C_GenerateSTOP(I2C1);
	while(I2C_IsLineBusy(I2C1) == TRUE);
}

