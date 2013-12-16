/**
  ******************************************************************************
  * @file    spilcd.c
  * @author  YANDLD
  * @version V2.4
  * @date    2013.5.23
  * @brief   ����ԭ�Ӻ˺��İ� BSP���� SPILCD����
  ******************************************************************************
  */
#include "spilcd.h"
#include "spi.h"


//��������SPILCD_WR_REG
//��  ������
//        ʵ��д��һ�ֽ�����
void SPILCD_WR_REG(uint8_t com)
{
	SPILCD_RS=0;
	SPI_ReadWriteByte(SPILCD_PORT_CS,com,SPI_PCS_Inactive);
}
//��������LCD_WR_REG
//��  ������
//        ʵ��д��һ�ֽ�����
void SPILCD_WR_DATA(uint8_t dat)
{
	SPILCD_RS=1;
	SPI_ReadWriteByte(SPILCD_PORT_CS,dat,SPI_PCS_Inactive);
}
//��������LCD_WR_REG
//��  ������
//        ׼����ʼд��GRAM
 void SPILCD_WriteRAM_Prepare(void)
{
	SPILCD_WR_REG(0x2c);   //дRAM
}	 

//��������LCD_WR_REG
//��  ������
//        дGRAM����
void SPILCD_WriteRAM(uint16_t RGB_Code)
{							    
    SPILCD_WR_DATA(RGB_Code>>8);
    SPILCD_WR_DATA(RGB_Code); 
}
//��������LCD_DisplayOn
//��  ������
//        ������ʾ
void LCD_DisplayOn(void)
{					   
    SPILCD_WR_REG(0x29);   //
}	 
//��������LCD_DisplayOff
//��  ������
//        �ر���ʾ
void LCD_DisplayOff(void)
{	   
    SPILCD_WR_REG(0x28);   //
}   
//��������LCD_SetWindow
//��  ����Xpos:������
//				Ypos:������
 void SPILCD_SetWindow(uint16_t xstat,uint16_t xend,uint16_t ystat,uint16_t yend)
{
	 SPILCD_WR_REG(0x2A);
   SPILCD_WR_DATA(xstat>>8);
   SPILCD_WR_DATA(xstat);
   SPILCD_WR_DATA(xend>>8);
   SPILCD_WR_DATA(xend);
  
   SPILCD_WR_REG(0x2B);
   SPILCD_WR_DATA(ystat>>8);
   SPILCD_WR_DATA(ystat);
   SPILCD_WR_DATA(yend>>8);
	SPILCD_WR_DATA(yend);
}
//������ɫ��ʾ
void LCD_InvDisplayOn()
{
  SPILCD_WR_REG(0x21);  
}
//�رշ�ɫ��ʾ
void LCD_InvDisplayOff()
{
  SPILCD_WR_REG(0x20); 
}   

//��������SPILCD_Init
//��  ������
//        LCD Ӳ����ʼ��
void SPILCD_Init()
{
	uint32_t delay_cnt = 0;
	SPI_InitTypeDef SPI_InitStruct1;
	GPIO_InitTypeDef GPIO_InitStruct1;
  //��ʼ��RST����
	GPIO_InitStruct1.GPIO_Pin = SPILCD_RST_PIN;
	GPIO_InitStruct1.GPIO_InitState = Bit_RESET;
	GPIO_InitStruct1.GPIO_IRQMode = GPIO_IT_DISABLE;
	GPIO_InitStruct1.GPIO_Mode = GPIO_Mode_OPP;
	GPIO_InitStruct1.GPIOx = SPILCD_RST_PORT;
	GPIO_Init(&GPIO_InitStruct1);
	//��ʼ��RS����
	GPIO_InitStruct1.GPIO_Pin = SPILCD_RS_PIN;
	GPIO_InitStruct1.GPIO_InitState = Bit_RESET;
	GPIO_InitStruct1.GPIO_IRQMode = GPIO_IT_DISABLE;
	GPIO_InitStruct1.GPIO_Mode = GPIO_Mode_OPP;
	GPIO_InitStruct1.GPIOx = SPILCD_RS_PORT;
	GPIO_Init(&GPIO_InitStruct1);
	//��ʼ��SPI���нӿ�����
	SPI_InitStruct1.SPIxDataMap = SPILCD_PORT_DATA;
	SPI_InitStruct1.SPIxPCSMap = SPILCD_PORT_CS;
	SPI_InitStruct1.SPI_DataSize = 8;
	SPI_InitStruct1.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;
	SPI_InitStruct1.SPI_Mode = SPI_Mode_Master;
	SPI_InitStruct1.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStruct1.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStruct1.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_Init(&SPI_InitStruct1);
	
	SPILCD_RST=0;    
	for(delay_cnt=0;delay_cnt<60000;delay_cnt++);
	SPILCD_RST=1;    
	for(delay_cnt=0;delay_cnt<60000;delay_cnt++);	
	SPILCD_WR_REG(0x11);       		  	//�ر�˯�ߣ���������
	for(delay_cnt=0;delay_cnt<60000;delay_cnt++);	
	
	SPILCD_WR_REG(0x3a);       		  	//ÿ�δ���16λ����(VIPF3-0=0101)��ÿ������16λ(IFPF2-0=101)
	SPILCD_WR_DATA(0x55);						
 
	SPILCD_WR_REG(0x26);       		  	
	SPILCD_WR_DATA(0x04);

	SPILCD_WR_REG(0xf2);              		//Driver Output Control(1)
	SPILCD_WR_DATA(0x01);
	
	SPILCD_WR_REG(0xe0);              		//Driver Output Control(1)
	SPILCD_WR_DATA(0x3f);
	SPILCD_WR_DATA(0x25);
	SPILCD_WR_DATA(0x1c);
	SPILCD_WR_DATA(0x1e);
	SPILCD_WR_DATA(0x20);
	SPILCD_WR_DATA(0x12);
	SPILCD_WR_DATA(0x2a);
	SPILCD_WR_DATA(0x90);
	SPILCD_WR_DATA(0x24);
	SPILCD_WR_DATA(0x11);
	SPILCD_WR_DATA(0x00);
	SPILCD_WR_DATA(0x00);
	SPILCD_WR_DATA(0x00);
	SPILCD_WR_DATA(0x00);
	SPILCD_WR_DATA(0x00);
	 
	SPILCD_WR_REG(0xe1);              //Driver Output Control(1)
	SPILCD_WR_DATA(0x20);
	SPILCD_WR_DATA(0x20);
	SPILCD_WR_DATA(0x20);
	SPILCD_WR_DATA(0x20);
	SPILCD_WR_DATA(0x05);
	SPILCD_WR_DATA(0x00);
	SPILCD_WR_DATA(0x15);
	SPILCD_WR_DATA(0xa7);
	SPILCD_WR_DATA(0x3d);
	SPILCD_WR_DATA(0x18);
	SPILCD_WR_DATA(0x25);
	SPILCD_WR_DATA(0x2a);
	SPILCD_WR_DATA(0x2b);
	SPILCD_WR_DATA(0x2b);  
	SPILCD_WR_DATA(0x3a);  
	
	SPILCD_WR_REG(0xb1);              	//������Ļˢ��Ƶ��
	SPILCD_WR_DATA(0x08);				   	//DIVA=8
	SPILCD_WR_DATA(0x08);				   	//VPA =8��Լ90Hz
						 
	SPILCD_WR_REG(0xb4);              	//LCD Driveing control
	SPILCD_WR_DATA(0x07);				  	//NLA=1,NLB=1,NLC=1
 
 
	SPILCD_WR_REG(0xc0);              //LCD Driveing control
	SPILCD_WR_DATA(0x0a);
	SPILCD_WR_DATA(0x02);
		
	SPILCD_WR_REG(0xc1);              //LCD Driveing control
	SPILCD_WR_DATA(0x02);

	SPILCD_WR_REG(0xc5);              //LCD Driveing control
	SPILCD_WR_DATA(0x4f);
	SPILCD_WR_DATA(0x5a);

	SPILCD_WR_REG(0xc7);              //LCD Driveing control
	SPILCD_WR_DATA(0x40);
	
	SPILCD_WR_REG(0x2a);              	//����MCU�ɲ�����LCD�ڲ�RAM��������ʼ����������
	SPILCD_WR_DATA(0x00);				   	//��������ʼ��ַ0x0000
	SPILCD_WR_DATA(0x00);					
	SPILCD_WR_DATA(0x00);				   	//�����������ַ0x007f(127)
	SPILCD_WR_DATA(0x7f);
 
	SPILCD_WR_REG(0x2b);              	//����MCU�ɲ�����LCD�ڲ�RAM��������ʼ��������
	SPILCD_WR_DATA(0x00);				   	//��������ʼ��ַ0x0000
	SPILCD_WR_DATA(0x00);
	SPILCD_WR_DATA(0x00);				  	//�����������ַ0x009f(159)
	SPILCD_WR_DATA(0x9f);

	SPILCD_WR_REG(0x36);              	//����MPU��DDRAM��Ӧ��ϵ
	SPILCD_WR_DATA(0xc0);					//MX=1,MY=1

	SPILCD_WR_REG(0xb7);              	//LCD Driveing control
	SPILCD_WR_DATA(0x00);				   	//CRL=0
	 
	SPILCD_WR_REG(0x29);   		  	//������Ļ��ʾ
	SPILCD_WR_REG(0x2c);   			//����ΪLCD��������/����ģʽ
	SPILCD_Clear(0x0000);
}


//��������SPILCD_Init
//��  ����X Y ���� 
//        ��X Y�ϴ��
void SPILCD_DrawPoint(uint16_t x, uint16_t y, uint16_t color)
{
  SPILCD_SetWindow(x,x+1,y,y+1);//���ù��λ�� 
	SPILCD_WriteRAM_Prepare();     //��ʼд��GRAM	 
	SPILCD_WriteRAM(color);
} 	 

//��������SPILCD_Clear
//��  ����Color ��ɫ      
void SPILCD_Clear(uint16_t Color)
{
	uint32_t index=0;      
	SPILCD_SetWindow(0,SPILCD_W-1,0,SPILCD_H-1);	 
	SPILCD_WriteRAM_Prepare();     //��ʼд��GRAM	 	  
	for(index=0;index<SPILCD_W*SPILCD_H;index++)
	{
		SPILCD_WriteRAM(Color);//��ʾ��������ɫ. 
	}
}  
//��������SPILCD_Fill
//��  ������ʼ�յ�����
void SPILCD_Fill(uint16_t xsta,uint16_t ysta,uint16_t xend,uint16_t yend,uint16_t color)
{                    
	uint32_t n;
	//���ô���										
  SPILCD_SetWindow(xsta,xend,ysta,yend);
	SPILCD_WriteRAM_Prepare();  //��ʼд��GRAM	 	   	   
	n=(uint32_t)(yend-ysta+1)*(xend-xsta+1);    
	while(n--){SPILCD_WriteRAM(color);}//��ʾ��������ɫ. 
	//�ָ�����
	SPILCD_SetWindow(0,SPILCD_W-1,0,SPILCD_H-1);	    
}



