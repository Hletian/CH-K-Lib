
#ifndef __OV7620_H__
#define __OV7620_H__

#include "gpio.h"
#include "dma.h"

#define OV7620_STATE_IDLE          (0)
#define OV7620_STATE_START         (1)
#define OV7620_STATE_COMPLETE      (2)

//����ͷ����ֱ���
#define OV7620_W  320
#define OV7620_H  240


// �����ͼ������ڴ��еĴ�С  //���ֵ��ñ�
#define CCD_IMAGE_W  111	
#define CCD_IMAGE_H  120      


//���Ŷ��� ���� PD0-7
#define OV7620_DATAPORT  PTD

//���ж�
#define OV7620_HREF_PORT  PTB
#define OV7620_HREF_PIN   0
//���ж�
#define OV7620_VSYNC_PORT PTB
#define OV7620_VSYNC_PIN  1
//�����ж�
#define OV7620_PCLK_PORT  PTC
#define OV7620_PCLK_PIN  17


typedef struct
{
	uint32_t FiledCnt;         //������
	uint8_t TransferCompleteFlag; //������ɱ�־ ��Ҫ�û���0
	uint8_t State;
	uint16_t HREFITCnt;           //���жϼ���
	uint16_t HREFCnt;             //��Ч�����жϼ���
}OV7620Dev_TypeDef;

extern OV7620Dev_TypeDef OV7620Dev;
extern uint8_t CCD_Image[CCD_IMAGE_H][CCD_IMAGE_W];   //ͼ������

//�ӿں���
void OV7620_Init(void);
void OV7620_StartTransfer(void);
#endif
