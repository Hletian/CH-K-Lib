#include "ov7620.h"

DMA_InitTypeDef DMA_InitStruct1;
uint8_t DMABuffer[400]; 
OV7620Dev_TypeDef OV7620Dev;
uint8_t CCD_Image[CCD_IMAGE_H][CCD_IMAGE_W];   //ͼ������ 
//��Ч����  ֻ�ɼ� 68 71 74 77�� ��
uint8_t  CCD_Valid_Line[CCD_IMAGE_H];

//���жϺ���
void OV7620_StartTransfer()
{
	GPIO_InitTypeDef GPIO_InitStruct1;
	//��״̬����Ϊ��ʼ����
	if(OV7620Dev.State == OV7620_STATE_IDLE)
	{
		OV7620Dev.State = OV7620_STATE_START;
	}
	//�����ж�
	GPIO_InitStruct1.GPIO_Pin = OV7620_VSYNC_PIN;
	GPIO_InitStruct1.GPIO_InitState = Bit_SET;
	GPIO_InitStruct1.GPIO_IRQMode = GPIO_IT_RISING;
	GPIO_InitStruct1.GPIO_Mode = GPIO_Mode_IPD;
	GPIO_InitStruct1.GPIOx = OV7620_VSYNC_PORT;
	GPIO_Init(&GPIO_InitStruct1);
	//GPIO_SetInterrupt(OV7620_VSYNC_PORT,OV7620_VSYNC_PIN,GPIO_IRQC_RISING);
}

//��ʼ��OV7620ģ��
void OV7620_Init()
{
	GPIO_InitTypeDef GPIO_InitStruct1;
	uint32_t i = 0;
	//��Ч���� 
	for(i=0;i<sizeof(CCD_Valid_Line);i++)
	{
		CCD_Valid_Line[i] = i*2; //*2�����������Ҫ������ͼ��߶ȷֱ� 
	}
	//����״̬
	OV7620Dev.State = OV7620_STATE_IDLE;
	//����ͷ���� D0-D7
	for(i=0;i<8;i++)
	{
		GPIO_InitStruct1.GPIO_Pin = i;
		GPIO_InitStruct1.GPIO_InitState = Bit_SET;
		GPIO_InitStruct1.GPIO_IRQMode = GPIO_IT_DISABLE;
		GPIO_InitStruct1.GPIO_Mode = GPIO_Mode_IN_FLOATING;
		GPIO_InitStruct1.GPIOx = OV7620_DATAPORT;
		GPIO_Init(&GPIO_InitStruct1);
	}
	
	//�����ж� PCLK
	GPIO_InitStruct1.GPIO_Pin = OV7620_PCLK_PIN;
	GPIO_InitStruct1.GPIO_InitState = Bit_SET;
	GPIO_InitStruct1.GPIO_IRQMode = GPIO_IT_DMA_RISING;
	GPIO_InitStruct1.GPIO_Mode = GPIO_Mode_IPD;
	GPIO_InitStruct1.GPIOx = OV7620_PCLK_PORT;
	GPIO_Init(&GPIO_InitStruct1);
	//���ж� HREF
	GPIO_InitStruct1.GPIO_Pin = OV7620_HREF_PIN;
	GPIO_InitStruct1.GPIO_InitState = Bit_SET;
	GPIO_InitStruct1.GPIO_IRQMode = GPIO_IT_RISING;
	GPIO_InitStruct1.GPIO_Mode = GPIO_Mode_IPD;
	GPIO_InitStruct1.GPIOx = OV7620_HREF_PORT;
	GPIO_Init(&GPIO_InitStruct1);
	// ���ж� VSYNC
	GPIO_InitStruct1.GPIO_Pin = OV7620_VSYNC_PIN;
	GPIO_InitStruct1.GPIO_InitState = Bit_SET;
	GPIO_InitStruct1.GPIO_IRQMode = GPIO_IT_RISING;
	GPIO_InitStruct1.GPIO_Mode = GPIO_Mode_IPD;
	GPIO_InitStruct1.GPIOx = OV7620_VSYNC_PORT;
	GPIO_Init(&GPIO_InitStruct1);
	
	NVIC_EnableIRQ(PORTB_IRQn);
  //����DMA
  DMA_InitStruct1.Channelx          = DMA_CH1;         //DMA 1ͨ��
  DMA_InitStruct1.PeripheralDMAReq   =PORTC_DMAREQ;    //C�˿�(PCLK) ����ѽ����    
  DMA_InitStruct1.MinorLoopLength = 170;               //������� ��������ͷÿ������������
	DMA_InitStruct1.TransferBytes = 2;                   //ÿ�δ���3���ֽ�       
	DMA_InitStruct1.DMAAutoClose  = ENABLE;              //�����ɼ�
	DMA_InitStruct1.EnableState = ENABLE;                //��ʼ���������ɼ�
	 
  DMA_InitStruct1.SourceBaseAddr =(uint32_t)&PTD->PDIR;//����ͷ�˿ڽ�D0-D7
	DMA_InitStruct1.SourceMajorInc = 0;                  //��ַ������
	DMA_InitStruct1.SourceDataSize = DMA_SRC_8BIT;       //8BIT����
	DMA_InitStruct1.SourceMinorInc = 0;

  DMA_InitStruct1.DestBaseAddr =(uint32_t)DMABuffer;  //DMA �ڴ�
	DMA_InitStruct1.DestMajorInc = 0;
	DMA_InitStruct1.DestDataSize = DMA_DST_8BIT;
	DMA_InitStruct1.DestMinorInc = 1;                   //ÿ�δ��� +1���ֽ�
}


void HIsr(void); 
void VIsr(void);
/***********************************************************************************************
 ���ܣ�PORTB�ⲿ�ж����
 �βΣ�0       
 ���أ�0
 ��⣺0
************************************************************************************************/
void PORTB_IRQHandler(void)
{
	uint8_t i = 31;
	for(i=31;i>0;i--)  //�ж��ĸ����ŵ��жϷ�����i��ʾ�ĸ����ŷ������ж�
	{
	  if((PORTB->ISFR>>i)==1)break;
	}
	switch(i)
	{ //�жϴ���
	  case OV7620_HREF_PIN: //���ж�
				HIsr();
			break;
	  case OV7620_VSYNC_PIN://���ж�
				VIsr();
			break;
    // ...		
	  default : break;
	}
    PORTB->ISFR|=PORT_ISFR_ISF_MASK;   //�����־λ 
}

//���ж�
void VIsr()
{
	GPIO_InitTypeDef GPIO_InitStruct1;
	switch(OV7620Dev.State)
	{
		case OV7620_STATE_IDLE:  //����
			break;
		case OV7620_STATE_START: //��ʼ���һ�����ж��źŵ���
			//�����ж�
			GPIO_InitStruct1.GPIO_Pin = OV7620_HREF_PIN;
			GPIO_InitStruct1.GPIO_InitState = Bit_SET;
			GPIO_InitStruct1.GPIO_IRQMode = GPIO_IT_RISING;
			GPIO_InitStruct1.GPIO_Mode = GPIO_Mode_IPD;
			GPIO_InitStruct1.GPIOx = OV7620_HREF_PORT;
			GPIO_Init(&GPIO_InitStruct1);
		  //������0
			OV7620Dev.HREFCnt = 0;
			OV7620Dev.HREFITCnt = 0;
			OV7620Dev.State = OV7620_STATE_COMPLETE; 
			break;
		case OV7620_STATE_COMPLETE: //��ʼ��ڶ������ж��źŵ������ɼ����
			OV7620Dev.TransferCompleteFlag = TRUE;
			//�ر����ж�
			GPIO_InitStruct1.GPIO_Pin = OV7620_HREF_PIN;
			GPIO_InitStruct1.GPIO_InitState = Bit_SET;
			GPIO_InitStruct1.GPIO_IRQMode = GPIO_IT_DISABLE;
			GPIO_InitStruct1.GPIO_Mode = GPIO_Mode_IPD;
			GPIO_InitStruct1.GPIOx = OV7620_HREF_PORT;
			GPIO_Init(&GPIO_InitStruct1);
      //�رճ��ж�
			GPIO_InitStruct1.GPIO_Pin = OV7620_VSYNC_PIN;
			GPIO_InitStruct1.GPIO_InitState = Bit_SET;
			GPIO_InitStruct1.GPIO_IRQMode = GPIO_IT_DISABLE;
			GPIO_InitStruct1.GPIO_Mode = GPIO_Mode_IPD;
			GPIO_InitStruct1.GPIOx = OV7620_VSYNC_PORT;
			GPIO_Init(&GPIO_InitStruct1);
		  //������ɣ��������̬
			OV7620Dev.State = OV7620_STATE_IDLE;
			OV7620Dev.FiledCnt ++; //������
			break;
		default:break;
	}
}

//��ͬ���ص��жϺ���
void HIsr() //�������жλص�����
{
	uint16_t i;
	if(OV7620Dev.HREFITCnt == (CCD_Valid_Line[OV7620Dev.HREFCnt]-1))
	{
		//����DMA����
		DMA_Init(&DMA_InitStruct1); //����һ������
	}
	if(OV7620Dev.HREFITCnt == CCD_Valid_Line[OV7620Dev.HREFCnt])
	{
		for(i=0;i<CCD_IMAGE_W;i++)
		{
			CCD_Image[OV7620Dev.HREFCnt][i] = DMABuffer[i*2]; //*3�����������Ҫ ����ͼ����
		}
		OV7620Dev.HREFCnt++;
	}
	//�м���
	OV7620Dev.HREFITCnt++;
}
