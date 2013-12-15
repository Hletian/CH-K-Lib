
#include "enet2.h"
#include "minishell.h"
#include <stdint.h>

#define ENET_TYPE_ARP   {0x08, 0x06}
// IP  ���ݱ�
#define ENET_TYPE_IP    {0x08, 0x00}
//������ַ
extern uint8_t  gCfgLoca_MAC[];
//Ŀ��MAC�����ַ���壬�㲥��ַ
uint8_t  gCfgDest_MAC[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
//��̫֡���ͣ�ARP����
uint8_t  gCfgEnet_Type[] = ENET_TYPE_ARP;
//��̫֡���ͻ�����
uint8_t  gTxBuffer[1520]; 

OS_FRAME enet_frame;
//�û����� LED���� ��ֻ��һ������
int DoEnet(int argc, char *argv[])
{
    uint16_t i;
		//������̫֡0~5�ֽ�Ϊ��Ŀ�ĵ�ַ
		gTxBuffer[0]  = gCfgDest_MAC[0];
		gTxBuffer[1]  = gCfgDest_MAC[1];
		gTxBuffer[2]  = gCfgDest_MAC[2];
		gTxBuffer[3]  = gCfgDest_MAC[3];
		gTxBuffer[4]  = gCfgDest_MAC[4];
		gTxBuffer[5]  = gCfgDest_MAC[5];
		//������̫֡1~11�ֽ�Ϊ��������ַ
		gTxBuffer[6]  = gCfgLoca_MAC[0];
		gTxBuffer[7]  = gCfgLoca_MAC[1];
		gTxBuffer[8]  = gCfgLoca_MAC[2];
		gTxBuffer[9]  = gCfgLoca_MAC[3];
		gTxBuffer[10] = gCfgLoca_MAC[4];
		gTxBuffer[11] = gCfgLoca_MAC[5];
		//������̫֡12~13�ֽ�Ϊ����������
		gTxBuffer[12] = gCfgEnet_Type[0];
		gTxBuffer[13] = gCfgEnet_Type[1];
		//������̫֡14�ֽ��Ժ������
		for(i=14; i<1520; i++)
		{
			gTxBuffer[i] = i;
		}
		//  sp = (U32 *)&frame->data[0];
		memcpy(enet_frame.data, gTxBuffer,100);
		enet_frame.index = 0;
		enet_frame.length = 100;
	 send_frame (&enet_frame);
	 int_enable_eth();
  //  ENET_MacSendData(gTxBuffer,100);	
		return 0;
}

//�û�����ע��ṹ
MINISHELL_CommandTableTypeDef Command_FunEnet =
{
    .name = "1",            //��������
    .maxargs = 3,             //������������
    .cmd = DoEnet,       //ʵ�ֺ����ӿ�
    .usage = "LED ON(OFF)",   //��;˵��
};


