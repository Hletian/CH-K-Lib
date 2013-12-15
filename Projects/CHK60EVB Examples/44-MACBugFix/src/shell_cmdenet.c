
#include "enet2.h"
#include "minishell.h"
#include <stdint.h>

#define ENET_TYPE_ARP   {0x08, 0x06}
// IP  数据报
#define ENET_TYPE_IP    {0x08, 0x00}
//本机地址
extern uint8_t  gCfgLoca_MAC[];
//目的MAC物理地址定义，广播地址
uint8_t  gCfgDest_MAC[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
//以太帧类型，ARP报文
uint8_t  gCfgEnet_Type[] = ENET_TYPE_ARP;
//以太帧发送缓冲区
uint8_t  gTxBuffer[1520]; 

OS_FRAME enet_frame;
//用户函数 LED控制 这只是一个例子
int DoEnet(int argc, char *argv[])
{
    uint16_t i;
		//设置以太帧0~5字节为：目的地址
		gTxBuffer[0]  = gCfgDest_MAC[0];
		gTxBuffer[1]  = gCfgDest_MAC[1];
		gTxBuffer[2]  = gCfgDest_MAC[2];
		gTxBuffer[3]  = gCfgDest_MAC[3];
		gTxBuffer[4]  = gCfgDest_MAC[4];
		gTxBuffer[5]  = gCfgDest_MAC[5];
		//设置以太帧1~11字节为：本机地址
		gTxBuffer[6]  = gCfgLoca_MAC[0];
		gTxBuffer[7]  = gCfgLoca_MAC[1];
		gTxBuffer[8]  = gCfgLoca_MAC[2];
		gTxBuffer[9]  = gCfgLoca_MAC[3];
		gTxBuffer[10] = gCfgLoca_MAC[4];
		gTxBuffer[11] = gCfgLoca_MAC[5];
		//设置以太帧12~13字节为：报文类型
		gTxBuffer[12] = gCfgEnet_Type[0];
		gTxBuffer[13] = gCfgEnet_Type[1];
		//设置以太帧14字节以后的数据
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

//用户函数注册结构
MINISHELL_CommandTableTypeDef Command_FunEnet =
{
    .name = "1",            //命令名字
    .maxargs = 3,             //包含的最大参数
    .cmd = DoEnet,       //实现函数接口
    .usage = "LED ON(OFF)",   //用途说明
};


