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


//超核 飞思卡尔 Kinetis固件库例程
//固件库版本 V2.4
//startup:        启动文件
//devices:        硬件板子资源支持库(部分实验用到)
//utilities:      软件模块支持库(部分实验用到)
//drivers:        固件库源码
//use:            用户代码 包含mian函数和 中断服务程序 

/*
* 入门知识:
QD: 正交编码 需要2根线 A线 B线 可以测量 转速和 旋转方向
*/


int main(void)
{
		uint32_t value;
		uint8_t dir;
    //初始化系统时钟 使用外部50M晶振 PLL倍频到100M
    SystemClockSetup(ClockSource_EX50M,CoreClock_100M);
    DelayInit();
    LED_Init(LED_PinLookup_CHKATOM, kNumOfLED);
    UART_DebugPortInit(UART4_RX_C14_TX_C15, 115200);
    UART_printf("FTM QuadratureDecoder Test\r\n");
	
    //QD Init
    FTM_QDInit(FTM2_QD_B18_PHA_B19_PHB);
		
    while(1)
    {
			  //Get Data for QD module
			  FTM_QDGetData(FTM2, &value, &dir);
        UART_printf("Value:%d  Direction%d\r\n", value, dir);
				DelayMs(50);
				LED_Toggle(kLED1);
    }
 }

