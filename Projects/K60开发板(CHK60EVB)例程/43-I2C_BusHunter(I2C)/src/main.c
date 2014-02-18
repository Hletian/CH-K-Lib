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

//MiniShell
#include "minishell.h"

//超核 飞思卡尔 Kinetis固件库例程
//固件库版本 V2.4
//startup:        启动文件
//devices:        外设芯片支持库(部分实验用到)
//utilities:      软件模块支持库(部分实验用到)
//drivers:        固件库源码


// I2C Hunter PORT define
#define I2C_HUNTER_MAP        I2C1_SCL_PC10_SDA_PC11
#define I2C_HUNTER_PORT       I2C1
#define I2C_HUNTER_BUS_SPEED  I2C_CLOCK_SPEED_300KHZ
const uint32_t I2C_HunterBusSpeedTable[]= {I2C_CLOCK_SPEED_50KHZ, I2C_CLOCK_SPEED_100KHZ, I2C_CLOCK_SPEED_150KHZ, I2C_CLOCK_SPEED_200KHZ, I2C_CLOCK_SPEED_250KHZ, I2C_CLOCK_SPEED_300KHZ};
//实现MINISHELL所需要的PutChar函数
static void Putc(uint8_t data)
{
	UART_SendData(UART4, data);
}
//实现MINISHELL所需要的GetChar函数
static uint8_t Getc(void)
{
	uint8_t ch;
  while(UART_ReceiveData(UART4, &ch) == FALSE);
	return ch;
}



//用户函数 LED控制 这只是一个例子
int CommandFun_I2C(int argc, char *argv[])
{
    uint8_t i,j;
	  uint8_t address_found = 0;
    //输入内容只有2段(空格为分隔符)   
    if(argc == 2)
		{
			  for(j=0; j< ARRAY_SIZE(I2C_HunterBusSpeedTable); j++)
				{
				    I2C_InitTypeDef I2C_InitStruct1;
						I2C_InitStruct1.I2CxMAP = I2C_HUNTER_MAP;
            I2C_InitStruct1.I2C_ClockSpeed = j;
						I2C_Init(&I2C_InitStruct1);
						address_found = 0;
            MINISHELL_printf("Begin scanning I2C bus at Speed:%d ...\r\n", j);
            for(i = 0; i < 255; i++)
            {
                
                I2C_GenerateSTART(I2C_HUNTER_PORT);
                I2C_Send7bitAddress(I2C_HUNTER_PORT, i, I2C_MASTER_WRITE);
							  //receive succ.
                if(!I2C_WaitAck(I2C_HUNTER_PORT))
								{
										I2C_GenerateSTOP(I2C_HUNTER_PORT);
										while((I2C_HUNTER_PORT->S & I2C_S_BUSY_MASK) == 1);
									  MINISHELL_printf("Address%d Resepned!\r\n", i);
									  address_found++;
								}
								else
								{
										I2C_GenerateSTOP(I2C_HUNTER_PORT);
										while((I2C_HUNTER_PORT->S & I2C_S_BUSY_MASK) == 1);
								}
						}
						MINISHELL_printf("scanning completed.%d address found!\r\n", address_found);
				}

		}
		return 0;
}

//注册 MiniShell函数安装结构
MINISHELL_InstallTypeDef MiniShell_InstallStruct1 = 
{
    .ctrl_putchar = Putc,
    .ctrl_getchar = Getc,
};

//用户函数注册结构
MINISHELL_CommandTableTypeDef MyCommand1 =
{
    .name = "I2C",            //命令名字
    .maxargs = 2,             //包含的最大参数
    .cmd = CommandFun_I2C,       //实现函数接口
    .usage = "I2C <scan>",   //用途说明
};


int main(void)
{
    SystemClockSetup(ClockSource_EX50M,CoreClock_100M);
    DelayInit();
    LED_Init(LED_PinLookup_CHK60EVB, kNumOfLED);
    UART_DebugPortInit(UART4_RX_C14_TX_C15, 115200);
	  DisplayCPUInfo();


	
	  UART_printf("I2C Bus Hunter! Build in :%s.\r\n", __DATE__);
	  //安装连接器
    MINISHELL_Install(&MiniShell_InstallStruct1);
    //注册接口函数
		MINISHELL_Register(&MyCommand1, 1);
	  //初始化SHELL
	  MINISHELL_Init();
	  //启动SHELL
	  MINISHELL_CmdHandleLoop("SHELL>>");
    while(1) {};

}

