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

//���� ��˼���� Kinetis�̼�������
//�̼���汾 V2.4
//startup:        �����ļ�
//devices:        ����оƬ֧�ֿ�(����ʵ���õ�)
//utilities:      ���ģ��֧�ֿ�(����ʵ���õ�)
//drivers:        �̼���Դ��



#define I2C_HUNTER_MAP    I2C1_SCL_PC10_SDA_PC11
#define I2C_HUNTER_PORT   I2C1

//ʵ��MINISHELL����Ҫ��PutChar����
static void Putc(uint8_t data)
{
	UART_SendData(UART4, data);
}
//ʵ��MINISHELL����Ҫ��GetChar����
static uint8_t Getc(void)
{
	uint8_t ch;
  while(UART_ReceiveData(UART4, &ch) == FALSE);
	return ch;
}

//�û����� LED���� ��ֻ��һ������
int CommandFun_I2C(int argc, char *argv[])
{
    uint8_t i;
	  uint8_t address_found = 0;
    //��������ֻ��2��(�ո�Ϊ�ָ���)   
    if(argc == 2)
		{
        if(!strcmp("scan", argv[1]))
        {
            MINISHELL_printf("Begin scanning I2C bus ...\r\n");
            for(i = 0; i < 255; i++)
            {
                
                I2C_GenerateSTART(I2C_HUNTER_PORT);
                I2C_Send7bitAddress(I2C_HUNTER_PORT,i,I2C_MASTER_WRITE);
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

//ע�� MiniShell������װ�ṹ
MINISHELL_InstallTypeDef MiniShell_InstallStruct1 = 
{
    .ctrl_putchar = Putc,
    .ctrl_getchar = Getc,
};

//�û�����ע��ṹ
MINISHELL_CommandTableTypeDef MyCommand1 =
{
    .name = "I2C",            //��������
    .maxargs = 2,             //������������
    .cmd = CommandFun_I2C,       //ʵ�ֺ����ӿ�
    .usage = "I2C <scan>",   //��;˵��
};


int main(void)
{
    SystemClockSetup(ClockSource_EX50M,CoreClock_100M);
    DelayInit();
    LED_Init(LED_PinLookup_CHK60EVB, kNumOfLED);
    UART_DebugPortInit(UART4_RX_C14_TX_C15, 115200);
	  DisplayCPUInfo();
    I2C_InitTypeDef I2C_InitStruct1;
    /* Initalize Kinetis I2C module  */
    I2C_InitStruct1.I2CxMAP = I2C_HUNTER_MAP;
    I2C_InitStruct1.I2C_ClockSpeed = I2C_CLOCK_SPEED_100KHZ;
    I2C_Init(&I2C_InitStruct1);
	
	  UART_printf("I2C Bus Hunter! Build in :%s.\r\n", __DATE__);
	  //��װ������
    MINISHELL_Install(&MiniShell_InstallStruct1);
    //ע��ӿں���
		MINISHELL_Register(&MyCommand1, 1);
	  //��ʼ��SHELL
	  MINISHELL_Init();
	  //����SHELL
	  MINISHELL_CmdHandleLoop("SHELL>>");
    while(1) {};

}

