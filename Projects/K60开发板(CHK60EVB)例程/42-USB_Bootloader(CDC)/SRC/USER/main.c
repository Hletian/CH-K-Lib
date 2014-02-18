#include "bootloader.h"
#include "bootloader_util.h"
#include "sys.h"
#include "uart.h"
#include "led.h"
#include "USB.h"
#include "USB_CDC.h" 
#include "def.h"
#include "key.h"

int main(void)
{   
	u32 i=0;



	KEY_Init();
	//UART_Init(UART4,115200);
	//������
	if((KEY1 == 0) || (KEY2 == 0))
	{	
		GetCPUInfo();
	  DelayInit();   //��ʼ����ʱ����
		LED_Init();
		USB_Init(); //һ����Ҫ96M�ڲ�FLL��Ƶ
		while(USB_IsDeviceEnumed() == 0);
		//����һ��LED��ʾ USBö�ٳɹ�
		LED0 = 1;
		BootloaderInit(UART3,115200,2000);
		while(1)
		{ 
			BootloaderProc(); //ִ��Boot������
		}
	}
	else
	{
		Fn_GoToUserApplication(APP_START_ADDR);
	}
}





