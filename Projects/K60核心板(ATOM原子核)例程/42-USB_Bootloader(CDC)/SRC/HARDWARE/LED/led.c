/***********************************************************************************************
 CHKD内部测试版本V0.1
 2012.12.3
************************************************************************************************/
#include "led.h"

//LED初始化
void LED_Init(void)
{	
	GPIO_Init(LED0_PORT,LED0_PIN,1,0);
//	GPIO_Init(LED1_PORT,LED1_PIN,1,0);
//	GPIO_Init(LED2_PORT,LED2_PIN,1,0);
//	GPIO_Init(LED3_PORT,LED3_PIN,1,0);
}
