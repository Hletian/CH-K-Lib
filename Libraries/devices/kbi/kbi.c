/**
  ******************************************************************************
  * @file    key.c
  * @author  YANDLD
  * @version V2.4
  * @date    2013.5.23
  * @brief   ����ԭ�Ӻ˺��İ� BSP���� KEY����
  ******************************************************************************
  */

#include "kbi.h"

#define KBI_STATE_MASK      0xC000u
#define KBI_STATE_SHIFT     14
#define KBI_STATE(x)        (((uint32_t)(((uint32_t)(x))<<KBI_STATE_SHIFT))&KBI_STATE_MASK)

#define KBI_RETSTATE_MASK   0x3000u
#define KBI_RETSTATE_SHIFT  12
#define KBI_RETSTATE(x)     (((uint32_t)(((uint32_t)(x))<<KBI_RETSTATE_SHIFT))&KBI_RETSTATE_MASK)

#define KBI_TIMEOUT_MASK    0x0FFFu
#define KBI_TIMEOUT_SHIFT   0
#define KBI_TIMEOUT(x)      (((uint32_t)(((uint32_t)(x))<<KBI_TIMEOUT_SHIFT))&KBI_TIMEOUT_MASK)

//! @brief Key process state machine 
typedef enum
{
    kKBI_StateInit,
    kKBI_State1,
    kKBI_State2,
    kKBI_State3,
}KBI_State_TypeDef;

static KBI_PinLookup_TypeDef* gKBI_PinLookupStruct = NULL;
static uint32_t gNumOfKEYs = 0;
static uint16_t gKBI_KeyStates[KBI_MAX_KEY_CNT] = {0};
  /**
  * @brief  init key componment 
  * @param  KBI_PinLookupStruct, pointer of KBI install struct
  * @param  NumOfKEYs: num of struct in KBI install struct
  * @retval None
  */
void KBI_Init(const KBI_PinLookup_TypeDef* KBI_PinLookupStruct, uint32_t NumOfKEYs)
{
    uint8_t i;
    GPIO_InitTypeDef GPIO_InitStruct1;
    uint32_t temp = (uint32_t)KBI_PinLookupStruct;
    if((KBI_PinLookupStruct != NULL) && (NumOfKEYs != 0))
		{
        gKBI_PinLookupStruct = (KBI_PinLookup_TypeDef*)temp;
        gNumOfKEYs = NumOfKEYs;
        for(i = 0; i < NumOfKEYs; i++)
        {
            GPIO_InitStruct1.GPIOx = (GPIO_Type*)KBI_PinLookupStruct[i].KBI_PortMoudle;
					  (Bit_SET == KBI_STATE_UP)?(GPIO_InitStruct1.GPIO_Mode = GPIO_Mode_IPU):(GPIO_InitStruct1.GPIO_Mode = GPIO_Mode_IPD);
            GPIO_InitStruct1.GPIO_IRQMode = GPIO_IT_DISABLE;
            GPIO_InitStruct1.GPIO_Pin = KBI_PinLookupStruct[i].KBI_PinIndex;
            GPIO_Init(&GPIO_InitStruct1);
        }
		}
}

uint8_t KBI_GetKeyValue(uint32_t KeyIndex)
{
    if(KeyIndex < gNumOfKEYs)
		{
            return GPIO_ReadInputDataBit((GPIO_Type*)gKBI_PinLookupStruct[KeyIndex].KBI_PortMoudle,
	                               gKBI_PinLookupStruct[KeyIndex].KBI_PinIndex);	
		}
		return 0xFF;
}

uint32_t KBI_GetNumOfKEY(void)
{
    return gNumOfKEYs;
}

uint32_t KBI_GetScanPeriodInUs(void)
{
    return KBI_SCAN_PERIOD_IN_US;
}

static void SetKeyState(uint32_t KeyIndex, KBI_State_TypeDef State)
{
    gKBI_KeyStates[KeyIndex] &= ~KBI_STATE_MASK;
    gKBI_KeyStates[KeyIndex] |= (State<<KBI_STATE_SHIFT);
}

static KBI_State_TypeDef GetKeyState(uint32_t KeyIndex)
{
    return (KBI_State_TypeDef)((gKBI_KeyStates[KeyIndex] & KBI_STATE_MASK) >> KBI_STATE_SHIFT);
}

static void SetKeyTime(uint32_t KeyIndex, uint16_t TimeInPeriod)
{
    gKBI_KeyStates[KeyIndex] &= ~KBI_TIMEOUT_MASK;
	  gKBI_KeyStates[KeyIndex] |= KBI_TIMEOUT(TimeInPeriod);
}

static uint16_t GetKeyTime(uint32_t KeyIndex)
{
    return ((gKBI_KeyStates[KeyIndex] & KBI_TIMEOUT_MASK) >> KBI_TIMEOUT_SHIFT);
}

static void SetRetState(uint32_t KeyIndex, KBI_KeyState_TypeDef RetValue)
{
    gKBI_KeyStates[KeyIndex] &= ~KBI_RETSTATE_MASK;
    gKBI_KeyStates[KeyIndex] |= (RetValue<<KBI_RETSTATE_SHIFT);
}

KBI_KeyState_TypeDef KBI_GetKeyState(uint32_t KeyIndex)
{
    return (KBI_KeyState_TypeDef)((gKBI_KeyStates[KeyIndex] & KBI_RETSTATE_MASK) >> KBI_RETSTATE_SHIFT);
}

void KBI_Scan(void)
{
    uint8_t i;

	  uint8_t keyPress;
    for(i = 0; i < gNumOfKEYs; i++)
    {
        SetRetState(i, kKBI_NO_KEY);
        keyPress = KBI_GetKeyValue(i);
        switch(GetKeyState(i))
        {
            case kKBI_StateInit: //������ʼ״̬
                if(keyPress == KBI_STATE_DOWN)
								{
                    SetKeyState(i, kKBI_State1);
								}
                break;
            case kKBI_State1:   //������ȷ��̬
                if(keyPress == KBI_STATE_DOWN)
                {
									  SetKeyTime(i, 0);
                    SetKeyState(i, kKBI_State2);
                }
                else
                {
                    SetKeyState(i, kKBI_StateInit);
                }
                break; 
            case kKBI_State2:  // ��ʱ�����ͷţ�˵���ǲ���һ�ζ̲���������S_key 
                if(keyPress == KBI_STATE_UP)  
                {
                    SetRetState(i, kKBI_SINGLE);
                    SetKeyState(i, kKBI_StateInit);
                }
								else if(GetKeyTime(i) > ((500*1000) / KBI_SCAN_PERIOD_IN_US))
								{  
									  SetRetState(i, kKBI_LONG);
                    SetKeyState(i, kKBI_State3);     
								}
                else  //�������£���ʱ��10ms��10msΪ������ѭ��ִ�м���� 
                { 
                    SetKeyTime(i, GetKeyTime(i)+1);
                }
                break;
            case kKBI_State3:
                if(keyPress == KBI_STATE_UP)
								{
                    SetKeyState(i, kKBI_StateInit);
								}
                break;
				}
    }
}
