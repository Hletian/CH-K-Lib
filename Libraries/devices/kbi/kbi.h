/**
  ******************************************************************************
  * @file    kbi.h
  * @author  YANDLD
  * @version V2.4
  * @date    2013.5.23
  * @brief   ����ԭ�Ӻ˺��İ� BSP���� KEY����
  ******************************************************************************
  */
#ifndef _KBI_H_
#define _KBI_H_

#include "gpio.h"

#ifdef __cplusplus
 extern "C" {
#endif

//!< KEY������ʱ��ƽ״̬
#define	KBI_STATE_DOWN                Bit_SET


#define KBI_STATE_UP                  !KBI_STATE_DOWN
//!< KEY_SCAN����ɨ������
#define KBI_SCAN_PERIOD_IN_US         (1000*10)
//!< ���֧�ֵİ�����
#define KBI_MAX_KEY_CNT               (8)
//!< �ж�Ϊ���� ���趨ʱ�� 
#define KBI_LONG_KEY_TIMEIN_US        (1000*500)
	 
//!< KBI ������װ�ṹ
typedef struct
{
    uint32_t KBI_PortMoudle;
    uint32_t KBI_PinIndex;
}KBI_PinLookup_TypeDef;
	 
//!< KBI���� ����ֵ����
typedef enum
{
    kKBI_NO_KEY,
    kKBI_SINGLE,
    kKBI_LONG,
}KBI_KeyState_TypeDef;

//!< API functions 
void KBI_Init(const KBI_PinLookup_TypeDef* KBI_PinLookupStruct, uint32_t NumOfKEYs);
KBI_KeyState_TypeDef KBI_GetKeyState(uint32_t KeyIndex);
uint8_t KBI_GetKeyValue(uint32_t KeyIndex);
uint32_t KBI_GetNumOfKEY(void);
uint32_t KBI_GetScanPeriodInUs(void);
void  KBI_Scan(void);


#ifdef __cplusplus
}
#endif

#endif
