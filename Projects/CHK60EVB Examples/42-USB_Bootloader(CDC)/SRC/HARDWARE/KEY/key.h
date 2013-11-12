#ifndef __KEY_H
#define __KEY_H	 
#include "sys.h"
#include "gpio.h"

//LEDͷ�ļ�
//IO ��
#define KEY1_PORT  PTC
#define KEY2_PORT  PTC
//����
#define KEY1_PIN  (17)
#define KEY2_PIN  (18)
//λ��
#define KEY1      PCin(KEY1_PIN)
#define KEY2      PCin(KEY2_PIN)
//������ʵ�ֵĽӿں���
void KEY_Init(void);

//��������ģʽ
typedef enum KEY_MODE  //����ɨ��ģʽ
{
	KEY_MODE_ONCES,
  KEY_MODE_CONTINUE,
}KEY_MODE;
//�������ؽ��ö��
typedef enum KRY_RES
{
	KEY_RES_KEY1 = 1,
	KEY_RES_KEY2 = 2,
	KEY_RES_NO_KEY = 0,
}KEY_RES;
//������ʵ�ֵĽӿں���
KEY_RES KEY_Scan(KEY_MODE mode);
void KEY_Init(void);

#endif

