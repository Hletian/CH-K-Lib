/***********************************************************************************************
//CH_Kinetis������  V2.3
//����    :YANDLD 
//E-MAIL  :yandld@126.com
//�޸�����:2013/2/14
//�汾��V2.3
//�Ա���http://upcmcu.taobao.com
//QQ    1453363089
//Copyright(C) YANDLD 2012-2022
//All rights reserved
************************************************************************************************/
#ifndef __WDOG_H__
#define __WDOG_H__
#include "sys.h"

//���Ź�����ͷ�ļ�
typedef unsigned short u16;
void WDOG_Init(u16 ms); //���Ź���ʼ�� �����������Ź�
void WDOG_Open(void);  //�������Ź�
void WDOG_Close(void);  //�رտ��Ź�
void WDOG_Feed(void);   //ι��,��ֹϵͳ��������
#endif
