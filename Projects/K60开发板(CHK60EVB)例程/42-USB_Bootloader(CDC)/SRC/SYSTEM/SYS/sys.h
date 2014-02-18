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
#ifndef __SYS_H__
#define __SYS_H__
#include <MK60DZ10.h>
#include "wdog.h"
//Ƶ������
#define CLOCK_SETUP     (0)	 
/* Ԥ�����ʱ������
   0 ... �ڲ�����
	       Core/System Clock: 96M 
				 Bus         Clock: 48M
				 FlexBus     Clock: 48M
				 Flash       Clock: 24M
   1 ... �ⲿ8M���� 
	       Core/System Clock: 96M 
				 Bus         Clock: 48M
				 FlexBus     Clock: 48M
				 Flash       Clock: 24M
   2 ... �ⲿ8M���� ��Ƶ
	       Core/System Clock: 200M 
				 Bus         Clock: 100M
				 FlexBus     Clock: 100M
				 Flash       Clock: 25M
   3 ... �ⲿ50M��Դ����
	       Core/System Clock: 96M 
				 Bus         Clock: 48M
				 FlexBus     Clock: 48M
				 Flash       Clock: 24M
   4 ... �ⲿ50M��Դ���� ��Ƶ
	       Core/System Clock: 200M 
				 Bus         Clock: 100M
				 FlexBus     Clock: 100M
				 Flash       Clock: 25M			 
*/
//�Ƿ�֧��uCOSII

#define SYSTEM_SUPPORT_OS		(0)		

/*
����ϵͳ�ļ����Ƿ�֧��UCOS
   0 ... ��֧��uCOS
	 1 ... ֧��  uCOS
*/

//�Ƿ��ô��ڴ�ӡ������Ϣ
#define DEBUG_PRINT         (0)
//����printf��ӡ��
#define DEBUG_UART_PORT     UART4       //ָ���Ǹ�������Ϊprintf�����
#define DEBUG_UART_BAUD     (115200)    //printf ������
//�ض��峣�����ݱ�������
typedef signed long  s32;
typedef signed short s16;
typedef signed char  s8;
typedef unsigned long  u32;
typedef unsigned short u16;
typedef unsigned char  u8;
//���ó������궨��
#ifndef TRUE
	#define TRUE		(1)
#endif
#ifndef FLASE
	#define FALSE		(0)
#endif
#ifndef NULL
	#define NULL 0
#endif
//λ  ���� ��� ��ѯ
#define SET_BIT(BitNumber, Register)        (Register |=(1<<BitNumber))
#define CLR_BIT(BitNumber, Register)        (Register &=~(1<<BitNumber))
#define CHK_BIT(BitNumber, Register)        (Register & (1<<BitNumber))
//�����ֵ����Сֵ 
#define  MAX( x, y ) ( ((x) > (y)) ? (x) : (y) ) 
#define  MIN( x, y ) ( ((x) < (y)) ? (x) : (y) ) 
//�õ�һ���ṹ����field��ռ�õ��ֽ��� 
#define FSIZ( type, field ) sizeof( ((type *) 0)->field ) 
//�õ�һ�������ĵ�ַ��word��ȣ� 
#define  B_PTR( var )  ( (byte *) (void *) &(var) ) 
#define  W_PTR( var )  ( (word *) (void *) &(var) ) 
//��������Ԫ�صĸ��� 
#define  ARR_SIZE( a )  ( sizeof( (a) ) / sizeof( (a[0]) ) ) 

#if (CLOCK_SETUP == 0)
    #define CPU_XTAL_CLK_HZ                 4000000u  //ʱ�ӷ�����Ƶ��Hz 
    #define CPU_XTAL32k_CLK_HZ              32768u    //ʱ�ӷ�����Ƶ��Hz 
    #define CPU_INT_SLOW_CLK_HZ             32768u    //�����ڲ�ʱ������Ƶ�� Hz 
    #define CPU_INT_FAST_CLK_HZ             4000000u  //�����ڲ�ʱ������Ƶ�� Hz
    #define DEFAULT_SYSTEM_CLOCK            96000000u //Ĭ��ϵͳʱ��Ƶ��
#elif (CLOCK_SETUP == 1)
    #define CPU_XTAL_CLK_HZ                 8000000u  //ʱ�ӷ�����Ƶ��Hz 
    #define CPU_XTAL32k_CLK_HZ              32768u    //ʱ�ӷ�����Ƶ��Hz /
    #define CPU_INT_SLOW_CLK_HZ             32768u    //�����ڲ�ʱ������Ƶ�� Hz 
    #define CPU_INT_FAST_CLK_HZ             4000000u  //�����ڲ�ʱ������Ƶ�� Hz
    #define DEFAULT_SYSTEM_CLOCK            96000000u //Ĭ��ϵͳʱ��Ƶ��
#elif (CLOCK_SETUP == 2)
    #define CPU_XTAL_CLK_HZ                 8000000u  //ʱ�ӷ�����Ƶ��Hz 
    #define CPU_XTAL32k_CLK_HZ              32768u    //ʱ�ӷ�����Ƶ��Hz /
    #define CPU_INT_SLOW_CLK_HZ             32768u    //�����ڲ�ʱ������Ƶ�� Hz 
    #define CPU_INT_FAST_CLK_HZ             4000000u  //�����ڲ�ʱ������Ƶ�� Hz
    #define DEFAULT_SYSTEM_CLOCK            200000000u //Ĭ��ϵͳʱ��Ƶ��
#elif (CLOCK_SETUP == 3)
    #define CPU_XTAL_CLK_HZ                 50000000u  //ʱ�ӷ�����Ƶ��Hz 
    #define CPU_XTAL32k_CLK_HZ              32768u    //ʱ�ӷ�����Ƶ��Hz /
    #define CPU_INT_SLOW_CLK_HZ             32768u    //�����ڲ�ʱ������Ƶ�� Hz 
    #define CPU_INT_FAST_CLK_HZ             4000000u  //�����ڲ�ʱ������Ƶ�� Hz
    #define DEFAULT_SYSTEM_CLOCK            96000000u //Ĭ��ϵͳʱ��Ƶ��
#elif (CLOCK_SETUP == 4)
    #define CPU_XTAL_CLK_HZ                 50000000u  //ʱ�ӷ�����Ƶ��Hz 
    #define CPU_XTAL32k_CLK_HZ              32768u    //ʱ�ӷ�����Ƶ��Hz /
    #define CPU_INT_SLOW_CLK_HZ             32768u    //�����ڲ�ʱ������Ƶ�� Hz 
    #define CPU_INT_FAST_CLK_HZ             4000000u  //�����ڲ�ʱ������Ƶ�� Hz
    #define DEFAULT_SYSTEM_CLOCK            200000000u //Ĭ��ϵͳʱ��Ƶ��
#endif
//��������Ϣ����Ƶ��Ϣ�ṹ
typedef struct
{
	u8 FamilyType;    //Kinetisϵ�м������ͺ� 
	u16 PinCnt;       //��������
	u8 ResetState;    //��λԭ��
	u8 SiliconRev;    //SiliconRev
	u32 PFlashSize;   //PFlash��С
	u32 FlexNVMSize;  //FlexNVM��С
	u32 RAMSize;      //RAM��С
	u8  ClkOpt;       //��Ƶѡ��
	u32 CoreClock;    //�ں�ʱ��
	u32 BusClock;     //����ʱ��
	u32 FlexBusClock; //FlexBusʱ��
	u32 FlashClock;   //Flashʱ��
} CPUInfoType_t;
extern CPUInfoType_t CPUInfo;
//������ʵ�ֵĽӿں����б�
void SystemSoftReset(void);                                     //��λ
void GetCPUInfo(void);                                          //��ô�������Ϣ
void SystemInit (void);                                         //ϵͳ��ʼ��
void SystemCoreClockUpdate (void);                              //���¼����ں�Ƶ��
void NVIC_Pgc (u32 PriorityGroup);                              //����NVIC�ж���
void NVIC_Setp (IRQn_Type IRQn, u32 priority);                  //����NVIC�ж����ȼ�
void EnableInterrupts(void);                                    //ʱ�����ж�
void DisableInterrupts(void);                                   //�ر����ж�
void SetVectorTableAdress(u32 offset);                          //�����ж�������ʼλ��
#endif
