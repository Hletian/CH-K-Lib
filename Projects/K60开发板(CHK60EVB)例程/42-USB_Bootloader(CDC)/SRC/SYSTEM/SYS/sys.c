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
#include "sys.h"
#include <string.h>
CPUInfoType_t CPUInfo; //������������Ϣ�ṹ��
uint32_t SystemCoreClock = DEFAULT_SYSTEM_CLOCK;
/***********************************************************************************************
 ���ܣ����ϵͳ��Ϣ
 �βΣ�0
 ���أ�0
 ��⣺��ô������ͺ� ����   ����ں�ʱ��,����ʱ�ӣ�FlexBusʱ�ӣ�Flashʱ�� ��     
************************************************************************************************/
void GetCPUInfo(void)
{
	//���Ȼ�ø�λ״̬��Ϣ
	if (MC->SRSH & MC_SRSH_SW_MASK)     CPUInfo.ResetState = 1;
	if (MC->SRSH & MC_SRSH_LOCKUP_MASK) CPUInfo.ResetState = 2;
	if (MC->SRSH & MC_SRSH_JTAG_MASK)   CPUInfo.ResetState = 3;
	if (MC->SRSL & MC_SRSL_POR_MASK)    CPUInfo.ResetState = 4;
	if (MC->SRSL & MC_SRSL_PIN_MASK)  	CPUInfo.ResetState = 5;
	if (MC->SRSL & MC_SRSL_COP_MASK)    CPUInfo.ResetState = 6;
	if (MC->SRSL & MC_SRSL_LOC_MASK)    CPUInfo.ResetState = 7;
	if (MC->SRSL & MC_SRSL_LVD_MASK)    CPUInfo.ResetState = 8;
	if (MC->SRSL & MC_SRSL_WAKEUP_MASK) CPUInfo.ResetState = 9;
	//ȷ��Kinetisϵ���ͺ�
	switch((SIM->SDID & SIM_SDID_FAMID(0x7))>>SIM_SDID_FAMID_SHIFT) 
	{  
    	case 0x0: CPUInfo.FamilyType = 10; break;
    	case 0x1: CPUInfo.FamilyType = 20; break;
		  case 0x2: CPUInfo.FamilyType = 30; break;
			case 0x3: CPUInfo.FamilyType = 40; break;
			case 0x4: CPUInfo.FamilyType = 60; break;
			case 0x5: CPUInfo.FamilyType = 70; break;
			case 0x6: CPUInfo.FamilyType = 50; break;
			case 0x7: CPUInfo.FamilyType = 53; break;
			 default: CPUInfo.FamilyType = 0;  break;
	}
	//��װ��Ϣ
	switch((SIM->SDID & SIM_SDID_PINID(0xF))>>SIM_SDID_PINID_SHIFT) 
	{  
		case 0x02: CPUInfo.PinCnt = 32;  break;
		case 0x04: CPUInfo.PinCnt = 48;  break;
		case 0x05: CPUInfo.PinCnt = 64;  break;
		case 0x06: CPUInfo.PinCnt = 80;  break;
		case 0x07: CPUInfo.PinCnt = 81;  break;
		case 0x08: CPUInfo.PinCnt = 100; break;
		case 0x09: CPUInfo.PinCnt = 104; break;
		case 0x0A: CPUInfo.PinCnt = 144; break;
		case 0x0C: CPUInfo.PinCnt = 196; break;
 		case 0x0E: CPUInfo.PinCnt = 256; break;
	  default:  CPUInfo.PinCnt = 0;   break;	
	}  
   //SiliconRevID
    switch((SIM->SDID & SIM_SDID_REVID(0xF))>>SIM_SDID_REVID_SHIFT) 
    { 
			case 0x0: CPUInfo.SiliconRev = 10;break;
			case 0x1: CPUInfo.SiliconRev = 11;break;
			case 0x2: CPUInfo.SiliconRev = 12;break;
			default:  CPUInfo.SiliconRev = 0; break;
    }	
	//PFlash��С
	switch((SIM->FCFG1 & SIM_FCFG1_PFSIZE(0xF))>>SIM_FCFG1_PFSIZE_SHIFT)
	{
		case 0x7: CPUInfo.PFlashSize = 128*1024;break;
		case 0x9: CPUInfo.PFlashSize = 256*1024;break;
		case 0xB: CPUInfo.PFlashSize = 512*1024;break;
		case 0xF: CPUInfo.PFlashSize = 512*1024;break;
	 	default:  CPUInfo.PFlashSize = 0*1024;  break; 		
	}
	//�����FlexNVM ȷ��FlexNVM��С
	if (SIM->FCFG2 & SIM_FCFG2_PFLSH_MASK) 
	{
		CPUInfo.FlexNVMSize = 0;
	}
	else
	{
		//ȷ��FLexNVM��С
		switch((SIM->FCFG1 & SIM_FCFG1_NVMSIZE(0xF))>>SIM_FCFG1_NVMSIZE_SHIFT)
		{
			case 0x0: CPUInfo.FlexNVMSize = 0;break;
			case 0x7: CPUInfo.FlexNVMSize = 128*1024;break;
			case 0x9: CPUInfo.FlexNVMSize = 256*1024;break;
			case 0xF: CPUInfo.FlexNVMSize = 256*1024;break;
			default:  CPUInfo.FlexNVMSize = 0;break; 		
		}
	}
	//RAM��С
	switch((SIM->SOPT1 & SIM_SOPT1_RAMSIZE(0xF))>>SIM_SOPT1_RAMSIZE_SHIFT)
	{
		case 0x5: CPUInfo.RAMSize = 32*1024;  break;
		case 0x7: CPUInfo.RAMSize = 64*1024;  break;
		case 0x8: CPUInfo.RAMSize = 96*1024;  break;
		case 0x9: CPUInfo.RAMSize = 128*1024; break;
		default:  CPUInfo.RAMSize = 0*1024;   break;	
	}
	//ʱ�Ӽ���
	SystemCoreClockUpdate();            //���¼���CoreClock
	CPUInfo.ClkOpt = CLOCK_SETUP;
	CPUInfo.CoreClock = SystemCoreClock;
	CPUInfo.BusClock=(SystemCoreClock/(((SIM->CLKDIV1&SIM_CLKDIV1_OUTDIV2_MASK)>>SIM_CLKDIV1_OUTDIV2_SHIFT)+1));     //����BusClock
	CPUInfo.FlexBusClock=(SystemCoreClock/(((SIM->CLKDIV1&SIM_CLKDIV1_OUTDIV3_MASK)>>SIM_CLKDIV1_OUTDIV3_SHIFT)+1)); //����FlexBusClock
	CPUInfo.FlashClock=(SystemCoreClock/(((SIM->CLKDIV1&SIM_CLKDIV1_OUTDIV4_MASK)>>SIM_CLKDIV1_OUTDIV4_SHIFT)+1));   //����FlashClock
}
/***********************************************************************************************
 ���ܣ���ʼ��ϵͳʱ�� 
 �βΣ�0
 ���أ�0
 ��⣺��ʼ��ϵͳʱ�ӣ�ʹ��PLL����FLL ��Ƶ ���CoreClock BusClock�� 
************************************************************************************************/
void SystemInit (void) 
{
	WDOG_Close();
	//ϵʱ�ӳ�ʼ��
	//�ڲ����� CLOCK_SETUP=0
#if (CLOCK_SETUP == 0)
	//SIM->CLKDIV1 = (u32)0x00110000u; //����ϵͳԤ��Ƶ��
	SIM->CLKDIV1 = (u32)0xFFFFFFFFu; //����ϵͳԤ��Ƶ�� ������Ϊ��Ϊ��ͷ�Ƶ
	// ת�� FEI ģʽ 
	MCG->C1 = (u8)0x06u;
	MCG->C2 = (u8)0x00u;
	MCG->C4|= (1<<6)|(1<<7)|(1<<5);   //�ڲ��ο�����ʱ��32.768KHZ  ��Ƶ���� 2197 ��Ƶ��Ϊ96MHZ �μ�MCG->C4�Ĵ���
	//��Ƶ����:  
	//SIM_CLKDIV1_OUTDIV1(0) CORE     CLOCK  1��Ƶ   UP TO 100M  
	//SIM_CLKDIV1_OUTDIV2(1) BUS      CLOCK  2��Ƶ   UP TO 50M 
	//SIM_CLKDIV1_OUTDIV3(1) FlexBus  ClOCK  2��Ƶ   UP TO 50M 
	//SIM_CLKDIV1_OUTDIV4(3) Flash    ClOCK  3��Ƶ   UP TO 25M 
	SIM->CLKDIV1 =(SIM_CLKDIV1_OUTDIV1(0)|SIM_CLKDIV1_OUTDIV2(1)|SIM_CLKDIV1_OUTDIV3(1)|SIM_CLKDIV1_OUTDIV4(3));
  MCG->C5 = (u8)0x00u;
  MCG->C6 = (u8)0x00u;
  while((MCG->S & MCG_S_IREFST_MASK) == 0u);  //��� FLL�ο�ʱ�����ڲ��ο�ʱ��
  while((MCG->S & 0x0Cu) != 0x00u);           //�ȴ�FLL��ѡ��
	//�ⲿ���� CLOCK_SETUP=1
#elif (CLOCK_SETUP == 1)
	SIM->CLKDIV1 =(SIM_CLKDIV1_OUTDIV1(0)|SIM_CLKDIV1_OUTDIV2(1)|SIM_CLKDIV1_OUTDIV3(1)|SIM_CLKDIV1_OUTDIV4(3));
  // ת�� FBE ģʽ 
	OSC->CR = (u8)0x00u;
	SIM->SOPT2 &= (u8)~(u8)0x01u;
	MCG->C2 = (u8)0x24u;
	MCG->C1 = (u8)0x9Au;
	MCG->C4 &= (u8)~(uint8_t)0xE0u;
	MCG->C5 = (u8)0x03u;
	MCG->C6 = (u8)0x00u;
	while((MCG->S & MCG_S_OSCINIT_MASK) == 0u);//��� FLL�ο�ʱ�����ڲ��ο�ʱ�� 
	while((MCG->S & MCG_S_IREFST_MASK) != 0u); //��� FLL�ο�ʱ�����ڲ��ο�ʱ��
	while((MCG->S & 0x0Cu) != 0x08u);          //�ȴ� FBE ��ѡ��
	MCG->C5 = (u8)0x03u;
	MCG->C6 = (u8)(0x40u|0x18u);		    //PLL48��Ƶ
	while((MCG->S & MCG_S_PLLST_MASK) == 0u);   //�ȴ�PLLS ʱ��Դת�� PLL
	while((MCG->S & MCG_S_LOCK_MASK) == 0u);    //�ȴ�����
  //ת��PEE
	MCG->C1 = (uint8_t)0x1Au;
	while((MCG->S & 0x0Cu) != 0x0Cu);           //�ȴ�PLL���
	while((MCG->S & MCG_S_LOCK_MASK) == 0u);      //�ȴ�PLL����
	//�ⲿ���� ��Ƶ CLOCK_SETUP=2
#elif (CLOCK_SETUP == 2)
	SIM->CLKDIV1 =(SIM_CLKDIV1_OUTDIV1(0)|SIM_CLKDIV1_OUTDIV2(1)|SIM_CLKDIV1_OUTDIV3(1)|SIM_CLKDIV1_OUTDIV4(7));
  // ת�� FBE ģʽ 
	OSC->CR = (u8)0x00u;
	SIM->SOPT2 &= (u8)~(u8)0x01u;
	MCG->C2 = (u8)0x24u;
	MCG->C1 = (u8)0x9Au;
	MCG->C4 &= (u8)~(uint8_t)0xE0u;
	MCG->C5 = (u8)0x03u;
	MCG->C6 = (u8)0x00u;
	while((MCG->S & MCG_S_OSCINIT_MASK) == 0u);//��� FLL�ο�ʱ�����ڲ��ο�ʱ�� 
	while((MCG->S & MCG_S_IREFST_MASK) != 0u); //��� FLL�ο�ʱ�����ڲ��ο�ʱ��
	while((MCG->S & 0x0Cu) != 0x08u);          //�ȴ� FBE ��ѡ��
	MCG->C5 = (u8)0x01u; //4��Ƶ
	MCG->C6 = (u8)(0x40u|MCG_C6_VDIV(26));		    //PLL48��Ƶ
	while((MCG->S & MCG_S_PLLST_MASK) == 0u);   //�ȴ�PLLS ʱ��Դת�� PLL
	while((MCG->S & MCG_S_LOCK_MASK) == 0u);    //�ȴ�����
  //ת��PEE
	MCG->C1 = (uint8_t)0x1Au;
	while((MCG->S & 0x0Cu) != 0x0Cu);           //�ȴ�PLL���
	while((MCG->S & MCG_S_LOCK_MASK) == 0u);      //�ȴ�PLL����
#elif (CLOCK_SETUP == 3)
	SIM->CLKDIV1 =(SIM_CLKDIV1_OUTDIV1(0)|SIM_CLKDIV1_OUTDIV2(1)|SIM_CLKDIV1_OUTDIV3(1)|SIM_CLKDIV1_OUTDIV4(3));
  // ת�� FBE ģʽ 
	OSC->CR = (u8)0x00u;
	SIM->SOPT2 &= (u8)~(u8)0x01u;
	MCG->C2 = (u8)0x24u;
	MCG->C1 = (u8)0x9Au;
	MCG->C4 &= (u8)~(uint8_t)0xE0u;
	MCG->C5 = (u8)0x03u;
	MCG->C6 = (u8)0x00u;
	while((MCG->S & MCG_S_OSCINIT_MASK) == 0u);//��� FLL�ο�ʱ�����ڲ��ο�ʱ�� 
	while((MCG->S & MCG_S_IREFST_MASK) != 0u); //��� FLL�ο�ʱ�����ڲ��ο�ʱ��
	while((MCG->S & 0x0Cu) != 0x08u);          //�ȴ� FBE ��ѡ��
	MCG->C5 = (u8)0x18u; //25��Ƶ /50M/25 =2M 
	MCG->C6 = (u8)(0x40u|MCG_C6_VDIV(24));		    //PLL48��Ƶ  ��� 50/25*48=96M 
	while((MCG->S & MCG_S_PLLST_MASK) == 0u);   //�ȴ�PLLS ʱ��Դת�� PLL
	while((MCG->S & MCG_S_LOCK_MASK) == 0u);    //�ȴ�����
  //ת��PEE
	MCG->C1 = (uint8_t)0x1Au;
	while((MCG->S & 0x0Cu) != 0x0Cu);           //�ȴ�PLL���
	while((MCG->S & MCG_S_LOCK_MASK) == 0u);      //�ȴ�PLL����
#elif (CLOCK_SETUP == 4)
	SIM->CLKDIV1 =(SIM_CLKDIV1_OUTDIV1(0)|SIM_CLKDIV1_OUTDIV2(1)|SIM_CLKDIV1_OUTDIV3(1)|SIM_CLKDIV1_OUTDIV4(7));
  // ת�� FBE ģʽ 
	OSC->CR = (u8)0x00u;
	SIM->SOPT2 &= (u8)~(u8)0x01u;
	MCG->C2 = (u8)0x24u;
	MCG->C1 = (u8)0x9Au;
	MCG->C4 &= (u8)~(uint8_t)0xE0u;
	MCG->C5 = (u8)0x03u;
	MCG->C6 = (u8)0x00u;
	while((MCG->S & MCG_S_OSCINIT_MASK) == 0u);//��� FLL�ο�ʱ�����ڲ��ο�ʱ�� 
	while((MCG->S & MCG_S_IREFST_MASK) != 0u); //��� FLL�ο�ʱ�����ڲ��ο�ʱ��
	while((MCG->S & 0x0Cu) != 0x08u);          //�ȴ� FBE ��ѡ��
	MCG->C5 = MCG_C5_PRDIV(12); //13��Ƶ /50M/13 = XXM
	MCG->C6 = (u8)(0x40u|MCG_C6_VDIV(28));		    //PLL48��Ƶ  ��� 50/13*52=200M 
	while((MCG->S & MCG_S_PLLST_MASK) == 0u);   //�ȴ�PLLS ʱ��Դת�� PLL
	while((MCG->S & MCG_S_LOCK_MASK) == 0u);    //�ȴ�����
  //ת��PEE
	MCG->C1 = (uint8_t)0x1Au;
	while((MCG->S & 0x0Cu) != 0x0Cu);           //�ȴ�PLL���
	while((MCG->S & MCG_S_LOCK_MASK) == 0u);      //�ȴ�PLL����
#endif 
	GetCPUInfo();  							//����ϵͳʱ��
}
/***********************************************************************************************
 ���ܣ�������Ƶ
 �βΣ�0
 ���أ�0
 ��⣺���¼����ں�ʱ��Ƶ�� �û����޸���Ƶ����������������Լ�����µ���Ƶ
************************************************************************************************/
void SystemCoreClockUpdate(void)
{
	u32 MCGOUTClock;            //�洢MCGģ��ʱ��Ƶ�ʱ���
	u8 Divider;
	if ((MCG->C1 & MCG_C1_CLKS_MASK) == 0x0u) 
	{
   //ѡ��FLL����PLL���
		if ((MCG->C6 & MCG_C6_PLLS_MASK) == 0x0u) //FLL
	  {
			if ((MCG->C1 & MCG_C1_IREFS_MASK) == 0x0u) 
			{
				if ((SIM->SOPT2 & SIM_SOPT2_MCGCLKSEL_MASK) == 0x0u) 
				{
            MCGOUTClock = CPU_XTAL_CLK_HZ;   //ϵͳ�������� MCG ʱ��
				}
				else 
				{ 
					MCGOUTClock = CPU_XTAL32k_CLK_HZ;  //RTC 32 kHz �������� MCG ʱ��
				} 
				Divider = (uint8_t)(1u << ((MCG->C1 & MCG_C1_FRDIV_MASK) >> MCG_C1_FRDIV_SHIFT));
				MCGOUTClock = (MCGOUTClock / Divider);  //����FLL�ο�ʱ�ӷ�Ƶ
				if ((MCG->C2 & MCG_C2_RANGE_MASK) != 0x0u) 
				{
					MCGOUTClock /= 32u;  //����߷�Χʹ�ܣ������32λ��Ƶ����Ч��
				} 
			} 
	   else 
	   {
        MCGOUTClock = CPU_INT_SLOW_CLK_HZ;     //ѡ�������ڲ��ο�ʱ��
       } 
      /* ѡ����ȷ�ĳ���ȥ���� MCG ���ʱ��*/
       switch (MCG->C4 & (MCG_C4_DMX32_MASK | MCG_C4_DRST_DRS_MASK)) {
        case 0x0u:
          MCGOUTClock *= 640u;
          break;
        case 0x20u:
          MCGOUTClock *= 1280u;
          break;
        case 0x40u:
          MCGOUTClock *= 1920u;
          break;
        case 0x60u:
          MCGOUTClock *= 2560u;
          break;
        case 0x80u:
          MCGOUTClock *= 732u;
          break;
        case 0xA0u:
          MCGOUTClock *= 1464u;
          break;
        case 0xC0u:
          MCGOUTClock *= 2197u;
          break;
        case 0xE0u:
          MCGOUTClock *= 2929u;
          break;
        default:
          break;
      }
    } 
	else 
	{ 
      /* ѡ�� PLL */
      Divider = (1u + (MCG->C5 & MCG_C5_PRDIV_MASK));
      MCGOUTClock = (u32)(CPU_XTAL_CLK_HZ / Divider);   //���� PLL �ο�ʱ��
      Divider = ((MCG->C6 & MCG_C6_VDIV_MASK) + 24u);
      MCGOUTClock *= Divider;        //���� MCG ���ʱ��
    } 
  }
  else if ((MCG->C1 & MCG_C1_CLKS_MASK) == 0x40u) 
     {
       /* ѡ���ڲ��ο�ʱ�� */
       if ((MCG->C2 & MCG_C2_IRCS_MASK) == 0x0u) 
	   {
         MCGOUTClock = CPU_INT_SLOW_CLK_HZ;    //ѡ�������ڲ��ο�ʱ�� 
       } 
	   else 
	   { 
         MCGOUTClock = CPU_INT_FAST_CLK_HZ;   //ѡ���ڲ���Ĳο�ʱ�� 
       } 
     } 
	 else if ((MCG->C1 & MCG_C1_CLKS_MASK) == 0x80u) 
	   {
         /* ѡ���ⲿ�ο�ʱ��*/
         if ((SIM->SOPT2 & SIM_SOPT2_MCGCLKSEL_MASK) == 0x0u) 
		 {
            MCGOUTClock = CPU_XTAL_CLK_HZ;     //ϵͳ�������� MCG ʱ��
         } 
		 else 
		 { 
            MCGOUTClock = CPU_XTAL32k_CLK_HZ;  //RTC 32 kHz �������� MCG ʱ��
         } 
       } 
	   else 
	   { 
          return;
       } 
   SystemCoreClock = (MCGOUTClock / (1u + ((SIM->CLKDIV1 & SIM_CLKDIV1_OUTDIV1_MASK) >> SIM_CLKDIV1_OUTDIV1_SHIFT)));
}
/***********************************************************************************************
 ���ܣ������жϷ�������
 �βΣ�PriorityGroup ��д���֡�0~7��
 ���أ�0
 ��⣺�˳����ǽ����жϷ������һ������һ�Σ������ظ�����
       һ��ϵͳ������һ�ַ��飬ÿ�ַ����������16���жϣ�����
		   ���ö���ж���ͬһ���ȼ���ϵͳ���������жϴ������Ȱ����ж�������
       ���˳����д���
************************************************************************************************/
void NVIC_Pgc(u32 PriorityGroup)
{
    u32 reg_value;
    u32 PriorityGroupTmp = (PriorityGroup & (uint32_t)0x07);            
    reg_value  =  SCB->AIRCR;                                    //���ɵļĴ������� 
    reg_value &= ~(SCB_AIRCR_VECTKEY_Msk | SCB_AIRCR_PRIGROUP_Msk);          
    reg_value  =  (reg_value|((uint32_t)0x5FA << SCB_AIRCR_VECTKEY_Pos)|
                    (PriorityGroupTmp << 8));            //дԿ�׺����ȼ�����
    SCB->AIRCR =  reg_value;
}
/***********************************************************************************************
 ���ܣ������ж����ȼ�����
 �βΣ�IRQn ��д�жϱ�ţ�priority �������ȼ�
 ���أ�0
 ��⣺�˺���COPY NVIC_SetPriority(IRQn_Type IRQn, u32 priority)���core_cm4.h
************************************************************************************************/
void NVIC_Setp(IRQn_Type IRQn, u32 priority)
{
  if(IRQn < 0) {
    SCB->SHP[((u32)(IRQn) & 0xF)-4] = ((priority << (8 - __NVIC_PRIO_BITS)) & 0xff); } 
  else {
    NVIC->IP[(u32)(IRQn)] = ((priority << (8 - __NVIC_PRIO_BITS)) & 0xff);    }   //Ϊ�ж��������ȼ�
}
/***********************************************************************************************
 ���ܣ���λ
 �βΣ�0
 ���أ�0
 ��⣺��λ
************************************************************************************************/
void SystemSoftReset(void)
{   
	SCB->AIRCR =0X05FA0000|(u32)0x04;	  
} 
/***********************************************************************************************
 ���ܣ�ʹ�����ж�
 �βΣ�0
 ���أ�0
 ��⣺����  CMSIS Cortex-M Core Function Access �� ���� ��.core_cmFunc.h
************************************************************************************************/
void EnableInterrupts(void)
{
	__enable_irq();
}
/***********************************************************************************************
 ���ܣ��ر����ж�
 �βΣ�0
 ���أ�0
 ��⣺����  CMSIS Cortex-M Core Function Access �� ���� ��.core_cmFunc.h
************************************************************************************************/
void DisableInterrupts(void)
{
	__disable_irq();
}
/***********************************************************************************************
 ���ܣ������ж���������ʼλ��
 �βΣ�0
 ���أ�0
 ��⣺
************************************************************************************************/
void SetVectorTableAdress(u32 offset)
{
	SCB->VTOR = offset;  //Ϊ���Bootloader�����ж�������
}

