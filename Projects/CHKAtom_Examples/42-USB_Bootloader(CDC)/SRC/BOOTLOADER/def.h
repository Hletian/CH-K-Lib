/***************************************************************************************************
		@File Name: 	global.h
		@Brief:			��bootloader��صĳ������ṹ�嶨��
		@Create Date:	2011.11.09
		@Revision History:
						2011.11.10 ���Ӵ��ڴ���ṹ��

****************************************************************************************************/

#ifndef _DEF_H_
#define _DEF_H_

#include <stdint.h>

/**************************************************************************************************

								����Ϊ����״̬����

***************************************************************************************************/


#define VERSION				0
#define SUB_VER				1

// #define USING_WDT
// #define xUSING_PROTECTION


#define BUF_SIZE            3072UL      /* �����С */

#define FEED_WDI_TIME		50			/* ι��ʱ�� */



/*************************************************************************************************

                ����Ϊ��FLASH��ص�����

**************************************************************************************************/


#define BASE_ADDR		((uint32_t)0x0000000UL)   /* FLASH��ʼ��ַ */

#define BOOTLOADER_ADDR		(BASE_ADDR)	          /* bootloader ��ʼ��ַ */
#define BOOTLOADER_SIZE		(20480UL)		      /* bootloader ��С :20KB */

  #define FLASH_SIZE              (512*1024UL)
  #define FLASH_PAGE_SIZE         2048UL
  #define END_ADDR                (BASE_ADDR + 512 * 1024UL-1)

#define APP_START_ADDR          (BASE_ADDR + BOOTLOADER_SIZE)
#define APP_SIZE                (492 * 1024UL)

#define APP_FLAG_ADDR           (APP_START_ADDR + APP_SIZE - sizeof(AppType_t))

#define UPDATE_START_ADDR       (APP_START_ADDR + APP_SIZE)
#define UPDATE_SIZE             APP_SIZE
#define UPDATE_FLAG_ADDR        (UPDATE_START_ADDR + UPDATE_SIZE - sizeof(AppType_t))

#define FLASH_PAGGS(flashsize, pg_size)   ( (flashsize) / (pg_size) + (((flashsize) % (pg_size) >0)?1:0) )

#define PARAMS_START_ADDR	(UPDATE_START_ADDR + UPDATE_SIZE)	/* ������ʼ��ַ */	


#endif
