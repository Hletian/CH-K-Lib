#include "usb_hid.h"
#include "usb.h"
u8 USB_HID_SendBuf[32]; //HID ���ͻ���
u8 USB_HID_SendReportLen;
//���ͱ���
void USB_HID_SendReport(void)
{
	u8 i;
	USB_EP_IN_Transfer(EP2,USB_HID_SendBuf,USB_HID_SendReportLen);
	for(i=0;i<USB_HID_SendReportLen;i++) //��ձ��滺����
	{
		USB_HID_SendBuf[i] = 0;
	}
}
//USB���ձ���
u8 USB_HID_RecReport(u8* buf)
{
	u8 *pu8EPBuffer;
	u8 temp;
	u8 len = 0;
	len = USB_EP_OUT_SizeCheck(EP3);
	temp = len;
	if(len)
	{
		pu8EPBuffer = BufferPointer[bEP3OUT_ODD];
		while(len--) *buf++=*pu8EPBuffer++;
	}
	len = temp;
	return len;
}
//������ݷ��ͺ���
void USB_HID_SetMouse(u8 HOffset,u8 VOffset,u8 SOffset,u8 Key)
{
		USB_HID_SendReportLen = 4;
		USB_HID_SendBuf[1] = HOffset;
		USB_HID_SendBuf[2] = VOffset;
		USB_HID_SendBuf[3] = SOffset;
		USB_HID_SendBuf[0] = Key;
}
//ͨ������ı����������Ķ��壬����֪�����ص����뱨�����8�ֽڡ�
//��һ�ֽڵ�8��bit������ʾ������Ƿ��£�����Shift��Alt�ȼ�����
//�ڶ��ֽ�Ϊ����ֵ��ֵΪ����0���������ڰ��ֽ���һ����ͨ����ֵ��
//���飬��û�м�����ʱ��ȫ��6���ֽ�ֵ��Ϊ0����ֻ��һ����ͨ������ʱ��
//�������ֽ��еĵ�һ�ֽ�ֵ��Ϊ�ð����ļ�ֵ������ļ�ֵ�뿴HID��
//��;���ĵ��������ж����ͨ��ͬʱ����ʱ����ͬʱ������Щ���ļ�ֵ��
//������µļ�̫�࣬���������ֽڶ�Ϊ0xFF�����ܷ���0x00����������
//����ϵͳ��Ϊ���м����Ѿ��ͷţ������ڼ�ֵ�������е��Ⱥ�˳����
//����ν�ģ�����ϵͳ�Ḻ�����Ƿ����¼����¡�����Ӧ�����ж϶˵�1
//�а�������ĸ�ʽ����ʵ�ʵļ������ݡ����⣬�����л�������һ���ֽ�
//��������棬����������LED����ġ�ֻʹ���˵�7λ����1λ�Ǳ���ֵ0��
//��ĳλ��ֵΪ1ʱ�����ʾ��Ӧ��LEDҪ����������ϵͳ�Ḻ��ͬ������
//����֮���LED����������������̣�һ������ּ��̵���ʱ����һ��
//Ҳ������������̱�����Ҫ�жϸ���LEDӦ�ú�ʱ������ֻ�ǵȴ�����
//���ͱ��������Ȼ����ݱ���ֵ��������Ӧ��LED�������ڶ˵�1����ж�
//�ж�����1�ֽڵ�������棬Ȼ�����ȡ������Ϊѧϰ���ϵ�LED�ǵ͵�ƽʱ
//������ֱ�ӷ��͵�LED�ϡ�����main�����а�������LED�Ĵ���Ͳ���Ҫ�ˡ�
//�������ݷ��ͺ���

//USBHID ���ü�ֵ �����ðٶ���
/*
��HID�ĵ���ժ�����ģ�����Ҳο�һ��
0 00 Reserved (no event indicated)9 N/A �� �� �� 4/101/104
1 01 Keyboard ErrorRollOver9 N/A �� �� �� 4/101/104
2 02 Keyboard POSTFail9 N/A �� �� �� 4/101/104
3 03 Keyboard ErrorUndefined9 N/A �� �� �� 4/101/104
4 04 Keyboard a and A4 31 �� �� �� 4/101/104
5 05 Keyboard b and B 50 �� �� �� 4/101/104
6 06 Keyboard c and C4 48 �� �� �� 4/101/104
7 07 Keyboard d and D 33 �� �� �� 4/101/104
8 08 Keyboard e and E 19 �� �� �� 4/101/104
9 09 Keyboard f and F 34 �� �� �� 4/101/104
10 0A Keyboard g and G 35 �� �� �� 4/101/104
11 0B Keyboard h and H 36 �� �� �� 4/101/104
12 0C Keyboard i and I 24 �� �� �� 4/101/104
13 0D Keyboard j and J 37 �� �� �� 4/101/104
14 0E Keyboard k and K 38 �� �� �� 4/101/104
15 0F Keyboard l and L 39 �� �� �� 4/101/104
16 10 Keyboard m and M4 52 �� �� �� 4/101/104
17 11 Keyboard n and N 51 �� �� �� 4/101/104
18 12 Keyboard o and O4 25 �� �� �� 4/101/104
19 13 Keyboard p and P4 26 �� �� �� 4/101/104
20 14 Keyboard q and Q4 17 �� �� �� 4/101/104
21 15 Keyboard r and R 20 �� �� �� 4/101/104
22 16 Keyboard s and S4 32 �� �� �� 4/101/104
23 17 Keyboard t and T 21 �� �� �� 4/101/104
24 18 Keyboard u and U 23 �� �� �� 4/101/104
25 19 Keyboard v and V 49 �� �� �� 4/101/104
26 1A Keyboard w and W4 18 �� �� �� 4/101/104
27 1B Keyboard x and X4 47 �� �� �� 4/101/104
28 1C Keyboard y and Y4 22 �� �� �� 4/101/104
29 1D Keyboard z and Z4 46 �� �� �� 4/101/104
30 1E Keyboard 1 and !4 2 �� �� �� 4/101/104
31 1F Keyboard 2 and @4 3 �� �� �� 4/101/104
32 20 Keyboard 3 and #4 4 �� �� �� 4/101/104
33 21 Keyboard 4 and $4 5 �� �� �� 4/101/104
34 22 Keyboard 5 and %4 6 �� �� �� 4/101/104
35 23 Keyboard 6 and ^4 7 �� �� �� 4/101/104
36 24 Keyboard 7 and &4 8 �� �� �� 4/101/104
37 25 Keyboard 8 and *4 9 �� �� �� 4/101/104
38 26 Keyboard 9 and (4 10 �� �� �� 4/101/104
39 27 Keyboard 0 and )4 11 �� �� �� 4/101/104

*/
#define NULL   0
//����KeyBoard����
//Fn Key�����⹦�ܼ��� ���λ1Ctrl ����ֱ�Alt Tab
//�����Ǽ��̻������� ÿ��ֻ��6������ͬʱ����
u8 USB_HID_SetKeyBoard(u8 FnKey,u8 *Keybuf)
{
	u8 i;
	if(Keybuf == NULL) return 1;
	USB_HID_SendReportLen = 8;
	USB_HID_SendBuf[0] = FnKey;
	for(i=2;i<8;i++)
	{
		USB_HID_SendBuf[i] = Keybuf[i-2]; 
	}
}


