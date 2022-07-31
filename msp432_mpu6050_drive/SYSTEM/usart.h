/****************************************************/
// MSP432P401R
// ��������
// Bilibili��m-RNA
// E-mail:m-RNA@qq.com
/****************************************************/

/******************   �汾����˵��   *****************
 * 
 * CCS֧��printf
 * Keil֧�ֱ�׼C���΢��
 * ��Keil�������ڿ��Բ���΢����
 * 
 * ? ��Ҫע�⣺
 * ��ʹ�ñ�׼C��ʱ�����޷�ʹ��scanf��
 * �����Ҫʹ��scanfʱ����ʹ��΢�� MicroLIB
 * �ٵ�Ƶʱ��Ƶ���£��߲�����ʹ�ô���ʱ������,
 * ����35768Hz��19200������,
 * ��ʹ�ô��������ʱ���Գ��Խ��Ͳ����ʡ�
 * ��baudrate_calculate��������ȥ�ļ��ڲ鿴��
 * 
 * **************************************************
 * 
 * ? v3.2  2021/10/28
 * �򻯶�CCS֧�ֵ�printf����
 *
 * ? v3.1  2021/10/18
 * ��Ӷ�CCS��printf֧��
 *
 * ? v3.0  2021/10/15
 * �˰汾֧��ʹ�� ��׼C��
 * �ļ���ʽ����Ϊ������ԭ��ͬ����
 * usart.c �� usart.h��������ֲ
 * ��֧��Keilƽ̨����
 *  
 * ? v2.1  2021/8/27
 * ���֧�ֹ̼���v3_21_00_05
 * ��֧�� MicroLIB ΢�⡢Keilƽ̨����
 * 
 * ? v2.0  2021/8/25
 * uart_init�����˲����ʴ����������ֱ�����ò����ʡ�
 * ����UART�Ĵ��뵥�����Ϊ��Ϊ
 * baudrate_calculate��c�ļ���h�ļ�
 * ��֧�� MicroLIB ΢�⡢Keilƽ̨����
 * 
 * ? v1.0 2021/7/17
 * ��֧�ֹ̼���v3_40_01_02
 * ������SMCLK 48MHz ������ 115200�ĳ�ʼ�����룬
 * �Խӱ�׼��������⣬ʹ����ʹ��printf��scanf����
 * ��֧�� MicroLIB ΢�⡢Keilƽ̨����
 * 
 ****************************************************/

#ifndef __USART_H
#define __USART_H
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
#include "stdio.h" //1.61328125kb

#ifdef __TI_COMPILER_VERSION__
//CCSƽ̨
#include "stdarg.h"
#include "string.h"
#define USART0_MAX_SEND_LEN     600                 //����ͻ����ֽ���
int printf(const char *str, ...);
#endif

void uart_init(uint32_t baudRate);

#endif
