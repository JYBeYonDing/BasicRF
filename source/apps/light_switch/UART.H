#ifndef __UART_H__
#define __UART_H__

extern void InitUart();   //初始化串口
extern void UartSendString(unsigned char *Data,int len);
// extern void uint32_2char(unsigned long num,unsigned char *Data); //无符号整形转化为字符串
// extern void uint16_2char(unsigned long num,unsigned char *Data); //无符号整形转化为字符串
extern void uint2char(unsigned long num,unsigned char *Data,int len);

#endif