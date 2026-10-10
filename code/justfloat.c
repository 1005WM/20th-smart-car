#include"justfloat.h"

typedef union
{
    float fdata;
    unsigned long ldata;
} FloatLongType;
 
/*
将浮点数f转化为4个字节数据存放在byte[4]中
*/
void Float_to_Byte(float f,unsigned char byte[])
{
    FloatLongType fl;
    fl.fdata=f;
    byte[0]=(unsigned char)fl.ldata;
    byte[1]=(unsigned char)(fl.ldata>>8);
    byte[2]=(unsigned char)(fl.ldata>>16);
    byte[3]=(unsigned char)(fl.ldata>>24);
}
 
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     将单个float数据类型的数据通过串口发送
// 参数说明     a需要传送的两组数据
// 返回参数     void
// 使用示例     JustFloat_Test_one(sin(t));
// 备注信息     通过vofa串口显示波形图
//-------------------------------------------------------------------------------------------------------------------
 
void JustFloat_Test_one(float a)   //justfloat 数据协议测试//可传输两组数据
{
uint8 byte[4]={0};     //float转化为4个字节数据
uint8 tail[4]={0x00, 0x00, 0x80, 0x7f};    //帧尾
//向上位机发送两个通道数据
Float_to_Byte(a,byte);
//u1_printf("%f\r\n",a);
uart_write_buffer(UART_0,byte,4);   //1转化为4字节数据 就是  0x00 0x00 0x80 0x3F
//发送帧尾
uart_write_buffer(UART_0,tail,4);   //帧尾为 0x00 0x00 0x80 0x7f
}
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     将两组float数据类型的数据通过串口发送
// 参数说明     a、b需要传送的两组数据
// 返回参数     void
// 使用示例     JustFloat_Test_two(sin(t),sin(2*t));
// 备注信息     通过vofa串口显示波形图
//-------------------------------------------------------------------------------------------------------------------
 
void JustFloat_Test_two(float a, float b)   //justfloat 数据协议测试//可传输两组数据
{
 
uint8 byte[4]={0};     //float转化为4个字节数据
uint8 tail[4]={0x00, 0x00, 0x80, 0x7f};    //帧尾
 
//向上位机发送两个通道数据
Float_to_Byte(a,byte);
//u1_printf("%f\r\n",a);
uart_write_buffer(UART_0,byte,4);   //1转化为4字节数据 就是  0x00 0x00 0x80 0x3F
 
Float_to_Byte(b,byte);
uart_write_buffer(UART_0,byte,4);   //2转换为4字节数据 就是  0x00 0x00 0x00 0x40
 
//发送帧尾
uart_write_buffer(UART_0,tail,4);   //帧尾为 0x00 0x00 0x80 0x7f
 
}
 
 
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     将三组float数据类型的数据通过串口发送
// 参数说明     a、b、c需要传送的三组数据
// 返回参数     void
// 使用示例     JustFloat_Test_three(sin(t),sin(2*t)，sin(3*t));
// 备注信息     通过vofa串口显示波形图
//-------------------------------------------------------------------------------------------------------------------
 
void JustFloat_Test_three(float a, float b,float c)   //justfloat 数据协议测试
{
 
uint8 byte[4]={0};     //float转化为4个字节数据
uint8 tail[4]={0x00, 0x00, 0x80, 0x7f};    //帧尾
 
//向上位机发送两个通道数据
Float_to_Byte(a,byte);
//u1_printf("%f\r\n",a);
uart_write_buffer(UART_0,byte,4);   //1转化为4字节数据 就是  0x00 0x00 0x80 0x3F
 
Float_to_Byte(b,byte);
uart_write_buffer(UART_0,byte,4);   //2转换为4字节数据 就是  0x00 0x00 0x00 0x40
 
Float_to_Byte(c,byte);
uart_write_buffer(UART_0,byte,4);   //2转换为4字节数据 就是  0x00 0x00 0x00 0x40
//发送帧尾
uart_write_buffer(UART_0,tail,4);   //帧尾为 0x00 0x00 0x80 0x7f 
}
