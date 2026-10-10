#include"vofa.h"
#include <math.h> 

uint8 uart_get_data[64];                                                 // 串口接收数据缓冲区
uint8 fifo_get_data[64];                                                // fifo 输出读出缓冲区
uint8  get_data = 0;                                                  // 接收数据变量
uint32 fifo_data_count = 0;                                         // fifo 数据个数
fifo_struct uart_data_fifo;
PIDNUM NUM;

void Init_uart(void)
{
      fifo_init(&uart_data_fifo, FIFO_DATA_8BIT, uart_get_data, 64); // 初始化 fifo 挂载缓冲区
      uart_init(UART_INDEX, UART_BAUDRATE, UART_TX_PIN, UART_RX_PIN);   // 初始化串口
      uart_rx_interrupt(UART_INDEX, 1);                   // 开启 UART_INDEX 的接收中断
 
      uart_write_string(UART_INDEX, "UART Text.");                  // 输出测试信息
      uart_write_byte(UART_INDEX, '\r');                                   // 输出回车
      uart_write_byte(UART_INDEX, '\n');
}

void Send_test(void)
{
    fifo_data_count = fifo_used(&uart_data_fifo);                           // 查看 fifo 是否有数据
            if(fifo_data_count != 0)                                                // 读取到数据了
            {
                fifo_read_buffer(&uart_data_fifo, fifo_get_data, &fifo_data_count, FIFO_READ_AND_CLEAN);    // 将 fifo 中数据读出并清空 fifo 挂载的缓冲
                uart_write_string(UART_INDEX, "\r\nUART get data:");                // 输出测试信息
                uart_write_buffer(UART_INDEX, fifo_get_data, fifo_data_count);      // 将读取到的数据发送出去
            }
            system_delay_ms(10);
}



//-------------------------------------------------------------------------------------------------------------------
// 函数简介       UART_INDEX 的接收中断处理函数 这个函数将在 UART_INDEX 对应的中断调用
// 参数说明       void
// 返回参数       void
// 使用示例       uart_rx_interrupt_handler();
//-------------------------------------------------------------------------------------------------------------------
void uart_rx_interrupt_handler (void)
{
//    get_data = uart_read_byte(UART_INDEX);                                      // 接收数据 while 等待式 不建议在中断使用
    uart_query_byte(UART_INDEX, &get_data);                                     // 接收数据 查询式 有数据会返回 TRUE 没有数据会返回 FALSE
    fifo_write_buffer(&uart_data_fifo, &get_data, 1);                           // 将数据写入 fifo 中
}
//-------------------------------------------------------------------------------------------------------------------
// 函数简介       将vofa中回传的数据解析
// 参数说明       void
// 返回参数       void
// 使用示例      data=Get_Data();

//-------------------------------------------------------------------------------------------------------------------
float Get_Data(void)
{
    uint8_t data_Start_Num = 0; // 记录数据位开始的地方
    uint8_t data_End_Num = 0; // 记录数据位结束的地方
    uint8_t data_Num = 0; // 记录数据位数
    uint8_t minus_Flag = 0; // 判断是不是负数
    float data_return = 0; // 解析得到的数据
    uint8_t valid = 0;

    
    for(uint8_t i=0; i<64; i++) 
    {
        if(fifo_get_data[i] == '=') 
        {
            data_Start_Num = i + 1;
            valid |= 0x01;  // 标记找到等号
        }
        if(fifo_get_data[i] == '!') 
        {
            data_End_Num = i - 1;
            valid |= 0x02;  // 标记找到感叹号
            break;
        }
    }
    if(valid != 0x03) return 0.0f; // 检查格式有效性

    // 优化2：安全处理负数
    uint8_t cursor = data_Start_Num;
    if(cursor <= data_End_Num && fifo_get_data[cursor] == '-') 
    {
        minus_Flag = 1;
        cursor++;
    }

    // 优化3：动态计算数值
    uint8_t decimal_pos = 0;
    uint8_t has_decimal = 0;
    data_return = 0.0f;
    
    for(uint8_t i=cursor; i<=data_End_Num; i++) 
    {
        if(fifo_get_data[i] == '.') 
        {
            has_decimal = 1;
            decimal_pos = i - cursor;
            continue;
        }
        
        // 字符转数字（更安全的写法）
        uint8_t digit = fifo_get_data[i] - '0';  // 替换原48为'0'
        if(digit > 9) return 0.0f; // 非法字符
        
        if(has_decimal)
            data_return += digit * powf(0.1, i - cursor - decimal_pos);
        else
            data_return = data_return * 10.0f + digit;
    }

    return minus_Flag ? -data_return : data_return;
   }
 
//-------------------------------------------------------------------------------------------------------------------
// 函数简介       将vofa中回传的数据解析分别赋值
// 参数说明       void
// 返回参数       void
// 使用示例      USART_PID_Adjust()
//-------------------------------------------------------------------------------------------------------------------
 
void USART_PID_Adjust()
{
    float data_Get = Get_Data(); // 存放接收到的数据
    printf("data=%.2f\r\n",data_Get);
        // if(fifo_get_data[0]=='P' && fifo_get_data[1]=='1') // 角度环P
        //     NUM.angle_p = data_Get;
        // else if(fifo_get_data[0]=='I' && fifo_get_data[1]=='1') // 角度环I
        //     NUM.angle_i = data_Get;
        // else if(fifo_get_data[0]=='D' && fifo_get_data[1]=='1') // 角度环D
        //     NUM.angle_d = data_Get;
        if(fifo_get_data[0] == 'P')
         {
            if(fifo_get_data[1] == '1') 
            NUM.angle_p = data_Get;      // 角度环P
            else if(fifo_get_data[1] == '2') 
            NUM.anglev_p = data_Get; // 角速度环P
            else if(fifo_get_data[1] == '3') 
            NUM.speed_p = data_Get; // 速度环P
        }
        else if(fifo_get_data[0] == 'I') 
        {
            if(fifo_get_data[1] == '1')
             NUM.angle_i = data_Get;      // 角度环I
            else if(fifo_get_data[1] == '2')
             NUM.anglev_i = data_Get; // 角速度环I
            else if(fifo_get_data[1] == '3')
             NUM.speed_i = data_Get; // 速度环I
        }
        else if(fifo_get_data[0] == 'D')
         {
            if(fifo_get_data[1] == '1') 
            NUM.angle_d = data_Get;      // 角度环D
            else if(fifo_get_data[1] == '2')
             NUM.anglev_d= data_Get; // 角速度环D
            else if(fifo_get_data[1] == '3')
             NUM.speed_d = data_Get; // 速度环D
        }
 }












void seekfree_send_speed_pid_wave(void)
      {
          seekfree_assistant_oscilloscope_data.channel_num = 6;

          seekfree_assistant_oscilloscope_data.data[0] = pid_speed_L.target;
          seekfree_assistant_oscilloscope_data.data[1] = (float)motor_L.encoder_speed;
          seekfree_assistant_oscilloscope_data.data[2] = pid_speed_L.out;

          seekfree_assistant_oscilloscope_data.data[3] = pid_speed_R.target;
          seekfree_assistant_oscilloscope_data.data[4] = (float)motor_R.encoder_speed;
          seekfree_assistant_oscilloscope_data.data[5] = pid_speed_R.out;

          seekfree_assistant_oscilloscope_send(&seekfree_assistant_oscilloscope_data);
      }
