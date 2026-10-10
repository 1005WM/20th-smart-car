#ifndef CODE_VOFA_H_
#define CODE_VOFA_H_


#include "zf_common_headfile.h"


void Init_uart(void);
void Send_test(void);
void uart_rx_interrupt_handler (void);
float Get_Data(void);
void USART_PID_Adjust();

typedef struct
{
   float angle_p;
   float angle_i;
   float angle_d;
   float speed_p;
   float speed_i;
   float speed_d;
   float anglev_p;
   float anglev_i;
   float anglev_d;
}PIDNUM;

extern PIDNUM NUM;
extern uint8 uart_get_data[64];                                                        // 串口接收数据缓冲区
extern uint8 fifo_get_data[64];                                                        // fifo 输出读出缓冲区
 
extern uint8  get_data;                                                      // 接收数据变量
extern uint32 fifo_data_count;
// fifo 数据个数
void seekfree_send_speed_pid_wave();
extern fifo_struct uart_data_fifo;

#define UART_INDEX              (DEBUG_UART_INDEX   )                           // 默认 UART_0
#define UART_BAUDRATE           (DEBUG_UART_BAUDRATE)                           // 默认 115200
#define UART_TX_PIN             (DEBUG_UART_TX_PIN  )                           // 默认 UART0_TX_P14_0
#define UART_RX_PIN             (DEBUG_UART_RX_PIN  )

#endif
