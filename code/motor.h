/*
 * motor.h
 *
 *  Created on: 2026年6月7日
 *      Author: hp
 */

#ifndef CODE_MOTOR_H_
#define CODE_MOTOR_H_

#include "zf_common_headfile.h"
typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float target;  //目标值
    float error_curr,error_last,error_prev;//误差项（当前、上次、上上次）
    float out;//输出
    float out_max;//输出上限
    float out_min;//输出下限

}Increase_pid;

typedef struct motor1
{
        int target_speed;     //目标速度
        int duty;               //pwm占空比
        int encoder_speed;         //编码器速度
        int encoder_raw;          //编码器原始计数值
        int32 total_encoder;           //编码器累计计数
}motor1;


typedef struct pid_position
{
     float Kp;      //比例系数（位置偏差越大输出越大）
     float Kd;       //微分系数（偏差变化速度越大输出越大）
     float output_limit;        //输出限幅
}pid_position;


void motor_init();
void Encoder_Data_Get(void);
void Encoder_Data_Get_Init();
void Motor_SetSpeed(int pwm1,int pwm2);
void Increase_pid_init(Increase_pid *pid,float kp,float ki,float kd,float out_min,float out_max);
float Speed_pid_left(Increase_pid *pid_left,float encoder_speed_left);
float Speed_pid_right(Increase_pid *pid_right,float encoder_speed_right);
void Encoder_Init();
void Pid_Init();
void circle_control();
void line_control(void);
void Encoder_Data_distance();

extern Increase_pid pid_speed_L;
extern Increase_pid pid_speed_R;
extern motor1 motor_L;
extern motor1 motor_R;
extern pid_position pos_pd;
extern pid_position line_pid;

extern float distance_L;
extern float distance_R;
extern float distance;

extern float base_speed;
extern volatile uint8 image_busy;
extern volatile uint8 image_ready;

#endif /* CODE_MOTOR_H_ */
