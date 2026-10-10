/*
 * motor.h
 *
 *  Created on: 2026锟斤拷6锟斤拷7锟斤拷
 *      Author: hp
 */

#ifndef CODE_MOTOR_H_
#define CODE_MOTOR_H_

#include "zf_common_headfile.h"
typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float target;  //目锟斤拷值
    float error_curr,error_last,error_prev;//锟斤拷锟斤拷锟斤拷锟�
    float out;//锟斤拷锟街�
    float out_max;//锟斤拷锟斤拷锟斤拷锟�
    float out_min;//锟斤拷锟斤拷锟斤拷锟�

}Increase_pid;

typedef struct motor1
{
        int target_speed;     //目锟斤拷锟劫讹拷
        int duty;               //pwm占锟秸憋拷
        int encoder_speed;         //锟斤拷锟斤拷锟斤拷锟劫讹拷
        int encoder_raw;          //锟斤拷锟斤拷锟节憋拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷
        int32 total_encoder;           //锟斤拷锟斤拷锟�
}motor1;


typedef struct pid_position
{
     float Kp;      //锟斤拷锟斤拷系锟斤拷锟斤拷锟斤拷锟斤拷位锟斤拷偏锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟�
     float Kd;       //微锟斤拷系锟斤拷锟斤拷锟斤拷锟斤拷锟劫讹拷锟斤拷锟斤拷
     float output_limit;        //锟斤拷锟斤拷薹锟�
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
