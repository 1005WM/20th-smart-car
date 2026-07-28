/*
 * motor.h
 *
 *  Created on: 2026��6��7��
 *      Author: hp
 */

#ifndef CODE_MOTOR_H_
#define CODE_MOTOR_H_

#include "zf_common_headfile.h"
typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float target;  //Ŀ��ֵ
    float error_curr,error_last,error_prev;//�������
    float out;//���ֵ
    float out_max;//�������
    float out_min;//�������

}Increase_pid;

typedef struct motor1
{
        int target_speed;     //Ŀ���ٶ�
        int duty;               //pwmռ�ձ�
        int encoder_speed;         //�������ٶ�
        int encoder_raw;          //�����ڱ�������������
        int32 total_encoder;           //�����
}motor1;


typedef struct pid_position
{
     float Kp;      //����ϵ��������λ��ƫ�������������
     float Kd;       //΢��ϵ���������ٶ�����
     float output_limit;        //����޷�
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


#endif /* CODE_MOTOR_H_ */
