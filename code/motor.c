/*
 * motor.c
 *
 *  Created on: 2026锟斤拷6锟斤拷7锟斤拷
 *      Author: hp
 */
#include "image.h"
#include "motor.h"
#include "zf_common_headfile.h"
/*
 * base_speed锟斤拷motor.c锟侥碉拷251锟叫达拷锟斤拷锟叫革拷值
 * */
//ATOM3_CH4_P11_9turn.....ATOM2_CH5_P11_10pwm
#define motor_right_turn ATOM2_CH4_P11_9
#define motor_right_pwm  ATOM3_CH5_P11_10
#define motor_left_turn  ATOM3_CH6_P11_11
#define motor_left_pwm   ATOM2_CH7_P11_12
#define max_pwm 2000

float distance_L;
float distance_R;
float distance;
motor1 motor_L;
motor1 motor_R;
pid_position pos_pd;

/*************************
 * 编码器初始化
 * 函数名void Encoder_Init();
 *************************/
void Encoder_Init()
{
    encoder_dir_init(TIM4_ENCODER,TIM4_ENCODER_CH1_P02_8,TIM4_ENCODER_CH2_P00_9);
    encoder_dir_init(TIM6_ENCODER,TIM6_ENCODER_CH1_P20_3,TIM6_ENCODER_CH2_P20_0);
}

/***************************
 * 编码器数据获取
 * 函数名void Encoder_Data_Get(void)
***************************/
void Encoder_Data_Get(void)
{
    motor_L.encoder_raw=-encoder_get_count(TIM4_ENCODER);
    motor_L.encoder_speed=motor_L.encoder_speed*0.5+motor_L.encoder_raw*0.5;     //0.5锟角平均拷锟斤拷锟斤拷权锟斤拷50%
    motor_L.total_encoder+= motor_L.encoder_raw;//锟斤拷锟斤拷锟斤拷锟桔硷拷值
    encoder_clear_count(TIM4_ENCODER);

    motor_R.encoder_raw=encoder_get_count(TIM6_ENCODER);
    motor_R.encoder_speed=motor_R.encoder_speed*0.5+motor_R.encoder_raw*0.5;
    motor_R.total_encoder+= motor_R.encoder_raw;//锟斤拷锟斤拷锟斤拷锟桔硷拷值
    encoder_clear_count(TIM6_ENCODER);

    Encoder_Data_distance();
}
void Encoder_Data_distance()
{
    distance_L=motor_L.total_encoder/5000.0f;
    distance_R=motor_R.total_encoder/5000.0f;
    distance=(distance_L+distance_R)/2;
}
/**********************
 * 编码器数据初始化
 * 函数名void Encoder_Data_Get_Init()
**************************/
void Encoder_Data_Get_Init()
{
    motor_L.encoder_speed=0;
    motor_R.encoder_speed=0;

    motor_L.total_encoder = 0;
    motor_R.total_encoder = 0;

    distance_L=0;
    distance_R=0;
}
/*********************** 
 * 电机初始化
 * 函数名void motor_init()
***********************/
void motor_init()
{
    pwm_init(motor_right_turn, 17000, 0);
    pwm_init(motor_right_pwm, 17000, 0);
    pwm_init(motor_left_turn, 17000, 0);
    pwm_init(motor_left_pwm, 17000, 0);
}
/**************************
 * 电机速度设置pwm
 * 函数名void Motor_SetSpeed(int pwm1,int pwm2)
*************************/
void Motor_SetSpeed(int pwm1,int pwm2)
{
    // 锟斤拷锟斤拷
    if(pwm1>max_pwm)pwm1=max_pwm;
    if(pwm1<-max_pwm)pwm1=-max_pwm;
    if(pwm1>=0)
    {

        pwm_set_duty(motor_left_turn,0);//锟斤拷锟斤拷锟斤拷转
        pwm_set_duty(motor_left_pwm,pwm1);//锟斤拷锟斤拷PWM

    }
    else
    {
        pwm_set_duty(motor_left_turn,-pwm1);//锟斤拷锟斤拷PWM
        pwm_set_duty(motor_left_pwm,0);//锟斤拷锟斤拷锟斤拷锟斤拷
    }
    //锟揭碉拷锟�
    if(pwm2>max_pwm)pwm2=max_pwm;
    if(pwm2<-max_pwm)pwm2=-max_pwm;
    if(pwm2>=0)
    {
        pwm_set_duty(motor_right_turn,0);
        pwm_set_duty(motor_right_pwm,pwm2);

    }
    else
    {
        pwm_set_duty(motor_right_turn,-pwm2);
        pwm_set_duty(motor_right_pwm,0);
    }
}
/********************
增量式PID结构体
函数名Increase_pid_init();

*********************/
void Increase_pid_init(Increase_pid *pid,float kp,float ki,float kd,float out_min,float out_max)
{
    pid->Kp=kp;
    pid->Ki=ki;
    pid->Kd=kd;
    pid->target = 0;
    pid->error_last = 0;
    pid->error_prev = 0;
    pid->out=0;
    pid->out_max=out_max;
    pid->out_min=out_min;
}
Increase_pid pid_speed_L;
Increase_pid pid_speed_R;
//增量式PID公式u(k) = Kp*(e(k)-e(k-1)) + Ki*e(k) + Kd*(e(k)-2e(k-1)+e(k-2))
//增量式PID左轮
float Speed_pid_left(Increase_pid *pid_left,float encoder_speed_left)
{
    pid_left->error_curr=pid_left->target-encoder_speed_left; //锟斤拷锟姐当前锟斤拷锟�
    //增量式PID
    pid_left->out+=(pid_left->Kp*(pid_left->error_curr-pid_left->error_last)+
                pid_left->Ki*pid_left->error_curr+
                    pid_left->Kd*(pid_left->error_curr-2*pid_left->error_last+pid_left->error_prev));

    //输出限制
    if(pid_left->out > pid_left->out_max)
    {
        pid_left->out = pid_left->out_max;
    }
    if(pid_left->out<pid_left->out_min)
    {
        pid_left->out=pid_left->out_min;
    }
    //更新误差
    pid_left->error_prev = pid_left->error_last;   //e(k-2),e(k-1)
    pid_left->error_last = pid_left->error_curr;    //e(k-1),e(k)

    return pid_left->out;
}
//增量式PID右轮
float Speed_pid_right(Increase_pid *pid_right,float encoder_speed_right)
{
     pid_right->error_curr=pid_right->target-encoder_speed_right; //右轮误差
    //增量式PID
    pid_right->out+=(pid_right->Kp*(pid_right->error_curr-pid_right->error_last)+
                pid_right->Ki*pid_right->error_curr+
                    pid_right->Kd*(pid_right->error_curr-2*pid_right->error_last+pid_right->error_prev));
    //输出限制
    if(pid_right->out > pid_right->out_max)
    {
        pid_right->out = pid_right->out_max;

    }
    if(pid_right->out<pid_right->out_min)
    {
        pid_right->out=pid_right->out_min;
    }
    //数据更新
    pid_right->error_prev = pid_right->error_last;   //e(k-2),e(k-1)
    pid_right->error_last = pid_right->error_curr;    //e(k-1),e(k)

    return pid_right->out;
}
/*****************************
 * 启动补偿
 * 函数名：static int Motor_Start_Compensate(float pid_out, float target, int encoder_speed, int min_pwm);
 *******************************/
static int Motor_Start_Compensate(float pid_out, float target, int encoder_speed, int min_pwm)
{
    int pwm = (int)pid_out;

    if(target == 0)
    {
        return 0;
    }
    // 只锟斤拷锟斤拷锟接硷拷锟斤拷没锟斤拷时锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟襟交革拷PID
    if(encoder_speed > -5 && encoder_speed < 5)
    {
        if(target > 0 && pwm < min_pwm)
        {
            pwm = min_pwm;
        }
        else if(target < 0 && pwm > -min_pwm)
        {
            pwm = -min_pwm;
        }
    }
    return pwm;
}
/*****************************************
 * 位置PD
 * 增量式PD u(k)=Kp*e(k)+Kd*(e(k)-e(k-1))
 * 函数名float PositionPID_calculate(pid_position*pid,int32 target_position,int32 actual_position,int32 actual_speed);
 * 各个参数说明：结构体数据，目标位置，实际位置，实际速度
 ***************************************/
float PositionPID_calculate(pid_position*pid,int32 target_position,int32 actual_position,int32 actual_speed)
{
    int32 error_position=target_position-actual_position;         //位置误差
    float P = pid->Kp*error_position;      //偏锟斤拷越锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟皆斤拷锟�
    float D = pid->Kd*(-actual_speed);    
    float output=P + D;            //原始锟斤拷锟斤拷锟斤拷
    //输出限制
    if(output>pid->output_limit)
    {
        output=pid->output_limit;
    }
    if(output<-pid->output_limit)
    {
        output=-pid->output_limit;
    }
    return output;
}
/******************************
 * 位置PD初始化
 * 函数名void PositionPid_Init(void);
 ********************************/
pid_position line_pid;
float line_error_last = 0;
void PositionPid_Init(void)
{
    line_pid.Kp = 1.0f;
    line_pid.Kd = 1.0f;
    line_pid.output_limit = 30.0f;
}
/******************************
 * PID进行赋值
 * 调节速度PID
 * 函数名void Pid_Init(void);
 ******************************/
void Pid_Init(void)
{
    Increase_pid_init(&pid_speed_L,5.0f,0.05f,0.0f,-1200,1200);//锟结构锟藉，kp,ki,kd,锟斤拷锟斤拷锟叫★拷锟斤拷锟斤拷锟斤拷锟�
    Increase_pid_init(&pid_speed_R,6.0f,0.008f,0.0f,-1500,1500);
    PositionPid_Init();
    //pid_speed_L.target = 70.0f;
    //pid_speed_R.target = 70.0f;
}
/*******************************
 * 综合控制
 * 函数名void line_control(void);
 *******************************/
void line_control(void)
{
    state_machine();
    float base_speed = 70.0f;//基础速度
    float line_error = Deviation();
    //int a=what_control();
    if(Go == turn_left_state || Go == turn_right_state)
       {
           line_error_last = 0.0f;      // 重置，出弯不会跳变
           return;
       }
    if(line_error>-2.0f && line_error<2.0f)
    {
        line_error=0.0f;
        line_error_last = 0.0f;

        pid_speed_L.target = base_speed;
        pid_speed_R.target = base_speed;
        return;
    }
    int32 actual_position = (int32)line_error;
    int32 actual_speed = (int32)(line_error - line_error_last);

    float turn_out = PositionPID_calculate(&line_pid, 0, actual_position, actual_speed);

    line_error_last = line_error;

    pid_speed_L.target = base_speed - turn_out; 
    pid_speed_R.target = base_speed + turn_out;
}
/******************************
 *启动补偿
 * 函数名void circle_control();
 ********************************/
void circle_control()
{
    float out_L = Speed_pid_left(&pid_speed_L,motor_L.encoder_speed);
    float out_R = Speed_pid_right(&pid_speed_R,motor_R.encoder_speed);
    int pwm_L = Motor_Start_Compensate(out_L, pid_speed_L.target, motor_L.encoder_speed, 800);
    int pwm_R = Motor_Start_Compensate(out_R, pid_speed_R.target, motor_R.encoder_speed, 850);
    Motor_SetSpeed((int)out_L, pwm_R);
}


