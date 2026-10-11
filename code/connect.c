/*
 * connect.c
 *
 *  Created on: 2026年6月28日
 *      Author: hp
 */

#include "image.h"
#include "motor.h"
#include "zf_common_headfile.h"


#define BASE_SPEED          300.0f

#define TURN_MIN_TICK       12
#define TURN_MAX_TICK       110
#define TURN_EXIT_ERROR     6.0f
//40
#define cooldown_total     4
uint8 cooldown=0;

float gyro_angle_start = 0;
driver_state Go = follow_state;

float turn_weighted_error = 0;
static uint16 turn_tick = 0;

typedef enum {
    follow_stright_road,
    turn_left_road,
    turn_right_road,
    turn_shizi_road,
    turn_sanjiguan_road,
}judge_go;

typedef enum {
    actoin_go,      // 直行
    action_left,    // 左转
    action_right,   // 右转
    action_U_road
} action_go;

typedef struct{
    judge_go judge;
    action_go action;
    float distance;
}path_node_t;


//正方向路径规划（新方法）
path_node_t path[30]=
{
    {turn_right_road,action_right,0.0f},//0
    {turn_right_road,action_right,0.0f},//1
    {turn_left_road,action_left,0.0f},//2
    {turn_left_road,action_left,0.0f},//3
    {turn_right_road,action_right,0.0f},//4
    {turn_right_road,actoin_go,0.0f},//5
    {turn_shizi_road,action_left,0.0f},//6
    {turn_left_road,action_left,0.0f},//7
    {turn_left_road,action_left,0.0f},//8
    {turn_left_road,action_left,0.0f},//9
    {turn_right_road,action_right,0.0f},//10
    {turn_left_road,action_left,0.0f},//11
    {turn_shizi_road,action_right,0.0f},//12
    {turn_right_road,action_right,0.0f},//13
    {turn_right_road,actoin_go,0.0f},//14
    {turn_shizi_road,action_left,0.0f},//15
    {turn_shizi_road,action_left,0.0f},//16
    {turn_left_road,actoin_go,0.0f},//17
    {turn_left_road,action_left,0.0f},//18
    {turn_right_road,action_right,0.0f},//19
    {turn_shizi_road,action_right,0.0f},//20
    {turn_right_road,action_right,0.0f},//21
    {turn_right_road,actoin_go,0.0f},//22
    {turn_right_road,actoin_go,0.0f},//23
    {turn_right_road,actoin_go,0.0f},//24
    {turn_right_road,action_right,0.0f},//25
};//第一个为识别路口，第二个为怎么走，第三个为距离
uint8 path_judge_match(int order, judge_go judge)
{
    switch(judge)
    {
        case follow_stright_road:
            return order == 0;

        case turn_left_road:
            return order == 3;

        case turn_right_road:
            return order == 4;

        case turn_shizi_road:
            return order == 9;
        case turn_sanjiguan_road:
            return order == 8;

        default:
            return 0;
    }
}
int path_index =0;
uint8  path_driver()
{
    if (cooldown > 0)
    {
        cooldown--;
        pid_speed_L.target = BASE_SPEED;
        pid_speed_R.target = BASE_SPEED;
        return 0;
    }

    if(path_index>30) return 0;
    int order =road_order_cache;
    path_node_t *now = &path[path_index];
    switch (Go)
    {
    case follow_state:
        {
            turn_tick = 0;
            line_pid.Kp = 1.1f;
            line_pid.Kd = 1.2f;
            line_pid.output_limit = 50.0f;//182-186
            if(!path_judge_match(order, now->judge))
            {
                return 0;
            }
            if (now->action == action_left&&distance>=30)
            {
                turn_weighted_error =  road_weighted_error_cache;
                gyro_angle_start = T_N;
                Go = turn_left_state;
                return 1;
            }
            if (now->action == action_right)
            {
                turn_weighted_error =  road_weighted_error_cache;
                gyro_angle_start = T_N;
                Go = turn_right_state;
                return 1;
            }

            if(now->action == actoin_go)
             {
                pid_speed_L.target = 300.0f;
                pid_speed_R.target = 300.0f;

                cooldown = 15;
                path_index++;
                return 0;
             }
                return 0;
            }
    case turn_left_state:
        {
            turn_tick++;
            if(turn_weighted_error>80.0f||turn_weighted_error<-80.0f)
             {
                turn_weighted_error=80.0f;
             }
            if(path_index==3||path_index==8)
            {
                float diff=1.9f*fabsf(turn_weighted_error);
                pid_speed_L.target=BASE_SPEED-diff;
                pid_speed_R.target=BASE_SPEED+diff;
            }
            else if(path_index==6||path_index==7)
            {

                float diff=2.5f*fabsf(turn_weighted_error);
                pid_speed_L.target=BASE_SPEED-diff;
                pid_speed_R.target=BASE_SPEED+diff;
            }
            else if(path_index==11)
            {
                float diff=2.4f*fabsf(turn_weighted_error);
                pid_speed_L.target=BASE_SPEED-diff;
                pid_speed_R.target=BASE_SPEED+diff;
            }
            else
            {
                float diff=2.1f*fabsf(turn_weighted_error);

                pid_speed_L.target=BASE_SPEED-diff;
                pid_speed_R.target=BASE_SPEED+diff;
            }
            if(turn_tick>=TURN_MIN_TICK)
            {
                if(fabsf(T_N - gyro_angle_start) >= 85.0f)
                {
                    pid_speed_L.target = BASE_SPEED;
                    pid_speed_R.target = BASE_SPEED;
                    Go = follow_state;
                    turn_tick = 0;
                    cooldown=cooldown_total;
                    if(path_index + 1 < 28)
                      {
                           path_node_t *next = &path[path_index + 1];
                           if(now->action == action_right && next->action == action_left)
                                   cooldown = 1;
                           if(now->judge==turn_left_road&&next->judge == turn_shizi_road)
                                   cooldown=1;
                      }
                    path_index++;
                }

            }
            return 1;
        }
        case turn_right_state:
        {
        turn_tick++;
        if(turn_weighted_error>80.0f||turn_weighted_error<-80.0f)
        {
            turn_weighted_error=80.0f;
        }
        if(path_index==13)
         {

             float diff=2.25f*fabsf(turn_weighted_error);

             pid_speed_L.target = BASE_SPEED + diff;
             pid_speed_R.target = BASE_SPEED - diff;
          }
        else
        {

            float diff=2.42f * fabsf(turn_weighted_error);

            pid_speed_L.target = BASE_SPEED + diff;
            pid_speed_R.target = BASE_SPEED - diff;
        }
        if(turn_tick>=TURN_MIN_TICK)
        {
            if(fabsf(T_N - gyro_angle_start) >= 85.0f)
            {
                pid_speed_L.target = BASE_SPEED;
                pid_speed_R.target = BASE_SPEED;
                Go = follow_state;
                turn_tick = 0;
                cooldown=cooldown_total;
                if(path_index + 1 < 28)
                {
                    path_node_t *next = &path[path_index + 1];
                    if(now->action == action_right && next->action == action_left)
                        cooldown = 2;

                }
                path_index++;
            }        
        }
        return 1;
        }
        default:
        {
            Go = follow_state;
            turn_tick = 0;
            return 0;
        }

    }

}