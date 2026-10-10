/*
 * connect.c
 *
 *  Created on: 2026年6月28日
 *      Author: hp
 */


#include "image.h"
#include "motor.h"
#include "zf_common_headfile.h"



#define BASE_SPEED          70.0f

#define TURN_MIN_TICK       20      // 10ms执行一次
#define TURN_MAX_TICK       110     // 1100ms后停止转向
#define TURN_EXIT_ERROR     6.0f


Driver_state Go = follow_state;
static uint16 turn_tick = 0;

路径表
*/
/*
typedef enum
{
    T_road,
    cross_road,
}road_state;
road_state T_road;
void path_table()
{
    T_road=follow_state;
    if(distance>25&&distance<35)
    {
        Go=T_road;
    }
    
}
uint8 state_machine(void)
{
    int order = judge_where();
    float error = Deviation();

    switch(Go)
    {
        case follow_state:
        {
            turn_tick = 0;
            line_pid.Kp = 1.0f;
            line_pid.Kd = 1.0f;
            line_pid.output_limit = 30.0f;
            if(order == 3)
            {
                Go = turn_left_state;
                return 1;
            }

            if(order == 4)
            {
                Go = turn_right_state;
                return 1;
            }

            return 0;
        }

        case turn_left_state:
        {
            turn_tick++;

           
            float diff=0.7f*fabsf(weighted_error);//通过偏差调节左右轮转速差
            pid_speed_L.target=BASE_SPEED-diff;
            pid_speed_R.target=BASE_SPEED+diff;

            if(turn_tick > TURN_MIN_TICK)
            {
                if((error > -TURN_EXIT_ERROR && error < TURN_EXIT_ERROR) || turn_tick > TURN_MAX_TICK)
                {
                    Go = follow_state;
                    turn_tick = 0;
                }
            }

            return 1;
        }

        case turn_right_state:
        {
            turn_tick++;

            
               float diff=1.0f * fabsf(weighted_error);//通过偏差调节左右轮转速差

               pid_speed_L.target = BASE_SPEED + diff;
               pid_speed_R.target = BASE_SPEED - diff;

            if(turn_tick > TURN_MIN_TICK)
            {
                if((error > -TURN_EXIT_ERROR && error < TURN_EXIT_ERROR) || turn_tick > TURN_MAX_TICK)
                {
                    Go = follow_state;
                    turn_tick = 0;
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
*/

/********************
路径表
*******************/
typedef struct 
{
    int road_type;
    driver_state state;
    float distance_min;
    float distance_max;
}road_table;
  
const road_table route_table[] =
{
    {4, turn_right_state, 0.0f},
    {4, turn_right_state, 28.0f,35.0f},
    {3, turn_left_state, 0.0f},
    {4, turn_right_state, 0.0f},
};
#define ROUTE_LEN  (sizeof(route_table) / sizeof(route_table[0]))

static uint16 turn_tick = 0;
//有distance计算距离
void state_machine()
{
   int order = judge_where();
   float error = Deviation();
   case follow_state:
   {
       turn_tick = 0;
       line_pid.Kp = 1.0f;
       line_pid.Kd = 1.0f;
       line_pid.output_limit = 30.0f;
       if(order == 3)
       {
           if((distance>28&&distance<35)||(distance>48&&distance<52))
           {
                if()
               Go = turn_right_state;
               return 1;
           }
       }
       if(order==4)
       {
            Go = turn_right_state;
            return 1;
       }
       return 0;
    }
    case turn_right_state:
    {
        turn_tick++;
        float diff=1.0f * fabsf(weighted_error);
        pid_speed_L.target = BASE_SPEED + diff;
        pid_speed_R.target = BASE_SPEED - diff;
          if(turn_tick > TURN_MIN_TICK)
            {
                if((error > -TURN_EXIT_ERROR && error < TURN_EXIT_ERROR) || turn_tick > TURN_MAX_TICK)
                {
                    Go = follow_state;
                    turn_tick = 0;
                }
            }
        return 1;
    }
    case turn_left_state:
    {
        turn_tick++;
        float diff=0.7f*fabsf(weighted_error);
        pid_speed_L.target=BASE_SPEED-diff;
        pid_speed_R.target=BASE_SPEED+diff;
        if(turn_tick > TURN_MIN_TICK)
            {
                if((error > -TURN_EXIT_ERROR && error < TURN_EXIT_ERROR) || turn_tick > TURN_MAX_TICK)
                {
                    Go = follow_state;
                    turn_tick = 0;
                }
            }

        return 1;
    }
}
