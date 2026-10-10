/*
 * connect.h
 *
 *  Created on: 2026��6��28��
 *      Author: hp
 */

#ifndef CODE_CONNECT_H_
#define CODE_CONNECT_H_
#include "zf_common_headfile.h"


typedef enum
{
    follow_state = 0,
    turn_left_state = 3,
    turn_right_state = 4,
    turn_back_state = 5,
} driver_state;

extern driver_state Go;

//extern  diff;
int what_control();
extern int path_index;


#endif /* CODE_CONNECT_H_ */
