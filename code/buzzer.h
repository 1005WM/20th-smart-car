/*
 * buzzer.h
 *
 *  Created on: 2026年7月5日
 *      Author: hp
 */

#ifndef CODE_BUZZER_H_
#define CODE_BUZZER_H_


#include "zf_common_headfile.h"

#define BUZZ_PIN   P33_10

void buzzer_init(void);
void buzzer_on(void);
void buzzer_off(void);
void buzzer_beep(uint16 ticks);    // ticks 以 10ms 为单位
void buzzer_tick(void);


#endif /* CODE_BUZZER_H_ */
