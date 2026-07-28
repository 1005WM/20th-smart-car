/*
 * buzzer.c
 *
 *  Created on: 2026年7月5日
 *      Author: hp
 */

#include "buzzer.h"

static uint16 buzz_timer = 0;

void buzzer_init(void)
{
    gpio_init(BUZZ_PIN, GPO, 0, GPO_PUSH_PULL);
}

void buzzer_on(void)  { gpio_high(BUZZ_PIN); }
void buzzer_off(void) { gpio_low(BUZZ_PIN);  }

void buzzer_beep(uint16 ticks)
{
    buzzer_on();
    buzz_timer = ticks;     // 比如 30 → 响 300ms
}

void buzzer_tick(void)
{
    if (buzz_timer > 0)
    {
        buzz_timer--;
        if (buzz_timer == 0)
            buzzer_off();
    }
}

