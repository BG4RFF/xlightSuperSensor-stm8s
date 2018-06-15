#ifndef __TIMER_FOUR_H
#define __TIMER_FOUR_H

// 1ms interupt
void Time4_Init(void);

typedef void (*TM4_CallBack_t)();

extern TM4_CallBack_t TIM4_1ms_handler;
extern TM4_CallBack_t TIM4_5ms_handler;
extern TM4_CallBack_t TIM4_10ms_handler;

// Timer function pointers
typedef void (*app_timer_timeout_handler_t)(uint8_t);
void timer_create(u8* timer_index, u8 timer_tag, app_timer_timeout_handler_t timerout_hander);
void timer_start(u8 timer_index, u32 duration);
void timer_stop(u8 timer_index);
void tick_timeout_handler(void);

#endif // __TIMER_FOUR_H