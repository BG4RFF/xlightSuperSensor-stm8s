#include <stm8s.h>
#include "timer_4.h"
#include "_global.h"

#define MAX_TIMER_NUMBER    8

uint8_t TIM4_Timer10ms = 0;

TM4_CallBack_t TIM4_1ms_handler = NULL;
TM4_CallBack_t TIM4_5ms_handler = NULL;
TM4_CallBack_t TIM4_10ms_handler = NULL;

typedef struct timer_manager_s
{
    u8   timer_index;
    u8   timer_tag;
    bool timer_started;
    s32  timer_left;
    app_timer_timeout_handler_t handler;
} timer_manager_t;

static timer_manager_t m_timer_manager[MAX_TIMER_NUMBER] = {0};
static u8 m_current_timer_num = 0;
static u8 m_running_timer_num = 0;
static s32 m_wait_timer_tick = 0;
static s32 system_past_tick = 0;
u16 tmrIdleDuration = 0;

void Time4_Init(void) {
  // Interval = 8us*125 = 1ms
  TIM4_TimeBaseInit(TIM4_PRESCALER_128, 124);
  TIM4_PrescalerConfig(TIM4_PRESCALER_128, TIM4_PSCRELOADMODE_IMMEDIATE);
  TIM4_ARRPreloadConfig(ENABLE);
  TIM4_ClearFlag(TIM4_FLAG_UPDATE);
  TIM4_ITConfig(TIM4_IT_UPDATE, ENABLE);
  TIM4_Cmd(ENABLE);
}

void timer_create(u8 *timer_index, u8 timer_tag, app_timer_timeout_handler_t timerout_hander)
{
  if (m_current_timer_num >= MAX_TIMER_NUMBER)
  {
    // No space for the new timer.
    return;
  }
  else
  {
    *timer_index = m_current_timer_num;
    m_timer_manager[m_current_timer_num].timer_index = m_current_timer_num;
    m_timer_manager[m_current_timer_num].timer_tag = timer_tag;
    m_timer_manager[m_current_timer_num].timer_started = FALSE;
    m_timer_manager[m_current_timer_num].handler = timerout_hander;
    m_current_timer_num ++;
  }
}

void timer_schedule(bool is_new)
{
  u8 index = 0;
  m_wait_timer_tick = 0x7FFFFFFF;
  m_running_timer_num = 0;
  
  for (index = 0; index < m_current_timer_num; index ++)
  {
    if (m_timer_manager[index].timer_started == TRUE)
    {
      if (is_new == FALSE)
      {
        m_timer_manager[index].timer_left -= system_past_tick;
      }
      if (m_timer_manager[index].timer_left > 0)
      {
        if (m_wait_timer_tick >= m_timer_manager[index].timer_left)
        {
          m_running_timer_num ++;
          m_wait_timer_tick = m_timer_manager[index].timer_left;
        }
      }
      else
      {
        m_timer_manager[index].timer_left = 0;
        m_timer_manager[index].timer_started = FALSE;
        m_timer_manager[index].handler(m_timer_manager[index].timer_tag);
      }
    }
  }
  system_past_tick = 0;
  if (m_wait_timer_tick == 0xFFFFFFFF)
  {
    m_wait_timer_tick = 0;
  }
}


void timer_start(u8 timer_index, u32 duration)
{
  m_timer_manager[timer_index].timer_started = TRUE;
  m_timer_manager[timer_index].timer_left = duration;
  
  timer_schedule(TRUE);
}

void timer_stop(u8 timer_index)
{
  m_timer_manager[timer_index].timer_started = FALSE;
}

void tick_timeout_handler(void)
{
  if (m_running_timer_num > 0)
  {
    system_past_tick ++;
    if (system_past_tick >= m_wait_timer_tick)
    {
      timer_schedule(FALSE);
    }
  }
}

INTERRUPT_HANDLER(TIM4_UPD_OVF_IRQHandler, 23)
{ 
#ifdef TEST
  PB5_High;
#endif
  // Timer callbacks
  if( TIM4_1ms_handler ) (*TIM4_1ms_handler)();
  
  if(TIM4_Timer10ms > 0) {
    TIM4_Timer10ms--;
    if( TIM4_Timer10ms % 5 == 2 ) {
      if( TIM4_5ms_handler ) (*TIM4_5ms_handler)();
    }
  } else {
    // Reset and go
    TIM4_Timer10ms = 10;
    if( TIM4_10ms_handler ) (*TIM4_10ms_handler)();

#ifdef EN_PANEL_BUTTONS
    tick_timeout_handler();
#endif    
  }

  TIM4_ClearITPendingBit(TIM4_IT_UPDATE);
#ifdef TEST
  PB5_Low;
#endif
}
