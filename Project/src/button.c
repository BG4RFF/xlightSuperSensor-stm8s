/*
 xlight SmartPanel button functions

- Functuon keys:
      PD7 -> KeyCenter
      PB3 -> Fn1
      PB4 -> Fn2
      PB5 -> Fn3

LEDs

*/

#include <stm8s.h>
#include "_global.h"
#include "timer_4.h"
#include "button.h"
#include "relay_key.h"
#include "xliNodeConfig.h"
#include "MyMessage.h"
#include "ProtocolParser.h"
#include "relay_key.h"
//---------------------------------------------------
// PIN Map
//---------------------------------------------------
// LED pin map

// Button pin map
#define BUTTONS_PORT           (GPIOB)
#define BUTTON_PIN_CENTER      (GPIO_PIN_5)
#define BUTTON_PIN_UP          (GPIO_PIN_4)
#define BUTTON_PIN_DOWN        (GPIO_PIN_3)

//---------------------------------------------------

// Set LED pin status
//#define ledSetPin(x, pin)       GPIO_WriteBit(LEDS_PORT, pin, ((x) > 0 ? SET : RESET))

// Get Button pin input
#define pinKeyCenter            ((BitStatus)(BUTTONS_PORT1->IDR & (uint8_t)BUTTON_PIN_CENTER))

#define BUTTON_DEBONCE_DURATION                 3       // The unit is 10 ms, so the duration is 30 ms.
#define BUTTON_WAIT_2S                          50      // The unit is 10 ms, so the duration is 500 ms.
#define BUTTON_WAIT_3S                          300     // The unit is 10 ms, so the duration is 3 s.
#define BUTTON_DOUBLE_BTN_DURATION              50      // The unit is 10 ms, so the duration is 500 ms.
#define BUTTON_DOUBLE_BTN_TRACK_DURATION        300     // The unit is 10 ms, so the duration is 3 s.

static button_timer_status_t  m_btn_timer_status[keylstDummy] = {BUTTON_STATUS_INIT};
static bool detect_double_btn_press[keylstDummy] = {FALSE};
static bool btn_is_pushed[keylstDummy] = {FALSE};
static uint8_t btn_bit_postion[keylstDummy];

static uint8_t m_timer_id_btn_detet[keylstDummy];
static uint8_t m_timer_id_double_btn_detet[keylstDummy];

static bool double_button_track = FALSE;
static uint8_t button_status = 0xFF;
static uint8_t button_first_detect_status = 0xFF;

static uint8_t m_timer_id_debonce_detet;

//////////// zhangqiaoli add for loop relay ////////////////////////
#define BUTTON_TIMEOUT            500          // The unit is 10 ms, so the duration is 5s.
uint8_t last_relay_key_index = 0;
uint16_t relay_loop_tick = 0;
uint8_t last_btn = 0;
uint8_t last_op = 0;
//////////// zhangqiaoli add for loop relay ////////////////////////

void app_button_event_handler(uint8_t _btn, button_event_t button_event);
void button_push(uint8_t _btn);
void button_release(uint8_t _btn);

static void btn_duration_timeout_handler(uint8_t _btn)
{
  // Assert button
  if( !IS_VALID_BUTTON(_btn) ) return;
  
  button_event_t button_event = BUTTON_INVALID;
  switch (m_btn_timer_status[_btn]) {
  case BUTTON_STATUS_INIT:
    break;
    
  case BUTTON_STATUS_LESS_2S:
    button_event = BUTTON_LONG_HOLD;
    timer_start(m_timer_id_btn_detet[_btn], BUTTON_WAIT_3S);
    m_btn_timer_status[_btn] = BUTTON_STATUS_MORE_2S;
    break;
    
  case BUTTON_STATUS_MORE_2S:
    button_event = BUTTON_VERY_LONG_HOLD;
    m_btn_timer_status[_btn] = BUTTON_STATUS_MORE_5S;
    break;
    
  case BUTTON_STATUS_MORE_5S:
    break;
    
  case BUTTON_STATUS_DOUBLE_TRACK:
    button_event = DOUBLE_BTN_TRACK;
    m_btn_timer_status[_btn] = BUTTON_STATUS_INIT;
    break;
    
  default:
    break;
  }
  
  if( button_event != BUTTON_INVALID ) {
    app_button_event_handler(_btn, button_event);
  }
}

void double_btn_timeout_handler(uint8_t _btn)
{
  // Assert button
  if( !IS_VALID_BUTTON(_btn) ) return;
  
  button_event_t button_event = BUTTON_SHORT_PRESS;
  detect_double_btn_press[_btn] = FALSE;
  m_btn_timer_status[_btn] = BUTTON_STATUS_INIT;
  timer_stop(m_timer_id_double_btn_detet[_btn]);
  app_button_event_handler(_btn, button_event);
}

void btn_debonce_timeout_handler(uint8_t _tag)
{
  uint8_t valid_button;
  uint8_t current_button;
  uint8_t changed_button;
  
  current_button = GPIO_ReadInputData(BUTTONS_PORT);

  valid_button = ~(current_button ^ button_first_detect_status);    
  changed_button = ((current_button^button_status) & valid_button);
  button_status = current_button;
  
  // Scan all buttons
  uint8_t _btn;
  for( _btn = 0; _btn < keylstDummy; _btn++ ) {    
    if ((changed_button & btn_bit_postion[_btn]) != 0)
    {
      timer_stop(m_timer_id_btn_detet[_btn]);
      if ((current_button & btn_bit_postion[_btn]) == 0)
      {
        button_push(_btn);
      }
      else
      {
        button_release(_btn);
      }
    }
  }
}

void button_init()
{
  uint8_t _btn;
  
  // Set button bit postion
  btn_bit_postion[keylstCenter] = BUTTON_PIN_CENTER;
  btn_bit_postion[keylstUp] = BUTTON_PIN_UP;
  btn_bit_postion[keylstDown] = BUTTON_PIN_DOWN;
  
  // Setup Interrupts
  disableInterrupts();
  GPIO_Init(BUTTONS_PORT, BUTTON_PIN_CENTER | BUTTON_PIN_UP | BUTTON_PIN_DOWN, GPIO_MODE_IN_PU_IT);
  EXTI_DeInit();
  EXTI_SetExtIntSensitivity(EXTI_PORT_GPIOB, EXTI_SENSITIVITY_RISE_FALL);
  enableInterrupts();

  // Create all timers
  for( _btn = 0; _btn < keylstDummy; _btn++ ) {
    timer_create(&m_timer_id_btn_detet[_btn], _btn, btn_duration_timeout_handler);
    timer_create(&m_timer_id_double_btn_detet[_btn], _btn, double_btn_timeout_handler);
  }
  timer_create(&m_timer_id_debonce_detet, 0, btn_debonce_timeout_handler);
  
  //////////// zhangqiaoli add for loop relay ////////////////////////
  last_relay_key_index = 0;
  relay_loop_tick = 0;
  last_btn = 0;
  last_op = 0;
//////////// zhangqiaoli add for loop relay ////////////////////////
  
}

uint8_t GetNextIndex(uint8_t relay_key_map,uint8_t index)
{ // key_map_index
  // 0 - all
  // n - bit n relay
  uint8_t nextindex = 0;
  for( uint8_t idx = index; idx < 8; idx++ ) {
    // Check key map
    if( BF_GET(relay_key_map, idx, 1) ) {      
      if(IsValidRelaykey(idx))
      {
        nextindex = idx+1;
        break;
      }
    }
  }
  return nextindex;
}

bool ToggleAll(uint8_t relay_key_map)
{ // Control a group relay switch based on the first switch status
  uint8_t lv_key;
  bool lv_onoff;
  bool onoff_beset = FALSE;
  for( uint8_t idx = 0; idx < 8; idx++ ) {
    // Check key map
    if( BF_GET(relay_key_map, idx, 1) ) {
      lv_key = idx + '1';
      if(!onoff_beset)
      {
         onoff_beset = TRUE;
         lv_onoff = !relay_get_key(lv_key);         
      }  
      if( relay_set_key(lv_key, lv_onoff) ) {
        Msg_Relay_Ack(NODEID_GATEWAY, lv_onoff ? V_RELAY_ON : V_RELAY_OFF, lv_key);
        //SendMyMessage();
      }
    }
  }
  return !lv_onoff;
}

bool ToggleLoop(uint8_t relay_key_map,uint8_t index)
{
   // index range(0-8)  0-all n-on bit n
    // only can switch when device state is off£»(timeout and device state is off ,switch to next)
    bool bMoveKey = FALSE;
    if( index == 0 ) 
    {
        bMoveKey = ToggleAll(relay_key_map);
        if( !bMoveKey ) 
        { //state on,start timer
          relay_loop_tick = 0;
        }
    } 
    else
    {
        uint8_t lv_key = index + '0';
        bool lv_onoff = !relay_get_key(lv_key);
        if( relay_set_key(lv_key, lv_onoff) ) {
          Msg_Relay_Ack(NODEID_GATEWAY, lv_onoff ? V_RELAY_ON : V_RELAY_OFF, lv_key);
          //SendMyMessage();
        }
        if(lv_onoff)
        {// Off -> On, stay at current relay key
          //state on,start timer
          relay_loop_tick = 0;
        }
        else
        {// On -> Off
          bMoveKey = TRUE;
        }      
    }
    // Move to next avalaible relay key
    if( bMoveKey &&  relay_loop_tick <= BUTTON_TIMEOUT) {
        last_relay_key_index = GetNextIndex(relay_key_map,index);
    }

    return bMoveKey;
}

void LoopAll(uint8_t relay_key_map,uint8_t lv_act)
{
  if(lv_act == BTN_ACT_TOGGLE)
  {
    ToggleLoop(relay_key_map,0);  
  }
  else
  {
    relay_loop_tick = 0;
    uint8_t lv_key;
    bool lv_onoff;
    for( uint8_t idx = 0; idx < 8; idx++ ) {
      // Check key map
      if( BF_GET(relay_key_map, idx, 1) ) {
        lv_key = idx + '1';
        lv_onoff = (BTN_ACT_ON == lv_act);
        if( relay_set_key(lv_key, lv_onoff) ) {
          Msg_Relay_Ack(NODEID_GATEWAY, lv_onoff ? V_RELAY_ON : V_RELAY_OFF, lv_key);
          //SendMyMessage();
        }
      }
    }
  }
}

void LoopOne(uint8_t relay_key_map,uint8_t lv_act,uint8_t index)
{
  // index range(0-8)  0-all n-on bit n
  if(lv_act == BTN_ACT_TOGGLE)
  { // toggle
    ToggleLoop(relay_key_map,index);
  }
  else
  { 
    uint8_t lv_key;
    bool lv_onoff = (BTN_ACT_ON == lv_act);
    relay_loop_tick = 0;
    for( uint8_t idx = 0; idx < 8; idx++ ) {
      // Check key map
      if( BF_GET(relay_key_map, idx, 1) ) {
        if( idx+1 == index)
        { 
          lv_key = index + '0';        
          if( relay_set_key(lv_key, lv_onoff) ) {
            Msg_Relay_Ack(NODEID_GATEWAY, lv_onoff ? V_RELAY_ON : V_RELAY_OFF, lv_key);         
            //SendMyMessage();
          }
        }
        else
        {
          lv_key = idx + '1';
          if( relay_set_key(lv_key, !lv_onoff) ) {
            Msg_Relay_Ack(NODEID_GATEWAY, !lv_onoff ? V_RELAY_ON : V_RELAY_OFF, lv_key);
            //SendMyMessage();
          }
        }
      }
    }
  }
}

// Button Actions
void Button_Action(uint8_t _op, uint8_t _btn) {
#ifdef EN_PANEL_BUTTONS  
  if( _btn < MAX_NUM_BUTTONS ) {
    uint8_t lv_key;
    uint8_t lv_op = BF_GET(gConfig.btnAction[_btn][_op].action, 5, 3);
    if( lv_op == _op ) {
      uint8_t lv_obj = BF_GET(gConfig.btnAction[_btn][_op].action, 2, 3);
      uint8_t lv_act = BF_GET(gConfig.btnAction[_btn][_op].action, 0, 2);
      bool lv_onoff;
      switch( lv_obj ) {
      case BTN_OBJ_SCAN_KEY_MAP:
        // scan key map and act on keys one by one
        for( uint8_t idx = 0; idx < 8; idx++ ) {
          // Check key map
          if( BF_GET(gConfig.btnAction[_btn][_op].keyMap, idx, 1) ) {
            lv_key = idx + '1';
            lv_onoff = (lv_act == BTN_ACT_TOGGLE ? !relay_get_key(lv_key) : BTN_ACT_ON == lv_act);
            if( relay_set_key(lv_key, lv_onoff) ) {
              Msg_Relay_Ack(NODEID_GATEWAY, lv_onoff ? V_RELAY_ON : V_RELAY_OFF, lv_key);
              SendMyMessage();
            }
          }
        }
        break;

      case BTN_OBJ_LOOP_KEY_MAP:
        // ToDo:... leave this to Qiaoli
        /// get one key from key map, act on it, and move to the next key
        if(last_op == _op && last_btn == _btn)
        {
          // loop from last index
          uint8_t key_map_index = last_relay_key_index;
          if(lv_act != BTN_ACT_TOGGLE)
          { // turnon & turnoff and toggle switching time is different£¬handle separately
            if(relay_loop_tick <= BUTTON_TIMEOUT )
            { // loop from next index(switch)
              key_map_index = GetNextIndex(gConfig.btnAction[_btn][_op].keyMap,last_relay_key_index);
              last_relay_key_index = key_map_index;
            }
          }         
          if(key_map_index == 0)
          { // all
            LoopAll(gConfig.btnAction[_btn][_op].keyMap,lv_act);
          }
          else
          { // set relay on bit key_map_index
            LoopOne(gConfig.btnAction[_btn][_op].keyMap,lv_act,key_map_index);
          }       
        }
        else
        { // loop from start(all)
          LoopAll(gConfig.btnAction[_btn][_op].keyMap,lv_act);
          last_relay_key_index = 0;
        }
        break;

      default:
        // Act on corresponding relay key
        lv_key = _btn + '1';
        lv_onoff = (lv_act == BTN_ACT_TOGGLE ? !relay_get_key(lv_key) : BTN_ACT_ON == lv_act);
        if( relay_set_key(lv_key, lv_onoff) ) {
          Msg_Relay_Ack(NODEID_GATEWAY, lv_onoff ? V_RELAY_ON : V_RELAY_OFF, lv_key);
        }
        break;
      }
      //////////// zhangqiaoli add for loop relay ////////////////////////     
      last_btn = _btn;
      last_op = _op;
      //////////// zhangqiaoli add for loop relay ////////////////////////
    }
  }
#endif  
}

void btn_short_button_press(uint8_t _btn)
{
  // Assert button
  if( !IS_VALID_BUTTON(_btn) ) return;
  
  switch( _btn ) {
  case keylstCenter:
  case keylstUp:
  case keylstDown:
    Button_Action(BTN_OP_SHORT_PRESS, _btn);
    break;
    
  default:
    break;
  }
}

void btn_double_button_press(uint8_t _btn)
{
  // Assert button
  if( !IS_VALID_BUTTON(_btn) ) return;
  
  switch( _btn ) {
  case keylstCenter:
  case keylstUp:
  case keylstDown:
    Button_Action(BTN_OP_DBL_PRESS, _btn);
    break;

  default:
    break;
  }  
}

void btn_long_hold_button_press(uint8_t _btn)
{
  // Assert button
  if( !IS_VALID_BUTTON(_btn) ) return;
  
  switch( _btn ) {
  case keylstCenter:
  case keylstUp:
  case keylstDown:
    Button_Action(BTN_OP_LONG_HOLD, _btn);
    break;

  default:
    break;
  }  
}

void btn_long_button_press(uint8_t _btn)
{
  // Assert button
  if( !IS_VALID_BUTTON(_btn) ) return;
  
  switch( _btn ) {
  case keylstCenter:
  case keylstUp:
  case keylstDown:
    Button_Action(BTN_OP_LONG_RELEASE, _btn);
    break;

  default:
    break;
  }
}

void btn_very_long_hold_button_press(uint8_t _btn)
{
  // Assert button
  if( !IS_VALID_BUTTON(_btn) ) return;
  
  switch( _btn ) {
  case keylstCenter:
  case keylstUp:
  case keylstDown:
    Button_Action(BTN_OP_VLONG_HOLD, _btn);
    break;

  default:
    break;
  }
}

void btn_very_long_button_press(uint8_t _btn)
{
  // Assert button
  if( !IS_VALID_BUTTON(_btn) ) return;
  
  switch( _btn ) {
  case keylstCenter:
  case keylstUp:
  case keylstDown:
    Button_Action(BTN_OP_VLONG_RELEASE, _btn);
    break;

  default:
    break;
  }
}

void btn_double_long_hold_press(uint8_t _btn1, uint8_t _btn2)
{
  // Assert button
  if( !IS_VALID_BUTTON(_btn1) || !IS_VALID_BUTTON(_btn2) ) return;
}

void app_button_event_handler(uint8_t _btn, button_event_t button_event)
{
  uint8_t sec_btn = keylstDummy;
  
  switch (button_event)
  {
  case BUTTON_INVALID:
    break;
    
  case BUTTON_SHORT_PRESS:
    btn_short_button_press(_btn);
    break;
    
  case BUTTON_DOUBLE_PRESS:
    btn_double_button_press(_btn);
    break;
    
  case BUTTON_LONG_HOLD:
    btn_long_hold_button_press(_btn);
    break;
    
  case BUTTON_LONG_PRESS:
    btn_long_button_press(_btn);
    break;
    
  case BUTTON_VERY_LONG_HOLD:
    btn_very_long_hold_button_press(_btn);
    break;
    
  case BUTTON_VERY_LONG_PRESS:
    btn_very_long_button_press(_btn);
    break;
    
  case DOUBLE_BTN_TRACK:
    //if( btn_is_pushed[keylstFn1] ) sec_btn = keylstFn1;
    //btn_double_long_hold_press(_btn, sec_btn);
    
  default:
    break;
  }
}

// Only use button1_timer to track double button long hold.
void check_track_double_button(void)
{
}

void button_push(uint8_t _btn)
{
  // Assert button
  if( !IS_VALID_BUTTON(_btn) ) return;
  
  btn_is_pushed[_btn] = TRUE;
  check_track_double_button();
  
  if (double_button_track == FALSE)
  {
    m_btn_timer_status[_btn] = BUTTON_STATUS_LESS_2S;
    timer_start(m_timer_id_btn_detet[_btn], BUTTON_WAIT_2S);
  }
}

void button_release(uint8_t _btn)
{
  // Assert button
  if( !IS_VALID_BUTTON(_btn) ) return;
  
  btn_is_pushed[_btn] = FALSE;
  button_event_t button_event = BUTTON_INVALID;
  
  check_track_double_button();
  
  switch (m_btn_timer_status[_btn])
  {
  case BUTTON_STATUS_INIT:
    break;
    
  case BUTTON_STATUS_LESS_2S:
    if (detect_double_btn_press[_btn] == FALSE)
    {
      detect_double_btn_press[_btn] = TRUE;
      timer_start(m_timer_id_double_btn_detet[_btn], BUTTON_DOUBLE_BTN_DURATION);  // 500ms
    }
    else
    {
      button_event = BUTTON_DOUBLE_PRESS;
      detect_double_btn_press[_btn] = FALSE;
      timer_stop(m_timer_id_double_btn_detet[_btn]);
    }
    break;
    
  case BUTTON_STATUS_MORE_2S:
    button_event = BUTTON_LONG_PRESS;
    break;
    
  case BUTTON_STATUS_MORE_5S:
    button_event = BUTTON_VERY_LONG_PRESS;
    break;
    
  default:
    break;
  }
  
  m_btn_timer_status[_btn] = BUTTON_STATUS_INIT;
  if (button_event != BUTTON_INVALID) {
    app_button_event_handler(_btn, button_event);
  }
}

void button_event_handler()
{
  button_first_detect_status = GPIO_ReadInputData(BUTTONS_PORT);
  timer_start(m_timer_id_debonce_detet, BUTTON_DEBONCE_DURATION);
}

INTERRUPT_HANDLER(EXTI_PORTB_IRQHandler, 4)
{
#ifdef EN_PANEL_BUTTONS
  button_event_handler();
#endif  
}