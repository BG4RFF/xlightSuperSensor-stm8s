#include <stm8s.h>
#include "_global.h"
#include "keySimulator.h"
#include "MyMessage.h"
#include "xliNodeConfig.h"

keyBuffer_t gKeyBuf[KEY_OP_MAX_BUFFERS];

typedef struct {
  uint8_t target;
  uint8_t key;
  GPIO_TypeDef* port;
  GPIO_Pin_TypeDef pin;
} keyPinMap_t;

#define KEYMAP_TABLE_ROWS          3
const keyPinMap_t keyMapTable[] = {
  // target     key     port    pin
  {0,           '1',    GPIOD,  GPIO_PIN_4},    // Up
  {0,           '2',    GPIOD,  GPIO_PIN_3},    // Down
  {0,           '3', 	GPIOD, 	GPIO_PIN_2}     // Stop
};

bool LookupKeyPinMap(uint8_t target, uint8_t key, GPIO_TypeDef **port, GPIO_Pin_TypeDef *pin)
{
  for( u8 i = 0; i < KEYMAP_TABLE_ROWS; i++ ) {
    if( target == keyMapTable[i].target && key == keyMapTable[i].key ) {
      *port = keyMapTable[i].port;
      *pin = keyMapTable[i].pin;
      return TRUE;
    }
  }
  return FALSE;
}

void keySimulator_init() {
  u8 i;
  for( i = 0; i < KEY_OP_MAX_BUFFERS; i++ ) {
    gKeyBuf[i].target = 0;
    gKeyBuf[i].keyNum = 0;    
    gKeyBuf[i].ptr = 0;
    gKeyBuf[i].tick = 0;
    gKeyBuf[i].key_on = FALSE;
    gKeyBuf[i].key_tick = 0;
    gKeyBuf[i].key_delay = 0;
    gKeyBuf[i].key_dbl_step = 0;
    memset(gKeyBuf[i].keys, 0x00, sizeof(keyStyle_t) * KEY_OP_MAX_KEYS);
  }
  
  for( i = 0; i < KEYMAP_TABLE_ROWS; i++ ) {
    GPIO_Init(keyMapTable[i].port, keyMapTable[i].pin, GPIO_MODE_OUT_PP_LOW_SLOW);
  }
}

// Parse keystring and put operation series into an available buffer
bool ProduceKeyOperation(u8 _target, const char *_keyString, u8 _len) {
  u8 _keyNum = 0;
  u8 _conkeyNum;
  bool _isConkey;
  for( u8 i = 0; i < KEY_OP_MAX_BUFFERS; i++ ) {
    if( gKeyBuf[i].keyNum == 0 ) {
      // Found available buffer
      _conkeyNum = 0;
      for( u8 j = 0; j < _len; j++ ) {
        // Delay
        _isConkey = FALSE;
        if( j == 0 ) gKeyBuf[i].keys[_keyNum].delay = 0;
        else {
          switch(_keyString[j]) {
          case KEY_DELI_NO_PAUSE:
            gKeyBuf[i].keys[_keyNum].delay = 0;
            break;
          case KEY_DELI_SMALL_PAUSE:
            gKeyBuf[i].keys[_keyNum].delay = 20;
            break;
          case KEY_DELI_NORMAL_PAUSE:
            gKeyBuf[i].keys[_keyNum].delay = 60;
            break;
          case KEY_DELI_LONG_PAUSE:
            gKeyBuf[i].keys[_keyNum].delay = 200;
            break;
          case KEY_DELI_VLONG_PAUSE:
            gKeyBuf[i].keys[_keyNum].delay = 500;
            break;
          case KEY_DELI_SAME_TIME:
            gKeyBuf[i].keys[_keyNum].delay = 0;
            _isConkey = TRUE;
          default:
            gKeyBuf[i].keys[_keyNum].delay = 0;
            break;
          }
          if( ++j >= _len ) break;
        }
        
        if( _isConkey && _keyNum > 0 ) {
          // Move to previous
          _keyNum--;
          _conkeyNum++;
          _conkeyNum %= KEY_OP_MAX_CON_KEYS;
        } else {
          _conkeyNum = 0;
        }
        gKeyBuf[i].keys[_keyNum].op = _keyString[j];
        if( ++j >= _len ) break;
        gKeyBuf[i].keys[_keyNum].keyID[_conkeyNum] = _keyString[j];
        if( _conkeyNum + 1 < KEY_OP_MAX_CON_KEYS ) {
          gKeyBuf[i].keys[_keyNum].keyID[_conkeyNum + 1] = 0;
        }
        if( ++_keyNum >= KEY_OP_MAX_KEYS ) break;
      }
      gKeyBuf[i].target = _target;
      gKeyBuf[i].ptr = 0;
      gKeyBuf[i].tick = 0;
      gKeyBuf[i].key_on = FALSE;
      gKeyBuf[i].key_tick = 0;
      gKeyBuf[i].key_delay = 0;
      gKeyBuf[i].key_dbl_step = 0;
      gKeyBuf[i].keyNum = _keyNum;
      return TRUE;
    }
  }
  return FALSE;
}

void WriteRelayKey(u8 _key, bool _on)
{
  bool rc = FALSE;
  switch( _key ) {
  case '1':
    BF_SET(gConfig.relay_key_value, _on, 0, 1);
    rc = TRUE;  
    break;

  case '2':
    BF_SET(gConfig.relay_key_value, _on, 1, 1);
    rc = TRUE;   
    break;

  case '3':
    BF_SET(gConfig.relay_key_value, _on, 2, 1);
    rc = TRUE;
    break;
  }
  
  if( rc ) gIsStatusChanged = TRUE;
}

u8 SimulateKeyPress(u8 _target, u8 _op, u8 *_keys) {
  GPIO_TypeDef *_port;
  GPIO_Pin_TypeDef _pin;
  u8 _delay = 0;
  
  for( u8 i = 0; i < KEY_OP_MAX_CON_KEYS; i++ ) {
    if( _keys[i] == 0 ) break;
    
    if( LookupKeyPinMap(_target, _keys[i], &_port, &_pin) ) {
      switch(_op) {
      case KEY_OP_STYLE_PRESS:
        relay_gpio_write_bit(_port, _pin, TRUE);
        WriteRelayKey(_keys[i],TRUE);
        _delay = 20;
        break;
      case KEY_OP_STYLE_FAST_PRESS:
        relay_gpio_write_bit(_port, _pin, TRUE);
        WriteRelayKey(_keys[i],TRUE);
        _delay = 10;
        break;
      case KEY_OP_STYLE_LONG_PRESS:
        relay_gpio_write_bit(_port, _pin, TRUE);
        WriteRelayKey(_keys[i],TRUE);
        _delay = 50;
        break;
      case KEY_OP_STYLE_VLONG_PRESS:
        relay_gpio_write_bit(_port, _pin, TRUE);
        WriteRelayKey(_keys[i],TRUE);
        _delay = 250;
        break;
      case KEY_OP_STYLE_HOLD:
        relay_gpio_write_bit(_port, _pin, TRUE);
        WriteRelayKey(_keys[i],TRUE);
        _delay = 0;
        break;
      case KEY_OP_STYLE_RELEASE:
        relay_gpio_write_bit(_port, _pin, FALSE);
        WriteRelayKey(_keys[i],FALSE);
        _delay = 0;
        break;
      case KEY_OP_STYLE_DBL_CLICK:
        relay_gpio_write_bit(_port, _pin, TRUE);
        WriteRelayKey(_keys[i],TRUE);
        _delay = 10;
        break;
      }
    }
  }
  return _delay;
}

bool FinishKeyPress(u8 _target, u8 _op, u8 *_keys, u8 _step) {
  GPIO_TypeDef *_port;
  GPIO_Pin_TypeDef _pin;
  bool rc = TRUE;

  for( u8 i = 0; i < KEY_OP_MAX_CON_KEYS; i++ ) {
    if( _keys[i] == 0 ) break;
    
    if( LookupKeyPinMap(_target, _keys[i], &_port, &_pin) ) {
      switch(_op) {
      case KEY_OP_STYLE_PRESS:
      case KEY_OP_STYLE_FAST_PRESS:
      case KEY_OP_STYLE_LONG_PRESS:
      case KEY_OP_STYLE_VLONG_PRESS:
        relay_gpio_write_bit(_port, _pin, FALSE);
        WriteRelayKey(_keys[i],FALSE);
        break;
      case KEY_OP_STYLE_DBL_CLICK:
        if( _step < 4 ) { // 1 - on to off; 2 - off to on; 3 - on to off;
          relay_gpio_write_bit(_port, _pin, _step % 2 == 0);
          WriteRelayKey(_keys[i],_step % 2 == 0);
          rc = FALSE;
        }
        break;
      }
    }
  }
  return rc;
}

void ScanKeyBuffer(u8 _idx) {
  u8 _k = gKeyBuf[_idx].ptr;
  if( _k < gKeyBuf[_idx].keyNum ) {
    if( gKeyBuf[_idx].key_on ) {
      // In the middle of key operation
      if( ++gKeyBuf[_idx].key_tick > gKeyBuf[_idx].key_delay ) {
        // Finished, move to next key
        gKeyBuf[_idx].key_tick = 0;
        if( FinishKeyPress(gKeyBuf[_idx].target, gKeyBuf[_idx].keys[_k].op, gKeyBuf[_idx].keys[_k].keyID, gKeyBuf[_idx].key_dbl_step) ) {
          gKeyBuf[_idx].key_on = FALSE;
          gKeyBuf[_idx].tick = 0;
          gKeyBuf[_idx].ptr++;
        } else {
          gKeyBuf[_idx].key_dbl_step++;
        }
      }
    } else {
      // Wait for next key operation
      if( ++gKeyBuf[_idx].tick > gKeyBuf[_idx].keys[_k].delay ) {
        // Trigger a new key
        gKeyBuf[_idx].key_delay = SimulateKeyPress(gKeyBuf[_idx].target, gKeyBuf[_idx].keys[_k].op, gKeyBuf[_idx].keys[_k].keyID);
        gKeyBuf[_idx].tick = 0;
        gKeyBuf[_idx].key_tick = 0;
        gKeyBuf[_idx].key_on = TRUE;
        gKeyBuf[_idx].key_dbl_step = (gKeyBuf[_idx].keys[_k].op == KEY_OP_STYLE_DBL_CLICK ? 1 : 0);
      }
    }
  } else {
    // All done
    gKeyBuf[_idx].keyNum = 0;
  }
}