#include <stm8s.h>
#include "_global.h"
#include "relay_key.h"
#include "MyMessage.h"

#define SOFT_KEY_1_PIN_PORT             GPIOD
#define SOFT_KEY_1_PIN_ID               GPIO_PIN_4

#define SOFT_KEY_2_PIN_PORT             GPIOD
#define SOFT_KEY_2_PIN_ID               GPIO_PIN_3

#define SOFT_KEY_3_PIN_PORT             GPIOD
#define SOFT_KEY_3_PIN_ID               GPIO_PIN_2

#define SOFT_KEY_4_PIN_PORT             GPIOD
#define SOFT_KEY_4_PIN_ID               GPIO_PIN_0

//#define SOFT_KEY_5_PIN_PORT             GPIOF
//#define SOFT_KEY_5_PIN_ID               GPIO_PIN_4

//#define SOFT_KEY_6_PIN_PORT             GPIOD
//#define SOFT_KEY_6_PIN_ID               GPIO_PIN_7

//#define SOFT_KEY_7_PIN_PORT             GPIOB
//#define SOFT_KEY_7_PIN_ID               GPIO_PIN_1

//#define SOFT_KEY_8_PIN_PORT             GPIOB
//#define SOFT_KEY_8_PIN_ID               GPIO_PIN_2

void relay_key_init()
{
#ifdef SOFT_KEY_1_PIN_ID
  GPIO_Init(SOFT_KEY_1_PIN_PORT, SOFT_KEY_1_PIN_ID, GPIO_MODE_OUT_PP_LOW_SLOW);
#endif  

#ifdef SOFT_KEY_2_PIN_ID
  GPIO_Init(SOFT_KEY_2_PIN_PORT, SOFT_KEY_2_PIN_ID, GPIO_MODE_OUT_PP_LOW_SLOW);
#endif  

#ifdef SOFT_KEY_3_PIN_ID
  GPIO_Init(SOFT_KEY_3_PIN_PORT, SOFT_KEY_3_PIN_ID, GPIO_MODE_OUT_PP_LOW_SLOW);
#endif  

#ifdef SOFT_KEY_4_PIN_ID
  GPIO_Init(SOFT_KEY_4_PIN_PORT, SOFT_KEY_4_PIN_ID, GPIO_MODE_OUT_PP_LOW_SLOW);
#endif  

#ifdef SOFT_KEY_5_PIN_ID
  GPIO_Init(SOFT_KEY_5_PIN_PORT, SOFT_KEY_5_PIN_ID, GPIO_MODE_OUT_PP_LOW_SLOW);
#endif  

#ifdef SOFT_KEY_6_PIN_ID
  GPIO_Init(SOFT_KEY_6_PIN_PORT, SOFT_KEY_6_PIN_ID, GPIO_MODE_OUT_PP_LOW_SLOW);
#endif  

#ifdef SOFT_KEY_7_PIN_ID
  GPIO_Init(SOFT_KEY_7_PIN_PORT, SOFT_KEY_7_PIN_ID, GPIO_MODE_OUT_PP_LOW_SLOW);
#endif  

#ifdef SOFT_KEY_8_PIN_ID
  GPIO_Init(SOFT_KEY_8_PIN_PORT, SOFT_KEY_8_PIN_ID, GPIO_MODE_OUT_PP_LOW_SLOW);
#endif
  
#ifdef EN_PANEL_BUTTONS
  // spotlight need restore last status
  // Restore previous on/off states
  relay_restore_keystates();
#endif
}

void relay_restore_keystates() {
  for( u8 key = '1'; key <= '8'; key++ ) {
    relay_set_key(key, BF_GET(gConfig.relay_key_value, key - '1', 1));
  }  
}

void relay_set_keys(uint8_t keyvalue) {
  for( u8 key = '1'; key <= '8'; key++ ) {
    relay_set_key(key, BF_GET(keyvalue, key - '1', 1));
  }  
}

void relay_gpio_write_bit(GPIO_TypeDef* GPIOx, GPIO_Pin_TypeDef PortPins, bool _on)
{
  if( _on ) {
    GPIO_WriteHigh(GPIOx, PortPins);
  } else {
    GPIO_WriteLow(GPIOx, PortPins);
  }  
}

bool relay_get_key(u8 _key)
{
  bool rc = FALSE;
  
  if( _key >= '1' && _key <= '8' ) {
    rc = BF_GET(gConfig.relay_key_value, _key - '1', 1);
  }
  
  return rc;
}

bool relay_set_key(u8 _key, bool _on)
{
  bool rc = FALSE;
  
  switch( _key ) {
  case '1':
#ifdef SOFT_KEY_1_PIN_ID
  relay_gpio_write_bit(SOFT_KEY_1_PIN_PORT, SOFT_KEY_1_PIN_ID, _on);
  BF_SET(gConfig.relay_key_value, _on, 0, 1);
  rc = TRUE;
#endif    
    break;

  case '2':
#ifdef SOFT_KEY_2_PIN_ID
  relay_gpio_write_bit(SOFT_KEY_2_PIN_PORT, SOFT_KEY_2_PIN_ID, _on);
  BF_SET(gConfig.relay_key_value, _on, 1, 1);
  rc = TRUE;
#endif    
    break;

  case '3':
#ifdef SOFT_KEY_3_PIN_ID
  relay_gpio_write_bit(SOFT_KEY_3_PIN_PORT, SOFT_KEY_3_PIN_ID, _on);
  BF_SET(gConfig.relay_key_value, _on, 2, 1);
  rc = TRUE;
#endif    
    break;

  case '4':
#ifdef SOFT_KEY_4_PIN_ID
  relay_gpio_write_bit(SOFT_KEY_4_PIN_PORT, SOFT_KEY_4_PIN_ID, _on);
  BF_SET(gConfig.relay_key_value, _on, 3, 1);
  rc = TRUE;
#endif    
    break;

  case '5':
#ifdef SOFT_KEY_5_PIN_ID
  relay_gpio_write_bit(SOFT_KEY_5_PIN_PORT, SOFT_KEY_5_PIN_ID, _on);
  BF_SET(gConfig.relay_key_value, _on, 4, 1);
  rc = TRUE;
#endif    
    break;

  case '6':
#ifdef SOFT_KEY_6_PIN_ID
  relay_gpio_write_bit(SOFT_KEY_6_PIN_PORT, SOFT_KEY_6_PIN_ID, _on);
  BF_SET(gConfig.relay_key_value, _on, 5, 1);
  rc = TRUE;
#endif    
    break;

  case '7':
#ifdef SOFT_KEY_7_PIN_ID
  relay_gpio_write_bit(SOFT_KEY_7_PIN_PORT, SOFT_KEY_7_PIN_ID, _on);
  BF_SET(gConfig.relay_key_value, _on, 6, 1);
  rc = TRUE;
#endif    
    break;

  case '8':
#ifdef SOFT_KEY_8_PIN_ID
  relay_gpio_write_bit(SOFT_KEY_8_PIN_PORT, SOFT_KEY_8_PIN_ID, _on);
  BF_SET(gConfig.relay_key_value, _on, 7, 1);
  rc = TRUE;
#endif    
    break;
  }
  
  if( rc ) gIsStatusChanged = TRUE;
  
  return rc;
}

bool IsValidRelaykey(u8 index)
{
  u8 key = index + '1';
  bool rc = FALSE;
  
  switch( key ) {
  case '1':
#ifdef SOFT_KEY_1_PIN_ID
  rc = TRUE;
#endif    
    break;

  case '2':
#ifdef SOFT_KEY_2_PIN_ID
  rc = TRUE;
#endif    
    break;

  case '3':
#ifdef SOFT_KEY_3_PIN_ID
  rc = TRUE;
#endif    
    break;

  case '4':
#ifdef SOFT_KEY_4_PIN_ID
  rc = TRUE;
#endif    
    break;

  case '5':
#ifdef SOFT_KEY_5_PIN_ID
  rc = TRUE;
#endif    
    break;

  case '6':
#ifdef SOFT_KEY_6_PIN_ID
  rc = TRUE;
#endif    
    break;

  case '7':
#ifdef SOFT_KEY_7_PIN_ID
  rc = TRUE;
#endif    
    break;

  case '8':
#ifdef SOFT_KEY_8_PIN_ID
  rc = TRUE;
#endif    
    break;
  }
  
  return rc;
}