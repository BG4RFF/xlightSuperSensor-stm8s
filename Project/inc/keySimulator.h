#ifndef __KEY_SIMULATOR_H
#define __KEY_SIMULATOR_H

#define KEY_OP_MAX_BUFFERS      4
#define KEY_OP_MAX_KEYS         10

typedef struct
{
  u16 delay;
  u8 op;
  u8 keyID;
} keyStyle_t;

typedef struct
{
  u8 target;
  u8 keyNum;
  u8 ptr;
  u16 tick;
  bool key_on;
  u8 key_tick;
  u8 key_delay;
  u8 key_dbl_step;
  keyStyle_t keys[KEY_OP_MAX_KEYS];
} keyBuffer_t;

extern keyBuffer_t gKeyBuf[KEY_OP_MAX_BUFFERS];

void keySimulator_init();
bool ProduceKeyOperation(u8 _target, const char *_keyString, u8 _len);
u8 SimulateKeyPress(u8 _target, u8 _op, u8 _key);
bool FinishKeyPress(u8 _target, u8 _op, u8 _key, u8 _step);
void ScanKeyBuffer(u8 _idx);

#endif /* __KEY_SIMULATOR_H */