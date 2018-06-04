
#include "infrared.h"
#include "timer_2.h"
#include "_global.h"

#define BUFFER_LEN 10
unsigned long  send_buf[BUFFER_LEN];
u8 send_buf_read_ptr = 0;
u8 send_buf_write_ptr = 0;
u8 send_buf_len = 0;
u16 ir_send_delay = 0;

#define BUFFER_AC_LEN 14
uint8_t air_haier_condition_buf[BUFFER_AC_LEN];
u8 ac_haier_buf_read_ptr = 0;
u8 ac_haier_buf_write_ptr = 0;
u8 ac_haier_buf_len = 0;

#define BUFFER_MEDIA_AC_LEN 3
uint8_t air_condition_media_buf[BUFFER_MEDIA_AC_LEN];
u8 ac_media_buf_read_ptr = 0;
u8 ac_media_buf_write_ptr = 0;
u8 ac_media_buf_len = 0;

#ifdef AIRCON_MEDIA
uint8_t mediaoff[4] = {0xB2,0x7B,0xE0,0x00};
uint8_t media_last_on_status[4] = {0xB2,0x3F,0xD0,0x00};
#endif
#ifdef AIRCON_HAIER
uint8_t haieroff[15] = {0xA6,0x20,0x00,0x00,0x00,0x40,0x00,0x20,0x00,0x00,0x00,0x00,0x85,0xAB,0x00};
// 26 cold autowind
uint8_t haier_last_on_status[15] = {0xA6,0xA2,0x00,0x00,0x40,0xA0,0x00,0x20,0x00,0x00,0x00,0x00,0x85,0xCD,0x00};
#endif

/*******************************************************************************
 * 名称: TIM1_PWM_Init
 * 功能: TIM1初始化函数 用作PWM输出 38khz
 * 形参: 无
 * 返回: 无
 * 说明: 无 
 ******************************************************************************/
void TIM1_PWM_Init(void)
{ 
  TIM1_DeInit();
  TIM1_TimeBaseInit(1-1, TIM1_COUNTERMODE_UP, 421, 0x00);       // 38khz 16000/38 = 421 
  
#ifdef CHANNEL1
  TIM1_OC1Init(TIM1_OCMODE_PWM2, TIM1_OUTPUTSTATE_ENABLE, TIM1_OUTPUTNSTATE_DISABLE,
               130, TIM1_OCPOLARITY_LOW, TIM1_OCNPOLARITY_HIGH, TIM1_OCIDLESTATE_SET,
              TIM1_OCNIDLESTATE_RESET); 
#endif
  
#ifdef CHANNEL4
  TIM1_OC4Init(TIM1_OCMODE_PWM2, TIM1_OUTPUTSTATE_ENABLE, 130, TIM1_OCPOLARITY_LOW, TIM1_OCIDLESTATE_RESET);
  TIM1_CCxCmd(TIM1_CHANNEL_4, ENABLE); 
  TIM1_OC4PreloadConfig(ENABLE);
#endif

  TIM1_Cmd(ENABLE);
  //TIM1_CtrlPWMOutputs(ENABLE);
}

/*******************************************************************************
 * 名称: Infrared_Send
 * 功能: 红外发送的开关开关
 * 形参: signed char status
 * 返回: 无
 * 说明: TRUE 打开 FALSE 关闭 
 ******************************************************************************/
void Infrared_Send_Status(bool status)
{
  //打开PWM
  if(status)
  {
    TIM1_CtrlPWMOutputs(ENABLE);
  }
  //关闭PWM
  else
  {
    TIM1_CtrlPWMOutputs(DISABLE);
  }
}

#define MAKR            Infrared_Send_Status(TRUE);
#define SPACE           Infrared_Send_Status(FALSE);
#define D9000US         Delay_50Us(180);
#define D4500US         Delay_50Us(90);
#define D550US          Delay_50Us(11);
#define D1700US         Delay_50Us(34);
#define D250US          Delay_50Us(5);
/*******************************************************************************
 * 名称: Infrared_Send
 * 功能: 红外发射
 * 形参: unsigned long data
 * 返回: 无
 * 说明: 无 
 ******************************************************************************/
void Infrared_Send(unsigned long data)
{
  
  uint8_t i = 0;
  
  
  SPACE 
  
  MAKR  D9000US
  SPACE D4500US
  
  for(i = 0; i < 32; i++)
  {
    MAKR        D550US
    if((data & 0x80000000) == 0)
    {
      SPACE     D550US
    }
    else
    {
      SPACE     D1700US 
    }
    
    data <<= 1;
  }
  
  MAKR   D250US
  SPACE
}

#define NEC_HDR_MARK	Infrared_Send_Status(TRUE);Delay_50Us(180);
#define NEC_HDR_SPACE	Infrared_Send_Status(FALSE);Delay_50Us(90);
#define NEC_BIT_MARK	Infrared_Send_Status(TRUE);Delay_50Us(11);
#define NEC_ONE_SPACE	Infrared_Send_Status(FALSE);Delay_50Us(34);
#define NEC_ZERO_SPACE	Infrared_Send_Status(FALSE);Delay_50Us(11);
#define NEC_RPT_SPACE	Infrared_Send_Status(FALSE);Delay_50Us(45);


/*******************************************************************************
 * 名称: NEC_Infrared_Send
 * 功能: 红外发射 类型NEC
 * 形参: unsigned long data
 * 返回: 无
 * 说明: 无 
 ******************************************************************************/
void NEC_Infrared_Send(unsigned long data)
{
  disableInterrupts();
  NEC_HDR_MARK
  NEC_HDR_SPACE
    
  //disableInterrupts();
  
  for (int i = 0; i < 32; i++) {
    if (data & 0x80000000) {
      NEC_BIT_MARK
      NEC_ONE_SPACE
    } 
    else {
      NEC_BIT_MARK
      NEC_ZERO_SPACE
    }
    data <<= 1;
  }
  NEC_BIT_MARK
  
  //enableInterrupts();
  
  Infrared_Send_Status(FALSE);
  enableInterrupts();
}


#define HAIER_HDR_MARK          Infrared_Send_Status(TRUE);     Delay_50Us(60);
#define HAIER_HDR_SPACE         Infrared_Send_Status(FALSE);    Delay_50Us(60);
#define HAIER_HDR_SPACE2        Infrared_Send_Status(FALSE);    Delay_50Us(90);
#define HAIER_BIT_MARK          Infrared_Send_Status(TRUE);     Delay_50Us(12);
#define HAIER_ONE_SPACE         Infrared_Send_Status(FALSE);    Delay_50Us(33);
#define HAIER_ZERO_SPACE	Infrared_Send_Status(FALSE);    Delay_50Us(12);
/*******************************************************************************
 * 名称: Haier_Infrared_Send
 * 功能: 红外发射 类型海尔
 * 形参: unsigned long data
 * 返回: 无
 * 说明: 无 
 ******************************************************************************/
void Haier_Infrared_Send(uint8_t data[], int len)
{
  printlog("Haier_Infrared_Send...\r\n");
  disableInterrupts();
  HAIER_HDR_MARK
  HAIER_HDR_SPACE
  HAIER_HDR_MARK
  HAIER_HDR_SPACE2
    
  //disableInterrupts();
  
  for(int i=0; i<len; i++)
  {
    uint8_t temp = data[i];
    for(int j=0; j<8; j++)
    {
      if (temp & 0x80) {
        HAIER_BIT_MARK
        HAIER_ONE_SPACE
      } 
      else 
      {
        HAIER_BIT_MARK
        HAIER_ZERO_SPACE
      }
      temp <<= 1;
    }
  }
  
  HAIER_BIT_MARK  
  
  Infrared_Send_Status(FALSE);
  enableInterrupts();
}


#define MEDIA_HDR_MARK          Infrared_Send_Status(TRUE);     Delay_50Us(88);
#define MEDIA_HDR_SPACE         Infrared_Send_Status(FALSE);    Delay_50Us(88);
#define MEDIA_BIT_MARK          Infrared_Send_Status(TRUE);     Delay_50Us(11);
#define MEDIA_ONE_SPACE         Infrared_Send_Status(FALSE);    Delay_50Us(32);
#define MEDIA_ZERO_SPACE	Infrared_Send_Status(FALSE);    Delay_50Us(11);

/*******************************************************************************
 * 名称: Media_Infrared_Send
 * 功能: 红外发射
 * 形参:  uint8_t base	 地址码
 *        uint8_t high	高位字节
 *        uint8_t low	低位字节
 * 返回: 无
 * 说明: 无 
 ******************************************************************************/
void Media_Infrared_Send(uint8_t base, uint8_t high, uint8_t low)
{
  printlog("Media_Infrared_Send...\r\n");
  disableInterrupts();
  MEDIA_HDR_MARK
  MEDIA_HDR_SPACE
  
  uint8_t temp = base;
  for(int i=0;i<8;i++) {
    MEDIA_BIT_MARK;
    if(temp & 0x80) {
      MEDIA_ONE_SPACE;
    }
    else {
      MEDIA_ZERO_SPACE;
    }
    
    temp <<= 1;
  }
  
  temp = base;
  for(int i=0;i<8;i++) {
    MEDIA_BIT_MARK;
    if(temp & 0x80) {
      MEDIA_ZERO_SPACE;
    }
    else {
      MEDIA_ONE_SPACE;
    }
    
    temp <<= 1;
  }
  
  temp = high;
  for(int i=0;i<8;i++) {
    MEDIA_BIT_MARK;
    if(temp & 0x80) {
      MEDIA_ONE_SPACE;
    }
    else {
      MEDIA_ZERO_SPACE;
    }
    
    temp <<= 1;
  }
  
  temp = high;
  for(int i=0;i<8;i++) {
    MEDIA_BIT_MARK;
    if(temp & 0x80) {
      MEDIA_ZERO_SPACE;
    }
    else {
      MEDIA_ONE_SPACE;
    }
    
    temp <<= 1;
  }
  
  temp = low;
  for(int i=0;i<8;i++) {
    MEDIA_BIT_MARK;
    if(temp & 0x80) {
      MEDIA_ONE_SPACE;
    }
    else {
      MEDIA_ZERO_SPACE;
    }
    
    temp <<= 1;
  }
  
  temp = low;
  for(int i=0;i<8;i++) {
    MEDIA_BIT_MARK;
    if(temp & 0x80) {
      MEDIA_ZERO_SPACE;
    }
    else {
      MEDIA_ONE_SPACE;
    }
    
    temp <<= 1;
  }
  
  MEDIA_BIT_MARK;
  MEDIA_HDR_SPACE;
  MEDIA_HDR_MARK;
  MEDIA_HDR_SPACE;
  
  temp = base;
  for(int i=0;i<8;i++) {
    MEDIA_BIT_MARK;
    if(temp & 0x80) {
      MEDIA_ONE_SPACE;
    }
    else {
      MEDIA_ZERO_SPACE;
    }
    
    temp <<= 1;
  }
  
  temp = base;
  for(int i=0;i<8;i++) {
    MEDIA_BIT_MARK;
    if(temp & 0x80) {
      MEDIA_ZERO_SPACE;
    }
    else {
      MEDIA_ONE_SPACE;
    }
    
    temp <<= 1;
  }
  
  temp = high;
  for(int i=0;i<8;i++) {
    MEDIA_BIT_MARK;
    if(temp & 0x80) {
      MEDIA_ONE_SPACE;
    }
    else {
      MEDIA_ZERO_SPACE;
    }
    
    temp <<= 1;
  }
  
  temp = high;
  for(int i=0;i<8;i++) {
    MEDIA_BIT_MARK;
    if(temp & 0x80) {
      MEDIA_ZERO_SPACE;
    }
    else {
      MEDIA_ONE_SPACE;
    }
    
    temp <<= 1;
  }
  
  temp = low;
  for(int i=0;i<8;i++) {
    MEDIA_BIT_MARK;
    if(temp & 0x80) {
      MEDIA_ONE_SPACE;
    }
    else {
      MEDIA_ZERO_SPACE;
    }
    
    temp <<= 1;
  }
  
  temp = low;
  for(int i=0;i<8;i++) {
    MEDIA_BIT_MARK;
    if(temp & 0x80) {
      MEDIA_ZERO_SPACE;
    }
    else {
      MEDIA_ONE_SPACE;
    }
    
    temp <<= 1;
  }
  MEDIA_BIT_MARK;
  Infrared_Send_Status(FALSE);
  enableInterrupts();
}

/*******************************************************************************
 * 名称: Infrared_Init()
 * 功能: 初始化函数
 * 形参: 无
 * 返回: 无
 * 说明: 无 
 ******************************************************************************/
void Infrared_Init(void)
{
  TIM1_PWM_Init();

  // Init Timer
  TIM2_Init();
  
  ir_send_delay = 0;
  send_buf_read_ptr = 0;
  send_buf_write_ptr = 0;
  send_buf_len = 0;
  
  ac_haier_buf_read_ptr = 0;
  ac_haier_buf_write_ptr = 0;
  ac_haier_buf_len = 0;
  
  ac_media_buf_read_ptr = 0;
  ac_media_buf_write_ptr = 0;
  ac_media_buf_len = 0;
  
  enableInterrupts(); 
}

bool Set_Send_Buf(u32 *buf, u8 len)
{
  if( send_buf_len + len > BUFFER_LEN ) return FALSE;
  
  for( u8 i=0; i<len; i++ ) {
    send_buf[send_buf_write_ptr++] = buf[i];
    send_buf_write_ptr %= BUFFER_LEN;
  }
  send_buf_len += len;
  return TRUE;
}

bool Set_AC_Buf(uint8_t *buf, u8 len)
{
  if( ac_haier_buf_len + len > BUFFER_AC_LEN ) return FALSE;
  
  for( u8 i=0; i<len; i++ ) {
    air_haier_condition_buf[ac_haier_buf_write_ptr++] = buf[i];
    ac_haier_buf_write_ptr %= BUFFER_AC_LEN;
  }
  ac_haier_buf_len += len;
  return TRUE;
}

bool Set_AC_Media_Buf(uint8_t *buf, u8 len)
{
  if( ac_media_buf_len + len > BUFFER_MEDIA_AC_LEN ) return FALSE;
  
  for( u8 i=0; i<len; i++ ) {
    air_condition_media_buf[ac_media_buf_write_ptr++] = buf[i];
    ac_media_buf_write_ptr %= BUFFER_MEDIA_AC_LEN;
  }
  ac_media_buf_len += len;
  return TRUE;
}

void IR_Send()
{
  // Send one element each time
  if( send_buf_len > 0 && ir_send_delay == 0 ) {
    NEC_Infrared_Send(send_buf[send_buf_read_ptr++]);
    send_buf_read_ptr %= BUFFER_LEN;
    send_buf_len--;
    // Start timer: delay 100ms
    if( send_buf_len > 0 ) {
      ir_send_delay = 10;
    }
  }
#ifdef EN_INFRARED  
  
  // Send all data at a time
  if( ac_haier_buf_len > 0 ) {
    Haier_Infrared_Send(air_haier_condition_buf, ac_haier_buf_len);
    if(air_haier_condition_buf[4] != 0x00)
    { // not off,record last open status
      //memcpy(haier_last_on_status,air_haier_condition_buf,ac_haier_buf_len);
      if(!isIdentityEqual(gConfig.aircondition_on_status,air_haier_condition_buf,ac_haier_buf_len))
      {
        memcpy(gConfig.aircondition_on_status,air_haier_condition_buf,ac_haier_buf_len);
        gIsStatusChanged = TRUE;
      }
    }
    ac_haier_buf_write_ptr = 0;
    ac_haier_buf_read_ptr = 0;
    ac_haier_buf_len = 0;
  }

    // Send all data at a time
  if( ac_media_buf_len > 0 ) {
    Media_Infrared_Send(air_condition_media_buf[0],air_condition_media_buf[1],air_condition_media_buf[2]);
    if(air_condition_media_buf[1] != 0x7B || air_condition_media_buf[2] != 0xE0)
    { // not off,record last open status
      //media_last_on_status[1] = air_condition_media_buf[1];
      //media_last_on_status[2] = air_condition_media_buf[2];
      if(!isIdentityEqual(gConfig.aircondition_on_status,air_condition_media_buf,ac_media_buf_len))
      {
        memcpy(gConfig.aircondition_on_status,air_condition_media_buf,ac_media_buf_len);
        gIsStatusChanged = TRUE;
      }
    }
    ac_media_buf_write_ptr = 0;
    ac_media_buf_read_ptr = 0;
    ac_media_buf_len = 0;
  }
  
#endif
}
