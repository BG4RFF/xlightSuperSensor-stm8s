#include "_global.h"
#include "rf24l01.h"
#include "MyMessage.h"
#include "xliNodeConfig.h"
#include "ProtocolParser.h"
#include "timer_4.h"
#include "FlashDataStorage.h"
#include "wwdg.h"
#include "Uart2Dev.h"

#ifdef ZENREMOTE
#include "relay_key.h"
#ifndef EN_PANEL_BUTTONS
#include "keySimulator.h"
#endif
#endif

#ifdef MULTI_SENSOR
#include "sen_multi.h"
#endif

#ifdef EN_INFRARED
#include "infrared.h"
#endif

#if (defined EN_SENSOR_ALS) || (defined EN_SENSOR_MIC)
#include "ADC1Dev.h"
#endif

#ifdef EN_SENSOR_ALS
#include "sen_als.h"
#endif

#ifdef EN_SENSOR_MIC
#include "sen_mic.h"
#endif

#ifdef EN_SENSOR_PIR
#include "sen_pir.h"
#endif

#ifdef EN_SENSOR_IRKEY
#include "sen_irkey.h"
#endif

#ifdef EN_SENSOR_PM25
#include "sen_pm25.h"
#endif

#ifdef EN_SENSOR_DHT
#include "sen_dht.h"
#endif

#ifdef EN_PANEL_BUTTONS
#include "button.h"
#endif
/*
License: MIT

Auther: Baoshi Sun
Email: bs.sun@datatellit.com, bs.sun@uwaterloo.ca
Github: https://github.com/sunbaoshi1975
Please visit xlight.ca for product details

RF24L01 connector pinout:
GND    VCC
CE     CSN
SCK    MOSI
MISO   IRQ

Connections:
  PC3 -> CE
  PC4 -> CSN
  PC7 -> MISO
  PC6 -> MOSI
  PC5 -> SCK
  PC2 -> IRQ

*/

#ifdef TEST
void testio()
{
  GPIO_Init(GPIOB , GPIO_PIN_5 , GPIO_MODE_OUT_PP_LOW_SLOW);
  GPIO_Init(GPIOB , GPIO_PIN_4 , GPIO_MODE_OUT_PP_LOW_SLOW);
  GPIO_Init(GPIOB , GPIO_PIN_3 , GPIO_MODE_OUT_PP_LOW_SLOW);
  GPIO_Init(GPIOB , GPIO_PIN_2 , GPIO_MODE_OUT_PP_LOW_SLOW);
  GPIO_Init(GPIOB , GPIO_PIN_1 , GPIO_MODE_OUT_PP_LOW_SLOW);
  GPIO_Init(GPIOD , GPIO_PIN_1 , GPIO_MODE_OUT_PP_LOW_SLOW);
  GPIO_Init(GPIOD , GPIO_PIN_7 , GPIO_MODE_OUT_PP_LOW_SLOW);
}
#endif

// RF channel for the sensor net, 0-127
#define RF24_CHANNEL	   		100

// System Startup Status
#define SYS_INIT                        0
#define SYS_RESET                       1
#define SYS_WAIT_NODEID                 2
#define SYS_WAIT_PRESENTED              3
#define SYS_RUNNING                     5

// Keep alive message interval, around 6 seconds
#define RTE_TM_KEEP_ALIVE               1500   // about 15s (1500 * 10ms)
#define MAX_RF_FAILED_TIME              10     // Reset RF module when reach max failed times of sending

// Sensor reading duration
#define SEN_MAX_SEND_INTERVAL           6000   // about 60s (6000 * 10ms)
#define SEN_READ_ALS                    600    // about 6s (600 * 10ms)
#define SEN_READ_MIC                    200    // about 2s (200 * 10ms)
#define SEN_READ_PIR                    10     // about 100ms (10 * 10ms)
#define SEN_READ_IRKEY                  50     // about 500ms (50 * 10ms)
#define SEN_READ_PM25                   800    // about 8s (800 * 10ms)
#define SEN_READ_DHT                    1000   // about 10s (1000 * 10ms)
#define SEN_COLLECT_DHT                 50     // about 500ms (50 * 10ms)

// For Gu'an Demo Classroom
#define ONOFF_RESET_TIMES               5     // on / off times to reset device, regular value is 3

#define RAPID_PRESENTATION                     // Don't wait for presentation-ack
#define REGISTER_RESET_TIMES            30     // default 5, super large value for show only to avoid ID mess


// Unique ID
#if defined(STM8S105) || defined(STM8S005) || defined(STM8AF626x)
  #define     UNIQUE_ID_ADDRESS         (0x48CD)
#endif
#if defined(STM8S103) || defined(STM8S003) ||  defined(STM8S903)
  #define     UNIQUE_ID_ADDRESS         (0x4865)
#endif

const UC RF24_BASE_RADIO_ID[ADDRESS_WIDTH] = {0x00,0x54,0x49,0x54,0x44};

// Public variables
Config_t gConfig;
MyMessage_t sndMsg, rcvMsg;
uint8_t *psndMsg = (uint8_t *)&sndMsg;
uint8_t *prcvMsg = (uint8_t *)&rcvMsg;
bool gIsConfigChanged = FALSE;
bool gNeedSaveBackup = FALSE;
bool gIsStatusChanged = FALSE;
bool gResetRF = FALSE;
bool gResetNode = FALSE;
uint8_t _uniqueID[UNIQUE_ID_LEN];

// Moudle variables
uint8_t mStatus = SYS_INIT;
bool mGotNodeID = FALSE;
uint8_t mutex = 0;

// Keep Alive Timer
uint16_t mTimerKeepAlive = 0;
uint8_t m_cntRFReset = 0;
uint8_t m_cntRFSendFailed = 0;

// curtain operation min interval
#define CURTAIN_MIN_INTERVAL  50 // 500ms
#define KEYSIMULATOR_BUF      3  
Key_Simulator_t m_keySim[KEYSIMULATOR_BUF];  // to process short time of continuous key
uint8_t keySimBufLen = 0;
uint8_t wKeySimIdx = 0;
uint8_t rKeySimIdx = 0;
uint8_t mLastOpKeysimTime = 0;
uint8_t mLastAddKeysimTime = 0;

#ifdef EN_SENSOR_ALS
   uint16_t als_tick = 0;
#endif

#ifdef EN_SENSOR_ALS
   uint16_t mic_tick = 0;
#endif
   
#ifdef EN_SENSOR_PIR
   uint16_t pir_tick = 0;
#endif

#ifdef EN_SENSOR_IRKEY
   uint16_t irk_tick = 0;
#endif

#ifdef EN_SENSOR_PM25  
   uint16_t pm25_tick = 0;
#endif 

#ifdef EN_SENSOR_DHT       
   uint16_t dht_tem_tick = 0;
   uint16_t dht_collect_tick = 0;
#endif
   
#ifdef MULTI_SENSOR
#define MULTI_SENSOR_LIVE_TIMEOUT 18000 // 10ms unit(3min)
uint16_t ariquality_tick = 0;
uint16_t tem_hum_tick = 0;
#endif
uint16_t filter_tick = 0;

bool AddKeySimToBuf(u8 _target, const char *_keyString, u8 _len)
{
  bool add = TRUE;
  if(keySimBufLen >= KEYSIMULATOR_BUF) return FALSE;
  else
  {
    if(mLastAddKeysimTime < CURTAIN_MIN_INTERVAL)
    {
      uint8_t lastIdx = (wKeySimIdx+KEYSIMULATOR_BUF-1)%KEYSIMULATOR_BUF;
      if( _len == m_keySim[lastIdx].keyLen && memcmp(m_keySim[lastIdx].keySimulator,_keyString,_len) == 0)
      {
        printlog("ignore;");
        add = FALSE;
        return TRUE;
      }
    }
    if(add)
    {
      printlog("add;");
      mLastAddKeysimTime = 0;
      memset(&m_keySim[wKeySimIdx],0,sizeof(Key_Simulator_t));
      m_keySim[wKeySimIdx].keyLen = _len;
      memcpy(m_keySim[wKeySimIdx].keySimulator,_keyString,_len);
      m_keySim[wKeySimIdx].target = _target;
      wKeySimIdx = (wKeySimIdx+1)%KEYSIMULATOR_BUF;
      keySimBufLen++;
    }
  }
  return add;
}

void OpKeySimBuf()
{
  if(keySimBufLen >0)
  {
    if(mLastOpKeysimTime >= CURTAIN_MIN_INTERVAL)
    { // contain same relay operation, must be executed sequentially
      //printlog("");
      ProduceKeyOperation(m_keySim[rKeySimIdx].target, m_keySim[rKeySimIdx].keySimulator, m_keySim[rKeySimIdx].keyLen);
      //memset(&m_keySim[rKeySimIdx],0,sizeof(Key_Simulator_t));
      rKeySimIdx = (rKeySimIdx+1)%KEYSIMULATOR_BUF;
      keySimBufLen--;
      mLastOpKeysimTime = 0;
    }
  }
}

uint8_t *Read_UniqueID(uint8_t *UniqueID, uint16_t Length)  
{
  Flash_ReadBuf(UNIQUE_ID_ADDRESS, UniqueID, Length);
  return UniqueID;
}

bool isIdentityEmpty(const UC *pId, UC nLen)
{
  for( int i = 0; i < nLen; i++ ) { if(pId[i] > 0) return FALSE; }
  return TRUE;
}

bool isIdentityEqual(const UC *pId1, const UC *pId2, UC nLen)
{
  for( int i = 0; i < nLen; i++ ) { if(pId1[i] != pId2[i]) return FALSE; }
  return TRUE;
}

bool isNodeIdRequired()
{
  return( isIdentityEmpty(gConfig.NetworkID, ADDRESS_WIDTH) || isIdentityEqual(gConfig.NetworkID, RF24_BASE_RADIO_ID, ADDRESS_WIDTH) );
}

void UpdateNodeAddress(uint8_t _tx) {
  memcpy(rx_addr, gConfig.NetworkID, ADDRESS_WIDTH);
  rx_addr[0] = gConfig.nodeID;
  memcpy(tx_addr, gConfig.NetworkID, ADDRESS_WIDTH);
  if( _tx == NODEID_RF_SCANNER ) {
    tx_addr[0] = NODEID_RF_SCANNER;
  } else {
    tx_addr[0] = (isNodeIdRequired() ? BASESERVICE_ADDRESS : NODEID_GATEWAY);
  }
  RF24L01_setup(gConfig.rfChannel, gConfig.rfDataRate, gConfig.rfPowerLevel, BROADCAST_ADDRESS);
}

// reset rf
void ResetRFModule()
{
  if(gResetRF)
  {
    RF24L01_init();
    NRF2401_EnableIRQ();
    UpdateNodeAddress(NODEID_GATEWAY);
    gResetRF=FALSE;
  }
  if(gResetNode)
  {
    mStatus = SYS_RESET;
    gResetNode=FALSE;
  }
}

// Save config to Flash
void SaveBackupConfig()
{
  if( gNeedSaveBackup ) {
    // back config FLASH
    if(Flash_WriteDataBlock(BACKUP_CONFIG_BLOCK_NUM, (uint8_t *)&gConfig, sizeof(gConfig)))
    {
      gNeedSaveBackup = FALSE;
    }
  }
}

// Save status to Flash
void SaveStatusData()
{
  // status data contain nodeid subid and networkid
  if(gIsStatusChanged)
  {
    gNeedSaveBackup = TRUE;
    uint8_t pData[50] = {0};
    uint16_t nLen = (uint8_t *)(&gConfig.rfChannel) - (uint8_t *)(&gConfig);
    memcpy(pData, (uint8_t *)&gConfig, nLen);
    if(Flash_WriteDataBlock(STATUS_DATA_NUM, pData, nLen))
    {
      gIsStatusChanged = FALSE;
    }
  }
}

// Save config data to Flash(can't be called at working time)
void SaveConfig()
{
  if( gIsConfigChanged ) {
    gNeedSaveBackup = TRUE;
    // Overwrite entire config FLASH 
    uint8_t Attmpts = 0;
    while(++Attmpts <= 3) {
      if(Flash_WriteDataBlock(0, (uint8_t *)&gConfig, sizeof(gConfig)))
      {
        gIsConfigChanged = FALSE;
        break;
      }
    }
  }
  if(!gIsConfigChanged)
  { // ensure rf info is up to date
    ResetRFModule();
  }
  SaveStatusData();
}

// Initialize Node Address and look forward to being assigned with a valid NodeID by the SmartController
void InitNodeAddress() {
  // Whether has preset node id
  gConfig.nodeID = XLA_PRODUCT_NODEID;
  memcpy(gConfig.NetworkID, RF24_BASE_RADIO_ID, ADDRESS_WIDTH);
}

bool IsStatusInvalid() {
  // gConfig.aircondStatus[2] > 32 temp more than 32
  return( gConfig.version > XLA_VERSION || gConfig.version < XLA_MIN_VER_REQUIREMENT);
}

bool IsConfigInvalid() {
  return( gConfig.version > XLA_VERSION || gConfig.version < XLA_MIN_VER_REQUIREMENT 
       || gConfig.nodeID == 0 || gConfig.type != XLA_PRODUCT_Type
       || gConfig.rfPowerLevel > RF24_PA_MAX || gConfig.rfChannel > 127 || gConfig.rfDataRate > RF24_250KBPS );
}

bool isNodeIdInvalid()
{
  return( gConfig.nodeID != XLA_PRODUCT_NODEID  );
}
// Load config from Flash
void LoadConfig()
{
      // Load the config area
  Flash_ReadBuf(FLASH_DATA_START_PHYSICAL_ADDRESS, (uint8_t *)&gConfig, sizeof(gConfig));
  uint16_t nStatusLen = (uint8_t *)(&gConfig.nodeID) - (uint8_t *)(&gConfig);
  if( IsConfigInvalid() ) {
    // If config isn't OK, then try to load config from backup area
    Flash_ReadBuf(BACKUP_CONFIG_ADDRESS+nStatusLen, (uint8_t *)&gConfig.nodeID, sizeof(gConfig)-nStatusLen);
    bool backupInvalid = IsConfigInvalid();
    InitNodeAddress();
    if( backupInvalid ) {
      // If neither valid, then initialize config with default settings
        memset(&gConfig, 0x00, sizeof(gConfig));
        gConfig.version = XLA_VERSION;
        InitNodeAddress();
        gConfig.subID = 0;
        gConfig.type = XLA_PRODUCT_Type;
        gConfig.rptTimes = 1;
        gConfig.rfChannel = RF24_CHANNEL;
        gConfig.rfPowerLevel = RF24_PA_MAX;
        gConfig.rfDataRate = RF24_250KBPS;
        
#ifdef EN_SENSOR_ALS
        gConfig.senMap |= sensorALS;
#endif
#ifdef EN_SENSOR_MIC
        gConfig.senMap |= sensorMIC;
#endif
#ifdef EN_SENSOR_PIR
        gConfig.senMap |= sensorPIR;
#endif
#ifdef EN_SENSOR_IRKEY
        gConfig.senMap |= sensorIRKey;
#endif      
#ifdef EN_SENSOR_DHT
        gConfig.senMap |= sensorDHT;
#endif
#ifdef EN_SENSOR_PM25
        gConfig.senMap |= sensorDUST;
#endif
    }
    gIsConfigChanged = TRUE;
    SaveConfig();
  } else {
    uint8_t bytVersion;
    Flash_ReadBuf(BACKUP_CONFIG_ADDRESS, (uint8_t *)&bytVersion, sizeof(bytVersion));
    if( bytVersion != gConfig.version ) gNeedSaveBackup = TRUE;
  }
  // Load the most recent status from FLASH
  uint8_t pData[50];
  memset(pData,0x00,sizeof(pData));
  uint16_t nLen = (uint8_t *)(&gConfig.rfChannel) - (uint8_t *)(&gConfig);
  Flash_ReadBuf(STATUS_DATA_ADDRESS, pData, nLen);
  if(pData[0] >= XLA_MIN_VER_REQUIREMENT && pData[0] <= XLA_VERSION)
  { // status data valid    
    memcpy(&gConfig,pData,nStatusLen);
    if(isIdentityEqual(gConfig.NetworkID, RF24_BASE_RADIO_ID, ADDRESS_WIDTH) && !isNodeIdInvalid() )
    { // valid nodeid but with default network config,can covered by status data or back data if they are valid
      uint16_t networkOffset = (uint8_t *)(&gConfig.NetworkID) - (uint8_t *)(&gConfig);
      if( !isIdentityEmpty(pData+networkOffset,sizeof(gConfig.NetworkID)) )
      {
        memcpy(gConfig.NetworkID,pData+networkOffset,sizeof(gConfig.NetworkID));
      } 
    } 
  }
  else
  { // load backup area for status data
    Flash_ReadBuf(BACKUP_CONFIG_ADDRESS, pData, nLen);
    if(pData[0] >= XLA_MIN_VER_REQUIREMENT && pData[0] <= XLA_VERSION)
    { // status data valid 
      memcpy(&gConfig,pData,nStatusLen);
      if(isIdentityEqual(gConfig.NetworkID, RF24_BASE_RADIO_ID, ADDRESS_WIDTH) && !isNodeIdInvalid())
      { // valid nodeid but with default network config,can covered by status data or back data if they are valid
        uint16_t networkOffset = (uint8_t *)(&gConfig.NetworkID) - (uint8_t *)(&gConfig);
        if( !isIdentityEmpty(pData+networkOffset,sizeof(gConfig.NetworkID)) )
        {
          memcpy(gConfig.NetworkID,pData+networkOffset,sizeof(gConfig.NetworkID));
        }        
      }
    }
  }
 
  if(IsStatusInvalid())
  {
    // default status value
    gConfig.version = XLA_VERSION;
    gConfig.state = 1;
  }
    // Engineering code
  if(XLA_PRODUCT_Type == ZEN_TARGET_SUPERSENSOR)
  {
    gConfig.senMap |= sensorALS;
    gConfig.senMap |= sensorMIC;
    gConfig.senMap |= sensorDHT;
    gConfig.senMap |= sensorDUST;
    gConfig.senMap |= sensorIRKey;
  }
  if(XLA_PRODUCT_Type == ZEN_TARGET_SPOTLIGHT)
  {
    gConfig.subID = 4;
  }
  if(XLA_PRODUCT_Type == ZEN_TARGET_AIRCONDITION)
  {
    gConfig.subID = 1;
  }
  if(XLA_PRODUCT_Type == ZEN_TARGET_AIRPURIFIER)
  {
    gConfig.subID = 2;
  }
  if(XLA_PRODUCT_Type == ZEN_TARGET_CURTAIN)
  {
    gConfig.subID = 8;
  }
#ifdef EN_PANEL_BUTTONS
    if( gConfig.btnAction[0][0].action > 0x0F || gConfig.btnAction[1][0].action > 0x0F ) {
      memset(gConfig.btnAction, 0x00, sizeof(Button_Action_t) * MAX_NUM_BUTTONS);
    }
#endif    
}

bool WaitMutex(uint32_t _timeout) {
  while(_timeout--) {
    if( mutex > 0 ) return TRUE;
    feed_wwdg();
  }
  return FALSE;
}

bool NeedUpdateRFAddress(uint8_t _dest) {
  bool rc = FALSE;
  if( sndMsg.header.destination == NODEID_RF_SCANNER && tx_addr[0] != NODEID_RF_SCANNER ) {
    UpdateNodeAddress(NODEID_RF_SCANNER);
    rc = TRUE;
  } else if( sndMsg.header.destination != NODEID_RF_SCANNER && tx_addr[0] != NODEID_GATEWAY ) {
    UpdateNodeAddress(NODEID_GATEWAY);
    rc = TRUE;
  }
  return rc;
}

// Send message and switch back to receive mode
bool SendMyMessage() {
  if( bMsgReady ) {
    
    // Change tx destination if necessary
    NeedUpdateRFAddress(sndMsg.header.destination);
      
    uint8_t lv_tried = 0;
    uint16_t delay;
    while (lv_tried++ <= gConfig.rptTimes ) {
      
      mutex = 0;
      if(RF24L01_set_mode_TX_timeout() == -1) 
        break;
      if(RF24L01_write_payload_timeout(psndMsg, PLOAD_WIDTH) == -1) 
        break;
      WaitMutex(0x1FFFF);
      if (mutex == 1) {
        m_cntRFSendFailed = 0;
        m_cntRFReset = 0;
        break; // sent sccessfully
      } else {
        m_cntRFSendFailed++;
        if( m_cntRFSendFailed >= MAX_RF_FAILED_TIME ) {
          m_cntRFSendFailed = 0;
          m_cntRFReset++;
          if( m_cntRFReset >= 3 ) {
            // Cold Reset
            if(XLA_PRODUCT_Type!=ZEN_TARGET_SPOTLIGHT)
            {
              WWDG->CR = 0x80;
            }         
            m_cntRFReset = 0;
            break;
          } else if( m_cntRFReset >= 2 ) {
            // Reset whole node
            mStatus = SYS_RESET;
            break;
          }

          // Reset RF module
          //RF24L01_DeInit();
          delay = 0x1FFF;
          while(delay--)feed_wwdg();
          RF24L01_init();
          NRF2401_EnableIRQ();
          UpdateNodeAddress(NODEID_GATEWAY);
          continue;
        }
      }
      
      //The transmission failed, Notes: mutex == 2 doesn't mean failed
      //It happens when rx address defers from tx address
      //asm("nop"); //Place a breakpoint here to see memory
      // Repeat the message if necessary
      delay = 0xFFF;
      while(delay--)feed_wwdg();
    }
    
    // Switch back to receive mode
    bMsgReady = 0;
    RF24L01_set_mode_RX();
    
    // Reset Keep Alive Timer
    mTimerKeepAlive = 0;
  }

  return(mutex > 0);
}

void GotNodeID() {
  mGotNodeID = TRUE;
  UpdateNodeAddress(NODEID_GATEWAY);
  gNeedSaveBackup = TRUE;
}

void GotPresented() {
  mStatus = SYS_RUNNING;
  gConfig.swTimes = 0;
  gIsStatusChanged = TRUE;
}

bool SayHelloToDevice(bool infinate) {
  uint8_t _count = 0;
  uint8_t _presentCnt = 0;
  bool _doNow = FALSE;

  // Update RF addresses and Setup RF environment
  UpdateNodeAddress(NODEID_GATEWAY);

  while(mStatus < SYS_RUNNING) {
    ////////////rfscanner process///////////////////////////////
    ProcessOutputCfgMsg(); 
    SendMyMessage();
    ResetRFModule();
    SaveConfig();
    ////////////rfscanner process/////////////////////////////// 
    if( _count++ == 0 ) {
      
      if( isNodeIdRequired() ) {
        mStatus = SYS_WAIT_NODEID;
        mGotNodeID = FALSE;
        // Request for NodeID
        Msg_RequestNodeID();
      } else {
        mStatus = SYS_WAIT_PRESENTED;
        // Send Presentation Message
        Msg_Presentation();
        _presentCnt++;
#ifdef RAPID_PRESENTATION
        // Don't wait for ack
        mStatus = SYS_RUNNING;
#endif       
      }
           
      if( !SendMyMessage() ) {
        if( !infinate ) return FALSE;
      } else {
        // Wait response
        uint16_t tick = 0xBFFF;
        while(tick-- && mStatus < SYS_RUNNING) {
          // Feed the Watchdog
          feed_wwdg();
          if( mStatus == SYS_WAIT_NODEID && mGotNodeID ) {
            mStatus = SYS_WAIT_PRESENTED;
            _presentCnt = 0;
            _doNow = TRUE;
            break;
          }
        }
      }
    }

    if( mStatus == SYS_RUNNING ) return TRUE;
    
    // Can't presented for a few times, then try request NodeID again
    // Either because SmartController is off, or changed
    if(  mStatus == SYS_WAIT_PRESENTED && _presentCnt >= REGISTER_RESET_TIMES && REGISTER_RESET_TIMES < 100 ) {
      _presentCnt = 0;
      // Reset RF Address
      InitNodeAddress();
      UpdateNodeAddress(NODEID_GATEWAY);
      mStatus = SYS_WAIT_NODEID;
      _doNow = TRUE;
    }
    
    // Reset switch count
    if( _count >= 10 && gConfig.swTimes > 0 ) {
      gConfig.swTimes = 0;
      gIsStatusChanged = TRUE;
      SaveConfig();
    }
    
    // Feed the Watchdog
    feed_wwdg();

    if( _doNow ) {
      // Send Message Immediately
      _count = 0;
      continue;
    }
    
    // Failed or Timeout, then repeat init-step
    //delay_ms(400);
    mutex = 0;
    WaitMutex(0x1FFFF);
    _count %= 20;  // Every 10 seconds
  }
  
  return TRUE;
}

int main( void ) {

#ifdef EN_SENSOR_ALS
   uint8_t pre_als_value = 0;
#endif

#ifdef EN_SENSOR_MIC
   uint16_t pre_mic_value = 0;
#endif
   
#ifdef EN_SENSOR_PIR
   bool pre_pir_st = FALSE;
   bool pir_st;
#endif
   
#ifdef EN_SENSOR_IRKEY
   uint8_t pre_irk_st = 0xFF;
   uint8_t irk_st;
#endif

#ifdef EN_SENSOR_PM25       
   uint16_t lv_pm2_5 = 0;
   uint8_t pm25_alivetick = 0;
#endif   

#ifdef EN_SENSOR_DHT
   uint16_t pre_dht_t = 0;
   uint16_t pre_dht_h = 0;
#endif   
 
#ifdef MULTI_SENSOR
   uint16_t pre_pm25 = 0;
   uint16_t pre_pm10 = 0;
   uint16_t pre_tvoc = 0;
   uint16_t pre_ch2o = 0;
   uint16_t pre_co2 = 0;
   int16_t pre_tem = 0;
   int16_t pre_hum = 0;
#endif
   memset(&m_keySim,0x00,sizeof(Key_Simulator_t)*KEYSIMULATOR_BUF); 
   
  //After reset, the device restarts by default with the HSI clock divided by 8.
  //CLK_DeInit();
  /* High speed internal clock prescaler: 1 */
  CLK_SYSCLKConfig(CLK_PRESCALER_HSIDIV1);  // now: HSI=16M prescale = 1; sysclk = 16M

  // Load config from Flash
  FLASH_DeInit();
  Read_UniqueID(_uniqueID, UNIQUE_ID_LEN);
  LoadConfig();
  // add for colorful light bar
  gConfig.btnAction[0][0].action = 0x08;
  gConfig.btnAction[0][0].keyMap = 15;

  // on / off 3 times to reset device
  gConfig.swTimes++;
  if( gConfig.swTimes >= ONOFF_RESET_TIMES ) {
    gConfig.swTimes = 0;
    InitNodeAddress();
  }
  
  // Init Watchdog
  wwdg_init();
  
  gIsStatusChanged = TRUE;
  SaveConfig();
  
  // Init sensors
#ifdef EN_SENSOR_ALS || EN_SENSOR_MIC
  ADC1_PinInit();
#endif
#ifdef EN_SENSOR_PIR
  pir_init();
#endif
#ifdef EN_SENSOR_IRKEY
  irk_init();
#endif
#ifdef EN_SENSOR_PM25
  pm25_init();
#endif
  
#ifdef EN_SENSOR_ALS || EN_SENSOR_MIC  
  // Init ADC
  ADC1_Config();
#endif 
 
#ifdef MULTI_SENSOR 
  multi_init();
#endif
  
  
#ifdef DEBUG_LOG
#if (!defined EN_SENSOR_PM25) && (!defined MULTI_SENSOR)
  // Init serial ports
  uart2_config(9600);
#endif
#endif

  printlog("start...\r\n");
  // Init timer
  TIM4_10ms_handler = tmrProcess;
  Time4_Init();
  
#ifdef EN_SENSOR_DHT
  DHT_init();
#else
#ifdef EN_INFRARED 
  Infrared_Init();
#endif
#endif  

#ifdef EN_PANEL_BUTTONS
  button_init();
#endif  
  keySimulator_init();
  relay_key_init(); 
  if(IS_TARGET_AIRPURIFIER(gConfig.type) )
  { // AIRPURIFIER restore
    relay_restore_keystates();
  }
#ifdef TEST
   testio();
#endif
  while(1) {
    // Go on only if NRF chip is presented
    disableInterrupts();
    gConfig.present = 0;
    RF24L01_init();
    u16 timeoutRFcheck = 0;
    while(!NRF24L01_Check()) {
      if( timeoutRFcheck > 50 ) {
        WWDG->CR = 0x80;
        break;
      }
      feed_wwdg();
    }
    // IRQ
    NRF2401_EnableIRQ();
    // Must establish connection firstly
    SayHelloToDevice(TRUE);  

    while (mStatus == SYS_RUNNING) {
      
      // Feed the Watchdog
      feed_wwdg();
      
#ifdef EN_INFRARED
      IR_Send();
#endif
      
      if( gConfig.state ) {
        
        // Read sensors
#ifdef EN_SENSOR_PIR
        /// Read PIR
        if( gConfig.senMap & sensorPIR ) {
          if( !bMsgReady && pir_tick > SEN_READ_PIR ) {
            pir_st = pir_read();
            if( pre_pir_st != pir_st || pir_tick > SEN_MAX_SEND_INTERVAL ) {
              // Reset read timer
              pir_tick = 0;
              // Send detection message
              pre_pir_st = pir_st;
              Msg_SenPIR(pre_pir_st);
            }
          }
        }
#endif

#ifdef EN_SENSOR_IRKEY
        /// Read IRKeys
        if( gConfig.senMap & sensorIRKey ) {
          if( !bMsgReady && irk_tick > SEN_READ_IRKEY ) {
            irk_st = irk_read();
            if( pre_irk_st != irk_st || irk_tick > SEN_MAX_SEND_INTERVAL ) {
              // Reset read timer
              irk_tick = 0;
              // Send detection message
              pre_irk_st = irk_st;
              Msg_SenIRKey(pre_irk_st);
            }
          }
        }
#endif
        
#ifdef EN_SENSOR_ALS
        /// Read ALS
        if( gConfig.senMap & sensorALS ) {
          if( !bMsgReady && als_tick > SEN_READ_ALS ) {
            if( als_ready ) {
              if( pre_als_value != als_value || als_tick > SEN_MAX_SEND_INTERVAL ) {
                // Reset read timer
                als_tick = 0;
                // Send brightness message
                pre_als_value = als_value;
                Msg_SenALS(pre_als_value);
              }
            }
          }
        }
#endif

#ifdef EN_SENSOR_MIC
        /// Read MIC
        if( gConfig.senMap & sensorMIC ) {
          if( !bMsgReady && mic_tick > SEN_READ_MIC ) {
            if( mic_ready ) {
              if( pre_mic_value != mic_value || mic_tick > SEN_MAX_SEND_INTERVAL ) {
                // Reset read timer
                mic_tick = 0;
                // Send brightness message
                pre_mic_value = mic_value;
                Msg_SenMIC(pre_mic_value);
              }
            }
          }
        }
#endif
        
#ifdef EN_SENSOR_PM25
        if( gConfig.senMap & sensorDUST ) {
          if( !bMsgReady && pm25_tick > SEN_READ_PM25 ) {
            if( pm25_ready ) {
              if( lv_pm2_5 != pm25_value || pm25_tick > SEN_MAX_SEND_INTERVAL ) {
                // Reset read timer
                pm25_tick = 0;
                lv_pm2_5 = pm25_value;
                if( lv_pm2_5 < 5 ) lv_pm2_5 = 8;
                // Send PM2.5 to Controller
                Msg_SenPM25(lv_pm2_5);
              } else if( pm25_alive ) {
                pm25_alive = FALSE;
                pm25_alivetick = 20;
              } else if( --pm25_alivetick == 0 ) {
                // Reset PM2.5 moudle or restart the node
                mStatus = SYS_RESET;
                pm25_init();
              }
            }
          }
        }
#endif
        
#ifdef EN_SENSOR_DHT
        /// Read DHT
        if( gConfig.senMap & sensorDHT ) {
          // Collect Data
          if( dht_collect_tick >= SEN_COLLECT_DHT ) {
            dht_collect_tick = 0;
            DHT_checkData();
          }
          // Read & Send Data
          if( !bMsgReady && dht_tem_tick > SEN_READ_DHT ) {
            if( dht_tem_ready || dht_hum_ready ) {
              if( (dht_tem_ready && pre_dht_t != dht_tem_value) || (dht_hum_ready && pre_dht_h != dht_hum_value) || dht_tem_tick > SEN_MAX_SEND_INTERVAL ) {
                dht_tem_tick = 0;
                if( dht_tem_ready && dht_hum_ready ) {
                  pre_dht_t = dht_tem_value;
                  pre_dht_h = dht_hum_value;
                  Msg_SenDHT(dht_tem_value,dht_hum_value, 0);
                } else if( dht_tem_ready ) {
                  pre_dht_t = dht_tem_value;
                  Msg_SenDHT(dht_tem_value,dht_hum_value, 1);  
                } else {
                  pre_dht_h = dht_hum_value;
                  Msg_SenDHT(dht_tem_value,dht_hum_value, 2);  
                }
              }
            }
          }
        }
#endif
        
#ifdef MULTI_SENSOR
        PraseMultiSensorMsg();
        if(multi_sensor_alive_tick >= MULTI_SENSOR_LIVE_TIMEOUT)
        {
            // Reset multi sensor moudle or restart the node
            mStatus = SYS_RESET;
            multi_init();
        }
        if( !bMsgReady && ariquality_tick > SEN_READ_PM25 ) {
          if(ariquality_tick > SEN_MAX_SEND_INTERVAL )
          {
            ariquality_tick = 0;
            Msg_SenAirQuality(pre_pm25,pre_pm10,pre_tvoc,pre_ch2o,pre_co2);
          }
          else if( pre_pm25 != pm25_value || pre_pm10 != pm10_value || pre_tvoc != tvoc_value || pre_ch2o != ch2o_value || pre_co2 != co2_value)
          {
            ariquality_tick = 0;
            pre_pm25 = pm25_value;
            pre_pm10 = pm10_value;
            pre_tvoc = tvoc_value;
            pre_ch2o = ch2o_value;
            pre_co2 = co2_value;
            // Send to Controller
            Msg_SenAirQuality(pre_pm25,pre_pm10,pre_tvoc,pre_ch2o,pre_co2);
          }
        }
        // tem & hum
        if( !bMsgReady && tem_hum_tick > SEN_READ_DHT ) {
            if( pre_tem != tem_value|| pre_hum != hum_value || tem_hum_tick > SEN_MAX_SEND_INTERVAL ) {
              tem_hum_tick = 0;
              pre_tem = tem_value;
              pre_hum = hum_value;
              Msg_SenTemHum(tem_value,hum_value);  
            }
        }
#endif
        // Idle Tick
        if( !bMsgReady ) {
          // Check Keep Alive Timer
          if( mTimerKeepAlive > RTE_TM_KEEP_ALIVE ) {
            Msg_Relay_KeyMap(NODEID_GATEWAY);
          }
        }
      } // End of if( gConfig.state )  
      ////////////rfscanner process///////////////////////////////
      ProcessOutputCfgMsg(); 
      // reset rf
      ResetRFModule();
      ////////////rfscanner process/////////////////////////////// 
      if(gConfig.filterTime>0 && filter_tick>=gConfig.filterTime*100)
      {
        filter_tick = 0;
        uint8_t relay = (gConfig.relay_key_value+1)%15+1;
        relay_set_keys(relay);
      }
      // Send message if ready
#ifdef TEST
      //PB4_High;
#endif
      SendMyMessage();
#ifdef TEST
      //PB4_Low;
#endif
      // Save Config if Changed
      SaveConfig();
      //OpKeySimBuf();
      
      // ToDo: Check heartbeats
      // mStatus = SYS_RESET, if timeout or received a value 3 times consecutively
    }
  }
}

// Execute timer operations
void tmrProcess() {
  // Tick
  mTimerKeepAlive++;
#ifdef EN_SENSOR_ALS
  als_tick++;
  if( als_tick % 10 == 0) als_checkData();
#ifdef EN_SENSOR_MIC
  mic_tick++;
  if( mic_tick % 10 == 2) mic_checkData();
#endif
#endif
#ifdef EN_SENSOR_PIR
  pir_tick++;
#endif
#ifdef EN_SENSOR_IRKEY
  irk_tick++;
#endif  
#ifdef EN_SENSOR_PM25
  pm25_tick++;
#endif
#ifdef EN_SENSOR_DHT
  dht_tem_tick++;
  dht_collect_tick++;
#endif
  
#ifdef EN_INFRARED
  // Ir-send timer count down
  if( ir_send_delay > 0 ) ir_send_delay--;
#endif  

#if (defined ZENREMOTE) && (!defined EN_PANEL_BUTTONS)
  if(IS_TARGET_CURTAIN(gConfig.type)) 
  {
    if(mLastOpKeysimTime < CURTAIN_MIN_INTERVAL)
      mLastOpKeysimTime++;
    if(mLastAddKeysimTime < CURTAIN_MIN_INTERVAL)
      mLastAddKeysimTime++;
  }
  OpKeySimBuf();
  // Send Keys
  for( u8 i = 0; i < KEY_OP_MAX_BUFFERS; i++ ) {
    if( gKeyBuf[i].keyNum > 0 ) {
      {
        ScanKeyBuffer(i);
      }
    }
  }
#endif
  
#ifdef EN_PANEL_BUTTONS  
  //////zql add for relay key//////////////
  if(relay_loop_tick < 5000) relay_loop_tick++;
  //////zql add for relay key//////////////
  if(filter_tick < gConfig.filterTime*100) filter_tick++;
#endif  

#ifdef MULTI_SENSOR
  multi_sensor_alive_tick++;
  ariquality_tick++;
  tem_hum_tick++;
#endif
  
    // Save config into backup area
   SaveBackupConfig();
  
}

INTERRUPT_HANDLER(EXTI_PORTC_IRQHandler, 5) {

  if(RF24L01_is_data_available()) {
#ifdef TEST
  PB3_High;
#endif
    //Packet was received
    RF24L01_clear_interrupts();
    RF24L01_read_payload(prcvMsg, PLOAD_WIDTH);
    bMsgReady = ParseProtocol();
#ifdef TEST
    PB3_Low;
#endif
    return;
  }
 
  uint8_t sent_info;
  if (sent_info = RF24L01_was_data_sent()) {
#ifdef TEST
  PB4_High;
#endif
    //Packet was sent or max retries reached
    RF24L01_clear_interrupts();
    mutex = sent_info;
#ifdef TEST
    PB4_Low;
#endif    
    return;
  }

   RF24L01_clear_interrupts();
}
