#ifndef __GLOBAL_H
#define __GLOBAL_H

#include <stm8s.h> //Required for the stdint typedefs
#include "stdio.h"
#include "string.h"
#include "stm8s_conf.h"

/* Exported types ------------------------------------------------------------*/
/// Comment off line to disable panel buttons(spotlight need)
//#define EN_PANEL_BUTTONS
/// Comment off line to disable infrared(aircondition need)
//#define EN_INFRARED
/// Comment off line to disable Relay key input
//#define EN_SENSOR_IRKEY
// Notes: EN_PANEL_BUTTONS & EN_SENSOR_IRKEY can't exist at the same time
//#ifdef EN_PANEL_BUTTONS
//#undef EN_SENSOR_IRKEY
//#endif

#ifdef ZENSENSOR
// Include Sensors
/// Comment off line to disable sensor
//#define EN_SENSOR_ALS
//#define EN_SENSOR_MIC
//#define EN_SENSOR_PIR
//#define MULTI_SENSOR
//#ifndef MULTI_SENSOR
//#define EN_SENSOR_DHT
//#define EN_SENSOR_PM25
//#define EN_SENSOR_MQ135
//#define EN_SENSOR_MQ2
//#define EN_SENSOR_MQ7
//#endif
#endif

#ifdef ZENREMOTE
#undef EN_SENSOR_DHT
#endif

//#define DEBUG_LOG

// Common Data Type
#define UC                        uint8_t
#define US                        uint16_t
#define UL                        uint32_t
#define SHORT                     int16_t
#define LONG                      int32_t

// Switch value for set power command
#define DEVICE_SW_OFF               0       // Turn Off
#define DEVICE_SW_ON                1       // Turn On
#define DEVICE_SW_TOGGLE            2       // Toggle
#define DEVICE_SW_DUMMY             3       // Detail followed

// Node type
#define NODE_TYP_GW               'g'
#define NODE_TYP_LAMP             'l'
#define NODE_TYP_REMOTE           'r'
#define NODE_TYP_SYSTEM           's'
#define NODE_TYP_THIRDPARTY       't'

// NodeID Convention
#define NODEID_GATEWAY          0
#define NODEID_MAINDEVICE       1
#define NODEID_MIN_DEVCIE       8
#define NODEID_MAX_DEVCIE       63
#define NODEID_MIN_REMOTE       64
#define NODEID_MAX_REMOTE       127
#define NODEID_PROJECTOR        128
#define NODEID_KEYSIMULATOR     129
#define NODEID_SUPERSENSOR      130
#define NODEID_SMARTPHONE       139
#define NODEID_MIN_GROUP        192
#define NODEID_MAX_GROUP        223
#define NODEID_RF_SCANNER       250
#define NODEID_DUMMY            255
#define BASESERVICE_ADDRESS     0xFE
#define BROADCAST_ADDRESS       0xFF

#define UNIQUE_ID_LEN           8
#define MAX_NUM_BUTTONS         4

// Target Type (mask)
#define ZEN_TARGET_CURTAIN      0x80
#define ZEN_TARGET_AIRPURIFIER  0x90
#define ZEN_TARGET_AIRCONDITION 0xA0
#define ZEN_TARGET_SPOTLIGHT    0xB0
#define ZEN_TARGET_SUPERSENSOR  0xC0

// I_GET_NONCE sub-type
enum {
    SCANNER_PROBE = 0,
    SCANNER_SETUP_RF,           // by NodeID & SubID
    SCANNER_SETUPDEV_RF,        // by UniqueID
    
    SCANNER_GETCONFIG = 8,      // by NodeID & SubID
    SCANNER_SETCONFIG,
    SCANNER_GETDEV_CONFIG,      // by UniqueID
    SCANNER_SETDEV_CONFIG,
    
    SCANNER_TEST_NODE = 16,     // by NodeID & SubID
    SCANNER_TEST_DEVICE,        // by UniqueID
};

typedef struct
{
  UC action;                                // Type of action
  UC keyMap;                                // Button Key Map: 8 bits for each button, one bit corresponds to one relay key
} Button_Action_t;

typedef struct
{
  uint8_t target;                            // target
  uint8_t keyLen;                            // KeySimulator length
  UC keySimulator[15];                       // string of keysimulator
} Key_Simulator_t;


// Xlight Application Identification
#define XLA_VERSION               0x08
#define XLA_ORGANIZATION          "xlight.ca"               // Default value. Read from EEPROM

#if XLA_VERSION > 0x07
#define XLA_MIN_VER_REQUIREMENT   0x08
typedef struct
{
  // Static & status parameters
  UC version                  :8;           // Data version, other than 0xFF
  UC present                  :1;           // 0 - not present; 1 - present
  UC state                    :1;           // SuperSensor On/Off
  UC swTimes                  :4;           // On/Off times
  UC reserved0                :2;
  UC relay_key_value          :8;           // Relay Key Bitmap
#ifdef EN_INFRARED  
  UC aircondition_on_status[20];            // aircondition last on status
#endif 

  // Configurable parameters
  UC nodeID;                                // Node ID for this device
  UC subID;                                 // SubID
  UC NetworkID[6];
  UC rfChannel;                             // RF Channel: [0..127]
  UC rfPowerLevel             :2;           // RF Power Level 0..3
  UC rfDataRate               :2;           // RF Data Rate [0..2], 0 for 1Mbps, or 1 for 2Mbps, 2 for 250kbs
  UC rptTimes                 :2;           // Sending message max repeat times [0..3]
  UC reserved1                :2;
  UC type;                                  // Type of SuperSensor
  US token;
  UC reserved2                :8;
  US senMap                   :16;          // Sensor Map
#ifdef EN_PANEL_BUTTONS  
  Button_Action_t btnAction[MAX_NUM_BUTTONS][8];
#endif  
} Config_t;
#else
#define XLA_MIN_VER_REQUIREMENT   0x03
typedef struct
{
  UC version                  :8;           // Data version, other than 0xFF
  UC nodeID;                                // Node ID for this device
  UC NetworkID[6];
  UC present                  :1;           // 0 - not present; 1 - present
  UC state                    :1;           // SuperSensor On/Off
  UC reserved                 :6;
  UC subID;                                 // SubID
  UC type;                                  // Type of SuperSensor
  US token;
  //char Organization[24];                    // Organization name
  //char ProductName[24];                     // Product name
  UC rfPowerLevel             :2;           // RF Power Level 0..3
  UC swTimes                  :3;           // On/Off times
  UC rptTimes                 :2;           // Sending message max repeat times [0..3]
  UC reserved1                :1;
  US senMap                   :16;          // Sensor Map
  UC relay_key_value          :8;           // Relay Key Bitmap
  UC rfChannel;                             // RF Channel: [0..127]
  UC rfDataRate               :2;           // RF Data Rate [0..2], 0 for 1Mbps, or 1 for 2Mbps, 2 for 250kbs
#ifdef EN_PANEL_BUTTONS  
  Button_Action_t btnAction[MAX_NUM_BUTTONS][8];
#endif  
} Config_t;
#endif

extern Config_t gConfig;
extern bool gIsChanged;
extern bool gNeedSaveBackup;
extern bool gIsStatusChanged;
extern bool gResetRF;
extern bool gResetNode;
extern uint8_t _uniqueID[UNIQUE_ID_LEN];

void printlog(uint8_t *pBuf);
void printnum(unsigned int num);
bool isIdentityEqual(const UC *pId1, const UC *pId2, UC nLen);
void GotNodeID();
void GotPresented();
bool SendMyMessage();
void tmrProcess();
void relay_gpio_write_bit(GPIO_TypeDef* GPIOx, GPIO_Pin_TypeDef PortPins, bool _on);
bool AddKeySimToBuf(u8 _target, const char *_keyString, u8 _len);
void ProcessMyMessage();

#define IS_MINE_SUBID(nSID)             ((nSID) == 0 || ((nSID) & gConfig.subID))
#define IS_TARGET_CURTAIN(nTag)         (((nTag) & 0xF0) == ZEN_TARGET_CURTAIN)
#define IS_TARGET_AIRPURIFIER(nTag)     (((nTag) & 0xF0) == ZEN_TARGET_AIRPURIFIER)
#define IS_TARGET_AIRCONDITION(nTag)    (((nTag) & 0xF0) == ZEN_TARGET_AIRCONDITION)
#define IS_TARGET_SPOTLIGHT(nTag)       (((nTag) & 0xF0) == ZEN_TARGET_SPOTLIGHT)
#define IS_TARGET_SUPERSENSOR(nTag)     (((nTag) & 0xF0) == ZEN_TARGET_SUPERSENSOR)

// Choose Product Name & Type
#ifdef ZENSENSOR
#define XLA_PRODUCT_NAME          "ZENSENSOR"
#define XLA_PRODUCT_Type          ZEN_TARGET_SUPERSENSOR
#define XLA_PRODUCT_NODEID        NODEID_SUPERSENSOR
#else
#define XLA_PRODUCT_NAME          "ZENREMOTE"
#define XLA_PRODUCT_Type          ZEN_TARGET_AIRPURIFIER
#define XLA_PRODUCT_NODEID        NODEID_KEYSIMULATOR
#endif

//#define TEST
#ifdef TEST
#define     PB5_Low                GPIO_WriteLow(GPIOB , GPIO_PIN_5)
#define     PB4_Low                GPIO_WriteLow(GPIOB , GPIO_PIN_4)
#define     PB3_Low                GPIO_WriteLow(GPIOB , GPIO_PIN_3)
#define     PB2_Low                GPIO_WriteLow(GPIOB , GPIO_PIN_2)
#define     PB1_Low                GPIO_WriteLow(GPIOB , GPIO_PIN_1)
#define     PD1_Low                GPIO_WriteLow(GPIOD , GPIO_PIN_1)
#define     PD2_Low                GPIO_WriteLow(GPIOD , GPIO_PIN_2)
#define     PD7_Low                GPIO_WriteLow(GPIOD , GPIO_PIN_7)
#define     PB5_High                GPIO_WriteHigh(GPIOB , GPIO_PIN_5)
#define     PB4_High                GPIO_WriteHigh(GPIOB , GPIO_PIN_4)
#define     PB3_High                GPIO_WriteHigh(GPIOB , GPIO_PIN_3)
#define     PB2_High                GPIO_WriteHigh(GPIOB , GPIO_PIN_2)
#define     PB1_High                GPIO_WriteHigh(GPIOB , GPIO_PIN_1)
#define     PD1_High                GPIO_WriteHigh(GPIOD , GPIO_PIN_1)
#define     PD2_High                GPIO_WriteHigh(GPIOD , GPIO_PIN_2)
#define     PD7_High                GPIO_WriteHigh(GPIOD , GPIO_PIN_7)
#endif

#endif /* __GLOBAL_H */
