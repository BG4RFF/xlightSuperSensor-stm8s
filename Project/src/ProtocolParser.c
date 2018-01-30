#include "ProtocolParser.h"
#include "_global.h"
#include "MyMessage.h"
#include "relay_key.h"
#include "keySimulator.h"
#include "xliNodeConfig.h"
#include "infrared.h"
#include "rf24l01.h"

uint8_t bMsgReady = 0;
void Process_SetConfig(u8 _len);
void Process_SetDevConfig(u8 _len);
void MsgScanner_ProbeAck() ;
void MsgScanner_ConfigAck(uint8_t offset,uint8_t cfglen,bool _isByUniqueid);
void Process_SetupRF(const UC *rfData,uint8_t rflen);

bool SendCfgBlock(uint8_t offset,uint8_t size,uint8_t isNeedUniqueid);
typedef struct
{
  uint8_t offset;
  uint8_t size;
  uint8_t byUniqueid;  // whether getconfig by uniqueid
}CfgBlock;
#define OUT_CFG_MESSAGE_LEN           16
CfgBlock out_cfg_msg_buf[OUT_CFG_MESSAGE_LEN];
u8 cfg_msg_out_buf_read_ptr = 0;
u8 cfg_msg_out_buf_write_ptr = 0;

bool AddCfgOutputBuf(uint8_t offset,uint8_t size,uint8_t isNeedUniqueid) {  
  CfgBlock cfgblock;
  cfgblock.offset = offset;
  cfgblock.size = size;
  cfgblock.byUniqueid = isNeedUniqueid;
  out_cfg_msg_buf[cfg_msg_out_buf_write_ptr++] = cfgblock;
  cfg_msg_out_buf_write_ptr %= OUT_CFG_MESSAGE_LEN;
  return TRUE;
}

bool ProcessOutputCfgMsg() {
  // Send output Cfg msg
  while( cfg_msg_out_buf_read_ptr != cfg_msg_out_buf_write_ptr) {   
    CfgBlock cfgblock = out_cfg_msg_buf[cfg_msg_out_buf_read_ptr++];
    SendCfgBlock(cfgblock.offset,cfgblock.size,TRUE);
    cfg_msg_out_buf_read_ptr %= OUT_CFG_MESSAGE_LEN;
  }
  return TRUE;
}

bool SendCfgBlock(uint8_t offset,uint8_t size,uint8_t isNeedUniqueid) {
  // Send output Cfg msg  
    build(NODEID_RF_SCANNER, gConfig.subID, C_INTERNAL, I_GET_NONCE_RESPONSE, 0, 1);
    // Common payload
    sndMsg.payload.data[0] = SCANNER_GETDEV_CONFIG;
    sndMsg.payload.data[1] = offset;
    uint8_t custom_playload = 2;
    if(isNeedUniqueid != 0) 
    {
      memcpy(sndMsg.payload.data + 2,_uniqueID, UNIQUE_ID_LEN);
      custom_playload += UNIQUE_ID_LEN;
    }  
    memcpy(sndMsg.payload.data + custom_playload, (void *)((uint16_t)(&gConfig) + offset), size);
    moSetLength(size+custom_playload);
    moSetPayloadType(P_CUSTOM);
    bMsgReady = 1;
    SendMyMessage();
}

// Send spotlight log
/*void SpotlightStatusLog(uint8_t _st,uint8_t _relaykey) { 
  char pBuf[20];
  memset(pBuf, 0x00, 20);
  pBuf[0] = 's';
  pBuf[1] = ' ';
  pBuf[2] = _st + '0';
  pBuf[3] = _relaykey;
  printlog((uint8_t *)pBuf);
}*/

// Assemble message
void build(uint8_t _destination, uint8_t _sensor, uint8_t _command, uint8_t _type, bool _enableAck, bool _isAck)
{
    sndMsg.header.version_length = PROTOCOL_VERSION;
    if(gConfig.nodeID != XLA_PRODUCT_NODEID)
    {
      gConfig.nodeID = XLA_PRODUCT_NODEID;
    }
    sndMsg.header.sender = gConfig.nodeID;
    sndMsg.header.destination = _destination;
    sndMsg.header.sensor = _sensor;
    sndMsg.header.type = _type;
    moSetCommand(_command);
    moSetRequestAck(_enableAck);
    moSetAck(_isAck);
}

bool AddKeyOperation(u8 _target, const char *_keyString, u8 _len)
{
  bool ret = FALSE;
  bool delay = FALSE;
  if( IS_TARGET_CURTAIN(gConfig.type)) delay = TRUE;
  if(delay)
  {
    AddKeySimToBuf(_target,_keyString,_len);
  }
  else
  {
    ret = ProduceKeyOperation(_target, _keyString, _len);
  }
  return ret;
}

uint8_t ParseProtocol(){
  if( rcvMsg.header.destination != gConfig.nodeID && !(rcvMsg.header.destination == BROADCAST_ADDRESS && (rcvMsg.header.sender == NODEID_RF_SCANNER)) ) return 0;
  
  uint8_t _cmd = miGetCommand();
  uint8_t _sender = rcvMsg.header.sender;  // The original sender
  uint8_t _type = rcvMsg.header.type;
  uint8_t _sensor = rcvMsg.header.sensor;
  uint8_t _lenPayl = miGetLength();
  bool _needAck = (bool)miGetRequestAck();
  bool _isAck = (bool)miGetAck();
  bool _OnOff;
  uint8_t targetSubID;
  
  switch( _cmd ) {
  case C_INTERNAL:
    if( _type == I_ID_RESPONSE ) {
      // Device/client got nodeID from Controller
      uint8_t lv_nodeID = _sensor;
      if( lv_nodeID == NODEID_GATEWAY || lv_nodeID == NODEID_DUMMY ) {
      } else {
        if( _lenPayl > 8 ) {
          // Verify _uniqueID        
          if(!isIdentityEqual(_uniqueID, rcvMsg.payload.data+8, UNIQUE_ID_LEN)) {
            return 0;
          }
        }
        if(gConfig.nodeID != XLA_PRODUCT_NODEID)
        {
          gConfig.nodeID = XLA_PRODUCT_NODEID;
          gIsChanged = TRUE;
        }   
        if(_isAck)
        { // request nodeid response
          memcpy(gConfig.NetworkID, rcvMsg.payload.data, sizeof(gConfig.NetworkID));
          gIsChanged = TRUE;
          GotNodeID();
        }
      }
    } else if( _type == I_REBOOT ) {
      if( IS_MINE_SUBID(_sensor) ) {
        // Verify token
        //if(!gConfig.present || gConfig.token == rcvMsg.payload.uiValue) {
          // Soft reset
          WWDG->CR = 0x80;
        //}
        return 0;
      }
    } else if( _type == I_GET_NONCE ) {
      // RF Scanner Probe
      if( _sender == NODEID_RF_SCANNER ) {
        if( rcvMsg.payload.data[0] == SCANNER_PROBE ) {      
          MsgScanner_ProbeAck();
        } else if( rcvMsg.payload.data[0] == SCANNER_SETUP_RF ) {
          if(!IS_MINE_SUBID(_sensor)) return 0;  
          Process_SetupRF(rcvMsg.payload.data + 1,_lenPayl-1);
        }
        else if( rcvMsg.payload.data[0] == SCANNER_SETUPDEV_RF ) {
          if(!isIdentityEqual(rcvMsg.payload.data + 1,_uniqueID,UNIQUE_ID_LEN)) return 0;
          Process_SetupRF(rcvMsg.payload.data + 1 + UNIQUE_ID_LEN,_lenPayl-1 - UNIQUE_ID_LEN);
        }
        else if( rcvMsg.payload.data[0] == SCANNER_SETCONFIG ) {
          if(!IS_MINE_SUBID(_sensor)) return 0;          
          uint8_t cfg_len = _lenPayl - 2;
          Process_SetConfig(cfg_len);
        }
        else if( rcvMsg.payload.data[0] == SCANNER_SETDEV_CONFIG ) {  
          if(!isIdentityEqual(rcvMsg.payload.data + 2,_uniqueID,UNIQUE_ID_LEN)) return 0;
          uint8_t cfg_len = _lenPayl - 10;
          Process_SetDevConfig(cfg_len);
        }
        else if( rcvMsg.payload.data[0] == SCANNER_GETDEV_CONFIG ) {  
          uint8_t offset = rcvMsg.payload.data[1];
          uint8_t cfgblock_len = rcvMsg.payload.data[10];
          if(!isIdentityEqual(rcvMsg.payload.data + 2,_uniqueID,UNIQUE_ID_LEN)) return 0;
          MsgScanner_ConfigAck(offset,cfgblock_len,TRUE); 
        }
        else if( rcvMsg.payload.data[0] == SCANNER_GETCONFIG ) { 
          if(!IS_MINE_SUBID(_sensor)) return 0;  
          uint8_t offset = rcvMsg.payload.data[1];
          uint8_t cfgblock_len = rcvMsg.payload.data[2];
          MsgScanner_ConfigAck(offset,cfgblock_len,FALSE);
        }
        return 1;
      }      
    } else if( _type == I_CONFIG ) {
      // Node Config
      switch( _sensor ) {
      case NCF_QUERY:
        // Inform controller with version & NCF data
        Msg_NodeConfigData(_sender);
        return 1;
        break;

      case NCF_DEV_SET_SUBID:
        gConfig.subID = rcvMsg.payload.data[0];
        break;

#ifdef EN_PANEL_BUTTONS        
      case NCF_PAN_SET_BTN_1:
      case NCF_PAN_SET_BTN_2:
      case NCF_PAN_SET_BTN_3:
      case NCF_PAN_SET_BTN_4:
        targetSubID = _sensor - NCF_PAN_SET_BTN_1;
        if( targetSubID < MAX_NUM_BUTTONS ) {
          uint8_t lv_op = BF_GET(rcvMsg.payload.data[0], 5, 3);
          gConfig.btnAction[targetSubID][lv_op].action = rcvMsg.payload.data[0];
          gConfig.btnAction[targetSubID][lv_op].keyMap = rcvMsg.payload.data[1];
        }
        break;
#endif
        
      case NCF_DEV_MAX_NMRT:
        gConfig.rptTimes = rcvMsg.payload.data[0];
        break;
        
      case NCF_MAP_SENSOR:
        gConfig.senMap = rcvMsg.payload.data[0] + rcvMsg.payload.data[1] * 256;
        break;
      }
      gIsChanged = TRUE;
      Msg_NodeConfigAck(_sender, _sensor);
      return 1;
    }
    break;
    
  case C_PRESENTATION:
#ifdef ZENSENSOR    
    if( _sensor == S_ZENSENSOR ) {
#else      
    if( _sensor == S_ZENREMOTE ) {
#endif      
      if( _isAck ) {
        // Device/client got Response to Presentation message, ready to work
        gConfig.token = rcvMsg.payload.uiValue;
        gConfig.present = (gConfig.token >  0);
        GotPresented();
        gIsStatusChanged = TRUE;
      }
    }
    break;
    
  case C_REQ:
    if( _needAck ) {
      if( IS_MINE_SUBID(_sensor) ) {
        if( _type == V_STATUS ) {
          Msg_DevOnOff(_sender);
          return 1;
        } else if( _type == V_RELAY_ON || _type == V_RELAY_OFF ) {
          _OnOff = relay_get_key(rcvMsg.payload.data[0]);
          Msg_Relay_Ack(_sender, _OnOff ? V_RELAY_ON : V_RELAY_OFF, rcvMsg.payload.data[0]);
          return 1;
        }
      }
    }    
    break;
    
  case C_SET:
    if( IS_MINE_SUBID(_sensor) && !_isAck ) {
      if( _type == V_STATUS) {
        if(gConfig.nodeID == NODEID_SUPERSENSOR)
        {
          // zensor no need process
          return 0;
        }
        else
        {
          if( IS_TARGET_CURTAIN(gConfig.type)) {
            // General Key Control
            if(rcvMsg.payload.bValue == DEVICE_SW_OFF)
            {
              targetSubID = gConfig.type & 0x0F;
              AddKeyOperation(targetSubID, CURTAIN_PAUSE, CURTAIN_PAUSE_LEN);
              _OnOff = AddKeyOperation(targetSubID, CURTAIN_OFF, CURTAIN_SW_LEN);
              //_OnOff = ProduceKeyOperation(targetSubID, CURTAIN_OFF, CURTAIN_SW_LEN);
            }
            else if(rcvMsg.payload.bValue == DEVICE_SW_ON)
            {
              targetSubID = gConfig.type & 0x0F;
              AddKeyOperation(targetSubID, CURTAIN_PAUSE, CURTAIN_PAUSE_LEN);
              _OnOff = AddKeyOperation(targetSubID, CURTAIN_ON, CURTAIN_SW_LEN);
            }
            if( _needAck ) {
              Msg_Relay_Ack(_sender, gConfig.type, _OnOff);
              return 1;
            }

          }else if( IS_TARGET_AIRPURIFIER(gConfig.type) ) {
            // General Key Control
            /// Get SubID
            // todo
            if(rcvMsg.payload.bValue == DEVICE_SW_OFF)
            {
              targetSubID = gConfig.type & 0x0F;
              _OnOff = AddKeyOperation(targetSubID, AIRPUR_STOP, AIRPUR_CMD_LEN);
            }
            else if(rcvMsg.payload.bValue == DEVICE_SW_ON)
            {
              targetSubID = gConfig.type & 0x0F;
              _OnOff = AddKeyOperation(targetSubID, AIRPUR_LOW, AIRPUR_CMD_LEN);
            }
            if( _needAck ) {
              Msg_Relay_Ack(_sender, gConfig.type, _OnOff);
              return 1;
            }
          }
          else if(IS_TARGET_AIRCONDITION(gConfig.type) )
          {
#ifdef EN_INFRARED
            if(rcvMsg.payload.bValue == DEVICE_SW_OFF)
            {
              printlog("off");
#if defined AIRCON_MEDIA
              Set_AC_Media_Buf(mediaoff, 3);
#elif defined AIRCON_HAIER
              Set_AC_Buf(haieroff, 14);
#endif
            }
            else if(rcvMsg.payload.bValue == DEVICE_SW_ON)
            { // todo
              printlog("on");
#if defined AIRCON_MEDIA    
              if(gConfig.aircondition_on_status[0] == mediaoff[0])
              {
                Set_AC_Media_Buf(gConfig.aircondition_on_status, 3);
              }
              else
              {
                Set_AC_Media_Buf(media_last_on_status, 3);
              }
#elif defined AIRCON_HAIER
              if(gConfig.aircondition_on_status[0] == haieroff[0])
              {
                Set_AC_Buf(gConfig.aircondition_on_status, 14);
              }
              else
              {
                Set_AC_Buf(haier_last_on_status, 14);
              }
#endif
            }
#endif
          }
          else if(IS_TARGET_SPOTLIGHT(gConfig.type) )
          {
            if( relay_set_key('1', rcvMsg.payload.bValue == DEVICE_SW_ON) ) {
              Msg_Relay_Ack(_sender, gConfig.type, rcvMsg.payload.bValue == DEVICE_SW_ON);
              return 1;
            }
          }
        }
      }
      else if( _type == V_RELAY_ON || _type == V_RELAY_OFF ) {
        for( uint8_t idx = 0; idx < _lenPayl; idx++ ) {
          // Send spotlight log
          //SpotlightStatusLog(_type == V_RELAY_ON,idx + '1');
          if( relay_set_key(rcvMsg.payload.data[idx], _type == V_RELAY_ON) ) {
            Msg_Relay_Ack(_sender, _type, rcvMsg.payload.data[idx]);
            //SendMyMessage();
          }
        }
      } else if( IS_TARGET_CURTAIN(gConfig.type) &&  IS_TARGET_CURTAIN(_type) ) {
        // General Key Control
        /// Get SubID
        targetSubID = _type & 0x0F;
        _OnOff = AddKeyOperation(targetSubID, rcvMsg.payload.data, _lenPayl);
        //printlog(rcvMsg.payload.data);
        if( _needAck ) {
          Msg_Relay_Ack(_sender, _type, _OnOff);
          return 1;
        }
      }else if( IS_TARGET_AIRPURIFIER(gConfig.type) &&  IS_TARGET_AIRPURIFIER(_type) ) {
        // General Key Control
        /// Get SubID
        targetSubID = _type & 0x0F;
        _OnOff = AddKeyOperation(targetSubID, rcvMsg.payload.data, _lenPayl);
        if( _needAck ) {
          Msg_Relay_Ack(_sender, _type, _OnOff);
          return 1;
        }
      }
      else if(IS_TARGET_AIRCONDITION(gConfig.type) && IS_TARGET_AIRCONDITION(_type) ) {
        // ToDo: air conditioner control code goes here
        if(_type == HAIER_CON)
        {
          // haier
          if( _lenPayl >= 14 ) {
            Set_AC_Buf(rcvMsg.payload.data, 14);
          }
        }
        else if(_type == MEDIA_CON)
        {
           if( _lenPayl >= 3 ) {
            Set_AC_Media_Buf(rcvMsg.payload.data, 3);
          }
        }
        else
        {
          // default haier
          if( _lenPayl >= 14 ) {
            Set_AC_Buf(rcvMsg.payload.data, 14);
          }
        }
      } 
      /*else if( IS_TARGET_AIRPURIFIER(_type) ) {
        // Parsing payload
        unsigned long buf[2];
        switch(rcvMsg.payload.data[1]) {
        case '1':
          buf[0] = 0x00FF00FFL;
          break;
        case '2':
          buf[0] = 0x00FF8877L;
          break;
        case '3':
          buf[0] = 0x00FFC837L;
          break;
        case '4':
          buf[0] = 0x00FF08F7L;
          break;
        case '5':
          buf[0] = 0x00FF28D7L;
          break;
        case '6':
          buf[0] = 0x00FF48B7L;
          break;
        case '7':
          buf[0] = 0x00FF54ABL;
          break;
        case '8':
          buf[0] = 0x00FF708FL;
          break;
        case '9':
          buf[0] = 0x00FF946BL;
          break;
        default:
          buf[0] = 0xFFFFFFFFL;
          break;
        }
        Set_Send_Buf(buf, 1);
      }*/
    }
    break;
  }
  
  return 0;
}

void Msg_NodeConfigAck(uint8_t _to, uint8_t _ncf) {
  build(_to, _ncf, C_INTERNAL, I_CONFIG, 0, 1);

  sndMsg.payload.data[0] = 1;      // OK
  moSetPayloadType(P_BYTE);
  moSetLength(1);
  bMsgReady = 1;
}

// Prepare NCF query ack message
void Msg_NodeConfigData(uint8_t _to) {
  uint8_t payl_len = 0;
  build(_to, NCF_QUERY, C_INTERNAL, I_CONFIG, 0, 1);

  sndMsg.payload.data[payl_len++] = gConfig.version;
  sndMsg.payload.data[payl_len++] = gConfig.subID;
  sndMsg.payload.data[payl_len++] = gConfig.type;
  sndMsg.payload.data[payl_len++] = gConfig.senMap % 256;
  sndMsg.payload.data[payl_len++] = gConfig.senMap / 256;
  sndMsg.payload.data[payl_len++] = gConfig.rptTimes;
  sndMsg.payload.data[payl_len++] = 0;     // Reservered
  sndMsg.payload.data[payl_len++] = 0;     // Reservered
  sndMsg.payload.data[payl_len++] = 0;     // Reservered
  sndMsg.payload.data[payl_len++] = 0;     // Reservered
  sndMsg.payload.data[payl_len++] = 0;     // Reservered
  sndMsg.payload.data[payl_len++] = 0;     // Reservered
  
  moSetLength(payl_len);
  moSetPayloadType(P_CUSTOM);
  bMsgReady = 1;
}

void Msg_RequestNodeID() {
  // Request NodeID for device
  build(BASESERVICE_ADDRESS, NODE_TYP_SYSTEM, C_INTERNAL, I_ID_REQUEST, 1, 0);
  moSetPayloadType(P_ULONG32);
  moSetLength(UNIQUE_ID_LEN);
  memcpy(sndMsg.payload.data, _uniqueID, UNIQUE_ID_LEN);
  bMsgReady = 1;
}

// Prepare device presentation message
void Msg_Presentation() {
#ifdef ZENSENSOR
  build(NODEID_GATEWAY, S_ZENSENSOR, C_PRESENTATION, gConfig.type, 1, 0);
#else  
  build(NODEID_GATEWAY, S_ZENREMOTE, C_PRESENTATION, gConfig.type, 1, 0);
#endif

  moSetPayloadType(P_ULONG32);
  moSetLength(UNIQUE_ID_LEN);
  memcpy(sndMsg.payload.data, _uniqueID, UNIQUE_ID_LEN);
  bMsgReady = 1;
}

// Prepare device On/Off status message
void Msg_DevOnOff(uint8_t _to) {
  build(_to, gConfig.subID, C_REQ, V_STATUS, 0, 1);
  moSetLength(1);
  moSetPayloadType(P_BYTE);
  sndMsg.payload.bValue = gConfig.state;
  bMsgReady = 1;
}

// Prepare relay key map message
void Msg_Relay_KeyMap(uint8_t _to) {
  build(_to, gConfig.subID, C_REQ, V_RELAY_MAP, 0, 1);
  moSetLength(1);
  moSetPayloadType(P_BYTE);
  sndMsg.payload.bValue = gConfig.relay_key_value;
  bMsgReady = 1;
}

// Prepare relay key ACK message
void Msg_Relay_Ack(uint8_t _to, uint8_t _type, uint8_t _key) {
  build(_to, gConfig.subID, C_REQ, _type, 0, 1);
  moSetLength(1);
  moSetPayloadType(P_BYTE);
  sndMsg.payload.bValue = _key;
  bMsgReady = 1;
}

#ifdef EN_SENSOR_ALS
// Prepare ALS message
void Msg_SenALS(uint8_t _value) {
  build(NODEID_GATEWAY, S_LIGHT_LEVEL, C_PRESENTATION, V_LIGHT_LEVEL, 0, 0);
  moSetPayloadType(P_BYTE);
  moSetLength(1);
  sndMsg.payload.data[0] = _value;
  bMsgReady = 1;
}
#endif

#ifdef EN_SENSOR_MIC
// Prepare MIC message
void Msg_SenMIC(uint16_t _value) {
  build(NODEID_GATEWAY, S_SOUND, C_PRESENTATION, V_LEVEL, 0, 0);
  moSetPayloadType(P_BYTE);
  moSetLength(2);
  sndMsg.payload.data[0] = _value % 256;
  sndMsg.payload.data[1] = _value / 256;
  bMsgReady = 1;
}
#endif

#ifdef EN_SENSOR_PIR
// Prepare PIR message
void Msg_SenPIR(bool _sw) {
  build(NODEID_GATEWAY, S_MOTION, C_PRESENTATION, V_STATUS, 0, 0);
  moSetPayloadType(P_BYTE);
  moSetLength(1);
  sndMsg.payload.data[0] = _sw;
  bMsgReady = 1;
}
#endif

#ifdef EN_SENSOR_IRKEY
// Prepare IR Key Bitmap message
void Msg_SenIRKey(uint8_t _sw) {
  build(NODEID_GATEWAY, S_IR, C_PRESENTATION, V_STATUS, 0, 0);
  moSetPayloadType(P_BYTE);
  moSetLength(1);
  sndMsg.payload.data[0] = _sw;
  bMsgReady = 1;
}
#endif

#ifdef EN_SENSOR_PM25
// Prepare PM2.5 message
void Msg_SenPM25(uint16_t _value) {
  build(NODEID_GATEWAY, S_DUST, C_PRESENTATION, V_LEVEL, 0, 0);
  moSetPayloadType(P_UINT16);
  moSetLength(2);
  sndMsg.payload.data[0] = _value % 256;
  sndMsg.payload.data[1] = _value / 256;
  bMsgReady = 1;  
}
#endif

#ifdef MULTI_SENSOR
void Msg_SenAirQuality(uint16_t pm25,uint16_t pm10,uint16_t tvoc,uint16_t ch2o,uint16_t co2)
{
  build(NODEID_GATEWAY, S_AIR_QUALITY, C_PRESENTATION, V_LEVEL, 0, 0);
  moSetPayloadType(P_BYTE);
  moSetLength(10);
  sndMsg.payload.data[0] = pm25 % 256;
  sndMsg.payload.data[1] = pm25 / 256;
  sndMsg.payload.data[2] = pm10 % 256;
  sndMsg.payload.data[3] = pm10 / 256;
  sndMsg.payload.data[4] = tvoc % 256;
  sndMsg.payload.data[5] = tvoc / 256;
  sndMsg.payload.data[6] = ch2o % 256;
  sndMsg.payload.data[7] = ch2o / 256;
  sndMsg.payload.data[8] = co2 % 256;
  sndMsg.payload.data[9] = co2 / 256;
  bMsgReady = 1; 
}

void Msg_SenTemHum(s16 tem,s16 hum) {
  build(NODEID_GATEWAY, S_TEMP, C_PRESENTATION, V_LEVEL, 0, 0);
  moSetPayloadType(P_BYTE);
  moSetLength(4);
  sndMsg.payload.data[0] = tem/100;
  sndMsg.payload.data[1] = tem%100;
  sndMsg.payload.data[2] = hum/100;
  sndMsg.payload.data[3] = hum%100;    
  bMsgReady = 1; 
}
#endif

#ifdef EN_SENSOR_DHT
// Prepare DHT message
/*
type 0  ---- all
type 1  ---- tem
type 2  ---- hum
*/
void Msg_SenDHT(s16 dht_t,s16 dht_h,u8 type) {
  /*printlog("send t=");
  printnum(dht_t);
  printlog(",h=");
  printnum(dht_h);*/
  if(type == 0)
  {
      build(NODEID_GATEWAY, S_TEMP, C_PRESENTATION, V_LEVEL, 0, 0);
      moSetPayloadType(P_BYTE);
      moSetLength(4);
      sndMsg.payload.data[0] = dht_t/100;
      sndMsg.payload.data[1] = dht_t%100;
      sndMsg.payload.data[2] = dht_h/100;
      sndMsg.payload.data[3] = dht_h%100;    
  }
  else if (type == 1)
  {
      build(NODEID_GATEWAY, S_TEMP, C_PRESENTATION, V_TEMP, 0, 0);
      moSetPayloadType(P_BYTE);
      moSetLength(2);
      sndMsg.payload.data[0] = dht_t/100;
      sndMsg.payload.data[1] = dht_t%100;

  }
  else if (type == 2)
  {
      build(NODEID_GATEWAY, S_HUM, C_PRESENTATION, V_HUM, 0, 0);
      moSetPayloadType(P_BYTE);
      moSetLength(2);
      sndMsg.payload.data[0] = dht_h/100;
      sndMsg.payload.data[1] = dht_h%100; 
  }
  bMsgReady = 1; 
}
#endif

//----------------------------------------------
// RF Scanner Messages
//----------------------------------------------
// Probe ack message
void MsgScanner_ProbeAck() {
  uint8_t payl_len = UNIQUE_ID_LEN + 1;
  build(NODEID_RF_SCANNER, 0x00, C_INTERNAL, I_GET_NONCE_RESPONSE, 0, 1);

  // Common payload
  sndMsg.payload.data[0] = SCANNER_PROBE;
  memcpy(sndMsg.payload.data + 1, _uniqueID, UNIQUE_ID_LEN);
  
  sndMsg.payload.data[payl_len++] = gConfig.version;
  sndMsg.payload.data[payl_len++] = gConfig.type;
  sndMsg.payload.data[payl_len++] = gConfig.nodeID;
  sndMsg.payload.data[payl_len++] = gConfig.subID;
  sndMsg.payload.data[payl_len++] = gConfig.rfChannel;
  sndMsg.payload.data[payl_len++] = (gConfig.rfDataRate << 2) + gConfig.rfPowerLevel;
  memcpy(sndMsg.payload.data + payl_len, gConfig.NetworkID, sizeof(gConfig.NetworkID));
  payl_len += sizeof(gConfig.NetworkID);
  
  moSetLength(payl_len);
  moSetPayloadType(P_CUSTOM);
  bMsgReady = 1;
}
//typedef struct
//{
//    uint8_t subtype;
//    uint8_t offset;
//    uint8_t uniqueid[8];
//    UC ConfigBlock[15];
//}MyMsgPayload_t
#define CFGBLOCK_SIZE    15
#define CFGBLOCK_NO_UNIQUEID_SIZE CFGBLOCK_SIZE+UNIQUE_ID_LEN
void MsgScanner_ConfigAck(uint8_t offset,uint8_t cfglen,bool _isByUniqueid) {
  uint8_t cfg_end_offset = cfglen;
  if(cfglen == 0) cfg_end_offset = sizeof(Config_t)-1;
  else
  {
    cfg_end_offset = offset + cfglen > sizeof(Config_t)-1?sizeof(Config_t)-1:offset + cfglen;
  }  
  while( offset < cfg_end_offset )
  {
    uint8_t left_len = cfg_end_offset - offset;
    uint8_t payl_len = left_len < CFGBLOCK_SIZE ? left_len : CFGBLOCK_SIZE;
    if(_isByUniqueid) AddCfgOutputBuf(offset,payl_len,1);
    else  
    {
      payl_len = left_len < CFGBLOCK_NO_UNIQUEID_SIZE ? left_len : CFGBLOCK_NO_UNIQUEID_SIZE;
      AddCfgOutputBuf(offset,payl_len,0);
    }
    offset+=payl_len;
    offset %= sizeof(Config_t);
  }
}

//////set config by nodeid&subid data struct/////////////////////
//typedef struct
//{
//    uint8_t subtype;
//    uint8_t offset;  //config offset
//    UC ConfigBlock[23];
//}MyMsgPayload_t
//////set config by nodeid&subid data struct/////////////////////
void Process_SetConfig(u8 _len) {
  uint8_t offset = rcvMsg.payload.data[1];
  memcpy((void *)((uint16_t)(&gConfig) + offset),rcvMsg.payload.data+2,_len);
  gIsChanged = TRUE;
}
//////set config by uniqueid data struct/////////////////////
//typedef struct
//{
//    uint8_t subtype;
//    uint8_t offset;   //config offset
//    uint8_t uniqueid[8];
//    
//    UC ConfigBlock[15];
//}MyMsgPayload_t
//////set config by uniqueid data struct/////////////////////
void Process_SetDevConfig(u8 _len) {
    uint8_t offset = rcvMsg.payload.data[1];
    memcpy((void *)((uint16_t)(&gConfig) + offset),rcvMsg.payload.data+2+UNIQUE_ID_LEN,_len);
    gIsChanged = TRUE;
}
//////set rf /////////////////////////////////////////////////
//typedef struct
//{
//    uint8_t subtype;
//    uint8_t uniqueid[8];
//    uint8_t channel;
//    uint8_t datarate;
//    uint8_t powerlevel;
//    uint8_t network[6];
//    uint8_t nodeid;        //unnecessary data field£¬has this field£¬need change nodeid£¬0 indicate ignore this parameter
//    uint8_t subid;         //unnecessary data field£¬has this field£¬need change subid
//}MyMsgPayload_t
//////set rf /////////////////////////////////////////////////
void Process_SetupRF(const UC *rfData,uint8_t rflen)
{
  bool bNeedChangeCfg = FALSE;
  if(rflen > 0 &&(*rfData)>=0 && (*rfData)<=127)
  {
    if(gConfig.rfChannel != (*rfData))
    {
      gConfig.rfChannel = (*rfData);
      gResetRF = TRUE;
    } 
  }
  rfData++;
  if(rflen > 1 &&(*rfData)>=RF24_1MBPS && (*rfData)<= RF24_250KBPS)
  {
    if(gConfig.rfDataRate != (*rfData))
    {
      gConfig.rfDataRate = (*rfData);
      gResetRF = TRUE;
    } 
  }
  rfData++;
  if(rflen > 2 &&(*rfData)>=RF24_PA_MIN && (*rfData)<= RF24_PA_ERROR)
  {
    if(gConfig.rfPowerLevel != (*rfData))
    {
      gConfig.rfPowerLevel = (*rfData);
      gResetRF = TRUE;
    } 
  }
  rfData++;
  bool bValidNet = FALSE;
  bool newNetwork[6] = {0};
  if(rflen > 8)
  {  
    for(uint8_t i = 0;i<6;i++)
    {
      if(*(rfData+i) != 0)
      {
        bValidNet=TRUE;
        break;
      }
    }
    if(isIdentityEqual(rfData,gConfig.NetworkID,sizeof(gConfig.NetworkID)))
    {
      bValidNet=FALSE;
    }
    else
    {
      memcpy(newNetwork,rfData,sizeof(newNetwork));
    }
  }
  rfData = rfData + sizeof(gConfig.NetworkID);
  bool bNeedResetNode = FALSE;
  if(rflen > 9 && (* rfData) != 0)
  {
    if(gConfig.nodeID != (* rfData))
    {
      gConfig.nodeID = (* rfData);
      bNeedResetNode = TRUE;
    }
  }
  rfData++; 
  if(rflen > 10)
  {
    if(gConfig.subID != (* rfData ))
    {
      gConfig.subID = (*rfData);
      bNeedChangeCfg = TRUE;
    }
  }
  if(bValidNet)
  {
    memcpy(gConfig.NetworkID,newNetwork,sizeof(gConfig.NetworkID));
    bNeedResetNode = TRUE;
  }
  if(bNeedResetNode)
    gResetNode = TRUE;
  if(gResetNode || gResetRF || bNeedChangeCfg)
  {
    gIsChanged = TRUE;
  }
}
//----------------------------------------------