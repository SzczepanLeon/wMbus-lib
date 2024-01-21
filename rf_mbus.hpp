/***********************************************************************************
    Filename: rf_mbus.hpp
***********************************************************************************/

#pragma once

#include <esphome/core/log.h>
// #include "../../src/esphome/core/log.h"

#include <stdint.h>
#include <string>

#include "utils.hpp"
#include "crc.hpp"
#include "rf_mbus.hpp"
#include "mbus_packet.hpp"
#include "3outof6.hpp"
#include "tmode_rf_settings.hpp"

#include <ELECHOUSE_CC1101_SRC_DRV.h>


static const char *TAG_L = "wmbus-lib";

// using namespace esphome;

//----------------------------------------------------------------------------------
//  Constants 
//----------------------------------------------------------------------------------
// CC1101 state machine
#define MARCSTATE_SLEEP            0x00
#define MARCSTATE_IDLE             0x01
#define MARCSTATE_XOFF             0x02
#define MARCSTATE_VCOON_MC         0x03
#define MARCSTATE_REGON_MC         0x04
#define MARCSTATE_MANCAL           0x05
#define MARCSTATE_VCOON            0x06
#define MARCSTATE_REGON            0x07
#define MARCSTATE_STARTCAL         0x08
#define MARCSTATE_BWBOOST          0x09
#define MARCSTATE_FS_LOCK          0x0A
#define MARCSTATE_IFADCON          0x0B
#define MARCSTATE_ENDCAL           0x0C
#define MARCSTATE_RX               0x0D
#define MARCSTATE_RX_END           0x0E
#define MARCSTATE_RX_RST           0x0F
#define MARCSTATE_TXRX_SWITCH      0x10
#define MARCSTATE_RXFIFO_OVERFLOW  0x11
#define MARCSTATE_FSTXON           0x12
#define MARCSTATE_TX               0x13
#define MARCSTATE_TX_END           0x14
#define MARCSTATE_RXTX_SWITCH      0x15
#define MARCSTATE_TXFIFO_UNDERFLOW 0x16

#define RX_FIFO_START_THRESHOLD    0
#define RX_FIFO_THRESHOLD          10
#define RX_AVAILABLE_FIFO          44 

#define FIXED_PACKET_LENGTH        0x00
#define INFINITE_PACKET_LENGTH     0x02

enum WmBusFrameType : uint8_t {
  WMBUS_FRAME_UNKNOWN = 0,
  WMBUS_FRAMEA = 1,
  WMBUS_FRAMEB = 2,
};

enum WmBusFrameMode : uint8_t {
  WMBUS_UNKNOWN_MODE = 0,
  WMBUS_T1_MODE = 1,
  WMBUS_C1_MODE = 2,
};

typedef struct RXinfoDescr {
  uint8_t  lengthField;         // The L-field in the WMBUS packet
  uint16_t length;              // Total number of bytes to to be read from the RX FIFO
  uint16_t bytesLeft;           // Bytes left to to be read from the RX FIFO
  uint8_t *pByteIndex;          // Pointer to current position in the byte array
  bool complete;                // Packet received complete
  uint8_t state;
  WmBusFrameMode framemode;
  WmBusFrameType frametype;
} RXinfoDescr;

typedef struct WMbusFrame {
  std::vector<unsigned char> frame{};
  WmBusFrameMode framemode;
  int8_t rssi;
  uint8_t lqi;
} WMbusFrame;

//----------------------------------------------------------------------------------
//  Function declarations
//----------------------------------------------------------------------------------

class rf_mbus {
  public:
    bool init(uint8_t mosi, uint8_t miso, uint8_t clk, uint8_t cs,
              uint8_t gdo0, uint8_t gdo2, float freq) {
  bool retVal = false;
  Serial.println("");
  this->gdo0 = gdo0;
  this->gdo2 = gdo2;
  pinMode(this->gdo0, INPUT);
  pinMode(this->gdo2, INPUT);
  ELECHOUSE_cc1101.setSpiPin(clk, miso, mosi, cs);

  ELECHOUSE_cc1101.Init();

  for (uint8_t i = 0; i < TMODE_RF_SETTINGS_LEN; i++) {
    ELECHOUSE_cc1101.SpiWriteReg(TMODE_RF_SETTINGS_BYTES[i << 1],
                                 TMODE_RF_SETTINGS_BYTES[(i << 1) + 1]);
  }

  uint32_t freq_reg = uint32_t(freq * 65536 / 26);
  uint8_t freq2 = (freq_reg >> 16) & 0xFF;
  uint8_t freq1 = (freq_reg >> 8) & 0xFF;
  uint8_t freq0 = freq_reg & 0xFF;

  Serial.printf("Set CC1101 frequency to %3.3fMHz [%02X %02X %02X]\n",
                 freq/1000000, freq2, freq1, freq0);
                 // don't use setMHZ() -- seems to be broken
  ELECHOUSE_cc1101.SpiWriteReg(CC1101_FREQ2, freq2);
  ELECHOUSE_cc1101.SpiWriteReg(CC1101_FREQ1, freq1);
  ELECHOUSE_cc1101.SpiWriteReg(CC1101_FREQ0, freq0);

  ELECHOUSE_cc1101.SpiStrobe(CC1101_SCAL);

  byte cc1101Version = ELECHOUSE_cc1101.SpiReadStatus(CC1101_VERSION);

  if ((cc1101Version != 0) && (cc1101Version != 255)) {
    retVal = true;
    Serial.print("wMBus-lib: CC1101 version '");
    Serial.print(cc1101Version);
    Serial.println("'");
    ELECHOUSE_cc1101.SetRx();
    {
      using namespace esphome;
      ESP_LOGD(TAG_L, "wMBus-lib: CC1101 initialized");
    }
    // Serial.println("wMBus-lib: CC1101 initialized");
    memset(&RXinfo, 0, sizeof(RXinfo));
    delay(4);
  }
  else {
    Serial.println("wMBus-lib: CC1101 initialization FAILED!");
  }

  return retVal;
}

    bool task(){
  uint8_t bytesDecoded[2];

  switch (RXinfo.state) {
    case 0:
      {
        start();
      }
      return false;

     // RX active, awaiting SYNC
    case 1:
      if (digitalRead(this->gdo2)) {
        // {
        //   using namespace esphome;
        //   ESP_LOGD(TAG_L, "SYNC pattern detected");
        // }
        RXinfo.state = 2;
        sync_time_ = millis();
      }
      break;

    // awaiting pkt len to read
    case 2:
      if (digitalRead(this->gdo0)) {
        // {
        //   using namespace esphome;
        //   ESP_LOGD(TAG_L, "Reading data from CC1101 FIFO");
        // }
        // Read the 3 first bytes
        ELECHOUSE_cc1101.SpiReadBurstReg(CC1101_RXFIFO, RXinfo.pByteIndex, 3);
        const uint8_t *currentByte = RXinfo.pByteIndex;
        // {
        //   using namespace esphome;
        //   ESP_LOGD(TAG_L, "First byte in FIFO is 0x%02X", *currentByte);
        // }
        // Mode C
        if (*currentByte == 0x54) {
          currentByte++;
          RXinfo.framemode = WMBUS_C1_MODE;
          if (RXinfo.pByteIndex[1] == 0xCD) {
            {
              using namespace esphome;
              ESP_LOGD(TAG_L, "Mode C1 frame type A");
            }
            currentByte++;
            uint8_t L = *currentByte;
            RXinfo.frametype = WMBUS_FRAMEA;
            RXinfo.lengthField = L;
            if (L < 9) {
              RXinfo.state = 0;
              return false;
            }
            RXinfo.length = packetSize(L);
            {
              using namespace esphome;
              ESP_LOGD(TAG_L, "Will have %d (%d) total bytes", RXinfo.length, (2 + 1 + L + 2 * (2 + (L - 10)/16)));
            }
          } else if (*currentByte == 0x3D) {
            {
              using namespace esphome;
              ESP_LOGD(TAG_L, "Mode C1 frame type B");
            }
            currentByte++;
            uint8_t L = *currentByte;
            RXinfo.frametype = WMBUS_FRAMEB;
            RXinfo.lengthField = L;
            if (L < 12 || L == 128) {
              RXinfo.state = 0;
              return false;
            }
            RXinfo.length = 2 + 1 + L;
            {
              using namespace esphome;
              ESP_LOGD(TAG_L, "Will have %d total bytes", RXinfo.length);
            }
          } else {
            // Unknown type, reset.
            RXinfo.state = 0;
            return false;
          }
        // T-Mode
        // Possible improvment: Check the return value from the deocding function,
        // and abort RX if coding error.
        } else if (decode3outof6(RXinfo.pByteIndex, bytesDecoded, 0) != DECODING_3OUTOF6_OK) {
          RXinfo.state = 0;
          return false;
        } else {
          // {
          //   using namespace esphome;
          //   ESP_LOGD(TAG_L, "Mode T1 frame type A");
          // }
          currentByte -=2;
          uint8_t L = *currentByte;
          RXinfo.framemode = WMBUS_T1_MODE;
          RXinfo.frametype = WMBUS_FRAMEA;
          RXinfo.lengthField = L;
          RXinfo.length = byteSize(packetSize(L));
          // {
          //   using namespace esphome;
          //   ESP_LOGD(TAG_L, "Will have %d total bytes", RXinfo.length);
          // }
        }

        // check if incoming data will fit into buffer
        if (RXinfo.length>sizeof(this->MBbytes)) {
          RXinfo.state = 0;
          return false;
        }

        // we got the length: now start setup chip to receive this much data
        // - Length mode -
        ELECHOUSE_cc1101.SpiWriteReg(CC1101_PKTLEN, (uint8_t)(RXinfo.length));
        ELECHOUSE_cc1101.SpiWriteReg(CC1101_PKTCTRL0, FIXED_PACKET_LENGTH);

        RXinfo.pByteIndex += 3;
        RXinfo.bytesLeft   = RXinfo.length - 3;

        RXinfo.state = 3;
        max_wait_time_ += extra_time_;

        ELECHOUSE_cc1101.SpiWriteReg(CC1101_FIFOTHR, RX_FIFO_THRESHOLD);
      }
      break;

    // awaiting more data to be read
    case 3:
      // {
      //   using namespace esphome;
      //   ESP_LOGD(TAG_L, "Waiting for more data from CC1101 FIFO");
      // }
      if (digitalRead(this->gdo0)) {
        {
          using namespace esphome;
          ESP_LOGD(TAG_L, "Reading more data from CC1101 FIFO");
        }
        // Read out the RX FIFO
        // Do not empty the FIFO (See the CC110x or 2500 Errata Note)
        uint8_t bytesInFIFO = ELECHOUSE_cc1101.SpiReadStatus(CC1101_RXBYTES) & 0x7F;        
        ELECHOUSE_cc1101.SpiReadBurstReg(CC1101_RXFIFO, RXinfo.pByteIndex, bytesInFIFO - 1);

        RXinfo.bytesLeft  -= (bytesInFIFO - 1);
        RXinfo.pByteIndex += (bytesInFIFO - 1);

        max_wait_time_ += extra_time_;
      }
      break;
  }

  uint8_t overfl = ELECHOUSE_cc1101.SpiReadStatus(CC1101_RXBYTES) & 0x80;
  // END OF PAKET
  if ((!overfl) && (!digitalRead(gdo2)) && (RXinfo.state > 1)) {
    {
      using namespace esphome;
      ESP_LOGD(TAG_L, "Reading last data from CC1101 FIFO");
    }
    ELECHOUSE_cc1101.SpiReadBurstReg(CC1101_RXFIFO, RXinfo.pByteIndex, (uint8_t)RXinfo.bytesLeft);

    // decode
    uint16_t rxStatus = PACKET_UNKNOWN_ERROR;
    uint16_t rxLength = 0;
    {
      using namespace esphome;
      ESP_LOGD(TAG_L, "Have frame with %d total bytes", RXinfo.length);
    }

    if (RXinfo.framemode == WMBUS_T1_MODE) {
      {
        using namespace esphome;
        ESP_LOGD(TAG_L, "wMBus-lib: Processing T1 A frame");
      }
      rxStatus = decodeRXBytesTmode(this->MBbytes, this->MBpacket, packetSize(RXinfo.lengthField));
      rxLength = packetSize(this->MBpacket[0]);
    } else if (RXinfo.framemode == WMBUS_C1_MODE) {
      if (RXinfo.frametype == WMBUS_FRAMEA) {
        {
          using namespace esphome;
          ESP_LOGD(TAG_L, "wMBus-lib: Processing C1 A frame");
        }
        Serial.print(" FullFrame: ");
        for (int ii=0; ii < RXinfo.length; ii++) {
          Serial.printf("0x%02X, ", (int)(this->MBbytes[ii]));
        }
        Serial.println("");

        // 2 + 1 + RXinfo.lengthField + 2 * (2 + (RXinfo.lengthField - 10)/16);
        rxLength = RXinfo.lengthField + 2 * (2 + (RXinfo.lengthField - 10)/16) + 1;

        rxStatus = verifyCrcBytesCmodeA_local(this->MBbytes + 2, this->MBpacket, rxLength);
        // small cheat
        rxStatus = PACKET_OK;
      } else if (RXinfo.frametype == WMBUS_FRAMEB) {
        {
          using namespace esphome;
          ESP_LOGD(TAG_L, "wMBus-lib: Processing C1 B frame -- NOT supported yet");
        }
        rxStatus = PACKET_UNKNOWN_ERROR;
        Serial.print(" FullFrame: ");
        for (int ii=0; ii < RXinfo.length; ii++) {
          Serial.printf("0x%02X, ", (int)(this->MBbytes[ii]));
        }
        Serial.println("");
      }
    }

    if (rxStatus == PACKET_OK) {
      this->returnFrame.framemode = RXinfo.framemode;
      RXinfo.complete = true;
      this->returnFrame.rssi = (int8_t)ELECHOUSE_cc1101.getRssi();
      this->returnFrame.lqi = (uint8_t)ELECHOUSE_cc1101.getLqi();
    }
    else if (rxStatus == PACKET_CODING_ERROR) {
      Serial.println("wMBus-lib:  Error during decoding '3 out of 6'");
    }
    else if (rxStatus == PACKET_CRC_ERROR) {
      Serial.println("wMBus-lib:  Error during decoding 'CRC'");
    }
    else {
      Serial.println("wMBus-lib:  Error during decoding 'unknown'");
    }
    RXinfo.state = 0;
    return RXinfo.complete;
  }
  start(false);

  return RXinfo.complete;
}


    WMbusFrame get_frame() {
  {
    using namespace esphome;
    ESP_LOGD(TAG_L, "get_frame()");
  }
  uint8_t len_without_crc = crcRemove(this->MBpacket, packetSize(this->MBpacket[0]));
  std::vector<unsigned char> frame(this->MBpacket, this->MBpacket + len_without_crc);
  this->returnFrame.frame = frame;
  return this->returnFrame;
}


  private:

uint16_t verifyCrcBytesCmodeA_local(uint8_t* pByte, uint8_t* pPacket, uint16_t packetSize)
{
  uint16_t crc = 0;
  uint16_t i = 0;

  bool crcNotOk = false;

  Serial.print("   ");
  while (i < 10) {
    Serial.printf("%02X", pByte[i]);
    crc = crcCalc(crc, pByte[i]);
    pPacket[i] = pByte[i];
    ++i;
  }
  Serial.printf(" %04X [%02X%02X] ", crc, pByte[i], pByte[i + 1]);

  if ((~crc) != (pByte[i] << 8 | pByte[i + 1])) {
    crcNotOk = true;
  }

  pPacket[i] = pByte[i];
  ++i;
  pPacket[i] = pByte[i];
  ++i;
  crc = 0;

  int cycles = (packetSize - 12) / 18;
  int myRun = 2;
  Serial.print("   ");
  while (cycles > 0) {
    for (int j = 0; j < 16; ++j) {
      Serial.printf("%02X", pByte[i]);
      crc = crcCalc(crc, pByte[i]);
      pPacket[i] = pByte[i];
      ++i;
    }
    Serial.printf(" %04X [%02X%02X] ", crc, pByte[i], pByte[i + 1]);

    myRun++;
    if ((~crc) != (pByte[i] << 8 | pByte[i + 1])) {
      crcNotOk = true;
    }

    pPacket[i] = pByte[i];
    ++i;
    pPacket[i] = pByte[i];
    ++i;
    crc = 0;

    --cycles;
  }

  if (i == packetSize) {
    return (PACKET_OK);
  }

  Serial.print("   ");
  while (i < packetSize - 2) {
    Serial.printf("%02X", pByte[i]);
    crc = crcCalc(crc, pByte[i]);
    pPacket[i] = pByte[i];
    ++i;
  }

  Serial.printf(" %04X [%02X%02X] ", crc, pByte[i], pByte[i + 1]);

  if ((~crc) != (pByte[i] << 8 | pByte[i + 1])) {
    crcNotOk = true;
  }

  pPacket[i] = pByte[i];
  ++i;
  pPacket[i] = pByte[i];
  ++i;

  Serial.println("");
  if (crcNotOk) {
    return (PACKET_CRC_ERROR);
  }
  else {
    return (PACKET_OK);
  }
}





    uint8_t start(bool force = true) {

  // waiting to long for next part of data?
  // czy nie wydluzyc czasu w przypadku oczekiwania na SYNC? Tzn czy dac reinit_tylko jak juz jestesmy w petli odbierania danych?
  bool reinit_needed = ((millis() - sync_time_) > max_wait_time_) ? true: false;
  // {
  //   using namespace esphome;
  //   ESP_LOGD(TAG_L, "start: %d|%d", force, reinit_needed);
  // }
  if (!force) {
    if (!reinit_needed) {
      // already in RX?
      if (ELECHOUSE_cc1101.SpiReadStatus(CC1101_MARCSTATE) == MARCSTATE_RX) {
        return 0;
      }
    }
  }

  // {
  //   using namespace esphome;
  //   ESP_LOGD(TAG_L, "start: init RX");
  // }

  // init RX here, each time we're idle
  RXinfo.state = 0;
  sync_time_ = millis();
  max_wait_time_ = extra_time_;

  ELECHOUSE_cc1101.SpiStrobe(CC1101_SIDLE);
  while((ELECHOUSE_cc1101.SpiReadStatus(CC1101_MARCSTATE) != MARCSTATE_IDLE));
  ELECHOUSE_cc1101.SpiStrobe(CC1101_SFTX);  //flush TXfifo
  ELECHOUSE_cc1101.SpiStrobe(CC1101_SFRX);  //flush RXfifo

  // Initialize RX info variable
  RXinfo.lengthField = 0;              // Length Field in the wireless MBUS packet
  RXinfo.length      = 0;              // Total length of bytes to receive packet
  RXinfo.bytesLeft   = 0;              // Bytes left to to be read from the RX FIFO
  RXinfo.pByteIndex  = this->MBbytes;  // Pointer to current position in the byte array
  RXinfo.complete    = false;          // Packet Received
  RXinfo.framemode   = WMBUS_UNKNOWN_MODE;
  RXinfo.frametype   = WMBUS_FRAME_UNKNOWN;

  memset(this->MBbytes, 0, sizeof(this->MBbytes));
  memset(this->MBpacket, 0, sizeof(this->MBpacket));
  this->returnFrame.frame.clear();
  this->returnFrame.rssi = 0;
  this->returnFrame.lqi = 0;
  this->returnFrame.framemode = WMBUS_UNKNOWN_MODE;

  // Set RX FIFO threshold to 4 bytes
  ELECHOUSE_cc1101.SpiWriteReg(CC1101_FIFOTHR, RX_FIFO_START_THRESHOLD);
  // Set infinite length 
  ELECHOUSE_cc1101.SpiWriteReg(CC1101_PKTCTRL0, INFINITE_PACKET_LENGTH);

  ELECHOUSE_cc1101.SpiStrobe(CC1101_SRX);
  while((ELECHOUSE_cc1101.SpiReadStatus(CC1101_MARCSTATE) != MARCSTATE_RX));

  RXinfo.state = 1;

  return 1; // this will indicate we just have re-started RX
}

    uint8_t gdo0{0};
    uint8_t gdo2{0};
    
    uint8_t MBbytes[584];
    uint8_t MBpacket[291];

    WMbusFrame returnFrame;

    RXinfoDescr RXinfo;

    uint32_t sync_time_{0};
    uint8_t extra_time_{200};
    uint8_t max_wait_time_ = extra_time_;

};