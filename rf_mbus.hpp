/***********************************************************************************
    Filename: rf_mbus.hpp
***********************************************************************************/

#pragma once

#include <esphome/core/log.h>
// #include "../../src/esphome/core/log.h"

#include <stdint.h>
#include <string>

// #include "utils.hpp"
// #include "crc.hpp"
// #include "rf_mbus.hpp"
// #include "mbus_packet.hpp"
// #include "3outof6.hpp"
#include "tmode_rf_settings.hpp"

#include <ELECHOUSE_CC1101_SRC_DRV.h>


static const char *TAG_L = "wmbus-lib";

// using namespace esphome;


//////

#include <string>
#include <vector>

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


#define BLOCK1A_SIZE 12     // Size of Block 1, format A
#define BLOCK1B_SIZE 10     // Size of Block 1, format B
#define BLOCK2B_SIZE 118    // Maximum size of Block 2, format B
#define BLOCK1_2B_SIZE 128

// Helper macros, collides with MSVC's stdlib.h unless NOMINMAX is used
#ifndef MAX
#define MAX(a,b) ((a) > (b) ? (a) : (b))
#endif
#ifndef MIN
#define MIN(a,b) ((a) < (b) ? (a) : (b))
#endif

enum RxLoopState : uint8_t {
  INIT_RX       = 0,
  WAIT_FOR_SYNC = 1,
  WAIT_FOR_DATA = 2,
  READ_DATA     = 3,
};

    // uint8_t MBbytes[584];
    // uint8_t MBpacket[291];

typedef struct {
    uint16_t  rxL;
    uint8_t   data[500];
} m_bus_data_in_t;


typedef struct {
    uint16_t  length;
    uint8_t   data[500];
} m_bus_data_t;


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
  RxLoopState state;
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

//

static char format_my_hex_char(uint8_t v) { return v >= 10 ? 'a' + (v - 10) : '0' + v; }
std::string format_my_hex(const uint8_t *data, size_t length) {
  std::string ret;
  ret.resize(length * 2);
  for (size_t i = 0; i < length; i++) {
    ret[2 * i] = format_my_hex_char((data[i] & 0xF0) >> 4);
    ret[2 * i + 1] = format_my_hex_char(data[i] & 0x0F);
  }
  return ret;
}
std::string format_my_hex(const std::vector<uint8_t> &data) { return format_my_hex(data.data(), data.size()); }

static char format_my_hex_pretty_char(uint8_t v) { return v >= 10 ? 'A' + (v - 10) : '0' + v; }
std::string format_my_hex_pretty(const uint8_t *data, size_t length) {
  if (length == 0)
    return "";
  std::string ret;
  ret.resize(3 * length - 1);
  for (size_t i = 0; i < length; i++) {
    ret[3 * i] = format_my_hex_pretty_char((data[i] & 0xF0) >> 4);
    ret[3 * i + 1] = format_my_hex_pretty_char(data[i] & 0x0F);
    if (i != length - 1)
      ret[3 * i + 2] = '.';
  }
  if (length > 4)
    return ret + " (" + std::to_string(length) + ")";
  return ret;
}
std::string format_my_hex_pretty(const std::vector<uint8_t> &data) { return format_my_hex_pretty(data.data(), data.size()); }

std::string format_my_hex_pretty(const uint16_t *data, size_t length) {
  if (length == 0)
    return "";
  std::string ret;
  ret.resize(5 * length - 1);
  for (size_t i = 0; i < length; i++) {
    ret[5 * i] = format_my_hex_pretty_char((data[i] & 0xF000) >> 12);
    ret[5 * i + 1] = format_my_hex_pretty_char((data[i] & 0x0F00) >> 8);
    ret[5 * i + 2] = format_my_hex_pretty_char((data[i] & 0x00F0) >> 4);
    ret[5 * i + 3] = format_my_hex_pretty_char(data[i] & 0x000F);
    if (i != length - 1)
      ret[5 * i + 2] = '.';
  }
  if (length > 4)
    return ret + " (" + std::to_string(length) + ")";
  return ret;
}
std::string format_my_hex_pretty(const std::vector<uint16_t> &data) { return format_my_hex_pretty(data.data(), data.size()); }

//

// Mapping from 6 bits to 4 bits. "3of6" coding used for Mode T
static uint8_t decode3of6(uint8_t t_byte) {
    uint8_t retVal{0xFF}; // Error
    switch(t_byte) {
        case 22:  retVal = 0x0;  break;  // 0x16
        case 13:  retVal = 0x1;  break;  // 0x0D
        case 14:  retVal = 0x2;  break;  // 0x0E
        case 11:  retVal = 0x3;  break;  // 0x0B
        case 28:  retVal = 0x4;  break;  // 0x1C
        case 25:  retVal = 0x5;  break;  // 0x19
        case 26:  retVal = 0x6;  break;  // 0x1A
        case 19:  retVal = 0x7;  break;  // 0x13
        case 44:  retVal = 0x8;  break;  // 0x2C
        case 37:  retVal = 0x9;  break;  // 0x25
        case 38:  retVal = 0xA;  break;  // 0x26
        case 35:  retVal = 0xB;  break;  // 0x23
        case 52:  retVal = 0xC;  break;  // 0x34
        case 49:  retVal = 0xD;  break;  // 0x31
        case 50:  retVal = 0xE;  break;  // 0x32
        case 41:  retVal = 0xF;  break;  // 0x29
        default:                 break;  // Error
    }
    return retVal;
}


static uint16_t crc16(uint8_t const t_message[], uint8_t t_nBytes, uint16_t t_polynomial, uint16_t t_init) {
    uint16_t remainder{t_init};

    for (uint8_t byte{0}; byte < t_nBytes; ++byte) {
        remainder ^= t_message[byte] << 8;
        for (uint8_t bit{0}; bit < 8; ++bit) {
            if (remainder & 0x8000) {
                remainder = (remainder << 1) ^ t_polynomial;
            }
            else {
                remainder = (remainder << 1);
            }
        }
    }
    return remainder;
}

// Validate CRC
static bool crcValid(const uint8_t *t_bytes, uint8_t t_crcOffset) {
    static const uint16_t CRC_POLY{0x3D65};
    uint16_t crcCalc = ~crc16(t_bytes, t_crcOffset, CRC_POLY, 0);
    uint16_t crcRead = (((uint16_t)t_bytes[t_crcOffset] << 8) | t_bytes[t_crcOffset+1]);
    if (crcCalc != crcRead) {
        {
          using namespace esphome;
          ESP_LOGD(TAG_L, "M-Bus: CRC error: Calculated 0x%0X, Read: 0x%0X", crcCalc, crcRead);
        }
        return false;
    }
    {
      using namespace esphome;
      ESP_LOGD(TAG_L, "M-Bus: CRC OK: Calculated 0x%0X, Read: 0x%0X", crcCalc, crcRead);
    }
    return true;
}


static bool mBusDecodeFormatA(const m_bus_data_t *t_in, m_bus_data_t *t_out) {
    uint8_t L = t_in->data[0];

    // Store length of data
    t_out->length  = (L - 9 + BLOCK1A_SIZE - 2);

    // Validate CRC
    if (!crcValid(t_in->data, 10)) {
        return false;
    }

    // Check length of package is sufficient
    uint8_t num_data_blocks = (L - 9 + 15) / 16;                                         // Data blocks are 16 bytes long + 2 CRC bytes (not counted in L)
    if ((L < 9) || (((L - 9 + (num_data_blocks * 2))) > (t_in->length - BLOCK1A_SIZE))) {  // add CRC bytes for each data block
        {
          using namespace esphome;
          ESP_LOGD(TAG_L, "M-Bus: Package (%u) too short for packet Length: %u", t_in->length, L);
          ESP_LOGD(TAG_L, "M-Bus: %u > %u", (L - 9 + (num_data_blocks * 2)), (t_in->length - BLOCK1A_SIZE));
        }
        return false;
    }

    memcpy(t_out->data, t_in->data, (BLOCK1A_SIZE - 2));
    // Get all remaining data blocks and concatenate into data array (removing CRC bytes)
    for (uint8_t n{0}; n < num_data_blocks; ++n) {
        const uint8_t *in_ptr = (t_in->data + BLOCK1A_SIZE + (n * 18));       // Pointer to where data starts. Each block is 18 bytes
        uint8_t *out_ptr      = (t_out->data + (n * 16) + BLOCK1A_SIZE - 2);  // Pointer into block where data starts.
        uint8_t block_size    = (MIN((L - 9 - (n * 16)), 16) + 2);              // Maximum block size is 16 Data + 2 CRC

        // Validate CRC
        if (!crcValid(in_ptr, (block_size - 2))) {
            return false;
        }

        // Get block data
        memcpy(out_ptr, in_ptr, block_size);
    }

    return true;
}

static bool mBusDecodeFormatB(const m_bus_data_t *t_in, m_bus_data_t *t_out) {
    uint8_t L = t_in->data[0];

    // Store length of data
    t_out->length = (L - (9 + 2) + BLOCK1B_SIZE - 2);

    // Check length of package is sufficient
    if ((L < 12) || ((L + 1) > t_in->length)) {  // L includes all bytes except itself
        printf("M-Bus: Package too short for Length: %u\n", L);
        return false;
    }

    // Validate CRC
    crcValid(t_in->data, MIN((L - 1), (BLOCK1B_SIZE + BLOCK2B_SIZE - 2)));

    // Get data from Block 2
    memcpy(t_out->data, t_in->data, (MIN((L - 11), (BLOCK2B_SIZE - 2))) + BLOCK1B_SIZE);

    // Extract extra block for long telegrams (not tested!)
    uint8_t L_OFFSET = (BLOCK1B_SIZE + BLOCK2B_SIZE - 1);  // How much to subtract from L (127)
    if (L > (L_OFFSET + 2)) {                              // Any more data? (besided 2 extra CRC)
        // Validate CRC
        if (!crcValid((t_in->data + BLOCK1B_SIZE + BLOCK2B_SIZE), (L - L_OFFSET - 2))) {
            return false;
        }

        // Get Block 3
        memcpy((t_out->data + (BLOCK2B_SIZE - 2)), (t_in->data + BLOCK2B_SIZE), (L - L_OFFSET - 2));

        t_out->length -= 2;   // Subtract the two extra CRC bytes
    }
    return true;
}


static bool decode3OutOf6(uint8_t *t_encodedData, uint8_t *t_decodedData, bool t_lastByte = false) {
  uint8_t data[4];

  if (t_lastByte) {  // If last byte, ignore postamble sequence
    data[0] = 0x00;
    data[1] = 0x00;
  }
  else {  // Perform decoding on the encoded data
    data[0] = decode3of6((*(t_encodedData + 2) & 0x3F)); 
    data[1] = decode3of6(((*(t_encodedData + 2) & 0xC0) >> 6) | ((*(t_encodedData + 1) & 0x0F) << 2));
  }

  data[2] = decode3of6(((*(t_encodedData + 1) & 0xF0) >> 4) | ((*t_encodedData & 0x03) << 4));
  data[3] = decode3of6(((*t_encodedData & 0xFC) >> 2));

  // Check for possible errors
  if ( (data[0] == 0xFF) | (data[1] == 0xFF) |
       (data[2] == 0xFF) | (data[3] == 0xFF) ) {
    return false;
  }

  // Prepare decoded output
  *t_decodedData = (data[3] << 4) | (data[2]);
  if (!t_lastByte) {
    *(t_decodedData + 1) = (data[1] << 4) | (data[0]);
  }

  return true;
} 

static bool decode3OutOf6(m_bus_data_t *t_data,  uint16_t packetSize) {
  // We can decode "in place"
  uint8_t *encodedData = t_data->data;
  uint8_t *decodedData = t_data->data; 

  uint16_t bytesDecoded{0};
  uint16_t bytesRemaining{packetSize};

  // Decode packet      
  while (bytesRemaining) {
    // If last byte
    if (bytesRemaining == 1) {
      if(!decode3OutOf6(encodedData, decodedData, true)) {
        printf("decodingStatus: error 01\n");
        return false;
      }
      bytesRemaining -= 1;
      bytesDecoded   += 1;
    }
    else {
      if(!decode3OutOf6(encodedData, decodedData)) {
        printf("decodingStatus: error 02\n");
        return false;
      }
      bytesRemaining -= 2;
      bytesDecoded   += 2;

      encodedData += 3;
      decodedData += 2;
    }
  }
  t_data->length = bytesDecoded;
  std::fill((std::begin(t_data->data) + t_data->length), std::end(t_data->data), 0);
  return true;
}

uint16_t byteSize(uint16_t t_packetSize) {
  // In T-mode data is 3 out of 6 coded.
  uint16_t size = (( 3 * t_packetSize) / 2);

  // If packetsize is a odd number 1 extra byte   
  // that includes the 4-postamble sequence must be
  // read.    
  if (t_packetSize % 2) {
    return (size + 1);
  }
  else {
    return (size);
  }
}

uint16_t packetSize(uint8_t t_L) {
  uint16_t nrBytes;
  uint8_t  nrBlocks;

  // The 2 first blocks contains 25 bytes when excluding CRC and the L-field
  // The other blocks contains 16 bytes when excluding the CRC-fields
  // Less than 26 (15 + 10) 
  if ( t_L < 26 ) {
    nrBlocks = 2;
  }
  else { 
    nrBlocks = (((t_L - 26) / 16) + 3);
  }

  // Add all extra fields, excluding the CRC fields
  nrBytes = t_L + 1;

  // Add the CRC fields, each block is contains 2 CRC bytes
  nrBytes += (2 * nrBlocks);

  return nrBytes;
}


//

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
    // {
    //   using namespace esphome;
    //   ESP_LOGD(TAG_L, "wMBus-lib: CC1101 initialized");
    // }
    esphome::ESP_LOGD(TAG_L, "wMBus-lib: CC1101 initialized");
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
    case INIT_RX:
      {
        start();
      }
      return false;

     // RX active, awaiting SYNC
    case WAIT_FOR_SYNC:
      if (digitalRead(this->gdo2)) {
        RXinfo.state = WAIT_FOR_DATA;
        sync_time_ = millis();
      }
      break;

    // awaiting pkt len to read
    case WAIT_FOR_DATA:
      if (digitalRead(this->gdo0)) {
        // Read the 3 first bytes,
        ELECHOUSE_cc1101.SpiReadBurstReg(CC1101_RXFIFO, data_in.data, 3);
        const uint8_t *currentByte = data_in.data;
        // Mode C
        if (*currentByte == 0x54) {
          currentByte++;
          RXinfo.framemode = WMBUS_C1_MODE;
          if (*currentByte == 0xCD) {
            currentByte++;
            uint8_t L = *currentByte;
            RXinfo.frametype = WMBUS_FRAMEA;
            RXinfo.lengthField = L;
            if (L < 9) {
              RXinfo.state = INIT_RX;
              return false;
            }
            RXinfo.length = packetSize(L);
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
              RXinfo.state = INIT_RX;
              return false;
            }
            RXinfo.length = 2 + 1 + L;
            {
              using namespace esphome;
              ESP_LOGD(TAG_L, "Will have %d total bytes", RXinfo.length);
            }
          } else {
            // Unknown type, reset.
            RXinfo.state = INIT_RX;
            return false;
          }
        // T-Mode
        // Possible improvment: Check the return value from the deocding function,
        // and abort RX if coding error.
        // } else if (decode3outof6(RXinfo.pByteIndex, bytesDecoded, 0) != DECODING_3OUTOF6_OK) {
        } else if (!decode3OutOf6(RXinfo.pByteIndex, bytesDecoded)) {
          RXinfo.state = INIT_RX;
          return false;
        } else {
          // {
          //   using namespace esphome;
          //   ESP_LOGD(TAG_L, "Mode T1 frame type A");
          // }
          uint8_t L = bytesDecoded[0];
          RXinfo.framemode = WMBUS_T1_MODE;
          RXinfo.frametype = WMBUS_FRAMEA;
          RXinfo.lengthField = L;
          RXinfo.length = byteSize(packetSize(L));
          // {
          //   using namespace esphome;
          //   ESP_LOGD(TAG_L, "Will have %d total bytes L  %", RXinfo.length, L);
          // }
          // RXinfo.lengthField = bytesDecoded[0];
          // RXinfo.length = byteSize(packetSize(RXinfo.lengthField));
          // {
          //   using namespace esphome;
          //   ESP_LOGD(TAG_L, "Will have %d total bytes L' %d", RXinfo.length, );
          // }
        }

        // check if incoming data will fit into buffer
        if (RXinfo.length>sizeof(this->MBbytes)) {
          RXinfo.state = INIT_RX;
          return false;
        }

        // we got the length: now start setup chip to receive this much data
        // - Length mode -
        ELECHOUSE_cc1101.SpiWriteReg(CC1101_PKTLEN, (uint8_t)(RXinfo.length));
        ELECHOUSE_cc1101.SpiWriteReg(CC1101_PKTCTRL0, FIXED_PACKET_LENGTH);

        RXinfo.pByteIndex += 3;
        RXinfo.bytesLeft   = RXinfo.length - 3;

        RXinfo.state = READ_DATA;
        max_wait_time_ += extra_time_;

        ELECHOUSE_cc1101.SpiWriteReg(CC1101_FIFOTHR, RX_FIFO_THRESHOLD);
      }
      break;

    // awaiting more data to be read
    case READ_DATA:
      // {
      //   using namespace esphome;
      //   ESP_LOGD(TAG_L, "Waiting for more data from CC1101 FIFO");
      // }
      if (digitalRead(this->gdo0)) {
        // {
        //   using namespace esphome;
        //   ESP_LOGD(TAG_L, "Reading more data from CC1101 FIFO");
        // }
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
  if ((!overfl) && (!digitalRead(gdo2)) && (RXinfo.state > WAIT_FOR_SYNC)) {
    // {
    //   using namespace esphome;
    //   ESP_LOGD(TAG_L, "Reading last data from CC1101 FIFO");
    // }
    ELECHOUSE_cc1101.SpiReadBurstReg(CC1101_RXFIFO, RXinfo.pByteIndex, (uint8_t)RXinfo.bytesLeft);

    // decode
    uint16_t rxStatus = 1;
    uint16_t rxLength = 0;
    {
      using namespace esphome;
      ESP_LOGD(TAG_L, "\n\nRX bytes %d, L %d (%02X), total frame length %d", RXinfo.length, RXinfo.lengthField, RXinfo.lengthField, packetSize(RXinfo.lengthField));
    }

    if (RXinfo.framemode == WMBUS_T1_MODE) {
      {
        using namespace esphome;
        ESP_LOGD(TAG_L, "wMBus-lib: Processing T1 A frame");
      }
      // rxStatus = decodeRXBytesTmode(this->MBbytes, this->MBpacket, packetSize(RXinfo.lengthField));
      // rxLength = packetSize(this->MBpacket[0]);
      //
      // przepisz dane z bufora
      data_in.length = RXinfo.length;
      for (int i = 0; i < RXinfo.length; i++) {
        data_in.data[i] = MBbytes[i];
      }

      std::vector<unsigned char> RawFrame(data_in.data, data_in.data + data_in.length);
      std::string rawTelegram = format_my_hex_pretty(RawFrame);
      {
        using namespace esphome;
        ESP_LOGD(TAG_L, "RAW Frame: %s", rawTelegram.c_str());
      }

      if (!decode3OutOf6(&data_in, packetSize(RXinfo.lengthField))) {
        {
          using namespace esphome;
          ESP_LOGD(TAG_L, "wMBus-lib: blad dekodowania 3z6");
        }
        rxStatus = 11;
        // return RXinfo.complete;
      }
      std::vector<unsigned char> T1Frame(data_in.data, data_in.data + data_in.length);
      std::string telegram = format_my_hex_pretty(T1Frame);
      {
        using namespace esphome;
        ESP_LOGD(TAG_L, "CRC Frame: %s", telegram.c_str());
      }
      // Decode
      if (!mBusDecodeFormatA(&data_in, &data_out)) {
        {
          using namespace esphome;
          ESP_LOGD(TAG_L, "wMBus-lib: blad dekodowania");
        }
        rxStatus = 22;
        // return RXinfo.complete;
      }
      //

      // rxStatus = 1;
    } else if (RXinfo.framemode == WMBUS_C1_MODE) {
      if (RXinfo.frametype == WMBUS_FRAMEA) {
        {
          using namespace esphome;
          ESP_LOGD(TAG_L, "wMBus-lib: Processing C1 A frame");
        }
      } else if (RXinfo.frametype == WMBUS_FRAMEB) {
        {
          using namespace esphome;
          ESP_LOGD(TAG_L, "wMBus-lib: Processing C1 B frame");
        }
      }
    }

    if (rxStatus == 1) {
      {
        using namespace esphome;
        ESP_LOGD(TAG_L, "Packet OK.");
      }
      this->returnFrame.framemode = RXinfo.framemode;
      RXinfo.complete = true;
      this->returnFrame.rssi = (int8_t)ELECHOUSE_cc1101.getRssi();
      this->returnFrame.lqi = (uint8_t)ELECHOUSE_cc1101.getLqi();
    }
    else if (rxStatus == 11) {
      {
        using namespace esphome;
        ESP_LOGD(TAG_L, "wMBus-lib:  Error during decoding '3 out of 6'");
      }
      // Serial.println("wMBus-lib:  Error during decoding '3 out of 6'");
    }
    else if (rxStatus == 22) {
      {
        using namespace esphome;
        ESP_LOGD(TAG_L, "wMBus-lib:  Error during decoding 'CRC'");
      }
      // Serial.println("wMBus-lib:  Error during decoding 'CRC'");
    }
    else {
      {
        using namespace esphome;
        ESP_LOGD(TAG_L, "wMBus-lib:  Error during decoding 'unknown'");
      }
      // Serial.println("wMBus-lib:  Error during decoding 'unknown'");
    }
    RXinfo.state = INIT_RX;
    return RXinfo.complete;
  }
  start(false);

  return RXinfo.complete;
}


    WMbusFrame get_frame() {
  // uint8_t len_without_crc = crcRemove(this->MBpacket, packetSize(this->MBpacket[0]));
  std::vector<unsigned char> frame(data_out.data, data_out.data + data_out.length);
  std::string telegram = format_my_hex_pretty(frame);
  {
    using namespace esphome;
    ESP_LOGD(TAG_L, "    Frame: %s", telegram.c_str());
  }
  this->returnFrame.frame = frame;
  return this->returnFrame;
}


  private:

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
  RXinfo.state = INIT_RX;
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

  std::fill( std::begin( data_in.data ), std::end( data_in.data ), 0 );

  // Set RX FIFO threshold to 4 bytes
  ELECHOUSE_cc1101.SpiWriteReg(CC1101_FIFOTHR, RX_FIFO_START_THRESHOLD);
  // Set infinite length 
  ELECHOUSE_cc1101.SpiWriteReg(CC1101_PKTCTRL0, INFINITE_PACKET_LENGTH);

  ELECHOUSE_cc1101.SpiStrobe(CC1101_SRX);
  while((ELECHOUSE_cc1101.SpiReadStatus(CC1101_MARCSTATE) != MARCSTATE_RX));

  RXinfo.state = WAIT_FOR_SYNC;

  return 1; // this will indicate we just have re-started RX
}

    uint8_t gdo0{0};
    uint8_t gdo2{0};
    
    uint8_t MBbytes[584];
    uint8_t MBpacket[291];

    m_bus_data_t data_in{0};  // Data from Physical layer decoded to bytes
    m_bus_data_t data_out{0}; // Data for Data Link layer

    WMbusFrame returnFrame;

    RXinfoDescr RXinfo;

    uint32_t sync_time_{0};
    uint8_t extra_time_{200};
    uint8_t max_wait_time_ = extra_time_;

};

