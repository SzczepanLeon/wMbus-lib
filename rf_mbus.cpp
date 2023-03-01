#include "utils.hpp"
#include "rf_mbus.hpp"
#include "mbus_packet.hpp"
#include "3outof6.hpp"
#include "tmode_rf_settings.hpp"

#include <ELECHOUSE_CC1101_SRC_DRV.h>

uint8_t MBbytes[584];

RXinfoDescr RXinfo;

uint32_t sync_time_{0};
uint8_t extra_time_ = 20;
uint8_t max_wait_time_ = extra_time_;

uint8_t rf_mbus_on(bool force) {
  // waiting to long for next part of data?
  bool reinit_needed = ((millis() - sync_time_) > max_wait_time_) ? true: false;

  if (!force) {
    if (!reinit_needed) {
      // already in RX?
      if (ELECHOUSE_cc1101.SpiReadStatus(CC1101_MARCSTATE) == MARCSTATE_RX) {
        return 0;
      }
    }
  }

  // init RX here, each time we're idle
  RXinfo.state = 0;
  sync_time_ = millis();
  max_wait_time_ = extra_time_;

  ELECHOUSE_cc1101.SpiStrobe(CC1101_SIDLE);
  while((ELECHOUSE_cc1101.SpiReadStatus(CC1101_MARCSTATE) != MARCSTATE_IDLE));
  ELECHOUSE_cc1101.SpiStrobe(CC1101_SFTX);  //flush TXfifo
  ELECHOUSE_cc1101.SpiStrobe(CC1101_SFRX);  //flush RXfifo

  // Initialize RX info variable
  RXinfo.lengthField = 0;           // Length Field in the wireless MBUS packet
  RXinfo.length      = 0;           // Total length of bytes to receive packet
  RXinfo.bytesLeft   = 0;           // Bytes left to to be read from the RX FIFO
  RXinfo.pByteIndex  = MBbytes;     // Pointer to current position in the byte array
  RXinfo.complete    = false;       // Packet Received

  memset(MBbytes, 0, sizeof(MBbytes));

  // Set RX FIFO threshold to 4 bytes
  ELECHOUSE_cc1101.SpiWriteReg(CC1101_FIFOTHR, RX_FIFO_START_THRESHOLD);
  // Set infinite length 
  ELECHOUSE_cc1101.SpiWriteReg(CC1101_PKTCTRL0, INFINITE_PACKET_LENGTH);

  ELECHOUSE_cc1101.SpiStrobe(CC1101_SRX);
  while((ELECHOUSE_cc1101.SpiReadStatus(CC1101_MARCSTATE) != MARCSTATE_RX));

  RXinfo.state = 1;

  return 1; // this will indicate we just have re-started RX
}

bool rf_mbus_init(uint8_t mosi, uint8_t miso, uint8_t clk, uint8_t cs, uint8_t gdo0, uint8_t gdo2) {
  bool retVal = false;
  Serial.println("");
  pinMode(gdo0, INPUT);
  pinMode(gdo2, INPUT);
  ELECHOUSE_cc1101.setSpiPin(clk, miso, mosi, cs);

  ELECHOUSE_cc1101.Init();

  for (uint8_t i = 0; i < TMODE_RF_SETTINGS_LEN; i++) {
    ELECHOUSE_cc1101.SpiWriteReg(TMODE_RF_SETTINGS_BYTES[i << 1],
                                 TMODE_RF_SETTINGS_BYTES[(i << 1) + 1]);
  }

  ELECHOUSE_cc1101.SpiStrobe(CC1101_SCAL);

  byte cc1101Version = ELECHOUSE_cc1101.SpiReadStatus(CC1101_VERSION);

  if ((cc1101Version != 0) && (cc1101Version != 255)) {
    retVal = true;
    Serial.print("wMBus-lib: CC1101 version '");
    Serial.print(cc1101Version);
    Serial.println("'");
    ELECHOUSE_cc1101.SetRx();
    Serial.println("wMBus-lib: CC1101 initialized");
    memset(&RXinfo, 0, sizeof(RXinfo));
    delay(4);
  }
  else {
    Serial.println("wMBus-lib: CC1101 initialization FAILED!");
  }

  return retVal;
}

bool rf_mbus_task(uint8_t* MBpacket, int8_t &rssi, uint8_t &lqi, byte gdo0, byte gdo2) {
  uint8_t bytesDecoded[2];

  switch (RXinfo.state) {
    case 0:
      {
        rf_mbus_on();
      }
      return false;

     // RX active, awaiting SYNC
    case 1:
      if (digitalRead(gdo2)) {
        RXinfo.state = 2;
        sync_time_ = millis();
      }
      break;

    // awaiting pkt len to read
    case 2:
      if (digitalRead(gdo0)) {
        // Read the 3 first bytes
        ELECHOUSE_cc1101.SpiReadBurstReg(CC1101_RXFIFO, RXinfo.pByteIndex, 3);

        // - Calculate the total number of bytes to receive -
        // T-Mode
        // Possible improvment: Check the return value from the deocding function,
        // and abort RX if coding error. 
        if (decode3outof6(RXinfo.pByteIndex, bytesDecoded, 0) != DECODING_3OUTOF6_OK) {
          RXinfo.state = 0;
          return false;
	      }		
        RXinfo.lengthField = bytesDecoded[0];
        RXinfo.length = byteSize(packetSize(RXinfo.lengthField));

        // check if incoming data will fit into buffer
        if (RXinfo.length>sizeof(MBbytes)) {
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
      if (digitalRead(gdo0)) {
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
    ELECHOUSE_cc1101.SpiReadBurstReg(CC1101_RXFIFO, RXinfo.pByteIndex, (uint8_t)RXinfo.bytesLeft);

    // decode
    uint16_t rxStatus = PACKET_CODING_ERROR;
    rxStatus = decodeRXBytesTmode(MBbytes, MBpacket, packetSize(RXinfo.lengthField));

    if (rxStatus == PACKET_OK) {
      RXinfo.complete = true;
      rssi = (int8_t)ELECHOUSE_cc1101.getRssi();
      lqi = (uint8_t)ELECHOUSE_cc1101.getLqi();
    }
    else if (rxStatus == PACKET_CODING_ERROR) {
      Serial.print("wMBus-lib: Error during decoding '3 out of 6'");
    }
    else if (rxStatus == PACKET_CRC_ERROR) {
      Serial.println("wMBus-lib: Error during decoding 'CRC'");
    }
    else {
      Serial.println("wMBus-lib: Error during decoding 'unknown'");
    }
    RXinfo.state = 0;
    return RXinfo.complete;
  }
  rf_mbus_on(false);

  return RXinfo.complete;
}
