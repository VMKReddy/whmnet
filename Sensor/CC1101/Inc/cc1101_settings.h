#ifndef CC1101_SETTINGS_H
#define CC1101_SETTINGS_H

#include "cc1101.h" /* Registers defs */
#include "cc1101_api.h" /* registerSettings_t type */

/* Import from SmartRfStudio */

// Packet length mode = Variable 
// Device address = 0 
// Symbol rate = 1.19948
// Modulation format = 2-GFSK
// TX power = 12dBm
// Address config = No address check 
// PA ramping = false
// Whitening = false 
// Packet bit length = 0 
// Carrier frequency = 867.999939
// Bit rate = 1.19948
// Deviation = 5.157471
// Manchester enable = false 
// Packet length = variable (max 255)

static const registerSetting_t cc1101Settings[]=
{
  {CC1101_IOCFG2,            0x2E}, //high Z
  {CC1101_IOCFG1,            0x2E}, //high Z
  {CC1101_IOCFG0,            0x06}, // RX/TX sync
  {CC1101_FIFOTHR,           0x47},
  {CC1101_SYNC1,             0xD3},
  {CC1101_SYNC0,             0x91},
  {CC1101_PKTLEN,            0xFF},
  {CC1101_PKTCTRL1,          0x04},
  {CC1101_PKTCTRL0,          0x05},
  {CC1101_ADDR,              0x00},
  {CC1101_CHANNR,            0x00},
  {CC1101_FSCTRL1,           0x06},
  {CC1101_FSCTRL0,           0x00},
  {CC1101_FREQ2,             0x21},
  {CC1101_FREQ1,             0x62},
  {CC1101_FREQ0,             0x76},
  {CC1101_MDMCFG4,           0xF5},
  {CC1101_MDMCFG3,           0x83},
  {CC1101_MDMCFG2,           0x93}, //rx optimized for power consumption
  {CC1101_MDMCFG1,           0x22},
  {CC1101_MDMCFG0,           0xF8},
  {CC1101_DEVIATN,           0x15},
  {CC1101_MCSM2,             0x07},
  {CC1101_MCSM1,             0x30},
  {CC1101_MCSM0,             0x18},
  {CC1101_FOCCFG,            0x16},
  {CC1101_BSCFG,             0x6C},
  {CC1101_AGCCTRL2,          0x03},
  {CC1101_AGCCTRL1,          0x40},
  {CC1101_AGCCTRL0,          0x91},
  {CC1101_WOREVT1,           0x87},
  {CC1101_WOREVT0,           0x6B},
  {CC1101_WORCTRL,           0xFB},
  {CC1101_FREND1,            0x56},
  {CC1101_FREND0,            0x10},
  {CC1101_FSCAL3,            0xE9},
  {CC1101_FSCAL2,            0x2A},
  {CC1101_FSCAL1,            0x00},
  {CC1101_FSCAL0,            0x1F},
  {CC1101_RCCTRL1,           0x41},
  {CC1101_RCCTRL0,           0x00},
  {CC1101_FSTEST,            0x59},
  {CC1101_PTEST,             0x7F},
  {CC1101_AGCTEST,           0x3F},
  {CC1101_TEST2,             0x81},
  {CC1101_TEST1,             0x35},
  {CC1101_TEST0,             0x09},
  {CC1101_PA_TABLE0,         0xC0},
};




#endif
