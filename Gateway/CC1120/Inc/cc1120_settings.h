#ifndef CC112X_SETTINGS_H
#define CC112X_SETTINGS_H

#include "cc1120.h" /* Registers defs */
#include "cc1120_api.h" /* registerSettings_t type */

/* Import from SmartRfStudio */

/* SYNC word and BW customized to be compatible with CC1101 Tx */

// RX filter BW = 100.000000
// Packet length mode = Variable 
// Performance mode = High Performance 
// Device address = 0 
// Symbol rate = 1.2 
// Modulation format = 2-GFSK
// TX power = 15 
// Address config = No address check 
// PA ramping = true 
// Whitening = false 
// Packet bit length = 0 
// Carrier frequency = 868.000000 
// Bit rate = 1.2 
// Deviation = 5.157471
// Manchester enable = false 
// Packet length = 255 

static const registerSetting_t cc1120Settings[]=
{
  {CC112X_IOCFG3,            0xB0},
  {CC112X_IOCFG2,            0x06},
  {CC112X_IOCFG1,            0xB0},
  {CC112X_IOCFG0,            0xB0},
  {CC112X_SYNC3,         	 0xD3},
  {CC112X_SYNC2,           	 0x91},
  {CC112X_SYNC1,     	     0xD3},
  {CC112X_SYNC0, 	         0x91},
  {CC112X_SYNC_CFG1,         0x0B},
  {CC112X_DEVIATION_M,       0x52},
  {CC112X_MODCFG_DEV_E,      0x0B},
  {CC112X_DCFILT_CFG,        0x1C},
  {CC112X_PREAMBLE_CFG1,     0x18},
  {CC112X_PREAMBLE_CFG0,     0x2A},
  {CC112X_FREQ_IF_CFG,       0x40},
  {CC112X_IQIC,              0xC6},
  {CC112X_CHAN_BW,           0x02},
  {CC112X_MDMCFG1,           0x46},
  {CC112X_MDMCFG0,           0x05},
  {CC112X_SYMBOL_RATE2,      0x43},
  {CC112X_SYMBOL_RATE1,      0xA6},
  {CC112X_SYMBOL_RATE0,      0xFC},
  {CC112X_AGC_REF,           0x20},
  {CC112X_AGC_CS_THR,        0x19},
  {CC112X_AGC_CFG1,          0xA9},
  {CC112X_AGC_CFG0,          0xCF},
  {CC112X_FIFO_CFG,          0x00},
  {CC112X_DEV_ADDR,          0x00},
  {CC112X_FS_CFG,            0x12},
  {CC112X_PKT_CFG2,          0x04},
  {CC112X_PKT_CFG1,          0x05},
  {CC112X_PKT_CFG0,          0x20},
  {CC112X_RFEND_CFG1,        0x0F}, /* Go to IDLE after Rx */
  {CC112X_RFEND_CFG0,        0x00}, /* Go to IDLE after Tx */
  {CC112X_PA_CFG2,           0x7F},
  {CC112X_PA_CFG1,           0x56},
  {CC112X_PA_CFG0,           0x7C},
  {CC112X_PKT_LEN,           0xFF},
  {CC112X_IF_MIX_CFG,        0x00},
  {CC112X_FREQOFF_CFG,       0x22},
  {CC112X_FREQ2,             0x6C},
  {CC112X_FREQ1,             0x80},
  {CC112X_FREQ0,             0x00},
  {CC112X_FS_DIG1,           0x00},
  {CC112X_FS_DIG0,           0x5F},
  {CC112X_FS_CAL1,           0x40},
  {CC112X_FS_CAL0,           0x0E},
  {CC112X_FS_DIVTWO,         0x03},
  {CC112X_FS_DSM0,           0x33},
  {CC112X_FS_DVC0,           0x17},
  {CC112X_FS_PFD,            0x50},
  {CC112X_FS_PRE,            0x6E},
  {CC112X_FS_REG_DIV_CML,    0x14},
  {CC112X_FS_SPARE,          0xAC},
  {CC112X_FS_VCO0,           0xB4},
  {CC112X_XOSC5,             0x0E},
  {CC112X_XOSC1,             0x03},
};


#endif
