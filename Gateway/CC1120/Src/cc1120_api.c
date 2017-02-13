#include "cc1120_api.h"
#include "cc1120.h"
#include "cc1120_settings.h"
#include "sleep.h"
#include "cc1120_conf.h"

#define RX_BUF_MAX_SIZE         (64)
#define PKT_TX_TIMEOUT_MS       (200)

/* Variable used to signal main process that rx/tx interrupt occured */
volatile uint8_t pktSemaphore = 0;
uint8_t *rxBufferCC1120;

/*
 * Prototypes
 */
static uint8_t cc1120_Reset(void);
static uint8_t cc1120_checkSpiCom(void);

uint8_t cc1120_open(SPI_HandleTypeDef *spi, uint8_t *rxbuf)
{
    uint8_t retval;

	/* Assign spi handle */
	cc1120_SPI_Init(spi);

	/* Do hard reset to the chip to start from a clean status */
	retval = cc1120_Reset();
    if (retval) {
        return CC1120_ERR_RESET;
    }

    /* Init rx buffer pointer */
    rxBufferCC1120 = rxbuf;

    /* Reinit packet semaphore */
    pktSemaphore = PKT_SEM_IDLE;
	
	/* Check spi com by checking partnumber register */
	return cc1120_checkSpiCom();
}

uint8_t cc1120_sleep()
{
    rfStatus_t status;
    return trxSpiCmdStrobe(CC112X_SPWD, &status);
}

static uint8_t cc1120_checkSpiCom(void)
{
	uint8_t rxData;
    uint8_t retval;
    rfStatus_t status;

	retval = cc112xSpiReadReg(CC112X_PARTNUMBER, &rxData, 1, &status); 
    if (retval) {
        return CC1120_ERR_SPI;
    }

	if (rxData == 0x48) {
		return 0;
	} else {
		return CC1120_ERR_PN;
	}
}

uint8_t cc1120_getPartVersion(uint8_t *partRev)
{
    rfStatus_t status;
    return cc112xSpiReadReg(CC112X_PARTVERSION, partRev, 1, &status);
}

static uint8_t cc1120_Reset(void)
{
    rfStatus_t status;

	/* Hard reset */
	cc1120_GpioReset();	

	/* Soft reset */
	return trxSpiCmdStrobe(CC112X_SRES, &status);
}

uint8_t cc1120_IdleToRx(void)
{
    uint8_t retval;
    rfStatus_t status;

	/* Make sure we are idle */
	retval = trxSpiCmdStrobe(CC112X_SIDLE, &status);
	if (retval) {
        return CC1120_ERR_SIDLE;
    }

    /* Flush the rx fifo */
	retval = trxSpiCmdStrobe(CC112X_SFRX, &status);
	if (retval) {
        return CC1120_ERR_SFRX;
    }

    /* Reinit packet semaphore */
    pktSemaphore = PKT_SEM_RX;    

	/* Move to Rx */
	retval = trxSpiCmdStrobe(CC112X_SRX, &status);
	if (retval) {
        return CC1120_ERR_SRX;
    }

    return 0;
}

uint8_t cc1120_RxTxToIdle(void)
{
    uint8_t retval;
    rfStatus_t status;

	/* Go to idle */
	retval = trxSpiCmdStrobe(CC112X_SIDLE, &status);
	if (retval) {
        return CC1120_ERR_SIDLE;
    }

    pktSemaphore = PKT_SEM_IDLE;

    return 0;
}

uint8_t cc1120_registerConfig(void)
{
  uint16_t i; 
  uint8_t writeByte;
  uint8_t retval;
  rfStatus_t status;

  /* Write each register to CC1120 */
  for(i=0; i<(sizeof(cc1120Settings)/sizeof(registerSetting_t)); i++) {
      writeByte = cc1120Settings[i].data;
      retval = cc112xSpiWriteReg(cc1120Settings[i].addr, &writeByte, 1, &status);
	  if (retval) {
          return CC1120_ERR_SPI;
      }
  }

  return 0;
}


uint8_t cc1120_sendPacket(uint8_t *pData, uint8_t length)
{
	rfStatus_t status;
    uint8_t retval;

	/* Write packet to Tx FIFO */
	/* First byte in pData must be length of packet */
	retval = cc112xSpiWriteTxFifo(pData, length, &status);
	if (retval) {
        return CC1120_ERR_SPI;
    }

    /* Check status, return if not IDLE */
    if ( (status & 0x70) != CC112X_STATE_IDLE ) {
        return CC1120_ERR_STATE;
    }

	/* Update packet semaphore */
	pktSemaphore = PKT_SEM_TXING;

	/* Transmit packet */
	retval = trxSpiCmdStrobe(CC112X_STX, &status);
	if (retval) {
        return CC1120_ERR_STX;
    }

	/* Go to sleep while packet is being transmitted, with timeout */
    retval = doSleepStop_WakeRTC_RTCCLK(PKT_TX_TIMEOUT_MS);
    if (retval) {
        /* We did not succeed in entering stop mode */
        return CC1120_ERR_STOP_MODE;
    }

    /* If there is a problem during TX, we will get there after waking up
       from a timeout condition instead of falling edge EXTI */

    /* Check if we have a timeout condition */
    if (sleepExitOnTimeout()) {
        /* Report error */
        return CC1120_ERR_TIMEOUT;
    } else {
        /* Check pktSemaphore value */
        if (pktSemaphore == PKT_SEM_TXING) {
            /* Case where MCU failed entering stop mode: no timeout
                but no EXTI interrupt either */
            return CC1120_ERR_NO_SLEEP;     
        } else if (pktSemaphore != PKT_SEM_IDLE) {
            /* Unknown cause of wrong value */
            return CC1120_ERR_PKT_SEM;
        }
    }

    /* Check status is ok: should by IDLE (IDLE after Tx) */
	retval = cc112xGetTxStatus(&status);
	if (retval) {
        return CC1120_ERR_SPI;
    }
	/*  if status is not right, report error */
    if ( (status & 0x70) != CC112X_STATE_IDLE) {
        /* Report error */
        return CC1120_ERR_PKT_TX;
    } else {
        /* We are in IDLE, put radio to SLEEP mode */
        retval = trxSpiCmdStrobe(CC112X_SPWD, &status);
	    if (retval) {
            return CC1120_ERR_SPWD;
        }

        /* Packet sent successfully */
        return 0;
    }
}

uint8_t gotoSafeModeAfterError(void)
{
    uint8_t retval;
    rfStatus_t status;   
 
    /* Make sure we are idle */
    retval = trxSpiCmdStrobe(CC112X_SIDLE, &status);
    if (retval) {
        return CC1120_ERR_SIDLE;
    }
    /* Flush the tx fifo */
    retval = trxSpiCmdStrobe(CC112X_SFTX, &status);
	if (retval) {
        return CC1120_ERR_SFTX;
    }
    /* Flush the rx fifo */
    retval = trxSpiCmdStrobe(CC112X_SFRX, &status);
	if (retval) {
        return CC1120_ERR_SFRX;
    }
    /* Put radio to SLEEP mode */
    retval = trxSpiCmdStrobe(CC112X_SPWD, &status);
	if (retval) {
        return CC1120_ERR_SPWD;
    }

    return 0;
}

void cc1120_pktSyncRxTx_Callback()
{
	rfStatus_t status;
    uint8_t retval;
    uint8_t rxBytes;
    uint8_t marcState;

	switch (pktSemaphore) {

	case PKT_SEM_TXING:
        /* Write packet semaphore */
        pktSemaphore = PKT_SEM_IDLE;
	    break;

    /* We are supposed to be in RX mode */
    case PKT_SEM_RX:

        /* Read number of bytes available in rx fifo */
        retval = cc112xSpiReadReg(CC112X_NUM_RXBYTES, &rxBytes, 1, &status);        
        if (retval) {
            pktSemaphore = PKT_SEM_ERR_SPI;
            break;
        }        

        /* If there are bytes in fifo */
        if (rxBytes == 0) {
            pktSemaphore = PKT_SEM_ERR_ZERO_LENGTH;
        } else if (rxBytes > CC1120_MAX_RX_SIZE) {
            pktSemaphore = PKT_SEM_ERR_MAX_SIZE;
        } else {
            /* Read MARCSTATE to check for RX FIFO error */
            retval = cc112xSpiReadReg(CC112X_MARCSTATE, &marcState, 1, &status);
            if (retval) {
                pktSemaphore = PKT_SEM_ERR_SPI;
                break;
            }

            /* Mask out MARCSTATE bits and check if we have a RX FIFO error */
            if((marcState & 0x1F) == 0x11) {
                pktSemaphore = PKT_SEM_ERR_RXFIFO;
            } else {
                // Read rxBytes bytes from RX FIFO
                retval = cc112xSpiReadRxFifo(rxBufferCC1120, rxBytes, &status);
                if (retval) {
                    pktSemaphore = PKT_SEM_ERR_SPI;
                    break;
                }

                // Check CRC ok (CRC_OK: bit7 in second status byte)
                // This assumes status bytes are appended in RX_FIFO
                // (PKT_CFG1.APPEND_STATUS = 1)
                // If CRC is disabled the CRC_OK field will read 1
                if(rxBufferCC1120[rxBytes - 1] & 0x80) {
                    /* Update packet semaphore */
                    pktSemaphore = PKT_SEM_MSG_RCVD;
                } else {
                    pktSemaphore = PKT_SEM_ERR_CRC;
                }
            }
        }

        break;

    default:
        /* Do nothing. We should never get there in normal operation. */
        break;    

    }
}

/*******************************************************************************
*   @fn         manualCalibration
*
*   @brief      Calibrates radio according to CC112x errata
*
*   @param      none
*
*   @return     none
*/
#define VCDAC_START_OFFSET 2
#define FS_VCO2_INDEX 0
#define FS_VCO4_INDEX 1
#define FS_CHP_INDEX 2
void cc1120_manualCalibration(void) {
  rfStatus_t status;
  uint8_t original_fs_cal2;
  uint8_t calResults_for_vcdac_start_high[3];
  uint8_t calResults_for_vcdac_start_mid[3];
  uint8_t marcstate;
  uint8_t writeByte;

  // 1) Set VCO cap-array to 0 (FS_VCO2 = 0x00)
  writeByte = 0x00;
  cc112xSpiWriteReg(CC112X_FS_VCO2, &writeByte, 1, &status);

  // 2) Start with high VCDAC (original VCDAC_START + 2):
  cc112xSpiReadReg(CC112X_FS_CAL2, &original_fs_cal2, 1, &status);
  writeByte = original_fs_cal2 + VCDAC_START_OFFSET;
  cc112xSpiWriteReg(CC112X_FS_CAL2, &writeByte, 1, &status);

  // 3) Calibrate and wait for calibration to be done
  //   (radio back in IDLE state)
  trxSpiCmdStrobe(CC112X_SCAL, &status);

  do {
      cc112xSpiReadReg(CC112X_MARCSTATE, &marcstate, 1, &status);
  } while (marcstate != 0x41);

  // 4) Read FS_VCO2, FS_VCO4 and FS_CHP register obtained with 
  //    high VCDAC_START value
  cc112xSpiReadReg(CC112X_FS_VCO2,
                   &calResults_for_vcdac_start_high[FS_VCO2_INDEX], 1, &status);
  cc112xSpiReadReg(CC112X_FS_VCO4,
                   &calResults_for_vcdac_start_high[FS_VCO4_INDEX], 1, &status);
  cc112xSpiReadReg(CC112X_FS_CHP,
                   &calResults_for_vcdac_start_high[FS_CHP_INDEX], 1, &status);

  // 5) Set VCO cap-array to 0 (FS_VCO2 = 0x00)
  writeByte = 0x00;
  cc112xSpiWriteReg(CC112X_FS_VCO2, &writeByte, 1, &status);

  // 6) Continue with mid VCDAC (original VCDAC_START):
  writeByte = original_fs_cal2;
  cc112xSpiWriteReg(CC112X_FS_CAL2, &writeByte, 1, &status);

  // 7) Calibrate and wait for calibration to be done
  //   (radio back in IDLE state)
  trxSpiCmdStrobe(CC112X_SCAL, &status);

  do {
    cc112xSpiReadReg(CC112X_MARCSTATE, &marcstate, 1, &status);
  } while (marcstate != 0x41);

  // 8) Read FS_VCO2, FS_VCO4 and FS_CHP register obtained 
  //    with mid VCDAC_START value
  cc112xSpiReadReg(CC112X_FS_VCO2,
                   &calResults_for_vcdac_start_mid[FS_VCO2_INDEX], 1, &status);
  cc112xSpiReadReg(CC112X_FS_VCO4,
                   &calResults_for_vcdac_start_mid[FS_VCO4_INDEX], 1, &status);
  cc112xSpiReadReg(CC112X_FS_CHP,
                   &calResults_for_vcdac_start_mid[FS_CHP_INDEX], 1, &status);

  // 9) Write back highest FS_VCO2 and corresponding FS_VCO
  //    and FS_CHP result
  if (calResults_for_vcdac_start_high[FS_VCO2_INDEX] >
    calResults_for_vcdac_start_mid[FS_VCO2_INDEX]) {
    writeByte = calResults_for_vcdac_start_high[FS_VCO2_INDEX];
    cc112xSpiWriteReg(CC112X_FS_VCO2, &writeByte, 1, &status);
    writeByte = calResults_for_vcdac_start_high[FS_VCO4_INDEX];
    cc112xSpiWriteReg(CC112X_FS_VCO4, &writeByte, 1, &status);
    writeByte = calResults_for_vcdac_start_high[FS_CHP_INDEX];
    cc112xSpiWriteReg(CC112X_FS_CHP, &writeByte, 1, &status);
  } else {
    writeByte = calResults_for_vcdac_start_mid[FS_VCO2_INDEX];
    cc112xSpiWriteReg(CC112X_FS_VCO2, &writeByte, 1, &status);
    writeByte = calResults_for_vcdac_start_mid[FS_VCO4_INDEX];
    cc112xSpiWriteReg(CC112X_FS_VCO4, &writeByte, 1, &status);
    writeByte = calResults_for_vcdac_start_mid[FS_CHP_INDEX];
    cc112xSpiWriteReg(CC112X_FS_CHP, &writeByte, 1, &status);
  }
}

