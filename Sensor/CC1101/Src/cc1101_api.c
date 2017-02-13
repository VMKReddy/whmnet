#include "cc1101_api.h"
#include "cc1101.h"
#include "cc1101_settings.h"
#include "sleep.h"
#include "cc1101_conf.h"

#define RX_BUF_MAX_SIZE         (64)
/* Adjust timeout depending on RF bitrate and application max packet length */
#define PKT_TX_TIMEOUT_MS       (150)

/* Variable used to signal main process that rx/tx interrupt occured */
volatile uint8_t pktSemaphore = 0;
uint8_t *rxBufferCC1101;

/*
 * Prototypes
 */
static uint8_t cc1101_Reset(void);
static uint8_t cc1101_checkSpiCom(void);

uint8_t cc1101_open(SPI_HandleTypeDef *spi, uint8_t *rxbuf)
{
    uint8_t retval;

	/* Assign spi handle */
	cc1101_SPI_Init(spi);

	/* Do hard reset to the chip to start from a clean status */
	retval = cc1101_Reset();
    if (retval) {
        return CC1101_ERR_RESET;
    }

    /* Init rx buffer pointer */
    rxBufferCC1101 = rxbuf;

    /* Reinit packet semaphore */
    pktSemaphore = PKT_SEM_IDLE;
	
	/* Check spi com by checking partnumber register */
	return cc1101_checkSpiCom();
}

uint8_t cc1101_sleep()
{
    rfStatus_t status;
    return trxSpiCmdStrobe(CC1101_SPWD, &status);
}

static uint8_t cc1101_checkSpiCom(void)
{
	uint8_t rxData;
    uint8_t retval;
    rfStatus_t status;

	retval = cc1101SpiReadReg(CC1101_VERSION, &rxData, 1, &status);
    if (retval) {
        return CC1101_ERR_SPI;
    }

    /* Should be 0x14 (new package) or 0x4 (old package) */
	if (rxData == 0x14 || rxData == 0x4) {
		return 0;
	} else {
		return CC1101_ERR_PN;
	}
}

uint8_t cc1120_getPartVersion(uint8_t *partRev)
{
    rfStatus_t status;
    return cc1101SpiReadReg(CC1101_VERSION, partRev, 1, &status);
}

static uint8_t cc1101_Reset(void)
{
    rfStatus_t status;

	/* Soft reset */
	return trxSpiCmdStrobe(CC1101_SRES, &status);
}

uint8_t cc1101_IdleToRx(void)
{
    uint8_t retval;
    rfStatus_t status;

	/* Make sure we are idle */
	retval = trxSpiCmdStrobe(CC1101_SIDLE, &status);
	if (retval) {
        return CC1101_ERR_SIDLE;
    }

    /* Flush the rx fifo */
	retval = trxSpiCmdStrobe(CC1101_SFRX, &status);
	if (retval) {
        return CC1101_ERR_SFRX;
    }

    /* Reinit packet semaphore */
    pktSemaphore = PKT_SEM_RX;    

	/* Move to Rx */
	retval = trxSpiCmdStrobe(CC1101_SRX, &status);
	if (retval) {
        return CC1101_ERR_SRX;
    }

    return 0;
}

uint8_t cc1101_RxTxToIdle(void)
{
    uint8_t retval;
    rfStatus_t status;

	/* Go to idle */
	retval = trxSpiCmdStrobe(CC1101_SIDLE, &status);
	if (retval) {
        return CC1101_ERR_SIDLE;
    }

    pktSemaphore = PKT_SEM_IDLE;

    return 0;
}

uint8_t cc1101_registerConfig(void)
{
  uint16_t i; 
  uint8_t writeByte;
  uint8_t retval;
  rfStatus_t status;

  /* Write each register to CC1120 */
  for(i=0; i<(sizeof(cc1101Settings)/sizeof(registerSetting_t)); i++) {
      writeByte = cc1101Settings[i].data;
      retval = cc1101SpiWriteReg(cc1101Settings[i].addr, &writeByte, 1, &status);
	  if (retval) {
          return CC1101_ERR_SPI;
      }
  }

  return 0;
}


uint8_t cc1101_sendPacket(uint8_t *pData, uint8_t length)
{
	rfStatus_t status;
    uint8_t retval;

	/* Write packet to Tx FIFO */
	/* First byte in pData must be length of packet */
	retval = cc1101SpiWriteTxFifo(pData, length, &status);
	if (retval) {
        return CC1101_ERR_SPI;
    }

    /* Check status, return if not IDLE */
    if ( (status & 0x70) != CC1101_STATE_IDLE ) {
        return CC1101_ERR_STATE;
    }

	/* Update packet semaphore */
	pktSemaphore = PKT_SEM_TXING;

	/* Transmit packet */
	retval = trxSpiCmdStrobe(CC1101_STX, &status);
	if (retval) {
        return CC1101_ERR_STX;
    }

	/* Go to sleep while packet is being transmitted, with timeout */
    retval = doSleepStop_WakeRTC_RTCCLK(PKT_TX_TIMEOUT_MS);
    if (retval) {
        /* We did not succeed in entering stop mode */
        return CC1101_ERR_STOP_MODE;
    }

    /* If there is a problem during TX, we will get there after waking up
       from a timeout condition instead of falling edge EXTI */

    /* Check if we have a timeout condition */
    if (sleepExitOnTimeout()) {
        /* Report error */
        return CC1101_ERR_TIMEOUT;
    } else {
        /* Check pktSemaphore value */
        if (pktSemaphore == PKT_SEM_TXING) {
            /* Case where MCU failed entering stop mode: no timeout
                but no EXTI interrupt either */
            return CC1101_ERR_NO_SLEEP;
        } else if (pktSemaphore != PKT_SEM_IDLE) {
            /* Unknown cause of wrong value */
            return CC1101_ERR_PKT_SEM;
        }
    }

    /* Need to wait a bit otherwise chip status byte still indicates TX */
    retval = doSleepStop_WakeRTC_RTCCLK(1);
    if (retval) {
        return CC1101_ERR_STOP_MODE;
    }

    /* Check status is ok: should by IDLE (IDLE after Tx) */
	retval = cc1101GetTxStatus(&status);
	if (retval) {
        return CC1101_ERR_SPI;
    }
	/*  if status is not right, report error */
    if ( (status & 0x70) != CC1101_STATE_IDLE) {
        /* Report error */
        return CC1101_ERR_PKT_TX;
    } else {
        /* We are in IDLE, put radio to SLEEP mode */
        retval = trxSpiCmdStrobe(CC1101_SPWD, &status);
	    if (retval) {
            return CC1101_ERR_SPWD;
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
    retval = trxSpiCmdStrobe(CC1101_SIDLE, &status);
    if (retval) {
        return CC1101_ERR_SIDLE;
    }
    /* Flush the tx fifo */
    retval = trxSpiCmdStrobe(CC1101_SFTX, &status);
	if (retval) {
        return CC1101_ERR_SFTX;
    }
    /* Flush the rx fifo */
    retval = trxSpiCmdStrobe(CC1101_SFRX, &status);
	if (retval) {
        return CC1101_ERR_SFRX;
    }
    /* Put radio to SLEEP mode */
    retval = trxSpiCmdStrobe(CC1101_SPWD, &status);
	if (retval) {
        return CC1101_ERR_SPWD;
    }

    return 0;
}

void cc1101_pktSyncRxTx_Callback()
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
        retval = cc1101SpiReadReg(CC1101_RXBYTES, &rxBytes, 1, &status);
        if (retval) {
            pktSemaphore = PKT_SEM_ERR_SPI;
            break;
        }        

        /* If there are bytes in fifo */
        if (rxBytes == 0) {
            pktSemaphore = PKT_SEM_ERR_ZERO_LENGTH;
        } else if (rxBytes > CC1101_MAX_RX_SIZE) {
            pktSemaphore = PKT_SEM_ERR_MAX_SIZE;
        } else {
            /* Read MARCSTATE to check for RX FIFO error */
            retval = cc1101SpiReadReg(CC1101_MARCSTATE, &marcState, 1, &status);
            if (retval) {
                pktSemaphore = PKT_SEM_ERR_SPI;
                break;
            }

            /* Mask out MARCSTATE bits and check if we have a RX FIFO error */
            if((marcState & 0x1F) == 0x11) {
                pktSemaphore = PKT_SEM_ERR_RXFIFO;
            } else {
                // Read rxBytes bytes from RX FIFO
                retval = cc1101SpiReadRxFifo(rxBufferCC1101, rxBytes, &status);
                if (retval) {
                    pktSemaphore = PKT_SEM_ERR_SPI;
                    break;
                }

                // Check CRC ok (CRC_OK: bit7 in second status byte)
                // This assumes status bytes are appended in RX_FIFO
                // (PKT_CFG1.APPEND_STATUS = 1)
                // If CRC is disabled the CRC_OK field will read 1
                if(rxBufferCC1101[rxBytes - 1] & 0x80) {
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

