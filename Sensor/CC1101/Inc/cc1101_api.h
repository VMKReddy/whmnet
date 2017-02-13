#ifndef CC1101_API_H
#define CC1101_API_H

#include "stm32l0xx_hal.h"

#define PKT_SEM_IDLE        (0)
#define PKT_SEM_TXING       (1)
#define PKT_SEM_MSG_RCVD    (2)
#define PKT_SEM_ERR_ZERO_LENGTH (3)
#define PKT_SEM_ERR_RXFIFO  (4)
#define PKT_SEM_ERR_CRC     (5)
#define PKT_SEM_ERR_SIZE    (6)
#define PKT_SEM_ERR_TX      (7)
#define PKT_SEM_RX          (8)
#define PKT_SEM_ERR_MAX_SIZE    (9)
#define PKT_SEM_ERR_SPI     (10)

#define CC1101_MAX_RX_SIZE  (64)

#define CC1101_ERR_TIMEOUT      (1)
#define CC1101_ERR_TX_FIFO      (2)
#define CC1101_ERR_SPI          (3)
#define CC1101_ERR_RESET        (4)
#define CC1101_ERR_PN           (5)
#define CC1101_ERR_SIDLE        (6)
#define CC1101_ERR_SFRX         (7)
#define CC1101_ERR_SRX          (8)
#define CC1101_ERR_STX          (9)
#define CC1101_ERR_SPWD         (10)
#define CC1101_ERR_SFTX         (11)
#define CC1101_ERR_STATE        (12)
#define CC1101_ERR_STOP_MODE    (13)
#define CC1101_ERR_NO_SLEEP     (14)
#define CC1101_ERR_PKT_SEM      (15)
#define CC1101_ERR_PKT_TX       (16)

uint8_t cc1101_registerConfig(void);
uint8_t cc1101_open(SPI_HandleTypeDef *spi, uint8_t *rxbuf);
uint8_t cc1101_sendPacket(uint8_t *pData, uint8_t length);
void cc1101_pktSyncRxTx_Callback();
uint8_t cc1101_sleep(void);
uint8_t cc1101_getPartVersion(uint8_t *partRev);
uint8_t gotoSafeModeAfterError(void);
uint8_t cc1101_RxTxToIdle(void);
uint8_t cc1101_IdleToRx(void);

#endif
