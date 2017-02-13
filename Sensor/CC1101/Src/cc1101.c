#include "../Inc/cc1101.h"
#include "stm32l0xx_hal.h"
#include "mxconstants.h"

SPI_HandleTypeDef *hspi;

/*
 * Constants
 */
#define RADIO_BURST_ACCESS   0x40
#define RADIO_SINGLE_ACCESS  0x00
#define RADIO_READ_ACCESS    0x80
#define RADIO_WRITE_ACCESS   0x00

/* Bit fields in the chip status byte */
#define STATUS_CHIP_RDYn_BM             0x80
#define STATUS_STATE_BM                 0x70
#define STATUS_FIFO_BYTES_AVAILABLE_BM  0x0F

/* GPIOs */
#define CC1101_CS_PORT 		CC1101_CS_GPIO_Port
#define CC1101_CS_PIN 		CC1101_CS_Pin
#define CC1101_MISO_PORT	GPIOA
#define CC1101_MISO_PIN		GPIO_PIN_6

/* Error codes */
#define CC1101_ERR_SPI_BEGIN (1)
#define CC1101_ERR_SPI_XFER  (2)

/*
 * Prototypes
 */
static uint8_t trx8BitRegAccess(uint8_t accessType, uint8_t addrByte, uint8_t *pData, uint16_t len, rfStatus_t *status);
static uint8_t CC1101_SPI_Begin();
static void CC1101_SPI_End();
static uint8_t trxReadWriteBurstSingle(uint8_t addr,uint8_t *pData, uint16_t len);



void cc1101_SPI_Init(SPI_HandleTypeDef *spi)
{
	/* Init global spi peripheral handle */
	hspi = spi;
}

void cc1101_SPI_Deinit(void)
{
	hspi = NULL;
}

/******************************************************************************
 * @fn          cc1101SpiReadReg
 *
 * @brief       Read value(s) from config/status/extended radio register(s).
 *              If len  = 1: Reads a single register
 *              if len != 1: Reads len register values in burst mode 
 *
 * input parameters
 *
 * @param       addr   - address of first register to read
 * @param       *pData - pointer to data array where read bytes are saved
 * @param       len   - number of bytes to read
 *
 * output parameters
 *
 * @return      rfStatus_t
 */
uint8_t cc1101SpiReadReg(uint8_t addr, uint8_t *pData, uint8_t len, rfStatus_t *status)
{
  uint8_t retval;

  /* Decide what register space is accessed */
  retval = trx8BitRegAccess((RADIO_BURST_ACCESS|RADIO_READ_ACCESS), addr, pData, len, status);

  return retval;
}


/******************************************************************************
 * @fn          cc1101SpiWriteReg
 *
 * @brief       Write value(s) to config/status/extended radio register(s).
 *              If len  = 1: Writes a single register
 *              if len  > 1: Writes len register values in burst mode 
 *
 * input parameters
 *
 * @param       addr   - address of first register to write
 * @param       *pData - pointer to data array that holds bytes to be written
 * @param       len    - number of bytes to write
 *
 * output parameters
 *
 * @return      rfStatus_t
 */
uint8_t cc1101SpiWriteReg(uint8_t addr, uint8_t *pData, uint8_t len, rfStatus_t *status)
{
  uint8_t retval;

  retval = trx8BitRegAccess((RADIO_BURST_ACCESS|RADIO_WRITE_ACCESS), addr, pData, len, status);

  return retval;
}


/*******************************************************************************
 * @fn          cc1101SpiWriteTxFifo
 *
 * @brief       Write pData to radio transmit FIFO.
 *
 * input parameters
 *
 * @param       *pData - pointer to data array that is written to TX FIFO
 * @param       len    - Length of data array to be written
 *
 * output parameters
 *
 * @return      error code
 */
uint8_t cc1101SpiWriteTxFifo(uint8_t *pData, uint8_t len, rfStatus_t *status)
{
  return trx8BitRegAccess((RADIO_BURST_ACCESS|RADIO_WRITE_ACCESS), CC1101_FIFO, pData, len, status);
}


/*******************************************************************************
 * @fn          cc1101SpiReadRxFifo
 *
 * @brief       Reads RX FIFO values to pData array
 *
 * input parameters
 *
 * @param       *pData - pointer to data array where RX FIFO bytes are saved
 * @param       len    - number of bytes to read from the RX FIFO
 *
 * output parameters
 *
 * @return      rfStatus_t
 */
uint8_t cc1101SpiReadRxFifo(uint8_t * pData, uint8_t len, rfStatus_t *status)
{
	return trx8BitRegAccess((RADIO_BURST_ACCESS|RADIO_READ_ACCESS), CC1101_FIFO, pData, len, status);
}


/******************************************************************************
 * @fn      cc1101GetTxStatus(void)
 *          
 * @brief   This function transmits a No Operation Strobe (SNOP) to get the 
 *          status of the radio and the number of free bytes in the TX FIFO.
 *          
 *          Status byte:
 *          
 *          ---------------------------------------------------------------------------
 *          |          |            |                                                 |
 *          | CHIP_RDY | STATE[2:0] | FIFO_BYTES_AVAILABLE (free bytes in the TX FIFO |
 *          |          |            |                                                 |
 *          ---------------------------------------------------------------------------
 *
 *
 * input parameters
 *
 * @param   none
 *
 * output parameters
 *         
 * @return  rfStatus_t 
 *
 */
uint8_t cc1101GetTxStatus(rfStatus_t * status)
{
    return trxSpiCmdStrobe(CC1101_SNOP, status);
}


/******************************************************************************
 *
 *  @fn       cc1101GetRxStatus(void)
 *
 *  @brief   
 *            This function transmits a No Operation Strobe (SNOP) with the 
 *            read bit set to get the status of the radio and the number of 
 *            available bytes in the RXFIFO.
 *            
 *            Status byte:
 *            
 *            --------------------------------------------------------------------------------
 *            |          |            |                                                      |
 *            | CHIP_RDY | STATE[2:0] | FIFO_BYTES_AVAILABLE (available bytes in the RX FIFO |
 *            |          |            |                                                      |
 *            --------------------------------------------------------------------------------
 *
 *
 * input parameters
 *
 * @param     none
 *
 * output parameters
 *         
 * @return    rfStatus_t 
 *
 */
uint8_t cc1101GetRxStatus(rfStatus_t *status)
{
    return trxSpiCmdStrobe((CC1101_SNOP | RADIO_READ_ACCESS), status);
}


/************** LOW-LEVEL SPI ACCESS FUNCTIONS ****************/

/*******************************************************************************
 * @fn          trx8BitRegAccess
 *
 * @brief       This function performs a read or write from/to a 8bit register
 *              address space. The function handles burst and single read/write
 *              as specfied in addrByte. Function assumes that chip is ready.
 *
 * input parameters
 *
 * @param       accessType - Specifies if this is a read or write and if it's
 *                           a single or burst access. Bitmask made up of
 *                           RADIO_BURST_ACCESS/RADIO_SINGLE_ACCESS/
 *                           RADIO_WRITE_ACCESS/RADIO_READ_ACCESS.
 * @param       addrByte - address byte of register.
 * @param       pData    - data array
 * @param       len      - Length of array to be read(TX)/written(RX)
 *
 * output parameters
 *
 * @return      chip status
 */
static uint8_t trx8BitRegAccess(uint8_t accessType, uint8_t addrByte, uint8_t *pData, uint16_t len, rfStatus_t *status)
{
  uint8_t retval;
  uint8_t txData;

  /* Pull CS_N low and wait for SO to go low before communication starts */
  retval = CC1101_SPI_Begin();
  if (retval) {
    return CC1101_ERR_SPI_BEGIN;
  }

  /* send register address byte and read back chip status */
  txData = accessType | addrByte;
  retval = HAL_SPI_TransmitReceive(hspi, &txData, status, 1, 1000);
  if (retval) {
    CC1101_SPI_End();
    return CC1101_ERR_SPI_XFER;
  }

  /* Send registers values */
  retval = trxReadWriteBurstSingle(accessType|addrByte,pData,len);
  if (retval) {
    CC1101_SPI_End();
    return CC1101_ERR_SPI_XFER;
  } 

  /* Pull CS_N high */
  CC1101_SPI_End();

  /* return the status byte value */
  return retval;
}


/*******************************************************************************
 * @fn          trxSpiCmdStrobe
 *
 * @brief       Send command strobe to the radio. Returns status byte read
 *              during transfer of command strobe. Validation of provided
 *              is not done. Function assumes chip is ready.
 *
 * input parameters
 *
 * @param       cmd - command strobe
 *
 * output parameters
 *
 * @return      status byte
 */
uint8_t trxSpiCmdStrobe(uint8_t cmd, rfStatus_t *status)
{
    uint8_t retval;

    retval = CC1101_SPI_Begin();
    if (retval) {
        return CC1101_ERR_SPI_BEGIN;
    }
    
    retval = HAL_SPI_TransmitReceive(hspi, &cmd, status, 1, 1000);
    if (retval) {
        return CC1101_ERR_SPI_XFER;
    }
    
    CC1101_SPI_End();

    return retval;
}


/*******************************************************************************
 * @fn          trxReadWriteBurstSingle
 *
 * @brief       When the address byte is sent to the SPI slave, the next byte
 *              communicated is the data to be written or read. The address
 *              byte that holds information about read/write -and single/
 *              burst-access is provided to this function.
 *
 *              Depending on these two bits this function will write len bytes to
 *              the radio in burst mode or read len bytes from the radio in burst
 *              mode if the burst bit is set. If the burst bit is not set, only
 *              one data byte is communicated.
 *
 *              NOTE: This function is used in the following way:
 *
 *              TRXEM_SPI_BEGIN();
 *              while(TRXEM_PORT_IN & TRXEM_SPI_MISO_PIN);
 *              ...[Depending on type of register access]
 *              trxReadWriteBurstSingle(uint8 addr,uint8 *pData,uint16 len);
 *              TRXEM_SPI_END();
 *
 * input parameters
 *
 * @param       none
 *
 * output parameters
 *
 * @return      void
 */
static uint8_t trxReadWriteBurstSingle(uint8_t addr,uint8_t *pData, uint16_t len)
{
  uint8_t retval = 0;

  /* Communicate len number of bytes: if RX - the procedure sends 0x00 to push bytes from slave*/
  if(addr & RADIO_READ_ACCESS)
  {
    if(addr & RADIO_BURST_ACCESS)
    {
      retval = HAL_SPI_Receive(hspi, pData, len, 1000);
    }
    else
    {
      retval = HAL_SPI_Receive(hspi, pData, 1, 1000);
    }
  }
  else
  {
    if(addr & RADIO_BURST_ACCESS)
    {
      /* Communicate len number of bytes: if TX - the procedure doesn't overwrite pData */
      retval = HAL_SPI_Transmit(hspi, pData, len, 1000);
    }
    else
    {
      retval = HAL_SPI_Transmit(hspi, pData, 1, 1000);
    }
  }

  return retval;
}

static uint8_t CC1101_SPI_Begin()
{
    uint32_t tickstart = 0;

    tickstart = HAL_GetTick();

	/* CS low */
	HAL_GPIO_WritePin(CC1101_CS_PORT, CC1101_CS_PIN, GPIO_PIN_RESET);
	
    /* Wait MISO low */
	while (HAL_GPIO_ReadPin(CC1101_MISO_PORT, CC1101_MISO_PIN) == GPIO_PIN_SET) {
        if ( (HAL_GetTick() - tickstart ) > 100 ) {
            /* Timeout ! return error */
            return 1;
        }
    }	

    return 0;
}

static void CC1101_SPI_End()
{
	/* CS high */
    HAL_GPIO_WritePin(CC1101_CS_PORT, CC1101_CS_PIN, GPIO_PIN_SET);
}
