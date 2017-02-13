/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "stm32l0xx_hal.h"

#include "cc1120_api.h"
#include <string.h>

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
SPI_HandleTypeDef hspi1;
UART_HandleTypeDef huart1;
CRC_HandleTypeDef hcrc;

/* Private variables ---------------------------------------------------------*/

/* CC1120 packet semaphore used to pass event from CC1120 to main app */
/* pktSemaphore is defined in cc1120_api.c */
extern volatile uint8_t pktSemaphore;
uint8_t rxBuffer[CC1120_MAX_RX_SIZE+10]; // overcautious margin

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_CRC_Init(void);

/* Private function prototypes -----------------------------------------------*/
static void errorHandler(uint8_t errorSource, uint8_t errorId);
static uint8_t setupCC1120(void);

#define LED_PORT	(GPIOC)
#define LED_PIN		(GPIO_PIN_14)
//#define DEBUG       (1)

#define ERROR_CC1120        (1)
#define ERROR_TMP112        (2)
#define ERROR_UART          (3)
#define ERROR_UART_SIZE_TYPE (4)
#define ERROR_PKT_SEM       (5)
#define ERROR_OTHER         (6)

#define UART_TYPE_REMOTE_DATA 	(0)
#define UART_TYPE_LOCAL_DATA  	(1)
#define UART_TYPE_LOG		  	(2)

int main(void)
{
    uint8_t retval;
    uint8_t rcvdMsgSize;
    uint32_t crc;
    uint8_t logBuf[10];

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* Configure the system clock */
    SystemClock_Config();

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_I2C1_Init();
    MX_SPI1_Init();
    MX_USART1_UART_Init();
    MX_CRC_Init();
    
    /* Signal end of hal init with a small blink */
    HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);

    /* CC1120 initizalization */
    retval = setupCC1120();
    if (retval) {
        errorHandler(ERROR_CC1120, retval);
    }

    /* Prepare fixed bytes of buffers */
    rxBuffer[0] = 0xAA; /* sync word */
    rxBuffer[1] = 0;    /* init size to 0 */ 
    rxBuffer[2] = UART_TYPE_REMOTE_DATA;

    memset(logBuf, 0, 10);  /* Initialize to 0 */
    logBuf[0] = 0xAA;       /* UART sync word */
    logBuf[1] = 1+1+4;      /* Type byte + error code byte + 4 bytes CRC */
    logBuf[2] = UART_TYPE_LOG;       /* Type = log/error */

    /* Put radio in rx (ready to receive packets through IRQ */
    retval = cc1120_IdleToRx();
    if (retval) {
        errorHandler(ERROR_CC1120, retval);
    }

    /* Note: pktSemaphore has now switched to PKT_SEM_RX */

    /*
    |---SYNC BYTE---|---LENGTH---|---RF MSG---|---CRC---|
    |       1       |     n      |      k     |    4    |
    |---------------|------------|------------|---------|
    */

    /* Infinite loop */
    while (1)
    {
        switch (pktSemaphore)
        {
        case PKT_SEM_RX:
            /* Do nothing. We'll spend most of time here. */
        break;

        case PKT_SEM_MSG_RCVD:
            /* Message size is stored in first byte and there are 2 bytes of status
               appended at the end, plus we have to include the length byte */
            rcvdMsgSize =  rxBuffer[3] + 3; // payload size + 1st byte (length of msg) + 2-byte status appended at the end of payload
            rxBuffer[1] =  rcvdMsgSize + 1; // account for type byte
            
            /* Compute CRC on uart tx buffer except sync byte and length byte */
            crc = HAL_CRC_Calculate(&hcrc, (uint32_t *)(&rxBuffer[2]), rxBuffer[1]);
            /* Xor CRC to be compliant with standard CRC32 (zip, ethernet, etc.) */
            crc = crc ^ 0xFFFFFFFF;
            
            /* Put CRC at the end of uart frame */
            memcpy(&rxBuffer[rxBuffer[1]+2], &crc, 4);
            rxBuffer[1] += 4; /* +4 CRC bytes */

            /* Send buffer over uart */
            /* Note: add 2 to size to account for sync word and length byte */
            retval = HAL_UART_Transmit(&huart1, rxBuffer, rxBuffer[1]+2, 1000);
            if (retval != HAL_OK) {
                errorHandler(ERROR_UART, retval);
            }

            /* Blink once for each message received */
            HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);
            HAL_Delay(50);
            HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);
            
            /* Move to IDLE state, clean up Rx FIFO and go back to Rx */
            retval = cc1120_IdleToRx();
            if (retval) {
                errorHandler(ERROR_CC1120, retval);
            }
        break;

        default: // all errors are processed here
            /* Log error over uart */
            logBuf[3] = pktSemaphore;
            /* Compute CRC on uart tx buffer except first sync byte and length byte */
            crc = HAL_CRC_Calculate(&hcrc, (uint32_t *)(&logBuf[2]), 2);
            /* Xor CRC to be compliant with standard CRC32 (zip, ethernet, etc.) */
            crc = crc ^ 0xFFFFFFFF;
            /* Put CRC at the end of uart frame */
            /* Note: extra msg length is already handled at init of logmsg buffer */
            memcpy(&logBuf[4], &crc, 4);

            /* Send buffer over uart */
            /* Note: add 2 to size (logBuf[1]) to account for sync word and length byte */
            retval = HAL_UART_Transmit(&huart1, logBuf, logBuf[1]+2, 1000);
            if (retval != HAL_OK) {
                errorHandler(ERROR_UART, retval);
            }

            /* Blink four times to report error */
            HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);
            HAL_Delay(50);
            HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);
            HAL_Delay(200);
            HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);
            HAL_Delay(50);
            HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);
            HAL_Delay(200);
			HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);
			HAL_Delay(50);
			HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);
			HAL_Delay(200);
			HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);
			HAL_Delay(50);
			HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);

            /* Move to IDLE state, clean up Rx FIFO and go back to Rx */
            retval = cc1120_IdleToRx();
            if (retval) {
                errorHandler(ERROR_CC1120, retval);
            }
        break; 
        }
    }
}


static uint8_t setupCC1120()
{
  uint8_t retval;
  uint8_t version;

  /* Initialize and test SPI connection */
  retval = cc1120_open(&hspi1, &rxBuffer[3]);
  if (retval) {
    return retval;
  }

  /* Configure CC1120 registers with rf config */
  retval = cc1120_registerConfig();
  if (retval) {
    return retval;
  }

  /* Cf. CC1120 Errata calibration */
  retval = cc1120_getPartVersion(&version);
  if (retval) {
    return retval;
  }

  if (version < 0x23) {
    /* Perform manual calibration of CC1120 */
    cc1120_manualCalibration();
  }

  return 0;
}


/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  __PWR_CLK_ENABLE();

  /* SCALE1 for max frequency */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0);

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

}

/* I2C1 init function */
void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00303D5B;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLED;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLED;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLED;
  HAL_I2C_Init(&hi2c1);

    /**Configure Analogue filter 
    */
  HAL_I2CEx_AnalogFilter_Config(&hi2c1, I2C_ANALOGFILTER_ENABLED);

}

/* SPI1 init function */
void MX_SPI1_Init(void)
{

  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLED;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLED;
  hspi1.Init.CRCPolynomial = 10;
  HAL_SPI_Init(&hspi1);

}

/* USART1 init function */
void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONEBIT_SAMPLING_DISABLED;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  HAL_UART_Init(&huart1);

}

void MX_CRC_Init(void)
{
   /*##-1- Configure the CRC peripheral #######################################*/
  hcrc.Instance = CRC;

  /* The configuration below makes output of CRC IP compliant with
     'standard' CRC32 used in Ethernet, Zip, PNG, etc. 
     Make sure that CRC output is then xored with 0xFFFFFFFF */
  hcrc.Init.DefaultPolynomialUse    = DEFAULT_POLYNOMIAL_ENABLE;
  hcrc.Init.DefaultInitValueUse     = DEFAULT_INIT_VALUE_DISABLE;
  hcrc.Init.InitValue               = 0xFFFFFFFF;
  hcrc.Init.InputDataInversionMode  = CRC_INPUTDATA_INVERSION_BYTE;
  hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_ENABLED;
  hcrc.InputDataFormat              = CRC_INPUTDATA_FORMAT_BYTES;

  /* DeInitializes the CRC peripheral */
  HAL_CRC_DeInit(&hcrc);

  /* Initialise CRC */
  HAL_CRC_Init(&hcrc);

  if(HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    errorHandler(ERROR_OTHER, 2);
  } 
}

/* The function below initializes only GPIO not used as alternate functions */
/* Alternate functions GPIO are configured in peripheral msp_init functions */

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
        * Free pins are configured automatically as Analog (this feature is enabled through 
        * the Code Generation settings)
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __GPIOC_CLK_ENABLE();
  __GPIOA_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : PC14 (LED) */
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 PA2 PA3 
                           PA11 PA12 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 (RF_CS) PA8 (RF_RESETN) */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 (CC1120 GPIO2: PKT_SYNC_RXTX) */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB1 PB4 PB3 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);
}


static void errorHandler(uint8_t source, uint8_t errorId)
{
#ifdef DEBUG
    uint8_t i;

	while(1) {

        for (i=0; i<source; i++) {
            HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);
            HAL_Delay(500);
            HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);
            HAL_Delay(300);
        }

        HAL_Delay(1500);

        for (i=0; i<errorId; i++) {
            HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);
            HAL_Delay(50);
            HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);
            HAL_Delay(450);
        }

        HAL_Delay(5000);
    }
#else
    /* Reset MCU */
    HAL_NVIC_SystemReset();
#endif
}


/* Function called when interrupt on any EXTI interrupt line */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	switch (GPIO_Pin) 
	{
		case GPIO_PIN_0:
			/* This is PB0 pin falling interrupt */
			cc1120_pktSyncRxTx_Callback();
		break;

		default:
		break;
	}
        
}



#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/*****END OF FILE****/
