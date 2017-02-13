/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32l0xx_hal.h"

#include "cc1101_api.h"
#include "sleep.h"

#define SENSOR_ID           (3)

//#define DEBUG               (0)

#ifdef DEBUG
    #define CONV_PERIOD_S       (10)
#else
    #define CONV_PERIOD_S       (300) /* 1 reading every 5 minutes */
#endif

#define MS56_CMD_RST		(0x1E)
#define MS56_CMD_PROM_READ	(0xA0)
#define MS56_CMD_CONV_P_256	(0x40)
#define MS56_CMD_CONV_T_256	(0x50)
#define MS56_CMD_ADC_READ	(0x00)
#define MS56_PROM_SIZE		(7)

#define ERROR_CC1101        (1)
#define ERROR_SHT21			(2)
#define ERROR_MS56			(3)
#define ERROR_OTHER         (4)
#define ERROR_SLEEP         (5)

#define LOG_MSG_REBOOT      (1)

#define SHT21_I2C_ADDR  	(0x80)
#define MS5637_I2C_ADDR 	(0xEC)

#define SHT21_READ_MAX_RETRIES	(5)

#define RTC_ASYNC_PREDIV    (0x7F)

#define MAX_PKT_ID			(0xf)
#define MAX_PKT_TYPE		(0xf)
#define TYPE_DATA		 	(0)
#define TYPE_LOG		  	(1)
#define PKT_ID_MASK			(0x0f)
#define PKT_TYPE_MASK		(0xf0)
#define PKT_TYPE_OFT		(4)

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim;
uint16_t tmpCC4[3] = {0,0,0};
__IO uint32_t uwLsiFreq = 0;
__IO uint32_t uwCaptureNumber = 0;
__IO uint32_t uwPeriodValue = 0;
uint16_t MS56_PROMData[MS56_PROM_SIZE];

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void errorHandler(uint8_t source, uint8_t errorId);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_RTC_Init(void);

/* Private function prototypes -----------------------------------------------*/
int8_t SHT21_setup(void);
int8_t SHT21_TriggerRHConv();
int8_t SHT21_TriggerTempConv();
int8_t SHT21_ReadTemp(uint16_t *temp);
int8_t SHT21_ReadHum(uint16_t *hum);
int8_t MS5637_ReadData(uint32_t *temp, uint32_t *press);
int8_t MS5637_Setup(void);
static void GetLSIFrequency(void);
static void configurePower(void);
static void GPIO_Disable_SWD(void);
static void blinkSequence(void);
static uint8_t CC1101_Setup();
static void GPIO_Enable_IRQs(void);
static int8_t buildHeader(uint8_t id, uint8_t type, uint8_t *hdr);

int main(void)
{
	uint8_t lsiMeasCnt=0;
	uint16_t SHTDigTemp, SHTDigHum;
	uint32_t MS56DigTemp, MS56DigPres;
	int64_t dT;
	int64_t OFF, SENS, PRES;
	int8_t retval;
	uint8_t status;
	uint8_t txPacket[7];

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* Configure the system clock */
	SystemClock_Config();

	/* Time to be able to program the chip with openOCD
	 * in case we are in stop mode or SWD pins have been disabled
	 */
	HAL_Delay(2000);

	/* Set SWD pins to ANALOG to save power */
	GPIO_Disable_SWD();

	/* Initialize GPIOs */
	MX_GPIO_Init();

	/* For sensors */
	MX_I2C1_Init();

	/* For CC1101 */
	MX_SPI1_Init();

	/** SLEEP MANAGER CONFIG **/

	/* First we measure LSI frequency */
	while (uwLsiFreq < 26000 || uwLsiFreq > 56000) {
	  GetLSIFrequency();
	  lsiMeasCnt++;
	  if (lsiMeasCnt > 4) {
		  errorHandler(ERROR_OTHER, 1);
	  }
	}

	/* Then init RTC with right prescalers calculated with LSI freq */
	MX_RTC_Init();

	/* Low power sleep init */
	sleepInit(&hrtc, uwLsiFreq);

	/* Configure low power mode features */
	configurePower();

	/** SENSORS INIT **/

	retval = MS5637_Setup();
	if (retval < 0) {
		errorHandler(ERROR_MS56, -retval);
	}

	retval = SHT21_setup();
	if (retval < 0) {
		errorHandler(ERROR_SHT21, -retval);
	}

	status = CC1101_Setup();
	if (status) {
		errorHandler(ERROR_CC1101, status);
	}

	/* Enable IRQs */
	GPIO_Enable_IRQs();

	/* Led blink at end of init */
	blinkSequence();

	/** SEND FIRST PACKET WITH SENSOR INFO **/

    txPacket[0] = 4; /* 1 byte header + 1 byte log + 2 bytes lsi frequency */
    buildHeader(SENSOR_ID, TYPE_LOG, &txPacket[1]); // build packet header
    txPacket[2] = LOG_MSG_REBOOT;
    txPacket[3] = uwLsiFreq >> 8;
    txPacket[4] = uwLsiFreq;
    status = cc1101_sendPacket(txPacket, 5);
    if (status) {
        errorHandler(ERROR_CC1101, status);
    }

    /* Prepare fixed bytes of sensor data packet */
    txPacket[0] = 6; /* 1 byte ID + 5 bytes data - exclude length byte */
    buildHeader(SENSOR_ID, TYPE_DATA, &txPacket[1]); // build packet header

	while(1)
	{
		/** BLINK **/
	#ifdef DEBUG
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
		status = doSleepStop_WakeRTC_RTCCLK(20);
		if (status) {
			errorHandler(ERROR_SLEEP, status);
		}
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
	#endif

		/** READ SENSORS **/

		// Start RH conversion
		retval = SHT21_TriggerRHConv();
		if (retval < 0) {
			errorHandler(ERROR_SHT21, -retval);
		}

		// While converting RH on SHT21, read pressure on MS5637
		retval = MS5637_ReadData(&MS56DigTemp, &MS56DigPres);
		if (retval < 0) {
			errorHandler(ERROR_MS56, -retval);
		}

		// Typical SHT21 RH conversion time is 12ms but we already spent
		// ~6ms in previous function
		HAL_Delay(6);

		// Read RH conversion result
		retval = SHT21_ReadHum(&SHTDigHum);
		if (retval < 0) {
			errorHandler(ERROR_SHT21, -retval);
		}

		// Start temperature conversion
		retval = SHT21_TriggerTempConv();
		if (retval < 0) {
			errorHandler(ERROR_SHT21, -retval);
		}

		// Typical temperature conversion time
		HAL_Delay(9);

		// Read result
		retval = SHT21_ReadTemp(&SHTDigTemp);
		if (retval < 0) {
			errorHandler(ERROR_SHT21, -retval);
		}

		/** COMPUTE MEASUREMENTS **/

		// MS5637 temperature result
		dT = (int64_t)MS56DigTemp - ((int64_t)MS56_PROMData[5] << 8);
		//TEMP = 2000 + (dT * MS56_PROMData[6] >> 23);

		// MS5637 pressure result
		OFF =  ( (int64_t)MS56_PROMData[2] << 17 ) +
			   ( (MS56_PROMData[4] * dT) >> 6 );
		SENS = ( (int64_t)MS56_PROMData[1] << 16 ) +
			   ( (MS56_PROMData[3] * dT) >> 7 );
		PRES = ( ((int64_t)MS56DigPres * SENS >> 21) - OFF ) >> 15;
		// we can keep 17 bits
		// or divide by 10 and keep 14 bits
		// or substract 85000 and keep 16 bits
		// or both and keep 12 bits

		/** SEND PACKET */

		/* ||      txPacket[4]		||                txPacket[3]		  ||   txPacket[2]	  ||
		 * || 00 | SHTDigHum[15:10] || SHTDigHum[9:5] | SHTDigTemp[15:13] || SHTDigTemp[12:5] ||
		 */
		txPacket[2] = (uint8_t)(SHTDigTemp >> 5);
		txPacket[3] = (uint8_t)( ((SHTDigTemp >> 13) & 0x7) | ((SHTDigHum >> 2) & 0xf8) );
		txPacket[4] = (uint8_t)(SHTDigHum >> 10);

		/* || txPacket[6]  || txPacket[5] ||
		 * || PRES[15:8]   || PRES[7:0]   ||
		 */
		PRES = PRES - 85000;
		txPacket[5] = (uint8_t)( PRES & 0xFF );
		txPacket[6] = (uint8_t)( PRES >> 8 );

		// Don't report MS5637 temperature measurement because it is less accurate than SHT21

		status = cc1101_sendPacket(txPacket, 7);
		if (status) {
			errorHandler(ERROR_CC1101, status);
		}

		/** SHUTDOWN EVERYTHING UNTIL NEXT MEAS. **/

		/* Bypass TPS converter operation (if was in bypass) */
		HAL_GPIO_WritePin(TPS_BYP_GPIO_Port, TPS_BYP_Pin, GPIO_PIN_RESET);

		/* Convert at CONV_PERIOD_S rate - now gow to sleep */
		status = doSleepStop_WakeRTC_CKSPRE(CONV_PERIOD_S);
		if (status) {
			errorHandler(ERROR_SLEEP, status);
		}

		/* Enable TPS converter operation (if was in bypass) */
		HAL_GPIO_WritePin(TPS_BYP_GPIO_Port, TPS_BYP_Pin, GPIO_PIN_SET);
	}

	/* We should never get there. */
	while(1);
}

static int8_t buildHeader(uint8_t id, uint8_t type, uint8_t *hdr)
{
	if (id > MAX_PKT_ID) {
		return -1;
	}

	if (type > MAX_PKT_TYPE) {
		return -1;
	}

	*hdr = id & PKT_ID_MASK;
	*hdr |= (type << PKT_TYPE_OFT) & PKT_TYPE_MASK;

	return 0;
}


/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_5;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0);

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_RTC;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00000708;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  HAL_I2C_Init(&hi2c1);

	/**Configure Analogue filter
	*/
  HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE);

}

/* RTC init function */
static void MX_RTC_Init(void)
{

  RTC_TimeTypeDef sTime;
  RTC_DateTypeDef sDate;

    /**Initialize RTC and set the Time and Date 
    */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = RTC_ASYNC_PREDIV;
  hrtc.Init.SynchPrediv = (uwLsiFreq / 128) - 1; // 1 Hz clock;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  HAL_RTC_Init(&hrtc);

  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD);

  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x0;

  HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD);
}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  HAL_SPI_Init(&hspi1);
}

static void GPIO_Disable_SWD(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;

	GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
        * Free pins are configured automatically as Analog (this feature is enabled through 
        * the Code Generation settings)
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pins : PC14 PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 PA8 PA12
                           PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_8|GPIO_PIN_12
                          |GPIO_PIN_15/*SPI : |GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7*/;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : CC1101_CS_Pin */
  GPIO_InitStruct.Pin = CC1101_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CC1101_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : CC1101_GDO2_Pin USR_IO_1_Pin USR_IO_2_Pin
                           USR_IO_3_Pin */
  GPIO_InitStruct.Pin = CC1101_GDO2_Pin|USR_IO_1_Pin|USR_IO_2_Pin
                          |USR_IO_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA3 (CC1101 GDO0: PKT_SYNC_RXTX) */
  GPIO_InitStruct.Pin = CC1101_GDO0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(CC1101_GDO0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB_USER_Pin */
  GPIO_InitStruct.Pin = PB_USER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(PB_USER_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB1 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_5/*I2C : |GPIO_PIN_6|GPIO_PIN_7*/;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : TPS_BYP_Pin LED_Pin */
  GPIO_InitStruct.Pin = TPS_BYP_Pin|LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* CC1101 CSn set to 1 */
  HAL_GPIO_WritePin(CC1101_CS_GPIO_Port, CC1101_CS_Pin, GPIO_PIN_SET);

  /* TPS62730 in switching mode (bypass active low) */
  HAL_GPIO_WritePin(TPS_BYP_GPIO_Port, TPS_BYP_Pin, GPIO_PIN_SET);

  /* LED off */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
}

static void GPIO_Enable_IRQs(void)
{
  /* EXTI interrupt init for CC1101 GDO0 */
  HAL_NVIC_SetPriority(EXTI2_3_IRQn, 2, 0);

  /* EXTI interrupt init for USER SW */
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 2, 0);

  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

  HAL_NVIC_ClearPendingIRQ(EXTI2_3_IRQn);
  HAL_NVIC_EnableIRQ(EXTI2_3_IRQn);
}

int8_t SHT21_setup(void)
{
	HAL_StatusTypeDef status;
	uint8_t reg;

	// sensor needs at most 15ms after power-up
	status = doSleepStop_WakeRTC_RTCCLK(20);
	if (status) {
		return -2;
	}

	/* Check if device ready to accept commands */
	status = HAL_I2C_IsDeviceReady(&hi2c1, SHT21_I2C_ADDR, 10, 1000);
	if (status ) {
		return -1;
	}

	/* Configure sensor resolution */

	status = HAL_I2C_Mem_Read(&hi2c1, SHT21_I2C_ADDR, 0xE7, I2C_MEMADD_SIZE_8BIT, &reg, 1, 10);
	if (status ) {
		return -1;
	}

	reg = reg | 0x81; // set Measurement resolution to 11bit both

	status = HAL_I2C_Mem_Write(&hi2c1, SHT21_I2C_ADDR, 0xE6, I2C_MEMADD_SIZE_8BIT, &reg, 1, 10);
	if (status ) {
		return -1;
	}

	return 0;
}

static uint8_t CC1101_Setup()
{
	uint8_t retval;

	/* Initialize and test SPI connection */
	retval = cc1101_open(&hspi1, NULL);
	if (retval) {
		return retval;
	}

	/* Configure CC1101 registers with rf config */
	retval = cc1101_registerConfig();
	if (retval) {
		return retval;
	}

	/* Go to sleep for now */
	retval = cc1101_sleep();
	if (retval) {
		return retval;
	}

  return 0;
}

int8_t SHT21_TriggerTempConv()
{
	uint8_t data;
	HAL_StatusTypeDef status;

	// trigger T meas, no hold master
	data = 0xf3;
	status = HAL_I2C_Master_Transmit(&hi2c1, SHT21_I2C_ADDR, &data, 1, 10);
	if (status ) {
		return -1;
	}

	return 0;
}


int8_t SHT21_ReadTemp(uint16_t *temp)
{
	uint8_t data[3];
	HAL_StatusTypeDef status;
	uint8_t retry = 0;

	// trigger T meas, no hold master
	data[0] = 0xf3;
	status = HAL_I2C_Master_Transmit(&hi2c1, SHT21_I2C_ADDR, data, 1, 10);
	if (status ) {
		return -1;
	}

	// typical delay
	HAL_Delay(9);

	// read temperature measurement
	status = HAL_I2C_Master_Receive(&hi2c1, SHT21_I2C_ADDR, data, 3, 10);
	while ( (status == HAL_ERROR) && (retry++ < SHT21_READ_MAX_RETRIES) ) {
		// measurement is not done, wait a bit then read again
		HAL_Delay(1);
		status = HAL_I2C_Master_Receive(&hi2c1, SHT21_I2C_ADDR, data, 3, 10);
	}
	if (status) {
		return -2;
	}

	// status bit 1 must be 0 (temperature measurement)
	if (data[1] & 2) {
		return -3;
	}

	// todo: check crc

	// set 2 status bits of LSB to 0 then concatenate MSB and LSB
	*temp = ((uint16_t)data[0] << 8) + (data[1] & 0xFC);

	return 0;
}

int8_t SHT21_TriggerRHConv()
{
	uint8_t data;
	HAL_StatusTypeDef status;

	// trigger RH measurement, no hold master
	data = 0xf5;
	status = HAL_I2C_Master_Transmit(&hi2c1, SHT21_I2C_ADDR, &data, 1, 10);
	if (status ) {
		return -1;
	}

	return 0;
}

int8_t SHT21_ReadHum(uint16_t *hum)
{
	uint8_t data[3];
	HAL_StatusTypeDef status;
	uint8_t retry = 0;

	// typical RH conversion time for 11bit resolution is 15ms

	// read RH measurement
	status = HAL_I2C_Master_Receive(&hi2c1, SHT21_I2C_ADDR, data, 3, 10);
	while ( (status == HAL_ERROR) && (retry++ < SHT21_READ_MAX_RETRIES) ) {
		// measurement is not done, wait a bit then read again
		HAL_Delay(1);
		status = HAL_I2C_Master_Receive(&hi2c1, SHT21_I2C_ADDR, data, 3, 10);
	}
	if (status) {
		return -7;
	}

	// status bit 1 must be 1 (RH measurement)
	if (!(data[1] & 2)) {
		return -8;
	}

	// todo: check crc -- see application note

	// set 2 status bits of LSB to 0 then concatenate MSB and LSB
	*hum = ((uint16_t)data[0] << 8) + (data[1] & 0xFC);

	return 0;
}



int8_t MS5637_ReadData(uint32_t *temp, uint32_t *press)
{
	HAL_StatusTypeDef status;
	uint8_t data[3];

	// Send temperature conversion command
	data[0] = MS56_CMD_CONV_T_256;
	status = HAL_I2C_Master_Transmit(&hi2c1, MS5637_I2C_ADDR, data, 1, 10);
	if (status ) {
		return -1;
	}

	// Wait for conversion - depends on oversampling ratio
	HAL_Delay(3);

	/* Read ADC result */

	data[0] = MS56_CMD_ADC_READ;
	status = HAL_I2C_Master_Transmit(&hi2c1, MS5637_I2C_ADDR, data, 1, 10);
	if (status ) {
		return -3;
	}

	status = HAL_I2C_Master_Receive(&hi2c1, MS5637_I2C_ADDR, data, 3, 10);
	if (status ) {
		return -4;
	}

	*temp = ((uint32_t)data[0] << 16) + (data[1] << 8) + data[2];

	/* Send pressure conversion command */
	data[0] = MS56_CMD_CONV_P_256;
	status = HAL_I2C_Master_Transmit(&hi2c1, MS5637_I2C_ADDR, data, 1, 10);
	if (status ) {
		return -5;
	}

	/* Wait for conversion - depends on oversampling ratio */
	HAL_Delay(3);

	/* Read ADC result */

	data[0] = MS56_CMD_ADC_READ;
	status = HAL_I2C_Master_Transmit(&hi2c1, MS5637_I2C_ADDR, data, 1, 10);
	if (status ) {
		return -7;
	}

	status = HAL_I2C_Master_Receive(&hi2c1, MS5637_I2C_ADDR, data, 3, 10);
	if (status ) {
		return -8;
	}

	*press = ((uint32_t)data[0] << 16) + (data[1] << 8) + data[2];

	return 0;
}

int8_t MS5637_Setup(void)
{
	HAL_StatusTypeDef status;
	uint8_t data[2];
	uint8_t i;

	// sensor needs at most 15ms after power-up
	status = doSleepStop_WakeRTC_RTCCLK(20);

	/* Check if device ready to accept commands */
	status = HAL_I2C_IsDeviceReady(&hi2c1, MS5637_I2C_ADDR, 10, 1000);
	if (status ) {
		return -1;
	}

	/* Send reset sequence */
	data[0] = 0x1E;
	status = HAL_I2C_Master_Transmit(&hi2c1, MS5637_I2C_ADDR, data, 1, 10);
	if (status ) {
		return -1;
	}

	/* Read PROM containing calibration data */
	for (i=0; i<MS56_PROM_SIZE; i++) {

		/* Send PROM read command */
		data[0] = MS56_CMD_PROM_READ + (i<<1);
		status = HAL_I2C_Master_Transmit(&hi2c1, MS5637_I2C_ADDR, data, 1, 10);
		if (status ) {
			return -1;
		}

		/* Read 16-bit data */
		status = HAL_I2C_Master_Receive(&hi2c1, MS5637_I2C_ADDR, data, 2, 10);
		if (status ) {
			return -1;
		}
		MS56_PROMData[i] = ((uint16_t)data[0] << 8) + data[1];
	}

	return 0;
}

static void blinkSequence()
{
    uint8_t i;
    uint8_t retval;

    for (i=0; i<2; i++) {
        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
        retval = doSleepStop_WakeRTC_RTCCLK(10);
        if (retval) {
            errorHandler(ERROR_SLEEP, retval);
        }

        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
        retval = doSleepStop_WakeRTC_RTCCLK(300);
        if (retval) {
            errorHandler(ERROR_SLEEP, retval);
        }
    }
}

static void configurePower()
{
  /* Enable Ultra low power mode */
  HAL_PWREx_EnableUltraLowPower();

  /* Enable the fast wake up from Ultra low power mode */
  //HAL_PWREx_EnableFastWakeUp();

  /* Select MSI as system clock source after Wake Up from Stop mode */
  __HAL_RCC_WAKEUPSTOP_CLK_CONFIG(RCC_StopWakeUpClock_MSI);
}

static void GetLSIFrequency(void)
{
    TIM_IC_InitTypeDef timICConfig;

    /* Configure the TIM peripheral */
    /* Set TIMx instance */
    htim.Instance = TIM21;

    /* Deinitialize the TIM21 peripheral registers to their default reset values */
    HAL_TIM_IC_DeInit(&htim);

    /* TIM21 configuration: Input Capture mode
       The LSI oscillator is connected to TIM21 CH1.
       The Rising edge is used as active edge.
       The TIM21 CCR1 is used to compute the frequency value.
    */
    htim.Init.Prescaler         = 0;
    htim.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim.Init.Period            = 0xFFFF;
    htim.Init.ClockDivision     = 0;
    HAL_TIM_IC_Init(&htim);

    /* Connect internally the TIM21_CH1 Input Capture to the LSI clock output */
    HAL_TIMEx_RemapConfig(&htim, TIM21_TI1_LSI);

    /* Configure the Input Capture of channel 1 */
    timICConfig.ICPolarity  = TIM_ICPOLARITY_RISING;
    timICConfig.ICSelection = TIM_ICSELECTION_DIRECTTI;
    timICConfig.ICPrescaler = TIM_ICPSC_DIV8;
    timICConfig.ICFilter    = 0;
    HAL_TIM_IC_ConfigChannel(&htim, &timICConfig, TIM_CHANNEL_1);

    uwCaptureNumber = 0;
    /* Start the TIM Input Capture measurement in interrupt mode */
    HAL_TIM_IC_Start_IT(&htim, TIM_CHANNEL_1);

    /* Wait until the TIM21 get 2 LSI edges */
    /* uwCaptureNumber is incremented on interrupt */
    while(uwCaptureNumber != 3);

    /* Disable TIM21 CC1 Interrupt Request */
    HAL_TIM_IC_Stop_IT(&htim, TIM_CHANNEL_1);

    /* Deinitialize the TIM21 peripheral registers to their default reset values */
    HAL_TIM_IC_DeInit(&htim);
}


/**
  * @brief  Input Capture callback in non blocking mode
  * @param  htim : TIM IC handle
  * @retval None
*/
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *ht)
{
  /* Get the Input Capture value */
  if (uwCaptureNumber < 3) {
	  tmpCC4[uwCaptureNumber++] = HAL_TIM_ReadCapturedValue(&htim, TIM_CHANNEL_1);

	  if (uwCaptureNumber == 3)
	  {
		if ( tmpCC4[2] > tmpCC4[1] )
		{
		  /* Compute the period length */
		  uwPeriodValue = (uint16_t)(0xFFFF - tmpCC4[1] + tmpCC4[2] + 1);
		}
		else
		{
		  /* Compute the period length */
		  uwPeriodValue = (uint16_t)(tmpCC4[2] - tmpCC4[1] + 1);
		}
		/* Frequency computation */
		uwLsiFreq = ((uint32_t)SystemCoreClock) * 8 / uwPeriodValue;
	  }
	}
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void errorHandler(uint8_t source, uint8_t errorId)
{
#ifdef DEBUG
    uint8_t i;

	while(1) {

        for (i=0; i<source; i++) {
            HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
            doSleepStop_WakeRTC_RTCCLK(500);
            HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
            doSleepStop_WakeRTC_RTCCLK(300);
        }

        doSleepStop_WakeRTC_RTCCLK(1500);

        for (i=0; i<errorId; i++) {
            HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
            doSleepStop_WakeRTC_RTCCLK(50);
            HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
            doSleepStop_WakeRTC_RTCCLK(450);
        }

       doSleepStop_WakeRTC_CKSPRE(5);
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
		case CC1101_GDO0_Pin:
			/* This is PB0 pin falling interrupt */
			cc1101_pktSyncRxTx_Callback();
		break;

		case PB_USER_Pin:
			/* This is User PB pin falling interrupt */
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


/****END OF FILE****/
