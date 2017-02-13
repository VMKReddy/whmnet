#ifndef CUSTOM_SLEEP_H
#define CUSTOM_SLEEP_H

#include "stm32l0xx_hal.h"

#define SLEEP_ERR_TIMEOUT_VALUE (1)
#define SLEEP_ERR_HAL           (2)

void sleepInit(RTC_HandleTypeDef *rtc, uint32_t lsiFrequency);
void doSleepStop_WakeExti(void);
uint8_t doSleepStop_WakeRTC_RTCCLK(uint32_t timeoutMS);
uint8_t doSleepStop_WakeRTC_CKSPRE(uint16_t timeoutSeconds);
uint8_t sleepExitOnTimeout(void);

#endif
