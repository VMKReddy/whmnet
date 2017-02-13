#include "../Inc/sleep.h"
#include "stm32l0xx_hal.h"

#define LSI_CLK_HZ	(38000) /* Datasheet say it is between 26 and 56 kHz ... need calibration! */

RTC_HandleTypeDef *phrtc;
uint32_t lsiFreqHz;
__IO uint8_t wokenUpByTimeout = 0;

void sleepInit(RTC_HandleTypeDef *rtc, uint32_t lsiFrequency)
{
  phrtc = rtc;
  lsiFreqHz = lsiFrequency;
}

void doSleepStop_WakeExti()
{
    /* Suspend systick because systick IRQ could prevent system
       from entering stop mode when executing WFI instruction */
    HAL_SuspendTick();

    /* Enter Stop Mode */
    /* Use the low power regulator instead of main regulator */
    HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
    
    /* Resume systick */
    HAL_ResumeTick();
}

uint8_t doSleepStop_WakeRTC_RTCCLK(uint32_t timeoutMS)
{
    /*## Setting the Wake up time ############################################*/
    /*  RTC Wakeup Interrupt Generation:
        Wakeup Time Base = (RTC_WAKEUPCLOCK_RTCCLK_DIV /(LSE or LSI))
        Wakeup Time = Wakeup Time Base * WakeUpCounter 
                    = (RTC_WAKEUPCLOCK_RTCCLK_DIV /(LSE or LSI)) * WakeUpCounter
        ==> WakeUpCounter = Wakeup Time / Wakeup Time Base
        To configure the wake up timer to 4s the WakeUpCounter is set to 0x1FFF:
          RTC_WAKEUPCLOCK_RTCCLK_DIV = RTCCLK_Div16 = 16 
          Wakeup Time Base = 16 /(~39.000KHz) = ~0,410 ms
          Wakeup Time = ~4s = 0,410ms  * WakeUpCounter
          ==> WakeUpCounter = ~4s/0,410ms = 9750 = 0x2616 */
    
    uint8_t retval;
    uint16_t timeout;
    float timeBase = ((float)2) / ((float)lsiFreqHz) * 1000;
    
    if ((timeoutMS) > (uint32_t)(timeBase*65536) ) {
	    return SLEEP_ERR_TIMEOUT_VALUE;
    }

    /* Substract 1 (see RTC_WUTR->WUT) */
    timeout = (uint16_t) ((float)timeoutMS / timeBase) - 1;

    /* Stop wakeup counter */
    retval = HAL_RTCEx_DeactivateWakeUpTimer(phrtc);
    if (retval) {
        return SLEEP_ERR_HAL;
    }

    /* Start wakeuptimer and associated interrupt */
    retval = HAL_RTCEx_SetWakeUpTimer_IT(phrtc, timeout, RTC_WAKEUPCLOCK_RTCCLK_DIV2);
    if (retval) {
        return SLEEP_ERR_HAL;
    }

    /* Suspend systick because systick IRQ could prevent system
       from entering stop mode when executing WFI instruction */
    HAL_SuspendTick();
    
    wokenUpByTimeout = 0;

    /* Enter Stop Mode */
    /* Use the low power regulator instead of main regulator */
    HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
  
    /* Resume systick */
    HAL_ResumeTick();
  
    /* Stop wakeup counter */
    retval = HAL_RTCEx_DeactivateWakeUpTimer(phrtc);
    if (retval) {
        return SLEEP_ERR_HAL;
    }    

    /* Clear Wake Up Flag */
    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);

    return 0;
}

uint8_t doSleepStop_WakeRTC_CKSPRE(uint16_t timeoutSeconds)
{
    uint8_t retval;

    /* Stop wakeup counter */
    retval = HAL_RTCEx_DeactivateWakeUpTimer(phrtc);
    if (retval) {
        return SLEEP_ERR_HAL;
    }

    /* Start wakeuptimer and associated interrupt */
    retval = HAL_RTCEx_SetWakeUpTimer_IT(phrtc, timeoutSeconds - 1, RTC_WAKEUPCLOCK_CK_SPRE_16BITS);
    if (retval) {
        return SLEEP_ERR_HAL;
    }
    
    /* Suspend systick because systick IRQ could prevent system
       from entering stop mode when executing WFI instruction */
    HAL_SuspendTick();
    
    /* Reset wakeup flag */
    wokenUpByTimeout = 0;

    /* Enter Stop Mode */
    /* Use the low power regulator instead of main regulator */
    HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
  
    /* Resume systick */
    HAL_ResumeTick();

    /* Stop wakeup counter */
    retval = HAL_RTCEx_DeactivateWakeUpTimer(phrtc);
    if (retval) {
        return SLEEP_ERR_HAL;
    }    

    /* Clear Wake Up Flag */
    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);

    return 0;
}

uint8_t sleepExitOnTimeout(void)
{
    return wokenUpByTimeout;
}


/**
  * @brief  RTC Wake Up callback
  * @param  None
  * @retval None
  */
void HAL_RTCEx_WakeUpTimerEventCallback(RTC_HandleTypeDef *hrtc)
{
  wokenUpByTimeout = 1;
}

/**
  * The 2 functions below are __weak redefinitions.
  * __weak defs are in stm32l0xx_hal.c
  */
void HAL_SuspendTick(void)
{
    /* Disable SysTick Interrupt */
    SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk;
    /* Disable SysTick */
    SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
}

void HAL_ResumeTick(void)
{
  /* Enable SysTick Interrupt */
  SysTick->CTRL  |= SysTick_CTRL_TICKINT_Msk;
  /* Enable SysTick */
  SysTick->CTRL  |= SysTick_CTRL_ENABLE_Msk;
}
