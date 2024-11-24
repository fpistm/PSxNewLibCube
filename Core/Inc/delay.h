
#ifndef _DELAY_H_
#define _DELAY_H_

#include "dwt.h"


static inline uint32_t millis(void)
{
  // ToDo: ensure no interrupts
  return HAL_GetTick();
}


static inline void delay(uint32_t ms)
{
  if (ms != 0) {
    uint32_t start = HAL_GetTick();
    do {
    } while (HAL_GetTick() - start < ms);
  }
}

/**
 * \brief Pauses the program for the amount of time (in microseconds) specified as parameter.
 *
 * \param us the number of microseconds to pause (uint32_t)
 */
static inline void delayMicroseconds(uint32_t) __attribute__((always_inline, unused));
static inline void delayMicroseconds(uint32_t us)
{
#if defined(DWT_BASE)
  int32_t start  = dwt_getCycles();
  int32_t cycles = us * (SystemCoreClock / 1000000);

  while ((int32_t)dwt_getCycles() - start < cycles);
#else
  __IO uint32_t currentTicks = SysTick->VAL;
  /* Number of ticks per millisecond */
  const uint32_t tickPerMs = SysTick->LOAD + 1;
  /* Number of ticks to count */
  const uint32_t nbTicks = ((us - ((us > 0) ? 1 : 0)) * tickPerMs) / 1000;
  /* Number of elapsed ticks */
  uint32_t elapsedTicks = 0;
  __IO uint32_t oldTicks = currentTicks;
  do {
    currentTicks = SysTick->VAL;
    elapsedTicks += (oldTicks < currentTicks) ? tickPerMs + oldTicks - currentTicks :
                    oldTicks - currentTicks;
    oldTicks = currentTicks;
  } while (nbTicks > elapsedTicks);
#endif
}
#endif /* _DELAY_H_ */
