/*
 * Simple microseconds delay routine, utilizing ARM's DWT
 * (Data Watchpoint and Trace Unit) and HAL library.
 * Intended to use with gcc compiler, but I hope it can be used
 * with any other C compiler across the Universe (provided that
 * ARM and CMSIS already invented) :)
 * Max K
 *
 *
 * This file is part of DWT_Delay package.
 * DWT_Delay is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License,
 * or (at your option) any later version.
 *
 * us_delay is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See
 * the GNU General Public License for more details.
 * http://www.gnu.org/licenses/.
 */

#include <Imported_Libs/dwt_delay.h>
#include "stm32wbxx_hal.h"          // change to whatever MCU you use

/**
 * Initialization routine.
 * You might need to enable access to DWT registers on Cortex-M7
 *   DWT->LAR = 0xC5ACCE55
 */
void DWT_Init(void)
{
  volatile unsigned int *DWT_CYCCNT   = (volatile unsigned int *)0xE0001004; //address of the register
  volatile unsigned int *DWT_CONTROL  = (volatile unsigned int *)0xE0001000; //address of the register
  volatile unsigned int *DWT_LAR      = (volatile unsigned int *)0xE0001FB0; //address of the register
  volatile unsigned int *SCB_DEMCR    = (volatile unsigned int *)0xE000EDFC; //address of the register

  *DWT_LAR = 0xC5ACCE55; // unlock (CM7)
  *SCB_DEMCR |= 0x01000000;
  *DWT_CYCCNT = 0; // reset the counter
  *DWT_CONTROL |= 1 ; // enable the counter

}

#if DWT_DELAY_NEWBIE
/**
 * If you are a newbie and see magic in DWT_Delay, consider this more
 * illustrative function, where you explicitly determine a counter
 * value when delay should stop while keeping things in bounds of uint32.
*/
void DWT_Delay(uint32_t us) // microseconds
{
    uint32_t startTick  = DWT->CYCCNT,
             targetTick = DWT->CYCCNT + us * (SystemCoreClock/1000000);

    // Must check if target tick is out of bounds and overflowed
    if (targetTick > startTick) {
        // Not overflowed
        while (DWT->CYCCNT < targetTick);
    } else {
        // Overflowed
        while (DWT->CYCCNT > startTick || DWT->CYCCNT < targetTick);
    }
}
#else
/**
 * Delay routine itself.
 * Time is in microseconds (1/1000000th of a second), not to be
 * confused with millisecond (1/1000th).
 *
 * No need to check an overflow. Let it just tick :)
 *
 * @param uint32_t us  Number of microseconds to delay for
 */
void DWT_Delay(uint32_t us) // microseconds
{

  DWT->CYCCNT = 0;
  uint32_t startTick = DWT->CYCCNT;
  uint32_t delayTicks = us * (SystemCoreClock/1000000);

  while (DWT->CYCCNT - startTick < delayTicks);
}

#endif
