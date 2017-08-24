/* Copyright 2017, Martin Castro
 * All rights reserved.
 *
 * This file is part of CIAA Firmware.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

/** \brief source para MCU


/*==================[inclusions]=============================================*/
#include "mcu_pwm.h"
#include "mcu.h"
#include "mcu_gpio.h"
#include "stdint.h"
#include "chip.h"
#include "os.h"

/*==================[macros and definitions]=================================*/

/** \brief cantidad de callback que se pueden registrar */

/** \brief Dio Type */

/*==================[internal data declaration]==============================*/
mcu_gpio_pinId_enum *pino;
/*==================[internal functions declaration]=========================*/

/*==================[internal data definition]===============================*/

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/

/*==================[external functions definition]==========================*/
extern void mcu_pwm_init(void){
	 SystemCoreClockUpdate();
	 SysTick_Config(SystemCoreClock/1000);		/* por defecto incializa el periodo en 1 ms*/

	   /* Timer */
	   Chip_TIMER_Init(LPC_TIMER1);
	   Chip_TIMER_PrescaleSet(LPC_TIMER1,
	#ifdef lpc1769
	         Chip_Clock_GetPeripheralClockRate(SYSCTL_PCLK_TIMER1) / 1000000 - 1
	#else
			 Chip_Clock_GetRate(CLK_MX_TIMER1) / 1000000 - 1
	#endif
	   );

	   Chip_TIMER_Reset(LPC_TIMER1);
	   Chip_TIMER_Enable(LPC_TIMER1);

	   NVIC_EnableIRQ(TIMER1_IRQn);

}

extern void mcu_pwm_config(mcu_gpio_pinId_enum pin, uint32_t period) ​
{
	Chip_TIMER_MatchEnableInt(LPC_TIMER1, 0);
	Chip_TIMER_ResetOnMatchEnable(LPC_TIMER1, 0);
	Chip_TIMER_StopOnMatchDisable(LPC_TIMER1, 0);
	Chip_TIMER_SetMatch(LPC_TIMER1, 0, period);

	mcu_gpio_setOut(pin, false);				/*apago el pin indicado*/
	pino = &pin;
}


extern void mcu_pwm_setDutyCicle(mcu_gpio_pinId_enum pin, uint32_t duty)
{
	Chip_TIMER_MatchEnableInt(LPC_TIMER1, 1);
	Chip_TIMER_ResetOnMatchDisable(LPC_TIMER1, 1);
	Chip_TIMER_StopOnMatchDisable(LPC_TIMER1, 1);
	Chip_TIMER_SetMatch(LPC_TIMER1, 1, duty);

}



void TIMER1_IRQHandler(void)
{
   if (Chip_TIMER_MatchPending(LPC_TIMER1, 0)) {
      Chip_TIMER_ClearMatch(LPC_TIMER1, 0);
      mcu_gpio_setOut(*pino, true);				/*apago el pin indicado*/

   }
   if (Chip_TIMER_MatchPending(LPC_TIMER1, 1)) {
      Chip_TIMER_ClearMatch(LPC_TIMER1, 1);
      mcu_gpio_setOut(*pino, false);				/*apago el pin indicado*/
   }
}

/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/
