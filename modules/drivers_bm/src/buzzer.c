	/* Copyright 2019,
 * Sebastian Mateos
 * smateos@ingenieria.uner.edu.ar
 * Facultad de Ingeniería
 * Universidad Nacional de Entre Ríos
 * Argentina
 *
 * All rights reserved.
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

/** \brief Bare Metal driver for buzzer in the EDU-CIAA board.
 **
 **/

/*
 * Initials     Name
 * ---------------------------
 * SM		Sebastian Mateos
 */

/*
 * modification history (new versions first)
 * -----------------------------------------------------------
 * 20190220 v0.1 SM initial version
 */

/*==================[inclusions]=============================================*/
#include "chip.h"
#include "buzzer.h"

/*==================[macros and definitions]=================================*/
/** Mapping CTOUT0 pin
 *
 * P4_2  en TFIL_2 como CTOUT0
 *
 * */
#define CTOUT1 1
#define CTOUT1_PORT 4
#define CTOUT1_PIN  1
#define CTOUT_FUNC FUNC1

#define INIT_FREC 1000

/*==================[internal data declaration]==============================*/
static bool init_buzzer = false; /**<  Indicates if the buzzer is initialized*/

/*==================[internal functions declaration]=========================*/

/*==================[internal data definition]===============================*/

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/


/*==================[external functions definition]==========================*/

/** \brief Initialization function */
uint8_t BuzzerInit(void)
{
	init_buzzer = true;

	/** Configuration of the SCT module a INIT_FREC*/
	Chip_SCTPWM_Init(LPC_SCT);
	Chip_SCTPWM_SetRate(LPC_SCT, INIT_FREC);

	/** Mapping of buzzer pin*/
	Chip_SCU_PinMux(CTOUT1_PORT , CTOUT1_PIN , SCU_MODE_INACT , CTOUT_FUNC);
	Chip_SCTPWM_SetOutPin(LPC_SCT, CTOUT1+1 , CTOUT1);

	/** Set duty cycle 50%*/
	Chip_SCTPWM_SetDutyCycle(LPC_SCT, CTOUT1+1, (Chip_SCTPWM_GetTicksPerCycle(LPC_SCT)/2));

	return init_buzzer;
}

/** \brief Function to turn on buzzer */
void BuzzerOn(void)
{
	if(!init_buzzer)
		BuzzerInit();

	Chip_SCTPWM_Start(LPC_SCT);
}

/** \brief Function to turn off buzzer */
void BuzzerOff(void)
{
	if(!init_buzzer)
		BuzzerInit();
	
	Chip_SCTPWM_Stop(LPC_SCT);
}

/** \brief Function to turn off buzzer */
void BuzzerSetFrec(uint16_t frec)
{
	if(!init_buzzer)
		BuzzerInit();
	Chip_SCTPWM_SetRate(LPC_SCT, frec);
	Chip_SCTPWM_Start(LPC_SCT);
	Chip_SCTPWM_SetDutyCycle(LPC_SCT, CTOUT1+1, (Chip_SCTPWM_GetTicksPerCycle(LPC_SCT)/2));
}

/*==================[end of file]============================================*/
