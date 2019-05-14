/* Copyright 2016, 
 * Leandro D. Medus
 * lmedus@bioingenieria.edu.ar
 * Eduardo Filomena
 * efilomena@bioingenieria.edu.ar
 * Juan Manuel Reta
 * jmrera@bioingenieria.edu.ar
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

/** \brief Bare Metal driver for switchs in the EDU-CIAA board.
 **
 **/

/*
 * Initials     Name
 * ---------------------------
 *	LM			Leandro Medus
 *  EF			Eduardo Filomena
 *  JMR			Juan Manuel Reta
 *  SM			Sebastian Mateos
 */

/*
 * modification history (new versions first)
 * -----------------------------------------------------------
 * 20160422 v0.1 initials initial version leo
 * 20160807 v0.2 modifications and improvements made by Eduardo Filomena
 * 20160808 v0.3 modifications and improvements made by Juan Manuel Reta
 * 20180210 v0.4 modifications and improvements made by Sebastian Mateos
 */

/*==================[inclusions]=============================================*/
#include "chip.h"
#include "switch.h"

/*==================[macros and definitions]=================================*/
/* Mapeo de pines Pulsadores
 * P1_0  en GPIO 0[4], SW1
 * P1_1  en GPIO 0[8], SW2
 * P1_2  en GPIO 0[9], SW3
 * P1_6  en GPIO 1[9], SW4
 * */
#define SW1_MUX_GROUP 	1
#define SW1_MUX_PIN 	0
#define SW1_GPIO_PORT 	0
#define SW1_GPIO_PIN 	4

#define SW2_MUX_GROUP 	1
#define SW2_MUX_PIN 	1
#define SW2_GPIO_PORT 	0
#define SW2_GPIO_PIN 	8

#define SW3_MUX_GROUP 	1
#define SW3_MUX_PIN 	2
#define SW3_GPIO_PORT 	0
#define SW3_GPIO_PIN 	9

#define SW4_MUX_GROUP 	1
#define SW4_MUX_PIN 	6
#define SW4_GPIO_PORT 	1
#define SW4_GPIO_PIN 	9

#define OUTPUT_DIRECTION 1
#define INPUT_DIRECTION 0

/*==================[internal data declaration]==============================*/
void (*ptrTecIntFunc[8])(); /**< Pointer to the function to be called at the interruption of each key*/
void (*ptrTecGrupIntFunc)(); /**< Pointer to the function to be called in the group interruption of the keys*/

/*==================[internal functions declaration]=========================*/

/** /brief Interruption function of a gpio entry. Call the indicated function for any of the keys.
 */
void GPIO0_IRQHandler(void);

/** \brief Interruption function of a gpio entry. Call the indicated function for any of the keys.
 */
void GPIO1_IRQHandler(void);

/**\brief Interruption function of a gpio entry. Call the indicated function for any of the keys.
 */
void GPIO2_IRQHandler(void);

/** \brief Interruption function of a gpio entry. Call the indicated function for any of the keys.
 */
void GPIO3_IRQHandler(void);

/** \brief Interruption function of a group of gpio entries. Call the function indicated for the assigned keys.
 */
void GINT0_IRQHandler(void);

/*==================[internal data definition]===============================*/

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/

/*==================[external functions definition]==========================*/
/** \brief Initialize method for the basic push-buttons in the EDU-CIAA board */
uint8_t SwitchesInit(void)
{
	/** \details
	 * This function initialize the four switches present in the EDU-CIAA board,
	 * with the correct parameters with LPCOpen methods.
	 *
	 * \param none
	 *
	 * \return uint8_t: the function return a 8 bits word, where the first four
	 * binaries positions represent each push-button.
	 * */

	/* Configuración del GPIO*/
	Chip_GPIO_Init(LPC_GPIO_PORT);

	/* Mapeo de pines Pulsadores */
	Chip_SCU_PinMux(SW1_MUX_GROUP,SW1_MUX_PIN,MD_PUP|MD_EZI|MD_ZI,FUNC0);
	Chip_SCU_PinMux(SW2_MUX_GROUP,SW2_MUX_PIN,MD_PUP|MD_EZI|MD_ZI,FUNC0);
	Chip_SCU_PinMux(SW3_MUX_GROUP,SW3_MUX_PIN,MD_PUP|MD_EZI|MD_ZI,FUNC0);
	Chip_SCU_PinMux(SW4_MUX_GROUP,SW4_MUX_PIN,MD_PUP|MD_EZI|MD_ZI,FUNC0);

	/* Configuración como entrada para los pulsadores */
	Chip_GPIO_SetDir(LPC_GPIO_PORT, SW1_GPIO_PORT,1<<SW1_GPIO_PIN,INPUT_DIRECTION);
	Chip_GPIO_SetDir(LPC_GPIO_PORT, SW2_GPIO_PORT,1<<SW2_GPIO_PIN,INPUT_DIRECTION);
	Chip_GPIO_SetDir(LPC_GPIO_PORT, SW3_GPIO_PORT,1<<SW3_GPIO_PIN,INPUT_DIRECTION);
	Chip_GPIO_SetDir(LPC_GPIO_PORT, SW4_GPIO_PORT,1<<SW4_GPIO_PIN,INPUT_DIRECTION);

	return true;
}

uint8_t SwitchesRead(void)
{
	uint8_t mask = 0;
	if (!Chip_GPIO_ReadPortBit(LPC_GPIO_PORT, SW1_GPIO_PORT, SW1_GPIO_PIN))
		  mask |= SWITCH_1;
	if (!Chip_GPIO_ReadPortBit(LPC_GPIO_PORT, SW2_GPIO_PORT, SW2_GPIO_PIN))
		  mask |= SWITCH_2;
	if (!Chip_GPIO_ReadPortBit(LPC_GPIO_PORT, SW3_GPIO_PORT, SW3_GPIO_PIN))
		  mask |= SWITCH_3;
	if (!Chip_GPIO_ReadPortBit(LPC_GPIO_PORT, SW4_GPIO_PORT, SW4_GPIO_PIN))
		  mask |= SWITCH_4;
	return mask;
}

void SwitchActivInt(switchgp_t gp, uint8_t tec, void *ptrIntFunc)
{
	ptrTecIntFunc[gp] = ptrIntFunc;

	/*Configura el canal de interrupción */
	if(tec == SWITCH_1)
		Chip_SCU_GPIOIntPinSel(gp, SW1_GPIO_PORT, SW1_GPIO_PIN);
	else if(tec == SWITCH_2)
		Chip_SCU_GPIOIntPinSel(gp, SW2_GPIO_PORT, SW2_GPIO_PIN);
	else if(tec == SWITCH_3)
		Chip_SCU_GPIOIntPinSel(gp, SW3_GPIO_PORT, SW3_GPIO_PIN);
	else if(tec == SWITCH_4)
		Chip_SCU_GPIOIntPinSel(gp, SW4_GPIO_PORT, SW4_GPIO_PIN);

	Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH(gp)); /* Limpia el estado de la interrupcion*/
	Chip_PININT_SetPinModeEdge(LPC_GPIO_PIN_INT, PININTCH(gp)); /* Interrupcion por flanco*/
	Chip_PININT_EnableIntLow(LPC_GPIO_PIN_INT, PININTCH(gp)); /* Interrupcion cuando el flanco es descendente*/

	NVIC_ClearPendingIRQ(32+gp); /* Limpia la interrupcion de PIN_INTX_IRQn definido en cmsis_43xx.h*/
	NVIC_EnableIRQ(32+gp); /* Habilita la interrupcion de PIN_INTX_IRQn definido en cmsis_43xx.h*/
}

void SwitchesActivGroupInt(uint8_t tecs, void *ptrIntFunc)
{
	ptrTecGrupIntFunc = ptrIntFunc;
	Chip_GPIOGP_SelectOrMode(LPC_GPIOGROUP, 0); /* Cualquier pulsador dispara la interrupcion*/
	Chip_GPIOGP_SelectEdgeMode(LPC_GPIOGROUP, 0); /* Interrupcion por flanco*/
	Chip_GPIOGP_ClearIntStatus(LPC_GPIOGROUP, 0); /* Limpia el estado de la interrupcion*/

	if(tecs == SWITCH_1)
	{
		Chip_GPIOGP_SelectLowLevel(LPC_GPIOGROUP, 0, SW1_GPIO_PORT, 1<<SW1_GPIO_PIN);/* Interrupcion por nivel bajo*/
		Chip_GPIOGP_EnableGroupPins(LPC_GPIOGROUP, 0, SW1_GPIO_PORT, 1<<SW1_GPIO_PIN); /* Habilita el pin para la interrupcion de grupo*/
	}
	if(tecs == SWITCH_2)
	{
		Chip_GPIOGP_SelectLowLevel(LPC_GPIOGROUP, 0, SW2_GPIO_PORT, 1<<SW2_GPIO_PIN);/* Interrupcion por nivel bajo*/
		Chip_GPIOGP_EnableGroupPins(LPC_GPIOGROUP, 0, SW2_GPIO_PORT, 1<<SW2_GPIO_PIN); /* Habilita el pin para la interrupcion de grupo*/
	}
	if(tecs == SWITCH_3)
	{
		Chip_GPIOGP_SelectLowLevel(LPC_GPIOGROUP, 0, SW3_GPIO_PORT, 1<<SW3_GPIO_PIN);/* Interrupcion por nivel bajo*/
		Chip_GPIOGP_EnableGroupPins(LPC_GPIOGROUP, 0, SW3_GPIO_PORT, 1<<SW3_GPIO_PIN); /* Habilita el pin para la interrupcion de grupo*/
	}
	if(tecs == SWITCH_4)
	{
		Chip_GPIOGP_SelectLowLevel(LPC_GPIOGROUP, 0, SW4_GPIO_PORT, 1<<SW4_GPIO_PIN);/* Interrupcion por nivel bajo*/
		Chip_GPIOGP_EnableGroupPins(LPC_GPIOGROUP, 0, SW4_GPIO_PORT, 1<<SW4_GPIO_PIN); /* Habilita el pin para la interrupcion de grupo*/
	}

    NVIC_EnableIRQ(40); /* Habilita la interrupcion de GINT0_IRQn definido en cmsis_43xx.h*/
}

void GPIO0_IRQHandler(void) //32
{
	Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH(GPIOGP0));
	ptrTecIntFunc[0]();
}

void GPIO1_IRQHandler(void) //33
{
	Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH(GPIOGP1));
	ptrTecIntFunc[1]();
}

void GPIO2_IRQHandler(void) //34
{
	Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH(GPIOGP2));
	ptrTecIntFunc[2]();
}

void GPIO3_IRQHandler(void) //35
{
	Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH(GPIOGP3));
	ptrTecIntFunc[3]();
}

void GINT0_IRQHandler(void) // 40
{
	Chip_GPIOGP_ClearIntStatus(LPC_GPIOGROUP, 0);
	ptrTecGrupIntFunc();
}

/** @} doxygen end group definition */
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/


/*
 * Para los pulsadores
 * Chip_GPIO_ReadValue()
 * Chip_GPIO_ReadPortBit()
 * */
