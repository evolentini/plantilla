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


#ifndef SWITCH_H
#define SWITCH_H
/** \brief Bare Metal header for switches in EDU-CIAA NXP
 **
 ** This is a driver for four switches mounted on the board
 **
 **/

/** \addtogroup CIAA_Firmware CIAA Firmware
 ** @{ */

/** \addtogroup Sources_LDM Leandro D. Medus Sources
 ** @{ */
/** \addtogroup Baremetal Bare Metal source file
 ** @{ */

/*
 * Initials     Name
 * ---------------------------
 *	LM			Leandro Medus
¨*  EF          Eduardo Filomena
 */

/*
 * modification history (new versions first)
 * -----------------------------------------------------------
 * 20160422 v0.1 initials initial version leo
 * 20160807 v0.2 modifications and improvements made by Eduardo Filomena 
 * 20160808 v0.3 modifications and improvements made by Juan Manuel Reta
 */

/*==================[inclusions]=============================================*/
#include "bool.h"
#include <stdint.h>

/*==================[macros]=================================================*/

/*==================[typedef]================================================*/
/** @typedef switchgp_t
 * @brief Define the key interrupt groups
 */
typedef enum {
	GPIOGP0=0, /**< Group of interruptions GPIO0_IRQHandler 32 */
	GPIOGP1, /**< Group of interruptions GPIO1_IRQHandler 33 */
	GPIOGP2, /**< Group of interruptions GPIO2_IRQHandler 34 */
	GPIOGP3, /**< Group of interruptions GPIO3_IRQHandler 35 */
} switchgp_t;

/*==================[external data declaration]==============================*/
enum SWITCHES {SWITCH_1=(1<<0), SWITCH_2=(1<<1), SWITCH_3=(1<<2), SWITCH_4=(1<<3)};

/*==================[external functions declaration]=========================*/
/** \brief Initialization function to control basic push-buttons in the EDU-CIAA BOARD 
 ** 
 ** \return TRUE if no error
 **/
uint8_t SwitchesInit(void);

/** \brief Function to read basic push-buttons 
 **
 ** \return 0 if no keypressed, SWITCH_1 SWITCH_2 SWITCH_3 SWITCH_4 in other case
 **/
uint8_t SwitchesRead(void);

/** \brief Enables the interruption of a particular key and assigns a group of interrupts defined in @ref switchgp_t
 * \param[in] gp Indicates to which group of interrupts the key is assigned
 * \param[in] tec Key that will interrupt
 * \param[in] ptrIntFunc Function to be called in the interruption
 */
void SwitchActivInt(switchgp_t gp, uint8_t tec, void *ptrIntFunc);

/** \brief Activate group interrupts and add to the interruption the chosen keys through the mask
 * \param[in] tecs Mask of keys that will be added to the group to interrupt. Where bit0-TEC4, bit1-TEC3, bit2-TEC2 and bit4-TEC1.
 * \param[in] void * Function to be called in the interruption.
 */
void SwitchesActivGroupInt(uint8_t tecs, void *ptrIntFunc);

/** @} doxygen end group definition */
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/
#endif /* #ifndef MI_NUEVO_PROYECTO_H */

