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
#ifndef LED_H
#define LED_H


/*==================[inclusions]=============================================*/
#include "bool.h"
#include <stdint.h>

/*==================[macros]=================================================*/
#define lpc4337            1
#define mk60fx512vlq15     2

/*==================[typedef]================================================*/

/*==================[external data declaration]==============================*/
/** \brief Definition of constants to reference the EDU-CIAA leds.
 **
 **/
enum LED_COLOR {
	LED_1 = 1,
	LED_2 = 2,
	LED_3 = 4,
	LED_RGB_B = 8,
	LED_RGB_G = 16,
	LED_RGB_R = 32
};


/** \brief Definition of constants to control the EDU-CIAA leds.
 **
 **/


/*==================[external functions declaration]=========================*/

/** \brief Initialization function of EDU-CIAA leds
 **
 ** \Mapping ports (PinMux function), set direction and initial state of leds ports
 **
 ** \param[in] No parameter
 ** 
 ** \return TRUE if no error
 **/
uint8_t LedsInit(void);


/** \brief Function to turn on a specific led 
 ** 
 ** \param[in] led 
 **
 ** \return FALSE if an error occurs, in other case returns TRUE
 **/
uint8_t LedOn(uint8_t led);

/** \brief Function to turn off a specific led 
 ** 
 ** \param[in] led 
 **
 ** \return FALSE if an error occurs, in other case returns TRUE
 **/
uint8_t LedOff(uint8_t led);

/** \brief Function to toggle a specific led 
 ** 
 ** \param[in] led 
 **
 ** \return FALSE if an error occurs, in other case returns TRUE
 **/
uint8_t LedToggle(uint8_t led);

/** \brief Function to turn off all led 
 ** 
 ** \param[in] No Parameter 
 **
 ** \return FALSE if an error occurs, in other case returns TRUE
 **/
uint8_t LedsOffAll(void);

/** \brief Function to turn on or off leds from a mask.
 **
 ** \param[in] mask
 **
 **/
void LedsMask(uint8_t mask);

/*==================[end of file]============================================*/
#endif /* #ifndef LED_H */

