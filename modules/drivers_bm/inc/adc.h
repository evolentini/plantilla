/* Copyright 2019,
 * Sebastian Mateos
 * smateos@ingenieria.uner.edu.ar
 * Leandro D. Medus
 * lmedus@bioingenieria.edu.ar
 * Eduardo Filomena
 * efilomena@bioingenieria.edu.ar
 * Juan Manuel Reta
 * jmrera@bioingenieria.edu.ar
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


#ifndef ADC_H
#define ADC_H

/** @brief AD Converter Bare Metal driver for the peripheral in the EDU-CIAA Board.
 *
 * This is a driver to control the peripheral Analog to Digital Converter.
 *
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
 * modification history
 * -----------------------------------------------------------
 * 20160610 v0.1 initials initial version leo
 * 20160807 v0.2 modifications and improvements made by Eduardo Filomena
 * 20160808 v0.3 modifications and improvements made by Juan Manuel Reta
 * 20190407 v1.0 modifications and improvements made by Sebastian Mateos
 */

/*==================[inclusions]=============================================*/
#include <stdint.h>
#include "bool.h"
/*==================[macros]=================================================*/


/*==================[typedef]================================================*/
/** @typedef The channels on one ADC peripheral*/
typedef enum {
	CH1 = 1,		/**< ADC channel 1 */
	CH2,		/**< ADC channel 2 */
	CH3		/**< ADC channel 3 */
} ADC_CHANNEL_t;

/** @typedef The ADC peripheral on the chip*/
typedef enum {
	ADC0 = 0,	/**< ADC channel 0 */
	ADC1		/**< ADC channel 1 */
} ADC_t;

/*==================[external data declaration]==============================*/


/*==================[external functions declaration]=========================*/
/** @brief Initialization function for ADC in the EDU-CIAA BOARD
 *
 * @param[in] periph Peripheral that will perform the conversion
 * @param[in] adc_channel Channel of the peripheral
 *
 * @return TRUE if no error
 **/
uint8_t ADCInit(ADC_t periph, ADC_CHANNEL_t adc_channel);

/** @brief reads value pooling method (blocking)
 *
 * @param[in] periph Peripheral that will perform the conversionby pooling
 * @param[in] adc_channel Channel of the peripheral
 *
 * @return ADC value (10 bits)
 **/
uint16_t ADCReadValuePooling(ADC_t periph, ADC_CHANNEL_t adc_channel);

/** @brief start of conversion
 *
 * @param[in] periph Peripheral that will perform the conversion
 * @param[in] adc_channel Channel of the peripheral
 *
 *
 **/
void ADCStart(ADC_t periph, ADC_CHANNEL_t adc_channel);

/** @brief reads converted value (non blocking)
 *
 * @param[in] periph Peripheral wich perform the conversion
 * @param[in] adc_channel Channel of the peripheral
 *
 * @return ADC value (10 bits)
 **/
uint16_t ADCReadValue(ADC_t periph, ADC_CHANNEL_t adc_channel);

/** @brief enable end of convertion interrupt
 *
 * @param[in] periph Peripheral that will perform the conversion
 * @param[in] adc_channel Channel of the peripheral
 * @param[in] adress of ISR (Interrupt Servide Routine)
 *
 **/
void ADCActivInt(ADC_t periph, ADC_CHANNEL_t adc_channel, void *pfunc);

/*==================[end of file]============================================*/
#endif /* #ifndef ADC_H */

