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

/** \brief Bare Metal driver for adc in the EDU-CIAA board.
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
 * modification history
 * -----------------------------------------------------------
 * 20160422 v0.1 initials initial version leo
 * 20160807 v0.2 modifications and improvements made by Eduardo Filomena
 * 20160808 v0.3 modifications and improvements made by Juan Manuel Reta
 * 20190407 v1.0 modifications and improvements made by Sebastian Mateos
 */


/*==================[inclusions]=============================================*/
#include "adc.h"
#include "chip.h"


/*==================[macros and definitions]=================================*/

/*==================[internal data declaration]==============================*/
void (*pIsrADC0[2])();
void (*pIsrADC1[2])();

uint8_t isrADC0 = 0;
uint8_t isrADC1 = 0;
/*==================[internal functions declaration]=========================*/

/*==================[internal data definition]===============================*/

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/
void ISR_ADC0()
{
	if(isrADC0 & 1<<CH1)
		pIsrADC0[0]();
	if(isrADC0 & 1<<CH2)
		pIsrADC0[1]();
	if(isrADC0 & 1<<CH3)
		pIsrADC0[2]();
}
void ISR_ADC1()
{
	if(isrADC1 & 1<<CH1)
		pIsrADC1[0]();
	if(isrADC1 & 1<<CH2)
		pIsrADC1[1]();
	if(isrADC1 & 1<<CH3)
		pIsrADC1[2]();
}
/*==================[external functions definition]==========================*/
/** \brief ADC Initialization method  */
uint8_t ADCInit(ADC_t periph, ADC_CHANNEL_t adc_channel)
{

	/** \details
	 * This function initialize the ADC peripheral in the EDU-CIAA board,
	 * with the correct parameters with LPCOpen library. It uses CH1
	 *
	 * \param none
	 *
	 * \return uint8_t: TBD (to support errors in the init function)
	 * */
	static ADC_CLOCK_SETUP_T configADC;

	configADC.adcRate=1000;		/** max 409 KHz*/
	configADC.burstMode=DISABLE;
	configADC.bitsAccuracy=ADC_10BITS;
	switch(periph)
	{
	case ADC0:
		Chip_ADC_Init(LPC_ADC0,&configADC);
		Chip_ADC_EnableChannel(LPC_ADC0,adc_channel,ENABLE);
		Chip_ADC_SetSampleRate(LPC_ADC0, &configADC,ADC_MAX_SAMPLE_RATE);
		break;
	case ADC1:
		Chip_ADC_Init(LPC_ADC1,&configADC);
		Chip_ADC_EnableChannel(LPC_ADC1,ADC_CH1,ENABLE);
		Chip_ADC_SetSampleRate(LPC_ADC1, &configADC,ADC_MAX_SAMPLE_RATE);
		break;
	}
	return TRUE;
}

/** \brief ADC Ch1 Acquisition method by pooling */
uint16_t ADCReadValuePooling(ADC_t periph, ADC_CHANNEL_t adc_channel)
{
	/** \details
	 * This function initialize the DAC peripheral in the EDU-CIAA board,
	 * with the correct parameters with LPCOpen methods.
	 *
	 * \param none
	 *
	 * \return uint8_t: TBD (to support errors in the init function)
	 * */
	uint16_t valueRead = 0 ;
	switch(periph)
		{
		case ADC0:
			/** Start Acquisition */
			Chip_ADC_SetStartMode(LPC_ADC0, ADC_START_NOW, ADC_TRIGGERMODE_RISING);
			/** The pooling magic! */
			while (Chip_ADC_ReadStatus(LPC_ADC0, adc_channel, ADC_DR_DONE_STAT) != SET)
			{
				/** pooooliiinnggg maaagggicccc plif plif pluf pluf */
			}
			/** Conversion complete, and value reading */
			Chip_ADC_ReadValue(LPC_ADC0,adc_channel, &valueRead);
			break;
		case ADC1:
			/** Start Acquisition */
			Chip_ADC_SetStartMode(LPC_ADC1, ADC_START_NOW, ADC_TRIGGERMODE_RISING);
			/** The pooling magic! */
			while (Chip_ADC_ReadStatus(LPC_ADC1, adc_channel, ADC_DR_DONE_STAT) != SET)
			{
				/** pooooliiinnggg maaagggicccc plif plif pluf pluf */
			}
			/** Conversion complete, and value reading */
			Chip_ADC_ReadValue(LPC_ADC1,adc_channel, &valueRead);
			break;
		}

	return valueRead;
}

/** Start Acquisition */
void ADCStart(ADC_t periph, ADC_CHANNEL_t adc_channel)
{
	switch(periph)
	{
		case ADC0:
			Chip_ADC_SetStartMode(LPC_ADC0, ADC_START_NOW, ADC_TRIGGERMODE_RISING);
			break;
		case ADC1:
			Chip_ADC_SetStartMode(LPC_ADC1, ADC_START_NOW, ADC_TRIGGERMODE_RISING);
			break;
	}
}

uint16_t ADCReadValue(ADC_t periph, ADC_CHANNEL_t adc_channel)
{
	uint16_t data;
	switch(periph)
	{
		case ADC0:
			  Chip_ADC_ReadValue(LPC_ADC0,adc_channel, &data);
			break;
		case ADC1:
			  Chip_ADC_ReadValue(LPC_ADC1,adc_channel, &data);
			break;
	}
	return data;
}
  
void ADCActivInt(ADC_t periph, ADC_CHANNEL_t adc_channel, void *pfunc)
{
	/*Enable interrupt for ADC channel */
	switch(periph)
	{
		case ADC0:
			Chip_ADC_Int_SetChannelCmd(LPC_ADC0, adc_channel, ENABLE);
			NVIC_EnableIRQ(ADC0_IRQn);
			pIsrADC0[adc_channel]=pfunc;
			isrADC0 |= 1<<adc_channel;
			break;
		case ADC1:
			Chip_ADC_Int_SetChannelCmd(LPC_ADC0, adc_channel, ENABLE);
			NVIC_EnableIRQ(ADC1_IRQn);
			pIsrADC1[adc_channel]=pfunc;
			isrADC1 |= 1<<adc_channel;
			break;
		}
}

void ADCDeactivInt(ADC_t periph, ADC_CHANNEL_t adc_channel)
{
	/*Disable interrupt for ADC channel */
	switch(periph)
	{
		case ADC0:
			Chip_ADC_Int_SetChannelCmd(LPC_ADC0,adc_channel,DISABLE);
			NVIC_DisableIRQ(ADC0_IRQn);
			isrADC0 &= ~(1<<adc_channel);
			break;
		case ADC1:
			Chip_ADC_Int_SetChannelCmd(LPC_ADC0,adc_channel,DISABLE);
			NVIC_DisableIRQ(ADC1_IRQn);
			isrADC1 &= ~(1<<adc_channel);
			break;
		}
}

/*==================[end of file]============================================*/
