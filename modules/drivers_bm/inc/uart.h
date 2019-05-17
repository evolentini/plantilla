/* Copyright 2018,
 * Sebastian Mateos
 * sebastianantoniomateos@gmail.com
 * Cátedra Electrónica Programable
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

/** @brief Driver Bare Metal para los pulsadores y teclas de la EDU-CIAA.
 **/

/*
 * Initials     Name
 * ---------------------------
 *  SM			Sebastian Mateos
 */

/*
 * modification history
 * -----------------------------------------------------------
 * 20180922 v0.1 version inicial SM
 */

#ifndef UART_H
#define UART_H



/*==================[inclusions]=============================================*/
#include <stdint.h>
#include "bool.h"

/*==================[macros]=================================================*/
#define BUFFSize 64 /**< Cantidad máxima de valores a alojar en el buffer de entrada y en el de salida*/
#define BAUDRATE_USB 115200 /**< Baudios por segundo de la uart USB*/
#define BAUDRATE_RS232 9600 /**< Baudios por segundo de la uart RS232*/
#define BAUDRATE_RS485 115200 /**< Baudios por segundo de la uart RS485*/

/*==================[typedef]================================================*/
/** @typedef struct configUart_t
 * @brief Opciones de configuracion de las uart de la EDU-CIAA
 **/
typedef struct
{
	uint32_t uart;
	uint8_t rxHwPort;
	uint8_t rxHwPin;
	uint8_t txHwPort;
	uint8_t txHwPin;
	uint8_t func;
	uint32_t baudrate;
	uint32_t configOpt;
	uint32_t configFifo;
}configUart_t;

/** @typedef enum uart_t
 * @brief Uarts disponibles en la EDU-CIAA
 */
typedef enum{
	UART_USB=0,
	UART_RS232,
	UART_RS485
}uart_t;

/** @typedef enum intFlag_t
 * @brief Opciones de configuracion de las interrupciones de la uart de la EDU-CIAA
 **/
typedef enum
{
	UART_RBRINT = (1 << 0),	/*!< RBR Interrupt enable */
	UART_THREINT = (1 << 1),	/*!< THR Interrupt enable */
	UART_RLSINT = (1 << 2),	/*!< RX line status interrupt enable */
	UART_MSINT = (1 << 3),	/*!< Modem status interrupt enable - valid for 11xx, 17xx/40xx UART1, 18xx/43xx UART1  only */
	UART_CTSINT = (1 << 7),	/*!< CTS signal transition interrupt enable - valid for 17xx/40xx UART1, 18xx/43xx UART1 only */
	UART_ABEOINT = (1 << 8),	/*!< Enables the end of auto-baud interrupt */
	UART_ABTOINT = (1 << 9)	/*!< Enables the auto-baud time-out interrupt */
}intFlag_t;

/*==================[external data declaration]==============================*/

/*==================[internal functions declaration]=========================*/
/** @fn void UARTInit(uart_t u)
 * @brief Inicializacion de la UART FTDI, de modo de trabajar sin interrupciones
 *  y utilizar funciones de lectura y escritura blockeantes
 * @param[in] u Indica que Uart va a inicializar
 */
void UARTInit(uart_t u);

/** @fn void UARTActivInt(uart_t u, void *ptrIntFunc)
 * @brief Activa la interrupcion de la uart y realiza la funcion pasada por parametro
 * @param[in] u Indica a que Uart  va a activar la interrupcion
 * @param[in] ptrIntFunc Puntero a la funcion que se desea realizar
 */
void UARTActivInt(uart_t u, void *ptrIntFunc);

/** @fn void UARTIntEnable(uart_t u, intFlag_t flag)
 * @brief Activa la interrupcion por recepcion de la uart y realiza la funcion pasada por parametro
 * @param[in] u Indica a que Uart le va a habilitar la interrupcion
 * @param[in] flag Bandera de interrupcion que se va a utilizar
 */
void UARTIntEnable(uart_t u, intFlag_t flag);

/** @fn void UARTIntDisable(uart_t u, intFlag_t flag)
 * @brief Habilita la interrupcion por recepcion de la uart y realiza la funcion pasada por parametro
 * @param[in] u Indica a que Uart le va a deshabilitar la interrupcion
 * @param[in] flag Bandera de interrupcion que se va a deshabilitar
 */
void UARTIntDisable(uart_t u, intFlag_t flag);

/** @fn uint32_t UARTTxState(uart_t u)
 * @brief Devuelve el estado de la linea de transmision de la uart
 * @param[in] u Indica de que Uart va a devolver el estado
 * @return Devuelve un true si el registro de transmision esta vacio
 */
uint32_t UARTTxState(uart_t u);

/** @fn uint32_t UARTRxState(uart_t u)
 * @brief Devuelve el estado de la linea de recepcion de la uart
 * @param[in] u Indica de que Uart va a devolver el estado
 * @return Devuelve un true si el registro de recepcion esta listo para recibir un dato
 */
uint32_t UARTRxState(uart_t u);

/** @fn uint8_t UARTReadByte(uart_t u, uint8_t* dat)
 * @brief Lectura de un byte de la uart
 * @param[in] uart Indica de que Uart va a leer el dato
 * @param[in] dat Puntero al buffer de recepcion
 * @return Devuelve un true si se leyo un dato o un false si no habia dato para leer
 */
uint8_t UARTReadByte(uart_t u, uint8_t* dat);

/** @fn void UARTSendByte(uart_t u, uint8_t* dat)
 * @brief Envio blockeante de un byte a traves de la uart
 * @param[in] u Indica a traves de que Uart va a enviar el dato
 * @param[in] dat Puntero al buffer de transmision
 */
void UARTSendByte(uart_t u, uint8_t* dat);

/** @fn void UARTSendString(uart_t u, uint8_t* msg)
 * @brief Envio blockeante de una cadena de bytes a traves de la uart
 * @param[in] u Indica a traves de que Uart va a enviar la cadena
 * @param[in] msg Puntero al buffer de transmision
 */
void UARTSendString(uart_t u, uint8_t* msg);

/** @fn void UARTInitRingBuffer(uart_t u)
 * @brief Inicializacion del buffer circular para la uart elegida
 * @param[in] u Indica para que Uart va a inicializar el buffer
 */
void UARTInitRingBuffer(uart_t u);

/** @fn unsigned int UARTSendRingBuffer(uart_t u, const uint8_t *data, unsigned int dataLen)
 * @brief Envio a traves del buffer circular de una cantidad limitada por BUFSize de datos
 * @param[in] u Indica a traves de que Uart va a enviar los datos
 * @param[in] dat Puntero al buffer de transmision
 * @param[in] datLen Longitud del buffer de transmision
 * @return Cantidad de datos transmitidos
 */
unsigned int UARTSendRingBuffer(uart_t u, const uint8_t *dat, unsigned int datLen);

/** @fn unsigned int UARTReadRingBuffer(uart_t u, uint8_t *data, unsigned int maxDat)
 * @brief Recepcion de datos a traves del buffer circular
 * @param[in] u Indica a traves de que Uart se van a recibir los datos
 * @param[in] dat Puntero al buffer de recepcion
 * @param[in] maxDat Cantidad maxima de datos que se va a recibir
 * @return Cantdidad de datos recibidos
 */
unsigned int UARTReadRingBuffer(uart_t u, uint8_t *dat, unsigned int maxDat);

/** @fn char* UARTItoa(uint32_t val, uint8_t base)
 * @brief Conversor de entero a ASCII
 * @param[in] val Valor entero que se desea convertir
 * @param[in] base Base sobre la cual se desea realizar la conversion
 * @return Puntero al primer elemento de la cadena convertida
 */
char* UARTItoa(uint32_t val, uint8_t base);

/*==================[end of file]============================================*/
#endif /* UART_H */

