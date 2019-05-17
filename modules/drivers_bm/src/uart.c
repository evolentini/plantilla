/* Copyright 2018,
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

/*==================[inclusions]=============================================*/
#include "uart.h"
#include "chip.h"


/*==================[macros and definitions]=================================*/
/*Direction Pin*/
#define DIR_RS485_MUX_GROUP 	6
#define DIR_RS485_MUX_PIN 	2

/*==================[internal data declaration]==============================*/

/*==================[internal data definition]===============================*/
static bool initRing[3] = {false, false, false}; /**< Indica si se ha llamado a la funcion de inicializacion del ring buffer*/
static bool initInt[3] = {false, false, false}; /**< Indica si se ha llamado a la funcion de activacion de interrupciones*/

static unsigned char rxBuff[3][BUFFSize]; /**< Vector para contener los datos asociados el buffer de entrada */
static unsigned char txBuff[3][BUFFSize]; /**< Vector para contener los datos asociados el buffer de salida */

static RINGBUFF_T txRing[3]; /**< Estructura asociado al buffer circular de transmisión. */
static RINGBUFF_T rxRing[3]; /**< Estructura asociada al buffer circular de recepción.  */

void (*ptrUartFunc[3])(); /**< Puntero a la funcion que se va a llamar en la interrupcion de la uart*/

const configUart_t uarts[] =
{
	{0x400C1000, 0x07, 0x02, 0x07, 0x01, FUNC6, BAUDRATE_USB,  UART_LCR_WLEN8 | UART_LCR_SBS_1BIT | UART_LCR_PARITY_DIS,  UART_FCR_FIFO_EN | UART_FCR_TRG_LEV0}, /*!< Configuracion LPC_USART2 (UART_USB)*/
	{0x400C2000, 0x02, 0x04, 0x02, 0x03, FUNC2, BAUDRATE_RS232, UART_LCR_WLEN8 | UART_LCR_SBS_1BIT | UART_LCR_PARITY_DIS, UART_FCR_FIFO_EN | UART_FCR_TRG_LEV3}, /*!< Configuracion LPC_USART3 (UART_RS232)*/
	{0x40081000, 0x09, 0x06, 0x09, 0x05, FUNC7, BAUDRATE_RS485, UART_LCR_WLEN8 | UART_LCR_SBS_1BIT | UART_LCR_PARITY_DIS, UART_FCR_FIFO_EN | UART_FCR_TRG_LEV0} /*!< Configuracion LPC_USART0 (UART_RS485)*/

};

/*==================[internal functions declaration]=========================*/

/*==================[external data definition]===============================*/

/*==================[external functions definition]==========================*/
/** @fn void UART2_IRQHandler (void)
 *  @brief Se llama cada vez que la UART_USB recibe un dato.
 *  LLama a la funcion indicada en la inicializacion o a la funcion que carga el ring buffer si el mismo esta configurado.
 */
void UART2_IRQHandler (void);

/** @fn void UART2_IRQHandler (void)
 *  @brief Se llama cada vez que la UART_RS232 recibe un dato.
 *  LLama a la funcion indicada en la inicializacion o a la funcion que carga el ring buffer si el mismo esta configurado.
 */
void UART3_IRQHandler (void);

/** @fn void UART2_IRQHandler (void)
 *  @brief Se llama cada vez que la UART_RS485 recibe un dato.
 *  LLama a la funcion indicada en la inicializacion o a la funcion que carga el ring buffer si el mismo esta configurado.
 */
void UART0_IRQHandler (void);

/*==================[internal functions definition]==========================*/

void UARTInit(uart_t u)
{
    Chip_SCU_PinMux(uarts[u].txHwPort, uarts[u].txHwPin, MD_PDN, uarts[u].func);
    Chip_SCU_PinMux(uarts[u].rxHwPort, uarts[u].rxHwPin, MD_PLN|MD_EZI|MD_ZI, uarts[u].func);
	Chip_UART_Init((LPC_USART_T *)uarts[u].uart);
    Chip_UART_ConfigData((LPC_USART_T *)uarts[u].uart, uarts[u].configOpt);
    Chip_UART_SetBaud((LPC_USART_T *)uarts[u].uart, uarts[u].baudrate);
    Chip_UART_SetupFIFOS((LPC_USART_T *)uarts[u].uart, uarts[u].configFifo);
    Chip_UART_TXEnable((LPC_USART_T *)uarts[u].uart);
    if(u == UART_RS485)
    {
    	Chip_UART_SetRS485Flags(LPC_USART0, UART_RS485CTRL_DCTRL_EN | UART_RS485CTRL_OINV_1);
    	Chip_SCU_PinMux(DIR_RS485_MUX_GROUP, DIR_RS485_MUX_PIN, MD_PDN, FUNC2);              /* P6_2: UART0_DIR */
    }
}

void UARTActivInt(uart_t u, void *ptrIntFunc)
{
	ptrUartFunc[u] = ptrIntFunc;
	switch(u)
		{
		case UART_USB:
			NVIC_SetPriority(26, 0x1f);
			NVIC_EnableIRQ(26); /* USART2_IRQn definido en cmsis_43xx.h*/
			break;
		case UART_RS232:
			NVIC_SetPriority(27, 0x1f);
			NVIC_EnableIRQ(27); /* USART3_IRQn definido en cmsis_43xx.h*/
			break;
		case UART_RS485:
			NVIC_SetPriority(28, 0x1f);
			NVIC_EnableIRQ(24); /* USART0_IRQn definido en cmsis_43xx.h*/
			break;
		}
	initInt[u] = true;
}
void UARTIntEnable(uart_t u, intFlag_t flag)
{
	Chip_UART_IntEnable((LPC_USART_T *)uarts[u].uart, flag);
}

void UARTIntDisable(uart_t u, intFlag_t flag)
{
	Chip_UART_IntDisable((LPC_USART_T *)uarts[u].uart, flag);
}

uint32_t UARTTxState(uart_t u)
{
	return Chip_UART_ReadLineStatus((LPC_USART_T *)uarts[u].uart) & UART_LSR_THRE; /* Line status: Transmit holding register empty */
}

uint32_t UARTRxState(uart_t u)
{
	return Chip_UART_ReadLineStatus((LPC_USART_T *)uarts[u].uart) & UART_LSR_RDR; /* Line status: Receive data ready */
}

uint8_t UARTReadByte(uart_t u, uint8_t* dat)
{
	if (UARTRxState(u))
	{
		*dat = Chip_UART_ReadByte((LPC_USART_T *)uarts[u].uart);
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}

void UARTSendByte(uart_t u, uint8_t* dat)
{
	while(UARTTxState(u) == 0);
	Chip_UART_SendByte((LPC_USART_T *)uarts[u].uart, *dat);
}

void UARTSendString(uart_t u, uint8_t* msg)
{
	while(*msg != 0)
	{
		while(UARTTxState(u) == 0);
		Chip_UART_SendByte((LPC_USART_T *)uarts[u].uart, (uint8_t)*msg);
		msg++;
	}
}

void UART2_IRQHandler (void)
{
	if(initRing[0])
		Chip_UART_IRQRBHandler((LPC_USART_T *)uarts[0].uart, &rxRing[0], &txRing[0]);
	if(initInt[0])
		ptrUartFunc[0]();
}

void UART3_IRQHandler(void)
{
	if(initRing[1])
		Chip_UART_IRQRBHandler((LPC_USART_T *)uarts[1].uart, &rxRing[1], &txRing[1]);
	if(initInt[1])
		ptrUartFunc[1]();
}

void UART0_IRQHandler (void)
{
	if(initRing[2])
		Chip_UART_IRQRBHandler((LPC_USART_T *)uarts[2].uart, &rxRing[2], &txRing[2]);
	if(initInt[2])
		ptrUartFunc[2]();
}

void UARTInitRingBuffer(uart_t u)
{
	RingBuffer_Init(&rxRing[u], rxBuff[u], 1, BUFFSize); /* Buffer de recepcion de 1 byte*/
	RingBuffer_Init(&txRing[u], txBuff[u], 1, BUFFSize);/* Buffer de transmision de 1 byte*/
	Chip_UART_SetupFIFOS((LPC_USART_T *)uarts[u].uart, (UART_FCR_FIFO_EN | UART_FCR_RX_RS | UART_FCR_TX_RS | UART_FCR_TRG_LEV3));
	Chip_UART_IntEnable((LPC_USART_T *)uarts[u].uart, (UART_IER_RBRINT | UART_IER_RLSINT));
	switch(u)
	{
	case UART_USB:
		NVIC_EnableIRQ(26); /* USART2_IRQn definido en cmsis_43xx.h*/
		break;
	case UART_RS232:
		NVIC_EnableIRQ(27); /* USART3_IRQn definido en cmsis_43xx.h*/
		break;
	case UART_RS485:
		NVIC_EnableIRQ(24); /* USART0_IRQn definido en cmsis_43xx.h*/
		break;
	}
	initRing[u] = true;
}

unsigned int UARTSendRingBuffer(uart_t u, const uint8_t *dat, unsigned int datLen)
{
	unsigned int transmited = 0;
	unsigned int toInsert;
	unsigned int freeSpc;
	while(transmited < datLen)
	{
		freeSpc = RingBuffer_GetFree(&txRing[u]);
		while(!freeSpc)
			freeSpc = RingBuffer_GetFree(&txRing[u]);
		toInsert = ((freeSpc > datLen) ? datLen : freeSpc);
		Chip_UART_SendRB((LPC_USART_T *)uarts[u].uart, &txRing[u], &dat[transmited], toInsert);
		transmited += toInsert;
	}
	return transmited;
}

unsigned int UARTReadRingBuffer(uart_t u, uint8_t *dat, unsigned int maxDat)
{
	return Chip_UART_ReadRB((LPC_USART_T *)uarts[u].uart, &rxRing[u], dat, maxDat);
}

char* UARTItoa(uint32_t val, uint8_t base)
{
	static char buf[32] = {0};

	uint32_t i = 30;

	for(; val && i ; --i, val /= base)

		buf[i] = "0123456789abcdef"[val % base];

	return &buf[i+1];
}

/*==================[end of file]============================================*/
