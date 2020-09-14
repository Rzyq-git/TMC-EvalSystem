/*
 * RXTX.c
 *
 *  Created on: 26.02.2019
 *      Author: LK
 */

#include "hal/RXTX.h"
#include "hal/WLAN.h"
#include "hal/UART.h"
#include "hal/UART_Host.h"

#if defined(ENABLE_UART_HOST_0)
UART0_Interrupt uart0_interrupt = UART0_INTERRUPT_HOST;
#elif defined(ENABLE_WLAN)
UART0_Interrupt uart0_interrupt = UART0_INTERRUPT_WLAN;
#else
UART0_Interrupt uart0_interrupt = UART0_INTERRUPT_UART;
#endif

#if defined(ENABLE_UART_HOST_2)
UART2_Interrupt uart2_interrupt = UART2_INTERRUPT_HOST;
#else
UART2_Interrupt uart2_interrupt = UART2_INTERRUPT_UART;
#endif

void UART0_RX_TX_IRQHandler(void)
{
	switch(uart0_interrupt) {
	case UART0_INTERRUPT_WLAN:
		UART0_RX_TX_IRQHandler_WLAN();
		break;
	case UART0_INTERRUPT_HOST:
		UART0_RX_TX_IRQHandler_Host();
		break;
	case UART0_INTERRUPT_UART:
	default:
		UART0_RX_TX_IRQHandler_UART();
		break;
	}
}

void UART2_RX_TX_IRQHandler(void)
{
	switch(uart2_interrupt) {
	case UART2_INTERRUPT_HOST:
		UART2_RX_TX_IRQHandler_Host();
		break;
	case UART2_INTERRUPT_UART:
	default:
		UART2_RX_TX_IRQHandler_UART();
		break;
	}
}
