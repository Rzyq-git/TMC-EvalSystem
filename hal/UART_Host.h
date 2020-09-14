/*
 * UART_Host.h
 *
 *  Created on: 14.09.2020
 *      Author: LK
 */

#ifndef TMC_EVALSYSTEM_HAL_UART_HOST_H_
#define TMC_EVALSYSTEM_HAL_UART_HOST_H_

#include "UART.h"

UART_Config UART_Host;

void UART0_RX_TX_IRQHandler_Host(void);
void UART2_RX_TX_IRQHandler_Host(void);

#endif /* TMC_EVALSYSTEM_HAL_UART_HOST_H_ */
