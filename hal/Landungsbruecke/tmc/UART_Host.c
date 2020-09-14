#include "hal/HAL.h"
#include "hal/UART_Host.h"
#include "hal/Landungsbruecke/freescale/Cpu.h"

#define BUFFER_SIZE         32
#define INTR_PRI            6
#define UART_TIMEOUT_VALUE  10
#define WRITE_READ_DELAY    10

static void init();
static void deInit();
static void tx(uint8_t ch);
static uint8_t rx(uint8_t *ch);
static void txN(uint8_t *str, uint8_t number);
static uint8_t rxN(uint8_t *ch, uint8_t number);
static void clearBuffers(void);
static uint32_t bytesAvailable();

static volatile uint8_t
	rxBuffer[BUFFER_SIZE],
	txBuffer[BUFFER_SIZE];

static volatile uint32_t available = 0;

UART_Config UART_Host =
{
	.mode = UART_MODE_DUAL_WIRE,
	.pinout = UART_PINS_1,
	.rxtx =
	{
		.init            = init,
		.deInit          = deInit,
		.rx              = rx,
		.tx              = tx,
		.rxN             = rxN,
		.txN             = txN,
		.clearBuffers    = clearBuffers,
		.baudRate        = 4800,
		.bytesAvailable  = bytesAvailable
	}
};

static RXTXBufferingTypeDef buffers =
{
	.rx =
	{
		.read    = 0,
		.wrote   = 0,
		.buffer  = rxBuffer
	},

	.tx =
	{
		.read    = 0,
		.wrote   = 0,
		.buffer  = txBuffer
	}
};

static void init()
{
	register uint16_t ubd = (CPU_BUS_CLK_HZ / 16) / UART_Host.rxtx.baudRate;

	// One wire UART communication needs the TxD pin to be in open drain mode
	// and a pull-up resistor on the RxD pin.
	switch(UART_Host.pinout) {
	case UART_PINS_2:
		HAL.IOs->pins->DIO10.configuration.GPIO_Mode  = GPIO_Mode_AF3;  // TxD (DIO10)
		HAL.IOs->pins->DIO11.configuration.GPIO_Mode  = GPIO_Mode_AF3;  // RxD (DIO11)
		HAL.IOs->pins->DIO10.configuration.GPIO_OType = GPIO_OType_OD;  // TxD as open drain output
		HAL.IOs->pins->DIO11.configuration.GPIO_PuPd  = GPIO_PuPd_UP;   // RxD with pull-up resistor
		HAL.IOs->config->set(&HAL.IOs->pins->DIO10);
		HAL.IOs->config->set(&HAL.IOs->pins->DIO11);
		SIM_SCGC4 |= SIM_SCGC4_UART0_MASK;
		UART_C2_REG(UART0_BASE_PTR) &= ~(UART_C2_TE_MASK | UART_C2_RE_MASK );
		UART_C1_REG(UART0_BASE_PTR) = 0;
		UART_C4_REG(UART0_BASE_PTR) = 0;
		UART_BDH_REG(UART0_BASE_PTR) = (ubd >> 8) & UART_BDH_SBR_MASK;
		UART_BDL_REG(UART0_BASE_PTR) = (ubd & UART_BDL_SBR_MASK);
		UART_C2_REG(UART0_BASE_PTR) |= (UART_C2_TE_MASK | UART_C2_RE_MASK | UART_C2_RIE_MASK);
		enable_irq(INT_UART0_RX_TX-16);
		break;
	case UART_PINS_3:
		HAL.IOs->pins->WIRELESS_TX.configuration.GPIO_Mode  = GPIO_Mode_AF3;  // TxD (DIO10)
		HAL.IOs->pins->WIRELESS_RX.configuration.GPIO_Mode  = GPIO_Mode_AF3;  // RxD (DIO11)
		HAL.IOs->pins->WIRELESS_TX.configuration.GPIO_OType = GPIO_OType_OD;  // TxD as open drain output
		HAL.IOs->pins->WIRELESS_RX.configuration.GPIO_PuPd  = GPIO_PuPd_UP;   // RxD with pull-up resistor
		HAL.IOs->config->set(&HAL.IOs->pins->WIRELESS_TX);
		HAL.IOs->config->set(&HAL.IOs->pins->WIRELESS_RX);
		SIM_SCGC4 |= SIM_SCGC4_UART0_MASK;
		UART_C2_REG(UART0_BASE_PTR) &= ~(UART_C2_TE_MASK | UART_C2_RE_MASK );
		UART_C1_REG(UART0_BASE_PTR) = 0;
		UART_C4_REG(UART0_BASE_PTR) = 0;
		UART_BDH_REG(UART0_BASE_PTR) = (ubd >> 8) & UART_BDH_SBR_MASK;
		UART_BDL_REG(UART0_BASE_PTR) = (ubd & UART_BDL_SBR_MASK);
		UART_C2_REG(UART0_BASE_PTR) |= (UART_C2_TE_MASK | UART_C2_RE_MASK | UART_C2_RIE_MASK);
		enable_irq(INT_UART0_RX_TX-16);
		break;
	case UART_PINS_1:
	default:
		SIM_SCGC4 |= SIM_SCGC4_UART2_MASK;
		UART_C1_REG(UART2_BASE_PTR) = 0;
		switch(UART_Host.mode) {
		case UART_MODE_SINGLE_WIRE:
			HAL.IOs->pins->DIO17.configuration.GPIO_Mode  = GPIO_Mode_AF3;  // TxD (DIO17)
			HAL.IOs->pins->DIO18.configuration.GPIO_Mode  = GPIO_Mode_AF3;  // RxD (DIO18)
			HAL.IOs->pins->DIO18.configuration.GPIO_OType = GPIO_OType_OD;  // RxD as open drain output
			HAL.IOs->pins->DIO17.configuration.GPIO_PuPd  = GPIO_PuPd_UP;   // TxD with pull-up resistor
			HAL.IOs->config->set(&HAL.IOs->pins->DIO17);
			HAL.IOs->config->set(&HAL.IOs->pins->DIO18);
			// Enable single wire UART
			UART_C1_REG(UART2_BASE_PTR) |= (UART_C1_LOOPS_MASK | UART_C1_RSRC_MASK);
			// Set TxD as output in single wire UART
			UART_C3_REG(UART2_BASE_PTR) |= UART_C3_TXDIR_MASK;
			break;
		case UART_MODE_DUAL_WIRE:
		default:
			HAL.IOs->pins->DIO17.configuration.GPIO_Mode  = GPIO_Mode_AF3;  // TxD (DIO17)
			HAL.IOs->pins->DIO18.configuration.GPIO_Mode  = GPIO_Mode_AF3;  // RxD (DIO18)
			HAL.IOs->pins->DIO17.configuration.GPIO_OType = GPIO_OType_OD;  // TxD as open drain output
			HAL.IOs->pins->DIO18.configuration.GPIO_PuPd  = GPIO_PuPd_UP;   // RxD with pull-up resistor
			HAL.IOs->config->set(&HAL.IOs->pins->DIO17);
			HAL.IOs->config->set(&HAL.IOs->pins->DIO18);
			break;
		}
		UART_C2_REG(UART2_BASE_PTR) &= ~(UART_C2_TE_MASK | UART_C2_RE_MASK );
		UART_BDH_REG(UART2_BASE_PTR) = (ubd >> 8) & UART_BDH_SBR_MASK;
		UART_BDL_REG(UART2_BASE_PTR) = (ubd & UART_BDL_SBR_MASK);
		UART_C2_REG(UART2_BASE_PTR) |= (UART_C2_TE_MASK | UART_C2_RE_MASK | UART_C2_RIE_MASK);
		enable_irq(INT_UART2_RX_TX-16);
		break;
	}

}

static void deInit()
{
	switch(UART_Host.pinout) {
	case UART_PINS_2:
		SIM_SCGC4 &= ~(SIM_SCGC4_UART0_MASK);
		HAL.IOs->pins->DIO10.configuration.GPIO_Mode = GPIO_Mode_IN;
		HAL.IOs->pins->DIO11.configuration.GPIO_Mode = GPIO_Mode_IN;
		HAL.IOs->config->set(&HAL.IOs->pins->DIO10);
		HAL.IOs->config->set(&HAL.IOs->pins->DIO11);
		disable_irq(INT_UART0_RX_TX-16);
		break;
	case UART_PINS_3:
		SIM_SCGC4 &= ~(SIM_SCGC4_UART0_MASK);
		HAL.IOs->pins->WIRELESS_TX.configuration.GPIO_Mode = GPIO_Mode_IN;
		HAL.IOs->pins->WIRELESS_RX.configuration.GPIO_Mode = GPIO_Mode_IN;
		HAL.IOs->config->set(&HAL.IOs->pins->WIRELESS_TX);
		HAL.IOs->config->set(&HAL.IOs->pins->WIRELESS_RX);
		disable_irq(INT_UART0_RX_TX-16);
		break;
	case UART_PINS_1:
	default:
		SIM_SCGC4 &= ~(SIM_SCGC4_UART2_MASK);
		HAL.IOs->pins->DIO17.configuration.GPIO_Mode = GPIO_Mode_IN;
		HAL.IOs->pins->DIO18.configuration.GPIO_Mode = GPIO_Mode_IN;
		HAL.IOs->config->set(&HAL.IOs->pins->DIO17);
		HAL.IOs->config->set(&HAL.IOs->pins->DIO18);
		disable_irq(INT_UART2_RX_TX-16);
		break;
	}

	clearBuffers();
}

void UART0_RX_TX_IRQHandler_Host(void)
{
	static uint8_t isSending = false;
	uint32_t status = UART0_S1;

	// Receive interrupt
	if(status & UART_S1_RDRF_MASK)
	{
		// One-wire UART communication:
		buffers.rx.buffer[buffers.rx.wrote] = UART0_D;
		if(!isSending) // Only move ring buffer index & available counter when the received byte wasn't the send echo
		{
			buffers.rx.wrote = (buffers.rx.wrote + 1) % BUFFER_SIZE;
			available++;
		}
	}

	// Transmission complete interrupt => do not ignore echo any more
	// after last bit has been sent.
	if(status & UART_S1_TC_MASK)
	{
		// Last bit has been sent
		isSending = false;
		UART0_C2 &= ~UART_C2_TCIE_MASK;
	}

	// Transmit buffer empty interrupt => send next byte if there is something
	// to be sent.
	if(status & UART_S1_TDRE_MASK)
	{
		if(buffers.tx.read != buffers.tx.wrote)
		{
			UART0_D = buffers.tx.buffer[buffers.tx.read];
			buffers.tx.read = (buffers.tx.read + 1) % BUFFER_SIZE;

			isSending = true; // Ignore echo
			UART0_C2 |= UART_C2_TCIE_MASK; // Turn on transmission complete interrupt
		}
		else
		{
			UART0_C2 &= ~UART_C2_TIE_MASK; // empty buffer -> turn off transmit buffer empty interrupt
		}
	}
}

void UART2_RX_TX_IRQHandler_Host(void)
{
	static uint8_t isSending = false;
	uint32_t status = UART2_S1;

	// Receive interrupt
	if(status & UART_S1_RDRF_MASK)
	{
		// One-wire UART communication:
		buffers.rx.buffer[buffers.rx.wrote] = UART2_D;
		if(!isSending) // Only move ring buffer index & available counter when the received byte wasn't the send echo
		{
			buffers.rx.wrote = (buffers.rx.wrote + 1) % BUFFER_SIZE;
			available++;
		}
	}

	// Transmission complete interrupt => do not ignore echo any more
	// after last bit has been sent.
	if(status & UART_S1_TC_MASK)
	{
		// Last bit has been sent
		isSending = false;
		UART2_C2 &= ~UART_C2_TCIE_MASK;
	}

	// Transmit buffer empty interrupt => send next byte if there is something
	// to be sent.
	if(status & UART_S1_TDRE_MASK)
	{
		if(buffers.tx.read != buffers.tx.wrote)
		{
			UART2_D = buffers.tx.buffer[buffers.tx.read];
			buffers.tx.read = (buffers.tx.read + 1) % BUFFER_SIZE;

			isSending = true; // Ignore echo
			UART2_C2 |= UART_C2_TCIE_MASK; // Turn on transmission complete interrupt
		}
		else
		{
			UART2_C2 &= ~UART_C2_TIE_MASK; // empty buffer -> turn off transmit buffer empty interrupt
		}
	}
}

static void tx(uint8_t ch)
{
	buffers.tx.buffer[buffers.tx.wrote] = ch;
	buffers.tx.wrote = (buffers.tx.wrote + 1) % BUFFER_SIZE;

	// enable send interrupt
	switch(UART_Host.pinout) {
	case UART_PINS_2:
	case UART_PINS_3:
		UART0_C2 |= UART_C2_TIE_MASK;
		break;
	case UART_PINS_1:
	default:
		UART2_C2 |= UART_C2_TIE_MASK;
		break;
	}
}

static uint8_t rx(uint8_t *ch)
{
	if(buffers.rx.read == buffers.rx.wrote)
		return 0;

	*ch = buffers.rx.buffer[buffers.rx.read];
	buffers.rx.read = (buffers.rx.read + 1) % BUFFER_SIZE;
	available--;

	return 1;
}

static void txN(uint8_t *str, uint8_t number)
{
	for(int32_t i = 0; i < number; i++) {
		tx(str[i]);
	}
}

static uint8_t rxN(uint8_t *str, uint8_t number)
{
	if(available < number)
		return 0;

	for(int32_t i = 0; i < number; i++)
		rx(&str[i]);

	return 1;
}

static void clearBuffers(void)
{
	switch(UART_Host.pinout) {
	case UART_PINS_2:
	case UART_PINS_3:
		disable_irq(INT_UART0_RX_TX-16);
		available         = 0;
		buffers.rx.read   = 0;
		buffers.rx.wrote  = 0;
		buffers.tx.read   = 0;
		buffers.tx.wrote  = 0;
		enable_irq(INT_UART0_RX_TX-16);
		break;
	case UART_PINS_1:
	default:
		disable_irq(INT_UART2_RX_TX-16);
		available         = 0;
		buffers.rx.read   = 0;
		buffers.rx.wrote  = 0;
		buffers.tx.read   = 0;
		buffers.tx.wrote  = 0;
		enable_irq(INT_UART2_RX_TX-16);
		break;
	}
}

static uint32_t bytesAvailable()
{
	return available;
}

