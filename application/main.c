#include <common.h>
#include <sys/platform.h>
#include <drivers/pwr/adi_pwr.h>
#include "adi_initialize.h"
#include "wifi.h"
#include "uart.h"
#include "uart_extra.h"
#include "delay.h"
#include "irq.h"
#include "irq_extra.h"
#include "error.h"

void initPower()
{
	if (ADI_PWR_SUCCESS != adi_pwr_Init())
		goto error;

	if (ADI_PWR_SUCCESS != adi_pwr_SetClockDivider(ADI_CLOCK_HCLK, 1u))
		goto error;

	if (ADI_PWR_SUCCESS != adi_pwr_SetClockDivider(ADI_CLOCK_PCLK, 1u))
		goto error;

	return;
error:
	printf("ERROR INITIALIZING POWER\n");
	exit(1);
}

int main(int argc, char *argv[])
{
	(void)argc;
	(void)argv;
	/**
	 * Initialize managed drivers and/or services that have been added to
	 * the project.
	 * @return zero on success
	 */
	adi_initComponents();
	initPower();

	/* Initialize uart */
	struct aducm_uart_init_param aducm_param = {
			.parity = UART_NO_PARITY,
			.stop_bits = UART_ONE_STOPBIT,
			.word_length = UART_WORDLEN_8BITS,
			.uart_mode = NONBLOCKING_MODE
	};
	struct uart_init_param uart_param = {0, BD_115200, &aducm_param};
	struct uart_desc *uart_desc;
	uart_init(&uart_desc, &uart_param);

	/* Initialize irq */
	struct irq_init_param	irq_param = {0, NULL};
	struct irq_ctrl_desc	*irq_desc;
	irq_ctrl_init(&irq_desc, &irq_param);

	/* Initialize wifi */

	//Example of parameter needed to configure the irq to be at each byte
	union irq_config_param irq_config;
	irq_config.uart_conf.uart_desc = uart_desc;
	irq_config.uart_conf.event_mask = READ_DONE;
	/* To configure external interrupt */
	//irq_config.external_conf = IRQ_RISING_EDGE;

	struct wifi_init_param wifi_param = {
			.irq_desc = irq_desc,
			.uart_desc = uart_desc,
			.uart_irq_conf = &irq_config,
			.uart_irq_id = UART_INT
	};

	wifi_desc wifi_d;
	wifi_init(&wifi_d, &wifi_param);
	wifi_write(wifi_d, (uint8_t *)"Salut", 5);
	uint8_t buff[1001];
	wifi_read(wifi_d, buff, 1000);
	buff[1000] = 0;
	printf("Received: %s\n", buff);
	return 0;
}

