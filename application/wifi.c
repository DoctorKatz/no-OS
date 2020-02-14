#include "irq.h"
#include "uart.h"
#include "error.h"
#include "wifi.h"
#include <stdlib.h>
#include <string.h>

struct wifi_desc {
	struct uart_desc	*uart_desc;
	uint8_t 		buff[1000];
	struct irq_ctrl_desc	*irq_desc;
	uint32_t		uart_irq_id;
	enum {
		WAITING_FOR_1000,
		WAITING_FOR_1
	}			status;
};

uint32_t ready;

static void wifi_callback(void *context, void *extra)
{
	(void)extra; //Used for platform specific application only

	struct wifi_desc *ldesc = context;

	if (ldesc->status == WAITING_FOR_1) {
		if (*ldesc->buff == ':') {
			//This is a non blocking function. When the read is done
			// a new int will be generated
			uart_read(ldesc->uart_desc, ldesc->buff, 1000);
			ldesc->status = WAITING_FOR_1000;
			return ;
		}
	}
	else
		ready = 1;

	//This is a non blocking function. When the read is done
	// a new int will be generated
	uart_read(ldesc->uart_desc, ldesc->buff, 1);
	ldesc->status = WAITING_FOR_1;

}

/**
 *
 * @param desc
 * @param param wifi_init_param.uart_irq_conf must be the value needed by the
 * irq_register to generate an interrupt every time a UART character is available
 * for reading
 * @return
 */
uint32_t wifi_init(wifi_desc *desc, struct wifi_init_param *param)
{
	struct wifi_desc *ldesc = calloc(1, sizeof(*ldesc));
	*desc = (void *)ldesc;

	ldesc->irq_desc = param->irq_desc;
	ldesc->uart_desc = param->uart_desc;
	ldesc->uart_irq_id = param->uart_irq_id;


	irq_register(ldesc->irq_desc, param->uart_irq_id, wifi_callback,
			ldesc, param->uart_irq_conf);
	irq_source_enable(ldesc->irq_desc, param->uart_irq_id);


	uart_read(ldesc->uart_desc, ldesc->buff, 1);
	ldesc->status = WAITING_FOR_1;

	return SUCCESS;
}

uint32_t wifi_read(wifi_desc desc, uint8_t *buff, uint32_t len)
{
	struct wifi_desc *ldesc = desc;
	//Example of read
	while (ready == 0);
	memcpy(buff, ldesc->buff, len);

	return SUCCESS;
}

uint32_t wifi_write(wifi_desc desc, uint8_t *buff, uint32_t len)
{
	struct wifi_desc *ldesc = desc;
	uart_write(ldesc->uart_desc, buff, len);
	return SUCCESS;
}
