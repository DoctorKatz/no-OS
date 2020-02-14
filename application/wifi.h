#ifndef WIFI_H_
#define WIFI_H_

struct wifi_init_param {
	struct uart_desc	*uart_desc;
	struct irq_ctrl_desc	*irq_desc;
	uint32_t		uart_irq_id;
	void 			*uart_irq_conf;
};

typedef void* wifi_desc;

uint32_t wifi_init(wifi_desc *desc, struct wifi_init_param *param);
uint32_t wifi_read(wifi_desc desc, uint8_t *buff, uint32_t len);
uint32_t wifi_write(wifi_desc desc, uint8_t *buff, uint32_t len);

#endif /* WIFI_H_ */
