/*
* Copyright (c) 2016-2019, The Linux Foundation. All rights reserved.
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 and
* only version 2 as published by the Free Software Foundation.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*/

#include <common.h>
#include <asm/global_data.h>
#include <asm/io.h>
#include <asm/errno.h>
#include <environment.h>

#include <asm/arch-qca-common/qpic_nand.h>
#include <asm/arch-qca-common/gpio.h>
#include <asm/arch-qca-common/uart.h>
#include <ipq5018.h>

DECLARE_GLOBAL_DATA_PTR;
extern int ipq_spi_init(u16);

void uart1_configure_mux(void)
{
	unsigned long cfg_rcgr;

	cfg_rcgr = readl(GCC_BLSP1_UART1_APPS_CFG_RCGR);
	/* Clear mode, src sel, src div */
	cfg_rcgr &= ~(GCC_UART_CFG_RCGR_MODE_MASK |
			GCC_UART_CFG_RCGR_SRCSEL_MASK |
			GCC_UART_CFG_RCGR_SRCDIV_MASK);

	cfg_rcgr |= ((UART1_RCGR_SRC_SEL << GCC_UART_CFG_RCGR_SRCSEL_SHIFT)
			& GCC_UART_CFG_RCGR_SRCSEL_MASK);

	cfg_rcgr |= ((UART1_RCGR_SRC_DIV << GCC_UART_CFG_RCGR_SRCDIV_SHIFT)
			& GCC_UART_CFG_RCGR_SRCDIV_MASK);

	cfg_rcgr |= ((UART1_RCGR_MODE << GCC_UART_CFG_RCGR_MODE_SHIFT)
			& GCC_UART_CFG_RCGR_MODE_MASK);

	writel(cfg_rcgr, GCC_BLSP1_UART1_APPS_CFG_RCGR);
}

void uart1_set_rate_mnd(unsigned int m,
		unsigned int n, unsigned int two_d)
{
	writel(m, GCC_BLSP1_UART1_APPS_M);
	writel(NOT_N_MINUS_M(n, m), GCC_BLSP1_UART1_APPS_N);
	writel(NOT_2D(two_d), GCC_BLSP1_UART1_APPS_D);
}

int uart1_trigger_update(void)
{
	unsigned long cmd_rcgr;
	int timeout = 0;

	cmd_rcgr = readl(GCC_BLSP1_UART1_APPS_CMD_RCGR);
	cmd_rcgr |= UART1_CMD_RCGR_UPDATE;
	writel(cmd_rcgr, GCC_BLSP1_UART1_APPS_CMD_RCGR);

	while (readl(GCC_BLSP1_UART1_APPS_CMD_RCGR) & UART1_CMD_RCGR_UPDATE) {
		if (timeout++ >= CLOCK_UPDATE_TIMEOUT_US) {
			printf("Timeout waiting for UART1 clock update\n");
			return -ETIMEDOUT;
			udelay(1);
		}
	}
	cmd_rcgr = readl(GCC_BLSP1_UART1_APPS_CMD_RCGR);
	return 0;
}

void reset_board(void)
{
	run_command("reset", 0);
}

void uart1_toggle_clock(void)
{
	unsigned long cbcr_val;

	cbcr_val = readl(GCC_BLSP1_UART1_APPS_CBCR);
	cbcr_val |= UART1_CBCR_CLK_ENABLE;
	writel(cbcr_val, GCC_BLSP1_UART1_APPS_CBCR);
}

void uart1_clock_config(unsigned int m,
		unsigned int n, unsigned int two_d)
{
	uart1_configure_mux();
	uart1_set_rate_mnd(m, n, two_d);
	uart1_trigger_update();
	uart1_toggle_clock();
}

void qca_serial_init(struct ipq_serial_platdata *plat)
{
	int node, uart1_node;

	writel(1, GCC_BLSP1_UART1_APPS_CBCR);
	node = fdt_path_offset(gd->fdt_blob, "/serial@78AF000/serial_gpio");
	if (node < 0) {
		printf("Could not find serial_gpio node\n");
		return;
	}

	if (plat->port_id == 1) {
		uart1_node = fdt_path_offset(gd->fdt_blob, "uart1");
		if (uart1_node < 0) {
			printf("Could not find uart1 node\n");
			return;
		}
	node = fdt_subnode_offset(gd->fdt_blob,
				uart1_node, "serial_gpio");
	uart1_clock_config(plat->m_value, plat->n_value, plat->d_value);
	writel(1, GCC_BLSP1_UART1_APPS_CBCR);
	}

	qca_gpio_init(node);
}

void reset_crashdump(void)
{
	return;
}

void board_nand_init(void)
{
#ifdef CONFIG_QCA_SPI
	int gpio_node;
	gpio_node = fdt_path_offset(gd->fdt_blob, "/spi/spi_gpio");
	if (gpio_node >= 0) {
		qca_gpio_init(gpio_node);
		ipq_spi_init(CONFIG_IPQ_SPI_NOR_INFO_IDX);
	}
#endif
}

void enable_caches(void)
{
	icache_enable();
}

void disable_caches(void)
{
	icache_disable();
}
/**
 * * Set the uuid in bootargs variable for mounting rootfilesystem
 */
int set_uuid_bootargs(char *boot_args, char *part_name, int buflen, bool gpt_flag)
{
	return 0;
}

unsigned long timer_read_counter(void)
{
	return 0;
}

void reset_cpu(unsigned long a)
{
	while(1);
}