/*
 * Copyright (c) 2015-2016, 2018, 2020 The Linux Foundation. All rights reserved.
 *
 * Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved.
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

#ifndef _DEVSOC_CLK_H
#define _DEVSOC_CLK_H

#include <asm/arch-qca-common/uart.h>

/*
 * UART registers
 */
#define GCC_BLSP1_UART1_BCR			0x1802028
#define GCC_BLSP1_UART2_BCR			0x1803028
#define GCC_BLSP1_UART3_BCR			0x1804028

#define GCC_BLSP1_UART_BCR(id)	((id < 1) ? \
				(GCC_BLSP1_UART1_BCR):\
				(GCC_BLSP1_UART1_BCR + (0x1000 * id)))

#define GCC_BLSP1_UART_APPS_CMD_RCGR(id)	(GCC_BLSP1_UART_BCR(id) + 0x04)
#define GCC_BLSP1_UART_APPS_CFG_RCGR(id)	(GCC_BLSP1_UART_BCR(id) + 0x08)
#define GCC_BLSP1_UART_APPS_M(id)		(GCC_BLSP1_UART_BCR(id) + 0x0c)
#define GCC_BLSP1_UART_APPS_N(id)		(GCC_BLSP1_UART_BCR(id) + 0x10)
#define GCC_BLSP1_UART_APPS_D(id)		(GCC_BLSP1_UART_BCR(id) + 0x14)
#define GCC_BLSP1_UART_APPS_CBCR(id)		(GCC_BLSP1_UART_BCR(id) + 0x18)

#define GCC_UART_CFG_RCGR_MODE_MASK		0x3000
#define GCC_UART_CFG_RCGR_SRCSEL_MASK		0x0700
#define GCC_UART_CFG_RCGR_SRCDIV_MASK		0x001F

#define GCC_UART_CFG_RCGR_MODE_SHIFT		12
#define GCC_UART_CFG_RCGR_SRCSEL_SHIFT		8
#define GCC_UART_CFG_RCGR_SRCDIV_SHIFT		0

#define UART_RCGR_SRC_SEL			0x1
#define UART_RCGR_SRC_DIV			0x0
#define UART_RCGR_MODE				0x2
#define UART_CMD_RCGR_UPDATE			0x1
#define UART_CMD_RCGR_ROOT_EN			0x2
#define UART_CBCR_CLK_ENABLE			0x1

#define NOT_2D(two_d)				(~two_d)
#define NOT_N_MINUS_M(n,m)			(~(n - m))
#define CLOCK_UPDATE_TIMEOUT_US			1000

#define CMD_UPDATE      			0x1
#define ROOT_EN         			0x2
#define CLK_ENABLE      			0x1

/*
 * Qpic SPI Nand clock
 */

#define GCC_QPIC_IO_MACRO_CMD_RCGR		0x1832004
#define GCC_QPIC_IO_MACRO_CFG_RCGR		0x1832008
#define GCC_QPIC_IO_MACRO_CBCR			0x183200C
#define GCC_QPIC_AHB_CBCR_ADDR			0x1832010
#define GCC_QPIC_CBCR_ADDR			0x1832014
#define GCC_QPIC_SREGR				0x1832018
#define GCC_QPIC_SLEEP_CBCR			0x183201C

#define IO_MACRO_CLK_320_MHZ			320000000
#define IO_MACRO_CLK_266_MHZ			266000000
#define IO_MACRO_CLK_228_MHZ			228000000
#define IO_MACRO_CLK_200_MHZ			200000000
#define IO_MACRO_CLK_100_MHZ			100000000
#define IO_MACRO_CLK_24MHZ			24000000

#define QPIC_IO_MACRO_CLK       		0
#define QPIC_CORE_CLK           		1
#define XO_CLK_SRC				2
#define GPLL0_CLK_SRC				3
#define FB_CLK_BIT				(1 << 4)
#define UPDATE_EN				0x1

/*
 * GCC-SDCC Registers
 */

#define GCC_SDCC1_BCR				0x01833000
#define GCC_SDCC1_APPS_CMD_RCGR			0x01833004
#define GCC_SDCC1_APPS_CFG_RCGR			0x01833008
#define GCC_SDCC1_APPS_M			0x0183300C
#define GCC_SDCC1_APPS_N			0x01833010
#define GCC_SDCC1_APPS_D			0x01833014
#define GCC_SDCC1_APPS_CBCR			0x01833034
#define GCC_SDCC1_AHB_CBCR			0x0183301C

#ifdef CONFIG_QCA_MMC
void emmc_clock_init(void);
void emmc_clock_reset(void);
#endif
#ifdef CONFIG_PCI_IPQ
void pcie_v2_clock_init(int pcie_id);
void pcie_v2_clock_deinit(int pcie_id);
#endif
int uart_clock_config(struct ipq_serial_platdata *plat);
#ifdef CONFIG_USB_XHCI_IPQ
void usb_clock_init(int id, int ssphy);
void usb_clock_deinit(void);
#endif

#endif /*IPQ9574_CLK_H*/
