/*
 * Copyright (c) 2016-2019 The Linux Foundation. All rights reserved.
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

#include <common.h>
#include <asm/arch-devsoc/clk.h>
#include <asm/io.h>
#include <asm/errno.h>

static void uart_configure_mux(u8 id)
{
	unsigned long cfg_rcgr;

	cfg_rcgr = readl(GCC_BLSP1_UART_APPS_CFG_RCGR(id));
	/* Clear mode, src sel, src div */
	cfg_rcgr &= ~(GCC_UART_CFG_RCGR_MODE_MASK |
			GCC_UART_CFG_RCGR_SRCSEL_MASK |
			GCC_UART_CFG_RCGR_SRCDIV_MASK);

	cfg_rcgr |= ((UART_RCGR_SRC_SEL << GCC_UART_CFG_RCGR_SRCSEL_SHIFT)
			& GCC_UART_CFG_RCGR_SRCSEL_MASK);

	cfg_rcgr |= ((UART_RCGR_SRC_DIV << GCC_UART_CFG_RCGR_SRCDIV_SHIFT)
			& GCC_UART_CFG_RCGR_SRCDIV_MASK);

	cfg_rcgr |= ((UART_RCGR_MODE << GCC_UART_CFG_RCGR_MODE_SHIFT)
			& GCC_UART_CFG_RCGR_MODE_MASK);

	writel(cfg_rcgr, GCC_BLSP1_UART_APPS_CFG_RCGR(id));
}

static int uart_trigger_update(u8 id)
{
	unsigned long cmd_rcgr;
	int timeout = 0;

	cmd_rcgr = readl(GCC_BLSP1_UART_APPS_CMD_RCGR(id));
	cmd_rcgr |= UART_CMD_RCGR_UPDATE | UART_CMD_RCGR_ROOT_EN;
	writel(cmd_rcgr, GCC_BLSP1_UART_APPS_CMD_RCGR(id));

	while (readl(GCC_BLSP1_UART_APPS_CMD_RCGR(id)) & UART_CMD_RCGR_UPDATE) {
		if (timeout++ >= CLOCK_UPDATE_TIMEOUT_US) {
			printf("Timeout waiting for UART clock update\n");
			return -ETIMEDOUT;
		}
		udelay(1);
	}
	return 0;
}

int uart_clock_config(struct ipq_serial_platdata *plat)
{
	unsigned long cbcr_val;
	int ret;

	uart_configure_mux(plat->port_id);

	writel(plat->m_value, GCC_BLSP1_UART_APPS_M(plat->port_id));
	writel(NOT_N_MINUS_M(plat->n_value, plat->m_value),
				GCC_BLSP1_UART_APPS_N(plat->port_id));
	writel(NOT_2D(plat->d_value), GCC_BLSP1_UART_APPS_D(plat->port_id));

	ret = uart_trigger_update(plat->port_id);
	if (ret)
		return ret;

	cbcr_val = readl(GCC_BLSP1_UART_APPS_CBCR(plat->port_id));
	cbcr_val |= UART_CBCR_CLK_ENABLE;
	writel(cbcr_val, GCC_BLSP1_UART_APPS_CBCR(plat->port_id));
	return 0;
}

#ifdef CONFIG_QPIC_NAND
void qpic_set_clk_rate(unsigned int clk_rate, int blk_type,
				int req_clk_src_type)
{
	return;
}
#endif

#ifdef CONFIG_QCA_MMC
void emmc_clock_init(void)
{
#ifdef QCA_CLOCK_ENABLE
	/* Enable root clock generator */
	writel(readl(GCC_SDCC1_APPS_CBCR)|0x1, GCC_SDCC1_APPS_CBCR);
	/* Add 10us delay for CLK_OFF to get cleared */
	udelay(10);
	writel(readl(GCC_SDCC1_AHB_CBCR)|0x1, GCC_SDCC1_AHB_CBCR);
	/* PLL0 - 192Mhz */
	writel(0x20B, GCC_SDCC1_APPS_CFG_RCGR);
	/* Delay for clock operation complete */
	udelay(10);
	writel(0x1, GCC_SDCC1_APPS_M);
	/* check this M, N D value while debugging
	 * because as per clock tool the actual M, N, D
	 * values are M=1, N=FA, D=F9
	 */
	writel(0xFC, GCC_SDCC1_APPS_N);
	writel(0xFD, GCC_SDCC1_APPS_D);
	/* Delay for clock operation complete */
	udelay(10);
	/* Update APPS_CMD_RCGR to reflect source selection */
	writel(readl(GCC_SDCC1_APPS_CMD_RCGR)|0x1, GCC_SDCC1_APPS_CMD_RCGR);
	/* Add 10us delay for clock update to complete */
	udelay(10);
#else
	return;
#endif
}

void emmc_clock_reset(void)
{
	writel(0x1, GCC_SDCC1_BCR);
	udelay(10);
	writel(0x0, GCC_SDCC1_BCR);
}
#endif
#ifdef CONFIG_PCI_IPQ
void pcie_v2_clock_init(int pcie_id)
{
#ifdef QCA_CLOCK_ENABLE
	int cfg, div;

	/* Configure pcie_aux_clk_src */
	cfg = (GCC_PCIE_AUX_CFG_RCGR_MN_MODE |
		GCC_PCIE_AUX_CFG_RCGR_SRC_SEL |
		GCC_PCIE_AUX_CFG_RCGR_SRC_DIV);
	writel(cfg, GCC_PCIE_REG(GCC_PCIE_AUX_CFG_RCGR, 0));
	writel(0x1, GCC_PCIE_REG(GCC_PCIE_AUX_M, 0));
	writel(0xFFFC, GCC_PCIE_REG(GCC_PCIE_AUX_N, 0));
	writel(0xFFFB, GCC_PCIE_REG(GCC_PCIE_AUX_D, 0));
	writel(CMD_UPDATE, GCC_PCIE_REG(GCC_PCIE_AUX_CMD_RCGR, 0));
	mdelay(100);
	writel(ROOT_EN, GCC_PCIE_REG(GCC_PCIE_AUX_CMD_RCGR, 0));

	/* Configure pcie_axi_m__clk_src */
	if ((pcie_id == 2) || (pcie_id == 3))
		div = GCC_PCIE_AXI_M_CFG_RCGR_SRC_DIV_LANE2;
	else
		div = GCC_PCIE_AXI_M_CFG_RCGR_SRC_DIV_LANE1;

	cfg = (GCC_PCIE_AXI_M_CFG_RCGR_SRC_SEL | div);
	writel(cfg, GCC_PCIE_REG(GCC_PCIE_AXI_M_CFG_RCGR, pcie_id));
	writel(CMD_UPDATE, GCC_PCIE_REG(GCC_PCIE_AXI_M_CMD_RCGR, pcie_id));
	mdelay(100);
	writel(ROOT_EN, GCC_PCIE_REG(GCC_PCIE_AXI_M_CMD_RCGR, pcie_id));

	/* Configure pcie_axi_s__clk_src */
	cfg = (GCC_PCIE_AXI_S_CFG_RCGR_SRC_SEL | GCC_PCIE_AXI_S_CFG_RCGR_SRC_DIV);
	writel(cfg, GCC_PCIE_REG(GCC_PCIE_AXI_S_CFG_RCGR, pcie_id));
	writel(CMD_UPDATE, GCC_PCIE_REG(GCC_PCIE_AXI_S_CMD_RCGR, pcie_id));
	mdelay(100);
	writel(ROOT_EN, GCC_PCIE_REG(GCC_PCIE_AXI_S_CMD_RCGR, pcie_id));

	/* Configure CBCRs */
	writel(CLK_ENABLE, GCC_PCIE_REG(GCC_PCIE_AHB_CBCR, pcie_id));
	writel(CLK_ENABLE, GCC_PCIE_REG(GCC_PCIE_AXI_M_CBCR, pcie_id));
	writel(CLK_ENABLE, GCC_PCIE_REG(GCC_PCIE_AXI_S_CBCR, pcie_id));
	writel(CLK_ENABLE, GCC_SNOC_PCIE0_1LANE_S_CBCR + (0x4 * pcie_id));
	switch(pcie_id){
	case 0:
		writel(CLK_ENABLE, GCC_ANOC_PCIE0_1LANE_M_CBCR);
	break;
	case 1:
		writel(CLK_ENABLE, GCC_ANOC_PCIE1_1LANE_M_CBCR);
	break;
	case 2:
		writel(CLK_ENABLE, GCC_ANOC_PCIE2_2LANE_M_CBCR);
	break;
	case 3:
		writel(CLK_ENABLE, GCC_ANOC_PCIE3_2LANE_M_CBCR);
	break;
	}
	writel(CLK_ENABLE, GCC_PCIE_REG(GCC_PCIE_AXI_S_BRIDGE_CBCR, pcie_id));
	writel(PIPE_CLK_ENABLE, GCC_PCIE_REG(GCC_PCIE_PIPE_CBCR, pcie_id));

	/* Configure pcie_rchng_clk_src */
	cfg = (GCC_PCIE_RCHNG_CFG_RCGR_SRC_SEL
			| GCC_PCIE_RCHNG_CFG_RCGR_SRC_DIV);
	writel(cfg, GCC_PCIE_REG(GCC_PCIE_RCHNG_CFG_RCGR, pcie_id));
	writel(CMD_UPDATE, GCC_PCIE_REG(GCC_PCIE_RCHNG_CMD_RCGR, pcie_id));
	mdelay(100);
	writel(ROOT_EN, GCC_PCIE_REG(GCC_PCIE_RCHNG_CMD_RCGR, pcie_id));

	writel(CLK_ENABLE, GCC_PCIE_REG(GCC_PCIE_AUX_CBCR, pcie_id));
#else
	return;
#endif
}

void pcie_v2_clock_deinit(int pcie_id)
{
#ifdef QCA_CLOCK_ENABLE
	writel(0x0, GCC_PCIE_REG(GCC_PCIE_AUX_CMD_RCGR, 0));
	mdelay(100);
	writel(0x0, GCC_PCIE_REG(GCC_PCIE_AHB_CBCR, pcie_id));
	writel(0x0, GCC_PCIE_REG(GCC_PCIE_AXI_M_CBCR, pcie_id));
	writel(0x0, GCC_PCIE_REG(GCC_PCIE_AXI_S_CBCR, pcie_id));
	writel(0x0, GCC_PCIE_REG(GCC_PCIE_AUX_CBCR, pcie_id));
	writel(0x0, GCC_PCIE_REG(GCC_PCIE_PIPE_CBCR, pcie_id));
	writel(0x0, GCC_PCIE_REG(GCC_PCIE_AXI_S_BRIDGE_CBCR, pcie_id));
	writel(0x0, GCC_PCIE_REG(GCC_PCIE_RCHNG_CFG_RCGR, pcie_id));
	writel(0x0, GCC_PCIE_REG(GCC_PCIE_RCHNG_CMD_RCGR, pcie_id));
	writel(0x0, GCC_SNOC_PCIE0_1LANE_S_CBCR + (0x4 * pcie_id));
	switch(pcie_id){
	case 0:
		writel(0x0, GCC_ANOC_PCIE0_1LANE_M_CBCR);
	break;
	case 1:
		writel(0x0, GCC_ANOC_PCIE1_1LANE_M_CBCR);
	break;
	case 2:
		writel(0x0, GCC_ANOC_PCIE2_2LANE_M_CBCR);
	break;
	case 3:
		writel(0x0, GCC_ANOC_PCIE3_2LANE_M_CBCR);
	break;
	}
#else
	return;
#endif
}
#endif
#ifdef CONFIG_USB_XHCI_IPQ
void usb_clock_init(int id, int ssphy)
{
#ifdef QCA_CLOCK_ENABLE
	int cfg;

	/* select usb phy mux */
	if (ssphy)
		writel(0x1, TCSR_USB_PCIE_SEL);

	/* Configure usb0_master_clk_src */
	cfg = (GCC_USB0_MASTER_CFG_RCGR_SRC_SEL |
		GCC_USB0_MASTER_CFG_RCGR_SRC_DIV);
	writel(cfg, GCC_USB0_MASTER_CFG_RCGR);
	writel(CMD_UPDATE, GCC_USB0_MASTER_CMD_RCGR);
	mdelay(100);
	writel(ROOT_EN, GCC_USB0_MASTER_CMD_RCGR);

	/* Configure usb0_mock_utmi_clk_src */
	cfg = (GCC_USB_MOCK_UTMI_SRC_SEL |
		GCC_USB_MOCK_UTMI_SRC_DIV);
	writel(cfg, GCC_USB0_MOCK_UTMI_CFG_RCGR);
	writel(GCC_USB_MOCK_UTMI_CLK_DIV, GCC_USB0_MOCK_UTMI_CBCR);
	writel(CMD_UPDATE, GCC_USB0_MOCK_UTMI_CMD_RCGR);
	mdelay(100);
	writel(ROOT_EN, GCC_USB0_MOCK_UTMI_CMD_RCGR);

	/* Configure usb0_aux_clk_src */
	cfg = (GCC_USB0_AUX_CFG_SRC_SEL |
		GCC_USB0_AUX_CFG_SRC_DIV);
	writel(cfg, GCC_USB0_AUX_CFG_RCGR);
	writel(CMD_UPDATE, GCC_USB0_AUX_CMD_RCGR);
	mdelay(100);
	writel(ROOT_EN, GCC_USB0_AUX_CMD_RCGR);

	/* Configure usb0_lfps_cmd_rcgr */
	cfg = (GCC_USB0_LFPS_CFG_SRC_SEL |
		GCC_USB0_LFPS_CFG_SRC_DIV);
	writel(cfg, GCC_USB0_LFPS_CFG_RCGR);
	writel(LFPS_M, GCC_USB0_LFPS_M);
	writel(LFPS_N, GCC_USB0_LFPS_N);
	writel(LFPS_D, GCC_USB0_LFPS_D);
	writel(readl(GCC_USB0_LFPS_CFG_RCGR) | GCC_USB0_LFPS_MODE,
			GCC_USB0_LFPS_CFG_RCGR);
	writel(CMD_UPDATE, GCC_USB0_LFPS_CMD_RCGR);
	mdelay(100);
	writel(ROOT_EN, GCC_USB0_LFPS_CMD_RCGR);

	/* Configure CBCRs */
	writel(CLK_DISABLE, GCC_SYS_NOC_USB0_AXI_CBCR);
	writel(CLK_ENABLE, GCC_SYS_NOC_USB0_AXI_CBCR);
	writel((readl(GCC_USB0_MASTER_CBCR) | CLK_ENABLE),
		GCC_USB0_MASTER_CBCR);
	writel(CLK_ENABLE, GCC_USB0_SLEEP_CBCR);
	writel((GCC_USB_MOCK_UTMI_CLK_DIV | CLK_ENABLE),
		GCC_USB0_MOCK_UTMI_CBCR);
	writel(CLK_DISABLE, GCC_USB0_PIPE_CBCR);
	writel(CLK_ENABLE, GCC_USB0_PHY_CFG_AHB_CBCR);
	writel(CLK_ENABLE, GCC_USB0_AUX_CBCR);
	writel(CLK_ENABLE, GCC_USB0_LFPS_CBCR);
#else
	return;
#endif
}

void usb_clock_deinit(void)
{

#ifdef QCA_CLOCK_ENABLE
	/* Disable clocks */
	writel(0x8000, GCC_USB0_PHY_CFG_AHB_CBCR);
	writel(0xcff0, GCC_USB0_MASTER_CBCR);
	writel(0, GCC_USB0_SLEEP_CBCR);
	writel(0, GCC_USB0_MOCK_UTMI_CBCR);
	writel(0, GCC_USB0_AUX_CBCR);
	writel(0, GCC_ANOC_USB_AXI_CBCR);
	writel(0, GCC_SNOC_USB_CBCR);
#else
	return;
#endif
}
#endif
