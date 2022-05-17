/*
 * Copyright (c) 2022, The Linux Foundation. All rights reserved.

 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
*/
#include "ipq_phy.h"
#include "ipq_qca8081.h"
#include "ipq_qca8084.h"
#include "ipq_qca8084_clk.h"
#include <linux/compat.h>
#include <malloc.h>

#ifdef DEBUG
#define pr_debug(fmt, args...) printf(fmt, ##args);
#else
#define pr_debug(fmt, args...)
#endif

extern int ipq_mdio_read(int mii_id,
		int regnum, ushort *data);
extern int ipq_mdio_write(int mii_id,
		int regnum, u16 data);
extern void qca8084_interface_uqxgmii_mode_set(void);
extern void qca8084_gcc_clock_init(void);
extern u8 qca8081_phy_get_link_status(u32 dev_id, u32 phy_id);
extern u32 qca8081_phy_get_duplex(u32 dev_id, u32 phy_id, fal_port_duplex_t *duplex);
extern u32 qca8081_phy_get_speed(u32 dev_id, u32 phy_id, fal_port_speed_t *speed);
extern void qca8084_uniphy_xpcs_autoneg_restart(uint32_t qca8084_port_id);
extern void qca8084_port_speed_clock_set(uint32_t qca8084_port_id,
						fal_port_speed_t speed);
extern void qca8084_uniphy_xpcs_speed_set(uint32_t qca8084_port_id,
						fal_port_speed_t speed);
extern void qca8084_port_clk_en_set(uint32_t qca8084_port_id, uint8_t mask,
						uint8_t enable);
extern void qca8084_port_clk_reset(uint32_t qca8084_port_id, uint8_t mask);
extern void qca8084_uniphy_uqxgmii_function_reset(uint32_t qca8084_port_id);

u16 qca8084_phy_reg_read(u32 phy_addr, u32 reg_id)
{
	return ipq_mdio_read(phy_addr, reg_id, NULL);
}

u16 qca8084_phy_reg_write(u32 phy_addr, u32 reg_id, u16 value)
{
	return ipq_mdio_write(phy_addr, reg_id, value);
}

u16 qca8084_phy_mmd_read(u32 phy_addr, u16 mmd_num, u16 reg_id)
{
	uint32_t reg_id_c45 = QCA8084_REG_C45_ADDRESS(mmd_num, reg_id);

	return ipq_mdio_read(phy_addr, reg_id_c45, NULL);
}

u16 qca8084_phy_mmd_write(u32 phy_addr, u16 mmd_num, u16 reg_id, u16 value)
{
	uint32_t reg_id_c45 = QCA8084_REG_C45_ADDRESS(mmd_num, reg_id);

	return ipq_mdio_write(phy_addr, reg_id_c45, value);
}

void qca8084_phy_modify_mii(uint32_t phy_addr, uint32_t mii_reg, uint32_t mask,
			    uint32_t value)
{
	uint16_t phy_data = 0, new_phy_data = 0;

	phy_data = qca8084_phy_reg_read(phy_addr, mii_reg);
	new_phy_data = (phy_data & ~mask) | value;
	qca8084_phy_reg_write(phy_addr, mii_reg, new_phy_data);
	/*check the mii register value*/
	phy_data = qca8084_phy_reg_read(phy_addr, mii_reg);
	pr_debug("phy_addr:0x%x, mii_reg:0x%x, phy_data:0x%x\n",
		phy_addr, mii_reg, phy_data);
}

void qca8084_phy_modify_mmd(uint32_t phy_addr, uint32_t mmd_num,
			    uint32_t mmd_reg, uint32_t mask, uint32_t value)
{
	uint16_t phy_data = 0, new_phy_data = 0;

	phy_data = qca8084_phy_mmd_read(phy_addr, mmd_num, mmd_reg);
	new_phy_data = (phy_data & ~mask) | value;
	qca8084_phy_mmd_write(phy_addr, mmd_num, mmd_reg, new_phy_data);
	/* check the mmd register value */
	phy_data = qca8084_phy_mmd_read(phy_addr, mmd_num, mmd_reg);
	pr_debug("phy_addr:0x%x, mmd_reg:0x%x, phy_data:0x%x\n",
		phy_addr, mmd_reg, phy_data);
}

void qca8084_phy_ipg_config(uint32_t phy_id, fal_port_speed_t speed)
{
	uint16_t phy_data = 0;

	phy_data = qca8084_phy_mmd_read(phy_id, QCA8084_PHY_MMD7_NUM,
					QCA8084_PHY_MMD7_IPG_10_11_ENABLE);

	phy_data &= ~QCA8084_PHY_MMD7_IPG_11_EN;

	/*If speed is 1G, enable 11 ipg tuning*/
	pr_debug("if speed is 1G, enable 11 ipg tuning\n");
	if (speed == FAL_SPEED_1000)
		phy_data |= QCA8084_PHY_MMD7_IPG_11_EN;

	qca8084_phy_mmd_write(phy_id, QCA8084_PHY_MMD7_NUM,
				QCA8084_PHY_MMD7_IPG_10_11_ENABLE, phy_data);
}

void qca8084_phy_function_reset(uint32_t phy_id)
{
	uint16_t phy_data = 0;

	phy_data = qca8084_phy_reg_read(phy_id, QCA8084_PHY_FIFO_CONTROL);

	qca8084_phy_reg_write(phy_id, QCA8084_PHY_FIFO_CONTROL,
				phy_data & (~QCA8084_PHY_FIFO_RESET));

	mdelay(50);

	qca8084_phy_reg_write(phy_id, QCA8084_PHY_FIFO_CONTROL,
				phy_data | QCA8084_PHY_FIFO_RESET);
}

void qca8084_phy_uqxgmii_speed_fixup(uint32_t phy_addr, uint32_t qca8084_port_id,
				     uint32_t status, fal_port_speed_t new_speed)
{
	uint32_t port_clock_en = 0;

	/*Restart the auto-neg of uniphy*/
	pr_debug("Restart the auto-neg of uniphy\n");
	qca8084_uniphy_xpcs_autoneg_restart(qca8084_port_id);

	/*set gmii+ clock to uniphy1 and ethphy*/
	pr_debug("set gmii,xgmii clock to uniphy and gmii to ethphy\n");
	qca8084_port_speed_clock_set(qca8084_port_id, new_speed);

	/*set xpcs speed*/
	pr_debug("set xpcs speed\n");
	qca8084_uniphy_xpcs_speed_set(qca8084_port_id, new_speed);

	/*GMII/XGMII clock and ETHPHY GMII clock enable/disable*/
	pr_debug("GMII/XGMII clock and ETHPHY GMII clock enable/disable\n");
	if (status == 0)
		port_clock_en = 1;
	qca8084_port_clk_en_set(qca8084_port_id,
				QCA8084_CLK_TYPE_UNIPHY | QCA8084_CLK_TYPE_EPHY,
				port_clock_en);

	pr_debug("UNIPHY GMII/XGMII interface and ETHPHY GMII interface reset and release\n");
	qca8084_port_clk_reset(qca8084_port_id,
			       QCA8084_CLK_TYPE_UNIPHY | QCA8084_CLK_TYPE_EPHY);

	pr_debug("ipg_tune and xgmii2gmii reset for uniphy and ETHPHY, function reset\n");
	qca8084_uniphy_uqxgmii_function_reset(qca8084_port_id);

	/*do ethphy function reset: PHY_FIFO_RESET*/
	pr_debug("do ethphy function reset\n");
	qca8084_phy_function_reset(phy_addr);

	/*change IPG from 10 to 11 for 1G speed*/
	qca8084_phy_ipg_config(phy_addr, new_speed);
}

void qca8084_phy_interface_mode_set(void)
{
	pr_debug("Configure QCA8084 as PORT_UQXGMII..\n");
	/*the work mode is PORT_UQXGMII in default*/
	qca8084_interface_uqxgmii_mode_set();

	/*init clock for PORT_UQXGMII*/
	qca8084_gcc_clock_init();

	/*init pinctrl for phy mode to be added later*/
}

void qca8084_cdt_thresh_init(u32 phy_id)
{
	qca8084_phy_mmd_write(phy_id, QCA8084_PHY_MMD3_NUM,
				QCA8084_PHY_MMD3_CDT_THRESH_CTRL3,
				QCA8084_PHY_MMD3_CDT_THRESH_CTRL3_VAL);
	qca8084_phy_mmd_write(phy_id, QCA8084_PHY_MMD3_NUM,
				QCA8084_PHY_MMD3_CDT_THRESH_CTRL4,
				QCA8084_PHY_MMD3_CDT_THRESH_CTRL4_VAL);
	qca8084_phy_mmd_write(phy_id, QCA8084_PHY_MMD3_NUM,
				QCA8084_PHY_MMD3_CDT_THRESH_CTRL5,
				QCA8084_PHY_MMD3_CDT_THRESH_CTRL5_VAL);
	qca8084_phy_mmd_write(phy_id, QCA8084_PHY_MMD3_NUM,
				QCA8084_PHY_MMD3_CDT_THRESH_CTRL6,
				QCA8084_PHY_MMD3_CDT_THRESH_CTRL6_VAL);
	qca8084_phy_mmd_write(phy_id, QCA8084_PHY_MMD3_NUM,
				QCA8084_PHY_MMD3_CDT_THRESH_CTRL7,
				QCA8084_PHY_MMD3_CDT_THRESH_CTRL7_VAL);
	qca8084_phy_mmd_write(phy_id, QCA8084_PHY_MMD3_NUM,
				QCA8084_PHY_MMD3_CDT_THRESH_CTRL9,
				QCA8084_PHY_MMD3_CDT_THRESH_CTRL9_VAL);
	qca8084_phy_mmd_write(phy_id, QCA8084_PHY_MMD3_NUM,
				QCA8084_PHY_MMD3_CDT_THRESH_CTRL13,
				QCA8084_PHY_MMD3_CDT_THRESH_CTRL13_VAL);
	qca8084_phy_mmd_write(phy_id, QCA8084_PHY_MMD3_NUM,
				QCA8084_PHY_MMD3_CDT_THRESH_CTRL14,
				QCA8084_PHY_MMD3_NEAR_ECHO_THRESH_VAL);
}

void qca8084_phy_modify_debug(u32 phy_addr, u32 debug_reg,
				u32 mask, u32 value)
{
	u16 phy_data = 0, new_phy_data = 0;

	qca8084_phy_reg_write(phy_addr, QCA8084_DEBUG_PORT_ADDRESS, debug_reg);
	phy_data = qca8084_phy_reg_read(phy_addr, QCA8084_DEBUG_PORT_DATA);
	if (phy_data == PHY_INVALID_DATA)
		pr_debug("qca8084_phy_reg_read failed\n");

	new_phy_data = (phy_data & ~mask) | value;
	qca8084_phy_reg_write(phy_addr, QCA8084_DEBUG_PORT_ADDRESS, debug_reg);
	qca8084_phy_reg_write(phy_addr, QCA8084_DEBUG_PORT_DATA, new_phy_data);

	/* check debug register value */
	qca8084_phy_reg_write(phy_addr, QCA8084_DEBUG_PORT_ADDRESS, debug_reg);
	phy_data = qca8084_phy_reg_read(phy_addr, QCA8084_DEBUG_PORT_DATA);
	pr_debug("phy_addr:0x%x, debug_reg:0x%x, phy_data:0x%x\n",
		phy_addr, debug_reg, phy_data);
}

void qca8084_phy_reset(u32 phy_addr)
{
	u16 phy_data;

	phy_data = qca8084_phy_reg_read(phy_addr, QCA8084_PHY_CONTROL);
	qca8084_phy_reg_write(phy_addr, QCA8084_PHY_CONTROL,
				 phy_data | QCA8084_CTRL_SOFTWARE_RESET);
}

void qca8084_phy_adc_edge_set(u32 phy_addr, u32 adc_edge)
{
	qca8084_phy_modify_debug(phy_addr,
		QCA8084_PHY_DEBUG_ANA_INTERFACE_CLK_SEL, 0xf0, adc_edge);
	qca8084_phy_reset(phy_addr);
}

void ipq_qca8084_phy_hw_init(struct phy_ops **ops, u32 phy_addr)
{
#ifdef DEBUG
	u16 phy_data;
#endif
	struct phy_ops *qca8084_ops;

	qca8084_ops = (struct phy_ops *)malloc(sizeof(struct phy_ops));
	if (!qca8084_ops) {
		pr_debug("Error allocating memory for phy ops\n");
		return;
	}

	/* Note that qca8084 PHY is based on qca8081 PHY and so the following
	 * ops functions required would be re-used from qca8081 */

	qca8084_ops->phy_get_link_status = qca8081_phy_get_link_status;
	qca8084_ops->phy_get_speed = qca8081_phy_get_speed;
	qca8084_ops->phy_get_duplex = qca8081_phy_get_duplex;
	*ops = qca8084_ops;

#ifdef DEBUG
	phy_data = qca8084_phy_reg_read(phy_addr, QCA8081_PHY_ID1);
	printf("PHY ID1: 0x%x\n", phy_data);
	phy_data = qca8084_phy_reg_read(phy_addr, QCA8081_PHY_ID2);
	printf("PHY ID2: 0x%x\n", phy_data);
#endif

	/* adjust CDT threshold */
	qca8084_cdt_thresh_init(phy_addr);

	/* invert ADC clock edge as falling edge to fix link issue */
	qca8084_phy_adc_edge_set(phy_addr, ADC_FALLING);
}
