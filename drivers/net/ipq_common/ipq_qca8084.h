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

#ifndef _QCA8084_PHY_H_
#define _QCA8084_PHY_H_

/*MII register*/
#define QCA8084_PHY_FIFO_CONTROL				0x19

/*MII register field*/
#define QCA8084_PHY_FIFO_RESET					0x3

/*MMD1 register*/
#define QCA8084_PHY_MMD1_NUM					0x1

/*MMD3 register*/
#define QCA8084_PHY_MMD3_NUM					0x3
#define QCA8084_PHY_MMD3_ADDR_8023AZ_EEE_2500M_CAPABILITY	0x15
#define QCA8084_PHY_MMD3_CDT_THRESH_CTRL3			0x8074
#define QCA8084_PHY_MMD3_CDT_THRESH_CTRL4			0x8075
#define QCA8084_PHY_MMD3_CDT_THRESH_CTRL5			0x8076
#define QCA8084_PHY_MMD3_CDT_THRESH_CTRL6			0x8077
#define QCA8084_PHY_MMD3_CDT_THRESH_CTRL7			0x8078
#define QCA8084_PHY_MMD3_CDT_THRESH_CTRL9			0x807a
#define QCA8084_PHY_MMD3_CDT_THRESH_CTRL13			0x807e
#define QCA8084_PHY_MMD3_CDT_THRESH_CTRL14			0x807f

/*MMD3 register field*/
#define QCA8084_PHY_EEE_CAPABILITY_2500M			0x1
#define QCA8084_PHY_MMD3_CDT_THRESH_CTRL3_VAL			0xc040
#define QCA8084_PHY_MMD3_CDT_THRESH_CTRL4_VAL			0xa060
#define QCA8084_PHY_MMD3_CDT_THRESH_CTRL5_VAL			0xc040
#define QCA8084_PHY_MMD3_CDT_THRESH_CTRL6_VAL			0xa060
#define QCA8084_PHY_MMD3_CDT_THRESH_CTRL7_VAL			0xc050
#define QCA8084_PHY_MMD3_CDT_THRESH_CTRL9_VAL			0xc060
#define QCA8084_PHY_MMD3_CDT_THRESH_CTRL13_VAL			0xb060
#define QCA8084_PHY_MMD3_NEAR_ECHO_THRESH_VAL			0x1eb0

/*MMD7 register*/
#define QCA8084_PHY_MMD7_NUM					0x7
#define QCA8084_PHY_MMD7_ADDR_8023AZ_EEE_2500M_CTRL		0x3e
#define QCA8084_PHY_MMD7_ADDR_8023AZ_EEE_2500M_PARTNER		0x3f
#define QCA8084_PHY_MMD7_IPG_10_11_ENABLE			0x901d

/*MMD7 register field*/
#define QCA8084_PHY_8023AZ_EEE_2500BT				0x1
#define QCA8084_PHY_MMD7_IPG_10_EN				0
#define QCA8084_PHY_MMD7_IPG_11_EN				0x1

/*DEBUG port analog register*/
#define QCA8084_PHY_DEBUG_ANA_INTERFACE_CLK_SEL			0x8b80
#define QCA8084_DEBUG_PORT_ADDRESS				29
#define QCA8084_DEBUG_PORT_DATA					30

#define QCA8084_PHY_CONTROL					0
#define QCA8084_CTRL_SOFTWARE_RESET				0x8000

#define PHY_INVALID_DATA			0xffff

#define QCA8084_MII_ADDR_C45			(1<<30)
#define QCA8084_REG_C45_ADDRESS(dev_type, reg_num) (QCA8084_MII_ADDR_C45 | \
			((dev_type & 0x1f) << 16) | (reg_num & 0xffff))

typedef enum {
	ADC_RISING = 0,
	ADC_FALLING = 0xf0,
}
qca8084_adc_edge_t;

#endif
