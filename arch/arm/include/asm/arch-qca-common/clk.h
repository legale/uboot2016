/*
 * Copyright (c) 2015-2016, 2018-2021 The Linux Foundation. All rights reserved.
 *
 * Copyright (c) 2022, Qualcomm Innovation Center, Inc. All rights reserved.
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

#ifndef CLK_H
#define CLK_H

/* I2C clocks configuration */
#ifdef CONFIG_IPQ_I2C

// IPQ5018, IPQ6018, IPQ807X
#if defined(CONFIG_IPQ5018) || defined(CONFIG_IPQ6018) || defined(CONFIG_IPQ807x)

#define GCC_BLSP1_QUP1_I2C_APPS_CBCR			0x1802008
#define GCC_BLSP1_QUP1_I2C_APPS_CMD_RCGR		0x180200C
#define GCC_BLSP1_QUP1_I2C_APPS_CFG_RCGR		0x1802010

#define GCC_BLSP1_QUP2_I2C_APPS_CMD_RCGR		0x1803000
#define GCC_BLSP1_QUP2_I2C_APPS_CFG_RCGR		0x1803004
#define GCC_BLSP1_QUP2_I2C_APPS_CBCR			0x1803010

#else	// IPQ9574, devsoc

#define GCC_BLSP1_QUP1_I2C_APPS_CMD_RCGR		0x1802018
#define GCC_BLSP1_QUP1_I2C_APPS_CFG_RCGR		0x180201C
#define GCC_BLSP1_QUP1_I2C_APPS_CBCR			0x1802024

#define GCC_BLSP1_QUP2_I2C_APPS_CMD_RCGR		0x1803018
#define GCC_BLSP1_QUP2_I2C_APPS_CFG_RCGR		0x180301C
#define GCC_BLSP1_QUP2_I2C_APPS_CBCR			0x1803024

#endif


#define BLSP1_QUP_BASE					0x078B5000
#define I2C_PORT_ID(reg)	((reg - BLSP1_QUP_BASE) / GCC_BLSP1_QUP_I2C_OFFSET_INC)
#define GCC_BLSP1_QUP_I2C_OFFSET_INC			0x1000
#define GCC_BLSP1_QUP1_I2C_APPS_CFG_RCGR_SRC_SEL	(1 << 8)
#define GCC_BLSP1_QUP1_I2C_APPS_CFG_RCGR_SRC_DIV	(0x1F << 0)

#define GCC_BLSP1_QUP_I2C_APPS_CMD_RCGR(id)	((id < 1) ? \
				(GCC_BLSP1_QUP1_I2C_APPS_CMD_RCGR):\
				(GCC_BLSP1_QUP2_I2C_APPS_CMD_RCGR + (GCC_BLSP1_QUP_I2C_OFFSET_INC * (id-1))))

#define GCC_BLSP1_QUP_I2C_APPS_CFG_RCGR(id)	((id < 1) ? \
				(GCC_BLSP1_QUP1_I2C_APPS_CFG_RCGR):\
				(GCC_BLSP1_QUP2_I2C_APPS_CFG_RCGR + (GCC_BLSP1_QUP_I2C_OFFSET_INC * (id-1))))

#define GCC_BLSP1_QUP_I2C_APPS_CBCR(id)		((id < 1) ? \
				(GCC_BLSP1_QUP1_I2C_APPS_CBCR):\
				(GCC_BLSP1_QUP2_I2C_APPS_CBCR + (GCC_BLSP1_QUP_I2C_OFFSET_INC * (id-1))))

#define CMD_UPDATE	0x1
#define ROOT_EN		0x2
#define CLK_ENABLE	0x1

void i2c_clock_config(void);
#endif


#endif /*CLK_H*/

