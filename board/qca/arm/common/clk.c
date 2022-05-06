/*
 * Copyright (c) 2015-2016, 2018-2020 The Linux Foundation. All rights reserved.
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

#include <common.h>
#include <asm/global_data.h>
#include <asm/io.h>
#include <asm/arch-qca-common/clk.h>
#include <asm/errno.h>
#include <fdtdec.h>

DECLARE_GLOBAL_DATA_PTR;

#ifdef CONFIG_IPQ_I2C
void i2c_clock_config(void)
{
	int cfg, i2c_id;
	int i2c_node;
	const u32 *i2c_base;
	int i;
	char alias[6];

	for (i = 0; i < CONFIG_IPQ_MAX_BLSP_QUPS; i++) {
		memset(alias, 0, 6);
		snprintf(alias, 5, "i2c%d", i);

		i2c_node = fdt_path_offset(gd->fdt_blob, alias);
		if (i2c_node >= 0) {
			i2c_base = fdt_getprop(gd->fdt_blob, i2c_node, "reg", NULL);
			if (i2c_base) {
				i2c_id = I2C_PORT_ID(fdt32_to_cpu(i2c_base[0]));

				/* Configure qup1_i2c_apps_clk_src */
				cfg = (GCC_BLSP1_QUP1_I2C_APPS_CFG_RCGR_SRC_SEL |
						GCC_BLSP1_QUP1_I2C_APPS_CFG_RCGR_SRC_DIV);
				writel(cfg, GCC_BLSP1_QUP_I2C_APPS_CFG_RCGR(i2c_id));

				writel(CMD_UPDATE, GCC_BLSP1_QUP_I2C_APPS_CMD_RCGR(i2c_id));
				mdelay(100);
				writel(ROOT_EN, GCC_BLSP1_QUP_I2C_APPS_CMD_RCGR(i2c_id));

				/* Configure CBCR */
				writel(CLK_ENABLE, GCC_BLSP1_QUP_I2C_APPS_CBCR(i2c_id));
			}
		}
	}
}
#endif
