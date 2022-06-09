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
#ifndef _QCA8084_CLK_H_
#define _QCA8084_CLK_H_

#define QCA8084_SWITCH_CORE_CLK			"qca8084_gcc_switch_core_clk"
#define QCA8084_APB_BRIDGE_CLK			"qca8084_gcc_apb_bridge_clk"

#define QCA8084_MAC0_TX_CLK			"qca8084_gcc_mac0_tx_clk"
#define QCA8084_MAC0_TX_UNIPHY1_CLK		"qca8084_gcc_mac0_tx_srds1_clk"

#define QCA8084_MAC0_RX_CLK			"qca8084_gcc_mac0_rx_clk"
#define QCA8084_MAC0_RX_UNIPHY1_CLK		"qca8084_gcc_mac0_rx_srds1_clk"

#define QCA8084_MAC1_TX_CLK			"qca8084_gcc_mac1_tx_clk"
#define QCA8084_MAC1_GEPHY0_TX_CLK		"qca8084_gcc_mac1_gephy0_tx_clk"
#define QCA8084_MAC1_UNIPHY1_CH0_RX_CLK		"qca8084_gcc_mac1_srds1_ch0_rx_clk"
#define QCA8084_MAC1_UNIPHY1_CH0_XGMII_RX_CLK	"qca8084_gcc_mac1_srds1_ch0_xgmii_rx_clk"

#define QCA8084_MAC1_RX_CLK			"qca8084_gcc_mac1_rx_clk"
#define QCA8084_MAC1_GEPHY0_RX_CLK		"qca8084_gcc_mac1_gephy0_rx_clk"
#define QCA8084_MAC1_UNIPHY1_CH0_TX_CLK		"qca8084_gcc_mac1_srds1_ch0_tx_clk"
#define QCA8084_MAC1_UNIPHY1_CH0_XGMII_TX_CLK	"qca8084_gcc_mac1_srds1_ch0_xgmii_tx_clk"

#define QCA8084_MAC2_TX_CLK			"qca8084_gcc_mac2_tx_clk"
#define QCA8084_MAC2_GEPHY1_TX_CLK		"qca8084_gcc_mac2_gephy1_tx_clk"
#define QCA8084_MAC2_UNIPHY1_CH1_RX_CLK		"qca8084_gcc_mac2_srds1_ch1_rx_clk"
#define QCA8084_MAC2_UNIPHY1_CH1_XGMII_RX_CLK	"qca8084_gcc_mac2_srds1_ch1_xgmii_rx_clk"

#define QCA8084_MAC2_RX_CLK			"qca8084_gcc_mac2_rx_clk"
#define QCA8084_MAC2_GEPHY1_RX_CLK		"qca8084_gcc_mac2_gephy1_rx_clk"
#define QCA8084_MAC2_UNIPHY1_CH1_TX_CLK		"qca8084_gcc_mac2_srds1_ch1_tx_clk"
#define QCA8084_MAC2_UNIPHY1_CH1_XGMII_TX_CLK	"qca8084_gcc_mac2_srds1_ch1_xgmii_tx_clk"

#define QCA8084_MAC3_TX_CLK			"qca8084_gcc_mac3_tx_clk"
#define QCA8084_MAC3_GEPHY2_TX_CLK		"qca8084_gcc_mac3_gephy2_tx_clk"
#define QCA8084_MAC3_UNIPHY1_CH2_RX_CLK		"qca8084_gcc_mac3_srds1_ch2_rx_clk"
#define QCA8084_MAC3_UNIPHY1_CH2_XGMII_RX_CLK	"qca8084_gcc_mac3_srds1_ch2_xgmii_rx_clk"

#define QCA8084_MAC3_RX_CLK			"qca8084_gcc_mac3_rx_clk"
#define QCA8084_MAC3_GEPHY2_RX_CLK		"qca8084_gcc_mac3_gephy2_rx_clk"
#define QCA8084_MAC3_UNIPHY1_CH2_TX_CLK		"qca8084_gcc_mac3_srds1_ch2_tx_clk"
#define QCA8084_MAC3_UNIPHY1_CH2_XGMII_TX_CLK	"qca8084_gcc_mac3_srds1_ch2_xgmii_tx_clk"

#define QCA8084_MAC4_TX_CLK			"qca8084_gcc_mac4_tx_clk"
#define QCA8084_MAC4_GEPHY3_TX_CLK		"qca8084_gcc_mac4_gephy3_tx_clk"
#define QCA8084_MAC4_UNIPHY1_CH3_RX_CLK		"qca8084_gcc_mac4_srds1_ch3_rx_clk"
#define QCA8084_MAC4_UNIPHY1_CH3_XGMII_RX_CLK	"qca8084_gcc_mac4_srds1_ch3_xgmii_rx_clk"

#define QCA8084_MAC4_RX_CLK			"qca8084_gcc_mac4_rx_clk"
#define QCA8084_MAC4_GEPHY3_RX_CLK		"qca8084_gcc_mac4_gephy3_rx_clk"
#define QCA8084_MAC4_UNIPHY1_CH3_TX_CLK		"qca8084_gcc_mac4_srds1_ch3_tx_clk"
#define QCA8084_MAC4_UNIPHY1_CH3_XGMII_TX_CLK	"qca8084_gcc_mac4_srds1_ch3_xgmii_tx_clk"

#define QCA8084_MAC5_TX_CLK			"qca8084_gcc_mac5_tx_clk"
#define QCA8084_MAC5_TX_UNIPHY0_CLK		"qca8084_gcc_mac5_tx_srds0_clk"
#define QCA8084_MAC5_TX_SRDS0_CLK_SRC		"qca8084_gcc_mac5_tx_srds0_clk_src"

#define QCA8084_MAC5_RX_CLK			"qca8084_gcc_mac5_rx_clk"
#define QCA8084_MAC5_RX_UNIPHY0_CLK		"qca8084_gcc_mac5_rx_srds0_clk"
#define QCA8084_MAC5_RX_SRDS0_CLK_SRC		"qca8084_gcc_mac5_rx_srds0_clk_src"

#define QCA8084_SEC_CTRL_CLK			"qca8084_gcc_sec_ctrl_clk"
#define QCA8084_SEC_CTRL_SENSE_CLK		"qca8084_gcc_sec_ctrl_sense_clk"

#define QCA8084_SRDS0_SYS_CLK			"qca8084_gcc_srds0_sys_clk"
#define QCA8084_SRDS1_SYS_CLK			"qca8084_gcc_srds1_sys_clk"
#define QCA8084_GEPHY0_SYS_CLK			"qca8084_gcc_gephy0_sys_clk"
#define QCA8084_GEPHY1_SYS_CLK			"qca8084_gcc_gephy1_sys_clk"
#define QCA8084_GEPHY2_SYS_CLK			"qca8084_gcc_gephy2_sys_clk"
#define QCA8084_GEPHY3_SYS_CLK			"qca8084_gcc_gephy3_sys_clk"

#define QCA8084_AHB_CLK				"qca8084_gcc_ahb_clk"
#define QCA8084_SEC_CTRL_AHB_CLK		"qca8084_gcc_sec_ctrl_ahb_clk"
#define QCA8084_TLMM_CLK			"qca8084_gcc_tlmm_clk"
#define QCA8084_TLMM_AHB_CLK			"qca8084_gcc_tlmm_ahb_clk"
#define QCA8084_CNOC_AHB_CLK			"qca8084_gcc_cnoc_ahb_clk"
#define QCA8084_MDIO_AHB_CLK			"qca8084_gcc_mdio_ahb_clk"
#define QCA8084_MDIO_MASTER_AHB_CLK		"qca8084_gcc_mdio_master_ahb_clk"

#define QCA8084_GLOBAL_RST			"qca8084_gcc_global_rst"
#define QCA8084_UNIPHY_XPCS_RST			"qca8084_uniphy_xpcs_rst"
#define QCA8084_GEPHY_DSP_HW_RST		"qca8084_gephy_dsp_hw_rst"
#define QCA8084_GEPHY_P3_MDC_SW_RST		"qca8084_gephy_p3_mdc_sw_rst"
#define QCA8084_GEPHY_P2_MDC_SW_RST		"qca8084_gephy_p2_mdc_sw_rst"
#define QCA8084_GEPHY_P1_MDC_SW_RST		"qca8084_gephy_p1_mdc_sw_rst"
#define QCA8084_GEPHY_P0_MDC_SW_RST		"qca8084_gephy_p0_mdc_sw_rst"



typedef enum {
	QCA8084_P_XO,
	QCA8084_P_UNIPHY0_RX,
	QCA8084_P_UNIPHY0_TX,
	QCA8084_P_UNIPHY1_RX,
	QCA8084_P_UNIPHY1_TX,
	QCA8084_P_UNIPHY1_RX312P5M,
	QCA8084_P_UNIPHY1_TX312P5M,
	QCA8084_P_MAX,
} qca8084_clk_parent_t;

struct qca8084_clk_data {
	unsigned long rate;
	unsigned int rcg_val;
	unsigned int cdiv_val;
	unsigned int cbc_val;
};

struct qca8084_parent_data {
	unsigned long prate;		/* RCG input clock rate */
	qca8084_clk_parent_t parent;	/* RCG parent clock id */
	int cfg;			/* RCG clock src value */
};

struct clk_lookup {
	unsigned int rcg;
	unsigned int cdiv;
	unsigned int cbc;
	unsigned int rst_bit;
	const char *clk_name;
	const unsigned long *support_rate;
	unsigned int num_rate;
	const struct qca8084_parent_data *pdata;
	unsigned int num_parent;
};

#define CLK_LOOKUP(_rcg, _cdiv, _cbc, _rst_bit, _clk_name,		\
		_rate, _num_rate, _pdata, _num_parent)			\
{									\
	.rcg = _rcg,							\
	.cdiv = _cdiv,							\
	.cbc = _cbc,							\
	.rst_bit = _rst_bit,						\
	.clk_name = _clk_name,						\
	.support_rate = _rate,						\
	.num_rate = _num_rate,						\
	.pdata = _pdata,						\
	.num_parent = _num_parent,					\
}

#define QCA8084_CLK_TYPE_EPHY			BIT(0)
#define QCA8084_CLK_TYPE_UNIPHY			BIT(1)
#define QCA8084_CLK_TYPE_MAC			BIT(2)

#define UQXGMII_SPEED_2500M_CLK			312500000
#define UQXGMII_SPEED_1000M_CLK			125000000
#define UQXGMII_SPEED_100M_CLK			25000000
#define UQXGMII_SPEED_10M_CLK			2500000
#define UQXGMII_XPCS_SPEED_2500M_CLK		78125000
#define QCA8084_AHB_CLK_RATE_104P17M		104160000
#define QCA8084_SYS_CLK_RATE_25M		25000000
#define QCA8084_XO_CLK_RATE_50M			50000000

#define QCA8084_CLK_BASE_REG			0x0c800000
#define QCA8084_CLK_MUX_SEL			0x300
#define QCA8084_UNIPHY0_MUX_SEL_MASK		BITS_MASK(0, 2)
#define QCA8084_UNIPHY0_SEL_MAC5		0x3
#define QCA8084_UNIPHY0_SEL_MAC4		0

#define RCGR_CMD_ROOT_OFF			BIT(31)
#define RCGR_CMD_UPDATE				BIT(0)
#define RCGR_SRC_SEL				BITS_MASK(8, 3)
#define RCGR_SRC_SEL_SHIFT			8
#define RCGR_HDIV				BITS_MASK(0, 5)
#define RCGR_HDIV_SHIFT				0
#define RCGR_DIV_BYPASS				0
#define RCGR_DIV_MAX				0x1f
#define CDIVR_DIVIDER_10			9	/* CDIVR divided by N + 1 */
#define CDIVR_DIVIDER				BITS_MASK(0, 4)
#define CDIVR_DIVIDER_SHIFT			0
#define CBCR_CLK_OFF				BIT(31)
#define CBCR_CLK_RESET				BIT(2)
#define CBCR_CLK_ENABLE				BIT(0)


/* work mode */
#define WORK_MODE
#define WORK_MODE_ID                                    0
#define WORK_MODE_OFFSET                                0xC90F030
#define WORK_MODE_E_LENGTH                              4
#define WORK_MODE_E_OFFSET                              0
#define WORK_MODE_NR_E                                  1

/* port5 sel */
#define WORK_MODE_PORT5_SEL
#define WORK_MODE_PORT5_SEL_BOFFSET                     5
#define WORK_MODE_PORT5_SEL_BLEN                        1
#define WORK_MODE_PORT5_SEL_FLAG                        HSL_RW

/* phy3 sel1 */
#define WORK_MODE_PHY3_SEL1
#define WORK_MODE_PHY3_SEL1_BOFFSET                     4
#define WORK_MODE_PHY3_SEL1_BLEN                        1
#define WORK_MODE_PHY3_SEL1_FLAG                        HSL_RW

/* phy3 sel0 */
#define WORK_MODE_PHY3_SEL0
#define WORK_MODE_PHY3_SEL0_BOFFSET                     3
#define WORK_MODE_PHY3_SEL0_BLEN                        1
#define WORK_MODE_PHY3_SEL0_FLAG                        HSL_RW

/* phy2 sel */
#define WORK_MODE_PHY2_SEL
#define WORK_MODE_PHY2_SEL_BOFFSET                      2
#define WORK_MODE_PHY2_SEL_BLEN                         1
#define WORK_MODE_PHY2_SEL_FLAG                         HSL_RW

/* phy1 sel */
#define WORK_MODE_PHY1_SEL
#define WORK_MODE_PHY1_SEL_BOFFSET                      1
#define WORK_MODE_PHY1_SEL_BLEN                         1
#define WORK_MODE_PHY1_SEL_FLAG                         HSL_RW

/* phy0 sel */
#define WORK_MODE_PHY0_SEL
#define WORK_MODE_PHY0_SEL_BOFFSET                      0
#define WORK_MODE_PHY0_SEL_BLEN                         1
#define WORK_MODE_PHY0_SEL_FLAG                         HSL_RW

#define QCA8084_WORK_MODE_MASK \
	(BITSM(WORK_MODE_PHY0_SEL_BOFFSET, WORK_MODE_PORT5_SEL_BOFFSET + 1))

typedef enum {
	QCA8084_SWITCH_MODE =
		(BIT(WORK_MODE_PHY3_SEL1_BOFFSET)),
	QCA8084_SWITCH_BYPASS_PORT5_MODE =
		(BIT(WORK_MODE_PORT5_SEL_BOFFSET)),
	QCA8084_PHY_UQXGMII_MODE =
		(BIT(WORK_MODE_PORT5_SEL_BOFFSET) |
		 BIT(WORK_MODE_PHY3_SEL0_BOFFSET) |
		 BIT(WORK_MODE_PHY2_SEL_BOFFSET) |
		 BIT(WORK_MODE_PHY1_SEL_BOFFSET) |
		 BIT(WORK_MODE_PHY0_SEL_BOFFSET)),
	QCA8084_PHY_SGMII_UQXGMII_MODE =
		(BIT(WORK_MODE_PORT5_SEL_BOFFSET) |
		 BIT(WORK_MODE_PHY2_SEL_BOFFSET) |
		 BIT(WORK_MODE_PHY1_SEL_BOFFSET) |
		 BIT(WORK_MODE_PHY0_SEL_BOFFSET)),
	QCA8084_WORK_MODE_MAX,
} qca8084_work_mode_t;


#endif                          /* _QCA8084_CLK_H_ */
