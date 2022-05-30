/*
 * Copyright (c) 2016-2019, The Linux Foundation. All rights reserved.
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
#include <asm/global_data.h>
#include <asm/io.h>
#include <asm/errno.h>
#include <environment.h>
#include <fdtdec.h>
#include <asm/arch-qca-common/gpio.h>
#include <asm/arch-qca-common/uart.h>
#include <asm/arch-qca-common/scm.h>
#include <asm/arch-qca-common/iomap.h>
#include <devsoc.h>
#ifdef CONFIG_QPIC_NAND
#include <asm/arch-qca-common/qpic_nand.h>
#include <nand.h>
#endif
#ifdef CONFIG_QCA_MMC
#include <mmc.h>
#include <sdhci.h>
#endif
#ifdef CONFIG_USB_XHCI_IPQ
#include <usb.h>
#endif

DECLARE_GLOBAL_DATA_PTR;

extern int ipq_spi_init(u16);

const char *rsvd_node = "/reserved-memory";
const char *del_node[] = {"uboot",
			  "sbl",
			  NULL};
const add_node_t add_fdt_node[] = {{}};

unsigned int qpic_frequency = 0, qpic_phase = 0;

#ifdef CONFIG_QPIC_SERIAL
extern unsigned int qpic_training_offset;
#endif

#ifdef CONFIG_QCA_MMC
struct sdhci_host mmc_host;
#endif

struct dumpinfo_t dumpinfo_n[] = {
	/* TZ stores the DDR physical address at which it stores the
	 * APSS regs, UTCM copy dump. We will have the TZ IMEM
	 * IMEM Addr at which the DDR physical address is stored as
	 * the start
	 *     --------------------
         *     |  DDR phy (start) | ----> ------------------------
         *     --------------------       | APSS regsave (8k)    |
         *                                ------------------------
         *                                |                      |
	 *                                | 	 UTCM copy	 |
         *                                |        (192k)        |
	 *                                |                      |
         *                                ------------------------
	 */

	/* Compressed EBICS dump follows descending order
	 * to use in-memory compression for which destination
	 * for compression will be address of EBICS2.BIN
	 *
	 * EBICS2 - (ddr size / 2) [to] end of ddr
	 * EBICS1 - uboot end addr [to] (ddr size / 2)
	 * EBICS0 - ddr start      [to] uboot start addr
	 */

	{ "EBICS0.BIN", 0x40000000, 0x10000000, 0 },
	{ "EBICS2.BIN", 0x60000000, 0x20000000, 0, 0, 0, 0, 1 },
	{ "EBICS1.BIN", CONFIG_UBOOT_END_ADDR, 0x10000000, 0, 0, 0, 0, 1 },
	{ "EBICS0.BIN", 0x40000000, CONFIG_QCA_UBOOT_OFFSET, 0, 0, 0, 0, 1 },
	{ "IMEM.BIN", 0x08600000, 0x00001000, 0 },
	{ "UNAME.BIN", 0, 0, 0, 0, 0, MINIMAL_DUMP },
	{ "CPU_INFO.BIN", 0, 0, 0, 0, 0, MINIMAL_DUMP },
	{ "DMESG.BIN", 0, 0, 0, 0, 0, MINIMAL_DUMP },
	{ "PT.BIN", 0, 0, 0, 0, 0, MINIMAL_DUMP },
	{ "WLAN_MOD.BIN", 0, 0, 0, 0, 0, MINIMAL_DUMP },
};
int dump_entries_n = ARRAY_SIZE(dumpinfo_n);

/* Compressed dumps:
 * EBICS_S2 - (ddr start + 256M) [to] end of ddr
 * EBICS_S1 - uboot end addr     [to] (ddr start + 256M)
 * EBICS_S0 - ddr start          [to] uboot start addr
 */

struct dumpinfo_t dumpinfo_s[] = {
	{ "EBICS_S0.BIN", 0x40000000, 0xA600000, 0 },
	{ "EBICS_S1.BIN", CONFIG_TZ_END_ADDR, 0x10000000, 0 },
	{ "EBICS_S2.BIN", 0x50000000, 0x10000000, 0, 0, 0, 0, 1 },
	{ "EBICS_S1.BIN", CONFIG_UBOOT_END_ADDR, 0x5B00000, 0, 0, 0, 0, 1 },
	{ "EBICS_S0.BIN", 0x40000000, CONFIG_QCA_UBOOT_OFFSET, 0, 0, 0, 0, 1 },
	{ "IMEM.BIN", 0x08600000, 0x00001000, 0 },
	{ "UNAME.BIN", 0, 0, 0, 0, 0, MINIMAL_DUMP },
	{ "CPU_INFO.BIN", 0, 0, 0, 0, 0, MINIMAL_DUMP },
	{ "DMESG.BIN", 0, 0, 0, 0, 0, MINIMAL_DUMP },
	{ "PT.BIN", 0, 0, 0, 0, 0, MINIMAL_DUMP },
	{ "WLAN_MOD.BIN", 0, 0, 0, 0, 0, MINIMAL_DUMP },
};
int dump_entries_s = ARRAY_SIZE(dumpinfo_s);

void qca_serial_init(struct ipq_serial_platdata *plat)
{
	int ret;

	if (plat->gpio_node >= 0) {
		qca_gpio_init(plat->gpio_node);
	}

	plat->port_id = UART_PORT_ID(plat->reg_base);
	ret = uart_clock_config(plat);
	if (ret)
		printf("UART clock config failed %d\n", ret);

	return;
}

void reset_board(void)
{
	run_command("reset", 0);
}

/*
 * Set the uuid in bootargs variable for mounting rootfilesystem
 */
#ifdef CONFIG_QCA_MMC
int set_uuid_bootargs(char *boot_args, char *part_name, int buflen,
			bool gpt_flag)
{
	int ret, len;
	block_dev_desc_t *blk_dev;
	disk_partition_t disk_info;

	blk_dev = mmc_get_dev(mmc_host.dev_num);
	if (!blk_dev) {
		printf("Invalid block device name\n");
		return -EINVAL;
	}

	if (buflen <= 0 || buflen > MAX_BOOT_ARGS_SIZE)
		return -EINVAL;

#ifdef CONFIG_PARTITION_UUIDS
	ret = get_partition_info_efi_by_name(blk_dev,
			part_name, &disk_info);
	if (ret) {
		printf("bootipq: unsupported partition name %s\n",part_name);
		return -EINVAL;
	}
	if ((len = strlcpy(boot_args, "root=PARTUUID=", buflen)) >= buflen)
		return -EINVAL;
#else
	if ((len = strlcpy(boot_args, "rootfsname=", buflen)) >= buflen)
		return -EINVAL;
#endif
	boot_args += len;
	buflen -= len;

#ifdef CONFIG_PARTITION_UUIDS
	if ((len = strlcpy(boot_args, disk_info.uuid, buflen)) >= buflen)
		return -EINVAL;
#else
	if ((len = strlcpy(boot_args, part_name, buflen)) >= buflen)
		return -EINVAL;
#endif
	boot_args += len;
	buflen -= len;

	if (gpt_flag && strlcpy(boot_args, " gpt", buflen) >= buflen)
		return -EINVAL;

	return 0;
}
#else
int set_uuid_bootargs(char *boot_args, char *part_name, int buflen,
			bool gpt_flag)
{
	return 0;
}
#endif

#ifdef CONFIG_QCA_MMC
void mmc_iopad_config(struct sdhci_host *host)
{
	u32 val;
	val = sdhci_readb(host, SDHCI_VENDOR_IOPAD);
	/*set bit 15 & 16*/
	val |= 0x18000;
	writel(val, host->ioaddr + SDHCI_VENDOR_IOPAD);
}

void sdhci_bus_pwr_off(struct sdhci_host *host)
{
	u32 val;

	val = sdhci_readb(host, SDHCI_HOST_CONTROL);
	sdhci_writeb(host,(val & (~SDHCI_POWER_ON)), SDHCI_POWER_CONTROL);
}

__weak void board_mmc_deinit(void)
{
	/*since we do not have misc register in devsoc
	 * so simply return from this function
	 */
	return;
}

int board_mmc_init(bd_t *bis)
{
	int node, gpio_node;
	int ret = 0;
	qca_smem_flash_info_t *sfi = &qca_smem_flash_info;
	node = fdt_path_offset(gd->fdt_blob, "mmc");
	if (node < 0) {
		printf("sdhci: Node Not found, skipping initialization\n");
		return -1;
	}

	gpio_node = fdt_subnode_offset(gd->fdt_blob, node, "mmc_gpio");
	if (node >= 0)
		qca_gpio_init(gpio_node);

	mmc_host.ioaddr = (void *)MSM_SDC1_SDHCI_BASE;
	mmc_host.voltages = MMC_VDD_165_195;
	mmc_host.version = SDHCI_SPEC_300;
	mmc_host.cfg.part_type = PART_TYPE_EFI;
	mmc_host.quirks = SDHCI_QUIRK_BROKEN_VOLTAGE;

	emmc_clock_reset();
	udelay(10);
	emmc_clock_init();

	if (add_sdhci(&mmc_host, 200000000, 400000)) {
		printf("add_sdhci fail!\n");
		return -1;
	}

	if (!ret && sfi->flash_type == SMEM_BOOT_MMC_FLASH) {
		ret = board_mmc_env_init(mmc_host);
	}

	return ret;
}
#else
int board_mmc_init(bd_t *bis)
{
	return 0;
}
#endif
#ifdef CONFIG_PCI_IPQ
void board_pci_init(int id)
{
	int node, gpio_node, ret, lane;
	char name[16];
	struct fdt_resource pci_rst;

	snprintf(name, sizeof(name), "pci%d", id);
	node = fdt_path_offset(gd->fdt_blob, name);
	if (node < 0) {
		printf("Could not find PCI%d in device tree\n", id);
		return;
	}

	gpio_node = fdt_subnode_offset(gd->fdt_blob, node, "pci_gpio");
	if (gpio_node >= 0)
		qca_gpio_init(gpio_node);

	lane = fdtdec_get_int(gd->fdt_blob, node, "lane", 1);

	/*
	 * setting dual port mode if PCIE1 & PCIE2 come up with 1 lane.
	 */
	if ((id == 1) || (id ==2)) {
		if (lane == 1)
			writel(TWO_PORT_MODE,
				(void *)TCSR_MODE_CTRL_2PORT_2LANE);
		else
			writel(TWO_LANE_MODE,
				(void *)TCSR_MODE_CTRL_2PORT_2LANE);
		mdelay(10);
	}

	ret = fdt_get_named_resource(gd->fdt_blob, node, "reg",
				"reg-names", "pci_rst", &pci_rst);
	if (ret == 0) {
		set_mdelay_clearbits_le32(pci_rst.start, 0x1, 10);
		set_mdelay_clearbits_le32(pci_rst.end + 1, 0x1, 10);
	}

	pcie_v2_clock_init(id);

	return;
}

void board_pci_deinit()
{
	int node, gpio_node, i, err, is_x2;
	char name[16];
	struct fdt_resource parf;
	struct fdt_resource pci_phy;

	for (i = 0; i < PCI_MAX_DEVICES; i++) {
		snprintf(name, sizeof(name), "pci%d", i);
		node = fdt_path_offset(gd->fdt_blob, name);
		if (node < 0) {
			printf("Could not find PCI%d in device tree\n", i);
			continue;
		}
		err = fdt_get_named_resource(gd->fdt_blob, node, "reg",
				"reg-names", "parf", &parf);

		writel(0x0, parf.start + 0x358);
		writel(0x1, parf.start + 0x40);

		err = fdt_get_named_resource(gd->fdt_blob, node, "reg",
				"reg-names", "pci_phy", &pci_phy);
		if (err < 0)
			continue;

		if ((i == 0) || (i == 1))
			is_x2 = 0;
		else
			is_x2 = 1;

		writel(0x1, pci_phy.start + (0x800 + (0x800 * is_x2)));
		writel(0x0, pci_phy.start + (0x804 + (0x800 * is_x2)));

		gpio_node = fdt_subnode_offset(gd->fdt_blob, node, "pci_gpio");
		if (gpio_node >= 0)
			qca_gpio_deinit(gpio_node);

		pcie_v2_clock_deinit(i);
	}

	return;
}
#endif
#ifdef CONFIG_USB_XHCI_IPQ
void board_usb_deinit(int id)
{
	int nodeoff;
	char node_name[8];

	snprintf(node_name, sizeof(node_name), "usb%d", id);
	nodeoff = fdt_path_offset(gd->fdt_blob, node_name);
	if (fdtdec_get_int(gd->fdt_blob, nodeoff, "qcom,emulation", 0))
		return;
}

int ipq_board_usb_init(void)
{
	int i, nodeoff, ssphy;
	char node_name[8];

	for (i=0; i < CONFIG_USB_MAX_CONTROLLER_COUNT; i++) {
		snprintf(node_name, sizeof(node_name), "usb%d", i);
		nodeoff = fdt_path_offset(gd->fdt_blob, node_name);
		if (nodeoff < 0){
			printf("USB: Node Not found, skipping initialization\n");
			return 0;
		}

		ssphy = fdtdec_get_int(gd->fdt_blob, nodeoff, "ssphy", 0);
		if (!fdtdec_get_int(gd->fdt_blob, nodeoff, "qcom,emulation", 0)) {

			usb_clock_init(i, ssphy);
		}else {
			/* Config user control register */
			writel(0x0C804010, USB30_GUCTL);
		}
	}

	return 0;
}
#endif

__weak int ipq_get_tz_version(char *version_name, int buf_size)
{
	return 1;
}

int apps_iscrashed(void)
{
	return 0;
}

void reset_crashdump(void)
{
	unsigned int ret = 0;
	unsigned int cookie = 0;

#ifdef CONFIG_IPQ_RUNTIME_FAILSAFE
	cookie = ipq_read_tcsr_boot_misc();
	fs_debug("\nFailsafe: %s: Clearing DLOAD and NonHLOS bits\n", __func__);
	cookie &= ~(DLOAD_BITS);
	cookie &= ~(IPQ_FS_NONHLOS_BIT);
#endif
	qca_scm_sdi();
	ret = qca_scm_dload(cookie);
	if (ret)
		printf ("Error in reseting the Magic cookie\n");
	return;
}

void psci_sys_reset(void)
{
	__invoke_psci_fn_smc(PSCI_RESET_SMC_ID, 0, 0, 0);
}

void qti_scm_pshold(void)
{
	return;
}

void reset_cpu(unsigned long a)
{
	reset_crashdump();

	psci_sys_reset();

	while(1);
}

void board_nand_init(void)
{
#ifdef CONFIG_QPIC_SERIAL
	/* check for nand node in dts
	 * if nand node in dts is disabled then
	 * simply return from here without
	 * initializing
	 */
	int node;
	node = fdt_path_offset(gd->fdt_blob, "/nand-controller");
	if (!fdtdec_get_is_enabled(gd->fdt_blob, node)) {
		printf("QPIC: disabled, skipping initialization\n");
	} else {
		qpic_nand_init(NULL);
	}
#endif
#ifdef CONFIG_QCA_SPI
	int gpio_node;
	gpio_node = fdt_path_offset(gd->fdt_blob, "/spi/spi_gpio");
	if (gpio_node >= 0) {
		qca_gpio_init(gpio_node);
#ifdef CONFIG_MTD_DEVICE
		ipq_spi_init(CONFIG_IPQ_SPI_NOR_INFO_IDX);
#endif
	}
#endif
}

void enable_caches(void)
{
	qca_smem_flash_info_t *sfi = &qca_smem_flash_info;
	smem_get_boot_flash(&sfi->flash_type,
		&sfi->flash_index,
		&sfi->flash_chip_select,
		&sfi->flash_block_size,
		&sfi->flash_density);
	icache_enable();
	/*Skips dcache_enable during JTAG recovery */
	if (sfi->flash_type)
		dcache_enable();
}

void disable_caches(void)
{
	icache_disable();
	dcache_disable();
}

unsigned long timer_read_counter(void)
{
	return 0;
}

void set_flash_secondary_type(qca_smem_flash_info_t *smem)
{
	return;
};

void ipq_fdt_fixup_usb_device_mode(void *blob)
{
	return;
}
