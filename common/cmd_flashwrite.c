/*
 * Copyright (c) 2018, 2020 The Linux Foundation. All rights reserved.

 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

/*
 * FlashWrite command support
 */
#include <common.h>
#include <command.h>
#include <asm/arch-qca-common/smem.h>
#include <part.h>
#include <linux/mtd/mtd.h>
#include <nand.h>
#include <mmc.h>
#include <sdhci.h>
#include <ubi_uboot.h>
#include <fdtdec.h>
#include <asm/arch-qca-common/qpic_nand.h>
#include <nand.h>


DECLARE_GLOBAL_DATA_PTR;
#ifndef CONFIG_SDHCI_SUPPORT
extern qca_mmc mmc_host;
#else
extern struct sdhci_host mmc_host;
#endif

#define SMEM_PTN_NAME_MAX     16
#define GPT_PART_NAME "0:GPT"
#define GPT_BACKUP_PART_NAME "0:GPTBACKUP"

#ifdef CONFIG_IPQ_MIBIB_RELOAD
#define HEADER_MAGIC1 0xFE569FAC
#define HEADER_MAGIC2 0xCD7F127A
#define HEADER_VERSION 4

#define SHA1_SIG_LEN 41

struct header {
	unsigned magic[2];
	unsigned version;
} __attribute__ ((__packed__));
#endif

static int write_to_flash(int flash_type, uint32_t address, uint32_t offset,
uint32_t part_size, uint32_t file_size, char *layout)
{

	char runcmd[256];
	int nand_dev = CONFIG_NAND_FLASH_INFO_IDX;

	if (((flash_type == SMEM_BOOT_NAND_FLASH) ||
		(flash_type == SMEM_BOOT_QSPI_NAND_FLASH))) {

		snprintf(runcmd, sizeof(runcmd), "nand device %d && ", nand_dev);

		if (strcmp(layout, "default") != 0) {

			snprintf(runcmd + strlen(runcmd), sizeof(runcmd),
						"ipq_nand %s && ", layout);
		}

		snprintf(runcmd + strlen(runcmd), sizeof(runcmd),
			"nand erase 0x%x 0x%x && "
			"nand write 0x%x 0x%x 0x%x && ",
			offset, part_size,
			address, offset, file_size);

	} else if (flash_type == SMEM_BOOT_MMC_FLASH) {

		snprintf(runcmd, sizeof(runcmd),
			"mmc erase 0x%x 0x%x && "
			"mmc write 0x%x 0x%x 0x%x && ",
			offset, part_size,
			address, offset, file_size);

	} else if (flash_type == SMEM_BOOT_SPI_FLASH) {

		snprintf(runcmd, sizeof(runcmd),
			"sf probe && "
			"sf erase 0x%x 0x%x && "
			"sf write 0x%x 0x%x 0x%x && ",
			offset, part_size,
			address, offset, file_size);
	}

	if (run_command(runcmd, 0) != CMD_RET_SUCCESS)
		return CMD_RET_FAILURE;

	return CMD_RET_SUCCESS;
}

static int fl_erase(int flash_type, uint32_t offset, uint32_t part_size,
							 char *layout)
{
	char runcmd[256];
	int nand_dev = CONFIG_NAND_FLASH_INFO_IDX;

	if (((flash_type == SMEM_BOOT_NAND_FLASH) ||
		(flash_type == SMEM_BOOT_QSPI_NAND_FLASH))) {

		snprintf(runcmd, sizeof(runcmd), "nand device %d && ", nand_dev);
		if (strcmp(layout, "default") != 0) {
			snprintf(runcmd + strlen(runcmd), sizeof(runcmd),
						"ipq_nand %s && ", layout);
		}

		snprintf(runcmd + strlen(runcmd), sizeof(runcmd),
					"nand erase 0x%x 0x%x ",
					 offset, part_size);

	} else if (flash_type == SMEM_BOOT_MMC_FLASH) {

		snprintf(runcmd, sizeof(runcmd),
				"mmc erase 0x%x 0x%x ",
				 offset, part_size);

	} else if (flash_type == SMEM_BOOT_SPI_FLASH) {

		snprintf(runcmd, sizeof(runcmd),
				"sf probe && "
				"sf erase 0x%x 0x%x ",
				 offset, part_size);
	}

	if (run_command(runcmd, 0) != CMD_RET_SUCCESS)
		return CMD_RET_FAILURE;

	return CMD_RET_SUCCESS;
}

#ifdef IPQ_UBI_VOL_WRITE_SUPPORT
int ubi_vol_present(char* ubi_vol_name)
{
	int i;
	int j=0;
	struct ubi_device *ubi;
	struct ubi_volume *vol;

	if (ubi_set_rootfs_part())
		return 0;

	ubi = ubi_devices[0];
	for (i = 0; ubi && i < (ubi->vtbl_slots + 1); i++) {
		vol = ubi->volumes[i];
		if (!vol)
			continue;	/* Empty record */
		if (vol->name_len <= UBI_VOL_NAME_MAX &&
		    strnlen(vol->name, vol->name_len + 1) == vol->name_len) {
			j++;
			if (!strncmp(ubi_vol_name, vol->name,
						UBI_VOL_NAME_MAX)) {
				return 1;
			}
		}

		if (j == ubi->vol_count - UBI_INT_VOL_COUNT)
			break;
	}

	printf("volume or partition %s not found\n", ubi_vol_name);
	return 0;
}

int write_ubi_vol(char* ubi_vol_name, uint32_t load_addr, uint32_t file_size)
{
	char runcmd[256];

	if (!strncmp(ubi_vol_name, "ubi_rootfs", UBI_VOL_NAME_MAX)) {
		snprintf(runcmd, sizeof(runcmd),
			"ubi remove rootfs_data &&"
			"ubi remove %s &&"
			"ubi create %s 0x%x &&"
			"ubi write 0x%x %s 0x%x &&"
			"ubi create rootfs_data",
			 ubi_vol_name, ubi_vol_name, file_size,
			 load_addr, ubi_vol_name, file_size);
	} else {
		snprintf(runcmd, sizeof(runcmd),
			"ubi write 0x%x %s 0x%x ",
			 load_addr, ubi_vol_name, file_size);
	}

	return run_command(runcmd, 0);
}
#endif

static int do_flash(cmd_tbl_t *cmdtp, int flag, int argc,
char * const argv[])
{
	int flash_cmd = 0;
	uint32_t offset, part_size, adj_size;
	uint32_t load_addr = 0;
	uint32_t file_size = 0;
	uint32_t size_block, start_block, file_size_cpy;
	char *part_name = NULL, *filesize, *loadaddr;
	int flash_type, ret, retn;
	unsigned int active_part = 0;
	char *layout = NULL;
#ifdef CONFIG_IPQ806X
	char* layout_linux[] = {"rootfs", "0:BOOTCONFIG", "0:BOOTCONFIG1"};
	int len, i;
#endif
	offset = 0;
	part_size = 0;
	layout = "default";
	retn = CMD_RET_FAILURE;

#ifdef CONFIG_QCA_MMC
	block_dev_desc_t *blk_dev;
#endif
	disk_partition_t disk_info = {0};
	qca_smem_flash_info_t *sfi = &qca_smem_flash_info;
#ifdef CONFIG_CMD_NAND
	nand_info_t *nand = &nand_info[CONFIG_NAND_FLASH_INFO_IDX];
#endif
	if (strcmp(argv[0], "flash") == 0)
		flash_cmd = 1;

	if (flash_cmd) {
		if ((argc < 2) || (argc > 4))
			return CMD_RET_USAGE;

		if (argc == 2) {
			loadaddr = getenv("fileaddr");
			if (loadaddr != NULL)
				load_addr = simple_strtoul(loadaddr, NULL, 16);
			else
				return CMD_RET_USAGE;

			filesize = getenv("filesize");
			if (filesize != NULL)
				file_size = simple_strtoul(filesize, NULL, 16);
			else
				return CMD_RET_USAGE;

		} else if (argc == 4) {
			load_addr = simple_strtoul(argv[2], NULL, 16);
			file_size = simple_strtoul(argv[3], NULL, 16);

		} else
			return CMD_RET_USAGE;

		file_size_cpy = file_size;
	}
	else {
		if (argc != 2)
			return CMD_RET_USAGE;
	}

	flash_type = sfi->flash_type;
	part_name = argv[1];

	if (((sfi->flash_type == SMEM_BOOT_NAND_FLASH) ||
		(sfi->flash_type == SMEM_BOOT_QSPI_NAND_FLASH))) {

		ret = smem_getpart(part_name, &start_block, &size_block);
		if (ret) {
#ifdef IPQ_UBI_VOL_WRITE_SUPPORT
			if (ubi_vol_present(part_name))
				return write_ubi_vol(part_name, load_addr,
								file_size);
#endif
			return retn;
		}

		offset = sfi->flash_block_size * start_block;
		part_size = sfi->flash_block_size * size_block;
#ifdef CONFIG_IPQ806X
		len = sizeof(layout_linux)/sizeof(layout_linux[0]);

		for (i = 0; i < len; i++) {

			if (!strncmp(layout_linux[i], part_name,
						SMEM_PTN_NAME_MAX)) {
				layout = "linux";
				break;
			}
		}

		if (i == len )
			layout = "sbl";
#endif

#ifdef CONFIG_QCA_MMC
	} else if (sfi->flash_type == SMEM_BOOT_MMC_FLASH ||
		sfi->flash_type == SMEM_BOOT_NO_FLASH) {

		blk_dev = mmc_get_dev(mmc_host.dev_num);
		if (blk_dev != NULL) {

			flash_type = SMEM_BOOT_MMC_FLASH;
			if (strncmp(GPT_PART_NAME,
					(const char *)part_name,
					sizeof(GPT_PART_NAME))  == 0) {
				file_size = file_size / blk_dev->blksz;
				offset = 0;
				part_size = (ulong) file_size;
			}
			else if (strncmp(GPT_BACKUP_PART_NAME,
					(const char *)part_name,
					sizeof(GPT_BACKUP_PART_NAME)) == 0) {
				file_size = file_size / blk_dev->blksz;
				offset = (ulong) blk_dev->lba - file_size;
				part_size = (ulong) file_size;
			}
			else
			{
				ret = get_partition_info_efi_by_name(blk_dev,
				part_name, &disk_info);
				if (ret)
					return retn;

				offset = (ulong)disk_info.start;
				part_size = (ulong)disk_info.size;
			}
		}
#endif
	} else if (sfi->flash_type == SMEM_BOOT_SPI_FLASH) {

		if (get_which_flash_param(part_name)) {

			/* NOR + NAND*/
			flash_type = SMEM_BOOT_NAND_FLASH;
			ret = getpart_offset_size(part_name, &offset, &part_size);
			if (ret)
				return retn;

		} else if (((sfi->flash_secondary_type == SMEM_BOOT_NAND_FLASH)||
				(sfi->flash_secondary_type == SMEM_BOOT_QSPI_NAND_FLASH))
				&& (strncmp(part_name, "rootfs", 6) == 0)) {

			flash_type = sfi->flash_secondary_type;

			if (sfi->rootfs.offset == 0xBAD0FF5E) {
				if (smem_bootconfig_info() == 0)
					active_part = get_rootfs_active_partition();

				offset = (ulong) active_part * IPQ_NAND_ROOTFS_SIZE;
				part_size = (ulong) IPQ_NAND_ROOTFS_SIZE;
			}

#ifdef CONFIG_QCA_MMC
		} else if ((smem_getpart(part_name, &start_block, &size_block)
				== -ENOENT) && (sfi->rootfs.offset == 0xBAD0FF5E)){

			/* NOR + EMMC */
			flash_type = SMEM_BOOT_MMC_FLASH;

			blk_dev = mmc_get_dev(mmc_host.dev_num);
			if (blk_dev != NULL) {

				if (strncmp(GPT_PART_NAME,
					(const char *)part_name,
					sizeof(GPT_PART_NAME))  == 0) {
					file_size = file_size / blk_dev->blksz;
					offset = 0;
					part_size = (ulong) file_size;
				}
				else if (strncmp(GPT_BACKUP_PART_NAME,
					(const char *)part_name,
					sizeof(GPT_BACKUP_PART_NAME)) == 0) {
					file_size = file_size / blk_dev->blksz;
					offset = (ulong) blk_dev->lba - file_size;
					part_size = (ulong) file_size;
				}
				else
				{
					ret = get_partition_info_efi_by_name(blk_dev,
							part_name, &disk_info);
					if (ret)
						return retn;

					offset = (ulong)disk_info.start;
					part_size = (ulong)disk_info.size;
				}
			}
#endif
		} else {

			ret = smem_getpart(part_name, &start_block,
							&size_block);
			if (ret) {
#ifdef IPQ_UBI_VOL_WRITE_SUPPORT
				if (ubi_vol_present(part_name))
					return write_ubi_vol(part_name,
						load_addr, file_size);
#endif
				return retn;
			}

			offset = sfi->flash_block_size * start_block;
			part_size = sfi->flash_block_size * size_block;
		}
	}

	if (flash_cmd) {
#ifdef CONFIG_CMD_NAND
		if (((flash_type == SMEM_BOOT_NAND_FLASH) ||
			(flash_type == SMEM_BOOT_QSPI_NAND_FLASH))) {

			adj_size = file_size % nand->writesize;
			if (adj_size)
				file_size = file_size + (nand->writesize - adj_size);
		}
#endif
		if (flash_type == SMEM_BOOT_MMC_FLASH) {

			if (disk_info.blksz) {
				file_size = file_size / disk_info.blksz;
				adj_size = file_size_cpy % disk_info.blksz;
				if (adj_size)
					file_size = file_size + 1;
			}
		}

		if (file_size > part_size) {
			printf("Image size is greater than partition memory\n");
			return CMD_RET_FAILURE;
		}

		ret = write_to_flash(flash_type, load_addr, offset, part_size,
							file_size, layout);
	} else
		ret = fl_erase(flash_type, offset, part_size, layout);

return ret;
}

#ifdef CONFIG_IPQ_MIBIB_RELOAD
static int do_mibib_reload(cmd_tbl_t *cmdtp, int flag, int argc,
char * const argv[])
{
	uint32_t load_addr, file_size;
	uint32_t page_size;
	uint8_t flash_type;
	struct header* mibib_hdr;
	qca_smem_flash_info_t *sfi = &qca_smem_flash_info;

	if (argc == 5) {
		flash_type = simple_strtoul(argv[1], NULL, 16);
		page_size = simple_strtoul(argv[2], NULL, 16);
		sfi->flash_block_size = simple_strtoul(argv[3], NULL, 16);
		sfi->flash_density = simple_strtoul(argv[4], NULL, 16);
		load_addr = getenv_ulong("fileaddr", 16, 0);
		file_size = getenv_ulong("filesize", 16, 0);
	} else
		return CMD_RET_USAGE;

	if (flash_type > 1) {
		printf("Invalid flash type \n");
		return CMD_RET_FAILURE;
	}

	if (file_size < 2 * page_size) {
		printf("Invalid filesize \n");
		return CMD_RET_FAILURE;
	}

	if (flash_type == 0) {
		/*NAND*/
#ifdef CONFIG_QPIC_SERIAL
		sfi->flash_type = SMEM_BOOT_QSPI_NAND_FLASH;
#else
		sfi->flash_type = SMEM_BOOT_NAND_FLASH;
#endif
	} else {
		/* NOR*/
		sfi->flash_type = SMEM_BOOT_SPI_FLASH;
	}

	mibib_hdr = (struct header*) load_addr;
	if (mibib_hdr->magic[0] == HEADER_MAGIC1 &&
		mibib_hdr->magic[1] == HEADER_MAGIC2 &&
		mibib_hdr->version == HEADER_VERSION) {

		load_addr += page_size;
	}
	else {
		printf("Header magic/version is invalid\n");
		return CMD_RET_FAILURE;
	}

	if (mibib_ptable_init((unsigned int*) load_addr)) {
		printf("Table magic is invalid\n");
		return CMD_RET_FAILURE;
	}

	return CMD_RET_SUCCESS;
}
#endif

#ifdef CONFIG_IPQ_XTRACT_N_FLASH
void print_fl_msg(char *fname, bool started, int ret)
{
	printf("######################################## ");
	printf("Flashing %s %s\n", fname,
			started ? "Started" : ret ? "Failed" : "Done");
}

static int do_xtract_n_flash(cmd_tbl_t *cmdtp, int flag, int argc,
char * const argv[])
{
	char runcmd[256], fname_stripped[32];
	char *file_name, *part_name;
	uint32_t load_addr, verbose;
	int ret = CMD_RET_SUCCESS;

	if (argc < 4)
		return CMD_RET_USAGE;

	verbose = getenv_ulong("verbose", 10, 0);
	load_addr = simple_strtoul(argv[1], NULL, 16);
	file_name = argv[2];
	part_name = argv[3];

	snprintf(fname_stripped , sizeof(fname_stripped),
		"%.*s:", strlen(file_name) - SHA1_SIG_LEN, file_name);

	if (verbose)
		print_fl_msg(fname_stripped, 1, ret);
	else
		setenv("stdout", "nulldev");

	snprintf(runcmd , sizeof(runcmd),
		"imxtract 0x%x %s && "
		"flash %s",
		load_addr, file_name,
		part_name);

	if (run_command(runcmd, 0) != CMD_RET_SUCCESS)
		ret = CMD_RET_FAILURE;

	if (verbose)
		print_fl_msg(fname_stripped, 0, ret);
	else {
		setenv("stdout", "serial");
		printf("Flashing %-30s %s\n", fname_stripped,
				ret ? "[ failed ]" : "[ done ]");
	}

	return ret;
}
#endif

#ifdef CONFIG_CMD_IPQ_FLASH_INIT
static int do_flash_init(cmd_tbl_t *cmdtp, int flag, int argc,
char * const argv[])
{
	int ret = 0;
	char *name = NULL;
	void *blk_dev = NULL;

	if (argc < 2)
		return CMD_RET_USAGE;

#ifdef CONFIG_QCA_MMC
	blk_dev = (void *)(mmc_get_dev(mmc_host.dev_num));

#endif
#ifdef CONFIG_QPIC_SERIAL
	int nand_dev = CONFIG_NAND_FLASH_INFO_IDX;
	name = nand_info[nand_dev].name;
#endif

	if (name || blk_dev) {
		printf("Either NAND or eMMC already initialized\n");
		return 0;
	}

#ifdef CONFIG_QCA_MMC
	if (!strncmp(argv[1], "mmc", 3)) {
		ret = do_mmc_init();
		if (!ret)
			ret = run_command("mmc info", 0);
	}
#endif
#ifdef CONFIG_QPIC_SERIAL
	if (!strncmp(argv[1], "nand", 4)) {
		do_nand_init();
		ret = (nand_info[nand_dev].name) ? 0: -1;
	}
#endif

	return ret;
}
#endif
U_BOOT_CMD(
	flash,       4,      0,      do_flash,
	"flash part_name \n"
	"\tflash part_name load_addr file_size \n",
	"flash the image at load_addr, given file_size in hex\n"
);

U_BOOT_CMD(
	flasherase,       4,      0,      do_flash,
	"flerase part_name \n",
	"erases on flash the given partition \n"
);

#ifdef CONFIG_IPQ_MIBIB_RELOAD
U_BOOT_CMD(
	mibib_reload,       5,      0,      do_mibib_reload,
	"mibib_reload fl_type pg_size blk_size chip_size\n",
	"reloads the smem partition info from mibib \n"
);
#endif

#ifdef CONFIG_IPQ_XTRACT_N_FLASH
U_BOOT_CMD(
	xtract_n_flash,       4,      0,      do_xtract_n_flash,
	"xtract_n_flash addr filename partname \n",
	"xtract the image and flash \n"
);
#endif

#ifdef CONFIG_CMD_IPQ_FLASH_INIT
U_BOOT_CMD(
	flashinit,       2,      0,      do_flash_init,
	"flashinit nand/mmc \n",
	"Init the flash \n"
);
#endif
