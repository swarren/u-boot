/*
 * Copyright (C) 2015 Stephen Warren <swarren@wwwdotorg.org>
 * (GPL-2.0+)
 *
 * Portions taken from U-Boot's previous FAT implementation
 * R/O (V)FAT 12/16/32 filesystem implementation by Marcus Sundberg
 * 2002-07-28 - rjones@nexus-tech.net - ported to ppcboot v1.1.6
 * 2003-03-10 - kharris@nexus-tech.net - ported to uboot
 *
 * Small portions taken from Barebox v2015.07.0
 * Copyright (c) 2007 Sascha Hauer <s.hauer@pengutronix.de>, Pengutronix
 * (GPL-2.0)
 *
 * SPDX-License-Identifier:     GPL-2.0+ GPL-2.0
 */

#include <common.h>
#include <fat.h>
#include <linux/ctype.h>
#include <part.h>
#include "diskio.h"
#include "ff.h"

static block_dev_desc_t *fat_dev;
static disk_partition_t *fat_part;
static FATFS fat_ff_fs;

/* Functions called by ff.c */

DSTATUS disk_initialize(BYTE pdrv)
{
	return 0;
}

DSTATUS disk_status(BYTE pdrv)
{
	return 0;
}

DRESULT disk_read(BYTE pdrv, BYTE *buff, DWORD sector, UINT count)
{
	int ret;

	debug("%s(sector=%d, count=%u)\n", __func__, sector, count);

	ret = fat_dev->block_read(fat_dev->dev, fat_part->start + sector,
				  count, buff);
	if (ret != count)
		return RES_ERROR;

	return RES_OK;
}

DRESULT disk_write(BYTE pdrv, const BYTE *buff, DWORD sector, UINT count)
{
	int ret;

	debug("%s(sector=%d, count=%u)\n", __func__, sector, count);

	ret = fat_dev->block_write(fat_dev->dev, fat_part->start + sector,
				   count, buff);
	if (ret != count)
		return RES_ERROR;

	return RES_OK;
}

DRESULT disk_ioctl(BYTE pdrv, BYTE cmd, void *buff)
{
	debug("%s(cmd=%d)\n", __func__, (int)cmd);

	switch (cmd) {
	case CTRL_SYNC:
		return RES_OK;
	default:
		return RES_ERROR;
	}
}

/* From Barebox v2015.07.0 fs/fat/fat.c */
WCHAR ff_convert(WCHAR src, UINT dir)
{
	if (src <= 0x80)
		return src;
	else
		return '?';
}

/* From Barebox v2015.07.0 fs/fat/fat.c */
WCHAR ff_wtoupper(WCHAR chr)
{
	if (chr <= 0x80)
		return toupper(chr);
	else
		return '?';
}

/* Functions that call into ff.c */

int fat_set_blk_dev(block_dev_desc_t *dev, disk_partition_t *part)
{
	FRESULT ff_ret;

	debug("%s()\n", __func__);

	fat_dev = dev;
	fat_part = part;

	ff_ret = f_mount(&fat_ff_fs, "0:", 1);
	if (ff_ret != FR_OK) {
		debug("f_mount() failed: %d\n", ff_ret);
		fat_dev = NULL;
		fat_part = NULL;
		return -1;
	}

	debug("f_mount() succeeded\n");
	return 0;
}

int fat_register_device(block_dev_desc_t *dev_desc, int part_no)
{
	disk_partition_t info;

	/* First unregister any current FAT filesystem */
	fat_dev = NULL;
	fat_part = NULL;

	/* Read the partition table, if present */
	if (get_partition_info(dev_desc, part_no, &info)) {
		if (part_no != 0) {
			printf("** Partition %d not valid on device %d **\n",
			       part_no, dev_desc->dev);
			return -1;
		}

		info.start = 0;
		info.size = dev_desc->lba;
		info.blksz = dev_desc->blksz;
		info.name[0] = 0;
		info.type[0] = 0;
		info.bootable = 0;
#ifdef CONFIG_PARTITION_UUIDS
		info.uuid[0] = 0;
#endif
	}

	return fat_set_blk_dev(dev_desc, &info);
}

int file_fat_detectfs(void)
{
	FRESULT ff_ret;
	TCHAR label[12];
	DWORD vsn;

	debug("%s()\n", __func__);

	if (!fat_dev) {
		printf("No current device\n");
		return 1;
	}

#ifdef HAVE_BLOCK_DEVICE
	printf("  Device %d: ", fat_dev->dev);
	dev_print(fat_dev);
#endif

	ff_ret = f_getlabel("0:", label, &vsn);
	if (ff_ret != FR_OK) {
		debug("f_getlabel() failed: %d\n", ff_ret);
		return -1;
	}

	printf("Filesystem:    ");
	switch (fat_ff_fs.fs_type) {
	case FS_FAT12:
		puts("FAT12\n");
		break;
	case FS_FAT16:
		puts("FAT16\n");
		break;
	case FS_FAT32:
		puts("FAT32\n");
		break;
	default:
		puts("<<unknown>>\n");
		break;
	}

	printf("Volume label:  ");
	if (!label[0]) {
		puts("<<no label>>\n");
	} else {
		puts(label);
		puts("\n");
	}

	printf("Volume serial: %08x\n", vsn);

	return 0;
}

int fat_exists(const char *filename)
{
	loff_t size;
	int ret;

	debug("%s(filename=%s)\n", __func__, filename);

	ret = fat_size(filename, &size);
	if (ret)
		return 0;

	return 1;
}

int fat_size(const char *filename, loff_t *size)
{
	FRESULT ff_ret;
	FILINFO finfo;

	debug("%s(filename=%s)\n", __func__, filename);

	memset(&finfo, 0, sizeof(finfo));

	ff_ret = f_stat(filename, &finfo);
	if (ff_ret != FR_OK) {
		debug("f_stat() failed: %d\n", ff_ret);
		return -1;
	}

	*size = finfo.fsize;

	return 0;
}

int file_fat_ls(const char *dir)
{
	FRESULT ff_ret;
	DIR d;
	bool d_valid = false;
	FILINFO finfo;
	char lfname[_MAX_LFN + 1];
	TCHAR *fname;

	debug("%s()\n", __func__);

	memset(&finfo, 0, sizeof(finfo));
	finfo.lfname = lfname;
	finfo.lfsize = sizeof(lfname);

	ff_ret = f_opendir(&d, dir);
	if (ff_ret != FR_OK) {
		debug("f_opendir() failed: %d\n", ff_ret);
		goto err;
	}
	d_valid = true;

	for (;;) {
		ff_ret = f_readdir(&d, &finfo);
		if (ff_ret != FR_OK) {
			debug("f_readdir() failed: %d\n", ff_ret);
			goto err;
		}
		if (!finfo.fname[0])
			break;

		if (*finfo.lfname)
			fname = finfo.lfname;
		else
			fname = finfo.fname;

		if (finfo.fattrib & AM_DIR)
			printf("            %s/\n", fname);
		else
			printf(" %8u   %s\n", finfo.fsize, fname);
	}

	ff_ret = f_closedir(&d);
	if (ff_ret != FR_OK) {
		debug("f_closedir() failed: %d\n", ff_ret);
		goto err;
	}

	return 0;

err:
	if (d_valid)
		f_closedir(&d);
	printf("** Unable to read dir %s **\n", dir);
	return -1;
}

int fat_read_file(const char *filename, void *buf, loff_t offset, loff_t len,
		  loff_t *actread)
{
	FRESULT ff_ret;
	FIL fil;
	bool fil_valid = false;
	UINT ff_actread;

	debug("%s(filename=%s, offset=%d, len=%d)\n", __func__, filename,
	      (int)offset, (int)len);

	if (!len)
		len = -1;

	ff_ret = f_open(&fil, filename, FA_READ);
	if (ff_ret != FR_OK) {
		debug("f_open() failed: %d\n", ff_ret);
		goto err;
	}
	fil_valid = true;

	ff_ret = f_lseek(&fil, offset);
	if (ff_ret != FR_OK) {
		debug("f_lseek() failed: %d\n", ff_ret);
		goto err;
	}

	ff_ret = f_read(&fil, buf, len, &ff_actread);
	if (ff_ret != FR_OK) {
		debug("f_read() failed: %d\n", ff_ret);
		goto err;
	}
	debug("f_read() read %u bytes\n", ff_actread);
	*actread = ff_actread;

	ff_ret = f_close(&fil);
	if (ff_ret != FR_OK) {
		debug("f_close() failed: %d\n", ff_ret);
		goto err;
	}

	return 0;

err:
	if (fil_valid)
		f_close(&fil);
	printf("** Unable to read file %s **\n", filename);
	return -1;
}

int file_fat_read(const char *filename, void *buffer, int maxsize)
{
	loff_t actread;
	int ret;

	ret = fat_read_file(filename, buffer, 0, maxsize, &actread);
	if (ret)
		return ret;
	else
		return actread;
}

#ifdef CONFIG_FAT_WRITE
int file_fat_write(const char *filename, void *buf, loff_t offset, loff_t len,
		   loff_t *actwrite)
{
	FRESULT ff_ret;
	FIL fil;
	bool fil_valid = false;
	UINT ff_actwrite;

	debug("%s(filename=%s, offset=%d, len=%d)\n", __func__, filename,
	      (int)offset, (int)len);

	if (!len)
		len = -1;

	ff_ret = f_open(&fil, filename, FA_WRITE | FA_OPEN_ALWAYS);
	if (ff_ret != FR_OK) {
		debug("f_open() failed: %d\n", ff_ret);
		goto err;
	}
	fil_valid = true;

	ff_ret = f_lseek(&fil, offset);
	if (ff_ret != FR_OK) {
		debug("f_lseek() failed: %d\n", ff_ret);
		goto err;
	}

	ff_ret = f_write(&fil, buf, len, &ff_actwrite);
	if (ff_ret != FR_OK) {
		debug("f_write() failed: %d\n", ff_ret);
		goto err;
	}
	debug("f_read() read %u bytes\n", ff_actwrite);
	*actwrite = ff_actwrite;

	ff_ret = f_truncate(&fil);
	if (ff_ret != FR_OK) {
		debug("f_truncate() failed: %d\n", ff_ret);
		goto err;
	}

	ff_ret = f_close(&fil);
	if (ff_ret != FR_OK) {
		debug("f_close() failed: %d\n", ff_ret);
		goto err;
	}

	return 0;

err:
	if (fil_valid)
		f_close(&fil);
	printf("** Unable to write file %s **\n", filename);
	return -1;
}
#endif

void fat_close(void)
{
	debug("%s()\n", __func__);

	fat_dev = NULL;
	fat_part = NULL;
}
