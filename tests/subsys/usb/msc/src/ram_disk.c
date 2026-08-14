/*
 * SPDX-FileCopyrightText: Copyright 2026 Renesas, Embedd
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/usb/usbd.h>
#include <zephyr/usb/class/usbd_msc.h>
#include <zephyr/fs/fs.h>
#include <zephyr/ztest.h>

#include <ff.h>

/*
 * The test case simulates up to two units using RAM.
 *
 * `USBD_DEFINE_MSC_LUN(id, ...)`'s LUN order (and therefore its device-side bLUN / host-side
 * "USB0_<n>" index) is the alphabetical order of the resulting `usbd_msc_lun_##id` symbol
 * (linker SORT_BY_NAME on the `usbd_msc_lun` iterable section), not declaration order. "ram0"
 * and "ram1" sort as LUN 0 and LUN 1 respectively.
 */
#if DT_NODE_HAS_STATUS(DT_NODELABEL(ramdisk0), okay)
USBD_DEFINE_MSC_LUN(ram0, "RAM0", "Zephyr", "RAMDisk", "0.00");
static struct fs_mount_t fs_mnt0;
static FATFS fat_fs0;
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(ramdisk1), okay)
USBD_DEFINE_MSC_LUN(ram1, "RAM1", "Zephyr", "RAMDisk", "0.00");
static struct fs_mount_t fs_mnt1;
static FATFS fat_fs1;
#endif

/**
 * @brief Setup a local FAT filesystem
 *
 * @param mnt_point Mount point string
 * @param fs_mnt    Mount point data
 * @param fat_fs    FAT file system data
 */
static void setup_disk(char const *mnt_point, struct fs_mount_t *fs_mnt, FATFS *fat_fs)
{
	struct fs_mount_t *mp = fs_mnt;
	struct fs_dir_t dir = {0};
	int result = 0;

	fs_dir_t_init(&dir);

	mp->type = FS_FATFS;
	mp->fs_data = fat_fs;
	mp->mnt_point = mnt_point;

	result = fs_mount(mp);
	zassert_ok(result, "Failed to mount filesystem: %i", result);
}

void ram_disk_setup(void)
{
#if DT_NODE_HAS_STATUS(DT_NODELABEL(ramdisk0), okay)
	setup_disk("/RAM0:", &fs_mnt0, &fat_fs0);
#endif
#if DT_NODE_HAS_STATUS(DT_NODELABEL(ramdisk1), okay)
	setup_disk("/RAM1:", &fs_mnt1, &fat_fs1);
#endif
}
