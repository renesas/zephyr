/*
 * SPDX-FileCopyrightText: Copyright 2026 Renesas, Embedd
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/ztest.h>
#include <zephyr/usb/usbd.h>
#include <zephyr/input/input.h>
#if DT_NODE_HAS_STATUS(DT_NODELABEL(ramdisk0), okay) ||                                            \
	DT_NODE_HAS_STATUS(DT_NODELABEL(ramdisk1), okay)
#include <sample_usbd.h>
#endif
#include <zephyr/usb/class/usbd_msc.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/storage/disk_access.h>
#include <zephyr/logging/log.h>
#include <zephyr/fs/fs.h>
#include <zephyr/ztest.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(test, LOG_LEVEL_INF);

#include <ff.h>
#include "usbh_desc.h"
#include "usbh_device.h"
#include "ram_disk.h"
#include "fault_disk.h"

/* USB Host controller */
USBH_CONTROLLER_DEFINE(test_uhs_ctx, DEVICE_DT_GET(DT_NODELABEL(zephyr_uhc0)));

/* File system test parameters */
#define SOME_FILE_NAME    "file"
#define SOME_DIR_NAME     "some"
#define SOME_REQUIRED_LEN MAX(sizeof(SOME_FILE_NAME), sizeof(SOME_DIR_NAME))
#define NUM_FILES         5

/* Test up to two logical units */
#define DISK_DRIVE_NAME_0 "USB0_0"
#define DISK_MOUNT_PT_0   "/" DISK_DRIVE_NAME_0 ":"

#if CONFIG_TEST_NUM_LUN > 1
#define DISK_DRIVE_NAME_1 "USB0_1"
#define DISK_MOUNT_PT_1   "/" DISK_DRIVE_NAME_1 ":"
#endif

#if CONFIG_TEST_NUM_LUN > 2
/* Third logical unit, backed by the fault-injectable disk */
#define DISK_DRIVE_NAME_FAULT "USB0_2"
#endif

/* USB Host controller context pointer for convenience */
struct usbh_context *const uhs_ctx = &test_uhs_ctx;
#if DT_NODE_HAS_STATUS(DT_NODELABEL(ramdisk0), okay) ||                                            \
	DT_NODE_HAS_STATUS(DT_NODELABEL(ramdisk1), okay)
/* USB Device controller context pointer for convenience */
static struct usbd_context *test_usbd = NULL;
#endif

/* Test data to be written to files */
static uint8_t const lorem_ipsum_buffer[] =
	"Lorem ipsum dolor sit amet, consectetur adipiscing elit. Pellentesque eget nunc vel "
	"mauris faucibus iaculis. Phasellus finibus porttitor arcu. "
	"Proin condimentum dignissim lectus, eu elementum eros hendrerit vitae. Phasellus id "
	"luctus sapien. Donec dapibus justo a urna sollicitudin, nec "
	"dapibus velit sodales. Duis posuere tellus in venenatis gravida. Curabitur vestibulum "
	"efficitur finibus. Curabitur elementum non sem imperdiet "
	"feugiat. Vivamus in orci tortor. Cras aliquam facilisis nisi ut suscipit. Phasellus sem "
	"purus, tincidunt quis lacinia efficitur, ultrices "
	"vehicula diam. Nunc posuere scelerisque tellus, eget viverra mauris suscipit non. "
	"Praesent eros lorem, tincidunt eu porttitor et, porta non "
	"ipsum. Sed ultricies purus in neque luctus varius. Nulla enim risus, posuere et molestie "
	"a, congue sed est. Nunc egestas laoreet sapien ut "
	"sagittis. Aliquam efficitur velit a augue maximus scelerisque. Donec euismod est vel odio "
	"vestibulum laoreet. Cras gravida luctus volutpat. "
	"Pellentesque eleifend non lorem vel semper. Nullam gravida dolor non rutrum interdum. "
	"Fusce vel lectus fringilla, varius lectus non, condimentum "
	"tortor. Ut ac justo ut tellus semper laoreet. Curabitur placerat, magna ac consequat "
	"scelerisque, nunc ex vulputate purus, a venenatis lectus "
	"nisi at dui. Vivamus luctus tellus massa. Vivamus sagittis, ex a ultrices ullamcorper, "
	"ante lacus pulvinar felis, a tempus nunc nulla eu massa. "
	"Proin elementum ornare ligula, id rutrum eros commodo at. Sed dignissim eros et molestie "
	"lacinia. Praesent ac tempus erat. Pellentesque vulputate "
	"ligula ex, vitae varius justo efficitur at. Lorem ipsum dolor sit amet, consectetur "
	"adipiscing elit. Vestibulum ac lacus sodales, vulputate purus "
	"a, sagittis dolor. Ut ut lacus faucibus massa interdum fringilla. Interdum et malesuada "
	"fames ac ante ipsum primis in faucibus. Integer "
	"ullamcorper dignissim orci. Aliquam erat volutpat.";

static void *suite_setup(void)
{
	int result = 0;

	result = usbh_init(uhs_ctx);
	zassert_ok(result, "Failed to initialize USB host");

	result = usbh_enable(uhs_ctx);
	zassert_ok(result, "Failed to enable USB host");
	k_msleep(500);

	result = uhc_bus_resume(uhs_ctx->dev);
	zassert_ok(result, "Failed to signal bus resume");
	k_msleep(500);

	result = uhc_bus_reset(uhs_ctx->dev);
	zassert_ok(result, "Failed to signal bus reset");

	result = uhc_sof_enable(uhs_ctx->dev);
	zassert_ok(result, "Failed to enable SoF generator");

#if DT_NODE_HAS_STATUS(DT_NODELABEL(ramdisk0), okay) ||                                            \
	DT_NODE_HAS_STATUS(DT_NODELABEL(ramdisk1), okay)
	ram_disk_setup();
	fault_disk_setup();

	test_usbd = sample_usbd_setup_device(NULL);
	zassert_not_null(test_usbd, "Failed to setup USB device");

	result = usbd_init(test_usbd);
	zassert_ok(result, "Failed to initialize device support");

	result = usbd_enable(test_usbd);
	zassert_ok(result, "Failed to enable device support");

#endif
	/* Allow the host time to reset the device. */
	k_msleep(1000);

	return NULL;
}

static void suite_shutdown(void *f)
{
	int result = 0;

#if DT_NODE_HAS_STATUS(DT_NODELABEL(ramdisk0), okay) ||                                            \
	DT_NODE_HAS_STATUS(DT_NODELABEL(ramdisk1), okay)
	result = usbd_disable(test_usbd);
	zassert_ok(result, "Failed to disable device support");

	result = usbd_shutdown(test_usbd);
	zassert_ok(result, "Failed to shutdown device support");
#endif

	result = usbh_disable(uhs_ctx);
	zassert_ok(result, "Failed to disable USB host");

	result = usbh_shutdown(uhs_ctx);
	zassert_ok(result, "Failed to shutdown host support");
}

ZTEST_SUITE(usbh_msc_suite, NULL, suite_setup, NULL, NULL, suite_shutdown);

/*
 * Initialize a disk
 */
static void initialize_disk(const char *name, const char *mount_point, struct fs_mount_t *mp)
{
	int result = 0;
	unsigned int attempts = 0;

	do {
		result = disk_access_ioctl(name, DISK_IOCTL_CTRL_INIT, NULL);
		/* The disk may not be mounted yet, give it some time */
		if (result != 0) {
			attempts++;
			zassert_true(attempts < 10, "Timed out waiting for disk");
			k_msleep(1000);
			continue;
		}
	} while (result != 0);

	zassert_equal(result, 0);

	mp->mnt_point = mount_point;

	result = fs_mount(mp);
	zassert_ok(result, "Unable to mount");
}

/*
 * Check unit metadata
 */
static void check_metadata(const char *name, uint32_t block_count, uint32_t block_size)
{
	uint64_t memory_size_mb = 0u;
	uint32_t actual_block_count = 0u;
	uint32_t actual_block_size = 0u;
	uint32_t actual_erase_block_size = 0u;
	int result = 0;

	/* IOCTL not supported for MSC devices */
	result = disk_access_ioctl(name, DISK_IOCTL_GET_ERASE_BLOCK_SZ, &actual_erase_block_size);
	zassert_equal(-ENOTSUP, result, "Erase should not be supported for MSC devices");

	result = disk_access_ioctl(name, DISK_IOCTL_GET_SECTOR_COUNT, &actual_block_count);
	zassert_ok(result, "Unable to get sector count");
	zassert_equal(actual_block_count, block_count, "Wrong block count: expected %lu, found %lu",
		      block_count, actual_block_count);

	result = disk_access_ioctl(name, DISK_IOCTL_GET_SECTOR_SIZE, &actual_block_size);
	zassert_ok(result, "Unable to get sector size");
	zassert_equal(actual_block_size, block_size, "Wrong block size: expected %lu, found %lu",
		      block_size, actual_block_size);

	zassert_equal(memory_size_mb,
		      (uint64_t)(actual_block_count * actual_block_size) / 1000000ul);
}

/*
 * Create a bunch of files to be later read
 */
static void write_files(const char *base_path)
{
	char path[128] = {0};
	struct fs_file_t file = {0};
	int result = 0;
	size_t total_length = strlen(lorem_ipsum_buffer);

	fs_file_t_init(&file);

	snprintf(path, sizeof(path), "%s/%s", base_path, SOME_DIR_NAME);
	result = fs_mkdir(path);
	zassert_equal(result == 0 || result == -EEXIST, 1, "Failed to create dir");

	for (size_t i = 0; i < NUM_FILES; i++) {
		snprintf(path, sizeof(path), "%s/%s/%s%zu.txt", base_path, SOME_DIR_NAME,
			 SOME_FILE_NAME, i);

		result = fs_open(&file, path, FS_O_CREATE | FS_O_WRITE | FS_O_TRUNC);
		zassert_ok(result, "Failed to create file");

		size_t written = 0;
		while (written < total_length) {
			int size = total_length - written;
			int result = fs_write(&file, &lorem_ipsum_buffer[written], size);
			zassert_true(result > 0, "Failed to write");
			written += result;
		}
		result = fs_close(&file);
		zassert_ok(result, "Failed to close file");

		result = fs_open(&file, path, FS_O_READ);
		zassert_ok(result, "Failed to reopen file");

		int length = 0;
		uint8_t buffer[65] = {};
		size_t read = 0;
		/* When reading leave 1 byte out for the terminating zero */
		while ((length = fs_read(&file, buffer, sizeof(buffer) - 1)) > 0) {
			zassert_equal(memcmp(&lorem_ipsum_buffer[read], buffer, length), 0,
				      "Read does not match!");
			read += length;
			memset(buffer, 0, sizeof(buffer));
		}
		fs_close(&file);
	}
}

/*
 * Verify the content of the previously created files
 */
static void check_files(const char *path)
{
	int result = 0;
	struct fs_dir_t dirp = {0};
	static struct fs_dirent entry = {0};
	int count = 0;
	uint32_t map = 0;
	size_t total_length = strlen(lorem_ipsum_buffer);

	fs_dir_t_init(&dirp);

	/* Verify fs_opendir() */
	result = fs_opendir(&dirp, path);
	zassert_ok(result, "Error opening dir");

	for (;;) {
		/* Verify fs_readdir() */
		result = fs_readdir(&dirp, &entry);

		/* entry.name[0] == 0 means end-of-dir */
		if (result || entry.name[0] == 0) {
			break;
		}

		if (entry.type == FS_DIR_ENTRY_FILE) {
			int file_index = 0;
			result = sscanf(entry.name, SOME_FILE_NAME "%i.txt", &file_index);
			map |= 1 << file_index;

			zassert_equal(result, 1, "Wrong file name");
			zassert_equal(entry.size, total_length, "Wrong file size");
		}
		count++;
	}

	zassert_equal(map, (1 << NUM_FILES) - 1, "Wrong file names");
	zassert_equal(count, NUM_FILES, "Wrong number of files");

	/* Verify fs_closedir() */
	result = fs_closedir(&dirp);
	zassert_ok(result, "Error closing dir");
}

ZTEST(usbh_msc_suite, test_usbh_msc_lun_1)
{
	static FATFS fat_fs = {0};
	static struct fs_mount_t mp = {
		.type = FS_FATFS,
		.fs_data = &fat_fs,
	};

	initialize_disk(DISK_DRIVE_NAME_0, DISK_MOUNT_PT_0, &mp);
	check_metadata(DISK_DRIVE_NAME_0, 1024, 512);
	write_files(DISK_MOUNT_PT_0);
	check_files(DISK_MOUNT_PT_0 "/" SOME_DIR_NAME);
	fs_unmount(&mp);
}

#if CONFIG_TEST_NUM_LUN > 1
ZTEST(usbh_msc_suite, test_usbh_msc_lun_2)
{
	static FATFS fat_fs = {0};
	static struct fs_mount_t mp = {
		.type = FS_FATFS,
		.fs_data = &fat_fs,
	};

	initialize_disk(DISK_DRIVE_NAME_1, DISK_MOUNT_PT_1, &mp);
	check_metadata(DISK_DRIVE_NAME_1, 512, 1024);
	write_files(DISK_MOUNT_PT_1);
	check_files(DISK_MOUNT_PT_1 "/" SOME_DIR_NAME);
	fs_unmount(&mp);
}
#endif

#if CONFIG_TEST_NUM_LUN > 2
/*
 * The following tests exercise error paths of the MSC driver's disk_access API, by making the
 * emulated device (the fault-injectable disk backing the third LUN) give an invalid or
 * unexpected response to the host's request.
 */

/*
 * Wait for the disk to become ready, without mounting a filesystem on it.
 */
static void wait_disk_ready(const char *name)
{
	int result = 0;
	unsigned int attempts = 0;

	do {
		result = disk_access_ioctl(name, DISK_IOCTL_CTRL_INIT, NULL);
		if (result != 0) {
			attempts++;
			zassert_true(attempts < 10, "Timed out waiting for disk");
			k_msleep(1000);
			continue;
		}
	} while (result != 0);
}

ZTEST(usbh_msc_suite, test_usbh_msc_fault_disk_normal_operation)
{
	uint8_t write_buffer[512];
	uint8_t read_buffer[512] = {0u};
	int result = 0;

	fault_disk_reset();
	wait_disk_ready(DISK_DRIVE_NAME_FAULT);

	memset(write_buffer, 0xA5, sizeof(write_buffer));

	result = disk_access_write(DISK_DRIVE_NAME_FAULT, write_buffer, 0, 1);
	zassert_ok(result, "Unable to write");

	result = disk_access_read(DISK_DRIVE_NAME_FAULT, read_buffer, 0, 1);
	zassert_ok(result, "Unable to read");

	zassert_mem_equal(write_buffer, read_buffer, sizeof(write_buffer), "Wrong data read back");
}

ZTEST(usbh_msc_suite, test_usbh_msc_fault_disk_oversized_transfer)
{
	uint8_t buffer[512];
	uint32_t sector_count = 0u;
	int result = 0;

	fault_disk_reset();
	wait_disk_ready(DISK_DRIVE_NAME_FAULT);

	result = disk_access_ioctl(DISK_DRIVE_NAME_FAULT, DISK_IOCTL_GET_SECTOR_COUNT,
				   &sector_count);
	zassert_ok(result, "Unable to get sector count");

	/* Reading or writing past the end of the disk should be rejected */
	result = disk_access_read(DISK_DRIVE_NAME_FAULT, buffer, sector_count, 1);
	zassert_equal(-EINVAL, result, "An out-of-range read should be rejected");

	result = disk_access_write(DISK_DRIVE_NAME_FAULT, buffer, sector_count, 1);
	zassert_equal(-EINVAL, result, "An out-of-range write should be rejected");

	/* More sectors than fit in a uint16_t should also be rejected */
	result = disk_access_read(DISK_DRIVE_NAME_FAULT, buffer, 0, 0x10000);
	zassert_equal(-EINVAL, result, "A read of more than 65535 sectors should be rejected");

	result = disk_access_write(DISK_DRIVE_NAME_FAULT, buffer, 0, 0x10000);
	zassert_equal(-EINVAL, result, "A write of more than 65535 sectors should be rejected");
}

ZTEST(usbh_msc_suite, test_usbh_msc_fault_disk_erase_not_supported)
{
	int result = disk_access_erase(DISK_DRIVE_NAME_FAULT, 0, 1, DISK_ACCESS_ERASE_PHYSICAL);

	zassert_equal(-ENOTSUP, result, "Erase should not be supported for MSC devices");
}

ZTEST(usbh_msc_suite, test_usbh_msc_fault_disk_write_error)
{
	uint8_t buffer[512] = {0};
	int result = 0;

	fault_disk_reset();
	wait_disk_ready(DISK_DRIVE_NAME_FAULT);

	fault_disk_set_write_error(true);

	result = disk_access_write(DISK_DRIVE_NAME_FAULT, buffer, 0, 1);
	zassert_true(result != 0, "A write error from the device should have surfaced");

	fault_disk_set_write_error(false);

	result = disk_access_status(DISK_DRIVE_NAME_FAULT);
	zassert_equal(DISK_STATUS_OK, result,
		      "A transient write error should not disable the whole unit");

	result = disk_access_write(DISK_DRIVE_NAME_FAULT, buffer, 0, 1);
	zassert_ok(result, "Unable to write after clearing the fault");
}

ZTEST(usbh_msc_suite, test_usbh_msc_fault_disk_read_error)
{
	uint8_t buffer[512] = {0};
	int result = 0;

	fault_disk_reset();
	wait_disk_ready(DISK_DRIVE_NAME_FAULT);

	fault_disk_set_read_error(true);

	result = disk_access_read(DISK_DRIVE_NAME_FAULT, buffer, 0, 1);
	zassert_true(result != 0, "A read error from the device should be surfaced");

	fault_disk_set_read_error(false);

	result = disk_access_status(DISK_DRIVE_NAME_FAULT);
	zassert_equal(DISK_STATUS_OK, result,
		      "A transient read error should not disable the whole unit");

	result = disk_access_read(DISK_DRIVE_NAME_FAULT, buffer, 0, 1);
	zassert_ok(result, "Unable to read after clearing the fault");
}

ZTEST(usbh_msc_suite, test_usbh_msc_fault_disk_medium_removed_and_reinserted)
{
	uint8_t buffer[512] = {0};
	int result = 0;

	fault_disk_reset();
	wait_disk_ready(DISK_DRIVE_NAME_FAULT);

	/* Simulate the medium being pulled */
	fault_disk_set_status_override(DISK_STATUS_NOMEDIA);

	result = disk_access_read(DISK_DRIVE_NAME_FAULT, buffer, 0, 1);
	zassert_equal(-ENOMEDIUM, result, "A missing medium should be reported");

	result = disk_access_status(DISK_DRIVE_NAME_FAULT);
	zassert_equal(DISK_STATUS_NOMEDIA, result, "Disk status should reflect the missing medium");

	/* Simulate the medium being reinserted */
	fault_disk_set_status_override(-1);

	result = disk_access_ioctl(DISK_DRIVE_NAME_FAULT, DISK_IOCTL_CTRL_DEINIT, NULL);
	zassert_ok(result, "Unable to deinitialize the disk");

	result = disk_access_ioctl(DISK_DRIVE_NAME_FAULT, DISK_IOCTL_CTRL_INIT, NULL);
	zassert_ok(result, "Unable to reinitialize the disk after reinsertion");

	result = disk_access_read(DISK_DRIVE_NAME_FAULT, buffer, 0, 1);
	zassert_ok(result, "Unable to read after the medium was reinserted");
}
#endif /* CONFIG_TEST_NUM_LUN > 2 */
