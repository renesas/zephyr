/*
 * Copyright (c) 2026 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <../dts/common/mem.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/crc.h>
#include <errno.h>
#include <zephyr/console/console.h>

#define SPI_FLASH_NODE            DT_NODELABEL(ext_flash_ctrl)
#define INTERNAL_FLASH_NODE       DT_NODELABEL(mram_ctrl)
#define INTERN_FLASH_BASE_ADDRESS DT_REG_ADDR(DT_CHOSEN(zephyr_flash))

#define OSPI_BASE_ADDRESS        0x90000000
#define PRIMARY_INTERNAL_OFFSET  0x0
#define RECOVERY_EXTERNAL_OFFSET 0x0
#define BACKUP_EXTERNAL_OFFSET   0x70000

#define PRIMARY_INTERNAL_MAX_SIZE  DT_SIZE_K(1023)
#define RECOVERY_EXTERNAL_MAX_SIZE DT_SIZE_K(448)
#define BACKUP_EXTERNAL_MAX_SIZE   DT_SIZE_K(1023)

#define RX_CHUNK_SIZE 4096
#define RX_PROGRESS_STEP 4096

#define XMODEM_SOH 0x01
#define XMODEM_EOT 0x04
#define XMODEM_ACK 0x06
#define XMODEM_NAK 0x15
#define XMODEM_CAN 0x18
#define XMODEM_CRC 0x43
#define XMODEM_BLOCK_SIZE 128
#define XMODEM_MAX_RETRIES 16

typedef enum {
	FLASH_TARGET_INTERNAL = 1,
	FLASH_TARGET_EXTERNAL = 2,
} flash_target_t;

typedef enum option {
	OPTION_XMODEM_FLASH = 1,
	OPTION_DUMP_FLASH = 2,
} option_t;

/* -------------------------------------------------------------------------
 * UART helpers
 * ------------------------------------------------------------------------- */

static char *app_getline(void)
{
	static char line[64];
	size_t i = 0;

	memset(line, 0, sizeof(line));

	while (i < sizeof(line) - 1) {
		int c = console_getchar();

		if (c < 0) {
			continue;
		}

		if (c == '\r' || c == '\n') {
			printk("\r\n");
			break;
		}

		if (c == '\b' || c == 0x7f) {
			if (i > 0) {
				i--;
				printk("\b \b");
			}
			continue;
		}

		if (isprint((unsigned char)c)) {
			line[i++] = (char)c;
			console_putchar((char)c);
		}
	}

	line[i] = '\0';
	return line;
}

static int parse_u32(const char *str, uint32_t *value)
{
	char *end;
	unsigned long parsed;

	errno = 0;
	parsed = strtoul(str, &end, 0);

	while (*end != 0) {
		if (!isspace((unsigned char)*end)) {
			return -EINVAL;
		}
		end++;
	}

	if (errno != 0 || parsed > UINT32_MAX) {
		return -EINVAL;
	}

	*value = (uint32_t)parsed;
	return 0;
}

static int erase_target_range(const struct device *flash_dev, uint32_t flash_offset,
			      size_t write_len, flash_target_t target)
{
	const char *target_str = (target == FLASH_TARGET_INTERNAL) ? "internal" : "external";
	struct flash_pages_info start_page;
	struct flash_pages_info end_page;
	uint32_t erase_start;
	size_t erase_len;
	uint32_t last_offset;
	int rc;

	/* NOR flash (external) always requires explicit erase; MRAM (internal) does
	 * not. */
	if (target != FLASH_TARGET_EXTERNAL) {
		return 0;
	}

	rc = flash_get_page_info_by_offs(flash_dev, flash_offset, &start_page);
	if (rc != 0) {
		printk("Failed to get start page info: %d\r\n", rc);
		return rc;
	}

	if (flash_offset != start_page.start_offset) {
		printk("External flash offset must be erase-page aligned (0x%08lx).\r\n",
		       (unsigned long)start_page.start_offset);
		return -EINVAL;
	}

	last_offset = flash_offset + write_len - 1U;
	rc = flash_get_page_info_by_offs(flash_dev, last_offset, &end_page);
	if (rc != 0) {
		printk("Failed to get end page info: %d\r\n", rc);
		return rc;
	}

	erase_start = start_page.start_offset;
	erase_len = (end_page.start_offset + end_page.size) - erase_start;

	printk("Erasing %s flash at offset 0x%08x, size %zu...\r\n", target_str, erase_start,
	       erase_len);

	rc = flash_erase(flash_dev, erase_start, erase_len);
	if (rc != 0) {
		printk("Flash erase failed! rc=%d\r\n", rc);
	}

	return rc;
}

static int timed_read_byte(uint8_t *byte, k_timeout_t timeout)
{
	ssize_t ret;

	console_set_rx_timeout(timeout);
	ret = console_read(NULL, byte, 1);
	console_set_rx_timeout(K_FOREVER);

	return (ret == 1) ? 0 : -EAGAIN;
}

static int xmodem_read_exact(uint8_t *buf, size_t len, k_timeout_t timeout)
{
	size_t received = 0;

	console_set_rx_timeout(timeout);
	while (received < len) {
		ssize_t ret = console_read(NULL, &buf[received], len - received);

		if (ret <= 0) {
			console_set_rx_timeout(K_FOREVER);
			return -EAGAIN;
		}

		received += (size_t)ret;
	}
	console_set_rx_timeout(K_FOREVER);

	return 0;
}

static int xmodem_receive_to_flash(const struct device *flash_dev, uint32_t flash_offset,
				   size_t image_len, size_t write_bs)
{
	uint8_t packet[XMODEM_BLOCK_SIZE];
	uint8_t write_buf[XMODEM_BLOCK_SIZE];
	uint8_t block = 1;
	size_t payload_received = 0;
	size_t written = 0;
	size_t max_payload = ROUND_UP(image_len, XMODEM_BLOCK_SIZE);
	int retries = 0;

	printk("Start TeraTerm XMODEM transfer now (CRC mode).\r\n");

	while (true) {
		uint8_t c;
		int rc;

		if (payload_received == 0U) {
			console_putchar(XMODEM_CRC);
		}
		rc = timed_read_byte(&c, K_SECONDS(1));
		if (rc != 0) {
			console_putchar(payload_received == 0U ? XMODEM_CRC : XMODEM_NAK);
			if (++retries >= XMODEM_MAX_RETRIES) {
				printk("XMODEM timeout waiting for sender.\r\n");
				return -ETIMEDOUT;
			}
			continue;
		}

		if (c == XMODEM_EOT) {
			console_putchar(XMODEM_ACK);
			if (written < image_len) {
				printk("XMODEM ended early: got %zu expected %zu.\r\n", written,
				       image_len);
				return -EIO;
			}
			printk("\r\nXMODEM receive complete: %zu image bytes, %zu protocol bytes.\r\n",
			       written, payload_received);
			return 0;
		}

		if (c == XMODEM_CAN) {
			printk("XMODEM canceled by sender.\r\n");
			return -ECANCELED;
		}

		if (c != XMODEM_SOH) {
			console_putchar(XMODEM_NAK);
			if (++retries >= XMODEM_MAX_RETRIES) {
				printk("XMODEM sync failed.\r\n");
				return -EIO;
			}
			continue;
		}

		uint8_t block_num;
		uint8_t block_inv;
		uint8_t crc_hi;
		uint8_t crc_lo;

		if (timed_read_byte(&block_num, K_SECONDS(2)) != 0 ||
		    timed_read_byte(&block_inv, K_SECONDS(2)) != 0 ||
		    xmodem_read_exact(packet, sizeof(packet), K_SECONDS(2)) != 0 ||
		    timed_read_byte(&crc_hi, K_SECONDS(2)) != 0 ||
		    timed_read_byte(&crc_lo, K_SECONDS(2)) != 0) {
			console_putchar(XMODEM_NAK);
			if (++retries >= XMODEM_MAX_RETRIES) {
				printk("XMODEM packet timeout.\r\n");
				return -ETIMEDOUT;
			}
			continue;
		}

		uint16_t rx_crc = ((uint16_t)crc_hi << 8) | crc_lo;
		uint16_t calc_crc = crc16_itu_t(0x0000, packet, sizeof(packet));

		if ((uint8_t)(block_num + block_inv) != 0xFF || rx_crc != calc_crc) {
			console_putchar(XMODEM_NAK);
			if (++retries >= XMODEM_MAX_RETRIES) {
				printk("XMODEM packet validation failed.\r\n");
				return -EIO;
			}
			continue;
		}

		if (block_num == (uint8_t)(block - 1U)) {
			console_putchar(XMODEM_ACK);
			continue;
		}

		if (block_num != block) {
			console_putchar(XMODEM_NAK);
			if (++retries >= XMODEM_MAX_RETRIES) {
				printk("XMODEM unexpected block: got %u expected %u.\r\n", block_num, block);
				return -EIO;
			}
			continue;
		}

		if (payload_received >= max_payload) {
			console_putchar(XMODEM_CAN);
			console_putchar(XMODEM_CAN);
			printk("XMODEM received more data than expected.\r\n");
			return -EFBIG;
		}

		if (written < image_len) {
			const uint8_t *write_data = packet;
			size_t copy_len = MIN(sizeof(packet), image_len - written);
			size_t write_len = ROUND_UP(copy_len, write_bs);

			if (write_len != copy_len) {
				memcpy(write_buf, packet, copy_len);
				memset(&write_buf[copy_len], 0xFF, write_len - copy_len);
				write_data = write_buf;
			}

			rc = flash_write(flash_dev, flash_offset + written, write_data, write_len);
			if (rc != 0) {
				console_putchar(XMODEM_CAN);
				console_putchar(XMODEM_CAN);
				printk("Flash write failed at offset 0x%08x: %d\r\n",
				       flash_offset + written, rc);
				return rc;
			}

			written += copy_len;
		}

		payload_received += sizeof(packet);
		block++;
		retries = 0;
		console_putchar(XMODEM_ACK);

	}
}

static int receive_and_write_image_xmodem(const struct device *flash_dev, size_t image_len,
					  uint32_t flash_offset, flash_target_t target)
{
	const char *target_str = (target == FLASH_TARGET_INTERNAL) ? "internal" : "external";
	uint32_t flash_base =
		(target == FLASH_TARGET_INTERNAL) ? INTERN_FLASH_BASE_ADDRESS : OSPI_BASE_ADDRESS;
	size_t write_bs = flash_get_write_block_size(flash_dev);
	size_t write_len = ROUND_UP(image_len, write_bs);
	int rc;

	if ((flash_offset % write_bs) != 0U) {
		printk("Flash offset must be aligned to write block size %zu.\r\n", write_bs);
		return -EINVAL;
	}

	if ((XMODEM_BLOCK_SIZE % write_bs) != 0U) {
		printk("Flash write block size %zu is not compatible with %u byte XMODEM blocks.\r\n",
		       write_bs, XMODEM_BLOCK_SIZE);
		return -EINVAL;
	}

	rc = erase_target_range(flash_dev, flash_offset, write_len, target);
	if (rc != 0) {
		return rc;
	}

	printk("Writing XMODEM data to %s flash at address 0x%08x...\r\n", target_str,
	       flash_base + flash_offset);

	rc = xmodem_receive_to_flash(flash_dev, flash_offset, image_len, write_bs);
	if (rc != 0) {
		return rc;
	}

	printk("Flash write complete: %zu bytes requested, %zu bytes programmed.\r\n", image_len,
	       write_len);
	return 0;
}

static int dump_flash_data(const struct device *flash_dev, uint32_t flash_offset,
			   size_t len, flash_target_t target)
{
	const char *target_str = (target == FLASH_TARGET_INTERNAL) ? "internal" : "external";
	uint32_t flash_base =
		(target == FLASH_TARGET_INTERNAL) ? INTERN_FLASH_BASE_ADDRESS : OSPI_BASE_ADDRESS;
	uint8_t row[16];
	size_t dumped = 0;
	int rc;

	printk("Dumping %zu bytes from %s flash at address 0x%08x\r\n", len, target_str,
	       flash_base + flash_offset);

	while (dumped < len) {
		size_t row_len = MIN(sizeof(row), len - dumped);

		rc = flash_read(flash_dev, flash_offset + dumped, row, row_len);
		if (rc != 0) {
			printk("Flash read failed at offset 0x%08x: %d\r\n",
			       flash_offset + dumped, rc);
			return rc;
		}

		printk("0x%08x:", flash_base + flash_offset + dumped);
		for (size_t i = 0; i < row_len; i++) {
			printk(" %02x", row[i]);
		}
		printk("\r\n");

		dumped += row_len;
	}

	printk("Dump complete.\r\n");
	return 0;
}

/* -------------------------------------------------------------------------
 * main_loop()
 * ------------------------------------------------------------------------- */

static int main_loop(void)
{
	const struct device *console_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
	const struct device *ext_flash_dev = DEVICE_DT_GET(SPI_FLASH_NODE);
	const struct device *int_flash_dev = DEVICE_DT_GET(INTERNAL_FLASH_NODE);
	flash_target_t target;
	option_t option;
	const struct device *flash_dev;
	uint32_t flash_offset = 0;
	uint32_t image_len = 0;
	uint32_t dump_len = 0;
	uint64_t flash_size;
	char *line;

	k_sleep(K_MSEC(500));
	int rc = console_init();
	if (rc != 0) {
		printk("Console init failed: %d\r\n", rc);
		return rc;
	}

	if (!device_is_ready(console_dev)) {
		return -ENODEV;
	}

	if (!device_is_ready(ext_flash_dev)) {
		printk("External flash not ready.\r\n");
		return -ENODEV;
	}

	if (!device_is_ready(int_flash_dev)) {
		printk("Internal flash not ready.\r\n");
		return -ENODEV;
	}

	printk("Launching External Flash writter application on %s\n", CONFIG_BOARD);

	while (1) {
		printk("\r\n");
		printk("  1 - XMODEM receive and write external flash at address 0x%08x\r\n",
		       (OSPI_BASE_ADDRESS));
		printk("  2 - Dump flash data\r\n");
		printk("\r\n  Enter option: ");

		line = app_getline();
		option = (option_t)atoi(line);

		if (option == OPTION_XMODEM_FLASH) {
			target = FLASH_TARGET_EXTERNAL;
			flash_dev = ext_flash_dev;

			printk("Enter image size in bytes (decimal or 0x...): ");
			line = app_getline();
			rc = parse_u32(line, &image_len);
			if (rc != 0 || image_len == 0U) {
				printk("Invalid image size.\r\n");
				continue;
			}

			printk("Enter Flash offset (decimal or 0x...): ");
			line = app_getline();
			rc = parse_u32(line, &flash_offset);
			if (rc != 0) {
				printk("Invalid flash offset.\r\n");
				continue;
			}

			rc = flash_get_size(flash_dev, &flash_size);
			if (rc != 0) {
				printk("Failed to get flash size: %d\r\n", rc);
				continue;
			}

			size_t write_bs = flash_get_write_block_size(flash_dev);
			size_t write_len = ROUND_UP((size_t)image_len, write_bs);
			if (flash_offset >= flash_size || write_len > (flash_size - flash_offset)) {
				printk("Image does not fit: offset=0x%08x len=%u flash_size=%llu\r\n",
				       flash_offset, image_len, flash_size);
				continue;
			}

			rc = receive_and_write_image_xmodem(flash_dev, (size_t)image_len, flash_offset,
							  target);
			if (rc == 0) {
				printk("XMODEM update complete!\r\n");
			} else {
				printk("XMODEM update failed: %d\r\n", rc);
			}
			continue;
		}

		if (option == OPTION_DUMP_FLASH) {
			uint32_t dump_target;

			printk("Dump target (1 internal, 2 external): ");
			line = app_getline();
			rc = parse_u32(line, &dump_target);
			if (rc != 0) {
				printk("Invalid dump target.\r\n");
				continue;
			}

			switch (dump_target) {
			case FLASH_TARGET_INTERNAL:
				target = FLASH_TARGET_INTERNAL;
				flash_dev = int_flash_dev;
				break;
			case FLASH_TARGET_EXTERNAL:
				target = FLASH_TARGET_EXTERNAL;
				flash_dev = ext_flash_dev;
				break;
			default:
				printk("Invalid dump target: %u\r\n", dump_target);
				continue;
			}

			printk("Enter Flash offset (decimal or 0x...): ");
			line = app_getline();
			rc = parse_u32(line, &flash_offset);
			if (rc != 0) {
				printk("Invalid flash offset.\r\n");
				continue;
			}

			printk("Enter dump length in bytes (decimal or 0x...): ");
			line = app_getline();
			rc = parse_u32(line, &dump_len);
			if (rc != 0 || dump_len == 0U) {
				printk("Invalid dump length.\r\n");
				continue;
			}

			rc = flash_get_size(flash_dev, &flash_size);
			if (rc != 0) {
				printk("Failed to get flash size: %d\r\n", rc);
				continue;
			}

			if (flash_offset >= flash_size || dump_len > (flash_size - flash_offset)) {
				printk("Dump range does not fit: offset=0x%08x len=%u flash_size=%llu\r\n",
				       flash_offset, dump_len, flash_size);
				continue;
			}

			dump_flash_data(flash_dev, flash_offset, (size_t)dump_len, target);
			continue;
		}

		printk("Invalid selection: %d\r\n", option);
		continue;
	}

	return 0;
}

/* -------------------------------------------------------------------------
 * main
 * ------------------------------------------------------------------------- */

int main(void)
{
	main_loop();
	return 0;
}
