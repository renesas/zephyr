/*
 * Copyright (c) 2026 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/drivers/psi5/psi5.h>
#include <zephyr/kernel.h>
#include <zephyr/ztest.h>

#define PSI5_NODE          DT_ALIAS(psi5_0)
#define PSI5_CHANNEL       0U
#define PSI5_MAX_RX_BUFFER 1U
#define PSI5_RX_TIMEOUT    K_MSEC(100)
#define PSI5_NO_CB_WINDOW  K_MSEC(20)
#define PSI5_TX_TIMEOUT    K_MSEC(200)
#define PSI5_STOP_TIMEOUT  K_MSEC(200)

#define PSI5_EMU_DATA            0x1E7U
#define PSI5_EMU_RESTART_DATA    0x0ABU
#define PSI5_EMU_DATA_LEN        10U
#define PSI5_EMU_BAD_PARITY_DATA (0x3F3U | BIT(PSI5_EMU_DATA_LEN))
#define PSI5_EMU_BAD_PARITY_LEN  (PSI5_EMU_DATA_LEN + 1U)
#define PSI5_EMU_SMC_RESET_DATA  0x0F0U
#define PSI5_EMU_SMC_CRC         0x20U
#define PSI5_SMC_EXP_ID          0x55U
#define PSI5_SMC_EXP_DATA        0x3E1U
#define PSI5_SMC_TIMEOUT         K_MSEC(100)

#define PSI5_TX_DATA_FIT     0x3FU
#define PSI5_TX_DATA_OVER    0x40U
#define PSI5_TX_DATA_OVER_HI UINT64_C(0x100000000)

#define PSI5_SMC_SLOT_NODE DT_NODELABEL(psi50_ch0_rx_slot0)
#define PSI5_TEST_SMC      DT_PROP_OR(PSI5_SMC_SLOT_NODE, has_smc, 0)
#define PSI5_CH_NODE       DT_NODELABEL(psi50_ch0)
#define PSI5_TEST_ASYNC    DT_PROP_OR(PSI5_CH_NODE, async_mode, 0)

int psi5_emu_init(void);
int psi5_emu_send_frame(uint32_t data, uint8_t data_len, bool has_parity);
int psi5_emu_send_smc_pattern(uint8_t crc6);
int psi5_emu_send_frame_synced(uint32_t data, uint8_t data_len, bool has_parity);

static const struct device *const psi5_dev = DEVICE_DT_GET(PSI5_NODE);
static struct k_sem rx_sem;
static struct k_sem smc_sem;
static struct k_sem tx_sem;
static struct psi5_frame data_frame[PSI5_MAX_RX_BUFFER];
static struct psi5_frame serial_frame[PSI5_MAX_RX_BUFFER];

struct rx_result_data {
	uint32_t callback_count;
	uint32_t error_count;
	uint32_t num_frames;
	uint8_t channel;
};

struct smc_result_data {
	uint32_t callback_count;
	uint32_t error_count;
	uint32_t num_frames;
};

static volatile struct rx_result_data rx_result;
static volatile struct smc_result_data smc_result;
static volatile uint32_t tx_callback_count;
static volatile int tx_status[4];

static void reset_rx_result(void)
{
	rx_result = (struct rx_result_data){0};
	data_frame[0] = (struct psi5_frame){0};
	k_sem_reset(&rx_sem);
}

static void reset_smc_result(void)
{
	smc_result = (struct smc_result_data){0};
	serial_frame[0] = (struct psi5_frame){0};
	k_sem_reset(&smc_sem);
}

static void rx_data_frame_cb(const struct device *dev, uint8_t channel_id,
			     uint32_t num_frame, void *user_data)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(user_data);

	rx_result.callback_count++;
	rx_result.error_count += (num_frame == 0U);
	rx_result.num_frames = num_frame;
	rx_result.channel = channel_id;
	k_sem_give(&rx_sem);
}

static void rx_serial_frame_cb(const struct device *dev, uint8_t channel_id,
			       uint32_t num_frame, void *user_data)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(channel_id);
	ARG_UNUSED(user_data);

	smc_result.callback_count++;
	smc_result.error_count += (num_frame == 0U);
	smc_result.num_frames = num_frame;
	k_sem_give(&smc_sem);
}

static void tx_cb(const struct device *dev, uint8_t channel_id, int status,
		  void *user_data)
{
	uint32_t request = (uint32_t)(uintptr_t)user_data;

	ARG_UNUSED(dev);
	ARG_UNUSED(channel_id);

	tx_callback_count++;
	if (request < ARRAY_SIZE(tx_status)) {
		tx_status[request] = status;
	}
	k_sem_give(&tx_sem);
}

static struct psi5_rx_callback_config serial_cb_cfg = {
	.callback = rx_serial_frame_cb,
	.frame = serial_frame,
	.max_num_frame = ARRAY_SIZE(serial_frame),
};

static struct psi5_rx_callback_config data_cb_cfg = {
	.callback = rx_data_frame_cb,
	.frame = data_frame,
	.max_num_frame = ARRAY_SIZE(data_frame),
};

static struct psi5_rx_callback_configs callback_configs = {
	.serial_frame = &serial_cb_cfg,
	.data_frame = &data_cb_cfg,
};

static int send_frame(uint32_t data, uint8_t data_len, bool has_parity)
{
	if (PSI5_TEST_ASYNC) {
		return psi5_emu_send_frame(data, data_len, has_parity);
	}
	return psi5_emu_send_frame_synced(data, data_len, has_parity);
}

/*
 * The driver does not wait: psi5_stop_sync() returns -EBUSY while a TX frame is still
 * in the TX buffer. The application retries until the frame has been sent.
 */
static int stop_sync_wait(void)
{
	k_timepoint_t end = sys_timepoint_calc(PSI5_STOP_TIMEOUT);
	int ret;

	do {
		ret = psi5_stop_sync(psi5_dev, PSI5_CHANNEL);
		if (ret != -EBUSY) {
			return ret;
		}
		k_msleep(1);
	} while (!sys_timepoint_expired(end));

	return ret;
}

static void *psi5_rx_setup(void)
{
	int ret;

	zassert_true(device_is_ready(psi5_dev), "PSI5 device is not ready");
	k_sem_init(&rx_sem, 0, 1);
	k_sem_init(&smc_sem, 0, 1);
	k_sem_init(&tx_sem, 0, 4);

	zassert_ok(psi5_emu_init(), "Failed to initialize PSI5 emulator");
	zassert_ok(psi5_register_callback(psi5_dev, PSI5_CHANNEL, callback_configs),
		   "Failed to register callbacks");

	ret = psi5_start_sync(psi5_dev, PSI5_CHANNEL);
	if (PSI5_TEST_ASYNC) {
		zassert_equal(ret, -ENOTSUP, "start_sync returned %d", ret);
	} else {
		zassert_ok(ret, "start_sync returned %d", ret);
	}

	return NULL;
}

static void psi5_rx_teardown(void *fixture)
{
	int ret;

	ARG_UNUSED(fixture);
	if (PSI5_TEST_ASYNC) {
		ret = psi5_stop_sync(psi5_dev, PSI5_CHANNEL);
		zassert_equal(ret, -ENOTSUP, "stop_sync returned %d", ret);
	} else {
		ret = stop_sync_wait();
		zassert_ok(ret, "stop_sync returned %d", ret);
	}
}

ZTEST(psi5_rx, test_receive_reference_data_frame)
{
	reset_rx_result();
	zassert_ok(send_frame(PSI5_EMU_DATA, PSI5_EMU_DATA_LEN, true),
		   "Failed to send reference frame");
	zassert_ok(k_sem_take(&rx_sem, PSI5_RX_TIMEOUT), "RX callback timeout");

	zassert_equal(rx_result.num_frames, 1U, "Expected one frame");
	zassert_equal(rx_result.channel, PSI5_CHANNEL, "Unexpected channel");
	zassert_equal(data_frame[0].type, PSI5_DATA_FRAME, "Unexpected type");
	zassert_equal(data_frame[0].data, PSI5_EMU_DATA, "Unexpected data");
	zassert_equal(data_frame[0].slot_number, 0U, "Unexpected slot");
}

ZTEST(psi5_rx, test_rx_frame_error)
{
	reset_rx_result();
	zassert_ok(send_frame(PSI5_EMU_BAD_PARITY_DATA, PSI5_EMU_BAD_PARITY_LEN, false),
		   "Failed to send bad-parity frame");
	zassert_ok(k_sem_take(&rx_sem, PSI5_RX_TIMEOUT), "Error callback timeout");
	k_msleep(10);

	zassert_equal(rx_result.num_frames, 0U, "Bad frame reported as valid");
	zassert_equal(rx_result.error_count, 1U, "Expected one error callback");
	zassert_equal(rx_result.callback_count, 1U, "Unexpected duplicate callback");

	reset_rx_result();
	zassert_ok(send_frame(PSI5_EMU_DATA, PSI5_EMU_DATA_LEN, true),
		   "Failed to send recovery frame");
	zassert_ok(k_sem_take(&rx_sem, PSI5_RX_TIMEOUT), "Recovery callback timeout");
	zassert_equal(rx_result.num_frames, 1U, "Receiver did not recover");
	zassert_equal(data_frame[0].data, PSI5_EMU_DATA, "Unexpected recovery data");
}

ZTEST(psi5_rx, test_rx_serial_message)
{
	if (!PSI5_TEST_SMC || !PSI5_TEST_ASYNC) {
		ztest_test_skip();
	}

	reset_rx_result();
	zassert_ok(send_frame(PSI5_EMU_SMC_RESET_DATA, PSI5_EMU_DATA_LEN, true),
		   "Failed to send SMC reset frame");
	(void)k_sem_take(&rx_sem, PSI5_RX_TIMEOUT);

	reset_rx_result();
	reset_smc_result();
	zassert_ok(psi5_emu_send_smc_pattern(PSI5_EMU_SMC_CRC),
		   "Failed to send SMC pattern");
	zassert_ok(k_sem_take(&smc_sem, PSI5_SMC_TIMEOUT), "SMC callback timeout");
	k_msleep(10);

	zassert_equal(smc_result.error_count, 0U, "SMC error reported");
	zassert_equal(smc_result.callback_count, 1U, "Unexpected SMC callback count");
	zassert_equal(smc_result.num_frames, 1U, "Expected one message");
	zassert_equal(serial_frame[0].type, PSI5_SERIAL_FRAME_8_BIT_ID, "Unexpected type");
	zassert_equal(serial_frame[0].serial.id, PSI5_SMC_EXP_ID, "Unexpected ID");
	zassert_equal(serial_frame[0].serial.data, PSI5_SMC_EXP_DATA, "Unexpected data");
	zassert_equal(serial_frame[0].crc, PSI5_EMU_SMC_CRC, "Unexpected CRC");
	zassert_equal(serial_frame[0].slot_number, 0U, "Unexpected slot");
	zassert_equal(rx_result.error_count, 0U, "Data error during SMC");
}

ZTEST(psi5_rx, test_rx_stop_start)
{
	int ret;

	if (PSI5_TEST_ASYNC) {
		ztest_test_skip();
	}

	zassert_ok(psi5_stop_sync(psi5_dev, PSI5_CHANNEL), "stop_sync failed");
	reset_rx_result();
	zassert_ok(psi5_emu_send_frame(PSI5_EMU_RESTART_DATA, PSI5_EMU_DATA_LEN, true),
		   "Failed to send while stopped");
	ret = k_sem_take(&rx_sem, PSI5_NO_CB_WINDOW);
	zassert_equal(ret, -EAGAIN, "Callback received while stopped");
	zassert_equal(rx_result.callback_count, 0U, "Unexpected callback while stopped");

	zassert_ok(psi5_start_sync(psi5_dev, PSI5_CHANNEL), "restart failed");
	reset_rx_result();
	zassert_ok(send_frame(PSI5_EMU_RESTART_DATA, PSI5_EMU_DATA_LEN, true),
		   "Failed to send after restart");
	zassert_ok(k_sem_take(&rx_sem, PSI5_RX_TIMEOUT), "RX timeout after restart");
	zassert_equal(rx_result.num_frames, 1U, "Expected one frame");
	zassert_equal(data_frame[0].data, PSI5_EMU_RESTART_DATA, "Stale or invalid data");
}

ZTEST(psi5_rx, test_async_mode)
{
	if (!PSI5_TEST_ASYNC) {
		ztest_test_skip();
	}

	zassert_equal(psi5_start_sync(psi5_dev, PSI5_CHANNEL), -ENOTSUP,
		      "start_sync must be unsupported");
	zassert_equal(psi5_stop_sync(psi5_dev, PSI5_CHANNEL), -ENOTSUP,
		      "stop_sync must be unsupported");
	zassert_equal(psi5_send(psi5_dev, PSI5_CHANNEL, 0x2AU, K_NO_WAIT, tx_cb, NULL),
		      -ENOTSUP, "send must be unsupported");

	reset_rx_result();
	zassert_ok(send_frame(PSI5_EMU_DATA, PSI5_EMU_DATA_LEN, true), "Send failed");
	zassert_ok(k_sem_take(&rx_sem, PSI5_RX_TIMEOUT), "Async RX timeout");
	zassert_equal(rx_result.num_frames, 1U, "Expected one frame");
	zassert_equal(data_frame[0].data, PSI5_EMU_DATA, "Unexpected data");
}

ZTEST(psi5_rx, test_tx_data_width)
{
	int ret;

	if (PSI5_TEST_ASYNC) {
		ztest_test_skip();
	}

	tx_callback_count = 0U;
	for (uint32_t i = 0U; i < ARRAY_SIZE(tx_status); i++) {
		tx_status[i] = 1;
	}
	k_sem_reset(&tx_sem);

	zassert_ok(psi5_send(psi5_dev, PSI5_CHANNEL, PSI5_TX_DATA_FIT,
			     PSI5_TX_TIMEOUT, tx_cb, (void *)0U), "Valid send failed");
	zassert_ok(k_sem_take(&tx_sem, PSI5_TX_TIMEOUT), "TX callback timeout");
	zassert_equal(tx_status[0], 0, "Unexpected TX status");

	zassert_equal(psi5_send(psi5_dev, PSI5_CHANNEL, PSI5_TX_DATA_OVER,
				K_NO_WAIT, tx_cb, (void *)1U), -EMSGSIZE,
		      "Oversized data was not rejected");
	zassert_equal(psi5_send(psi5_dev, PSI5_CHANNEL, PSI5_TX_DATA_OVER_HI,
				K_NO_WAIT, tx_cb, (void *)1U), -EMSGSIZE,
		      "High-bit data was not rejected");
	ret = k_sem_take(&tx_sem, PSI5_NO_CB_WINDOW);
	zassert_equal(ret, -EAGAIN, "Callback received for rejected data");

	zassert_ok(psi5_send(psi5_dev, PSI5_CHANNEL, 0x01U, PSI5_TX_TIMEOUT,
			     tx_cb, (void *)1U), "Send after error failed");
	zassert_ok(k_sem_take(&tx_sem, PSI5_TX_TIMEOUT), "TX callback timeout");

	zassert_ok(stop_sync_wait(), "stop_sync failed");
	zassert_equal(psi5_send(psi5_dev, PSI5_CHANNEL, PSI5_TX_DATA_OVER,
				K_NO_WAIT, tx_cb, (void *)2U), -ENETDOWN,
		      "Stopped state must take priority");
	zassert_ok(psi5_start_sync(psi5_dev, PSI5_CHANNEL), "restart failed");
}

ZTEST(psi5_rx, test_tx_rx_concurrent)
{
	if (PSI5_TEST_ASYNC) {
		ztest_test_skip();
	}

	tx_callback_count = 0U;
	for (uint32_t i = 0U; i < ARRAY_SIZE(tx_status); i++) {
		tx_status[i] = 1;
	}
	k_sem_reset(&tx_sem);
	reset_rx_result();

	zassert_ok(psi5_send(psi5_dev, PSI5_CHANNEL, 0x15U, K_NO_WAIT,
			     tx_cb, (void *)0U), "Concurrent TX failed");
	zassert_ok(send_frame(PSI5_EMU_DATA, PSI5_EMU_DATA_LEN, true),
		   "Concurrent RX frame failed");
	zassert_ok(k_sem_take(&rx_sem, PSI5_RX_TIMEOUT), "Concurrent RX timeout");
	zassert_equal(data_frame[0].data, PSI5_EMU_DATA, "Unexpected RX data");
	zassert_ok(k_sem_take(&tx_sem, PSI5_TX_TIMEOUT), "Concurrent TX timeout");
	zassert_equal(tx_status[0], 0, "Unexpected TX status");

	/* Back-to-back request: tx_sem was released by the previous completion */
	zassert_ok(psi5_send(psi5_dev, PSI5_CHANNEL, 0x2AU, K_NO_WAIT,
			     tx_cb, (void *)1U), "Second request failed");
	zassert_ok(k_sem_take(&tx_sem, PSI5_TX_TIMEOUT), "Second request timeout");
	zassert_equal(tx_status[1], 0, "Unexpected second TX status");
}

ZTEST_SUITE(psi5_rx, NULL, psi5_rx_setup, NULL, NULL, psi5_rx_teardown);
