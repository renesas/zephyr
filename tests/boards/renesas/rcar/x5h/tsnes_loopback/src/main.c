/*
 * Copyright (c) 2026 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(tsnes_loopback_test, LOG_LEVEL_INF);

#include <zephyr/ztest.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_core.h>
#include <zephyr/net/net_mgmt.h>
#include <zephyr/net/net_context.h>
#include <zephyr/net/net_pkt.h>
#include <zephyr/net/ethernet.h>
#include <zephyr/sys/util.h>
#include <zephyr/random/random.h>

#define TSNES_DEV_NAME      DT_NODELABEL(eth_test)
#define TEST_MAC_ADDR       {0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC}
#define TEST_ETHER_TYPE     0x88B5 /* Local Experimental EtherType */
#define MAX_WAIT_MS         1000
#define ETH_HDR_LEN         14
#define IFACE_UP_TIMEOUT_MS 3000

struct rx_frame {
	sys_snode_t node;
	size_t len;
	uint8_t data[1514];
};

static const struct device *tsnes_dev = DEVICE_DT_GET(TSNES_DEV_NAME);
static const struct device *tsnes_eth_dev;
static const struct ethernet_api *tsnes_eth_api;
static struct net_if *tsnes_iface;
static struct net_context *rx_ctx;
static K_FIFO_DEFINE(rx_fifo);
static K_SEM_DEFINE(iface_up_sem, 0, 1);

static void iface_up_handler(struct net_mgmt_event_callback *cb, uint64_t mgmt_event,
			     struct net_if *iface)
{
	ARG_UNUSED(cb);
	ARG_UNUSED(iface);

	if (mgmt_event == NET_EVENT_IF_UP) {
		k_sem_give(&iface_up_sem);
	}
}

static void wait_for_iface_up(struct net_if *iface)
{
	struct net_mgmt_event_callback cb;
	int ret;

	if (net_if_is_up(iface)) {
		return;
	}

	net_mgmt_init_event_callback(&cb, iface_up_handler, NET_EVENT_IF_UP);
	net_mgmt_add_event_callback(&cb);

	LOG_INF("Waiting for network interface to come up...");
	ret = k_sem_take(&iface_up_sem, K_MSEC(IFACE_UP_TIMEOUT_MS));

	net_mgmt_del_event_callback(&cb);

	zassert_equal(ret, 0,
		      "Network interface did not come up within %d ms -- check R-Switch3 "
		      "forwarding, PCS bring-up, and MAC-PHY link status",
		      IFACE_UP_TIMEOUT_MS);
}

static void eth_rx_cb(struct net_context *ctx, struct net_pkt *pkt, union net_ip_header *ip_hdr,
		      union net_proto_header *proto_hdr, int status, void *user_data)
{
	ARG_UNUSED(ctx);
	ARG_UNUSED(ip_hdr);
	ARG_UNUSED(proto_hdr);
	ARG_UNUSED(user_data);

	if (!pkt || status < 0) {
		LOG_WRN("RX callback error: status=%d pkt=%p", status, pkt);
		return;
	}

	size_t len = net_pkt_get_len(pkt);

	if (len < ETH_HDR_LEN) {
		net_pkt_unref(pkt);
		return;
	}

	struct rx_frame *entry = k_malloc(sizeof(*entry));

	if (!entry) {
		LOG_WRN("RX callback packet allocation failed");
		net_pkt_unref(pkt);
		return;
	}

	entry->len = MIN(len, sizeof(entry->data));
	if (net_pkt_read(pkt, entry->data, entry->len) < 0) {
		LOG_WRN("RX callback failed to read packet");
		k_free(entry);
		net_pkt_unref(pkt);
		return;
	}

	net_pkt_unref(pkt);

	uint16_t eth_type = (entry->data[12] << 8) | entry->data[13];

	if (eth_type != TEST_ETHER_TYPE) {
		k_free(entry);
		return;
	}

	k_fifo_put(&rx_fifo, entry);
}

static int send_test_packet(uint16_t size, const uint8_t *payload)
{
	uint8_t mac[] = TEST_MAC_ADDR;
	uint8_t buf[1514];

	if (size < ETH_HDR_LEN) {
		size = ETH_HDR_LEN;
	}
	if (size > sizeof(buf)) {
		size = sizeof(buf);
	}

	memcpy(&buf[0], mac, 6); /* Dst MAC */
	memcpy(&buf[6], mac, 6); /* Src MAC */
	uint16_t eth_type = htons(TEST_ETHER_TYPE);

	memcpy(&buf[12], &eth_type, 2);

	/* Fill Payload */
	if (payload && size > ETH_HDR_LEN) {
		memcpy(&buf[ETH_HDR_LEN], payload, size - ETH_HDR_LEN);
	}

	struct net_pkt *pkt = net_pkt_alloc_with_buffer(tsnes_iface, size, AF_UNSPEC, 0, K_FOREVER);

	if (!pkt) {
		return -ENOMEM;
	}

	if (net_pkt_write(pkt, buf, size) < 0) {
		net_pkt_unref(pkt);
		return -EINVAL;
	}

	net_pkt_cursor_init(pkt);

	int ret = tsnes_eth_api->send(tsnes_eth_dev, pkt);

	if (ret < 0) {
		net_pkt_unref(pkt);
		return ret;
	}

	net_pkt_unref(pkt);

	return size;
}

static int receive_test_packet(uint8_t *buf, size_t buf_size, int timeout_ms)
{
	struct rx_frame *entry = k_fifo_get(&rx_fifo, K_MSEC(timeout_ms));

	if (!entry) {
		return -EAGAIN;
	}

	size_t len = MIN(buf_size, entry->len);

	memcpy(buf, entry->data, len);
	k_free(entry);

	return len;
}

static void *tsnes_test_setup(void)
{
	tsnes_iface = net_if_lookup_by_dev(tsnes_dev);
	zassert_not_null(tsnes_iface, "Network interface not found");

	tsnes_eth_dev = tsnes_dev;
	tsnes_eth_api = (const struct ethernet_api *)tsnes_dev->api;

	wait_for_iface_up(tsnes_iface);

	k_fifo_init(&rx_fifo);

	int ret = net_context_get(AF_PACKET, SOCK_RAW, IPPROTO_RAW, &rx_ctx);

	zassert_equal(ret, 0, "net_context_get failed: %d", ret);

	struct sockaddr_ll addr = {
		.sll_family = AF_PACKET,
		.sll_protocol = htons(TEST_ETHER_TYPE),
		.sll_ifindex = net_if_get_by_iface(tsnes_iface),
	};

	ret = net_context_bind(rx_ctx, (struct sockaddr *)&addr, sizeof(addr));
	zassert_equal(ret, 0, "net_context_bind failed: %d", ret);

	ret = net_context_recv(rx_ctx, eth_rx_cb, K_NO_WAIT, NULL);
	zassert_equal(ret, 0, "net_context_recv failed: %d", ret);

	k_msleep(100);

	return NULL;
}

static void tsnes_test_teardown(void *fixture)
{
	ARG_UNUSED(fixture);

	if (rx_ctx) {
		net_context_put(rx_ctx);
		rx_ctx = NULL;
	}

	while (true) {
		struct rx_frame *entry = k_fifo_get(&rx_fifo, K_NO_WAIT);

		if (!entry) {
			break;
		}
		k_free(entry);
	}
}

/* Scenario 1: Basic MAC Loopback Init & Link Status */
ZTEST(tsnes_loopback_suite, test_init_and_link_status)
{
	LOG_INF("Testing Scenario 1: Init and Link Status");

	zassert_true(device_is_ready(tsnes_dev), "TSNES device is not ready");
	zassert_true(net_if_is_up(tsnes_iface), "Network interface is not UP");
}

/* Scenario 2: Simple Tx/Rx Loopback Data Integrity with Random Data */
ZTEST(tsnes_loopback_suite, test_single_packet_integrity)
{
	LOG_INF("Testing Scenario 2: Single Packet Integrity (Random Data)");

	int ret;
	uint16_t test_size = 128;
	uint16_t payload_len = test_size - ETH_HDR_LEN;
	static uint8_t tx_payload[1514];
	static uint8_t rx_buf[1514];

	sys_rand_get(tx_payload, payload_len);

	ret = send_test_packet(test_size, tx_payload);
	zassert_equal(ret, test_size, "Failed to send packet");

	ret = receive_test_packet(rx_buf, sizeof(rx_buf), MAX_WAIT_MS);
	zassert_true(ret > 0, "Timeout or error waiting for loopback packet");
	zassert_equal(ret, test_size, "Received length mismatch");

	zassert_mem_equal(&rx_buf[ETH_HDR_LEN], tx_payload, payload_len,
			  "Random payload data mismatch");
}

/* Scenario 3: Loopback Burst Traffic */
ZTEST(tsnes_loopback_suite, test_burst_traffic)
{
	LOG_INF("Testing Scenario 3: Burst Traffic (Stress Test with SeqNum + Random)");

	int ret;
	int burst_count = 50;
	int received_count = 0;
	uint16_t test_size = 256;
	uint16_t payload_len = test_size - ETH_HDR_LEN;
	static uint8_t rx_buf[1514];

	static uint8_t master_rand[256];

	sys_rand_get(master_rand, payload_len);

	for (int i = 0; i < burst_count; i++) {
		static uint8_t tx_payload[256];

		memcpy(tx_payload, master_rand, payload_len);

		tx_payload[0] = (i >> 24) & 0xFF;
		tx_payload[1] = (i >> 16) & 0xFF;
		tx_payload[2] = (i >> 8) & 0xFF;
		tx_payload[3] = i & 0xFF;

		ret = send_test_packet(test_size, tx_payload);
		if (ret == -ENOBUFS || ret == -ENOMEM) {
			i--;
			k_msleep(10);
			continue;
		}

		zassert_equal(ret, test_size, "Failed to send packet %d in burst", i);
	}

	for (int i = 0; i < burst_count; i++) {
		ret = receive_test_packet(rx_buf, sizeof(rx_buf), 100);
		if (ret > 0) {
			received_count++;

			uint32_t seq = (rx_buf[ETH_HDR_LEN] << 24) |
				       (rx_buf[ETH_HDR_LEN + 1] << 16) |
				       (rx_buf[ETH_HDR_LEN + 2] << 8) | rx_buf[ETH_HDR_LEN + 3];
			zassert_equal(seq, i, "Sequence mismatch in burst: expected %d, got %d", i,
				      seq);

			zassert_mem_equal(&rx_buf[ETH_HDR_LEN + 4], &master_rand[4],
					  payload_len - 4,
					  "Data corruption in burst test at packet %d", i);
		} else {
			break;
		}
	}

	zassert_equal(received_count, burst_count,
		      "Lost packets in burst test (received %d/%d). Possible Descriptor stall or "
		      "Cache drop.",
		      received_count, burst_count);
}

/* Scenario 4: Variable Packet Size Test */
ZTEST(tsnes_loopback_suite, test_variable_packet_sizes)
{
	LOG_INF("Testing Scenario 4: Variable Packet Sizes (Random Data)");

	int ret;

	uint16_t sizes[] = {64, 256, 1024, 1500};
	static uint8_t tx_payload[1514];
	static uint8_t rx_buf[1514];

	for (int i = 0; i < ARRAY_SIZE(sizes); i++) {
		uint16_t current_size = sizes[i];
		uint16_t payload_len = current_size - ETH_HDR_LEN;

		LOG_INF("Testing packet size: %d", current_size);

		sys_rand_get(tx_payload, payload_len);

		ret = send_test_packet(current_size, tx_payload);
		zassert_equal(ret, current_size, "Failed to send packet of size %d", current_size);

		ret = receive_test_packet(rx_buf, sizeof(rx_buf), MAX_WAIT_MS);
		zassert_true(ret > 0, "Timeout receiving packet of size %d", current_size);
		zassert_equal(ret, current_size, "Length mismatch for size %d", current_size);

		zassert_mem_equal(&rx_buf[ETH_HDR_LEN], tx_payload, payload_len,
				  "Payload data mismatch for size %d", current_size);
	}
}

ZTEST_SUITE(tsnes_loopback_suite, NULL, tsnes_test_setup, NULL, NULL, tsnes_test_teardown);
