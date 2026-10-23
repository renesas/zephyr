/*
 * Copyright (c) 2026 Renesas Electronics Corporation
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <errno.h>
#include <stdbool.h>
#include <string.h>

#include <zephyr/drivers/mbox.h>

#include <openamp/open_amp.h>
#include <openamp/version.h>
#include <metal/sys.h>
#include <metal/io.h>
#include <resource_table.h>
#include <addr_translation.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(rpmsg_echo);

#if !DT_HAS_CHOSEN(zephyr_ipc_shm)
#error "Sample requires definition of shared memory for rpmsg"
#endif

/* Constants derived from device tree */
#define SHM_NODE       DT_CHOSEN(zephyr_ipc_shm)
#define SHM_START_ADDR DT_REG_ADDR(SHM_NODE)
#define SHM_SIZE       DT_REG_SIZE(SHM_NODE)

#define APP_TASK_STACK_SIZE (1024)

#ifdef CONFIG_SAMPLE_RPMSG_ECHO_KERNELSPACE
/*
 * ROM Writer cannot program the SDRAM resource-table address. Keep this
 * template with the SRAM firmware image so Zephyr can initialize the live
 * table before Linux remoteproc reads and updates it.
 */
static const struct fw_resource_table resource_table_template = RESOURCE_TABLE_INIT;
#endif

static void rpmsg_echo_rsc_table_get(void **table_ptr, int *length)
{
	rsc_table_get(table_ptr, length);

#ifdef CONFIG_SAMPLE_RPMSG_ECHO_KERNELSPACE
	memcpy(*table_ptr, &resource_table_template, sizeof(resource_table_template));
#endif
}

K_THREAD_STACK_DEFINE(thread_mng_stack, APP_TASK_STACK_SIZE);
K_THREAD_STACK_DEFINE(thread_rp_client_stack, APP_TASK_STACK_SIZE);

static struct k_thread thread_mng_data;
static struct k_thread thread_rp_client_data;

static const struct mbox_dt_spec tx_channel = MBOX_DT_SPEC_GET(DT_CHOSEN(zephyr_ipc), tx);
static const struct mbox_dt_spec rx_channel = MBOX_DT_SPEC_GET(DT_CHOSEN(zephyr_ipc), rx);

static metal_phys_addr_t shm_physmap = SHM_START_ADDR;
static metal_phys_addr_t rsc_tab_physmap;

static struct metal_io_region shm_io_data; /* shared memory */
static struct metal_io_region rsc_io_data; /* rsc_table memory */

struct rpmsg_rcv_msg {
	void *data;
	size_t len;
};

static struct metal_io_region *shm_io = &shm_io_data;

static struct metal_io_region *rsc_io = &rsc_io_data;
static struct rpmsg_virtio_device rvdev;

static void *rsc_table;
static struct rpmsg_device *rpdev;
static bool metal_initialized;
static bool mbox_enabled;
static bool rpmsg_initialized;

static char rx_sc_msg[20];
static struct rpmsg_endpoint sc_ept;
static struct rpmsg_rcv_msg sc_msg = {.data = rx_sc_msg};

static K_SEM_DEFINE(data_sem, 0, 1);
static K_SEM_DEFINE(data_sc_sem, 0, 1);

static void platform_mbox_callback(const struct device *dev, mbox_channel_id_t channel_id,
				   void *user_data, struct mbox_msg *data)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(user_data);
	ARG_UNUSED(data);

	LOG_DBG("%s: msg received from mailbox %d", __func__, channel_id);
	k_sem_give(&data_sem);
}

static int rpmsg_recv_cs_callback(struct rpmsg_endpoint *ept, void *data, size_t len, uint32_t src,
				  void *priv)
{
	ARG_UNUSED(ept);
	ARG_UNUSED(src);
	ARG_UNUSED(priv);

	if (len > sizeof(rx_sc_msg)) {
		LOG_ERR("RPMsg payload too large: %zu", len);
		return RPMSG_ERR_BUFF_SIZE;
	}

	memcpy(sc_msg.data, data, len);
	sc_msg.len = len;
	k_sem_give(&data_sc_sem);

	return RPMSG_SUCCESS;
}

static void new_service_cb(struct rpmsg_device *rdev, const char *name, uint32_t src)
{
	ARG_UNUSED(rdev);
	ARG_UNUSED(src);

	LOG_INF("%s: message received from service %s", __func__, name);
}

static void receive_message(unsigned char **msg, unsigned int *len)
{
	ARG_UNUSED(msg);
	ARG_UNUSED(len);

	int status = k_sem_take(&data_sem, K_FOREVER);

	if (status == 0) {
		/* MFIS does not identify which virtqueue was kicked. */
		rproc_virtio_notified(rvdev.vdev, RSC_NOTIFY_ID_ANY);
	}
}

static int mailbox_notify(void *priv, uint32_t id)
{
	ARG_UNUSED(priv);
	ARG_UNUSED(id);

	int ret = mbox_send_dt(&tx_channel, NULL);

	if (ret == -EBUSY) {
		/*
		 * Another MBOX notification is already pending.
		 * The new vring update is already visible in shared memory and
		 * will be processed when Linux handles the pending notification.
		 */
		LOG_DBG("MBOX notification already pending");
		return 0;
	}

	if (ret != 0) {
		LOG_ERR("Failed to notify Linux: %d", ret);
	}

	return ret;
}

static void cleanup_system(void)
{
	if (mbox_enabled) {
		mbox_set_enabled_dt(&rx_channel, false);
		mbox_enabled = false;
	}

	if (rpmsg_initialized) {
		rpmsg_deinit_vdev(&rvdev);
		rpmsg_initialized = false;
	}

	if (metal_initialized) {
		metal_finish();
		metal_initialized = false;
	}
}

static int platform_init(void)
{
	int rsc_size;
	struct metal_init_params metal_params = METAL_INIT_DEFAULTS;
	int status;

	/* Initialize libmetal */
	status = metal_init(&metal_params);
	if (status != 0) {
		LOG_ERR("metal_init: failed: %d", status);
		return status;
	}
	metal_initialized = true;

	/* Declare shared memory region */
	metal_io_init(shm_io, (void *)SHM_START_ADDR, &shm_physmap, SHM_SIZE, -1, 0,
		      addr_translation_get_ops(shm_physmap));

	/* Declare resource table region */
	rpmsg_echo_rsc_table_get(&rsc_table, &rsc_size);
	rsc_tab_physmap = (uintptr_t)rsc_table;

	metal_io_init(rsc_io, rsc_table, &rsc_tab_physmap, rsc_size, -1, 0, NULL);

	/* Setup MBOX */
	if (mbox_is_ready_dt(&tx_channel) == false) {
		LOG_ERR("MBOX TX device is not ready");
		return -ENODEV;
	}

	if (mbox_is_ready_dt(&rx_channel) == false) {
		LOG_ERR("MBOX RX device is not ready");
		return -ENODEV;
	}

	status = mbox_register_callback_dt(&rx_channel, platform_mbox_callback, NULL);
	if (status != 0) {
		LOG_ERR("mbox_register_callback_dt failed: %d", status);
		return status;
	}

	status = mbox_set_enabled_dt(&rx_channel, true);
	if (status != 0) {
		LOG_ERR("mbox_set_enabled_dt failed: %d", status);
		return status;
	}
	mbox_enabled = true;

	return 0;
}

static struct rpmsg_device *platform_create_rpmsg_vdev(unsigned int vdev_index, unsigned int role,
						       void (*rst_cb)(struct virtio_device *vdev),
						       rpmsg_ns_bind_cb ns_cb)
{
	struct fw_rsc_vdev_vring *vring_rsc;
	struct virtio_device *vdev;
	int ret;

	ARG_UNUSED(vdev_index);

	vdev = rproc_virtio_create_vdev(role, VDEV_ID, rsc_table_to_vdev(rsc_table), rsc_io, NULL,
					mailbox_notify, rst_cb);

	if (vdev == NULL) {
		LOG_ERR("failed to create vdev");
		return NULL;
	}

	/* Wait master rpmsg init completion */
	rproc_virtio_wait_remote_ready(vdev);

	vring_rsc = rsc_table_get_vring0(rsc_table);
	ret = rproc_virtio_init_vring(vdev, 0, vring_rsc->notifyid, (void *)vring_rsc->da, shm_io,
				      vring_rsc->num, vring_rsc->align);
	if (ret != 0) {
		LOG_ERR("failed to init vring 0");
		goto failed;
	}

	vring_rsc = rsc_table_get_vring1(rsc_table);
	ret = rproc_virtio_init_vring(vdev, 1, vring_rsc->notifyid, (void *)vring_rsc->da, shm_io,
				      vring_rsc->num, vring_rsc->align);
	if (ret != 0) {
		LOG_ERR("failed to init vring 1");
		goto failed;
	}

	ret = rpmsg_init_vdev(&rvdev, vdev, ns_cb, shm_io, NULL);
	if (ret != 0) {
		LOG_ERR("failed rpmsg_init_vdev");
		goto failed;
	}
	rpmsg_initialized = true;

	return rpmsg_virtio_get_rpmsg_device(&rvdev);

failed:
	rproc_virtio_remove_vdev(vdev);

	return NULL;
}

static void app_rpmsg_client_sample(void *arg1, void *arg2, void *arg3)
{
	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);

	unsigned int msg_cnt = 0;
	int ret = 0;

	k_sem_take(&data_sc_sem, K_FOREVER);

	LOG_INF("OpenAMP[remote] Linux sample client responder started");

	ret = rpmsg_create_ept(&sc_ept, rpdev, "rpmsg-client-sample", RPMSG_ADDR_ANY,
			       RPMSG_ADDR_ANY, rpmsg_recv_cs_callback, NULL);
	if (ret != 0) {
		LOG_ERR("[Linux sample client] Could not create endpoint: %d", ret);
		goto task_end;
	}

	while (msg_cnt < 100) {
		k_sem_take(&data_sc_sem, K_FOREVER);
		msg_cnt++;
		LOG_INF("[Linux sample client] incoming msg %u: %.*s", msg_cnt, (int)sc_msg.len,
			(char *)sc_msg.data);
		ret = rpmsg_send(&sc_ept, sc_msg.data, sc_msg.len);
		if (ret < 0) {
			LOG_ERR("Failed to echo RPMsg payload: %d", ret);
			break;
		}
	}
	rpmsg_destroy_ept(&sc_ept);

task_end:
	LOG_INF("OpenAMP Linux sample client responder ended");
}

static void rpmsg_mng_task(void *arg1, void *arg2, void *arg3)
{
	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);

	unsigned char *msg;
	unsigned int len;
	int ret = 0;

	LOG_INF("OpenAMP[remote] Linux responder demo started");

	/* Initialize platform */
	ret = platform_init();
	if (ret != 0) {
		LOG_ERR("Failed to initialize platform: %d", ret);
		goto task_end;
	}

	rpdev = platform_create_rpmsg_vdev(0, VIRTIO_DEV_DEVICE, NULL, new_service_cb);
	if (rpdev == NULL) {
		LOG_ERR("Failed to create rpmsg virtio device");
		goto task_end;
	}

	/* start the rpmsg clients */
	k_sem_give(&data_sc_sem);

	while (1) {
		receive_message(&msg, &len);
	}

task_end:
	cleanup_system();

	LOG_INF("OpenAMP demo ended");
}

int main(void)
{
	LOG_INF("OpenAMP library version: %s", openamp_version());
	LOG_INF("Starting application threads!");
	k_thread_create(&thread_mng_data, thread_mng_stack, APP_TASK_STACK_SIZE, rpmsg_mng_task,
			NULL, NULL, NULL, K_PRIO_COOP(8), 0, K_NO_WAIT);
	k_thread_create(&thread_rp_client_data, thread_rp_client_stack, APP_TASK_STACK_SIZE,
			app_rpmsg_client_sample, NULL, NULL, NULL, K_PRIO_COOP(7), 0, K_NO_WAIT);

	return 0;
}
