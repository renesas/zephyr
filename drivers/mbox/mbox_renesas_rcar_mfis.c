/*
 * Copyright (c) 2026 Renesas Electronics Corporation
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT renesas_rcar_mfis_common_mbox

#include <stdint.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/mbox.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/barrier.h>
#include <zephyr/sys/util.h>
#include "rp_mfis_mbox.h"

LOG_MODULE_REGISTER(mbox_renesas_rcar_mfis, CONFIG_MBOX_LOG_LEVEL);

#define MBOX_RCAR_MFIS_MAX_MSG_SIZE        4
#define MBOX_RCAR_MFIS_COMMON_MAX_CHANNELS 64
#define MBOX_RCAR_MFIS_SCP_MAX_CHANNELS    12

typedef enum mfis_domain {
	MFIS_COMMON,
	MFIS_SCP,
} mbox_rcar_mfis_domain_t;

struct mbox_rcar_mfis_config {
	mbox_rcar_mfis_domain_t domain;
	uint32_t max_channels;
	uint64_t channel_mask;
	const unsigned int *irqs;
	void (*irq_configure)(void);
};

struct mbox_rcar_mfis_chan_data {
	mbox_callback_t callback;
	void *user_data;
	bool enabled;
};

struct mbox_rcar_mfis_data {
	struct mbox_rcar_mfis_chan_data *channels;
	struct mfis_channel *common_channels;
	struct k_spinlock data_lock;
};

static const char *mbox_rcar_mfis_domain_name(mbox_rcar_mfis_domain_t domain)
{
	switch (domain) {
	case MFIS_COMMON:
		return "MFIS common";
	case MFIS_SCP:
		return "MFIS SCP";
	default:
		return "MFIS unknown";
	}
}

/**
 * @brief Check whether a channel is valid for this MFIS device instance.
 */
static int mbox_rcar_mfis_validate_channel(const struct device *dev, mbox_channel_id_t channel_id)
{
	const struct mbox_rcar_mfis_config *config = dev->config;

	if (channel_id >= config->max_channels ||
	    (config->channel_mask & BIT64(channel_id)) == 0U) {
		LOG_ERR("%s channel id %u is not available",
			mbox_rcar_mfis_domain_name(config->domain), channel_id);
		return -EINVAL;
	}

	return 0;
}

/**
 * Interrupt handler
 */
static void mbox_rcar_mfis_isr(const struct device *dev, mbox_channel_id_t channel_id)
{
	const struct mbox_rcar_mfis_config *config = dev->config;
	struct mbox_rcar_mfis_data *data = dev->data;
	struct mbox_msg msg;
	uint32_t local_msg;
	int ret, clear_ret;
	bool pending = false;

	if (config->domain == MFIS_COMMON) {
		ret = rp_mfis_common_check_rx_pending(&data->common_channels[channel_id], &pending);

		if (ret != 0) {
			LOG_ERR("%s channel %u failed to check RX pending: %d",
				mbox_rcar_mfis_domain_name(config->domain), channel_id, ret);
			return;
		}

		if (!pending) {
			/* Do not process spurious ISR */
			return;
		}

		/* Read message */
		ret = rp_mfis_common_get(&data->common_channels[channel_id], &local_msg);
	} else {
		ret = rp_mfis_scp_check_rx_pending(channel_id, &pending);

		if (ret != 0) {
			LOG_ERR("%s channel %u failed to check RX pending: %d",
				mbox_rcar_mfis_domain_name(config->domain), channel_id, ret);
			return;
		}

		if (!pending) {
			/* Do not process spurious ISR */
			return;
		}

		/* Read message */
		ret = rp_mfis_scp_get(channel_id, &local_msg);
	}

	if (ret != 0) {
		LOG_ERR("%s channel %u failed to read message: %d",
			mbox_rcar_mfis_domain_name(config->domain), channel_id, ret);
		goto exit;
	}

	if (data->channels[channel_id].enabled && data->channels[channel_id].callback) {
		msg.data = &local_msg;
		msg.size = MBOX_RCAR_MFIS_MAX_MSG_SIZE;

		data->channels[channel_id].callback(dev, channel_id,
						    data->channels[channel_id].user_data, &msg);
	}

exit:
	if (config->domain == MFIS_COMMON) {
		/* Clear interrupt */
		clear_ret = rp_mfis_common_clear(&data->common_channels[channel_id]);
	} else {
		/* Clear interrupt */
		clear_ret = rp_mfis_scp_clear(channel_id);
	}

	if (clear_ret != 0) {
		LOG_ERR("%s channel %u failed to clear interrupt: %d",
			mbox_rcar_mfis_domain_name(config->domain), channel_id, clear_ret);
		return;
	}
}

/**
 * @brief Try to send a message over the MBOX device.
 */
static int mbox_rcar_mfis_send(const struct device *dev, mbox_channel_id_t channel_id,
			       const struct mbox_msg *msg)
{
	const struct mbox_rcar_mfis_config *config = dev->config;
	struct mbox_rcar_mfis_data *data = dev->data;
	int ret;
	k_spinlock_key_t key;
	bool pending;
	uint32_t message = 0;

	ret = mbox_rcar_mfis_validate_channel(dev, channel_id);
	if (ret != 0) {
		return ret;
	}

	if (msg != NULL) {
		if (msg->size > MBOX_RCAR_MFIS_MAX_MSG_SIZE) {
			LOG_ERR("Message size %d is not valid. Maximum size is %d", msg->size,
				MBOX_RCAR_MFIS_MAX_MSG_SIZE);
			return -EMSGSIZE;
		}

		/* Data transfer mode */
		if (msg->data != NULL && msg->size != 0U) {
			/* Copy message */
			memcpy(&message, msg->data, msg->size);
		} else {
			LOG_ERR("Invalid message data/size");
			return -EINVAL;
		}
	} else {
		/* Signalling mode */
		message = 0;
	}

	key = k_spin_lock(&data->data_lock);

	if (config->domain == MFIS_COMMON) {
		ret = rp_mfis_common_check_tx_pending(&data->common_channels[channel_id], &pending);
		if (ret != 0) {
			goto unlock;
		}

		if (pending) {
			ret = -EBUSY;
			goto unlock;
		}

		ret = rp_mfis_common_send(&data->common_channels[channel_id], message);
		if (ret != 0) {
			goto unlock;
		}

		barrier_dsync_fence_full();

		ret = rp_mfis_common_trigger(&data->common_channels[channel_id], 0);
		if (ret != 0) {
			goto unlock;
		}
	} else {
		ret = rp_mfis_scp_check_tx_pending(channel_id, &pending);
		if (ret != 0) {
			goto unlock;
		}

		if (pending) {
			ret = -EBUSY;
			goto unlock;
		}

		ret = rp_mfis_scp_send(channel_id, message);
		if (ret != 0) {
			goto unlock;
		}

		barrier_dsync_fence_full();

		ret = rp_mfis_scp_trigger(channel_id);
		if (ret != 0) {
			goto unlock;
		}
	}

unlock:
	k_spin_unlock(&data->data_lock, key);

	return ret;
}

/**
 * @brief Register a callback function on a channel for incoming messages.
 */
static int mbox_rcar_mfis_reg_callback(const struct device *dev, mbox_channel_id_t channel_id,
				       mbox_callback_t cb, void *user_data)
{
	struct mbox_rcar_mfis_data *data = dev->data;
	k_spinlock_key_t key;
	int ret;

	ret = mbox_rcar_mfis_validate_channel(dev, channel_id);
	if (ret != 0) {
		return ret;
	}

	key = k_spin_lock(&data->data_lock);
	data->channels[channel_id].callback = cb;
	data->channels[channel_id].user_data = user_data;
	k_spin_unlock(&data->data_lock, key);

	return 0;
}

/**
 * @brief Enable (disable) interrupts and callbacks for inbound channels.
 */
static int mbox_rcar_mfis_set_enabled(const struct device *dev, mbox_channel_id_t channel_id,
				      bool enabled)
{
	const struct mbox_rcar_mfis_config *config = dev->config;
	struct mbox_rcar_mfis_data *data = dev->data;
	k_spinlock_key_t key;
	int ret = 0;

	ret = mbox_rcar_mfis_validate_channel(dev, channel_id);
	if (ret != 0) {
		return ret;
	}

	key = k_spin_lock(&data->data_lock);
	if ((enabled && data->channels[channel_id].enabled) ||
	    (!enabled && !data->channels[channel_id].enabled)) {
		ret = -EALREADY;
		goto unlock;
	}

	if (enabled && !data->channels[channel_id].callback) {
		LOG_WRN("Enabling %s channel %u without a registered callback",
			mbox_rcar_mfis_domain_name(config->domain), channel_id);
	}

	if (enabled) {
		irq_enable(config->irqs[channel_id]);
	} else {
		irq_disable(config->irqs[channel_id]);
	}

	data->channels[channel_id].enabled = enabled;

unlock:
	k_spin_unlock(&data->data_lock, key);

	return ret;
}

/**
 * @brief Return the maximum number of bytes possible in an outbound message.
 */
static int mbox_rcar_mfis_mtu_get(const struct device *dev)
{
	ARG_UNUSED(dev);

	return MBOX_RCAR_MFIS_MAX_MSG_SIZE;
}

/**
 * @brief Return the maximum number of channels.
 */
static uint32_t mbox_rcar_mfis_max_channels_get(const struct device *dev)
{
	const struct mbox_rcar_mfis_config *config = dev->config;

	return config->max_channels;
}

static DEVICE_API(mbox, mbox_rcar_mfis_driver_api) = {
	.send = mbox_rcar_mfis_send,
	.register_callback = mbox_rcar_mfis_reg_callback,
	.mtu_get = mbox_rcar_mfis_mtu_get,
	.max_channels_get = mbox_rcar_mfis_max_channels_get,
	.set_enabled = mbox_rcar_mfis_set_enabled,
};

static int mbox_rcar_mfis_init(const struct device *dev)
{
	const struct mbox_rcar_mfis_config *config = dev->config;

	if (config->domain == MFIS_COMMON) {
		rp_mfis_common_unlock_write();
	}

	if (config->irq_configure != NULL) {
		config->irq_configure();
	}

	return 0;
}

/**
 * ************************* DRIVER REGISTER SECTION ***************************
 */

/* clang-format off */
#define MFIS_COMMON_CH_IDX(node_id) DT_REG_ADDR_RAW(node_id)
#define MFIS_COMMON_RX_INT_IDX(node_id)                                                            \
	COND_CODE_1(DT_ENUM_IDX(node_id, renesas_local_side),                                      \
		    (UTIL_X2(MFIS_COMMON_CH_IDX(node_id))),                                        \
		    (UTIL_INC(UTIL_X2(MFIS_COMMON_CH_IDX(node_id)))))
#define MFIS_COMMON_IRQN_BY_IDX(node_id, idx)         DT_IRQN_BY_IDX(node_id, idx)
#define MFIS_COMMON_IRQ_PRIORITY_BY_IDX(node_id, idx) DT_IRQ_BY_IDX(node_id, idx, priority)
#define MFIS_COMMON_IRQ_FLAGS_BY_IDX(node_id, idx)    DT_IRQ_BY_IDX(node_id, idx, flags)
#define MFIS_COMMON_IRQN(node_id)                                                                  \
	MFIS_COMMON_IRQN_BY_IDX(DT_PARENT(node_id), MFIS_COMMON_RX_INT_IDX(node_id))
#define MFIS_COMMON_IRQ_PRIORITY(node_id)                                                          \
	MFIS_COMMON_IRQ_PRIORITY_BY_IDX(DT_PARENT(node_id), MFIS_COMMON_RX_INT_IDX(node_id))
#define MFIS_COMMON_IRQ_FLAGS(node_id)                                                             \
	MFIS_COMMON_IRQ_FLAGS_BY_IDX(DT_PARENT(node_id), MFIS_COMMON_RX_INT_IDX(node_id))

#define MFIS_COMMON_TYPE(node_id)                                                                  \
	COND_CODE_1(DT_ENUM_IDX(node_id, renesas_local_side),                                      \
		    (MFIS_TYPE_SENDER),                                                            \
		    (MFIS_TYPE_RECEVER))

#define MBOX_RCAR_MFIS_CHANNEL_INIT(node_id)                                                       \
	[DT_REG_ADDR(node_id)] = {                                                                 \
		.ch = DT_REG_ADDR(node_id),                                                        \
		.type = MFIS_COMMON_TYPE(node_id),                                                 \
	},

#define MBOX_RCAR_MFIS_COMMON_IRQ_INIT(node_id) [DT_REG_ADDR(node_id)] = MFIS_COMMON_IRQN(node_id),

#define MBOX_RCAR_MFIS_CHANNEL_MASK(node_id) BIT64(DT_REG_ADDR(node_id))

#define MBOX_RCAR_MFIS_COMMON_CHANNEL_MASK(inst)                                                   \
	COND_CODE_0(DT_INST_CHILD_NUM_STATUS_OKAY(inst),                                           \
		    (0ULL),                                                                        \
		    (DT_INST_FOREACH_CHILD_STATUS_OKAY_SEP(inst,                                   \
		     MBOX_RCAR_MFIS_CHANNEL_MASK, (|))))

#define MBOX_RCAR_MFIS_SCP_CHANNEL_MASK(inst) BIT64_MASK(DT_INST_NUM_IRQS(inst))

#define MBOX_RCAR_MFIS_COMMON_CHANNEL_ASSERT(node_id)                                              \
	BUILD_ASSERT(DT_REG_ADDR(node_id) < MBOX_RCAR_MFIS_COMMON_MAX_CHANNELS,                    \
		     "MFIS common channel exceeds maximum channel count");

#define MBOX_RCAR_MFIS_COMMON_ASSERTS(inst)                                                        \
	DT_INST_FOREACH_CHILD_STATUS_OKAY(inst, MBOX_RCAR_MFIS_COMMON_CHANNEL_ASSERT)

#define MBOX_RCAR_MFIS_SCP_ASSERTS(inst)                                                           \
	BUILD_ASSERT(DT_INST_NUM_IRQS(inst) <= MBOX_RCAR_MFIS_SCP_MAX_CHANNELS,                    \
		     "MFIS SCP interrupt count exceeds maximum channel count");

#define MBOX_RCAR_MFIS_ISR_NAME(node_id) CONCAT(mbox_rcar_mfis_isr_common_, node_id)

#define MBOX_RCAR_MFIS_ISR_DEFINE(node_id)                                                         \
	static void MBOX_RCAR_MFIS_ISR_NAME(node_id)(const struct device *dev)                     \
	{                                                                                          \
		mbox_rcar_mfis_isr(dev, DT_REG_ADDR(node_id));                                     \
	}

#define MBOX_RCAR_MFIS_COMMON_IRQ_CONNECT(node_id)                                                 \
	IRQ_CONNECT(MFIS_COMMON_IRQN(node_id), MFIS_COMMON_IRQ_PRIORITY(node_id),                  \
		    MBOX_RCAR_MFIS_ISR_NAME(node_id), DEVICE_DT_GET(DT_PARENT(node_id)),           \
		    MFIS_COMMON_IRQ_FLAGS(node_id));

#define MBOX_RCAR_MFIS_COMMON_IRQ_CONFIGURE(inst)                                                  \
	static void mbox_rcar_mfis_common_##inst##_irq_configure(void)                             \
	{                                                                                          \
		DT_INST_FOREACH_CHILD_STATUS_OKAY(inst, MBOX_RCAR_MFIS_COMMON_IRQ_CONNECT)         \
	}

#define MBOX_RCAR_MFIS_SCP_ISR_NAME(inst, channel) CONCAT(mbox_rcar_mfis_isr_scp_, inst, _, channel)

#define MBOX_RCAR_MFIS_SCP_ISR_DEFINE(channel, inst)                                               \
	static void MBOX_RCAR_MFIS_SCP_ISR_NAME(inst, channel)(const struct device *dev)           \
	{                                                                                          \
		mbox_rcar_mfis_isr(dev, channel);                                                  \
	}

#define MBOX_RCAR_MFIS_SCP_IRQ_CONNECT(channel, inst)                                              \
	IRQ_CONNECT(DT_INST_IRQN_BY_IDX(inst, channel),                                            \
		    DT_INST_IRQ_BY_IDX(inst, channel, priority),                                   \
		    MBOX_RCAR_MFIS_SCP_ISR_NAME(inst, channel), DEVICE_DT_INST_GET(inst),          \
		    DT_INST_IRQ_BY_IDX(inst, channel, flags));

#define MBOX_RCAR_MFIS_SCP_IRQ_INIT(channel, inst) [channel] = DT_INST_IRQN_BY_IDX(inst, channel),

#define MBOX_RCAR_MFIS_SCP_IRQ_CONFIGURE(inst)                                                     \
	static void mbox_rcar_mfis_scp_##inst##_irq_configure(void)                                \
	{                                                                                          \
		LISTIFY(DT_INST_NUM_IRQS(inst), MBOX_RCAR_MFIS_SCP_IRQ_CONNECT, (), inst)          \
	}

#define MBOX_RCAR_MFIS_COMMON_INIT(inst)                                                           \
	MBOX_RCAR_MFIS_COMMON_ASSERTS(inst)                                                        \
	DT_INST_FOREACH_CHILD_STATUS_OKAY(inst, MBOX_RCAR_MFIS_ISR_DEFINE)                         \
	MBOX_RCAR_MFIS_COMMON_IRQ_CONFIGURE(inst)                                                  \
	static struct mbox_rcar_mfis_chan_data                                                     \
		mfis_common_channels_data_common_##inst[MBOX_RCAR_MFIS_COMMON_MAX_CHANNELS];       \
	static struct mfis_channel                                                                 \
		mfis_common_hal_channels_##inst[MBOX_RCAR_MFIS_COMMON_MAX_CHANNELS] = {            \
			DT_INST_FOREACH_CHILD_STATUS_OKAY(inst, MBOX_RCAR_MFIS_CHANNEL_INIT)};     \
	static const unsigned int mfis_common_irqs_##inst[MBOX_RCAR_MFIS_COMMON_MAX_CHANNELS] = {  \
		DT_INST_FOREACH_CHILD_STATUS_OKAY(inst, MBOX_RCAR_MFIS_COMMON_IRQ_INIT)};          \
	static const struct mbox_rcar_mfis_config mbox_rcar_mfis_common_##inst##_config = {        \
		.max_channels = MBOX_RCAR_MFIS_COMMON_MAX_CHANNELS,                                \
		.domain = MFIS_COMMON,                                                             \
		.channel_mask = MBOX_RCAR_MFIS_COMMON_CHANNEL_MASK(inst),                          \
		.irqs = mfis_common_irqs_##inst,                                                   \
		.irq_configure = mbox_rcar_mfis_common_##inst##_irq_configure,                     \
	};                                                                                         \
	static struct mbox_rcar_mfis_data mbox_rcar_mfis_common_##inst##_data = {                  \
		.channels = mfis_common_channels_data_common_##inst,                               \
		.common_channels = mfis_common_hal_channels_##inst,                                \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(inst, mbox_rcar_mfis_init, NULL,                                     \
			      &mbox_rcar_mfis_common_##inst##_data,                                \
			      &mbox_rcar_mfis_common_##inst##_config, PRE_KERNEL_1,                \
			      CONFIG_MBOX_INIT_PRIORITY, &mbox_rcar_mfis_driver_api);

DT_INST_FOREACH_STATUS_OKAY(MBOX_RCAR_MFIS_COMMON_INIT)

#undef DT_DRV_COMPAT

#define DT_DRV_COMPAT renesas_rcar_mfis_scp_mbox

#define MBOX_RCAR_MFIS_SCP_INIT(inst)                                                              \
	MBOX_RCAR_MFIS_SCP_ASSERTS(inst)                                                           \
	LISTIFY(DT_INST_NUM_IRQS(inst), MBOX_RCAR_MFIS_SCP_ISR_DEFINE, (), inst)                   \
	MBOX_RCAR_MFIS_SCP_IRQ_CONFIGURE(inst)                                                     \
	static struct mbox_rcar_mfis_chan_data                                                     \
		mfis_common_channels_data_scp_##inst[MBOX_RCAR_MFIS_SCP_MAX_CHANNELS];             \
	static const unsigned int mfis_scp_irqs_##inst[MBOX_RCAR_MFIS_SCP_MAX_CHANNELS] = {        \
		LISTIFY(DT_INST_NUM_IRQS(inst), MBOX_RCAR_MFIS_SCP_IRQ_INIT, (), inst)};           \
	static const struct mbox_rcar_mfis_config mbox_rcar_mfis_scp_##inst##_config = {           \
		.max_channels = MBOX_RCAR_MFIS_SCP_MAX_CHANNELS,                                   \
		.domain = MFIS_SCP,                                                                \
		.channel_mask = MBOX_RCAR_MFIS_SCP_CHANNEL_MASK(inst),                             \
		.irqs = mfis_scp_irqs_##inst,                                                      \
		.irq_configure = mbox_rcar_mfis_scp_##inst##_irq_configure,                        \
	};                                                                                         \
	static struct mbox_rcar_mfis_data mbox_rcar_mfis_scp_##inst##_data = {                     \
		.channels = mfis_common_channels_data_scp_##inst,                                  \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(inst, mbox_rcar_mfis_init, NULL, &mbox_rcar_mfis_scp_##inst##_data,  \
			      &mbox_rcar_mfis_scp_##inst##_config, PRE_KERNEL_1,                   \
			      CONFIG_MBOX_INIT_PRIORITY, &mbox_rcar_mfis_driver_api);

DT_INST_FOREACH_STATUS_OKAY(MBOX_RCAR_MFIS_SCP_INIT)

/* clang-format on */
