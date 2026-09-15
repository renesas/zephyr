/*
 * Copyright (c) 2026 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/firmware/scmi/renesas/reset.h>

DT_SCMI_PROTOCOL_DEFINE_NODEV(DT_INST(0, renesas_scmi_reset_status), NULL,
			      SCMI_RENESAS_VENDOR_PROTOCOL_SUPPORTED_VERSION);

struct scmi_renesas_reset_status_request {
	uint32_t domain_id;
} __packed;

struct scmi_renesas_reset_status_reply {
	int32_t status;
	uint32_t reset_status;
} __packed;

int scmi_renesas_reset_status_get(uint32_t domain_id, bool *asserted)
{
	struct scmi_protocol *proto = &SCMI_PROTOCOL_NAME(SCMI_PROTOCOL_RENESAS_VENDOR);
	struct scmi_renesas_reset_status_request request = {
		.domain_id = domain_id,
	};
	struct scmi_renesas_reset_status_reply response;
	struct scmi_message msg;
	struct scmi_message reply;
	int ret;

	if (asserted == NULL || proto->id != SCMI_PROTOCOL_RENESAS_VENDOR) {
		return -EINVAL;
	}

	msg.hdr = SCMI_MESSAGE_HDR_MAKE(SCMI_RENESAS_VENDOR_MSG_RESET_GET_STATUS, SCMI_COMMAND,
					proto->id, 0U);
	msg.len = sizeof(request);
	msg.content = &request;

	reply.hdr = msg.hdr;
	reply.len = sizeof(response);
	reply.content = &response;

	ret = scmi_send_message(proto, &msg, &reply, false);
	if (ret < 0) {
		return ret;
	}

	if (response.status != SCMI_SUCCESS) {
		return scmi_status_to_errno(response.status);
	}

	if (response.reset_status > SCMI_RENESAS_VENDOR_RESET_RELEASED) {
		return -EPROTO;
	}

	/* SCP returns 0 for asserted and 1 for released. */
	*asserted = response.reset_status == SCMI_RENESAS_VENDOR_RESET_ASSERTED;

	return 0;
}
