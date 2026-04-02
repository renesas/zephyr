/*
 * Copyright (c) 2026 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_PINCTRL_RENESAS_RH850U2C_PINCTRL_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_PINCTRL_RENESAS_RH850U2C_PINCTRL_H_

#include "pinctrl-rh850-common.h"

/* Port definition */
#define PORT_02                                                                                \
	(2 << RH850_PORT_PWE_POS | 0 << RH850_PORT_TYPE_POS | 2 << RH850_PORT_GROUP_POS |          \
	 0) /* IO P Group 02 */
#define PORT_03                                                                                \
	(3 << RH850_PORT_PWE_POS | 0 << RH850_PORT_TYPE_POS | 3 << RH850_PORT_GROUP_POS |          \
	 0) /* IO P Group 03 */
#define PORT_04                                                                                \
	(4 << RH850_PORT_PWE_POS | 0 << RH850_PORT_TYPE_POS | 4 << RH850_PORT_GROUP_POS |          \
	 0) /* IO P Group 04 */
#define PORT_06                                                                                \
	(6 << RH850_PORT_PWE_POS | 0 << RH850_PORT_TYPE_POS | 6 << RH850_PORT_GROUP_POS |          \
	 0) /* IO P Group 06 */
#define PORT_08                                                                                \
	(7 << RH850_PORT_PWE_POS | 0 << RH850_PORT_TYPE_POS | 8 << RH850_PORT_GROUP_POS |          \
	 0) /* IO P Group 08 */

#define PORT_10                                                                                \
	(8 << RH850_PORT_PWE_POS | 0 << RH850_PORT_TYPE_POS | 10 << RH850_PORT_GROUP_POS |         \
	 0) /* IO P Group 10 */
#define PORT_17                                                                                \
	(10 << RH850_PORT_PWE_POS | 0 << RH850_PORT_TYPE_POS | 17 << RH850_PORT_GROUP_POS |        \
	 0) /* IO P Group 17 */

#define PORT_20                                                                                \
	(12 << RH850_PORT_PWE_POS | 0 << RH850_PORT_TYPE_POS | 20 << RH850_PORT_GROUP_POS |        \
	 0) /* IO P Group 20 */
#define PORT_21                                                                                \
	(13 << RH850_PORT_PWE_POS | 0 << RH850_PORT_TYPE_POS | 21 << RH850_PORT_GROUP_POS |        \
	 0) /* IO P Group 21 */
#define PORT_22                                                                                \
	(14 << RH850_PORT_PWE_POS | 0 << RH850_PORT_TYPE_POS | 22 << RH850_PORT_GROUP_POS |        \
	 0) /* IO P Group 22 */
#define PORT_24                                                                                \
	(15 << RH850_PORT_PWE_POS | 0 << RH850_PORT_TYPE_POS | 24 << RH850_PORT_GROUP_POS |        \
	 0) /* IO P Group 24 */
#define PORT_24                                                                                \
	(16 << RH850_PORT_PWE_POS | 0 << RH850_PORT_TYPE_POS | 27 << RH850_PORT_GROUP_POS |        \
	 0) /* IO P Group 27 */

#define PORT_AP00                                                                              \
	(20 << RH850_PORT_PWE_POS | 2 << RH850_PORT_TYPE_POS | 0 << RH850_PORT_GROUP_POS |         \
	 0) /* IO port AP00 */
#define PORT_AP01                                                                              \
	(21 << RH850_PORT_PWE_POS | 2 << RH850_PORT_TYPE_POS | 1 << RH850_PORT_GROUP_POS |         \
	 0) /* IO port AP01 */
#define PORT_AP02                                                                              \
	(22 << RH850_PORT_PWE_POS | 2 << RH850_PORT_TYPE_POS | 2 << RH850_PORT_GROUP_POS |         \
	 0) /* IO port AP02 */
#define PORT_AP03                                                                              \
	(23 << RH850_PORT_PWE_POS | 2 << RH850_PORT_TYPE_POS | 3 << RH850_PORT_GROUP_POS |         \
	 0) /* IO port AP03 */
#define PORT_AP04                                                                              \
	(24 << RH850_PORT_PWE_POS | 2 << RH850_PORT_TYPE_POS | 4 << RH850_PORT_GROUP_POS |         \
	 0) /* IO port AP04 */

#define PORT_JP00                                                                              \
	(255 << RH850_PORT_PWE_POS | 1 << RH850_PORT_TYPE_POS | 0 << RH850_PORT_GROUP_POS |        \
	 0) /* IO port JP00 */

#define PORT_IP00                                                                              \
	(26 << RH850_PORT_PWE_POS | 3 << RH850_PORT_TYPE_POS | 0 << RH850_PORT_GROUP_POS |         \
	 0) /* IO port IP00 */


#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_PINCTRL_RENESAS_RH850U2C_PINCTRL_H_ */
