/*
 * Copyright (c) 2026 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_PINCTRL_RENESAS_RH850U2A_PINCTRL_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_PINCTRL_RENESAS_RH850U2A_PINCTRL_H_

#include "pinctrl-rh850-common.h"

/* Port definition */
#define PORT_00                                                                               \
	(1 << RH850_PORT_PWE_POS | 0 << RH850_PORT_TYPE_POS | 0 << RH850_PORT_GROUP_POS |         \
	 0) /* IO P Group 00 */
#define PORT_01                                                                               \
	(2 << RH850_PORT_PWE_POS | 0 << RH850_PORT_TYPE_POS | 1 << RH850_PORT_GROUP_POS |         \
	 0) /* IO P Group 01 */
#define PORT_02                                                                               \
	(3 << RH850_PORT_PWE_POS | 0 << RH850_PORT_TYPE_POS | 2 << RH850_PORT_GROUP_POS |         \
	 0) /* IO P Group 02 */
#define PORT_03                                                                               \
	(4 << RH850_PORT_PWE_POS | 0 << RH850_PORT_TYPE_POS | 3 << RH850_PORT_GROUP_POS |         \
	 0) /* IO P Group 03 */
#define PORT_04                                                                               \
	(5 << RH850_PORT_PWE_POS | 0 << RH850_PORT_TYPE_POS | 4 << RH850_PORT_GROUP_POS |         \
	 0) /* IO P Group 04 */
#define PORT_05                                                                               \
	(6 << RH850_PORT_PWE_POS | 0 << RH850_PORT_TYPE_POS | 5 << RH850_PORT_GROUP_POS |         \
	 0) /* IO P Group 05 */
#define PORT_06                                                                               \
	(7 << RH850_PORT_PWE_POS | 0 << RH850_PORT_TYPE_POS | 6 << RH850_PORT_GROUP_POS |         \
	 0) /* IO P Group 06 */
#define PORT_08                                                                               \
	(8 << RH850_PORT_PWE_POS | 0 << RH850_PORT_TYPE_POS | 8 << RH850_PORT_GROUP_POS |         \
	 0) /* IO P Group 08 */
#define PORT_09                                                                               \
	(9 << RH850_PORT_PWE_POS | 0 << RH850_PORT_TYPE_POS | 9 << RH850_PORT_GROUP_POS |         \
	 0) /* IO P Group 09 */

#define PORT_10                                                                               \
	(10 << RH850_PORT_PWE_POS | 0 << RH850_PORT_TYPE_POS | 10 << RH850_PORT_GROUP_POS |       \
	 0) /* IO P Group 10 */
#define PORT_11                                                                               \
	(11 << RH850_PORT_PWE_POS | 0 << RH850_PORT_TYPE_POS | 11 << RH850_PORT_GROUP_POS |       \
	 0) /* IO P Group 11 */
#define PORT_12                                                                               \
	(12 << RH850_PORT_PWE_POS | 0 << RH850_PORT_TYPE_POS | 12 << RH850_PORT_GROUP_POS |       \
	 0) /* IO P Group 12 */
#define PORT_17                                                                               \
	(13 << RH850_PORT_PWE_POS | 0 << RH850_PORT_TYPE_POS | 17 << RH850_PORT_GROUP_POS |       \
	 0) /* IO P Group 17 */
#define PORT_18                                                                               \
	(14 << RH850_PORT_PWE_POS | 0 << RH850_PORT_TYPE_POS | 18 << RH850_PORT_GROUP_POS |       \
	 0) /* IO P Group 18 */
#define PORT_19                                                                               \
	(15 << RH850_PORT_PWE_POS | 0 << RH850_PORT_TYPE_POS | 19 << RH850_PORT_GROUP_POS |       \
	 0) /* IO P Group 19 */

#define PORT_20                                                                               \
	(16 << RH850_PORT_PWE_POS | 0 << RH850_PORT_TYPE_POS | 20 << RH850_PORT_GROUP_POS |       \
	 0) /* IO P Group 20 */
#define PORT_21                                                                               \
	(17 << RH850_PORT_PWE_POS | 0 << RH850_PORT_TYPE_POS | 21 << RH850_PORT_GROUP_POS |       \
	 0) /* IO P Group 21 */
#define PORT_22                                                                               \
	(18 << RH850_PORT_PWE_POS | 0 << RH850_PORT_TYPE_POS | 22 << RH850_PORT_GROUP_POS |       \
	 0) /* IO P Group 22 */
#define PORT_23                                                                               \
	(19 << RH850_PORT_PWE_POS | 0 << RH850_PORT_TYPE_POS | 23 << RH850_PORT_GROUP_POS |       \
	 0) /* IO P Group 23 */
#define PORT_24                                                                               \
	(20 << RH850_PORT_PWE_POS | 0 << RH850_PORT_TYPE_POS | 24 << RH850_PORT_GROUP_POS |       \
	 0) /* IO P Group 24 */

#define PORT_JP00                                                                             \
	(255 << RH850_PORT_PWE_POS | 1 << RH850_PORT_TYPE_POS | 0 << RH850_PORT_GROUP_POS |       \
	 0) /* IO JP Group 00 */

#define PORT_AP00                                                                             \
	(21 << RH850_PORT_PWE_POS | 2 << RH850_PORT_TYPE_POS | 0 << RH850_PORT_GROUP_POS |        \
	 0) /* IO AP Group 00 */

#define PORT_AP01                                                                             \
	(22 << RH850_PORT_PWE_POS | 2 << RH850_PORT_TYPE_POS | 1 << RH850_PORT_GROUP_POS |        \
	 0) /* IO AP Group 01 */

#define PORT_AP02                                                                             \
	(23 << RH850_PORT_PWE_POS | 2 << RH850_PORT_TYPE_POS | 2 << RH850_PORT_GROUP_POS |        \
	 0) /* IO AP Group 02 */

#define PORT_AP03                                                                             \
	(24 << RH850_PORT_PWE_POS | 2 << RH850_PORT_TYPE_POS | 3 << RH850_PORT_GROUP_POS |        \
	 0) /* IO AP Group 03 */

#define PORT_AP04                                                                             \
	(25 << RH850_PORT_PWE_POS | 2 << RH850_PORT_TYPE_POS | 4 << RH850_PORT_GROUP_POS |        \
	 0) /* IO AP Group 04 */

#define PORT_AP05                                                                             \
	(26 << RH850_PORT_PWE_POS | 2 << RH850_PORT_TYPE_POS | 5 << RH850_PORT_GROUP_POS |        \
	 0) /* IO AP Group 05 */

#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_PINCTRL_RENESAS_RH850U2A_PINCTRL_H_ */
