/*
 * Copyright (c) 2026 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

GROUP_START(BACKUP_BUFFER)

	SECTION_PROLOGUE(_RCAR_BACKUP_BUFFER_SECTION_NAME, (NOLOAD),)
	{
		*(.rcar_backup_buffer)
		*(".rcar_backup_buffer.*")
	} GROUP_LINK_IN(BACKUP_BUFFER)

GROUP_END(BACKUP_BUFFER)
