#include <zephyr/devicetree.h>

#if DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(option_setting_mem_cfg_sec))

#if DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(option_setting_dualsel))
SECTION_DATA_PROLOGUE(.option_setting_dualsel, DT_REG_ADDR(DT_NODELABEL(option_setting_dualsel)),)
{
	KEEP(*(.option_setting_dualsel))
} GROUP_LINK_IN(OPTION_SETTING_DUALSEL)
#endif

#if DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(option_setting_sas))
SECTION_DATA_PROLOGUE(.option_setting_sas, DT_REG_ADDR(DT_NODELABEL(option_setting_sas)),)
{
	KEEP(*(.option_setting_sas))
} GROUP_LINK_IN(OPTION_SETTING_SAS)
#endif

#if DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(option_setting_sas2))
SECTION_DATA_PROLOGUE(.option_setting_sas2, DT_REG_ADDR(DT_NODELABEL(option_setting_sas2)),)
{
	KEEP(*(.option_setting_sas2))
} GROUP_LINK_IN(OPTION_SETTING_SAS2)
#endif

#if DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(option_setting_banksel_sec))
SECTION_DATA_PROLOGUE(.option_setting_banksel_sec, DT_REG_ADDR(DT_NODELABEL(option_setting_banksel_sec)),)
{
	KEEP(*(.option_setting_banksel_sec))
} GROUP_LINK_IN(OPTION_SETTING_BANKSEL_SEC)
#endif

#if DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(option_setting_blsel_sec))
SECTION_DATA_PROLOGUE(.option_setting_blsel_sec, DT_REG_ADDR(DT_NODELABEL(option_setting_blsel_sec)),)
{
	KEEP(*(.option_setting_blsel_sec))
} GROUP_LINK_IN(OPTION_SETTING_BLSEL_SEC)
#endif

#if DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(option_setting_banksel_sel))
SECTION_DATA_PROLOGUE(.option_setting_banksel_sel, DT_REG_ADDR(DT_NODELABEL(option_setting_banksel_sel)),)
{
	KEEP(*(.option_setting_banksel_sel))
} GROUP_LINK_IN(OPTION_SETTING_BANKSEL_SEL)
#endif

#if DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(option_setting_blsel_sel))
SECTION_DATA_PROLOGUE(.option_setting_blsel_sel, DT_REG_ADDR(DT_NODELABEL(option_setting_blsel_sel)),)
{
	KEEP(*(.option_setting_blsel_sel))
} GROUP_LINK_IN(OPTION_SETTING_BLSEL_SEL)
#endif

#if DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(option_setting_bpsdm_sec))
SECTION_DATA_PROLOGUE(.option_setting_bpsdm_sec, DT_REG_ADDR(DT_NODELABEL(option_setting_bpsdm_sec)),)
{
	KEEP(*(.option_setting_bpsdm_sec))
} GROUP_LINK_IN(OPTION_SETTING_BPSDM_SEC)
#endif

#if DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(option_setting_bps_sec))
SECTION_DATA_PROLOGUE(.option_setting_bps_sec, DT_REG_ADDR(DT_NODELABEL(option_setting_bps_sec)),)
{
	KEEP(*(.option_setting_bps_sec))
} GROUP_LINK_IN(OPTION_SETTING_BPS_SEC)
#endif

#if DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(option_setting_bps_sec_dual))
SECTION_DATA_PROLOGUE(.option_setting_bps_sec_dual, DT_REG_ADDR(DT_NODELABEL(option_setting_bps_sec_dual)),)
{
	KEEP(*(.option_setting_bps_sec_dual))
} GROUP_LINK_IN(OPTION_SETTING_BPS_SEC_DUAL)
#endif

#endif


#if DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(option_setting_opt_sec))

#if DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(option_setting_data_flash_hoemrtpk))
SECTION_DATA_PROLOGUE(.option_setting_data_flash_hoemrtpk, DT_REG_ADDR(DT_NODELABEL(option_setting_data_flash_hoemrtpk)),)
{
	KEEP(*(.option_setting_data_flash_hoemrtpk))
} GROUP_LINK_IN(OPTION_SETTING_DATA_FLASH_HOEMRTPK)
#endif

#if DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(option_setting_data_flash_fsblctrl0))
SECTION_DATA_PROLOGUE(.option_setting_data_flash_fsblctrl0, DT_REG_ADDR(DT_NODELABEL(option_setting_data_flash_fsblctrl0)),)
{
	KEEP(*(.option_setting_data_flash_fsblctrl0))
} GROUP_LINK_IN(OPTION_SETTING_DATA_FLASH_FSBLCTRL0)
#endif

#if DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(option_setting_data_flash_fsblctrl1))
SECTION_DATA_PROLOGUE(.option_setting_data_flash_fsblctrl1, DT_REG_ADDR(DT_NODELABEL(option_setting_data_flash_fsblctrl1)),)
{
	KEEP(*(.option_setting_data_flash_fsblctrl1))
} GROUP_LINK_IN(OPTION_SETTING_DATA_FLASH_FSBLCTRL1)
#endif

#if DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(option_setting_data_flash_fsblctrl2))
SECTION_DATA_PROLOGUE(.option_setting_data_flash_fsblctrl2, DT_REG_ADDR(DT_NODELABEL(option_setting_data_flash_fsblctrl2)),)
{
	KEEP(*(.option_setting_data_flash_fsblctrl2))
} GROUP_LINK_IN(OPTION_SETTING_DATA_FLASH_FSBLCTRL2)
#endif

#if DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(option_setting_data_flash_samr))
SECTION_DATA_PROLOGUE(.option_setting_data_flash_samr, DT_REG_ADDR(DT_NODELABEL(option_setting_data_flash_samr)),)
{
	KEEP(*(.option_setting_data_flash_samr))
} GROUP_LINK_IN(OPTION_SETTING_DATA_FLASH_SAMR)
#endif

#if DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(option_setting_data_flash_sacc0))
SECTION_DATA_PROLOGUE(.option_setting_data_flash_sacc0, DT_REG_ADDR(DT_NODELABEL(option_setting_data_flash_sacc0)),)
{
	KEEP(*(.option_setting_data_flash_sacc0))
} GROUP_LINK_IN(OPTION_SETTING_DATA_FLASH_SACC0)
#endif

#if DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(option_setting_data_flash_sacc10))
SECTION_DATA_PROLOGUE(.option_setting_data_flash_sacc10, DT_REG_ADDR(DT_NODELABEL(option_setting_data_flash_sacc10)),)
{
	KEEP(*(.option_setting_data_flash_sacc10))
} GROUP_LINK_IN(OPTION_SETTING_DATA_FLASH_SACC10)
#endif

#if DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(option_setting_data_flash_sacc1))
SECTION_DATA_PROLOGUE(.option_setting_data_flash_sacc1, DT_REG_ADDR(DT_NODELABEL(option_setting_data_flash_sacc1)),)
{
	KEEP(*(.option_setting_data_flash_sacc1))
} GROUP_LINK_IN(OPTION_SETTING_DATA_FLASH_SACC1)
#endif

#if DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(option_setting_data_flash_sacc11))
SECTION_DATA_PROLOGUE(.option_setting_data_flash_sacc11, DT_REG_ADDR(DT_NODELABEL(option_setting_data_flash_sacc11)),)
{
	KEEP(*(.option_setting_data_flash_sacc11))
} GROUP_LINK_IN(OPTION_SETTING_DATA_FLASH_SACC11)
#endif

#if DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(option_setting_data_flash_sacc2))
SECTION_DATA_PROLOGUE(.option_setting_data_flash_sacc2, DT_REG_ADDR(DT_NODELABEL(option_setting_data_flash_sacc2)),)
{
	KEEP(*(.option_setting_data_flash_sacc2))
} GROUP_LINK_IN(OPTION_SETTING_DATA_FLASH_SACC2)
#endif

#if DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(option_setting_data_flash_sacc12))
SECTION_DATA_PROLOGUE(.option_setting_data_flash_sacc12, DT_REG_ADDR(DT_NODELABEL(option_setting_data_flash_sacc12)),)
{
	KEEP(*(.option_setting_data_flash_sacc12))
} GROUP_LINK_IN(OPTION_SETTING_DATA_FLASH_SACC12)
#endif

#if DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(option_setting_data_flash_sacc3))
SECTION_DATA_PROLOGUE(.option_setting_data_flash_sacc3, DT_REG_ADDR(DT_NODELABEL(option_setting_data_flash_sacc3)),)
{
	KEEP(*(.option_setting_data_flash_sacc3))
} GROUP_LINK_IN(OPTION_SETTING_DATA_FLASH_SACC3)
#endif

#if DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(option_setting_data_flash_sacc13))
SECTION_DATA_PROLOGUE(.option_setting_data_flash_sacc13, DT_REG_ADDR(DT_NODELABEL(option_setting_data_flash_sacc13)),)
{
	KEEP(*(.option_setting_data_flash_sacc13))
} GROUP_LINK_IN(OPTION_SETTING_DATA_FLASH_SACC13)
#endif

#if DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(option_setting_gpotp))
SECTION_DATA_PROLOGUE(.option_setting_gpotp, DT_REG_ADDR(DT_NODELABEL(option_setting_gpotp)),)
{
	KEEP(*(.option_setting_gpotp))
} GROUP_LINK_IN(OPTION_SETTING_GPOTP)
#endif

#if DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(option_setting_ofs0))
SECTION_DATA_PROLOGUE(.option_setting_ofs0, DT_REG_ADDR(DT_NODELABEL(option_setting_ofs0)),)
{
	KEEP(*(.option_setting_ofs0))
} GROUP_LINK_IN(OPTION_SETTING_OFS0)
#endif

#if DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(option_setting_ofs2))
SECTION_DATA_PROLOGUE(.option_setting_ofs2, DT_REG_ADDR(DT_NODELABEL(option_setting_ofs2)),)
{
	KEEP(*(.option_setting_ofs2))
} GROUP_LINK_IN(OPTION_SETTING_OFS2)
#endif

#if DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(option_setting_mde))
SECTION_DATA_PROLOGUE(.option_setting_mde, DT_REG_ADDR(DT_NODELABEL(option_setting_mde)),)
{
	KEEP(*(.option_setting_mde))
} GROUP_LINK_IN(OPTION_SETTING_MDE)
#endif

#if DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(option_setting_ofs1_sec))
SECTION_DATA_PROLOGUE(.option_setting_ofs1_sec, DT_REG_ADDR(DT_NODELABEL(option_setting_ofs1_sec)),)
{
	KEEP(*(.option_setting_ofs1_sec))
} GROUP_LINK_IN(OPTION_SETTING_OFS1_SEC)
#endif

#if DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(option_setting_ofs3_sec))
SECTION_DATA_PROLOGUE(.option_setting_ofs3_sec, DT_REG_ADDR(DT_NODELABEL(option_setting_ofs3_sec)),)
{
	KEEP(*(.option_setting_ofs3_sec))
} GROUP_LINK_IN(OPTION_SETTING_OFS3_SEC)
#endif

#if DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(option_setting_ofs1_sel))
SECTION_DATA_PROLOGUE(.option_setting_ofs1_sel, DT_REG_ADDR(DT_NODELABEL(option_setting_ofs1_sel)),)
{
	KEEP(*(.option_setting_ofs1_sel))
} GROUP_LINK_IN(OPTION_SETTING_OFS1_SEL)
#endif

#if DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(option_setting_ofs3_sel))
SECTION_DATA_PROLOGUE(.option_setting_ofs3_sel, DT_REG_ADDR(DT_NODELABEL(option_setting_ofs3_sel)),)
{
	KEEP(*(.option_setting_ofs3_sel))
} GROUP_LINK_IN(OPTION_SETTING_OFS3_SEL)
#endif

#if DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(option_setting_pbps_sec))
SECTION_DATA_PROLOGUE(.option_setting_pbps_sec, DT_REG_ADDR(DT_NODELABEL(option_setting_pbps_sec)),)
{
	KEEP(*(.option_setting_pbps_sec))
} GROUP_LINK_IN(OPTION_SETTING_PBPS_SEC)
#endif

#if DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(option_setting_pbps_sec_dual))
SECTION_DATA_PROLOGUE(.option_setting_pbps_sec_dual, DT_REG_ADDR(DT_NODELABEL(option_setting_pbps_sec_dual)),)
{
	KEEP(*(.option_setting_pbps_sec_dual))
} GROUP_LINK_IN(OPTION_SETTING_PBPS_SEC_DUAL)
#endif

#if DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(option_setting_pbpsdm_sec))
SECTION_DATA_PROLOGUE(.option_setting_pbpsdm_sec, DT_REG_ADDR(DT_NODELABEL(option_setting_pbpsdm_sec)),)
{
	KEEP(*(.option_setting_pbpsdm_sec))
} GROUP_LINK_IN(OPTION_SETTING_PBPSDM_SEC)
#endif

#if DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(option_setting_revoke))
SECTION_DATA_PROLOGUE(.option_setting_revoke, DT_REG_ADDR(DT_NODELABEL(option_setting_revoke)),)
{
	KEEP(*(.option_setting_revoke))
} GROUP_LINK_IN(OPTION_SETTING_REVOKE)
#endif

#if DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(option_setting_zhuk))
SECTION_DATA_PROLOGUE(.option_setting_zhuk, DT_REG_ADDR(DT_NODELABEL(option_setting_zhuk)),)
{
	KEEP(*(.option_setting_zhuk))
} GROUP_LINK_IN(OPTION_SETTING_ZHUK)
#endif

#if DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(option_setting_arccs))
SECTION_DATA_PROLOGUE(.option_setting_arccs, DT_REG_ADDR(DT_NODELABEL(option_setting_arccs)),)
{
	KEEP(*(.option_setting_arccs))
} GROUP_LINK_IN(OPTION_SETTING_ARCCS)
#endif

#if DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(option_setting_ubcsr))
SECTION_DATA_PROLOGUE(.option_setting_ubcsr, DT_REG_ADDR(DT_NODELABEL(option_setting_ubcsr)),)
{
	KEEP(*(.option_setting_ubcsr))
} GROUP_LINK_IN(OPTION_SETTING_UBCSR)
#endif

#if DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(option_setting_ofsps))
SECTION_DATA_PROLOGUE(.option_setting_ofsps, DT_REG_ADDR(DT_NODELABEL(option_setting_ofsps)),)
{
	KEEP(*(.option_setting_ofsps))
} GROUP_LINK_IN(OPTION_SETTING_OFSPS)
#endif

#if DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(option_setting_ofsps2))
SECTION_DATA_PROLOGUE(.option_setting_ofsps2, DT_REG_ADDR(DT_NODELABEL(option_setting_ofsps2)),)
{
	KEEP(*(.option_setting_ofsps2))
} GROUP_LINK_IN(OPTION_SETTING_OFSPS2)
#endif

#endif

#if DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(option_setting_anti_rollback_sec))

#if DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(option_setting_arcs))
SECTION_DATA_PROLOGUE(.option_setting_arcs, DT_REG_ADDR(DT_NODELABEL(option_setting_arcs)),)
{
	KEEP(*(.option_setting_arcs))
} GROUP_LINK_IN(OPTION_SETTING_ARCS)
#endif

#if DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(option_setting_arcnsn))
SECTION_DATA_PROLOGUE(.option_setting_arcnsn, DT_REG_ADDR(DT_NODELABEL(option_setting_arcnsn)),)
{
	KEEP(*(.option_setting_arcnsn))
} GROUP_LINK_IN(OPTION_SETTING_ARCNSN)
#endif

#if DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(option_setting_arcoembl))
SECTION_DATA_PROLOGUE(.option_setting_arcoembl, DT_REG_ADDR(DT_NODELABEL(option_setting_arcoembl)),)
{
	KEEP(*(.option_setting_arcoembl_arcnsn))
} GROUP_LINK_IN(OPTION_SETTING_ARCOEMBL_ARCNSN)
#endif

#endif
