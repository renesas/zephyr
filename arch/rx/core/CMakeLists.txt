# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources(
  switch.S
  cpu_idle.c
  prep_c.c
  irq_manage.c
  reset.S
  thread.c
  vects.c
  isr_exit.S
  fatal.c
  reboot.c
  nmi.c
)

zephyr_library_sources_ifdef(CONFIG_IRQ_OFFLOAD irq_offload.c)
zephyr_library_sources_ifdef(CONFIG_LLEXT elf.c)
zephyr_library_sources_ifdef(CONFIG_RX_MPU mpu.c)
zephyr_library_sources_ifdef(CONFIG_ARCH_CACHE cache.c)
zephyr_library_sources_ifdef(CONFIG_DEBUG_COREDUMP coredump.c)
