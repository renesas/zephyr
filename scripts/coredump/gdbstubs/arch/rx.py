#!/usr/bin/env python3
#
# Copyright (c) 2026 Renesas Electronics Corporation
#
# SPDX-License-Identifier: Apache-2.0

import binascii
import logging
import struct

from gdbstubs.gdbstub import GdbStub

logger = logging.getLogger("gdbstub")


class RegNum:
    """Native RX register numbers used by RX GDB.

      0-15: R0-R15 (R0 is the active stack pointer)
      16: USP (User Stack Pointer)
      17: ISP (Interrupt Stack Pointer)
      18: PSW
      19: PC
    """

    R0 = 0
    R1 = 1
    R2 = 2
    R3 = 3
    R4 = 4
    R5 = 5
    R6 = 6
    R7 = 7
    R8 = 8
    R9 = 9
    R10 = 10
    R11 = 11
    R12 = 12
    R13 = 13
    R14 = 14
    R15 = 15

    USP = 16
    ISP = 17
    PSW = 18
    PC = 19


_ACC_FIELD_NAMES = {
    1: ["acc_l", "acc_h"],  # RXv1: one 64-bit accumulator
    2: ["acc0_l", "acc0_h", "acc0_g", "acc1_l", "acc1_h", "acc1_g"],  # RXv2/v3/v3e
}


class GdbStub_RX(GdbStub):
    """GDB stub for the Renesas RX architecture.

    The architecture block layout is selected by the header version set in
    arch/rx/core/coredump.c: 1 for RXv1 (one 64-bit accumulator) and 2 for
    RXv2/RXv3/RXv3e (two 72-bit accumulators).

    """

    ARCH_DATA_BLK_VERSIONS = tuple(_ACC_FIELD_NAMES)

    GDB_SIGNAL_DEFAULT = 4  # SIGILL

    # R0-R15, USP, ISP, PSW and PC. USP/ISP are unavailable in every version.
    GDB_G_PKT_NUM_REGS = 20

    def __init__(self, logfile, elffile):
        super().__init__(logfile=logfile, elffile=elffile)

        self.registers = {}
        self.accumulators = {}
        self.gdb_signal = self.GDB_SIGNAL_DEFAULT

        self.parse_arch_data_block()

    def parse_arch_data_block(self):
        """Parse the RX architecture block, using hdr_ver to pick the layout."""
        arch_data = self.logfile.get_arch_data()

        if arch_data is None:
            logger.error("RX architecture data block is missing")
            return

        arch_data_blk = arch_data["data"]
        arch_data_ver = arch_data["hdr_ver"]

        if arch_data_ver not in self.ARCH_DATA_BLK_VERSIONS:
            logger.error(
                "Unsupported RX architecture block version: %d",
                arch_data_ver,
            )
            return

        field_names = ["sp"] + _ACC_FIELD_NAMES[arch_data_ver] + [f"r{i}" for i in range(1, 16)]
        field_names += ["pc", "psw"]

        arch_data_blk_struct = "<" + ("I" * len(field_names))
        arch_data_blk_size = struct.calcsize(arch_data_blk_struct)

        if len(arch_data_blk) != arch_data_blk_size:
            logger.error(
                "Invalid RX architecture block size: got %d bytes, expected %d "
                "(header version %d)",
                len(arch_data_blk),
                arch_data_blk_size,
                arch_data_ver,
            )
            return

        values = dict(zip(field_names, struct.unpack(arch_data_blk_struct, arch_data_blk)))

        self.registers[RegNum.R0] = values["sp"]

        # Not exposed to GDB, see the class docstring.
        for name in _ACC_FIELD_NAMES[arch_data_ver]:
            self.accumulators[name] = values[name]

        for reg_num in range(RegNum.R1, RegNum.R15 + 1):
            self.registers[reg_num] = values[f"r{reg_num}"]

        self.registers[RegNum.PC] = values["pc"]
        self.registers[RegNum.PSW] = values["psw"]

        logger.debug(
            "RX block parsed (hdr_ver=%d): R0=0x%08x PC=0x%08x PSW=0x%08x",
            arch_data_ver,
            self.registers[RegNum.R0],
            self.registers[RegNum.PC],
            self.registers[RegNum.PSW],
        )

    @staticmethod
    def _encode_u32(value):
        """Encode a 32-bit register value for the GDB remote protocol."""
        return binascii.hexlify(struct.pack("<I", value))

    def send_registers_packet(self, registers):
        """Send R0-R15, USP, ISP, PSW and PC for the GDB 'g' packet."""
        packet = bytearray()

        for reg_num in range(self.GDB_G_PKT_NUM_REGS):
            if reg_num in registers:
                packet.extend(self._encode_u32(registers[reg_num]))
            else:
                # Eight 'x' characters mark an unavailable 32-bit register.
                packet.extend(b"x" * 8)

        self.put_gdb_packet(bytes(packet))

    def handle_register_group_read_packet(self):
        """Handle the GDB 'g' packet."""
        self.send_registers_packet(self.registers)

    def handle_register_single_read_packet(self, pkt):
        """Handle the GDB 'p' packet for supported 32-bit registers."""
        pkt_str = pkt.decode("ascii")

        try:
            reg_num = int(pkt_str[1:], 16)
        except ValueError:
            logger.warning("Malformed RX register-read packet: %s", pkt_str)
            self.put_gdb_packet(b"")
            return

        if reg_num in self.registers:
            self.put_gdb_packet(self._encode_u32(self.registers[reg_num]))
        else:
            self.put_gdb_packet(b"x" * 8)

    def handle_register_single_write_packet(self, pkt):
        """Reject register writes because a coredump is read-only."""
        logger.debug(
            "Ignoring RX register-write packet for read-only coredump: %r",
            pkt,
        )
        self.put_gdb_packet(b"E01")

    def arch_supports_thread_operations(self):
        """Thread-aware GDB operations are not supported by this stub."""
        return False
