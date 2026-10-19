# MRAM dispatcher with signed OSPI code

This sample shows how to **build one Zephyr image, split it into two signed
binaries, and have software verify each binary on its own** before running it.
It targets the Renesas EK-RA8M2 (CM85), which has on-chip MRAM and an external
octal OSPI flash. Both signatures use **ML-DSA-87** (FIPS 204).

## Contents

- [What this sample demonstrates](#what-this-sample-demonstrates)
- [Requirements](#requirements)
- [Guideline](#guideline)
  - [Step 1: generate the ML-DSA-87 keys](#step-1-generate-the-ml-dsa-87-keys)
  - [Step 2: build](#step-2-build)
  - [Step 3: look at the output binaries](#step-3-look-at-the-output-binaries)
  - [Step 4 (optional): check the signatures on the host](#step-4-optional-check-the-signatures-on-the-host)
  - [Step 5: flash](#step-5-flash)
  - [Step 6: run and observe](#step-6-run-and-observe)
- [How the split works](#how-the-split-works)
- [Memory map](#memory-map)
- [What each verifier checks, and what it does not](#what-each-verifier-checks-and-what-it-does-not)
- [Configuration reference](#configuration-reference)
- [Files](#files)
- [Troubleshooting](#troubleshooting)

## What this sample demonstrates

One Zephyr application is linked once into a single ELF. A post-link step then
cuts that ELF into **two binaries**, each with its own signature and its own
verifier:

```
                     one Zephyr build (zephyr.elf)
                                 │
              ┌──────────────────┴───────────────────┐
              ▼                                      ▼
   Binary A: MRAM image                    Binary B: OSPI image
   kernel, vector table, main()            src/ext_code.c (XIP)
   signed with imgtool                     signed with scripts/ospi_sign.py
   key A (MCUboot key)                     key B (OSPI key)
              │                                      │
   verified by MCUboot at boot             verified by the MRAM app (main())
   (secure boot, ML-DSA-87)                before it calls into the OSPI code
              │                                      │
   zephyr_internal.signed.bin              zephyr_ospi.signed.bin
   -> MRAM  0x02010000                     -> OSPI  0x90000000
```

| | Binary A (MRAM) | Binary B (OSPI) |
|---|---|---|
| Contains | MCUboot-wrapped Zephyr app, including the verifier for B | the code of `ext_code.c`, executed in place |
| Signed by | `imgtool` (MCUboot key, key A) | `scripts/ospi_sign.py` (OSPI key, key B) |
| Verified by | MCUboot, before the app is booted | the MRAM app, before the OSPI code is called |
| Public key lives in | MCUboot image | the MRAM app (compiled into binary A) |
| File | `zephyr_internal.signed.bin` | `zephyr_ospi.signed.bin` |

Because MCUboot authenticates binary A, and binary A contains the public key for
binary B, trust in B is anchored in key A. The two keys are otherwise
independent and can be rotated separately.

Boot flow:

```
 reset -> MCUboot (MRAM 0x02000000)
            verifies binary A in slot0 ---fail--> do not boot it
          -> MRAM app (slot0 0x02010000)
            main(): verify binary B ------fail--> print message, do not run it
          -> function_in_ext_flash()   (executes in place from OSPI 0x90000000)
```

## Requirements

- A Zephyr workspace with the Renesas modules this branch uses (MCUboot with
  ML-DSA support, `mldsa-native`, TF-PSA-Crypto).
- `srec_cat` (package `srecord`) on the host. It pads, byte-swaps and merges the
  hex output.
- The `dilithium-py` Python package. `imgtool` and `scripts/ospi_sign.py` use it
  for ML-DSA-87. If you cannot install it into the west Python environment,
  install it elsewhere and add it to `PYTHONPATH`:

  ```sh
  pip install --target ~/.local/lib/zephyr-pylibs dilithium-py
  export PYTHONPATH=~/.local/lib/zephyr-pylibs
  ```

- The Renesas Flash Programmer (RFP), to program the MRAM and the external OSPI
  flash (see [Step 5](#step-5-flash)).

## Guideline

All commands run from the workspace root (the directory that contains `zephyr/`
and `bootloader/`).

### Step 1: generate the ML-DSA-87 keys

You need **two** keys, one per binary. Use the `imgtool` from the workspace's
MCUboot, because the copy installed from PyPI may not support ML-DSA:

```sh
IMGTOOL=bootloader/mcuboot/scripts/imgtool.py

python3 $IMGTOOL keygen -t mldsa-87 -k mcuboot_mldsa87.pem   # key A: signs binary A
python3 $IMGTOOL keygen -t mldsa-87 -k ospi_mldsa87.pem      # key B: signs binary B
```

Notes:

- Each file is a PKCS#8 PEM holding the 32-byte ML-DSA seed. Keep both private
  keys secret and out of version control. Losing key A means you can no longer
  sign images that MCUboot will accept.
- You can skip this step for a quick trial. Without keys the build uses
  MCUboot's public debug key for A and a throw-away key generated in the build
  directory for B, and it prints a warning for each. These are for development
  only. Do not ship them.
- The public keys are derived at build time. You do not need to extract them.

### Step 2: build

```sh
west build -p always --sysbuild -b ek_ra8m2/r7ka8m2jflcac/cm85 \
    zephyr/samples/boards/renesas/mram_ospi_dispatch -d build -- \
    -DSB_CONFIG_BOOT_SIGNATURE_KEY_FILE=\"$PWD/mcuboot_mldsa87.pem\" \
    -DOSPI_SIGN_KEY=$PWD/ospi_mldsa87.pem
```

- `SB_CONFIG_BOOT_SIGNATURE_KEY_FILE` is key A, used by MCUboot and `imgtool`.
- `OSPI_SIGN_KEY` is key B. A plain `-D` works with and without `--sysbuild`.
- Use absolute paths for the key files.

### Step 3: look at the output binaries

Paths are relative to the build directory (`build/`):

| File | What it is | Address to program |
|---|---|---|
| `mcuboot/zephyr/zephyr.hex` | MCUboot itself (not signed, it is the root of trust) | MRAM `0x02000000` |
| `mram_ospi_dispatch/zephyr_internal.signed.bin` | **binary A**, imgtool-signed (header, image, TLVs) | MRAM `0x02010000` (slot0) |
| `mram_ospi_dispatch/zephyr_ospi.signed.bin` | **binary B**, code plus ML-DSA signature, byte-swapped | OSPI offset 0, which is `0x90000000` |

The same images also exist as hex, which carries its own addresses:
`zephyr_internal.hex` and `zephyr_ospi.hex`. `zephyr_ospi_raw.hex` is binary B
without padding or byte-swap, for tools that swap by themselves.
`zephyr_merged.hex` is A and B together without MCUboot.

A `.bin` has no address, so enter the addresses above in your programmer (RFP).

Do not flash these: `mram_ospi_dispatch/zephyr/zephyr.bin` and `zephyr.hex` are
the unsigned dispatcher, which MCUboot rejects, and `mcuboot/zephyr/zephyr.bin`
is a 14 MB gap-filled file because of the option-byte sections.

### Step 4 (optional): check the signatures on the host

Before touching the board, you can check each signature separately.

Binary B, against the public key compiled into binary A:

```python
import re
from dilithium_py.ml_dsa import ML_DSA_87

d = "build/mram_ospi_dispatch/"
pk = bytes(int(x, 16) for x in re.findall(r"0x([0-9a-f]{2})", open(d + "ospi_pubkey.h").read()))
ok = ML_DSA_87.verify(pk, open(d + "ospi_text.bin", "rb").read(), open(d + "ospi_sig.bin", "rb").read())
print("binary B signature valid:", ok)
```

Binary A carries an `MLDSA87` TLV. Inspect it with:

```sh
python3 bootloader/mcuboot/scripts/imgtool.py dumpinfo \
    build/mram_ospi_dispatch/zephyr/zephyr.signed.bin
```

Flipping any bit of the signed bytes should make the matching check fail.
`imgtool verify` printed `Key type does not match TLV record` for binary A in
this setup; the cause was not investigated, and it may be a limitation of that
command with ML-DSA. A direct check of the signature over header, payload and
protected TLVs succeeded.

### Step 5: flash

Use the Renesas Flash Programmer (RFP). It programs the on-chip MRAM and, with
its external-flash feature, the octal OSPI flash. Program three images, each at
its own address:

| # | File (in `build/`) | Program to |
|---|---|---|
| 1 | `mcuboot/zephyr/zephyr.hex` | MRAM `0x02000000` |
| 2 | `mram_ospi_dispatch/zephyr_internal.signed.bin` (binary A) | MRAM `0x02010000` |
| 3 | `mram_ospi_dispatch/zephyr_ospi.signed.bin` (binary B) | external OSPI flash, offset 0 (`0x90000000`) |

Notes:

- A `.bin` has no address, so enter the start address for images 2 and 3 in RFP.
  For image 1 use the hex, which carries its own addresses. Do not use
  `mcuboot/zephyr/zephyr.bin`: it is a 14 MB gap-filled file.
- `zephyr_ospi.signed.bin` is already byte-swapped and padded for the octal read
  order. It was loaded into RFP as-is, with no further swap, and worked on the
  board. Do not apply another byte-swap in RFP.
- The three images are independent and can be programmed in any order or on
  their own. A missing or mismatching binary B does not brick the board: the MRAM
  app refuses to run it.
- If you change only key B or `ext_code.c`, binary A changes too, because the
  public key for B is compiled into it. Reprogram both binary A and binary B.
- The OSPI_B driver writes its calibration pattern to the last 4 KB of the OSPI
  flash on first use, if it is not already there. Leave that area alone when you
  program or erase the external flash.

The exact RFP menu steps and the external-flash settings depend on your RFP
version, so follow its user manual for those.

### Step 6: run and observe

Open the console (UART at 115200). Expected, in order:

1. MCUboot starts and, if binary A verifies, jumps to the MRAM app. If A does
   not verify, MCUboot reports an error and does not boot it.
2. The MRAM app prints `External flash image verified`, then the addresses of
   the functions it runs from MRAM, from OSPI flash and from SRAM.
3. If binary B is missing or does not match its signature, it prints
   `External flash image verification FAILED, not executing it` instead and
   does not call into the OSPI flash.

To see the second verifier reject something, flip one byte in
`zephyr_ospi.signed.bin` (or flash a binary B built with a different key B),
reflash only binary B, and reboot. MCUboot still boots binary A, and binary A
refuses the OSPI code.

## How the split works

### Placing code in the OSPI flash

`CMakeLists.txt` uses the normal code-relocation mechanism with a custom linker
script:

```cmake
zephyr_code_relocate(FILES src/ext_code.c LOCATION EXTFLASH_TEXT NOCOPY)  # XIP from OSPI
zephyr_code_relocate(FILES src/ext_code.c LOCATION RAM_DATA)              # data copied to RAM
zephyr_code_relocate(FILES src/sram_code.c LOCATION RAM)                  # code and data in SRAM
```

`NOCOPY` links the section at its OSPI address and does not copy it at boot. It
becomes the output section `.extflash_text_reloc`, with the symbols
`__extflash_text_reloc_start` and `__extflash_text_reloc_size`.
`linker_mram_ospi.ld` defines the `EXTFLASH` region at `0x90000000`, and
`ospi_sig.ld` adds a `.extflash_sig` section for the signature right behind the
code.

### Signing binary B

The signature covers linked bytes, so it cannot be part of the link. The
`split_flash` target runs after the final link and does this:

1. `objcopy -O binary -j .extflash_text_reloc` extracts the exact code bytes
   (`ospi_text.bin`).
2. `scripts/ospi_sign.py sign` signs them with ML-DSA-87 in pure mode and writes
   the 4627-byte signature (`ospi_sig.bin`). It reuses imgtool's key class, so
   key B is the same kind of PEM as key A.
3. `objcopy --update-section .extflash_sig=ospi_sig.bin` writes the signature into
   the reserved slot of a copy of the ELF (`zephyr_signed.elf`).
4. The OSPI sections are cut out, padded with `0xFF` to an even length (the
   total is odd), and byte-swapped in 16-bit units (`srec_cat -byte_swap 2`) for
   the octal read order. The signature is computed on the un-swapped bytes.

The public key for B is turned into `ospi_pubkey.h` at configure time and
compiled into binary A.

### Signing binary A

With `--sysbuild`, MCUboot is built with the ML-DSA signature type
(`sysbuild.conf`). Zephyr's own signing step runs `imgtool` on `zephyr.bin`.
Without help, that binary would also contain the OSPI sections and span
`0x02000000` to `0x90000000`, about 2 GB. `CMakeLists.txt` therefore adds
`--remove-section` flags for the two OSPI sections to the `bintools`
`elfconvert_flag` property, so the file imgtool signs contains only the MRAM
content.

### Verifying binary B at run time

`ospi_image_verify()` in `src/ospi_verify.c`:

1. initialises PSA Crypto;
2. imports the public key as an ML-DSA-87 verify-only key (for ML-DSA, the PSA
   "key bits" value is the parameter set, `87`, not the key size);
3. calls `psa_verify_message()` over
   `[__extflash_text_reloc_start, +__extflash_text_reloc_size)` with the
   signature at `__extflash_sig_start`. The OSPI window is memory-mapped, so
   this is the exact byte stream the CPU would fetch.

`main()` calls the OSPI function only when this returns success.

ML-DSA-87 verification with mldsa-native's reduced-RAM mode needs about 41 KB of
stack (`MLD_TOTAL_ALLOC_87_VERIFY`), so the main stack is 64 KB
(`CONFIG_MAIN_STACK_SIZE=65536`), the same as MCUboot's ML-DSA build. A smaller
stack overflows silently unless `CONFIG_HW_STACK_PROTECTION` is enabled.

## Memory map

MRAM (CM85, `0x02000000`, 768 KB), using the partitions the board already
defines:

| Region | Address | Size | Content |
|---|---|---|---|
| `mcuboot` | `0x02000000` | 64 KB | MCUboot (about 50 KB used) |
| `image-0` (slot0) | `0x02010000` | 344 KB | binary A (about 75 KB used) |
| `image-1` (slot1) | `0x02066000` | 344 KB | update slot (swap) |
| `storage` | `0x020BC000` | 16 KB | storage |

OSPI0 flash (MX25LW51245G, 64 MB, mapped at `0x90000000`):

| Region | Address | Size | Content |
|---|---|---|---|
| `.extflash_text_reloc` | `0x90000000` | code size (56 bytes today) | code of `ext_code.c` |
| `.extflash_sig` | right after the code (`0x90000038`) | 4627 bytes | ML-DSA-87 signature |
| `auto-calib-pattern` | `0x93FFF000` | 4 KB | OSPI calibration pattern, written by the driver |

The `EXTFLASH` linker region excludes the last 4 KB so code can never overlap the
calibration pattern.

## What each verifier checks, and what it does not

What the two checks give you:

- MCUboot only boots a binary A signed with key A.
- Binary A only runs binary B if it is signed with key B.
- Each binary can be re-signed or replaced on its own, as long as the matching
  key and public key stay consistent. Replacing key B needs a new binary A,
  because the public key for B is part of A.

What they do not give you:

- **Check once, run later.** Binary B is verified at boot and then executed from
  external flash. Someone who can modify the OSPI contents or bus after the
  check is not detected. This protects against a tampered or wrong image at
  rest, not against a live physical attack.
- **Only the code section is signed.** The signature covers
  `.extflash_text_reloc`. The data of `ext_code.c` is copied to RAM from binary
  A, so MCUboot already covers it. Anything else you write to the OSPI flash is
  not checked.
- **No version or rollback protection for binary B.** Swap and rollback apply to
  binary A only. After updating A, update B to the matching build. If they do not
  match, verification fails and B is not run.
- **Debug keys are not secure.** MCUboot's default key is public, and the default
  key B is regenerated for each fresh build directory.
- **`disable_mpu_rasr_xn()` in `main.c`** clears the execute-never bit on the MPU
  regions so the SRAM demo can run. It comes from the upstream sample and
  should be removed in a real design.

## Configuration reference

| Setting | Where | Purpose |
|---|---|---|
| `SB_CONFIG_BOOTLOADER_MCUBOOT`, `SB_CONFIG_BOOT_SIGNATURE_TYPE_ML_DSA` | `sysbuild.conf` | build MCUboot with ML-DSA-87 |
| `SB_CONFIG_BOOT_SIGNATURE_KEY_FILE` | command line | key A (MCUboot and imgtool) |
| `OSPI_SIGN_KEY` | command line | key B (OSPI image) |
| `CONFIG_BOOTLOADER_MCUBOOT` | board `.conf` | link the app into slot0 behind the image header |
| `CONFIG_CODE_DATA_RELOCATION`, `CONFIG_HAVE_CUSTOM_LINKER_SCRIPT`, `CONFIG_CUSTOM_LINKER_SCRIPT` | `prj.conf` | relocation with `linker_mram_ospi.ld` |
| `CONFIG_CODE_DATA_RELOCATION_SRAM` | board `.conf` | required by the OSPI_B driver with relocation on MPU targets |
| `CONFIG_FLASH` | board `.conf`, `prj.conf` | OSPI_B flash driver |
| `CONFIG_PSA_CRYPTO`, `CONFIG_MBEDTLS_PSA_CRYPTO_C`, `CONFIG_PSA_WANT_ALG_SHAKE256` | board `.conf` | PSA Crypto |
| `CONFIG_TF_PSA_CRYPTO_PQCP_MLDSA_ENABLED`, `..._MLDSA_87_ENABLED`, `..._REDUCE_RAM` | board `.conf` | mldsa-native driver, ML-DSA-87, reduced RAM |
| `CONFIG_MAIN_STACK_SIZE=65536` | board `.conf` | stack for the ML-DSA verifier (needs about 41 KB) |

The devicetree overlay `boards/ek_ra8m2_r7ka8m2jflcac_cm85.overlay` enables
`ospi0` with the MX25LW51245G (OPI mode, 133 MHz, 8D dummy cycles 20, DDR sampling
extension 7), labels the flash node `ospi0_ext_flash`, and keeps the
`auto-calib-pattern` partition at `0x3fff000`.

## Files

| File | Purpose |
|---|---|
| `CMakeLists.txt` | code relocation, key handling, the `split_flash` target (sign, split, swap, bin and hex outputs) |
| `linker_mram_ospi.ld` | defines the `EXTFLASH` region at `0x90000000` |
| `ospi_sig.ld` | linker snippet for the `.extflash_sig` section |
| `scripts/ospi_sign.py` | `genkey`, `header` and `sign` subcommands (ML-DSA-87) |
| `src/main.c` | verifies binary B, then calls the OSPI and SRAM functions |
| `src/ospi_verify.c` | PSA ML-DSA-87 verification and the signature slot |
| `src/ext_code.c` | the code that runs from OSPI flash (binary B) |
| `src/sram_code.c` | the code that runs from the SRAM copy |
| `boards/ek_ra8m2_r7ka8m2jflcac_cm85.overlay` | enables OSPI0 and its flash |
| `boards/ek_ra8m2_r7ka8m2jflcac_cm85.conf` | crypto, relocation and MCUboot options |
| `sysbuild.conf` | MCUboot and signature type |
| `sample.yaml` | build-only test entry |

## Troubleshooting

- **`ModuleNotFoundError: No module named 'dilithium_py'`**: install it and set
  `PYTHONPATH` (see [Requirements](#requirements)). `imgtool` and `ospi_sign.py`
  both need it.
- **`imgtool keygen` rejects the type**: use `-t mldsa-87`, not `mldsa87`, and
  the `imgtool.py` from `bootloader/mcuboot/scripts`.
- **`Using a generated DEVELOPMENT ML-DSA-87 key` warning**: no `OSPI_SIGN_KEY`
  was given. The key lives in the build directory and the public key in binary A
  matches it, but a pristine rebuild generates a new key and an old binary B
  will no longer verify.
- **`Using default MCUboot signing key file` warning**: no
  `SB_CONFIG_BOOT_SIGNATURE_KEY_FILE` was given.
- **imgtool error that the image exceeds the slot**: the OSPI sections ended up in
  `zephyr.bin`. Check that the `--remove-section` flags in `CMakeLists.txt` are in
  effect.
- **`External flash image verification FAILED`**: the OSPI flash does not hold the
  binary B built together with this binary A (stale or wrong image, a different
  key B, wrong byte order or a bad write). Reflash `zephyr_ospi.signed.bin` from
  the same build and check the byte swap.
- **`key import failed (-134)`**: `PSA_ERROR_NOT_SUPPORTED`. The ML-DSA key
  attributes must use 87 as the key-bits value. The current source already does.
- **Crash right after `main()` starts, with zeroed registers**: look for a stack
  overflow in the ML-DSA verifier. The stack must be at least 41 KB; enable
  `CONFIG_HW_STACK_PROTECTION` to make an overflow visible.
- **MCUboot bus fault reading the end of a slot (for example offset `0x65ff0`)**:
  seen on a board whose MRAM had never been programmed there. The suspected cause
  is that code MRAM uses ECC and unprogrammed areas read back as an uncorrectable
  error. This is not confirmed. Check `MRCRAES` (`0x4013C018`) and `MRCRTEA`
  (`0x4013C01C`), and write `0xFF` over the slot and storage area before the first
  flash.
- **`imgtool verify` reports `Key type does not match TLV record`**: see
  [Step 4](#step-4-optional-check-the-signatures-on-the-host).
- **Build error about `CODE_DATA_RELOCATION_SRAM`**: the OSPI_B driver requires it
  whenever `CODE_DATA_RELOCATION` is on for an MPU target. It is set in the board
  `.conf`.
